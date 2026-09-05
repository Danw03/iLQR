function [Uopt, J, iter, solver_log] = solve_iLQR(Xc, Xref, R, Upre, contact, p)
% Constrained iLQR using stage-wise subQPs.
% Xie, Liu, Hauser (2017), "Differential Dynamic Programming with
% Nonlinear Constraints", Algorithms 1-2. Dynamics use iLQR Jacobians;
% the second derivatives of dynamics in full DDP are omitted.
% Xc: 13 x 1, Xref: 13*N x 1, R: 12*N x 1, contact: 4 x N.
% Outputs: Uopt is 12 x (N-1); solver_log columns follow solver iterations.

opts = p.solver;
N = p.k;
Q_weight = p.Q_weight;
R_weight = p.R_weight;
mu1 = opts.regulization1;
mu2 = opts.regulization2;

qp_options = optimoptions('quadprog', ...
    'Algorithm', 'active-set', ...
    'Display', 'off', ...
    'MaxIterations', opts.qp_max_iter, ...
    'ConstraintTolerance', opts.feasibility_tolerance);

Xref = reshape(Xref, 13, N);
R = reshape(R, 12, N);
Uopt = warm_start(Upre, contact, p);
X = roll_out(Xc, Uopt, R, contact, p);
J = calc_cost(X, Xref, Uopt, p);
iter = 0;
delta_J = inf;

solver_log.active_count = [];
solver_log.active_added = [];
solver_log.active_removed = [];
solver_log.J_before = [];
solver_log.J_candidate = [];
solver_log.reduction = [];
solver_log.accepted = [];
solver_log.mu1_before = [];
solver_log.mu2_before = [];
solver_log.mu1_after = [];
solver_log.mu2_after = [];
solver_log.backward_ok = [];
solver_log.qp_exitflag = zeros(N-1, 0); % 0: no QP solved (e.g. flight).
solver_log.failed_stage = [];         % 0: no failed stage.
solver_log.trust_region = [];
solver_log.trust_restarts = [];
previous_active_history = cell(1, N-1);

while iter < opts.max_iter && delta_J > opts.tolerance
    J_init = J;
    mu1_before = mu1;
    mu2_before = mu2;
    active_history = cell(1, N-1);
    u_ff = zeros(12, N-1);
    K = zeros(12, 13, N-1);
    Qu_history = zeros(12, N-1);
    Quu_history = zeros(12, 12, N-1);
    Qux_history = zeros(12, 13, N-1);
    backward_ok = true;
    failed_stage = 0;

    %% Backward pass: Algorithm 1
    Vx = Q_weight * (X(:, N) - Xref(:, N));
    Vxx = Q_weight;

    for i = N-1:-1:1
        [A, B] = get_AB(X(:, i), Uopt(:, i), R(:, i), contact(:, i), p);
        Vxx_reg = Vxx + mu1 * eye(13);
        Qx = Q_weight * (X(:, i) - Xref(:, i)) + A' * Vx;
        Qu = R_weight * Uopt(:, i) + B' * Vx;
        Qxx = Q_weight + A' * Vxx_reg * A;
        Quu = R_weight + B' * Vxx_reg * B + mu2 * eye(12);
        Qux = B' * Vxx_reg * A;
        Quu = 0.5 * (Quu + Quu');

        % Swing forces are fixed at zero, so optimize only stance forces.
        control_idx = find(repelem(contact(:, i) == 1, 3));
        j = zeros(12, 1);
        K_i = zeros(12, 13);
        if ~isempty(control_idx)
            [~, chol_flag] = chol(Quu(control_idx, control_idx));
            if chol_flag ~= 0
                backward_ok = false;
                failed_stage = i;
                break
            end

            [g, Gu] = get_constraints(Uopt(:, i), contact(:, i), p);
            [j_active, K_active, active_rows, lambda_active] = ...
                solve_constrained_policy(Quu(control_idx, control_idx), ...
                Qu(control_idx), Qux(control_idx, :), g, Gu(:, control_idx), ...
                opts.active_tolerance, opts.lambda_tolerance); %#ok<ASGLU>
            j(control_idx) = j_active;
            K_i(control_idx, :) = K_active;
            active_history{i} = active_rows;
        end

        u_ff(:, i) = j;
        K(:, :, i) = K_i;
        Qu_history(:, i) = Qu;
        Quu_history(:, :, i) = Quu;
        Qux_history(:, :, i) = Qux;

        % Equation (8): substitute the constrained policy into Q.
        Vx = Qx + K_i' * Quu * j + Qux' * j + K_i' * Qu;
        Vxx = Qxx + K_i' * Quu * K_i + Qux' * K_i + K_i' * Qux;
        Vxx = 0.5 * (Vxx + Vxx');
    end

    %% Forward pass: Algorithm 2, equation (19)
    forward_ok = false;
    e = opts.trust_region; % Reset the trust bound for each forward pass.
    trust_restarts = 0;
    qp_exitflag = zeros(N-1, 1);
    J_new = inf;

    if backward_ok
        for trust_iter = 0:opts.max_trust_region_restarts
            X_new = zeros(13, N);
            U_new = zeros(12, N-1);
            X_new(:, 1) = Xc;
            qp_exitflag(:) = 0;
            failed_stage = 0;
            shrink_trust_region = false;
            trust_restarts = trust_iter;

            for i = 1:N-1
                delta_x = X_new(:, i) - X(:, i);
                control_idx = find(repelem(contact(:, i) == 1, 3));
                nu = numel(control_idx);

                if nu > 0
                    H = Quu_history(control_idx, control_idx, i);
                    f = Qu_history(control_idx, i) ...
                        + Qux_history(control_idx, :, i) * delta_x;

                    % g has no state dependence in this force model (gx=0).
                    % g(u_nominal) + Gu*delta_u <= 0 is exact here.
                    [g, Gu] = get_constraints(Uopt(:, i), contact(:, i), p);
                    Aineq = Gu(:, control_idx);
                    bineq = -g;
                    lb = -e * ones(nu, 1);
                    ub =  e * ones(nu, 1);
                    delta_u0 = zeros(nu, 1); % Feasible nominal trajectory.

                    [delta_u, ~, exitflag] = quadprog(H, f, Aineq, bineq, ...
                        [], [], lb, ub, delta_u0, qp_options);
                    qp_exitflag(i) = exitflag;
                    if exitflag <= 0
                        failed_stage = i;
                        shrink_trust_region = (exitflag == -2);
                        break
                    end

                    U_new(control_idx, i) = Uopt(control_idx, i) + delta_u;
                    g_new = get_constraints(U_new(:, i), contact(:, i), p);
                    if any(~isfinite(U_new(:, i))) ...
                            || any(g_new > opts.feasibility_tolerance)
                        failed_stage = i;
                        shrink_trust_region = true;
                        break
                    end
                end

                X_new(:, i+1) = forward_dynamics(X_new(:, i), ...
                    U_new(:, i), R(:, i), contact(:, i), p);
            end

            if failed_stage == 0
                forward_ok = true;
                J_new = calc_cost(X_new, Xref, U_new, p);
                break
            end
            if ~shrink_trust_region ...
                    || trust_iter == opts.max_trust_region_restarts ...
                    || e * opts.trust_region_decay < opts.min_trust_region
                break
            end
            e = opts.trust_region_decay * e;
        end
    end

    %% Accept the complete trajectory and update regularization
    actual_reduction = J_init - J_new;
    accepted = forward_ok && isfinite(J_new) && J_new < J_init;
    if accepted
        X = X_new;
        Uopt = U_new;
        J = J_new;
        delta_J = actual_reduction / max(1, abs(J_init));
        mu1 = opts.beta * mu1;
        mu2 = opts.beta * mu2;
    else
        delta_J = inf;
        mu1 = opts.alpha * mu1;
        mu2 = opts.alpha * mu2;
    end

    %% Debug log (one column per solver iteration)
    active_count = sum(cellfun(@numel, active_history));
    active_added = 0;
    active_removed = 0;
    if iter > 0
        for i = 1:N-1
            active_added = active_added + ...
                numel(setdiff(active_history{i}, previous_active_history{i}));
            active_removed = active_removed + ...
                numel(setdiff(previous_active_history{i}, active_history{i}));
        end
    end

    log_idx = iter + 1;
    solver_log.active_count(log_idx) = active_count;
    solver_log.active_added(log_idx) = active_added;
    solver_log.active_removed(log_idx) = active_removed;
    solver_log.J_before(log_idx) = J_init;
    solver_log.J_candidate(log_idx) = J_new;
    solver_log.reduction(log_idx) = actual_reduction;
    solver_log.accepted(log_idx) = accepted;
    solver_log.mu1_before(log_idx) = mu1_before;
    solver_log.mu2_before(log_idx) = mu2_before;
    solver_log.mu1_after(log_idx) = mu1;
    solver_log.mu2_after(log_idx) = mu2;
    solver_log.backward_ok(log_idx) = backward_ok;
    solver_log.qp_exitflag(:, log_idx) = qp_exitflag;
    solver_log.failed_stage(log_idx) = failed_stage;
    solver_log.trust_region(log_idx) = e;
    solver_log.trust_restarts(log_idx) = trust_restarts;
    previous_active_history = active_history;
    iter = iter + 1;
end

end
