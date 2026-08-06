function [Uopt, J, iter, solver_log] = solve_iLQR(Xc, Xref, R, Upre, contact, p)
%% Trajectory optimizer based on iLQR
% Xc: current State (13 x 1)
% Xref: reference over a horizon (13*k x 1)
% R: foot placement (12*k x 1)
% Upre: previous control input 
% contact: contact sequence (4 x k)
% p: parameters (Q_weight, R_weight)

opts = p.solver;

% Initialize
iter = 0;

mu1 = opts.regulization1;
mu2 = opts.regulization2;

Alpha = opts.alpha;
Beta = opts.beta;

Q_weight = p.Q_weight;
R_weight = p.R_weight;
N = p.k;

solver_log.active_count = [];
solver_log.active_added = [];
solver_log.active_removed = [];

solver_log.J_before = [];
solver_log.J_candidate = [];
solver_log.reduction = [];
solver_log.accepted = [];

previous_active_history = cell(1, N-1);
has_previous_active = false;

Xref = reshape(Xref, 13, N);    % 13*N X 1 --> 13 X N
R = reshape(R, 12, N);          % 12*N X 1 --> 12 X N

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

previous_active_history = cell(1, N-1);
has_previous_active = false;

Uopt = warm_start(Upre, contact, p);
X = roll_out(Xc, Uopt, R, contact, p);

J = calc_cost(X, Xref, Uopt, p);
delta_J = inf;

%% Solver Loop
while iter < opts.max_iter && delta_J > opts.tolerance
    active_history = cell(1, N-1);

    u_ff = zeros(12, N-1);
    K = zeros(12, 13, N-1);
    Qu_history = zeros(12, N-1);

    %% Backward Pass
    Vx = Q_weight * (X(:, N) - Xref(:, N));
    Vxx = Q_weight;

    for i = N-1:-1:1
        [A, B] = get_AB(X(:, i), Uopt(:, i), R(:, i), contact(:, i), p);

        Qx = Q_weight * (X(:, i) - Xref(:, i)) + A'*Vx;
        Qu = R_weight * Uopt(:, i) + B'*Vx;
        Qu_history(:, i) = Qu;

        Qxx = Q_weight + A'*(Vxx + mu1 * eye(13))*A;
        Quu = R_weight + B'*(Vxx + mu1 * eye(13))*B + mu2 * eye(12);
        Qux = B'*(Vxx + mu1 * eye(13))*A;

        % Update policy
        Quu = 0.5 * (Quu + Quu');

        [g, Gu] = get_constraints(Uopt(:, i), contact(:, i), p);
        
        [j, K_i, active_rows, lambda_active] = solve_constrained_policy(Quu, Qu, Qux, g, Gu, opts.active_tolerance, opts.lambda_tolerance);
        
        u_ff(:, i) = j;
        K(:, :, i) = K_i;

        active_history{i} = active_rows(:);

        % Update Vx, Vxx
        Vx = Qx - K(:, :, i)' * Quu * u_ff(:, i);
        Vxx = Qxx - K(:, :, i)' * Quu * K(:, :, i);
    end

    %% Active-set tracking
    active_count = sum(cellfun(@numel, active_history));
    
    active_added = 0;
    active_removed = 0;
    
    if has_previous_active
        for i = 1:N-1
            active_added = active_added + numel(setdiff(active_history{i}, previous_active_history{i}));
            active_removed = active_removed + numel(setdiff( previous_active_history{i}, active_history{i}));
        end
    end

    %% Forward Pass
    J_init = J;

    X_new = zeros(13, N);
    U_new = zeros(12, N-1);

    X_new(:, 1) = X(:, 1);

    for i = 1:N-1
        delta_X = X_new(:, i) - X(:, i);

        u_raw = Uopt(:, i) + u_ff(:, i) + K(:, :, i) * delta_X;
        U_new(:, i) = clamp_force(u_raw, contact(:, i), p);

        X_new(:, i+1) = forward_dynamics(X_new(:, i), U_new(:, i), R(:, i), contact(:, i), p);
    end

    % Acceptance check
    J_new = calc_cost(X_new, Xref, U_new, p);
    actual_reduction = J_init - J_new;

    accepted = isfinite(J_new) && J_new < J_init;

    if accepted
        % Accept
        X = X_new;
        Uopt = U_new;
        J = J_new;
    
        delta_J = actual_reduction / max(1, abs(J_init));
    
        mu1 = Beta*mu1;
        mu2 = Beta*mu2;
    
    else
        % Reject
        delta_J = inf;
    
        mu1 = Alpha*mu1;
        mu2 = Alpha*mu2;
    end

    %% Save tracking log
    log_idx = iter + 1;
    solver_log.active_count(log_idx) = active_count;
    solver_log.active_added(log_idx) = active_added;
    solver_log.active_removed(log_idx) = active_removed;
    solver_log.J_before(log_idx) = J_init;
    solver_log.J_candidate(log_idx) = J_new;
    solver_log.reduction(log_idx) = actual_reduction;
    solver_log.accepted(log_idx) = accepted;
    previous_active_history = active_history;
    has_previous_active = true;
    
    iter = iter + 1;
    
end

end
