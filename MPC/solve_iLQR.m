function [Uopt, J, iter] = solve_iLQR(Xc, Xref, R, Upre, contact, p)
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

Q_weight = p.Q_weight;
R_weight = p.R_weight;
N = p.k;

Xref = reshape(Xref, 13, N);    % 13*N X 1 --> 13 X N
R = reshape(R, 12, N);          % 12*N X 1 --> 12 X N

Uopt = warm_start(Upre, contact, p);
X = roll_out(Xc, Uopt, R, contact, p);

J = calc_cost(X, Xref, Uopt, p);
delta_J = inf;

%% Solver Loop
while iter < opts.max_iter && delta_J > opts.tolerance
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
        u_ff(:, i) = - Quu \ Qu;
        K(:, :, i) = - Quu \ Qux;

        % Update Vx, Vxx
        Vx = Qx - K(:, :, i)' * Quu * u_ff(:, i);
        Vxx = Qxx - K(:, :, i)' * Quu * K(:, :, i);
    end

    %% Line Search
    alpha = 1;
    lineSearchIter = 0;
    J_new = inf;
    
    X_new = zeros(13, N);
    U_new = zeros(12, N-1);
    accepted = false;
    
    while lineSearchIter < opts.max_line_search_iter
        dJ = 0;
        X_new(:, 1) = X(:, 1);
        
        %% Forward Rollout
        for i = 1:N-1
            % un' = un + alpha * uff + K(Xn' - Xn)
            U_new(:, i) = Uopt(:, i) + alpha * u_ff(:, i) + K(:, :, i) * (X_new(:, i) - X(:, i));

            % Xn+1 = f(Xn, Un)
            X_new(:, i+1) = forward_dynamics(X_new(:, i), U_new(:, i), R(:, i), contact(:, i), p);

            dJ = dJ + alpha * (Qu_history(:, i)' * u_ff(:, i));
        end
        
        J_new = calc_cost(X_new, Xref, U_new, p);
        
        % Armijo Termination Condition
        if isfinite(J_new) && J_new < J + opts.armijo * dJ
            accepted = true;
            break;
        end

        alpha = opts.alpha_decay * alpha;
        lineSearchIter = lineSearchIter + 1;
    end
    
    if ~accepted
        error('solve_iLQR:LineSearchFailed', ...
            ['Line search failed at solver iteration %d. ' ...
             'No acceptable trajectory was found after %d attempts.'], ...
            iter, lineSearchIter);
    end

    % Update trajectory
    X = X_new;
    Uopt = U_new;

    delta_J = (J - J_new) / max(1, J);

    J = J_new;

    iter = iter + 1;

end

end
