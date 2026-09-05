function [j, K, active_rows, lambda_active] = solve_constrained_policy(Quu, Qu, Qux, g, Gu, active_tol, lambda_tol)
% Algorithm 1, lines 8-17: identify active constraints, then solve the KKT
% system for j and K. Force constraints have gx=0, so D=0 in equation (13).
% Backward constraints are tangent constraints C*delta_u=0 (not -g).

nu = size(Quu, 1);
nx = size(Qux, 2);
candidate_rows = find(g >= -active_tol);
active_rows = zeros(0, 1);
lambda_active = zeros(0, 1);

if ~isempty(candidate_rows)
    C = Gu(candidate_rows, :);
    na = size(C, 1);
    KKT = [Quu, C'; C, zeros(na)];
    candidate_solution = KKT \ [-Qu; zeros(na, 1)];
    lambda_candidate = candidate_solution(nu+1:end);

    % Algorithm 1, line 13: retain constraints with positive multipliers.
    active_rows = candidate_rows(lambda_candidate > lambda_tol);
end

if isempty(active_rows)
    j = -(Quu \ Qu);
    K = -(Quu \ Qux);
    return
end

C = Gu(active_rows, :);
na = size(C, 1);
KKT = [Quu, C'; C, zeros(na)];
rhs = [-Qu, -Qux; zeros(na, 1 + nx)];
solution = KKT \ rhs;
j = solution(1:nu, 1);
K = solution(1:nu, 2:end);
lambda_active = solution(nu+1:end, 1);

end
