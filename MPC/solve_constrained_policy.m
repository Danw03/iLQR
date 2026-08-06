function [j, K, active_rows, lambda_active] = solve_constrained_policy(Quu, Qu, Qux, g, Gu, active_tol, lambda_tol)

nu = size(Quu, 1);
nx = size(Qux, 2);

active_rows = find(g >= -active_tol);

while true
    if isempty(active_rows)
        j = -Quu \ Qu;
        K = -Quu \ Qux;
        lambda_active = zeros(0, 1);
        return
    end

    C = Gu(active_rows, :);
    na = size(C, 1);

    KKT = [Quu, C';
           C,    zeros(na)];

    rhs = [-Qu, -Qux;
           zeros(na, 1 + nx)];

    solution = KKT \ rhs;

    j_candidate = solution(1:nu, 1);
    K_candidate = solution(1:nu, 2:end);
    lambda_candidate = solution(nu+1:end, 1);

    negative_dual = lambda_candidate < -lambda_tol;

    if ~any(negative_dual)
        j = j_candidate;
        K = K_candidate;
        lambda_active = lambda_candidate;
        return
    end

    active_rows(negative_dual) = [];
end

end