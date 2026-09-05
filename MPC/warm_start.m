function U = warm_start(Upre, contact, p)
%% generate initial guess
% contact: contact sequence (4 x k)
% p: parameters (Q_weight, R_weight)

N = p.k;
n_controls = N - 1;
nu = 12;
support_force = -p.m * p.g;
U = zeros(nu, n_controls);

has_previous_solution = size(Upre, 1) == nu ...
    && size(Upre, 2) == n_controls ...
    && all(isfinite(Upre(:))) ...
    && any(abs(Upre(:)) > 1e-12);

if has_previous_solution
    U(:, 1:n_controls-1) = Upre(:, 2:n_controls);
    columns_to_initialize = n_controls;
else
    % cold start
    columns_to_initialize = 1:n_controls;
end

for k = columns_to_initialize
    stance_legs = find(contact(:, k) == 1);
    if isempty(stance_legs)
        continue
    end

    fz_per_leg = support_force / numel(stance_legs);
    for leg = stance_legs(:)'
        U(3*leg, k) = fz_per_leg;
    end
end

for k = 1:n_controls
    % CDDP starts from a feasible trajectory, including new stance contacts.
    % Projection is only for initialization; forward updates use subQPs.
    U(:, k) = clamp_force(U(:, k), contact(:, k), p);
end

end
