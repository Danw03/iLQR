function [A, B] = get_AB(Xc, u, r, contact, p)
%% Analytic Jacobian of the subdivided Forward-Euler dynamics.
% Xc: current state (13 X 1)
% u: control input (12 X 1)
% r: foot placement (12 X 1)
% contact: contact state (4 X 1)
% p: parameters

n_substeps = p.solver.substeps;
h = p.control_dt / n_substeps;

nx = numel(Xc);
nu = numel(u);
A = eye(nx);
B = zeros(nx, nu);
Xsub = Xc;

for substep = 1:n_substeps
    [Xdot, Ac, Bc] = get_Jacobians(Xsub, u, r, contact, p);

    As = eye(nx) + h * Ac;
    Bs = h * Bc;

    B = As * B + Bs;
    A = As * A;

    Xsub = Xsub + h * Xdot;
end

end
