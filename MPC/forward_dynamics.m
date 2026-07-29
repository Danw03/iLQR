function Xnext = forward_dynamics(Xc, u, r, contact, p)
%% Discrete-time dynamics over one MPC control step.
% Xc: current State (13 x 1)
% u: control input (12 x 1)
% r: foot placement (12 x 1)
% contact: contact state (4 x 1)
% p: parameters

n_substeps = p.solver.substeps;
h = p.control_dt / n_substeps;

Xnext = Xc;
for substep = 1:n_substeps
    Xdot = get_Jacobians(Xnext, u, r, contact, p);
    Xnext = Xnext + h * Xdot;
end

end
