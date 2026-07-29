function Xdot = simDynamics(Xc, u, r, p)
% Continuous-time dynamics for simulation, including external disturbance.
[F_dist, tau_dist] = genDisturbance(p);
Xdot = rigidBodyDerivative(Xc, u, r, p, F_dist, tau_dist);
end
