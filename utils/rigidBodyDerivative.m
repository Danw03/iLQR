function Xdot = rigidBodyDerivative(Xc, u, r, p, F_ext, tau_ext)
%RIGIDBODYDERIVATIVE Centroidal rigid-body dynamics shared by MPC and sim.

if nargin < 5
    F_ext = zeros(3, 1);
end

if nargin < 6
    tau_ext = zeros(3, 1);
end

phi = Xc(1);
theta = Xc(2);
psi = Xc(3);

omega = Xc(7:9);
v = Xc(10:12);

R_world = Rz(psi) * Ry(theta) * Rx(phi);
I_world = R_world * p.I_body * R_world';

sum_f = zeros(3, 1);
sum_tau = zeros(3, 1);

for leg = 1:4
    idx = 3*(leg-1)+1 : 3*leg;
    f_i = u(idx);
    r_i = r(idx);

    sum_f = sum_f + f_i;
    sum_tau = sum_tau + skewSymmetric(r_i) * f_i;
end

theta_dot = eulerJacobian(theta, psi) \ omega;
pos_dot = v;

coriolis = skewSymmetric(omega) * (I_world * omega);
omega_dot = I_world \ (sum_tau + tau_ext - coriolis);
velocity_dot = (sum_f + F_ext) / p.m + [0; 0; p.g];

Xdot = [theta_dot; pos_dot; omega_dot; velocity_dot; 0];
end
