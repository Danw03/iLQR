function [Xdot, Ac, Bc] = get_Jacobians(Xc, u, r, contact, p)
%% get dynamics and analytic Jacobians.

u_active = zeros(size(u));
for leg = 1:4
    if contact(leg) == 1
        idx = 3*(leg-1) + (1:3);
        u_active(idx) = u(idx);
    end
end

Xdot = rigidBodyDerivative(Xc, u_active, r, p);

% for fuction "forward_dynamics.m", only need Xdot.
if nargout < 2
    return
end

nx = numel(Xc);
nu = numel(u);
Ac = zeros(nx, nx);
Bc = zeros(nx, nu);

phi = Xc(1);
theta = Xc(2);
psi = Xc(3);
omega = Xc(7:9);
omega_dot = Xdot(7:9);

sin_phi = sin(phi);
cos_phi = cos(phi);
sin_theta = sin(theta);
cos_theta = cos(theta);
sin_psi = sin(psi);
cos_psi = cos(psi);

Rx_k = Rx(phi);
Ry_k = Ry(theta);
Rz_k = Rz(psi);

dRx_dphi = [0,        0,         0;
             0, -sin_phi, -cos_phi;
             0,  cos_phi, -sin_phi];

dRy_dtheta = [-sin_theta, 0,  cos_theta;
                        0, 0,          0;
               -cos_theta, 0, -sin_theta];

dRz_dpsi = [-sin_psi, -cos_psi, 0;
             cos_psi, -sin_psi, 0;
                   0,        0, 0];

R_world = Rz_k * Ry_k * Rx_k;
dR = {Rz_k * Ry_k * dRx_dphi, ...
      Rz_k * dRy_dtheta * Rx_k, ...
      dRz_dpsi * Ry_k * Rx_k};

I_world = R_world * p.I_body * R_world';

% Euler-angle rates for a world-frame angular velocity.
sec_theta = 1 / cos_theta;
tan_theta = tan(theta);

T_euler = [ cos_psi*sec_theta, sin_psi*sec_theta, 0;
                    -sin_psi,             cos_psi, 0;
             cos_psi*tan_theta,  sin_psi*tan_theta, 1];

dT_dtheta = [cos_psi*sec_theta*tan_theta, ...
             sin_psi*sec_theta*tan_theta, 0;
                                           0, 0, 0;
             cos_psi*sec_theta^2, ...
             sin_psi*sec_theta^2, 0];

dT_dpsi = [-sin_psi*sec_theta,  cos_psi*sec_theta, 0;
                    -cos_psi,           -sin_psi, 0;
             -sin_psi*tan_theta, cos_psi*tan_theta, 0];

Ac(1:3, 2) = dT_dtheta * omega;
Ac(1:3, 3) = dT_dpsi * omega;
Ac(1:3, 7:9) = T_euler;
Ac(4:6, 10:12) = eye(3);

for angle_idx = 1:3
    dI_world = dR{angle_idx} * p.I_body * R_world' + R_world * p.I_body * dR{angle_idx}';

    Ac(7:9, angle_idx) = I_world \ (-dI_world * omega_dot -skewSymmetric(omega) * (dI_world * omega));
end

angular_momentum = I_world * omega;
Ac(7:9, 7:9) = I_world \ (skewSymmetric(angular_momentum) -skewSymmetric(omega) * I_world);

for leg = 1:4
    if contact(leg) == 0
        continue
    end

    idx = 3*(leg-1) + (1:3);
    r_i = r(idx);

    Bc(7:9, idx) = I_world \ skewSymmetric(r_i);
    Bc(10:12, idx) = (1 / p.m) * eye(3);
end

end
