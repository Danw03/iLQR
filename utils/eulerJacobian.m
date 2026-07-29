function J = eulerJacobian(theta, psi)
% function to get Jacobian marix for simulation (non-linear)
% eq (9) from the main paper
J = [cos(theta)*cos(psi) -sin(psi) 0;
     cos(theta)*sin(psi)  cos(psi) 0;
             -sin(theta)         0 1];
end