function [Xseries, R, p] = genSim(Xc, Uopt, R, contact, p)
% function to generate Sim.

% Xc: current state (13 x 1)
% Uopt: Solution from QP solver (12*k x 1)
% R: foot placement sequence over a horizon (12*k x 1)
% contact: contact sequence in a horizon
% p: parameter structure

r = R(1:12, 1);
c = contact(1:4, 1);
f_opt = Uopt(1:12, 1);

contact_mask = repelem(c ~= 0, 3);
f_opt(~contact_mask) = 0;

steps = round(p.control_dt / p.sim_dt);
Xseries = zeros(13, steps);

for i = 1:steps
    Xdot = simDynamics(Xc, f_opt, r, p);
    Xc = Xc + Xdot * p.sim_dt;
    p.global_time = p.global_time + p.sim_dt;

    omega = Xc(7:9, 1);  
    v = Xc(10:12, 1);
    for j = 1:4
        if c(j) == 1
            idx = 3*(j-1)+1 : 3*j;
            r(idx, 1) = r(idx, 1) - (v + cross(omega, r(idx, 1))) * p.sim_dt;
        end
    end

    Xseries(:, i) = Xc;
end

for j = 1:4
    if c(j) == 1
        idx = 3*(j-1)+1 : 3*j;
        R(idx, 1) = r(idx, 1);
    end
end
end