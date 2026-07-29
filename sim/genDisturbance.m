function [F_dist, tau_dist] = genDisturbance(p)
% function to generate disturbance(외란)

t_i = p.disturbance.time(1);
t_f = p.disturbance.time(2);

F_ext = p.disturbance.F;
r_offset = p.disturbance.offset;

F_dist = [0; 0; 0];
tau_dist = [0; 0; 0];

if p.global_time >= t_i && p.global_time < t_f
    F_dist = F_ext;
    tau_dist = cross(r_offset, F_dist);
end

end