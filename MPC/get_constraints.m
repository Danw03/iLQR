function [g, Gu] = get_constraints(u, contact, p)
% Stance-leg force constraints: g(u) = Gu*u - h <= 0

mu = p.mu;

Gfoot = [ 1,  0, -mu;
         -1,  0, -mu;
          0,  1, -mu;
          0, -1, -mu;
          0,  0,  1;
          0,  0, -1];

hfoot = [0;
         0;
         0;
         0;
         p.f_Max;
       

stance_legs = find(contact == 1);
num_stance = numel(stance_legs);

g  = zeros(6*num_stance, 1);
Gu = zeros(6*num_stance, 12);

for j = 1:num_stance
    leg = stance_legs(j);

    force_idx = 3*(leg-1) + (1:3);
    row_idx = 6*(j-1) + (1:6);

    Gu(row_idx, force_idx) = Gfoot;
    g(row_idx) = Gfoot*u(force_idx) - hfoot;
end

end