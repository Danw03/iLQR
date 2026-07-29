function Xseries = roll_out(Xc, Uopt, R, contact, p)
%% forward roll out (execute once for initialzation)
% Xc: current State (13 x 1)
% Uopt: control input (12 X N-1)
% R: foot placement (12 x N) 
% p: parameters (Q_weight, R_weight, sim_dt, control_dt, k)

N = p.k;

Xseries = zeros(13, N);
Xseries(:, 1) = Xc;

for i = 1 : N-1
    r = R(:, i);
    u = Uopt(:, i);
    c = contact(:, i);

    Xc = forward_dynamics(Xc, u, r, c, p);

    Xseries(:, i+1) = Xc;
end

end
