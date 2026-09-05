%% iLQR based trajectory optimization for MIT Cheetah 3

clc;
clear;
close all;

project_root = fileparts(mfilename('fullpath'));
addpath(fullfile(project_root, 'genRef'), ...
        fullfile(project_root, 'MPC'), ...
        fullfile(project_root, 'sim'), ...
        fullfile(project_root, 'utils'));

%% Default Parameters
params.m = 43;
params.g = -9.8;
params.I_body = diag([0.41, 2.1, 2.1]);
params.body_l = 0.73;
params.body_w = 0.24;
params.body_h = 0.24;
params.mu = 0.6;
params.f_min = 10;
params.f_Max = 666;
params.global_time = 0;
params.sim_dt = 0.001;

%% MPC Parameters
params.k = 20;      % horizon length: 0.5s (= 0.025 * 20)
params.control_dt = 0.025;


%% iLQR Parameters
params.solver.max_iter = 10;
params.solver.tolerance = 1e-2;
params.solver.active_tolerance = 1e-3;
params.solver.lambda_tolerance = 1e-9;
params.solver.regulization1 = 1e-5;
params.solver.regulization2 = 1e-5;
params.solver.alpha = 2.0;
params.solver.beta = 0.9;
% Algorithm 2: forward subQP and trust-region settings.
params.solver.trust_region = 1000;
params.solver.trust_region_decay = 0.5;
params.solver.min_trust_region = 1e-3;
params.solver.max_trust_region_restarts = 12;
params.solver.qp_max_iter = 100;
params.solver.feasibility_tolerance = 1e-7;
params.solver.substeps = 1;

%% Gait Parameters
% 0: Standing on all
% 1: Trotting
% 2: Pronking/Jumping
% 3: Bounding
% 4: Galloping
% 5: Pacing
params.gait = 1;
params.v_des = [5; 0; 0];  % m/s
params.a_des = 10;         % m/s^2
params.w_des = 0.0;        % rad/s
params.alpha_des = 100;    % rad/s^2

params.t_stance = 0.12;
params.t_swing = 0.18;

%% Disturbance Parameters
params.disturbance.time = [2; 2.2];
params.disturbance.F = [0; 0; 0];
params.disturbance.offset = [0; -0.12; 0.12]; % 0.365 0.12 0.12

%% Weight Parameters
params.Q_weight = diag([ 1  1  1,  ... % roll, pitch, yaw weight
                         0  0 50,  ... % x, y, z weight
                         0  0  1,  ... % wx, wy, wz weight
                         1  1  1,  ... % vx, vy, vz weight
                         0]);

params.R_weight = 1e-6 * eye(12);

%% Simulation Configuration
steps = 400;    % simulation time: 10s (0.025 * 400 = 10)
sim_steps = round(params.control_dt / params.sim_dt);


%% Initialization
Xc = zeros(13, 1);
Xc(1:13, 1) = [0; 0;   0;   % Euler's angles: roll, pitch, yaw
               0; 0; 0.6;   % Position: x, y, z
               0; 0;   0;   % Angular velocity
               0; 0;   0;   % Velocity
               params.g];   % Gravity acc. (cf. eq (16)-(17) from the main paper)

Uopt = zeros(12, params.k);

R_init = [ 0.365;  0.12; -0.6;
           0.365; -0.12; -0.6;
          -0.365;  0.12; -0.6;
          -0.365; -0.12; -0.6];

Xref = genRef(Xc, params);
contact = get_Contact(params);
R = get_R(Xc, Xref, contact, params, repmat(R_init, params.k, 1));


%% History Structure
history.X = zeros(13, sim_steps * steps);
history.F = zeros(12, steps);
history.R = zeros(12, steps);
history.Xref = zeros(13 * params.k, steps);
history.F_total = zeros(1, steps);
history.contact = zeros(1, steps);
history.solver_time = zeros(1, steps);
history.solver_cost = zeros(1, steps);
history.solver_iter = zeros(1, steps);


%% Main Loop
for step = 1:steps
    Xref = genRef(Xc, params);
    contact = get_Contact(params);
    R = get_R(Xc, Xref, contact, params, R);

    tic;
    [Uopt, final_cost, iter, solver_log] = solve_iLQR(Xc, Xref, R, Uopt, contact, params);
    history.solver_time(step) = toc;
    history.solver_cost(step) = final_cost;
    history.solver_iter(step) = iter;
    
    [Xseries, R, params] = genSim(Xc, Uopt(:, 1), R, contact, params);

    history = genHistory(history, step, sim_steps, Xseries, R, Uopt(:, 1), Xref, contact(:, 1));

    Xc = Xseries(:, end);

    print_solver_summary(step, steps, final_cost, iter, solver_log);
end

finalReport(history);
visualize(history, params);
