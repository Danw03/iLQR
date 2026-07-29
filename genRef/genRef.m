function Xref = genRef(Xc, p)
%% Generate a state reference aligned with the MPC state nodes.
% Xc: current state [13 x 1]
% Xref: stacked reference [13*p.k x 1]

nx = 13;
N = p.k;
dt = p.control_dt;

v_target_body = p.v_des(1:2);
a_limit = abs(p.a_des);
w_target = (pi / 180) * p.w_des;
alpha_limit = abs((pi / 180) * p.alpha_des);

Xref_mat = zeros(nx, N);

% Node 1 corresponds to the current rollout state X(:,1).
Xref_mat(3, 1) = Xc(3);
Xref_mat(4:5, 1) = Xc(4:5);
Xref_mat(6, 1) = 0.6;
Xref_mat(9, 1) = Xc(9);
Xref_mat(10:11, 1) = Xc(10:11);
Xref_mat(13, 1) = Xc(13);

for k = 1:N-1
    yaw = Xref_mat(3, k);
    position = Xref_mat(4:5, k);
    yaw_rate = Xref_mat(9, k);
    velocity = Xref_mat(10:11, k);

    % Body-frame velocity command expressed in the current heading frame.
    v_target_world = Rz(yaw) * [v_target_body; 0];
    velocity_error = v_target_world(1:2) - velocity;
    acceleration = velocity_error / dt;

    acceleration_norm = norm(acceleration);
    if acceleration_norm > a_limit && acceleration_norm > 0
        acceleration = (a_limit / acceleration_norm) * acceleration;
    end

    % Acceleration-limited yaw-rate command.
    angular_acceleration = (w_target - yaw_rate) / dt;
    angular_acceleration = min(max( ...
        angular_acceleration, -alpha_limit), alpha_limit);

    yaw_next = yaw + yaw_rate*dt ...
             + 0.5*angular_acceleration*dt^2;
    yaw_rate_next = yaw_rate + angular_acceleration*dt;

    position_next = position + velocity*dt ...
                  + 0.5*acceleration*dt^2;
    velocity_next = velocity + acceleration*dt;

    Xref_mat(:, k+1) = Xref_mat(:, k);
    Xref_mat(1:2, k+1) = 0;
    Xref_mat(3, k+1) = yaw_next;
    Xref_mat(4:5, k+1) = position_next;
    Xref_mat(6, k+1) = 0.6;
    Xref_mat(7:8, k+1) = 0;
    Xref_mat(9, k+1) = yaw_rate_next;
    Xref_mat(10:11, k+1) = velocity_next;
    Xref_mat(12, k+1) = 0;
    Xref_mat(13, k+1) = Xc(13);
end

Xref = Xref_mat(:);

end
