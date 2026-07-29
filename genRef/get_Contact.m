function contact = get_Contact(p)
%% function to get contact sequence
% p: default parameters (contain gait, k, global_time)

contact = ones(4, p.k);

% mode 0: static
if p.gait == 0
    contact = ones(4, p.k);


% mode 1: trotting
elseif p.gait == 1
    swing_time = p.t_swing;
    stance_time = p.t_stance;

    period = swing_time + stance_time;

    t = round(p.global_time, 4);
    for i = 1:p.k
        phase = round(mod(t, period), 4);
        phase_inv = round(mod(t + 0.5*period, period), 4);

        if phase < round(stance_time, 4)
            contact(1, i) = 1;      % FL Stance
            contact(4, i) = 1;      % RR Stance
        else
            contact(1, i) = 0;      % FL Swing
            contact(4, i) = 0;      % RR Swing
        end

        if phase_inv < round(stance_time, 4)
            contact(2, i) = 1;      % FR Stance
            contact(3, i) = 1;      % RL Stance
        else
            contact(2, i) = 0;      % FR Swing
            contact(3, i) = 0;      % RL Swing
        end


        
        t = round(t + p.control_dt, 4);
    end

% mode 2: pronking/jumping
elseif p.gait == 2
    swing_time = p.t_swing;
    stance_time = p.t_stance;

    period = swing_time + stance_time;

    t = round(p.global_time, 4);
    for i = 1:p.k
        phase = round(mod(t, period), 4);
        % phase_inv = round(mod(t + 0.5*period, period), 4);

        if phase < round(stance_time, 4)
            contact(1, i) = 1;      % FL Stance
            contact(2, i) = 1;      % FR Stance
            contact(3, i) = 1;      % RL Stance
            contact(4, i) = 1;      % RR Stance
        else
            contact(1, i) = 0;      % FL Swing
            contact(2, i) = 0;      % FR Swing
            contact(3, i) = 0;      % RL Swing
            contact(4, i) = 0;      % RR Swing
        end

        t = round(t + p.control_dt, 4);
    end

% mode 3: bounding
elseif p.gait == 3
    swing_time = p.t_swing;
    stance_time = p.t_stance;

    period = swing_time + stance_time;

    t = round(p.global_time, 4);
    for i = 1:p.k
        phase = round(mod(t, period), 4);
        phase_inv = round(mod(t + 0.5*period, period), 4);

        if phase < round(stance_time, 4)
            contact(1, i) = 1;      % FL Stance
            contact(2, i) = 1;      % FR Stance
        else
            contact(1, i) = 0;      % FL Swing
            contact(2, i) = 0;      % FR Swing
        end

        if phase_inv < round(stance_time, 4)
            contact(3, i) = 1;      % RL Stance
            contact(4, i) = 1;      % RR Stance
        else
            contact(3, i) = 0;      % RL Swing
            contact(4, i) = 0;      % RR Swing
        end


        
        t = round(t + p.control_dt, 4);
    end

% mode 4: Galloping
elseif p.gait == 4
    swing_time = p.t_swing;
    stance_time = p.t_stance;
    delta_t = 0.5 * stance_time;
    period = swing_time + stance_time;
    t = round(p.global_time, 4);
    
    for i = 1:p.k
        phase_FL = round(mod(t, period), 4);
        phase_FR = round(mod(t + delta_t, period), 4);
        phase_RL = round(mod(t + 0.5*period, period), 4);
        phase_RR = round(mod(t + 0.5*period + delta_t, period), 4);

        contact(1, i) = (phase_FL < round(stance_time, 4));
        contact(2, i) = (phase_FR < round(stance_time, 4));
        contact(3, i) = (phase_RL < round(stance_time, 4));
        contact(4, i) = (phase_RR < round(stance_time, 4));
        
        t = round(t + p.control_dt, 4);
    end

% mode 5: Pacing
elseif p.gait == 5
    swing_time = p.t_swing;
    stance_time = p.t_stance;

    period = swing_time + stance_time;

    t = round(p.global_time, 4);
    for i = 1:p.k
        phase = round(mod(t, period), 4);
        phase_inv = round(mod(t + 0.5*period, period), 4);

        if phase < round(stance_time, 4)
            contact(1, i) = 1;      % FL Stance
            contact(3, i) = 1;      % RL Stance
        else
            contact(1, i) = 0;      % FL Swing
            contact(3, i) = 0;      % RL Swing
        end

        if phase_inv < round(stance_time, 4)
            contact(2, i) = 1;      % FR Stance
            contact(4, i) = 1;      % RR Stance
        else
            contact(2, i) = 0;      % FR Swing
            contact(4, i) = 0;      % RR Swing
        end


        
        t = round(t + p.control_dt, 4);
    end

else
    contact = ones(4, p.k);
end

end