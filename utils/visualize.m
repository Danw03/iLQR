function visualize(history, params)
    L = params.body_l / 2;
    W = params.body_w / 2;
    H = params.body_h / 2;

    X_history = history.X;
    f_history = history.F;
    R_history = history.R;
    
    % =========================================================================
    % --- 현재 날짜/시간 기반 폴더 생성 (datetime 사용) ---
    % =========================================================================
    % 'now' 대신 datetime('now')를 사용하고 Format을 지정합니다.
    t = datetime('now', 'Format', 'yyyyMMdd_HHmmss');
    timestamp = char(t); % fullfile과의 호환성을 위해 문자열로 변환
    
    save_dir = fullfile('Fig', timestamp);
    
    if ~exist(save_dir, 'dir')
        mkdir(save_dir);
    end

    % 애니메이션 창 설정
    fig = figure('Name', 'MIT Cheetah 3 - 3D Animation', 'Color', 'w', 'Position', [100, 100, 800, 600]);
    
    gif_filename = fullfile(save_dir, 'cheetah_simulation.gif');
    
    mpc_steps = size(f_history, 2);
    steps_per_mpc = round(params.control_dt / params.sim_dt);
    
    % 외란 시각화용 전역 시간 설정 (초기화)
    if ~isfield(params, 'global_time')
        params.global_time = 0;
    end

    current_time = 0;
    
    % 다리 선 시각화를 위한 고관절(Hip) 오프셋
    hip_offset = [ 0.365,  0.12, 0;
                   0.365, -0.12, 0;
                  -0.365,  0.12, 0;
                  -0.365, -0.12, 0]';
    leg_names = {'FL', 'FR', 'RL', 'RR'};
              
    for k = 1:mpc_steps
        % 2. 현재 프레임의 데이터 파싱
        idx = (k-1) * steps_per_mpc + 1;
        if idx > size(X_history, 2), idx = size(X_history, 2); end 
        
        x_current = X_history(:, idx);
        R_feet = R_history(:, k); 
        f_opt  = f_history(:, k);
        
        roll  = x_current(1);
        pitch = x_current(2);
        yaw   = x_current(3);
        p_com = x_current(4:6); 
        
        % ZYX 오일러 각 회전 변환 행렬 (Body to World)
        R_world = Rz(yaw) * Ry(pitch) * Rx(roll);
        
        % 3. 몸통(Box) 꼭짓점 정의 및 회전 (Body -> World)
        box_corners = [ L,  W,  H;  L, -W,  H; -L, -W,  H; -L,  W,  H;
            L,  W, -H;  L, -W, -H; -L, -W, -H; -L,  W, -H ]';
        box_corners_world = R_world * box_corners + p_com;
        
        cla; % 프레임 지우기
        hold on; grid on; axis equal;
        
        % 시점 및 축 고정 (로봇을 따라다니는 카메라)
        view(3);
        axis([p_com(1)-1.0, p_com(1)+1.0, p_com(2)-1.0, p_com(2)+1.0, 0, 1.2]);
        xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
        
        % a) 로봇 몸통 (Wireframe)
        top_idx = [1 2 3 4 1];
        plot3(box_corners_world(1, top_idx), box_corners_world(2, top_idx), box_corners_world(3, top_idx), 'k-', 'LineWidth', 1);
        bot_idx = [5 6 7 8 5];
        plot3(box_corners_world(1, bot_idx), box_corners_world(2, bot_idx), box_corners_world(3, bot_idx), 'k-', 'LineWidth', 1);
        for p = 1:4
            plot3(box_corners_world(1, [p, p+4]), box_corners_world(2, [p, p+4]), box_corners_world(3, [p, p+4]), 'k-', 'LineWidth', 1);
        end
        plot3(p_com(1), p_com(2), p_com(3), 'ko', 'MarkerSize', 8, 'MarkerFaceColor', 'k');
        text(p_com(1), p_com(2), p_com(3) + 0.05, 'COM', 'FontSize', 10, 'FontWeight', 'bold', 'Color', 'k');
        
        % b) 다리 위치 및 힘(GRF) 벡터 시각화
        f_scale = 0.003;
        grf_color = 'r'; 
        
        for i = 1:4
            leg_idx = 3*(i-1) + 1 : 3*i;
            r_i = R_feet(leg_idx);
            p_foot = p_com + r_i;
                        
            % 발 라벨 추가
            text(p_foot(1), p_foot(2), p_foot(3) - 0.05, leg_names{i}, 'Color', 'k', 'FontSize', 8);
            
            % f 벡터 (지면 반력 화살표)
            f_i = f_opt(leg_idx);
            if norm(f_i) > 1
                quiver3(p_foot(1), p_foot(2), p_foot(3), ...
                    f_i(1)*f_scale, f_i(2)*f_scale, f_i(3)*f_scale, ...
                    0, 'Color', grf_color, 'LineWidth', 1.5, 'MaxHeadSize', 0.5);
            end
        end 

        % c) 외란(Disturbance Force) 시각화
        if isfield(params, 'disturbance')
            d_start = params.disturbance.time(1);
            d_end   = params.disturbance.time(2);

            f_dist_val = params.disturbance.F;
            f_dist_offset = params.disturbance.offset;
            
            % 현재 시간이 외란 발생 구간 내에 있을 때만 표시
            if current_time >= d_start && current_time <= d_end && norm(f_dist_val) > 0
                
                % 외란 작용점 계산 (Body -> World)
                % COM 위치에서 바디 회전을 고려한 offset을 더함
                p_dist_origin = p_com + R_world * f_dist_offset;
                
                % 외란 화살표 그리기 (파란색, 두껍게)
                d_scale = 0.005; % 외란 가시성을 위해 GRF보다 약간 크게 설정 가능
                quiver3(p_dist_origin(1), p_dist_origin(2), p_dist_origin(3), ...
                        f_dist_val(1)*d_scale, f_dist_val(2)*d_scale, f_dist_val(3)*d_scale, ...
                        0, 'Color', 'b', 'LineWidth', 1.5, 'MaxHeadSize', 0.5);
                
                % 외란 텍스트 표시
                text(p_dist_origin(1), p_dist_origin(2), p_dist_origin(3) + 0.1, ...
                    'Disturbance', 'Color', 'b', 'FontSize', 10, 'FontWeight', 'bold');
            end
        end
        
        % --- 현재 상태 정보 텍스트 오버레이 ---
        current_time = (k-1) * params.control_dt;
        v_com = x_current(10:12);
        info_str = sprintf('Time : %5.2f s\nPos  : [%5.2f, %5.2f, %5.2f] m\nVel  : [%5.2f, %5.2f, %5.2f] m/s', ...
                            current_time, p_com(1), p_com(2), p_com(3), v_com(1), v_com(2), v_com(3));
        text(p_com(1) - 0.8, p_com(2), p_com(3) + 0.8, info_str, ...
            'FontSize', 8, 'FontWeight', 'bold', 'FontName', 'Consolas', ...
            'BackgroundColor', [1 1 1 0.8], 'EdgeColor', 'k', ... 
            'Margin', 1, 'HorizontalAlignment', 'left');
        
        drawnow;
        
        % --- GIF 프레임 쓰기 ---
        frame = getframe(fig);
        im = frame2im(frame);
        [imind, cm] = rgb2ind(im, 256);
        if k == 1
            imwrite(imind, cm, gif_filename, 'gif', 'Loopcount', inf, 'DelayTime', params.control_dt);
        else
            imwrite(imind, cm, gif_filename, 'gif', 'WriteMode', 'append', 'DelayTime', params.control_dt);
        end

        current_time = current_time + params.control_dt;
    end

    % =========================================================================
    % --- 결과 그래프 시각화 및 자동 저장 파트 ---
    % =========================================================================
    
    t_state = (0:size(X_history, 2)-1) * params.sim_dt;
    t_force = (0:size(f_history, 2)-1) * params.control_dt;
    
    % 1) State 그래프
    fig_state = figure('Name', 'Robot States', 'Position', [150, 150, 1200, 800], 'Color', 'w');
    
    subplot(2, 2, 1);
    plot(t_state, X_history(4:6, :), 'LineWidth', 1.5);
    title('Position (X, Y, Z)'); xlabel('Time (s)'); ylabel('Position (m)');
    legend('X', 'Y', 'Z', 'Location', 'best'); grid on;
    
    subplot(2, 2, 2);
    plot(t_state, X_history(1:3, :), 'LineWidth', 1.5);
    title('Angle (Roll, Pitch, Yaw)'); xlabel('Time (s)'); ylabel('Angle (rad)');
    legend('Roll', 'Pitch', 'Yaw', 'Location', 'best'); grid on;
    
    subplot(2, 2, 3);
    plot(t_state, X_history(10:12, :), 'LineWidth', 1.5);
    title('Linear Velocity (v_x, v_y, v_z)'); xlabel('Time (s)'); ylabel('Velocity (m/s)');
    legend('v_x', 'v_y', 'v_z', 'Location', 'best'); grid on;
    
    subplot(2, 2, 4);
    plot(t_state, X_history(7:9, :), 'LineWidth', 1.5);
    title('Angular Velocity (\omega_x, \omega_y, \omega_z)'); xlabel('Time (s)'); ylabel('Angular Vel (rad/s)');
    legend('\omega_x', '\omega_y', '\omega_z', 'Location', 'best'); grid on;
    
    saveas(fig_state, fullfile(save_dir, 'state_graphs.png'));
    
    % 2) Force 그래프
    fig_force = figure('Name', 'Ground Reaction Forces', 'Position', [200, 200, 1200, 800], 'Color', 'w');
    leg_titles = {'FL (Front Left)', 'FR (Front Right)', 'RL (Rear Left)', 'RR (Rear Right)'};
    
    for i = 1:4
        subplot(2, 2, i);
        leg_idx = 3*(i-1) + 1 : 3*i;
        plot(t_force, f_history(leg_idx, :), 'LineWidth', 1.5);
        title(leg_titles{i}); xlabel('Time (s)'); ylabel('Force (N)');
        legend('F_x', 'F_y', 'F_z', 'Location', 'best'); grid on;
    end
    
    saveas(fig_force, fullfile(save_dir, 'force_graphs.png'));

    % 1) History 데이터 CSV 저장
    writematrix(X_history, fullfile(save_dir, 'X_history.csv'));
    writematrix(R_history, fullfile(save_dir, 'R_history.csv'));
    writematrix(f_history, fullfile(save_dir, 'f_history.csv'));
    
    % 2) Params 구조체 TXT 저장
    param_file = fullfile(save_dir, 'params.txt');
    fid = fopen(param_file, 'w');
    if fid ~= -1
        fprintf(fid, '=== Simulation Parameters ===\n');
        fields = fieldnames(params);
        for i = 1:numel(fields)
            val = params.(fields{i});
            if isnumeric(val) || islogical(val)
                fprintf(fid, '%s: %s\n', fields{i}, mat2str(val));
            elseif ischar(val) || isstring(val)
                fprintf(fid, '%s: %s\n', fields{i}, char(val));
            elseif isstruct(val)
                fprintf(fid, '%s: [Struct]\n', fields{i});
                subfields = fieldnames(val);
                for j = 1:numel(subfields)
                    subval = val.(subfields{j});
                    if isnumeric(subval) || islogical(subval)
                        fprintf(fid, '  .%s: %s\n', subfields{j}, mat2str(subval));
                    elseif ischar(subval) || isstring(subval)
                        fprintf(fid, '  .%s: %s\n', subfields{j}, char(subval));
                    end
                end
            else
                fprintf(fid, '%s: [Unsupported Type]\n', fields{i});
            end
        end
        fclose(fid);
    else
        warning('파라미터 텍스트 파일을 생성할 수 없습니다.');
    end
    
    disp(['[' save_dir ']']);
end
