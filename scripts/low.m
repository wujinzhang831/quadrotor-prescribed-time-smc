%% 对比分析：姿态指令滤波前后对四旋翼无人机控制性能的影响
% 1. 无滤波：直接使用 quadrotor_smc 输出的期望角度
% 2. 有滤波：对期望角度进行一阶低通滤波（fc=5Hz）
% 然后分别进行相同的切换策略（滑模→PID）和内环 PID 伺服控制
% 绘制三个对比图，并增加性能分析

clear; clc; close all;

%% ================== 通用仿真参数 ==================
desired_pos = [0, 0, 0];   % 目标位置 (m)
T_preset = 9;              % 预设收敛时间 (s)
dt = 0.001;                % 仿真步长 (s)
convergence_threshold = 0.01;  % 位置收敛阈值 (m)

% 切换时间（与原始文件一致）
switch_time = T_preset - 0.495*exp(9-T_preset) - 0.405;

% PID 位置控制器参数（切换后使用）
Kp_x = 64.5;   Ki_x = 20;   Kd_x = 30;
Kp_y = 64.5;   Ki_y = 20;   Kd_y = 30;
Kp_z = 64.5;   Ki_z = 20;   Kd_z = 35;
max_force = 30;

% 低通滤波器参数（仅用于有滤波情况）
fc = 5;                    % 截止频率 5Hz
tau = 1 / (2*pi*fc);
alpha = exp(-dt / tau);    % 滤波系数

%% ================== 仿真 1：无滤波 ==================
fprintf('正在运行无滤波仿真...\n');
[t1, angles_raw, forces_raw] = quadrotor_smc(desired_pos, T_preset);
N1 = length(t1);

% 将滑模输出的期望角度直接用于后续控制（不滤波）
angles_nofilter = angles_raw;

% 位置动力学（含切换和 PID 位置控制器）
[x_nofilter, y_nofilter, z_nofilter, vx_nofilter, vy_nofilter, vz_nofilter, ...
 ax_nofilter, ay_nofilter, az_nofilter, angles_out_nofilter] = ...
    run_dynamics(t1, angles_nofilter, forces_raw, desired_pos, ...
                 switch_time, dt, Kp_x, Ki_x, Kd_x, Kp_y, Ki_y, Kd_y, Kp_z, Ki_z, Kd_z, max_force);

% 内环 PID 姿态跟踪
[angles_actual_nofilter, ~, ~] = PID_smc(t1, angles_out_nofilter);

%% ================== 仿真 2：有低通滤波 ==================
fprintf('正在运行有滤波仿真...\n');
[t2, angles_raw2, forces_raw2] = quadrotor_smc2(desired_pos, T_preset);
N2 = length(t2);

% 对期望角度进行一阶低通滤波
roll_filt = zeros(1, N2);
pitch_filt = zeros(1, N2);
yaw_filt = zeros(1, N2);
roll_filt(1) = angles_raw2.roll(1);
pitch_filt(1) = angles_raw2.pitch(1);
yaw_filt(1) = angles_raw2.yaw(1);
for k = 2:N2
    roll_filt(k)  = alpha * roll_filt(k-1)  + (1-alpha) * angles_raw2.roll(k);
    pitch_filt(k) = alpha * pitch_filt(k-1) + (1-alpha) * angles_raw2.pitch(k);
    yaw_filt(k)   = alpha * yaw_filt(k-1)   + (1-alpha) * angles_raw2.yaw(k);
end
angles_filtered.roll  = roll_filt;
angles_filtered.pitch = pitch_filt;
angles_filtered.yaw   = yaw_filt;

% 位置动力学
[x_filter, y_filter, z_filter, vx_filter, vy_filter, vz_filter, ...
 ax_filter, ay_filter, az_filter, angles_out_filter] = ...
    run_dynamics(t2, angles_filtered, forces_raw2, desired_pos, ...
                 switch_time, dt, Kp_x, Ki_x, Kd_x, Kp_y, Ki_y, Kd_y, Kp_z, Ki_z, Kd_z, max_force);

% 内环 PID 姿态跟踪
[angles_actual_filter, ~, ~] = PID_smc(t2, angles_out_filter);

%% ================== 图 1：滤波前后姿态指令对比 ==================
figure('Name', '姿态指令对比', 'Position', [100, 100, 1200, 800]);

% 滚转角指令
subplot(3,1,1);
plot(t1, angles_raw.roll*180/pi, 'r--', 'LineWidth', 1.2); hold on;
plot(t2, angles_filtered.roll*180/pi, 'b-', 'LineWidth', 1.2);
xlabel('时间 (s)'); ylabel('滚转角 (deg)');
title('期望滚转角指令对比');
legend('原始 (无滤波)', '低通滤波后', 'Location', 'best');
grid on; xlim([0, T_preset*1.2]);

% 俯仰角指令
subplot(3,1,2);
plot(t1, angles_raw.pitch*180/pi, 'r--', 'LineWidth', 1.2); hold on;
plot(t2, angles_filtered.pitch*180/pi, 'b-', 'LineWidth', 1.2);
xlabel('时间 (s)'); ylabel('俯仰角 (deg)');
title('期望俯仰角指令对比');
legend('原始 (无滤波)', '低通滤波后', 'Location', 'best');
grid on; xlim([0, T_preset*1.2]);

% 偏航角指令
subplot(3,1,3);
plot(t1, angles_raw.yaw*180/pi, 'r--', 'LineWidth', 1.2); hold on;
plot(t2, angles_filtered.yaw*180/pi, 'b-', 'LineWidth', 1.2);
xlabel('时间 (s)'); ylabel('偏航角 (deg)');
title('期望偏航角指令对比');
legend('原始 (无滤波)', '低通滤波后', 'Location', 'best');
grid on; xlim([0, T_preset*1.2]);
sgtitle('滤波前后姿态指令对比', 'FontSize', 14, 'FontWeight', 'bold');

%% ================== 图 2：内环姿态跟踪误差对比（有滤波 vs 无滤波）==================
figure('Name', '姿态跟踪误差对比', 'Position', [100, 100, 1200, 800]);

% 滚转角跟踪误差
subplot(3,1,1);
err_roll_nofilter = (angles_out_nofilter.roll - angles_actual_nofilter.roll) * 180/pi;
err_roll_filter   = (angles_out_filter.roll   - angles_actual_filter.roll)   * 180/pi;
plot(t1, err_roll_nofilter, 'r--', 'LineWidth', 1.2); hold on;
plot(t2, err_roll_filter,   'b-',  'LineWidth', 1.2);
xlabel('时间 (s)'); ylabel('滚转角误差 (deg)');
title('滚转角跟踪误差');
legend('无滤波', '有滤波', 'Location', 'best');
grid on; xlim([0, T_preset*1.2]);

% 俯仰角跟踪误差
subplot(3,1,2);
err_pitch_nofilter = (angles_out_nofilter.pitch - angles_actual_nofilter.pitch) * 180/pi;
err_pitch_filter   = (angles_out_filter.pitch   - angles_actual_filter.pitch)   * 180/pi;
plot(t1, err_pitch_nofilter, 'r--', 'LineWidth', 1.2); hold on;
plot(t2, err_pitch_filter,   'b-',  'LineWidth', 1.2);
xlabel('时间 (s)'); ylabel('俯仰角误差 (deg)');
title('俯仰角跟踪误差');
legend('无滤波', '有滤波', 'Location', 'best');
grid on; xlim([0, T_preset*1.2]);

% 偏航角跟踪误差
subplot(3,1,3);
err_yaw_nofilter = (angles_out_nofilter.yaw - angles_actual_nofilter.yaw) * 180/pi;
err_yaw_filter   = (angles_out_filter.yaw   - angles_actual_filter.yaw)   * 180/pi;
plot(t1, err_yaw_nofilter, 'r--', 'LineWidth', 1.2); hold on;
plot(t2, err_yaw_filter,   'b-',  'LineWidth', 1.2);
xlabel('时间 (s)'); ylabel('偏航角误差 (deg)');
title('偏航角跟踪误差');
legend('无滤波', '有滤波', 'Location', 'best');
grid on; xlim([0, T_preset*1.2]);

sgtitle('加入滤波器后的内环姿态跟踪误差对比', 'FontSize', 14, 'FontWeight', 'bold');

%% ================== 图 3：加入滤波器后的位置跟踪响应 ==================
figure('Name', '位置跟踪响应（有滤波）', 'Position', [100, 100, 1200, 400]);

subplot(1,3,1);
plot(t2, x_filter, 'b-', 'LineWidth', 1.5); hold on;
plot(t2, ones(size(t2))*desired_pos(1), 'r--', 'LineWidth', 1.2);
xlabel('时间 (s)'); ylabel('X位置 (m)');
title('X方向位置');
legend('实际', '目标', 'Location', 'best');
grid on; xlim([0, T_preset*1.2]);

subplot(1,3,2);
plot(t2, y_filter, 'b-', 'LineWidth', 1.5); hold on;
plot(t2, ones(size(t2))*desired_pos(2), 'r--', 'LineWidth', 1.2);
xlabel('时间 (s)'); ylabel('Y位置 (m)');
title('Y方向位置');
legend('实际', '目标', 'Location', 'best');
grid on; xlim([0, T_preset*1.2]);

subplot(1,3,3);
plot(t2, z_filter, 'b-', 'LineWidth', 1.5); hold on;
plot(t2, ones(size(t2))*desired_pos(3), 'r--', 'LineWidth', 1.2);
xlabel('时间 (s)'); ylabel('Z位置 (m)');
title('Z方向位置');
legend('实际', '目标', 'Location', 'best');
grid on; xlim([0, T_preset*1.2]);

sgtitle('加入低通滤波器后的位置跟踪响应', 'FontSize', 14, 'FontWeight', 'bold');

fprintf('\n所有绘图完成！\n');

%% ================== 新增：滤波器性能影响分析 ==================
fprintf('\n========== 滤波器性能影响分析 ==========\n');

% 1. 姿态指令平滑度（总变差）
TV_roll_raw = sum(abs(diff(angles_raw.roll)));
TV_roll_filt = sum(abs(diff(angles_filtered.roll)));
TV_pitch_raw = sum(abs(diff(angles_raw.pitch)));
TV_pitch_filt = sum(abs(diff(angles_filtered.pitch)));
TV_yaw_raw = sum(abs(diff(angles_raw.yaw)));
TV_yaw_filt = sum(abs(diff(angles_filtered.yaw)));

fprintf('姿态指令总变差（越小越平滑）:\n');
fprintf('  滚转角: 原始 %.4f rad, 滤波后 %.4f rad (改善 %.2f%%)\n', ...
    TV_roll_raw, TV_roll_filt, (1 - TV_roll_filt/TV_roll_raw)*100);
fprintf('  俯仰角: 原始 %.4f rad, 滤波后 %.4f rad (改善 %.2f%%)\n', ...
    TV_pitch_raw, TV_pitch_filt, (1 - TV_pitch_filt/TV_pitch_raw)*100);
fprintf('  偏航角: 原始 %.4f rad, 滤波后 %.4f rad (改善 %.2f%%)\n', ...
    TV_yaw_raw, TV_yaw_filt, (1 - TV_yaw_filt/TV_yaw_raw)*100);

% 2. 内环姿态跟踪误差 RMSE 和最大误差
% 误差（弧度）
err_roll_nofilter_rad = angles_out_nofilter.roll - angles_actual_nofilter.roll;
err_pitch_nofilter_rad = angles_out_nofilter.pitch - angles_actual_nofilter.pitch;
err_yaw_nofilter_rad = angles_out_nofilter.yaw - angles_actual_nofilter.yaw;
err_roll_filter_rad = angles_out_filter.roll - angles_actual_filter.roll;
err_pitch_filter_rad = angles_out_filter.pitch - angles_actual_filter.pitch;
err_yaw_filter_rad = angles_out_filter.yaw - angles_actual_filter.yaw;

% RMSE
rmse_roll_nofilter = sqrt(mean(err_roll_nofilter_rad.^2));
rmse_roll_filter = sqrt(mean(err_roll_filter_rad.^2));
rmse_pitch_nofilter = sqrt(mean(err_pitch_nofilter_rad.^2));
rmse_pitch_filter = sqrt(mean(err_pitch_filter_rad.^2));
rmse_yaw_nofilter = sqrt(mean(err_yaw_nofilter_rad.^2));
rmse_yaw_filter = sqrt(mean(err_yaw_filter_rad.^2));

% 最大绝对误差（度）
max_err_roll_nofilter_deg = max(abs(err_roll_nofilter_rad))*180/pi;
max_err_roll_filter_deg = max(abs(err_roll_filter_rad))*180/pi;
max_err_pitch_nofilter_deg = max(abs(err_pitch_nofilter_rad))*180/pi;
max_err_pitch_filter_deg = max(abs(err_pitch_filter_rad))*180/pi;
max_err_yaw_nofilter_deg = max(abs(err_yaw_nofilter_rad))*180/pi;
max_err_yaw_filter_deg = max(abs(err_yaw_filter_rad))*180/pi;

fprintf('\n内环姿态跟踪误差 RMSE (rad):\n');
fprintf('  滚转角: 原始 %.6f, 滤波后 %.6f (改善 %.2f%%)\n', ...
    rmse_roll_nofilter, rmse_roll_filter, (1 - rmse_roll_filter/rmse_roll_nofilter)*100);
fprintf('  俯仰角: 原始 %.6f, 滤波后 %.6f (改善 %.2f%%)\n', ...
    rmse_pitch_nofilter, rmse_pitch_filter, (1 - rmse_pitch_filter/rmse_pitch_nofilter)*100);
fprintf('  偏航角: 原始 %.6f, 滤波后 %.6f (改善 %.2f%%)\n', ...
    rmse_yaw_nofilter, rmse_yaw_filter, (1 - rmse_yaw_filter/rmse_yaw_nofilter)*100);

fprintf('\n内环姿态跟踪最大绝对误差 (deg):\n');
fprintf('  滚转角: 原始 %.4f°, 滤波后 %.4f°\n', max_err_roll_nofilter_deg, max_err_roll_filter_deg);
fprintf('  俯仰角: 原始 %.4f°, 滤波后 %.4f°\n', max_err_pitch_nofilter_deg, max_err_pitch_filter_deg);
fprintf('  偏航角: 原始 %.4f°, 滤波后 %.4f°\n', max_err_yaw_nofilter_deg, max_err_yaw_filter_deg);

% 3. 位置跟踪性能
x_err_nofilter = x_nofilter - desired_pos(1);
y_err_nofilter = y_nofilter - desired_pos(2);
z_err_nofilter = z_nofilter - desired_pos(3);
x_err_filter = x_filter - desired_pos(1);
y_err_filter = y_filter - desired_pos(2);
z_err_filter = z_filter - desired_pos(3);

% 位置 RMSE
rmse_pos_nofilter = sqrt(mean(x_err_nofilter.^2 + y_err_nofilter.^2 + z_err_nofilter.^2));
rmse_pos_filter = sqrt(mean(x_err_filter.^2 + y_err_filter.^2 + z_err_filter.^2));

% 收敛时间（进入 ±0.01m 误差带并保持）
threshold = 0.01;
% 无滤波
idx_nofilter = find(all(abs([x_err_nofilter; y_err_nofilter; z_err_nofilter]) <= threshold, 1), 1, 'first');
if ~isempty(idx_nofilter)
    settling_time_nofilter = t1(idx_nofilter);
else
    settling_time_nofilter = NaN;
end
% 有滤波
idx_filter = find(all(abs([x_err_filter; y_err_filter; z_err_filter]) <= threshold, 1), 1, 'first');
if ~isempty(idx_filter)
    settling_time_filter = t2(idx_filter);
else
    settling_time_filter = NaN;
end

% 最终稳态误差（最后 0.5 秒的平均）
steady_len = round(0.5 / dt);
if length(x_err_nofilter) >= steady_len
    steady_err_nofilter = mean(sqrt(x_err_nofilter(end-steady_len+1:end).^2 + ...
                                    y_err_nofilter(end-steady_len+1:end).^2 + ...
                                    z_err_nofilter(end-steady_len+1:end).^2));
    steady_err_filter = mean(sqrt(x_err_filter(end-steady_len+1:end).^2 + ...
                                  y_err_filter(end-steady_len+1:end).^2 + ...
                                  z_err_filter(end-steady_len+1:end).^2));
else
    steady_err_nofilter = NaN;
    steady_err_filter = NaN;
end

fprintf('\n位置跟踪性能:\n');
fprintf('  位置 RMSE (m): 原始 %.4f, 滤波后 %.4f (改善 %.2f%%)\n', ...
    rmse_pos_nofilter, rmse_pos_filter, (1 - rmse_pos_filter/rmse_pos_nofilter)*100);
fprintf('  收敛时间 (s): 原始 %.3f, 滤波后 %.3f\n', settling_time_nofilter, settling_time_filter);
fprintf('  稳态误差 (m): 原始 %.6f, 滤波后 %.6f\n', steady_err_nofilter, steady_err_filter);

% 图4：性能对比条形图
figure('Name', '性能对比', 'Position', [100, 100, 1000, 600]);

% 子图1：姿态指令总变差对比
subplot(2,2,1);
bar_data = [TV_roll_raw, TV_roll_filt; TV_pitch_raw, TV_pitch_filt; TV_yaw_raw, TV_yaw_filt];
bar(bar_data);
set(gca, 'XTickLabel', {'Roll', 'Pitch', 'Yaw'});
ylabel('总变差 (rad)');
title('姿态指令平滑度');
legend('原始', '滤波后');
grid on;

% 子图2：内环跟踪RMSE对比
subplot(2,2,2);
bar_data_rmse = [rmse_roll_nofilter, rmse_roll_filter; ...
                 rmse_pitch_nofilter, rmse_pitch_filter; ...
                 rmse_yaw_nofilter, rmse_yaw_filter];
bar(bar_data_rmse);
set(gca, 'XTickLabel', {'Roll', 'Pitch', 'Yaw'});
ylabel('RMSE (rad)');
title('内环姿态跟踪误差');
legend('原始', '滤波后');
grid on;

% 子图3：位置RMSE对比
subplot(2,2,3);
bar([rmse_pos_nofilter, rmse_pos_filter]);
set(gca, 'XTickLabel', {'位置误差'});
ylabel('RMSE (m)');
title('位置跟踪误差');
legend('原始', '滤波后');
grid on;

% 子图4：收敛时间对比
subplot(2,2,4);
bar([settling_time_nofilter, settling_time_filter]);
set(gca, 'XTickLabel', {'收敛时间'});
ylabel('时间 (s)');
title('收敛时间 (进入±0.01m)');
legend('原始', '滤波后');
grid on;

sgtitle('滤波器性能影响对比', 'FontSize', 14, 'FontWeight', 'bold');

fprintf('\n分析完成！\n');

%% ================== 辅助函数 ==================

% 位置动力学（包含滑模→PID切换，改进欧拉法）
function [x, y, z, vx, vy, vz, ax, ay, az, angles_out] = ...
    run_dynamics(t, angles_desired, forces, desired_pos, switch_time, dt, ...
                 Kp_x, Ki_x, Kd_x, Kp_y, Ki_y, Kd_y, Kp_z, Ki_z, Kd_z, max_force)
    N = length(t);
    % 系统参数
    m = 0.905;   g = 9.8;
    kx = 0.1; ky = 0.1; kz = 0.1;
    x0 = 5; y0 = -3; z0 = 10;
    
    % 状态初始化
    x = zeros(1,N); y = zeros(1,N); z = zeros(1,N);
    vx = zeros(1,N); vy = zeros(1,N); vz = zeros(1,N);
    ax = zeros(1,N); ay = zeros(1,N); az = zeros(1,N);
    x(1)=x0; y(1)=y0; z(1)=z0;
    
    % 外部扰动
    d_max = 0.5;
    freq_x=0.5; freq_y=0.6; freq_z=0.7;
    d_x_func = @(tt) d_max * sin(2*pi*freq_x*tt);
    d_y_func = @(tt) d_max * cos(2*pi*freq_y*tt);
    d_z_func = @(tt) d_max * sin(2*pi*freq_z*tt + pi/4);
    
    % PID 积分项
    integral_x = 0; integral_y = 0; integral_z = 0;
    
    % 存储实际使用的期望角度（可能切换）
    roll_out = zeros(1,N);
    pitch_out = zeros(1,N);
    yaw_out = zeros(1,N);
    
    for k = 1:N-1
        t_k = t(k);
        % 判断当前时刻使用滑模还是 PID
        if t_k >= switch_time
            e_x = desired_pos(1) - x(k);
            e_y = desired_pos(2) - y(k);
            e_z = desired_pos(3) - z(k);
            integral_x = integral_x + e_x * dt;
            integral_y = integral_y + e_y * dt;
            integral_z = integral_z + e_z * dt;
            integral_x = max(min(integral_x, 10), -10);
            integral_y = max(min(integral_y, 10), -10);
            integral_z = max(min(integral_z, 10), -10);
            [u_x_k, u_y_k, u_z_k, phi_k, theta_k, psi_k] = ...
                compute_pid_u(x(k), y(k), z(k), vx(k), vy(k), vz(k), ...
                              desired_pos, m, g, Kp_x, Ki_x, Kd_x, Kp_y, Ki_y, Kd_y, Kp_z, Ki_z, Kd_z, max_force, ...
                              integral_x, integral_y, integral_z);
            roll_out(k) = phi_k;
            pitch_out(k) = theta_k;
            yaw_out(k) = psi_k;
        else
            u_x_k = forces.u_x(k);
            u_y_k = forces.u_y(k);
            u_z_k = forces.u_z(k);
            roll_out(k) = angles_desired.roll(k);
            pitch_out(k) = angles_desired.pitch(k);
            yaw_out(k) = angles_desired.yaw(k);
        end
        
        % 扰动
        d_x = d_x_func(t_k)/m;
        d_y = d_y_func(t_k)/m;
        d_z = d_z_func(t_k)/m;
        
        % 预测加速度
        ax_pred = u_x_k/m - (kx/m)*vx(k) + d_x;
        ay_pred = u_y_k/m - (ky/m)*vy(k) + d_y;
        az_pred = u_z_k/m - g - (kz/m)*vz(k) + d_z;
        
        vx_pred = vx(k) + ax_pred * dt;
        vy_pred = vy(k) + ay_pred * dt;
        vz_pred = vz(k) + az_pred * dt;
        x_pred = x(k) + vx(k) * dt;
        y_pred = y(k) + vy(k) * dt;
        z_pred = z(k) + vz(k) * dt;
        
        % 下一时刻控制力（用于校正）
        t_next = t(k+1);
        if t_next >= switch_time
            [u_x_next, u_y_next, u_z_next, ~, ~, ~] = ...
                compute_pid_u(x_pred, y_pred, z_pred, vx_pred, vy_pred, vz_pred, ...
                              desired_pos, m, g, Kp_x, Ki_x, Kd_x, Kp_y, Ki_y, Kd_y, Kp_z, Ki_z, Kd_z, max_force, ...
                              integral_x, integral_y, integral_z);
        else
            u_x_next = forces.u_x(k+1);
            u_y_next = forces.u_y(k+1);
            u_z_next = forces.u_z(k+1);
        end
        d_x_next = d_x_func(t_next)/m;
        d_y_next = d_y_func(t_next)/m;
        d_z_next = d_z_func(t_next)/m;
        
        ax_corr = u_x_next/m - (kx/m)*vx_pred + d_x_next;
        ay_corr = u_y_next/m - (ky/m)*vy_pred + d_y_next;
        az_corr = u_z_next/m - g - (kz/m)*vz_pred + d_z_next;
        
        ax_avg = (ax_pred + ax_corr)/2;
        ay_avg = (ay_pred + ay_corr)/2;
        az_avg = (az_pred + az_corr)/2;
        
        ax(k) = ax_avg; ay(k) = ay_avg; az(k) = az_avg;
        
        vx(k+1) = vx(k) + ax_avg * dt;
        vy(k+1) = vy(k) + ay_avg * dt;
        vz(k+1) = vz(k) + az_avg * dt;
        x(k+1) = x(k) + (vx(k) + vx(k+1))/2 * dt;
        y(k+1) = y(k) + (vy(k) + vy(k+1))/2 * dt;
        z(k+1) = z(k) + (vz(k) + vz(k+1))/2 * dt;
        
        % 数值稳定
        vx(k+1) = max(min(vx(k+1), 10), -10);
        vy(k+1) = max(min(vy(k+1), 10), -10);
        vz(k+1) = max(min(vz(k+1), 10), -10);
    end
    % 最后一时刻赋值
    ax(N) = ax(N-1); ay(N) = ay(N-1); az(N) = az(N-1);
    roll_out(N) = roll_out(N-1);
    pitch_out(N) = pitch_out(N-1);
    yaw_out(N) = yaw_out(N-1);
    angles_out.roll = roll_out;
    angles_out.pitch = pitch_out;
    angles_out.yaw = yaw_out;
end

% 位置 PID + 姿态解算（偏航角设为 0.2 rad）
function [u_x, u_y, u_z, phi, theta, psi] = compute_pid_u(x, y, z, vx, vy, vz, ...
    desired_pos, m, g, Kp_x, Ki_x, Kd_x, Kp_y, Ki_y, Kd_y, Kp_z, Ki_z, Kd_z, max_force, ...
    integral_x, integral_y, integral_z)
    e_x = desired_pos(1) - x;
    e_y = desired_pos(2) - y;
    e_z = desired_pos(3) - z;
    u_x = Kp_x * e_x + Ki_x * integral_x - Kd_x * vx;
    u_y = Kp_y * e_y + Ki_y * integral_y - Kd_y * vy;
    u_z = Kp_z * e_z + Ki_z * integral_z - Kd_z * vz + m * g;
    u_x = max(min(u_x, max_force), -max_force);
    u_y = max(min(u_y, max_force), -max_force);
    u_z = max(min(u_z, max_force), -max_force);
    
    psi = 0.2;   % 偏航角指令
    f = m * (g - u_z/m);
    f = max(f, 1.0);
    a_x = -u_x / m;
    a_y = -u_y / m;
    sin_phi = (a_y * m) / f;
    sin_phi = max(min(sin_phi, 1), -1);
    phi = asin(sin_phi);
    cos_phi = cos(phi);
    if abs(cos_phi) < 1e-6
        theta = 0;
    else
        sin_theta = -(a_x * m) / (f * cos_phi);
        sin_theta = max(min(sin_theta, 1), -1);
        theta = asin(sin_theta);
    end
    max_angle = 30 * pi/180;
    phi = max(min(phi, max_angle), -max_angle);
    theta = max(min(theta, max_angle), -max_angle);
end

% 内环 PID 姿态控制器（与原始文件一致）
function [angles_actual, control_torques, performance_summary] = PID_smc(t, angles_desired, ~)
    dt = t(2)-t(1);
    N = length(t);
    roll_desired = angles_desired.roll;
    pitch_desired = angles_desired.pitch;
    yaw_desired = angles_desired.yaw;
    
    Kp_roll=5000; Ki_roll=10; Kd_roll=150;
    Kp_pitch=5000; Ki_pitch=1; Kd_pitch=150;
    Kp_yaw=8; Ki_yaw=2; Kd_yaw=1.5;
    max_torque=0.5;
    Ixx=0.0081; Iyy=0.0081; Izz=0.0142;
    B_roll=0.05; B_pitch=0.05; B_yaw=0.03;
    
    roll_actual=zeros(1,N); pitch_actual=zeros(1,N); yaw_actual=zeros(1,N);
    roll_rate=zeros(1,N); pitch_rate=zeros(1,N); yaw_rate=zeros(1,N);
    roll_acc=zeros(1,N); pitch_acc=zeros(1,N); yaw_acc=zeros(1,N);
    roll_error=zeros(1,N); pitch_error=zeros(1,N); yaw_error=zeros(1,N);
    roll_error_int=0; pitch_error_int=0; yaw_error_int=0;
    roll_error_deriv=0; pitch_error_deriv=0; yaw_error_deriv=0;
    roll_torque=zeros(1,N); pitch_torque=zeros(1,N); yaw_torque=zeros(1,N);
    
    for k=1:N-1
        % Roll
        roll_error(k)=roll_desired(k)-roll_actual(k);
        roll_error_int=roll_error_int+roll_error(k)*dt;
        roll_error_int=max(min(roll_error_int,2/Ki_roll),-2/Ki_roll);
        if k>1
            raw_deriv=(roll_error(k)-roll_error(k-1))/dt;
            roll_error_deriv=0.1*raw_deriv+0.9*roll_error_deriv;
        else
            roll_error_deriv=0;
        end
        roll_torque(k)=Kp_roll*roll_error(k)+Ki_roll*roll_error_int+Kd_roll*roll_error_deriv;
        roll_torque(k)=max(min(roll_torque(k),max_torque),-max_torque);
        % Pitch
        pitch_error(k)=pitch_desired(k)-pitch_actual(k);
        pitch_error_int=pitch_error_int+pitch_error(k)*dt;
        pitch_error_int=max(min(pitch_error_int,2/Ki_pitch),-2/Ki_pitch);
        if k>1
            raw_deriv=(pitch_error(k)-pitch_error(k-1))/dt;
            pitch_error_deriv=0.1*raw_deriv+0.9*pitch_error_deriv;
        else
            pitch_error_deriv=0;
        end
        pitch_torque(k)=Kp_pitch*pitch_error(k)+Ki_pitch*pitch_error_int+Kd_pitch*pitch_error_deriv;
        pitch_torque(k)=max(min(pitch_torque(k),max_torque),-max_torque);
        % Yaw
        yaw_error(k)=yaw_desired(k)-yaw_actual(k);
        yaw_error_int=yaw_error_int+yaw_error(k)*dt;
        yaw_error_int=max(min(yaw_error_int,2/Ki_yaw),-2/Ki_yaw);
        if k>1
            raw_deriv=(yaw_error(k)-yaw_error(k-1))/dt;
            yaw_error_deriv=0.1*raw_deriv+0.9*yaw_error_deriv;
        else
            yaw_error_deriv=0;
        end
        yaw_torque(k)=Kp_yaw*yaw_error(k)+Ki_yaw*yaw_error_int+Kd_yaw*yaw_error_deriv;
        yaw_torque(k)=max(min(yaw_torque(k),max_torque),-max_torque);
        % Dynamics
        roll_acc(k)=(roll_torque(k)-B_roll*roll_rate(k))/Ixx;
        roll_rate(k+1)=roll_rate(k)+roll_acc(k)*dt;
        roll_actual(k+1)=roll_actual(k)+roll_rate(k)*dt;
        pitch_acc(k)=(pitch_torque(k)-B_pitch*pitch_rate(k))/Iyy;
        pitch_rate(k+1)=pitch_rate(k)+pitch_acc(k)*dt;
        pitch_actual(k+1)=pitch_actual(k)+pitch_rate(k)*dt;
        yaw_acc(k)=(yaw_torque(k)-B_yaw*yaw_rate(k))/Izz;
        yaw_rate(k+1)=yaw_rate(k)+yaw_acc(k)*dt;
        yaw_actual(k+1)=yaw_actual(k)+yaw_rate(k)*dt;
    end
    roll_error(end)=roll_desired(end)-roll_actual(end);
    pitch_error(end)=pitch_desired(end)-pitch_actual(end);
    yaw_error(end)=yaw_desired(end)-yaw_actual(end);
    roll_acc(end)=roll_acc(end-1); pitch_acc(end)=pitch_acc(end-1); yaw_acc(end)=yaw_acc(end-1);
    
    angles_actual.roll=roll_actual; angles_actual.pitch=pitch_actual; angles_actual.yaw=yaw_actual;
    control_torques.roll=roll_torque; control_torques.pitch=pitch_torque; control_torques.yaw=yaw_torque;
    performance_summary.roll.max_error_deg=max(abs(roll_error))*180/pi;
    performance_summary.pitch.max_error_deg=max(abs(pitch_error))*180/pi;
    performance_summary.yaw.max_error_deg=max(abs(yaw_error))*180/pi;
end