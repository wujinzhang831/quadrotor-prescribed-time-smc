clear; clc; close all;

%% ================== 预设参数配置 ==================
desired_pos = [0, 0, 0];   % 目标位置 [x,y,z] (m)
T_preset = 9;              % 预设收敛时间 (s)
convergence_threshold = 0.01;  % 位置收敛阈值 (m)
dt = 0.001;                % 仿真步长 (s)

%% ================== 调用外环滑模控制函数 ==================
[t, angles_smc, control_forces] = quadrotor_smc2(desired_pos, T_preset);
N = length(t);     % 仿真总步数

%% ================== 低通滤波器设计 ==================
fc = 5;                % 截止频率 (Hz)
tau = 1 / (2*pi*fc);   % 时间常数 (s)
alpha = exp(-dt / tau);% 滤波系数

% 对三个期望角度分别进行滤波
roll_raw = angles_smc.roll;
pitch_raw = angles_smc.pitch;
yaw_raw = angles_smc.yaw;

roll_filt = zeros(1, N);
pitch_filt = zeros(1, N);
yaw_filt = zeros(1, N);

% 初始化
roll_filt(1) = roll_raw(1);
pitch_filt(1) = pitch_raw(1);
yaw_filt(1) = yaw_raw(1);

% 递推滤波
for k = 2:N
    roll_filt(k) = alpha * roll_filt(k-1) + (1-alpha) * roll_raw(k);
    pitch_filt(k) = alpha * pitch_filt(k-1) + (1-alpha) * pitch_raw(k);
    yaw_filt(k) = alpha * yaw_filt(k-1) + (1-alpha) * yaw_raw(k);
end

% 将滤波后的角度赋回结构体
angles_smc.roll = roll_filt;
angles_smc.pitch = pitch_filt;
angles_smc.yaw = yaw_filt;

%% ================== 切换时间及 PID 参数设置 ==================
switch_time = T_preset - 0.495*exp(9-T_preset) - 0.405;   % 切换时刻 (s)
idx_switch = find(t >= switch_time, 1, 'first');          % 切换点索引

% PID 位置控制器参数（需根据实际调整）
Kp_x = 64.5;   Ki_x = 20;   Kd_x = 30;   % X方向
Kp_y = 64.5;   Ki_y = 20;   Kd_y = 30;   % Y方向
Kp_z = 64.5;   Ki_z = 20;   Kd_z = 35;   % Z方向（已包含重力补偿）
max_force = 30;            % 控制力限幅 (N)

% 初始化积分项
integral_x = 0;
integral_y = 0;
integral_z = 0;

% 初始化新的期望角度和控制力数组（先复制原值，后面覆盖切换后的部分）
new_roll   = angles_smc.roll;
new_pitch  = angles_smc.pitch;
new_yaw    = angles_smc.yaw;
new_u_x    = control_forces.u_x;
new_u_y    = control_forces.u_y;
new_u_z    = control_forces.u_z;

%% ================== 无人机实际位置轨迹（含控制切换）==================
m = 0.905;   % 质量 (kg)
g = 9.8;     % 重力加速度 (m/s^2)
kx = 0.1;    % X方向阻力系数
ky = 0.1;    % Y方向阻力系数
kz = 0.1;    % Z方向阻力系数

% 初始位置
x0 = 5;     % 初始X位置 (m)
y0 = -3;    % 初始Y位置 (m)
z0 = 10;    % 初始Z位置 (m)

% 初始化状态数组
x_actual = zeros(1, N);  % 实际X位置
y_actual = zeros(1, N);  % 实际Y位置
z_actual = zeros(1, N);  % 实际Z位置
vx_actual = zeros(1, N); % 实际X速度
vy_actual = zeros(1, N); % 实际Y速度
vz_actual = zeros(1, N); % 实际Z速度
ax_actual = zeros(1, N); % 实际X加速度
ay_actual = zeros(1, N); % 实际Y加速度
az_actual = zeros(1, N); % 实际Z加速度

% 设置初始条件
x_actual(1) = x0;
y_actual(1) = y0;
z_actual(1) = z0;

% 外部扰动函数（与 quadrotor_smc2 中一致）
d_max = 0.5;
freq_x = 0.5; freq_y = 0.6; freq_z = 0.7;
d_x_func = @(tt) d_max * sin(2*pi*freq_x*tt);
d_y_func = @(tt) d_max * cos(2*pi*freq_y*tt);
d_z_func = @(tt) d_max * sin(2*pi*freq_z*tt + pi/4);

% ================== 动力学模型（含切换逻辑）==================
for k = 1:N-1
    % 当前状态
    x_k = x_actual(k);
    y_k = y_actual(k);
    z_k = z_actual(k);
    vx_k = vx_actual(k);
    vy_k = vy_actual(k);
    vz_k = vz_actual(k);
    t_k = t(k);

    % ---- 计算当前时刻的控制力 (u_k) 并记录期望角度 ----
    if t_k >= switch_time
        % 使用 PID 位置控制器
        e_x = desired_pos(1) - x_k;
        e_y = desired_pos(2) - y_k;
        e_z = desired_pos(3) - z_k;
        integral_x = integral_x + e_x * dt;
        integral_y = integral_y + e_y * dt;
        integral_z = integral_z + e_z * dt;
        % 积分限幅
        integral_x = max(min(integral_x, 10), -10);
        integral_y = max(min(integral_y, 10), -10);
        integral_z = max(min(integral_z, 10), -10);
        
        [u_x_k, u_y_k, u_z_k, phi_k, theta_k, psi_k] = ...
            compute_pid_u(x_k, y_k, z_k, vx_k, vy_k, vz_k, ...
                          desired_pos, m, g, Kp_x, Ki_x, Kd_x, Kp_y, Ki_y, Kd_y, Kp_z, Ki_z, Kd_z, max_force, ...
                          integral_x, integral_y, integral_z);
        new_u_x(k) = u_x_k;
        new_u_y(k) = u_y_k;
        new_u_z(k) = u_z_k;
        new_roll(k)   = phi_k;
        new_pitch(k)  = theta_k;
        new_yaw(k)    = psi_k;
    else
        % 使用原滑模控制力（已滤波的角度仍保留在 angles_smc 中）
        u_x_k = control_forces.u_x(k);
        u_y_k = control_forces.u_y(k);
        u_z_k = control_forces.u_z(k);
        new_u_x(k) = u_x_k;
        new_u_y(k) = u_y_k;
        new_u_z(k) = u_z_k;
        new_roll(k)   = angles_smc.roll(k);
        new_pitch(k)  = angles_smc.pitch(k);
        new_yaw(k)    = angles_smc.yaw(k);
    end

    % 当前扰动
    d_x_k = d_x_func(t_k) / m;
    d_y_k = d_y_func(t_k) / m;
    d_z_k = d_z_func(t_k) / m;

    % 预测加速度（使用当前控制力）
    ax = u_x_k/m - (kx/m)*vx_k + d_x_k;
    ay = u_y_k/m - (ky/m)*vy_k + d_y_k;
    az = u_z_k/m - g - (kz/m)*vz_k + d_z_k;

    % 预测速度和位置（用于下一步校正）
    vx_pred = vx_k + ax * dt;
    vy_pred = vy_k + ay * dt;
    vz_pred = vz_k + az * dt;
    x_pred = x_k + vx_k * dt;
    y_pred = y_k + vy_k * dt;
    z_pred = z_k + vz_k * dt;

    % ---- 计算下一时刻的控制力 (u_next) 用于校正 ----
    t_next = t(k+1);
    if t_next >= switch_time
        [u_x_next, u_y_next, u_z_next, ~, ~, ~] = ...
            compute_pid_u(x_pred, y_pred, z_pred, vx_pred, vy_pred, vz_pred, ...
                          desired_pos, m, g, Kp_x, Ki_x, Kd_x, Kp_y, Ki_y, Kd_y, Kp_z, Ki_z, Kd_z, max_force, ...
                          integral_x, integral_y, integral_z);
        new_u_x(k+1) = u_x_next;
        new_u_y(k+1) = u_y_next;
        new_u_z(k+1) = u_z_next;
    else
        u_x_next = control_forces.u_x(k+1);
        u_y_next = control_forces.u_y(k+1);
        u_z_next = control_forces.u_z(k+1);
        new_u_x(k+1) = u_x_next;
        new_u_y(k+1) = u_y_next;
        new_u_z(k+1) = u_z_next;
    end

    % 下一时刻扰动
    d_x_next = d_x_func(t_next) / m;
    d_y_next = d_y_func(t_next) / m;
    d_z_next = d_z_func(t_next) / m;

    % 校正加速度（使用 u_next 和预测状态）
    ax_corr = u_x_next/m - (kx/m)*vx_pred + d_x_next;
    ay_corr = u_y_next/m - (ky/m)*vy_pred + d_y_next;
    az_corr = u_z_next/m - g - (kz/m)*vz_pred + d_z_next;

    % 平均加速度
    ax_avg = (ax + ax_corr) / 2;
    ay_avg = (ay + ay_corr) / 2;
    az_avg = (az + az_corr) / 2;

    % 存储平均加速度
    ax_actual(k) = ax_avg;
    ay_actual(k) = ay_avg;
    az_actual(k) = az_avg;

    % 更新速度和位置（梯形法）
    vx_actual(k+1) = vx_k + ax_avg * dt;
    vy_actual(k+1) = vy_k + ay_avg * dt;
    vz_actual(k+1) = vz_k + az_avg * dt;

    x_actual(k+1) = x_k + (vx_k + vx_actual(k+1)) / 2 * dt;
    y_actual(k+1) = y_k + (vy_k + vy_actual(k+1)) / 2 * dt;
    z_actual(k+1) = z_k + (vz_k + vz_actual(k+1)) / 2 * dt;

    % 数值稳定性检查
    vx_actual(k+1) = max(min(vx_actual(k+1), 10), -10);
    vy_actual(k+1) = max(min(vy_actual(k+1), 10), -10);
    vz_actual(k+1) = max(min(vz_actual(k+1), 10), -10);
end

% 最后一个时刻的加速度用前一个值填充
ax_actual(N) = ax_actual(N-1);
ay_actual(N) = ay_actual(N-1);
az_actual(N) = az_actual(N-1);

% ---- 处理最后一个时刻的期望角度 ----
if t(end) >= switch_time
    [~, ~, ~, phi_end, theta_end, psi_end] = ...
        compute_pid_u(x_actual(end), y_actual(end), z_actual(end), ...
                      vx_actual(end), vy_actual(end), vz_actual(end), ...
                      desired_pos, m, g, Kp_x, Ki_x, Kd_x, Kp_y, Ki_y, Kd_y, Kp_z, Ki_z, Kd_z, max_force, ...
                      integral_x, integral_y, integral_z);
    new_roll(end)   = phi_end;
    new_pitch(end)  = theta_end;
    new_yaw(end)    = psi_end;
else
    new_roll(end)   = angles_smc.roll(end);
    new_pitch(end)  = angles_smc.pitch(end);
    new_yaw(end)    = angles_smc.yaw(end);
end

% 更新期望角度和控制力结构体
angles_smc.roll   = new_roll;
angles_smc.pitch  = new_pitch;
angles_smc.yaw    = new_yaw;
control_forces.u_x = new_u_x;
control_forces.u_y = new_u_y;
control_forces.u_z = new_u_z;

% 更新用于绘图的控制力变量
u_x = control_forces.u_x;
u_y = control_forces.u_y;
u_z = control_forces.u_z;

%% ================== 调用内环 PID 伺服控制函数（已添加角度滤波） ==================
% 注意：期望角度 angles_smc 已经经过低通滤波
[angles_actual, control_torques, performance_summary] = PID_smc(t, angles_smc, control_forces);

%% ================== 计算位置误差 ==================
x_error = x_actual - desired_pos(1);
y_error = y_actual - desired_pos(2);
z_error = z_actual - desired_pos(3);

%% ================== 重新计算实际收敛时间 ==================
T_actual = NaN;
for k = 1:N
    if all(abs(x_error(k:end)) < convergence_threshold) && ...
       all(abs(y_error(k:end)) < convergence_threshold) && ...
       all(abs(z_error(k:end)) < convergence_threshold)
        T_actual = t(k);
        break;
    end
end

%% ================== 结果可视化 ==================

% Figure 1：位置、速度、加速度（3x3 子图）
figure('Position', [100, 100, 1400, 1000], 'Name', '位置、速度、加速度响应');

% 位置
subplot(3,3,1);
plot(t, x_actual, 'b-', 'LineWidth', 1.2);
hold on;
plot(t, ones(1,N)*desired_pos(1), 'r--', 'LineWidth', 1);
if ~isnan(T_actual)
    plot([T_actual, T_actual], [min(x_actual), max(x_actual)], 'g:', 'LineWidth', 1.5);
end
xlabel('时间 (s)'); ylabel('X位置 (m)');
title('X位置'); grid on;
xlim([0, T_preset*1.5]);

subplot(3,3,2);
plot(t, y_actual, 'b-', 'LineWidth', 1.2);
hold on;
plot(t, ones(1,N)*desired_pos(2), 'r--', 'LineWidth', 1);
if ~isnan(T_actual)
    plot([T_actual, T_actual], [min(y_actual), max(y_actual)], 'g:', 'LineWidth', 1.5);
end
xlabel('时间 (s)'); ylabel('Y位置 (m)');
title('Y位置'); grid on;
xlim([0, T_preset*1.5]);

subplot(3,3,3);
plot(t, z_actual, 'b-', 'LineWidth', 1.2);
hold on;
plot(t, ones(1,N)*desired_pos(3), 'r--', 'LineWidth', 1);
if ~isnan(T_actual)
    plot([T_actual, T_actual], [min(z_actual), max(z_actual)], 'g:', 'LineWidth', 1.5);
end
xlabel('时间 (s)'); ylabel('Z位置 (m)');
title('Z位置'); grid on;
xlim([0, T_preset*1.5]);

% 速度
subplot(3,3,4);
plot(t, vx_actual, 'b-', 'LineWidth', 1.2);
hold on;
plot([0, max(t)], [0, 0], 'k--', 'LineWidth', 0.5);
if ~isnan(T_actual)
    plot([T_actual, T_actual], [min(vx_actual), max(vx_actual)], 'g:', 'LineWidth', 1.5);
end
xlabel('时间 (s)'); ylabel('v_x (m/s)');
title('X方向速度'); grid on;
xlim([0, T_preset*1.5]);

subplot(3,3,5);
plot(t, vy_actual, 'b-', 'LineWidth', 1.2);
hold on;
plot([0, max(t)], [0, 0], 'k--', 'LineWidth', 0.5);
if ~isnan(T_actual)
    plot([T_actual, T_actual], [min(vy_actual), max(vy_actual)], 'g:', 'LineWidth', 1.5);
end
xlabel('时间 (s)'); ylabel('v_y (m/s)');
title('Y方向速度'); grid on;
xlim([0, T_preset*1.5]);

subplot(3,3,6);
plot(t, vz_actual, 'b-', 'LineWidth', 1.2);
hold on;
plot([0, max(t)], [0, 0], 'k--', 'LineWidth', 0.5);
if ~isnan(T_actual)
    plot([T_actual, T_actual], [min(vz_actual), max(vz_actual)], 'g:', 'LineWidth', 1.5);
end
xlabel('时间 (s)'); ylabel('v_z (m/s)');
title('Z方向速度'); grid on;
xlim([0, T_preset*1.5]);

% 加速度
subplot(3,3,7);
plot(t, ax_actual, 'b-', 'LineWidth', 1.2);
hold on;
plot([0, max(t)], [0, 0], 'k--', 'LineWidth', 0.5);
if ~isnan(T_actual)
    plot([T_actual, T_actual], [min(ax_actual), max(ax_actual)], 'g:', 'LineWidth', 1.5);
end
xlabel('时间 (s)'); ylabel('a_x (m/s^2)');
title('X方向加速度'); grid on;
xlim([0, T_preset*1.5]);

subplot(3,3,8);
plot(t, ay_actual, 'b-', 'LineWidth', 1.2);
hold on;
plot([0, max(t)], [0, 0], 'k--', 'LineWidth', 0.5);
if ~isnan(T_actual)
    plot([T_actual, T_actual], [min(ay_actual), max(ay_actual)], 'g:', 'LineWidth', 1.5);
end
xlabel('时间 (s)'); ylabel('a_y (m/s^2)');
title('Y方向加速度'); grid on;
xlim([0, T_preset*1.5]);

subplot(3,3,9);
plot(t, az_actual, 'b-', 'LineWidth', 1.2);
hold on;
plot([0, max(t)], [-g, -g], 'k--', 'LineWidth', 0.5, 'DisplayName', '-g');
if ~isnan(T_actual)
    plot([T_actual, T_actual], [min(az_actual), max(az_actual)], 'g:', 'LineWidth', 1.5);
end
xlabel('时间 (s)'); ylabel('a_z (m/s^2)');
title('Z方向加速度'); grid on;
xlim([0, T_preset*1.5]);

sgtitle(sprintf('四旋翼无人机状态响应 (实际到达时刻: %.3f s)', T_actual), ...
    'FontSize', 14, 'FontWeight', 'bold');

% Figure 2：3D轨迹
figure('Position', [200, 200, 800, 600], 'Name', '3D轨迹');
plot3(x_actual, y_actual, z_actual, 'b-', 'LineWidth', 1.5);
hold on;
plot3(desired_pos(1), desired_pos(2), desired_pos(3), 'ro', ...
     'MarkerSize', 8, 'LineWidth', 2, 'DisplayName', '目标点');
plot3(x0, y0, z0, 'go', 'MarkerSize', 8, 'LineWidth', 2, 'DisplayName', '起始点');
if ~isnan(T_actual)
    [~, idx] = min(abs(t - T_actual));
    plot3(x_actual(idx), y_actual(idx), z_actual(idx), ...
         'ms', 'MarkerSize', 10, 'LineWidth', 2, ...
         'DisplayName', sprintf('到达点 (t=%.3f s)', T_actual));
end
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
title(['3D飞行轨迹 (预设时间: ', num2str(T_preset), 's)']);
grid on; view(45, 30); legend('Location', 'best');
xlim([min(x_actual)-1, max(x_actual)+1]);
ylim([min(y_actual)-1, max(y_actual)+1]);
zlim([min(z_actual)-1, max(z_actual)+1]);

% Figure 3：姿态角跟踪性能（仅保留期望与实际对比，去除误差子图）
figure('Position', [100, 100, 1200, 800], 'Name', '姿态角跟踪性能');

roll_desired = angles_smc.roll;
pitch_desired = angles_smc.pitch;
yaw_desired = angles_smc.yaw;

roll_actual = angles_actual.roll;
pitch_actual = angles_actual.pitch;
yaw_actual = angles_actual.yaw;

% 滚转角跟踪
subplot(3,1,1);
plot(t, roll_desired*180/pi, 'r--', 'LineWidth', 1.5, 'DisplayName', '期望');
hold on;
plot(t, roll_actual*180/pi, 'b-', 'LineWidth', 1, 'DisplayName', '实际');
if ~isnan(T_actual)
    plot([T_actual, T_actual], [min(roll_actual*180/pi), max(roll_actual*180/pi)], ...
        'g:', 'LineWidth', 1.5, 'DisplayName', sprintf('位置到达: %.3f s', T_actual));
end
xlabel('时间 (s)'); ylabel('滚转角 (°)');
title('滚转角跟踪'); grid on; legend('Location', 'best');
xlim([0, T_preset*1.2]);

% 俯仰角跟踪
subplot(3,1,2);
plot(t, pitch_desired*180/pi, 'r--', 'LineWidth', 1.5, 'DisplayName', '期望');
hold on;
plot(t, pitch_actual*180/pi, 'b-', 'LineWidth', 1, 'DisplayName', '实际');
if ~isnan(T_actual)
    plot([T_actual, T_actual], [min(pitch_actual*180/pi), max(pitch_actual*180/pi)], ...
        'g:', 'LineWidth', 1.5, 'DisplayName', sprintf('位置到达: %.3f s', T_actual));
end
xlabel('时间 (s)'); ylabel('俯仰角 (°)');
title('俯仰角跟踪'); grid on; legend('Location', 'best');
xlim([0, T_preset*1.2]);

% 偏航角跟踪
subplot(3,1,3);
plot(t, yaw_desired*180/pi, 'r--', 'LineWidth', 1.5, 'DisplayName', '期望');
hold on;
plot(t, yaw_actual*180/pi, 'b-', 'LineWidth', 1, 'DisplayName', '实际');
if ~isnan(T_actual)
    plot([T_actual, T_actual], [min(yaw_actual*180/pi), max(yaw_actual*180/pi)], ...
        'g:', 'LineWidth', 1.5, 'DisplayName', sprintf('位置到达: %.3f s', T_actual));
end
xlabel('时间 (s)'); ylabel('偏航角 (°)');
title('偏航角跟踪'); grid on; legend('Location', 'best');
xlim([0, T_preset*1.2]);

sgtitle(sprintf('内环PID伺服控制姿态角跟踪性能 (位置到达时刻: %.3f s)', T_actual), ...
    'FontSize', 14, 'FontWeight', 'bold');

% 命令行输出性能总结
fprintf('\n========== 预定时间滑模控制仿真结果 ==========\n');
fprintf('预设收敛时间 T = %.2f s\n', T_preset);
fprintf('期望目标位置: [%.1f, %.1f, %.1f] m\n', desired_pos(1), desired_pos(2), desired_pos(3));
fprintf('初始位置: [%.1f, %.1f, %.1f] m\n', x0, y0, z0);
fprintf('收敛阈值: %.4f m (位置误差恒小于此值)\n', convergence_threshold);
if ~isnan(T_actual)
    fprintf('实际到达时刻: %.3f s\n', T_actual);
    fprintf('提前/延迟: %.3f s\n', T_preset - T_actual);
    fprintf('最终位置误差: [%.4f, %.4f, %.4f] m\n', x_actual(end), y_actual(end), z_actual(end));
else
    fprintf('未在仿真时间内达到收敛要求\n');
    fprintf('最终位置: [%.4f, %.4f, %.4f] m\n', x_actual(end), y_actual(end), z_actual(end));
end

%% ================== 局部函数定义 ==================

% 位置 PID 控制器及角度解算（偏航角设为 0.2 rad）
function [u_x, u_y, u_z, phi, theta, psi] = compute_pid_u(x, y, z, vx, vy, vz, ...
    desired_pos, m, g, Kp_x, Ki_x, Kd_x, Kp_y, Ki_y, Kd_y, Kp_z, Ki_z, Kd_z, max_force, ...
    integral_x, integral_y, integral_z)
    % PID 位置控制律（带积分项）
    e_x = desired_pos(1) - x;
    e_y = desired_pos(2) - y;
    e_z = desired_pos(3) - z;

    u_x = Kp_x * e_x + Ki_x * integral_x - Kd_x * vx;
    u_y = Kp_y * e_y + Ki_y * integral_y - Kd_y * vy;
    u_z = Kp_z * e_z + Ki_z * integral_z - Kd_z * vz + m * g;   % 重力补偿

    % 限幅
    u_x = max(min(u_x, max_force), -max_force);
    u_y = max(min(u_y, max_force), -max_force);
    u_z = max(min(u_z, max_force), -max_force);

    % 根据控制力解算期望姿态角（偏航角设为 0.2 rad，非零）
    psi = 0.2;
    f = m * (g - u_z/m);          % 总推力
    f = max(f, 1.0);               % 避免除零
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

    % 角度限幅（±30度）
    max_angle = 30 * pi/180;
    phi = max(min(phi, max_angle), -max_angle);
    theta = max(min(theta, max_angle), -max_angle);
end

% 内环姿态 PID 伺服控制函数
function [angles_actual, control_torques, performance_summary] = PID_smc(t, angles_desired, ~)
    % 输入:
    %   t                - 时间向量
    %   angles_desired   - 结构体，包含 roll, pitch, yaw (rad)
    %   ~                - 保留接口，未使用
    % 输出:
    %   angles_actual    - 结构体，包含实际角度 roll, pitch, yaw (rad)
    %   control_torques  - 结构体，包含控制力矩 roll, pitch, yaw (N·m)
    %   performance_summary - 性能指标结构体

    dt = t(2) - t(1);
    N = length(t);

    % 提取期望角度
    roll_desired = angles_desired.roll;
    pitch_desired = angles_desired.pitch;
    yaw_desired = angles_desired.yaw;

    % PID 参数（可根据需要调整）
    Kp_roll = 5000.0;   Ki_roll = 10.0;   Kd_roll = 150.0;
    Kp_pitch = 5000.0;  Ki_pitch = 1.0;   Kd_pitch = 150.0;
    Kp_yaw = 8.0;       Ki_yaw = 2.0;     Kd_yaw = 1.5;
    max_torque = 0.5;   % 最大力矩 (N·m)

    % 无人机转动惯量和阻尼
    Ixx = 0.0081;   Iyy = 0.0081;   Izz = 0.0142;
    B_roll = 0.05;  B_pitch = 0.05; B_yaw = 0.03;

    % 状态初始化
    roll_actual = zeros(1, N);
    pitch_actual = zeros(1, N);
    yaw_actual = zeros(1, N);
    roll_rate_actual = zeros(1, N);
    pitch_rate_actual = zeros(1, N);
    yaw_rate_actual = zeros(1, N);
    roll_accel_actual = zeros(1, N);
    pitch_accel_actual = zeros(1, N);
    yaw_accel_actual = zeros(1, N);

    % PID 误差变量
    roll_error = zeros(1, N);
    pitch_error = zeros(1, N);
    yaw_error = zeros(1, N);
    roll_error_integral = 0;
    pitch_error_integral = 0;
    yaw_error_integral = 0;
    roll_error_derivative = zeros(1, N);
    pitch_error_derivative = zeros(1, N);
    yaw_error_derivative = zeros(1, N);
    roll_torque = zeros(1, N);
    pitch_torque = zeros(1, N);
    yaw_torque = zeros(1, N);

    % 主循环
    for k = 1:N-1
        % 滚转角控制
        roll_error(k) = roll_desired(k) - roll_actual(k);
        roll_error_integral = roll_error_integral + roll_error(k) * dt;
        roll_error_integral = max(min(roll_error_integral, 2/Ki_roll), -2/Ki_roll);
        if k > 1
            roll_error_derivative(k) = (roll_error(k) - roll_error(k-1)) / dt;
        else
            roll_error_derivative(k) = 0;
        end
        alpha = 0.1;
        if k > 1
            roll_error_derivative(k) = alpha * roll_error_derivative(k) + (1-alpha) * roll_error_derivative(k-1);
        end
        roll_torque(k) = Kp_roll * roll_error(k) + Ki_roll * roll_error_integral + Kd_roll * roll_error_derivative(k);
        roll_torque(k) = max(min(roll_torque(k), max_torque), -max_torque);

        % 俯仰角控制
        pitch_error(k) = pitch_desired(k) - pitch_actual(k);
        pitch_error_integral = pitch_error_integral + pitch_error(k) * dt;
        pitch_error_integral = max(min(pitch_error_integral, 2/Ki_pitch), -2/Ki_pitch);
        if k > 1
            pitch_error_derivative(k) = (pitch_error(k) - pitch_error(k-1)) / dt;
        else
            pitch_error_derivative(k) = 0;
        end
        if k > 1
            pitch_error_derivative(k) = alpha * pitch_error_derivative(k) + (1-alpha) * pitch_error_derivative(k-1);
        end
        pitch_torque(k) = Kp_pitch * pitch_error(k) + Ki_pitch * pitch_error_integral + Kd_pitch * pitch_error_derivative(k);
        pitch_torque(k) = max(min(pitch_torque(k), max_torque), -max_torque);

        % 偏航角控制
        yaw_error(k) = yaw_desired(k) - yaw_actual(k);
        yaw_error_integral = yaw_error_integral + yaw_error(k) * dt;
        yaw_error_integral = max(min(yaw_error_integral, 2/Ki_yaw), -2/Ki_yaw);
        if k > 1
            yaw_error_derivative(k) = (yaw_error(k) - yaw_error(k-1)) / dt;
        else
            yaw_error_derivative(k) = 0;
        end
        if k > 1
            yaw_error_derivative(k) = alpha * yaw_error_derivative(k) + (1-alpha) * yaw_error_derivative(k-1);
        end
        yaw_torque(k) = Kp_yaw * yaw_error(k) + Ki_yaw * yaw_error_integral + Kd_yaw * yaw_error_derivative(k);
        yaw_torque(k) = max(min(yaw_torque(k), max_torque), -max_torque);

        % 动力学更新
        roll_accel_actual(k) = (roll_torque(k) - B_roll * roll_rate_actual(k)) / Ixx;
        roll_rate_actual(k+1) = roll_rate_actual(k) + roll_accel_actual(k) * dt;
        roll_actual(k+1) = roll_actual(k) + roll_rate_actual(k) * dt;

        pitch_accel_actual(k) = (pitch_torque(k) - B_pitch * pitch_rate_actual(k)) / Iyy;
        pitch_rate_actual(k+1) = pitch_rate_actual(k) + pitch_accel_actual(k) * dt;
        pitch_actual(k+1) = pitch_actual(k) + pitch_rate_actual(k) * dt;

        yaw_accel_actual(k) = (yaw_torque(k) - B_yaw * yaw_rate_actual(k)) / Izz;
        yaw_rate_actual(k+1) = yaw_rate_actual(k) + yaw_accel_actual(k) * dt;
        yaw_actual(k+1) = yaw_actual(k) + yaw_rate_actual(k) * dt;
    end

    % 最后一时刻赋值
    roll_error(end) = roll_desired(end) - roll_actual(end);
    pitch_error(end) = pitch_desired(end) - pitch_actual(end);
    yaw_error(end) = yaw_desired(end) - yaw_actual(end);
    roll_accel_actual(end) = roll_accel_actual(end-1);
    pitch_accel_actual(end) = pitch_accel_actual(end-1);
    yaw_accel_actual(end) = yaw_accel_actual(end-1);
    roll_error_derivative(end) = roll_error_derivative(end-1);
    pitch_error_derivative(end) = pitch_error_derivative(end-1);
    yaw_error_derivative(end) = yaw_error_derivative(end-1);

    % 打包输出
    angles_actual.roll = roll_actual;
    angles_actual.pitch = pitch_actual;
    angles_actual.yaw = yaw_actual;
    control_torques.roll = roll_torque;
    control_torques.pitch = pitch_torque;
    control_torques.yaw = yaw_torque;

    % 简单性能指标（可选）
    performance_summary.roll.max_error_deg = max(abs(roll_error)) * 180/pi;
    performance_summary.pitch.max_error_deg = max(abs(pitch_error)) * 180/pi;
    performance_summary.yaw.max_error_deg = max(abs(yaw_error)) * 180/pi;
end