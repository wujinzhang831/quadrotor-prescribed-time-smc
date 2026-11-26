clear; clc; close all;

%% ================== 预设参数配置 ==================
desired_pos = [0, 0, 0];   % 目标位置 [x,y,z] (m)，与xyz_2.m一致
T_preset = 9;              % 预设收敛时间 (s)，与xyz_2.m一致
convergence_threshold = 0.05;  % 放宽阈值，适配实际收敛特性
dt = 0.001;                % 仿真步长（与外环一致）

%% ================== 调用外环滑模控制函数 ==================
[t, angles_smc, control_forces] = quadrotor_smc(desired_pos, T_preset);
N = length(t);     % 仿真总步数

%% ================== 调用内环PID伺服控制函数 ==================
[angles_actual, control_torques, performance_summary] = PID_smc(t, angles_smc, control_forces);

%% ================== 解算无人机实际位置轨迹（核心修正）==================
% 系统参数（必须与quadrotor_smc和xyz_2.m完全一致）
m = 0.905;   % 质量 (kg)
g = 9.8;     % 重力加速度 (m/s^2)
kx = 0.1;    % X方向阻力系数
ky = 0.1;    % Y方向阻力系数
kz = 0.1;    % Z方向阻力系数

% 初始位置（必须与外环一致）
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
% 新增：加速度存储数组
ax_actual = zeros(1, N);
ay_actual = zeros(1, N);
az_actual = zeros(1, N);

% 设置初始条件
x_actual(1) = x0;
y_actual(1) = y0;
z_actual(1) = z0;

% 外环输出的控制力（直接复用）
u_x = control_forces.u_x;
u_y = control_forces.u_y;
u_z = control_forces.u_z;

% 外部扰动（必须与quadrotor_smc一致）
d_max = 0.5;
freq_x = 0.5; freq_y = 0.6; freq_z = 0.7;
d_x_func = @(tt) d_max * sin(2*pi*freq_x*tt);
d_y_func = @(tt) d_max * cos(2*pi*freq_y*tt);
d_z_func = @(tt) d_max * sin(2*pi*freq_z*tt + pi/4);

% ================== 动力学模型 ==================
for k = 1:N-1
    % 当前时间
    t_k = t(k);
    
    % 获取当前控制力
    u_x_k = u_x(k);
    u_y_k = u_y(k);
    u_z_k = u_z(k);
    
    % 获取当前状态
    x_k = x_actual(k);
    y_k = y_actual(k);
    z_k = z_actual(k);
    vx_k = vx_actual(k);
    vy_k = vy_actual(k);
    vz_k = vz_actual(k);
    
    % 计算外部扰动（除以质量得到加速度）
    d_x_k = d_x_func(t_k) / m;
    d_y_k = d_y_func(t_k) / m;
    d_z_k = d_z_func(t_k) / m;
    
    % ===== X方向动力学 =====
    ax = u_x_k/m - (kx/m)*vx_k + d_x_k;
    
    % ===== Y方向动力学 =====
    ay = u_y_k/m - (ky/m)*vy_k + d_y_k;
    
    % ===== Z方向动力学 =====
    az = u_z_k/m - g - (kz/m)*vz_k + d_z_k;
    
    % ===== 状态更新（改进欧拉法） =====
    % 预测步
    vx_pred = vx_k + ax * dt;
    vy_pred = vy_k + ay * dt;
    vz_pred = vz_k + az * dt;
    
    x_pred = x_k + vx_k * dt;
    y_pred = y_k + vy_k * dt;
    z_pred = z_k + vz_k * dt;
    
    % 校正步（使用下一时刻的控制力和扰动）
    t_next = t(k+1);
    u_x_next = u_x(k+1);
    u_y_next = u_y(k+1);
    u_z_next = u_z(k+1);
    
    d_x_next = d_x_func(t_next) / m;
    d_y_next = d_y_func(t_next) / m;
    d_z_next = d_z_func(t_next) / m;
    
    ax_corr = u_x_next/m - (kx/m)*vx_pred + d_x_next;
    ay_corr = u_y_next/m - (ky/m)*vy_pred + d_y_next;
    az_corr = u_z_next/m - g - (kz/m)*vz_pred + d_z_next;
    
    % 平均预测和校正的加速度
    ax_avg = (ax + ax_corr) / 2;
    ay_avg = (ay + ay_corr) / 2;
    az_avg = (az + az_corr) / 2;
    
    % 存储平均加速度
    ax_actual(k) = ax_avg;
    ay_actual(k) = ay_avg;
    az_actual(k) = az_avg;
    
    % 更新速度和位置
    vx_actual(k+1) = vx_k + ax_avg * dt;
    vy_actual(k+1) = vy_k + ay_avg * dt;
    vz_actual(k+1) = vz_k + az_avg * dt;
    
    x_actual(k+1) = x_k + (vx_k + vx_pred) / 2 * dt;
    y_actual(k+1) = y_k + (vy_k + vy_pred) / 2 * dt;
    z_actual(k+1) = z_k + (vz_k + vz_pred) / 2 * dt;
    
    % 数值稳定性检查（防止积分发散）
    vx_actual(k+1) = max(min(vx_actual(k+1), 10), -10);
    vy_actual(k+1) = max(min(vy_actual(k+1), 10), -10);
    vz_actual(k+1) = max(min(vz_actual(k+1), 10), -10);
end

% 最后一个时刻的加速度（简单填充）
ax_actual(N) = ax_actual(N-1);
ay_actual(N) = ay_actual(N-1);
az_actual(N) = az_actual(N-1);

%% ================== 计算位置误差 ==================
x_error = x_actual - desired_pos(1);
y_error = y_actual - desired_pos(2);
z_error = z_actual - desired_pos(3);

%% ================== 重新计算实际收敛时间 ==================
% 使用更稳健的收敛判定：连续100个点都在阈值内
converged_flag = false;
T_actual = NaN;

% 查找收敛点
for k = 1:N-100
    if all(abs(x_error(k:k+99)) < convergence_threshold) && ...
       all(abs(y_error(k:k+99)) < convergence_threshold) && ...
       all(abs(z_error(k:k+99)) < convergence_threshold)
        T_actual = t(k);
        converged_flag = true;
        break;
    end
end

% 如果没有找到连续100点的收敛，放宽条件到连续50点
if ~converged_flag
    for k = 1:N-50
        if all(abs(x_error(k:k+49)) < convergence_threshold) && ...
           all(abs(y_error(k:k+49)) < convergence_threshold) && ...
           all(abs(z_error(k:k+49)) < convergence_threshold)
            T_actual = t(k);
            converged_flag = true;
            break;
        end
    end
end

%% ================== 可视化（重构为与SMC_fenduan_gai.m一致）==================

% 新 Figure 1：位置、速度、加速度（3x3 子图）
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

% 新 Figure 2：3D轨迹（单独）
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

% Figure 3：姿态角跟踪性能（仅保留三个跟踪子图，删除误差图）
figure('Position', [100, 100, 1000, 800], 'Name', '姿态角跟踪性能');

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

%% ================== 收敛结果输出（保留原样）==================
fprintf('\n========== 预定时间滑模控制仿真结果 ==========\n');
fprintf('预设收敛时间 T = %.2f s\n', T_preset);
fprintf('期望目标位置: [%.1f, %.1f, %.1f] m\n', desired_pos(1), desired_pos(2), desired_pos(3));
fprintf('初始位置: [%.1f, %.1f, %.1f] m\n', x0, y0, z0);
fprintf('收敛阈值: %.3f m\n', convergence_threshold);
if ~isnan(T_actual)
    fprintf('实际到达时刻: %.3f s\n', T_actual);
    fprintf('提前/延迟: %.3f s\n', T_preset - T_actual);
    fprintf('最终位置误差: [%.4f, %.4f, %.4f] m\n', x_error(end), y_error(end), z_error(end));
else
    fprintf('未在仿真时间内达到收敛要求\n');
    fprintf('最终位置: [%.4f, %.4f, %.4f] m\n', x_actual(end), y_actual(end), z_actual(end));
end