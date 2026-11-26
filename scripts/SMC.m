clear; clc; close all;

%% ================== 预设参数配置 ==================
desired_pos = [0, 0, 0];   % 目标位置 [x,y,z] (m)
T_preset = 9;              % 预设收敛时间 (s)
convergence_threshold = 0.05;  % 阈值
dt = 0.001;                % 仿真步长

%% ================== 调用外环滑模控制函数 ==================
[t, angles_smc, control_forces] = quadrotor_smc(desired_pos, T_preset);
N = length(t);     % 仿真总步数

%% ================== 调用内环PID伺服控制函数 ==================
[angles_actual, control_torques, performance_summary] = PID_smc(t, angles_smc, control_forces);

%% ================== 无人机实际位置轨迹==================
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

% 设置初始条件
x_actual(1) = x0;
y_actual(1) = y0;
z_actual(1) = z0;

% 外环输出的控制力（直接复用）
u_x = control_forces.u_x;
u_y = control_forces.u_y;
u_z = control_forces.u_z;

% 外部扰动
d_max = 0.5;
freq_x = 0.5; freq_y = 0.6; freq_z = 0.7;
d_x_func = @(tt) d_max * sin(2*pi*freq_x*tt);
d_y_func = @(tt) d_max * cos(2*pi*freq_y*tt);
d_z_func = @(tt) d_max * sin(2*pi*freq_z*tt + pi/4);

% ================== 动力学模型 ==================
% 关键修正：使用正确的符号方向
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
    
    % ===== X方向动力学（与xyz_2.m中的f_x + g_x * u_x + delta_x一致） =====
    % 根据get_system_params('x', ...)函数：f_x = -(kx/m) * vx, g_x = 1/m
    % 因此：加速度 = f_x + g_x * u_x + delta_x
    %      = -(kx/m)*vx + (1/m)*u_x + d_x_func/m
    % 简化为：加速度 = u_x/m - (kx/m)*vx + d_x_k
    ax = u_x_k/m - (kx/m)*vx_k + d_x_k;
    
    % ===== Y方向动力学（与xyz_2.m一致） =====
    ay = u_y_k/m - (ky/m)*vy_k + d_y_k;
    
    % ===== Z方向动力学（与xyz_2.m一致） =====
    % 注意：Z方向有重力项，根据get_system_params('z', ...): f_z = -g - (kz/m)*vz
    az = u_z_k/m - g - (kz/m)*vz_k + d_z_k;
    
    % ===== 状态更新（使用改进欧拉法提高精度） =====
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

%% ================== 计算位置误差 ==================
x_error = x_actual - desired_pos(1);
y_error = y_actual - desired_pos(2);
z_error = z_actual - desired_pos(3);

%% ================== 重新计算实际收敛时间 ==================
% 使用新定义：从t时刻开始，之后所有时刻位置误差恒小于0.001m
T_actual = NaN;

% 查找满足条件的时刻
for k = 1:N
    % 检查从当前时刻k到结束的所有点是否都满足条件
    if all(abs(x_error(k:end)) < convergence_threshold) && ...
       all(abs(y_error(k:end)) < convergence_threshold) && ...
       all(abs(z_error(k:end)) < convergence_threshold)
        T_actual = t(k);
        break;
    end
end

%% ================== 结果可视化（修正后） ==================
figure('Position', [100, 100, 1200, 800], 'Name', '预定时间滑模控制仿真结果');

% ========== 子图1：位置随时间变化（XY/Z） ==========
subplot(2,3,1);
plot(t, x_actual, 'b-', 'LineWidth', 1.2, 'DisplayName', '实际X');
hold on;
plot(t, ones(1,N)*desired_pos(1), 'r--', 'LineWidth', 1, 'DisplayName', '目标X');
if ~isnan(T_actual)
    plot([T_actual, T_actual], [min(x_actual), max(x_actual)], 'g:', ...
        'LineWidth', 1.5, 'DisplayName', sprintf('实际到达: %.3f s', T_actual));
end
xlabel('时间 (s)'); ylabel('X位置 (m)');
title('X位置跟踪'); grid on; legend('Location', 'best');
xlim([0, T_preset*1.5]); % 显示更长时间以观察收敛

subplot(2,3,2);
plot(t, y_actual, 'b-', 'LineWidth', 1.2, 'DisplayName', '实际Y');
hold on;
plot(t, ones(1,N)*desired_pos(2), 'r--', 'LineWidth', 1, 'DisplayName', '目标Y');
if ~isnan(T_actual)
    plot([T_actual, T_actual], [min(y_actual), max(y_actual)], 'g:', ...
        'LineWidth', 1.5, 'DisplayName', sprintf('实际到达: %.3f s', T_actual));
end
xlabel('时间 (s)'); ylabel('Y位置 (m)');
title('Y位置跟踪'); grid on; legend('Location', 'best');
xlim([0, T_preset*1.5]);

subplot(2,3,3);
plot(t, z_actual, 'b-', 'LineWidth', 1.2, 'DisplayName', '实际Z');
hold on;
plot(t, ones(1,N)*desired_pos(3), 'r--', 'LineWidth', 1, 'DisplayName', '目标Z');
if ~isnan(T_actual)
    plot([T_actual, T_actual], [min(z_actual), max(z_actual)], 'g:', ...
        'LineWidth', 1.5, 'DisplayName', sprintf('实际到达: %.3f s', T_actual));
end
xlabel('时间 (s)'); ylabel('Z位置 (m)');
title('Z位置跟踪'); grid on; legend('Location', 'best');
xlim([0, T_preset*1.5]);

% ========== 子图2：3D空间轨迹 ==========
subplot(2,3,4);
plot3(x_actual, y_actual, z_actual, 'b-', 'LineWidth', 1.5);
hold on;
plot3(desired_pos(1), desired_pos(2), desired_pos(3), 'ro', ...
     'MarkerSize', 8, 'LineWidth', 2, 'DisplayName', '目标点');
plot3(x0, y0, z0, 'go', 'MarkerSize', 8, 'LineWidth', 2, 'DisplayName', '起始点');
if ~isnan(T_actual)
    % 找到最接近T_actual的时间索引
    [~, idx] = min(abs(t - T_actual));
    plot3(x_actual(idx), y_actual(idx), z_actual(idx), ...
         'ms', 'MarkerSize', 10, 'LineWidth', 2, ...
         'DisplayName', sprintf('到达点 (t=%.3f s)', T_actual));
end
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
title(['3D轨迹 (预设时间: ', num2str(T_preset), 's)']);
grid on; view(45, 30); legend('Location', 'best');
xlim([min(x_actual)-1, max(x_actual)+1]);
ylim([min(y_actual)-1, max(y_actual)+1]);
zlim([min(z_actual)-1, max(z_actual)+1]);

% ========== 子图3：位置误差曲线 ==========
subplot(2,3,5);
semilogy(t, abs(x_error), 'b-', 'LineWidth', 1, 'DisplayName', '|X误差|');
hold on;
semilogy(t, abs(y_error), 'r-', 'LineWidth', 1, 'DisplayName', '|Y误差|');
semilogy(t, abs(z_error), 'g-', 'LineWidth', 1, 'DisplayName', '|Z误差|');
plot(t, ones(1,N)*convergence_threshold);
if ~isnan(T_actual)
    plot([T_actual, T_actual], [1e-6, 10], 'm:', 'LineWidth', 1.5, ...
        'DisplayName', sprintf('实际到达: %.3f s', T_actual));
end
xlabel('时间 (s)'); ylabel('位置误差绝对值 (m)');
title('位置误差收敛情况（对数坐标）'); 
grid on; legend('Location', 'best');
xlim([0, T_preset*1.5]);
ylim([1e-6, max([max(abs(x_error)), max(abs(y_error)), max(abs(z_error)), 1])]);

% ========== 子图4：控制力变化 ==========
subplot(2,3,6);
plot(t, u_x, 'b-', 'LineWidth', 1, 'DisplayName', 'u_x');
hold on;
plot(t, u_y, 'r-', 'LineWidth', 1, 'DisplayName', 'u_y');
plot(t, u_z, 'g-', 'LineWidth', 1, 'DisplayName', 'u_z');
if ~isnan(T_actual)
    plot([T_actual, T_actual], [min([u_x, u_y, u_z]), max([u_x, u_y, u_z])], ...
        'm:', 'LineWidth', 1.5, 'DisplayName', sprintf('实际到达: %.3f s', T_actual));
end
xlabel('时间 (s)'); ylabel('控制力 (N)');
title('外环滑模控制力'); grid on; legend('Location', 'best');
xlim([0, T_preset*1.5]);

%% ================== 在图形窗口添加文本信息 ==================
% 创建文本注释
annotation_text = sprintf('预设收敛时间 T = %.2f s\n', T_preset);
annotation_text = [annotation_text, sprintf('目标位置: [%.1f, %.1f, %.1f] m\n', ...
    desired_pos(1), desired_pos(2), desired_pos(3))];
annotation_text = [annotation_text, sprintf('初始位置: [%.1f, %.1f, %.1f] m\n', x0, y0, z0)];
annotation_text = [annotation_text, sprintf('收敛阈值: %.4f m\n', convergence_threshold)];

if ~isnan(T_actual)
    annotation_text = [annotation_text, sprintf('\n实际到达时刻: %.3f s\n', T_actual)];
    annotation_text = [annotation_text, sprintf('提前/延迟: %.3f s\n', T_preset - T_actual)];
    
    % 计算最终位置误差
    final_x_error = x_actual(end);
    final_y_error = y_actual(end);
    final_z_error = z_actual(end);
    final_position = [final_x_error, final_y_error, final_z_error];
    annotation_text = [annotation_text, sprintf('最终位置误差: [%.4f, %.4f, %.4f] m\n', ...
        final_x_error, final_y_error, final_z_error)];
else
    annotation_text = [annotation_text, sprintf('\n未在仿真时间内达到收敛要求\n')];
    
    % 显示最终误差
    final_x_error = x_actual(end);
    final_y_error = y_actual(end);
    final_z_error = z_actual(end);
    annotation_text = [annotation_text, sprintf('最终位置: [%.4f, %.4f, %.4f] m\n', ...
        final_x_error, final_y_error, final_z_error)];
end

% 添加文本到图形窗口
ax = axes('Position', [0.02, 0.02, 0.25, 0.15], 'Visible', 'off');
text(0, 0.5, annotation_text, 'FontSize', 10, 'VerticalAlignment', 'middle', ...
    'BackgroundColor', [0.95, 0.95, 0.95], 'EdgeColor', 'k', 'LineWidth', 1);

% 添加总标题
sgtitle(sprintf('四旋翼无人机预定时间滑模控制 (T=%.1fs)', T_preset), ...
    'FontSize', 14, 'FontWeight', 'bold');

%% ================== 输出结果到命令行 ==================
fprintf('\n========== 预定时间滑模控制仿真结果 ==========\n');
fprintf('预设收敛时间 T = %.2f s\n', T_preset);
fprintf('期望目标位置: [%.1f, %.1f, %.1f] m\n', desired_pos(1), desired_pos(2), desired_pos(3));
fprintf('初始位置: [%.1f, %.1f, %.1f] m\n', x0, y0, z0);
fprintf('收敛阈值: %.4f m (位置误差恒小于此值)\n', convergence_threshold);

%% ================== 姿态角跟踪性能可视化 ==================
figure('Position', [100, 100, 1000, 800], 'Name', '姿态角跟踪性能');

% 期望角度
roll_desired = angles_smc.roll;
pitch_desired = angles_smc.pitch;
yaw_desired = angles_smc.yaw;

% 实际角度
roll_actual = angles_actual.roll;
pitch_actual = angles_actual.pitch;
yaw_actual = angles_actual.yaw;

% 滚转角跟踪
subplot(3,2,1);
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

subplot(3,2,2);
plot(t, (roll_desired - roll_actual)*180/pi, 'k-', 'LineWidth', 1);
if ~isnan(T_actual)
    plot([T_actual, T_actual], [min((roll_desired - roll_actual)*180/pi), ...
        max((roll_desired - roll_actual)*180/pi)], 'g:', 'LineWidth', 1.5);
end
xlabel('时间 (s)'); ylabel('滚转角误差 (°)');
title('滚转角跟踪误差'); grid on;
xlim([0, T_preset*1.2]);

% 俯仰角跟踪
subplot(3,2,3);
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

subplot(3,2,4);
plot(t, (pitch_desired - pitch_actual)*180/pi, 'k-', 'LineWidth', 1);
if ~isnan(T_actual)
    plot([T_actual, T_actual], [min((pitch_desired - pitch_actual)*180/pi), ...
        max((pitch_desired - pitch_actual)*180/pi)], 'g:', 'LineWidth', 1.5);
end
xlabel('时间 (s)'); ylabel('俯仰角误差 (°)');
title('俯仰角跟踪误差'); grid on;
xlim([0, T_preset*1.2]);

% 偏航角跟踪
subplot(3,2,5);
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

subplot(3,2,6);
plot(t, (yaw_desired - yaw_actual)*180/pi, 'k-', 'LineWidth', 1);
if ~isnan(T_actual)
    plot([T_actual, T_actual], [min((yaw_desired - yaw_actual)*180/pi), ...
        max((yaw_desired - yaw_actual)*180/pi)], 'g:', 'LineWidth', 1.5);
end
xlabel('时间 (s)'); ylabel('偏航角误差 (°)');
title('偏航角跟踪误差'); grid on;
xlim([0, T_preset*1.2]);

% 添加总体标题
sgtitle(sprintf('内环PID伺服控制姿态角跟踪性能 (位置到达时刻: %.3f s)', T_actual), ...
    'FontSize', 14, 'FontWeight', 'bold');