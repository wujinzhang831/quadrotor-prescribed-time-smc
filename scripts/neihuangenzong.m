%% 四旋翼姿态PID控制器：阶跃与正弦跟踪性能测试
% 本脚本提取了姿态PID控制器，分别测试对阶跃参考信号和正弦参考信号的跟踪能力
% 动力学模型：I * theta_ddot = tau - B * theta_dot
% PID控制器包含：比例、积分（带抗饱和）、微分（带一阶低通滤波）、输出限幅

clear; clc; close all;

%% ================== 系统参数设置 ==================
% 转动惯量 (kg·m²)  -- 以滚转轴为例
I = 0.0081;      % Ixx
% 阻尼系数 (N·m·s/rad)
B = 0.05;

%% ================== PID控制器参数 ==================
% 参考原文件中的滚转PID参数
Kp = 5000.0;     % 比例增益
Ki = 10.0;       % 积分增益
Kd = 150.0;      % 微分增益

% 控制输出限幅 (N·m)
max_torque = 0.5;

% 微分项低通滤波器系数 (0~1, 越小滤波越强)
alpha = 0.1;

% 积分抗饱和限幅 (可选)
integral_limit = 2 / Ki;   % 积分项限幅值

%% ================== 仿真时间设置 ==================
dt = 0.001;                % 时间步长 (s)
t_end = 1;                 % 仿真时长 (s)
t = 0:dt:t_end;
N = length(t);

%% ================== 定义参考输入信号 ==================
% 阶跃信号：幅度 30° (转换为弧度)
step_amplitude_deg = 30;
step_amplitude_rad = step_amplitude_deg * pi/180;
ref_step = step_amplitude_rad * ones(size(t));

% 正弦信号：幅度 20°，频率 0.5 Hz
sin_amplitude_deg = 20;
sin_amplitude_rad = sin_amplitude_deg * pi/180;
sin_freq = 0.5;            % Hz
ref_sin = sin_amplitude_rad * sin(2*pi*sin_freq * t);

%% ================== 阶跃响应仿真 ==================
fprintf('正在进行阶跃响应仿真...\n');
[theta_step, theta_dot_step, torque_step, error_step, t] = pid_simulation(...
    ref_step, dt, I, B, Kp, Ki, Kd, max_torque, alpha, integral_limit);

% 计算阶跃响应性能指标
final_value = step_amplitude_rad;
% 稳态误差 (取最后5%时间的平均)
steady_idx = round(0.95 * N);
steady_error_deg = mean(abs(error_step(steady_idx:end))) * 180/pi;
% 最大超调量
max_theta_rad = max(theta_step);
overshoot_percent = (max_theta_rad - final_value) / final_value * 100;
if overshoot_percent < 0
    overshoot_percent = 0;
end
% 上升时间 (10% -> 90%)
target_10 = 0.1 * final_value;
target_90 = 0.9 * final_value;
idx_10 = find(theta_step >= target_10, 1, 'first');
idx_90 = find(theta_step >= target_90, 1, 'first');
if ~isempty(idx_10) && ~isempty(idx_90)
    rise_time = t(idx_90) - t(idx_10);
else
    rise_time = NaN;
end
% 稳定时间 (进入 ±2% 误差带)
settling_tolerance = 0.02 * final_value;
settle_idx = find(abs(error_step) <= settling_tolerance, 1, 'first');
if ~isempty(settle_idx)
    settling_time = t(settle_idx);
else
    settling_time = NaN;
end

fprintf('阶跃响应性能指标:\n');
fprintf('  稳态误差: %.3f deg\n', steady_error_deg);
fprintf('  超调量: %.2f %%\n', overshoot_percent);
fprintf('  上升时间: %.3f s\n', rise_time);
fprintf('  稳定时间(±2%%): %.3f s\n', settling_time);

%% ================== 正弦跟踪仿真 ==================
fprintf('\n正在进行正弦跟踪仿真...\n');
[theta_sin, theta_dot_sin, torque_sin, error_sin, ~] = pid_simulation(...
    ref_sin, dt, I, B, Kp, Ki, Kd, max_torque, alpha, integral_limit);

% 计算正弦跟踪的均方根误差 (RMSE)
rmse_deg = sqrt(mean(error_sin.^2)) * 180/pi;
% 最大绝对误差
max_error_deg = max(abs(error_sin)) * 180/pi;
fprintf('正弦跟踪性能指标 (幅值%.0f°, 频率%.1fHz):\n', sin_amplitude_deg, sin_freq);
fprintf('  RMSE: %.3f deg\n', rmse_deg);
fprintf('  最大绝对误差: %.3f deg\n', max_error_deg);

%% ================== 绘图 ==================
% 设置图形样式
linewidth = 1.5;
fontsize = 11;

% 图1：阶跃响应
figure('Name', '阶跃响应', 'Position', [100, 100, 1200, 800]);

subplot(2,2,1);
plot(t, ref_step*180/pi, 'r--', 'LineWidth', linewidth); hold on;
plot(t, theta_step*180/pi, 'b-', 'LineWidth', linewidth);
xlabel('时间 (s)', 'FontSize', fontsize);
ylabel('角度 (deg)', 'FontSize', fontsize);
title('阶跃响应：角度跟踪', 'FontSize', fontsize+1);
legend('参考(30°)', '实际', 'Location', 'best');
grid on;
xlim([0, t_end]);

subplot(2,2,2);
plot(t, error_step*180/pi, 'g-', 'LineWidth', linewidth);
xlabel('时间 (s)', 'FontSize', fontsize);
ylabel('误差 (deg)', 'FontSize', fontsize);
title('阶跃响应：跟踪误差', 'FontSize', fontsize+1);
grid on;
xlim([0, t_end]);

subplot(2,2,3);
plot(t, theta_dot_step*180/pi, 'm-', 'LineWidth', linewidth);
xlabel('时间 (s)', 'FontSize', fontsize);
ylabel('角速度 (deg/s)', 'FontSize', fontsize);
title('阶跃响应：角速度', 'FontSize', fontsize+1);
grid on;
xlim([0, t_end]);

subplot(2,2,4);
plot(t, torque_step, 'c-', 'LineWidth', linewidth);
xlabel('时间 (s)', 'FontSize', fontsize);
ylabel('力矩 (N·m)', 'FontSize', fontsize);
title('阶跃响应：控制力矩', 'FontSize', fontsize+1);
grid on;
xlim([0, t_end]);

sgtitle('四旋翼姿态PID控制器 —— 阶跃响应', 'FontSize', 14, 'FontWeight', 'bold');

% 图2：正弦跟踪
figure('Name', '正弦跟踪', 'Position', [100, 100, 1200, 800]);

subplot(2,2,1);
plot(t, ref_sin*180/pi, 'r--', 'LineWidth', linewidth); hold on;
plot(t, theta_sin*180/pi, 'b-', 'LineWidth', linewidth);
xlabel('时间 (s)', 'FontSize', fontsize);
ylabel('角度 (deg)', 'FontSize', fontsize);
title(sprintf('正弦跟踪：%.0f°, %.1fHz', sin_amplitude_deg, sin_freq), 'FontSize', fontsize+1);
legend('参考', '实际', 'Location', 'best');
grid on;
xlim([0, t_end]);

subplot(2,2,2);
plot(t, error_sin*180/pi, 'g-', 'LineWidth', linewidth);
xlabel('时间 (s)', 'FontSize', fontsize);
ylabel('误差 (deg)', 'FontSize', fontsize);
title(sprintf('跟踪误差 (RMSE=%.3f°)', rmse_deg), 'FontSize', fontsize+1);
grid on;
xlim([0, t_end]);

subplot(2,2,3);
plot(t, theta_dot_sin*180/pi, 'm-', 'LineWidth', linewidth);
xlabel('时间 (s)', 'FontSize', fontsize);
ylabel('角速度 (deg/s)', 'FontSize', fontsize);
title('角速度响应', 'FontSize', fontsize+1);
grid on;
xlim([0, t_end]);

subplot(2,2,4);
plot(t, torque_sin, 'c-', 'LineWidth', linewidth);
xlabel('时间 (s)', 'FontSize', fontsize);
ylabel('力矩 (N·m)', 'FontSize', fontsize);
title('控制力矩', 'FontSize', fontsize+1);
grid on;
xlim([0, t_end]);

sgtitle('四旋翼姿态PID控制器 —— 正弦跟踪', 'FontSize', 14, 'FontWeight', 'bold');

% 图3：误差对比
figure('Name', '误差对比', 'Position', [100, 100, 800, 400]);
plot(t, error_step*180/pi, 'b-', 'LineWidth', linewidth); hold on;
plot(t, error_sin*180/pi, 'r-', 'LineWidth', linewidth);
xlabel('时间 (s)', 'FontSize', fontsize);
ylabel('跟踪误差 (deg)', 'FontSize', fontsize);
title('PID控制器误差对比', 'FontSize', fontsize+1);
legend('阶跃响应误差', '正弦跟踪误差', 'Location', 'best');
grid on;
xlim([0, t_end]);

fprintf('\n仿真完成！\n');

%% ================== 函数定义（必须放在文件末尾） ==================
function [theta, theta_dot, torque, error, t] = pid_simulation(ref, dt, I, B, Kp, Ki, Kd, max_torque, alpha, integral_limit)
    N = length(ref);
    t = 0:dt:(N-1)*dt;   % 生成时间向量
    
    % 状态初始化
    theta = zeros(1, N);          % 角度
    theta_dot = zeros(1, N);      % 角速度
    theta_ddot = zeros(1, N);     % 角加速度
    torque = zeros(1, N);         % 控制力矩
    error = zeros(1, N);          % 跟踪误差
    
    % PID变量
    error_integral = 0;
    error_derivative = 0;
    prev_error = 0;
    
    for k = 1:N-1
        % 当前参考值
        ref_k = ref(k);
        
        % 误差计算
        error(k) = ref_k - theta(k);
        
        % 积分项（带抗饱和）
        error_integral = error_integral + error(k) * dt;
        % 抗饱和限幅
        error_integral = max(min(error_integral, integral_limit), -integral_limit);
        
        % 微分项（带一阶低通滤波）
        if k > 1
            raw_derivative = (error(k) - prev_error) / dt;
            error_derivative = alpha * raw_derivative + (1-alpha) * error_derivative;
        else
            error_derivative = 0;
        end
        
        % PID控制律
        torque(k) = Kp * error(k) + Ki * error_integral + Kd * error_derivative;
        % 输出限幅
        torque(k) = max(min(torque(k), max_torque), -max_torque);
        
        % 动力学更新
        theta_ddot(k) = (torque(k) - B * theta_dot(k)) / I;
        theta_dot(k+1) = theta_dot(k) + theta_ddot(k) * dt;
        theta(k+1) = theta(k) + theta_dot(k) * dt;
        
        % 保存上一时刻误差用于微分
        prev_error = error(k);
    end
    % 最后一个点的误差
    error(N) = ref(N) - theta(N);
    torque(N) = torque(N-1);
    theta_ddot(N) = theta_ddot(N-1);
end