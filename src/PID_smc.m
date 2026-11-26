function [angles_actual, control_torques, performance_summary] = PID_smc(t, angles_smc, ~)
%% 四旋翼无人机姿态PID伺服控制系统
% 输入:
%   t: 时间向量 (从quadrotor_smc函数输出)
%   angles_smc: 结构体，包含期望角度数据
%     - roll: 期望滚转角 (rad)
%     - pitch: 期望俯仰角 (rad)
%     - yaw: 期望偏航角 (rad)
%   ~: 占位符，接受quadrotor_smc的第三个输出但不使用
%
% 输出:
%   angles_actual: 结构体，包含实际角度数据
%     - roll: 实际滚转角 (rad)
%     - pitch: 实际俯仰角 (rad)
%     - yaw: 实际偏航角 (rad)
%   control_torques: 结构体，包含控制力矩数据
%     - roll: 滚转控制力矩 (N·m)
%     - pitch: 俯仰控制力矩 (N·m)
%     - yaw: 偏航控制力矩 (N·m)
%   performance_summary: 结构体，包含性能指标

%% ================== 参数设置 ==================
% 从输入提取期望角度
roll_desired = angles_smc.roll;    % 期望滚转角 (rad)
pitch_desired = angles_smc.pitch;  % 期望俯仰角 (rad)
yaw_desired = angles_smc.yaw;      % 期望偏航角 (rad)

% 计算时间步长和长度
dt = t(2) - t(1);
N = length(t);

% 预设时间（从时间向量推断）
preset_time = max(t) / 2;

%% ================== PID控制器参数设置 ==================
% 滚转角PID参数
Kp_roll = 5000.0;     % 比例增益
Ki_roll = 1.0;      % 积分增益
Kd_roll = 150.0;     % 微分增益

% 俯仰角PID参数
Kp_pitch = 20000.0;    % 比例增益
Ki_pitch = 1.0;      % 积分增益
Kd_pitch = 500.0;    % 微分增益

% 偏航角PID参数
Kp_yaw = 8.0;       % 比例增益
Ki_yaw = 2.0;       % 积分增益
Kd_yaw = 1.5;       % 微分增益

% 控制输出限幅
max_torque = 0.5;   % 最大力矩 (N·m)

%% ================== 初始化状态变量 ==================
% 实际角度（初始值为0）
roll_actual = zeros(1, N);
pitch_actual = zeros(1, N);
yaw_actual = zeros(1, N);

% 实际角速度（初始值为0）
roll_rate_actual = zeros(1, N);
pitch_rate_actual = zeros(1, N);
yaw_rate_actual = zeros(1, N);

% 实际角加速度（初始值为0）
roll_accel_actual = zeros(1, N);
pitch_accel_actual = zeros(1, N);
yaw_accel_actual = zeros(1, N);

% 期望角速度（通过对期望角度微分得到）
roll_rate_desired = zeros(1, N);
pitch_rate_desired = zeros(1, N);
yaw_rate_desired = zeros(1, N);

% 期望角加速度（通过对期望角速度微分得到）
roll_accel_desired = zeros(1, N);
pitch_accel_desired = zeros(1, N);
yaw_accel_desired = zeros(1, N);

% PID误差项
roll_error = zeros(1, N);
pitch_error = zeros(1, N);
yaw_error = zeros(1, N);

roll_error_integral = 0;
pitch_error_integral = 0;
yaw_error_integral = 0;

roll_error_derivative = zeros(1, N);
pitch_error_derivative = zeros(1, N);
yaw_error_derivative = zeros(1, N);

% 控制力矩
roll_torque = zeros(1, N);
pitch_torque = zeros(1, N);
yaw_torque = zeros(1, N);

%% ================== 无人机动力学参数 ==================
% 转动惯量 (kg·m²)
Ixx = 0.0081;   % 绕X轴转动惯量
Iyy = 0.0081;   % 绕Y轴转动惯量
Izz = 0.0142;   % 绕Z轴转动惯量

% 阻尼系数
B_roll = 0.05;   % 滚转角阻尼
B_pitch = 0.05;  % 俯仰角阻尼
B_yaw = 0.03;    % 偏航角阻尼

%% ================== 计算期望角速度和角加速度 ==================
% 使用中心差分法计算期望角速度
for k = 2:N-1
    roll_rate_desired(k) = (roll_desired(k+1) - roll_desired(k-1)) / (2*dt);
    pitch_rate_desired(k) = (pitch_desired(k+1) - pitch_desired(k-1)) / (2*dt);
    yaw_rate_desired(k) = (yaw_desired(k+1) - yaw_desired(k-1)) / (2*dt);
end

% 处理边界点
roll_rate_desired(1) = (roll_desired(2) - roll_desired(1)) / dt;
roll_rate_desired(N) = (roll_desired(N) - roll_desired(N-1)) / dt;

pitch_rate_desired(1) = (pitch_desired(2) - pitch_desired(1)) / dt;
pitch_rate_desired(N) = (pitch_desired(N) - pitch_desired(N-1)) / dt;

yaw_rate_desired(1) = (yaw_desired(2) - yaw_desired(1)) / dt;
yaw_rate_desired(N) = (yaw_desired(N) - yaw_desired(N-1)) / dt;

% 使用中心差分法计算期望角加速度
for k = 2:N-1
    roll_accel_desired(k) = (roll_rate_desired(k+1) - roll_rate_desired(k-1)) / (2*dt);
    pitch_accel_desired(k) = (pitch_rate_desired(k+1) - pitch_rate_desired(k-1)) / (2*dt);
    yaw_accel_desired(k) = (yaw_rate_desired(k+1) - yaw_rate_desired(k-1)) / (2*dt);
end

% 处理边界点
roll_accel_desired(1) = (roll_rate_desired(2) - roll_rate_desired(1)) / dt;
roll_accel_desired(N) = (roll_rate_desired(N) - roll_rate_desired(N-1)) / dt;

pitch_accel_desired(1) = (pitch_rate_desired(2) - pitch_rate_desired(1)) / dt;
pitch_accel_desired(N) = (pitch_rate_desired(N) - pitch_rate_desired(N-1)) / dt;

yaw_accel_desired(1) = (yaw_rate_desired(2) - yaw_rate_desired(1)) / dt;
yaw_accel_desired(N) = (yaw_rate_desired(N) - yaw_rate_desired(N-1)) / dt;

%% ================== PID伺服控制主循环 ==================
for k = 1:N-1
    % ===== 滚转角控制 =====
    % 计算滚转角误差
    roll_error(k) = roll_desired(k) - roll_actual(k);
    
    % 更新积分项（带抗饱和）
    roll_error_integral = roll_error_integral + roll_error(k) * dt;
    roll_error_integral = max(min(roll_error_integral, 2/Ki_roll), -2/Ki_roll); % 积分抗饱和
    
    % 计算微分项
    if k > 1
        roll_error_derivative(k) = (roll_error(k) - roll_error(k-1)) / dt;
    else
        roll_error_derivative(k) = 0;
    end
    
    % 应用一阶低通滤波器到微分项
    alpha = 0.1;  % 滤波器系数
    if k > 1
        roll_error_derivative(k) = alpha * roll_error_derivative(k) + (1-alpha) * roll_error_derivative(k-1);
    end
    
    % PID控制律
    roll_torque(k) = Kp_roll * roll_error(k) + Ki_roll * roll_error_integral + Kd_roll * roll_error_derivative(k);
    
    % 控制输出限幅
    roll_torque(k) = max(min(roll_torque(k), max_torque), -max_torque);
    
    % ===== 俯仰角控制 =====
    % 计算俯仰角误差
    pitch_error(k) = pitch_desired(k) - pitch_actual(k);
    
    % 更新积分项
    pitch_error_integral = pitch_error_integral + pitch_error(k) * dt;
    pitch_error_integral = max(min(pitch_error_integral, 2/Ki_pitch), -2/Ki_pitch);
    
    % 计算微分项
    if k > 1
        pitch_error_derivative(k) = (pitch_error(k) - pitch_error(k-1)) / dt;
    else
        pitch_error_derivative(k) = 0;
    end
    
    % 滤波
    if k > 1
        pitch_error_derivative(k) = alpha * pitch_error_derivative(k) + (1-alpha) * pitch_error_derivative(k-1);
    end
    
    % PID控制律
    pitch_torque(k) = Kp_pitch * pitch_error(k) + Ki_pitch * pitch_error_integral + Kd_pitch * pitch_error_derivative(k);
    
    % 控制输出限幅
    pitch_torque(k) = max(min(pitch_torque(k), max_torque), -max_torque);
    
    % ===== 偏航角控制 =====
    % 计算偏航角误差
    yaw_error(k) = yaw_desired(k) - yaw_actual(k);
    
    % 更新积分项
    yaw_error_integral = yaw_error_integral + yaw_error(k) * dt;
    yaw_error_integral = max(min(yaw_error_integral, 2/Ki_yaw), -2/Ki_yaw);
    
    % 计算微分项
    if k > 1
        yaw_error_derivative(k) = (yaw_error(k) - yaw_error(k-1)) / dt;
    else
        yaw_error_derivative(k) = 0;
    end
    
    % 滤波
    if k > 1
        yaw_error_derivative(k) = alpha * yaw_error_derivative(k) + (1-alpha) * yaw_error_derivative(k-1);
    end
    
    % PID控制律
    yaw_torque(k) = Kp_yaw * yaw_error(k) + Ki_yaw * yaw_error_integral + Kd_yaw * yaw_error_derivative(k);
    
    % 控制输出限幅
    yaw_torque(k) = max(min(yaw_torque(k), max_torque), -max_torque);
    
    % ===== 动力学方程更新 =====
    % 滚转角动力学：Ixx * α_roll = τ_roll - B_roll * ω_roll
    roll_accel_actual(k) = (roll_torque(k) - B_roll * roll_rate_actual(k)) / Ixx;
    roll_rate_actual(k+1) = roll_rate_actual(k) + roll_accel_actual(k) * dt;
    roll_actual(k+1) = roll_actual(k) + roll_rate_actual(k) * dt;
    
    % 俯仰角动力学：Iyy * α_pitch = τ_pitch - B_pitch * ω_pitch
    pitch_accel_actual(k) = (pitch_torque(k) - B_pitch * pitch_rate_actual(k)) / Iyy;
    pitch_rate_actual(k+1) = pitch_rate_actual(k) + pitch_accel_actual(k) * dt;
    pitch_actual(k+1) = pitch_actual(k) + pitch_rate_actual(k) * dt;
    
    % 偏航角动力学：Izz * α_yaw = τ_yaw - B_yaw * ω_yaw
    yaw_accel_actual(k) = (yaw_torque(k) - B_yaw * yaw_rate_actual(k)) / Izz;
    yaw_rate_actual(k+1) = yaw_rate_actual(k) + yaw_accel_actual(k) * dt;
    yaw_actual(k+1) = yaw_actual(k) + yaw_rate_actual(k) * dt;
end

% 计算最后一时刻的误差
roll_error(end) = roll_desired(end) - roll_actual(end);
pitch_error(end) = pitch_desired(end) - pitch_actual(end);
yaw_error(end) = yaw_desired(end) - yaw_actual(end);

% 计算实际角速度（最后一时刻）
roll_accel_actual(end) = roll_accel_actual(end-1);
pitch_accel_actual(end) = pitch_accel_actual(end-1);
yaw_accel_actual(end) = yaw_accel_actual(end-1);

roll_error_derivative(end) = roll_error_derivative(end-1);
pitch_error_derivative(end) = pitch_error_derivative(end-1);
yaw_error_derivative(end) = yaw_error_derivative(end-1);

%% ================== 性能指标计算 ==================
% 定义收敛阈值
angle_threshold = 0.5 * pi/180;  % 0.5度

% 初始化性能指标结构体
metrics.roll = struct();
metrics.pitch = struct();
metrics.yaw = struct();

% 计算滚转角性能指标
metrics.roll.max_error = max(abs(roll_error)) * 180/pi;  % 最大误差（度）
metrics.roll.steady_state_error = mean(abs(roll_error(end-100:end))) * 180/pi;  % 稳态误差（度）

% 上升时间（从10%到90%）
roll_magnitude = max(abs(roll_desired)) * 0.9;
if roll_magnitude > 0
    idx_10 = find(abs(roll_actual) >= roll_magnitude*0.1, 1, 'first');
    idx_90 = find(abs(roll_actual) >= roll_magnitude*0.9, 1, 'first');
    if ~isempty(idx_10) && ~isempty(idx_90) && idx_90 > idx_10
        metrics.roll.rise_time = t(idx_90) - t(idx_10);
    else
        metrics.roll.rise_time = NaN;
    end
else
    metrics.roll.rise_time = NaN;
end

% 稳定时间（进入±angle_threshold范围内的时间）
settling_idx = find(abs(roll_error) <= angle_threshold, 1, 'first');
if ~isempty(settling_idx)
    metrics.roll.settling_time = t(settling_idx);
else
    metrics.roll.settling_time = NaN;
end

% 计算俯仰角性能指标
metrics.pitch.max_error = max(abs(pitch_error)) * 180/pi;
metrics.pitch.steady_state_error = mean(abs(pitch_error(end-100:end))) * 180/pi;

pitch_magnitude = max(abs(pitch_desired)) * 0.9;
if pitch_magnitude > 0
    idx_10 = find(abs(pitch_actual) >= pitch_magnitude*0.1, 1, 'first');
    idx_90 = find(abs(pitch_actual) >= pitch_magnitude*0.9, 1, 'first');
    if ~isempty(idx_10) && ~isempty(idx_90) && idx_90 > idx_10
        metrics.pitch.rise_time = t(idx_90) - t(idx_10);
    else
        metrics.pitch.rise_time = NaN;
    end
else
    metrics.pitch.rise_time = NaN;
end

settling_idx = find(abs(pitch_error) <= angle_threshold, 1, 'first');
if ~isempty(settling_idx)
    metrics.pitch.settling_time = t(settling_idx);
else
    metrics.pitch.settling_time = NaN;
end

% 计算偏航角性能指标
metrics.yaw.max_error = max(abs(yaw_error)) * 180/pi;
metrics.yaw.steady_state_error = mean(abs(yaw_error(end-100:end))) * 180/pi;

yaw_magnitude = max(abs(yaw_desired)) * 0.9;
if yaw_magnitude > 0
    idx_10 = find(abs(yaw_actual) >= yaw_magnitude*0.1, 1, 'first');
    idx_90 = find(abs(yaw_actual) >= yaw_magnitude*0.9, 1, 'first');
    if ~isempty(idx_10) && ~isempty(idx_90) && idx_90 > idx_10
        metrics.yaw.rise_time = t(idx_90) - t(idx_10);
    else
        metrics.yaw.rise_time = NaN;
    end
else
    metrics.yaw.rise_time = NaN;
end

settling_idx = find(abs(yaw_error) <= angle_threshold, 1, 'first');
if ~isempty(settling_idx)
    metrics.yaw.settling_time = t(settling_idx);
else
    metrics.yaw.settling_time = NaN;
end

%% ================== 结果输出 ==================
% 实际角度输出
angles_actual.roll = roll_actual;      % 实际滚转角 (rad)
angles_actual.pitch = pitch_actual;    % 实际俯仰角 (rad)
angles_actual.yaw = yaw_actual;        % 实际偏航角 (rad)

% 实际角速度输出
angles_actual.roll_rate = roll_rate_actual;      % 实际滚转角速度 (rad/s)
angles_actual.pitch_rate = pitch_rate_actual;    % 实际俯仰角速度 (rad/s)
angles_actual.yaw_rate = yaw_rate_actual;        % 实际偏航角速度 (rad/s)

% 实际角加速度输出
angles_actual.roll_accel = roll_accel_actual;      % 实际滚转角加速度 (rad/s²)
angles_actual.pitch_accel = pitch_accel_actual;    % 实际俯仰角加速度 (rad/s²)
angles_actual.yaw_accel = yaw_accel_actual;        % 实际偏航角加速度 (rad/s²)

% 控制力矩输出
control_torques.roll = roll_torque;    % 滚转控制力矩 (N·m)
control_torques.pitch = pitch_torque;  % 俯仰控制力矩 (N·m)
control_torques.yaw = yaw_torque;      % 偏航控制力矩 (N·m)

% 性能指标输出
performance_summary.time_vector = t;
performance_summary.metrics = metrics;
performance_summary.desired_angles = [roll_desired; pitch_desired; yaw_desired];
performance_summary.actual_angles = [roll_actual; pitch_actual; yaw_actual];
performance_summary.control_torques = [roll_torque; pitch_torque; yaw_torque];
performance_summary.errors = [roll_error; pitch_error; yaw_error];

end % 函数结束