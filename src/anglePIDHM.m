function [t, phi, theta, psi] = anglePIDHM(desired_positions, T_set)
%% 四旋翼无人机姿态角伺服控制函数
% 输入参数：
%   desired_positions: 期望位置 [x_desired; y_desired; z_desired]
%   T_set: 预定收敛时间 (秒)
%
% 输出参数：
%   t: 时间向量
%   phi: 实际滚转角随时间变化数据 (rad)
%   theta: 实际俯仰角随时间变化数据 (rad)
%   psi: 实际偏航角随时间变化数据 (rad)

%% ================== 参数设置 ==================
% PID控制器参数
% 滚转角控制器参数
Kp_phi = 200.0;      % 比例增益
Ki_phi = 8.0;        % 积分增益
Kd_phi = 30.0;       % 微分增益
N_phi = 100;         % 滤波器系数（低通滤波器截止频率）

% 俯仰角控制器参数
Kp_theta = 70.0;     % 比例增益
Ki_theta = 5.0;      % 积分增益
Kd_theta = 8.0;      % 微分增益
N_theta = 100;       % 滤波器系数

% 偏航角控制器参数
Kp_psi = 12.0;       % 比例增益
Ki_psi = 6.0;        % 积分增益
Kd_psi = 1.5;        % 微分增益
N_psi = 100;         % 滤波器系数

% 控制系统限制
max_torque_phi = 5.0;     % 滚转角最大控制力矩 (N·m)
max_torque_theta = 5.0;   % 俯仰角最大控制力矩 (N·m)
max_torque_psi = 3.0;     % 偏航角最大控制力矩 (N·m)

max_rate_phi = 120 * pi/180;    % 滚转角最大角速度 (rad/s)
max_rate_theta = 120 * pi/180;  % 俯仰角最大角速度 (rad/s)
max_rate_psi = 90 * pi/180;     % 偏航角最大角速度 (rad/s)

% 初始状态（三个角度初始值均为零）
phi0 = 0;       % 初始滚转角 (rad)
theta0 = 0;     % 初始俯仰角 (rad)
psi0 = 0;       % 初始偏航角 (rad)

p0 = 0;         % 初始滚转角速度 (rad/s)
q0 = 0;         % 初始俯仰角速度 (rad/s)
r0 = 0;         % 初始偏航角速度 (rad/s)

%% ================== 获取期望姿态角轨迹 ==================
% 调用四旋翼控制函数，获取期望的姿态角
[t, desired_angles_deg] = quadrotor_smc(desired_positions, T_set);

% 将期望角度从度转换为弧度
desired_phi = desired_angles_deg(:, 2) * pi/180;    % 期望滚转角 (rad)
desired_theta = desired_angles_deg(:, 3) * pi/180;  % 期望俯仰角 (rad)
desired_psi = desired_angles_deg(:, 1) * pi/180;    % 期望偏航角 (rad)

N = length(t);
dt = t(2) - t(1);

% 计算期望角速度（用于前馈控制）
desired_p = gradient(desired_phi, dt);    % 期望滚转角速度
desired_q = gradient(desired_theta, dt);  % 期望俯仰角速度
desired_r = gradient(desired_psi, dt);    % 期望偏航角速度

% 限制期望角速度范围
desired_p = min(max(desired_p, -max_rate_phi), max_rate_phi);
desired_q = min(max(desired_q, -max_rate_theta), max_rate_theta);
desired_r = min(max(desired_r, -max_rate_psi), max_rate_psi);

%% ================== 初始化控制系统 ==================
% 初始化状态变量
phi = zeros(N, 1);      % 实际滚转角 (rad)
theta = zeros(N, 1);    % 实际俯仰角 (rad)
psi = zeros(N, 1);      % 实际偏航角 (rad)

phi(1) = phi0;
theta(1) = theta0;
psi(1) = psi0;

p = zeros(N, 1);        % 实际滚转角速度 (rad/s)
q = zeros(N, 1);        % 实际俯仰角速度 (rad/s)
r = zeros(N, 1);        % 实际偏航角速度 (rad/s)

p(1) = p0;
q(1) = q0;
r(1) = r0;

% 初始化控制变量
u_phi = zeros(N, 1);    % 滚转角控制力矩 (N·m)
u_theta = zeros(N, 1);  % 俯仰角控制力矩 (N·m)
u_psi = zeros(N, 1);    % 偏航角控制力矩 (N·m)

% 初始化误差变量
e_phi = zeros(N, 1);    % 滚转角误差
e_theta = zeros(N, 1);  % 俯仰角误差
e_psi = zeros(N, 1);    % 偏航角误差

% 初始化PID控制器状态
% 滚转角PID
integral_phi = 0;       % 积分项
prev_error_phi = 0;     % 上一次误差
prev_derivative_phi = 0;% 上一次微分

% 俯仰角PID
integral_theta = 0;
prev_error_theta = 0;
prev_derivative_theta = 0;

% 偏航角PID
integral_psi = 0;
prev_error_psi = 0;
prev_derivative_psi = 0;

% 四旋翼动力学参数
Ixx = 0.015;    % 绕X轴转动惯量 (kg·m²)
Iyy = 0.015;    % 绕Y轴转动惯量 (kg·m²)
Izz = 0.028;    % 绕Z轴转动惯量 (kg·m²)

% 阻尼系数
damping_p = 0.02;   % 滚转角速度阻尼
damping_q = 0.02;   % 俯仰角速度阻尼
damping_r = 0.01;   % 偏航角速度阻尼

%% ================== 主控制循环 ==================
for k = 1:N-1
    % ===== 计算当前时刻的误差 =====
    e_phi(k) = desired_phi(k) - phi(k);
    e_theta(k) = desired_theta(k) - theta(k);
    e_psi(k) = desired_psi(k) - psi(k);
    
    % ===== 滚转角PID控制 =====
    % 比例项
    P_phi = Kp_phi * e_phi(k);
    
    % 积分项（带抗饱和）
    integral_phi = integral_phi + e_phi(k) * dt;
    % 积分抗饱和：如果控制量达到上限，停止积分
    if abs(u_phi(k)) >= max_torque_phi * 0.9
        integral_phi = integral_phi - e_phi(k) * dt;
    end
    I_phi = Ki_phi * integral_phi;
    
    % 微分项（带滤波器）
    derivative_phi = (e_phi(k) - prev_error_phi) / dt;
    % 低通滤波器
    derivative_phi_filtered = prev_derivative_phi + dt * N_phi * (derivative_phi - prev_derivative_phi);
    D_phi = Kd_phi * derivative_phi_filtered;
    
    % 前馈项（使用期望角速度）
    FF_phi = 0.5 * desired_p(k);  % 前馈增益
    
    % 总控制量
    u_phi_raw = P_phi + I_phi + D_phi + FF_phi;
    
    % 控制量限幅
    u_phi(k+1) = min(max(u_phi_raw, -max_torque_phi), max_torque_phi);
    
    % 更新历史值
    prev_error_phi = e_phi(k);
    prev_derivative_phi = derivative_phi_filtered;
    
    % ===== 俯仰角PID控制 =====
    % 比例项
    P_theta = Kp_theta * e_theta(k);
    
    % 积分项
    integral_theta = integral_theta + e_theta(k) * dt;
    if abs(u_theta(k)) >= max_torque_theta * 0.9
        integral_theta = integral_theta - e_theta(k) * dt;
    end
    I_theta = Ki_theta * integral_theta;
    
    % 微分项
    derivative_theta = (e_theta(k) - prev_error_theta) / dt;
    derivative_theta_filtered = prev_derivative_theta + dt * N_theta * (derivative_theta - prev_derivative_theta);
    D_theta = Kd_theta * derivative_theta_filtered;
    
    % 前馈项
    FF_theta = 0.5 * desired_q(k);
    
    % 总控制量
    u_theta_raw = P_theta + I_theta + D_theta + FF_theta;
    u_theta(k+1) = min(max(u_theta_raw, -max_torque_theta), max_torque_theta);
    
    % 更新历史值
    prev_error_theta = e_theta(k);
    prev_derivative_theta = derivative_theta_filtered;
    
    % ===== 偏航角PID控制 =====
    % 比例项
    P_psi = Kp_psi * e_psi(k);
    
    % 积分项
    integral_psi = integral_psi + e_psi(k) * dt;
    if abs(u_psi(k)) >= max_torque_psi * 0.9
        integral_psi = integral_psi - e_psi(k) * dt;
    end
    I_psi = Ki_psi * integral_psi;
    
    % 微分项
    derivative_psi = (e_psi(k) - prev_error_psi) / dt;
    derivative_psi_filtered = prev_derivative_psi + dt * N_psi * (derivative_psi - prev_derivative_psi);
    D_psi = Kd_psi * derivative_psi_filtered;
    
    % 前馈项
    FF_psi = 0.5 * desired_r(k);
    
    % 总控制量
    u_psi_raw = P_psi + I_psi + D_psi + FF_psi;
    u_psi(k+1) = min(max(u_psi_raw, -max_torque_psi), max_torque_psi);
    
    % 更新历史值
    prev_error_psi = e_psi(k);
    prev_derivative_psi = derivative_psi_filtered;
    
    % ===== 姿态动力学更新 =====
    % 简化版四旋翼姿态动力学
    % 滚转角动力学
    p_dot = (u_phi(k+1) - damping_p * p(k)) / Ixx;
    p(k+1) = p(k) + dt * p_dot;
    
    % 俯仰角动力学
    q_dot = (u_theta(k+1) - damping_q * q(k)) / Iyy;
    q(k+1) = q(k) + dt * q_dot;
    
    % 偏航角动力学
    r_dot = (u_psi(k+1) - damping_r * r(k)) / Izz;
    r(k+1) = r(k) + dt * r_dot;
    
    % 角度更新（考虑角速度限制）
    p_limited = min(max(p(k+1), -max_rate_phi), max_rate_phi);
    q_limited = min(max(q(k+1), -max_rate_theta), max_rate_theta);
    r_limited = min(max(r(k+1), -max_rate_psi), max_rate_psi);
    
    phi(k+1) = phi(k) + dt * p_limited;
    theta(k+1) = theta(k) + dt * q_limited;
    psi(k+1) = psi(k) + dt * r_limited;
    
    % 角度限制（±180度）
    phi(k+1) = wrapToPi(phi(k+1));
    theta(k+1) = wrapToPi(theta(k+1));
    psi(k+1) = wrapToPi(psi(k+1));
end

% 计算最后一点的误差
e_phi(end) = desired_phi(end) - phi(end);
e_theta(end) = desired_theta(end) - theta(end);
e_psi(end) = desired_psi(end) - psi(end);

end

%% ================== 辅助函数 ==================
% 角度限制在[-π, π]范围内
function angle = wrapToPi(angle)
    angle = mod(angle + pi, 2*pi) - pi;
end

