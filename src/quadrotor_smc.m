function [t, angles, control_forces] = quadrotor_smc(desired_pos, T)
%% 四旋翼无人机位置预定时间滑模控制仿真
% 输入:
%   desired_pos: [x_desired, y_desired, z_desired] 期望位置向量
%   T: 预定收敛时间 (s)
%
% 输出:
%   t: 时间向量
%   angles: 结构体，包含各时刻的偏航角、滚转角、俯仰角 (rad)
%   control_forces: 结构体，包含各时刻的控制力 (N)

%% ================== 系统参数 ==================
m = 0.905;          % 质量 (kg)
g = 9.8;            % 重力加速度 (m/s^2)

% 阻力系数
kx = 0.1;           % X方向阻力系数 (N·s/m)
ky = 0.1;           % Y方向阻力系数 (N·s/m)
kz = 0.1;           % Z方向阻力系数 (N·s/m)

% 初始状态
x0 = 5;             % 初始X位置 (m)
y0 = -3;            % 初始Y位置 (m)
z0 = 10;            % 初始高度 (m)
vx0 = 0;            % 初始X速度 (m/s)
vy0 = 0;            % 初始Y速度 (m/s)
vz0 = 0;            % 初始Z速度 (m/s)

% 期望位置（从输入获取）
x_desired = desired_pos(1);  % 目标X位置
y_desired = desired_pos(2);  % 目标Y位置  
z_desired = desired_pos(3);  % 目标高度

% 外部扰动设置
d_max = 0.5;        % 最大扰动幅值 (N)
freq_x = 0.5;       % X方向扰动频率 (Hz)
freq_y = 0.6;       % Y方向扰动频率 (Hz)
freq_z = 0.7;       % Z方向扰动频率 (Hz)

%% ================== 控制器参数设置 ==================
% 核心参数调整
a_x = -0.7916;      % X方向滑模面参数（增大负值加速收敛）
a_y = -0.7916;      % Y方向滑模面参数
a_z = -0.7916;      % Z方向滑模面参数

% 收敛参数
alpha_val = 0.9;    % α ∈ (0.5, 1)
beta_val = 1.2;     % β > 1

c = 10/T;          % 控制增益，确保在预定时间T内收敛
d = 10/T;          % 控制增益

phi = 0.001;        % 更小的边界层以获得更精确控制
R_type = 1;         % 使用R1函数（sin²型）

% 延迟时间
h = T/3;            % h = τ = T/3

% 扰动上界
rho_x = d_max/m + 0.1;  % X方向扰动上界，增加裕度
rho_y = d_max/m + 0.1;  % Y方向扰动上界
rho_z = d_max/m + 0.1;  % Z方向扰动上界

%% ================== 仿真设置 ==================
% 时间设置
dt = 0.001;                     % 仿真步长 (s)
t = 0:dt:T*2;                   % 延长仿真时间以观察收敛情况
N = length(t);

% 初始化状态数组
% X方向
x1 = zeros(1, N);               % X位置误差
x2 = zeros(1, N);               % X速度误差
u_x = zeros(1, N);              % X方向控制力
s_x = zeros(1, N);              % X方向滑模面
K_x = zeros(1, N);              % X方向时变增益
Kdot_x = zeros(1, N);           % X方向K(t)导数

% Y方向
y1 = zeros(1, N);               % Y位置误差
y2 = zeros(1, N);               % Y速度误差
u_y = zeros(1, N);              % Y方向控制力
s_y = zeros(1, N);              % Y方向滑模面
K_y = zeros(1, N);              % Y方向时变增益
Kdot_y = zeros(1, N);           % Y方向K(t)导数

% Z方向
z1 = zeros(1, N);               % Z位置误差
z2 = zeros(1, N);               % Z速度误差
u_z = zeros(1, N);              % Z方向控制力
s_z = zeros(1, N);              % Z方向滑模面
K_z = zeros(1, N);              % Z方向时变增益
Kdot_z = zeros(1, N);           % Z方向K(t)导数

% 角度输出
phi_angle = zeros(1, N);        % 滚转角 (rad)
theta_angle = zeros(1, N);      % 俯仰角 (rad)
psi_angle = zeros(1, N);        % 偏航角 (rad)
thrust = zeros(1, N);           % 总推力 (N)

% R函数值
R_val = zeros(1, N);            % R(t)函数值

% 历史状态缓冲区
history_buffer_size = ceil(h/dt) + 10;

% X方向历史状态
x1_history = zeros(1, history_buffer_size);
x2_history = zeros(1, history_buffer_size);

% Y方向历史状态
y1_history = zeros(1, history_buffer_size);
y2_history = zeros(1, history_buffer_size);

% Z方向历史状态
z1_history = zeros(1, history_buffer_size);
z2_history = zeros(1, history_buffer_size);

% 初始条件
% X方向
x1(1) = x0 - x_desired;
x2(1) = vx0;
x1_history(1) = x1(1);
x2_history(1) = x2(1);

% Y方向
y1(1) = y0 - y_desired;
y2(1) = vy0;
y1_history(1) = y1(1);
y2_history(1) = y2(1);

% Z方向
z1(1) = z0 - z_desired;
z2(1) = vz0;
z1_history(1) = z1(1);
z2_history(1) = z2(1);

% 外部扰动函数
d_x = @(t) d_max * sin(2*pi*freq_x*t);
d_y = @(t) d_max * cos(2*pi*freq_y*t);
d_z = @(t) d_max * sin(2*pi*freq_z*t + pi/4);

%% ================== 主仿真循环 ==================
for k = 1:N-1
    current_t = t(k);
    
    % ===== X方向控制 =====
    % 计算X方向的K(t)和K'(t)
    [K_x(k), Kdot_x(k)] = K_func(current_t, h, a_x, R_type);
    
    % 获取X方向历史状态
    if current_t >= h
        hist_idx = max(1, k - ceil(h/dt));
        x1_h = x1(hist_idx);
        x2_h = x2(hist_idx);
    else
        x1_h = x1(1);
        x2_h = x2(1);
    end
    
    % 计算X方向滑模面
    s_x(k) = x2(k) - a_x*x1(k) + K_x(k)*x1_h;
    
    % 计算X方向扰动
    delta_x = d_x(current_t) / m;
    
    % 计算X方向控制输入
    sgn_sx = sat(s_x(k), phi);
    [f_x, g_x] = get_system_params('x', x2(k), m, g, kx, ky, kz);
    
    u_x(k) = (1/g_x) * ( ...
        -c/(2^alpha_val) * sig(abs(s_x(k)), 2*alpha_val-1) * sign(s_x(k)) ...
        -d/(2^beta_val) * sig(abs(s_x(k)), 2*beta_val-1) * sign(s_x(k)) ...
        -rho_x * sgn_sx ...
        - f_x ...
        + a_x * x2(k) ...
        - K_x(k) * x2_h ...
        - Kdot_x(k) * x1_h ...
        );
    
    % ===== Y方向控制 =====
    % 计算Y方向的K(t)和K'(t)
    [K_y(k), Kdot_y(k)] = K_func(current_t, h, a_y, R_type);
    
    % 获取Y方向历史状态
    if current_t >= h
        hist_idx = max(1, k - ceil(h/dt));
        y1_h = y1(hist_idx);
        y2_h = y2(hist_idx);
    else
        y1_h = y1(1);
        y2_h = y2(1);
    end
    
    % 计算Y方向滑模面
    s_y(k) = y2(k) - a_y*y1(k) + K_y(k)*y1_h;
    
    % 计算Y方向扰动
    delta_y = d_y(current_t) / m;
    
    % 计算Y方向控制输入
    sgn_sy = sat(s_y(k), phi);
    [f_y, g_y] = get_system_params('y', y2(k), m, g, kx, ky, kz);
    
    u_y(k) = (1/g_y) * ( ...
        -c/(2^alpha_val) * sig(abs(s_y(k)), 2*alpha_val-1) * sign(s_y(k)) ...
        -d/(2^beta_val) * sig(abs(s_y(k)), 2*beta_val-1) * sign(s_y(k)) ...
        -rho_y * sgn_sy ...
        - f_y ...
        + a_y * y2(k) ...
        - K_y(k) * y2_h ...
        - Kdot_y(k) * y1_h ...
        );
    
    % ===== Z方向控制 =====
    % 计算Z方向的K(t)和K'(t)
    [K_z(k), Kdot_z(k)] = K_func(current_t, h, a_z, R_type);
    
    % 获取Z方向历史状态
    if current_t >= h
        hist_idx = max(1, k - ceil(h/dt));
        z1_h = z1(hist_idx);
        z2_h = z2(hist_idx);
    else
        z1_h = z1(1);
        z2_h = z2(1);
    end
    
    % 计算Z方向滑模面
    s_z(k) = z2(k) - a_z*z1(k) + K_z(k)*z1_h;
    
    % 计算Z方向扰动
    delta_z = d_z(current_t) / m;
    
    % 计算Z方向控制输入
    sgn_sz = sat(s_z(k), phi);
    [f_z, g_z] = get_system_params('z', z2(k), m, g, kx, ky, kz);
    
    u_z(k) = (1/g_z) * ( ...
        -c/(2^alpha_val) * sig(abs(s_z(k)), 2*alpha_val-1) * sign(s_z(k)) ...
        -d/(2^beta_val) * sig(abs(s_z(k)), 2*beta_val-1) * sign(s_z(k)) ...
        -rho_z * sgn_sz ...
        - f_z ...
        + a_z * z2(k) ...
        - K_z(k) * z2_h ...
        - Kdot_z(k) * z1_h ...
        );
    
    % ===== 角度解算 =====
    % 根据控制输入解算姿态角和推力
    [phi_angle(k), theta_angle(k), psi_angle(k), thrust(k)] = ...
        control_to_angles(u_x(k), u_y(k), u_z(k), m, g);
    
    % ===== 计算R(t)函数值 =====
    R_val(k) = R_h_func(current_t, h, R_type);
    
    % ===== 状态更新 =====
    % X方向状态更新
    x1(k+1) = x1(k) + dt * x2(k);
    x2(k+1) = x2(k) + dt * (f_x + g_x * u_x(k) + delta_x);
    
    % Y方向状态更新
    y1(k+1) = y1(k) + dt * y2(k);
    y2(k+1) = y2(k) + dt * (f_y + g_y * u_y(k) + delta_y);
    
    % Z方向状态更新
    z1(k+1) = z1(k) + dt * z2(k);
    z2(k+1) = z2(k) + dt * (f_z + g_z * u_z(k) + delta_z);
    
    % 更新历史缓冲区
    if k < history_buffer_size
        x1_history(k+1) = x1(k+1);
        x2_history(k+1) = x2(k+1);
        y1_history(k+1) = y1(k+1);
        y2_history(k+1) = y2(k+1);
        z1_history(k+1) = z1(k+1);
        z2_history(k+1) = z2(k+1); 
    end
end


% 计算最后一点的值
t_end = t(end);

% X方向
[K_x(end), Kdot_x(end)] = K_func(t_end, h, a_x, R_type);
if t_end >= h
    hist_idx = max(1, N - ceil(h/dt));
    x1_h = x1(hist_idx);
    x2_h = x2(hist_idx);
else
    x1_h = x1(1);
    x2_h = x2(1);
end
s_x(end) = x2(end) - a_x*x1(end) + K_x(end)*x1_h;
sgn_sx = sat(s_x(end), phi);
[f_x, g_x] = get_system_params('x', x2(end), m, g, kx, ky, kz);
u_x(end) = (1/g_x) * ( ...
    -c/(2^alpha_val) * sig(abs(s_x(end)), 2*alpha_val-1) * sign(s_x(end)) ...
    -d/(2^beta_val) * sig(abs(s_x(end)), 2*beta_val-1) * sign(s_x(end)) ...
    -rho_x * sgn_sx ...
    - f_x ...
    + a_x * x2(end) ...
    - K_x(end) * x2_h ...
    - Kdot_x(end) * x1_h ...
    );

% Y方向
[K_y(end), Kdot_y(end)] = K_func(t_end, h, a_y, R_type);
if t_end >= h
    hist_idx = max(1, N - ceil(h/dt));
    y1_h = y1(hist_idx);
    y2_h = y2(hist_idx);
else
    y1_h = y1(1);
    y2_h = y2(1);
end
s_y(end) = y2(end) - a_y*y1(end) + K_y(end)*y1_h;
sgn_sy = sat(s_y(end), phi);
[f_y, g_y] = get_system_params('y', y2(end), m, g, kx, ky, kz);
u_y(end) = (1/g_y) * ( ...
    -c/(2^alpha_val) * sig(abs(s_y(end)), 2*alpha_val-1) * sign(s_y(end)) ...
    -d/(2^beta_val) * sig(abs(s_y(end)), 2*beta_val-1) * sign(s_y(end)) ...
    -rho_y * sgn_sy ...
    - f_y ...
    + a_y * y2(end) ...
    - K_y(end) * y2_h ...
    - Kdot_y(end) * y1_h ...
    );

% Z方向
[K_z(end), Kdot_z(end)] = K_func(t_end, h, a_z, R_type);
if t_end >= h
    hist_idx = max(1, N - ceil(h/dt));
    z1_h = z1(hist_idx);
    z2_h = z2(hist_idx);
else
    z1_h = z1(1);
    z2_h = z2(1);
end
s_z(end) = z2(end) - a_z*z1(end) + K_z(end)*z1_h;
sgn_sz = sat(s_z(end), phi);
[f_z, g_z] = get_system_params('z', z2(end), m, g, kx, ky, kz);
u_z(end) = (1/g_z) * ( ...
    -c/(2^alpha_val) * sig(abs(s_z(end)), 2*alpha_val-1) * sign(s_z(end)) ...
    -d/(2^beta_val) * sig(abs(s_z(end)), 2*beta_val-1) * sign(s_z(end)) ...
    -rho_z * sgn_sz ...
    - f_z ...
    + a_z * z2(end) ...
    - K_z(end) * z2_h ...
    - Kdot_z(end) * z1_h ...
    );

% 最后一点的角度解算和R(t)计算
[phi_angle(end), theta_angle(end), psi_angle(end), thrust(end)] = ...
    control_to_angles(u_x(end), u_y(end), u_z(end), m, g);
R_val(end) = R_h_func(t_end, h, R_type);

%% ================== 结果输出 ==================
% 将结果打包到输出结构体中
angles.yaw = psi_angle;      % 偏航角 (rad)
angles.roll = phi_angle;     % 滚转角 (rad)
angles.pitch = theta_angle;  % 俯仰角 (rad)
angles.thrust = thrust;      % 总推力 (N)

control_forces.u_x = u_x;    % X方向控制力 (N)
control_forces.u_y = u_y;    % Y方向控制力 (N)
control_forces.u_z = u_z;    % Z方向控制力 (N)

% 可选：输出收敛时间信息
convergence_threshold = 0.01; % 收敛阈值
x_converged_idx = find(abs(x1) < convergence_threshold, 1, 'first');
y_converged_idx = find(abs(y1) < convergence_threshold, 1, 'first');
z_converged_idx = find(abs(z1) < convergence_threshold, 1, 'first');

if ~isempty(x_converged_idx) && ~isempty(y_converged_idx) && ~isempty(z_converged_idx)
    convergence_time = max([t(x_converged_idx), t(y_converged_idx), t(z_converged_idx)]);
    fprintf('预定收敛时间: %.2f s\n', T);
    fprintf('实际收敛时间: %.3f s\n', convergence_time);
    fprintf('三个方向位置误差均小于 %.3f m\n', convergence_threshold);
else
    fprintf('未在仿真时间内收敛到指定精度\n');
end

end % 主函数结束

%% ================== 辅助函数定义 ==================
% 符号函数（带边界层）
function result = sat(x, phi_val)
    result = min(max(x/phi_val, -1), 1);
end

% sig函数：sig^γ(s) = |s|^γ * sign(s)
function result = sig(s_val, gamma)
    result = abs(s_val)^gamma * sign(s_val);
end

% 计算R_h(t)函数（基于论文表2）
function R = R_h_func(t, h, type)
    % t: 当前时间
    % h: 延迟时间
    % type: 1-R1, 2-R2, 3-R3
    
    % 将时间映射到周期[0, 2h]
    t_mod = mod(t, 2*h);
    
    if t_mod < h
        R = 0;
    else
        if type == 1
            % R1(t) = sin^2(π(t-h)/h)
            R = sin(pi*(t_mod-h)/h)^2;
            
        elseif type == 2
            % R2(t) = (t-h)^2 * (2h-t)^2 / h^4 * C (平滑周期函数)
            % 这是一个在[h,2h]上平滑的函数，在h和2h处为0
            if t_mod <= 2*h
                C = 16;  % 使得在t_mod=1.5h时，R2≈1
                R = C * ((t_mod-h)^2 * (2*h-t_mod)^2) / h^4;
            else
                R = 0;
            end
            
        elseif type == 3
            % R3(t) = (t-h)^3 * (2h-t)^3 / h^6 * C (更平滑的周期函数)
            if t_mod <= 2*h
                C = 64;  % 归一化因子
                R = C * ((t_mod-h)^3 * (2*h-t_mod)^3) / h^6;
            else
                R = 0;
            end
            
        else
            error('R_type must be 1, 2, or 3');
        end
    end
end

% 计算K(t)函数（避免递归的简化版本）
function [K_val, Kdot_val] = K_func(t, h, a_val, R_type)
    % 计算K(t)和K'(t)
    
    if t < h
        K_val = 0;
        Kdot_val = 0;
    else
        % 计算τ = t - h
        tau = t - h;
        
        % 将τ映射到[0, 2h]区间
        tau_mod = mod(tau, 2*h);
        
        if tau_mod < h
            K_val = 0;
            Kdot_val = 0;
        else
            % 计算s = τ_mod - h，s ∈ [0, h]
            s = tau_mod - h;
            
            if a_val == 0
                % a=0时的简化表达式
                if R_type == 1
                    % R1: K(t) = (2/h) * sin^2(πs/h)
                    K_val = (2/h) * sin(pi*s/h)^2;
                    Kdot_val = (2*pi/h^2) * sin(2*pi*s/h);
                elseif R_type == 2
                    % R2: K(t) = (2/h) * R2(t)
                    R_val = R_h_func(t, h, R_type);
                    K_val = (2/h) * R_val;
                    
                    % 数值计算导数
                    dt_small = 1e-6;
                    R_val_plus = R_h_func(t+dt_small, h, R_type);
                    R_val_minus = R_h_func(t-dt_small, h, R_type);
                    K_val_plus = (2/h) * R_val_plus;
                    K_val_minus = (2/h) * R_val_minus;
                    Kdot_val = (K_val_plus - K_val_minus) / (2*dt_small);
                elseif R_type == 3
                    % R3: K(t) = (2/h) * R3(t)
                    R_val = R_h_func(t, h, R_type);
                    K_val = (2/h) * R_val;
                    
                    % 数值计算导数
                    dt_small = 1e-6;
                    R_val_plus = R_h_func(t+dt_small, h, R_type);
                    R_val_minus = R_h_func(t-dt_small, h, R_type);
                    K_val_plus = (2/h) * R_val_plus;
                    K_val_minus = (2/h) * R_val_minus;
                    Kdot_val = (K_val_plus - K_val_minus) / (2*dt_small);
                end
            else
                % a≠0的一般情况
                % 统一使用数值导数，避免复杂推导
                R_val = R_h_func(t, h, R_type);
                K_val = (2/h) * R_val * exp(-2*a_val*(t-h));
                
                % 数值计算导数
                dt_small = 1e-6;
                R_val_plus = R_h_func(t+dt_small, h, R_type);
                R_val_minus = R_h_func(t-dt_small, h, R_type);
                K_val_plus = (2/h) * R_val_plus * exp(-2*a_val*(t+dt_small-h));
                K_val_minus = (2/h) * R_val_minus * exp(-2*a_val*(t-dt_small-h));
                Kdot_val = (K_val_plus - K_val_minus) / (2*dt_small);
            end
        end
    end
end

% 各方向系统动力学函数
function [f, g] = get_system_params(dir, state, m_val, g_val, kx_val, ky_val, kz_val)
    % 获取各方向的f和g函数值
    % dir: 'x', 'y', 'z'
    % state: 当前状态（速度）
    
    if dir == 'x'
        f = -(kx_val/m_val) * state;
        g = 1/m_val;
    elseif dir == 'y'
        f = -(ky_val/m_val) * state;
        g = 1/m_val;
    elseif dir == 'z'
        f = -g_val - (kz_val/m_val) * state;
        g = 1/m_val;
    else
        error('方向必须是 x, y 或 z');
    end
end

% 控制输入到角度解算函数
function [phi, theta, psi, f] = control_to_angles(u_x, u_y, u_z, m, g)
    % 根据控制输入解算无人机姿态角和总推力
    % 输入: u_x, u_y, u_z - 三个方向的控制力 (N)
    %       m - 质量 (kg)
    %       g - 重力加速度 (m/s^2)
    % 输出: phi - 滚转角 (rad)
    %       theta - 俯仰角 (rad)
    %       psi - 偏航角 (rad)
    %       f - 总推力 (N)
    
    % 假设偏航角ψ=0（大多数情况下无人机保持固定朝向）
    psi = 0;
    
    % 计算总推力 f = m * g - m * u_z
    f = m * (g - u_z/m);
    
    % 确保推力非负，但设置更大的最小推力
    f = max(f, 1.0); % 最小推力1.0N避免除以零
    
    % 计算期望加速度
    a_x = -u_x / m;
    a_y = -u_y / m;
    
    % 计算滚转角φ
    sin_phi = (a_y * m) / f;
    sin_phi = max(min(sin_phi, 1), -1);
    phi = asin(sin_phi);
    
    % 计算俯仰角θ
    cos_phi = cos(phi);
    if abs(cos_phi) < 1e-6
        theta = 0;
    else
        sin_theta = -(a_x * m) / (f * cos_phi);
        sin_theta = max(min(sin_theta, 1), -1);
        theta = asin(sin_theta);
    end
    
    % 角度限制在合理范围内（±30度）
    max_angle = 30 * pi/180;
    phi = max(min(phi, max_angle), -max_angle);
    theta = max(min(theta, max_angle), -max_angle);
end