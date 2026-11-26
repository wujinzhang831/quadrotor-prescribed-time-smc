% startup.m  --  将核心函数目录加入 MATLAB 搜索路径
% 用法：在仓库根目录执行 startup，然后运行 scripts/ 下的任意脚本。
root = fileparts(mfilename('fullpath'));
addpath(fullfile(root, 'src'));
addpath(fullfile(root, 'scripts'));
fprintf('quadrotor-prescribed-time-smc: src/ 与 scripts/ 已加入路径\n');
