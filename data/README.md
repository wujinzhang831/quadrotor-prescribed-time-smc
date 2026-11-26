# 数据说明 / Data card

## 1. 目录

| 路径 | 内容 | 说明 |
| --- | --- | --- |
| `raw/quadrotor_simulation_results.mat` | MATLAB 原始记录（18 001 采样点，`dt = 1 ms`，0–18 s） | 结构体 `simulation_results`，17 个字段 |
| `raw/quadrotor_simulation_results_corrected.mat` | 修正版记录（10 001 点，0–10 s） | 含 z 轴发散的失败样本，保留用于问题复现 |
| `processed/A_smc.csv` | 纯预定时间滑模 | 100 Hz，25 列 |
| `processed/B_smc_filter.csv` | 滑模 + 指令滤波 | 同上 |
| `processed/C_smc_pid.csv` | 滑模 → PID 分段 | 同上 |
| `processed/D_full.csv` | 滤波 + 分段（推荐） | 同上 |
| `processed/metrics.csv` | 四种方案性能指标 | 一行一个方案 |
| `processed/metrics.json` | 同上，含全部参数 | 机器可读 |

## 2. CSV 字段

| 列名 | 含义 | 单位 |
| --- | --- | --- |
| `t_s` | 时间 | s |
| `x_m`, `y_m`, `z_m` | 位置 | m |
| `vx_mps`, `vy_mps`, `vz_mps` | 速度 | m/s |
| `ax_mps2`, `ay_mps2`, `az_mps2` | 加速度 | m/s² |
| `ex_m`, `ey_m`, `ez_m` | 位置误差（位置 − 期望） | m |
| `ux_N`, `uy_N`, `uz_N` | 外环虚拟控制力 | N |
| `sx_mps`, `sy_mps`, `sz_mps` | 滑模变量 | m/s |
| `roll_cmd_deg`, `pitch_cmd_deg` | 姿态指令 | ° |
| `thrust_cmd_N` | 总推力指令 | N |
| `roll_actual_deg`, `pitch_actual_deg` | 内环实际姿态 | ° |
| `mode_smc1_pid2` | 控制器标志（1 = 滑模，2 = PID） | — |

## 3. 统计口径

- **收敛时间**：三轴 `max|e|` 首次小于阈值（5 cm / 1 cm）并保持的时刻。
- **末端误差**：仿真终点（18 s）三轴误差的最大值。
- **姿态跟踪 RMS**：滚转、俯仰跟踪误差的均方根，取两者较大值。
- **指令变化率**：相邻控制周期姿态指令差分的最大值。

## 4. 复算方式

```bash
python tools/export_results.py    # 直接生成 processed/*.csv 与 metrics.*
```

MATLAB 侧对应 `scripts/SMC_fin.m` 等脚本，输出结构体
`quadrotor_simulation_results.mat`。

## 5. 许可

数据以 CC BY 4.0 发布，引用请注明本仓库。
