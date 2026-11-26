# -*- coding: utf-8 -*-
"""Generate every figure that appears in the report and in the repository docs."""
from __future__ import annotations

import math
import os
import sys

import numpy as np

import plotlib as P
from qsmc_sim import Params, metrics, run_attitude_loop, run_position_loop

_HERE = os.path.dirname(os.path.abspath(__file__))
# in the repository the script lives in tools/ and the figures live one level up
OUT = os.path.join(os.path.dirname(_HERE) if os.path.basename(_HERE) == "tools" else _HERE, "figures")
DEG = np.degrees

P_MAIN = Params()
T = P_MAIN.T
T_SWITCH = T - 0.495 * math.exp(9 - T) - 0.405


def decimate(a, step):
    return np.asarray(a)[::step]


# ---------------------------------------------------------------------------
def fig_flow(path):
    cv = P.Canvas(1500, 1560)
    cx = 750
    w, h = 620, 96
    steps = [
        ("S1  参数装订：m, g, k, T, h = T/3, α, β, ρ, φ, 采样步长 Δt", None),
        ("S2  采样位置/速度，构造误差 e₁ = p − p_d, e₂ = v", None),
        ("S3  计算 R₁(t)、K(t)、K̇(t)，构造滑模面\ns = e₂ − a·e₁ + K(t)·e₁(t − h)", None),
        ("S4  计算三轴等效控制力\nu = g⁻¹[−c·sig^α(s) − d·sig^β(s) − ρ·sat(s/φ) − f + a·e₂ − K·e₂(t−h)]", None),
        ("S5  力/力矩分配：解算滚转、俯仰指令 (φ_d, θ_d) 与总推力 T_c", None),
        ("S6  姿态指令一阶低通滤波（截止频率 f_c）", None),
        ("S7  内环 PID 姿态跟踪，输出力矩 τ，积分位置动力学", None),
        ("S8  是否到达切换时刻 t_s？", "decision"),
        ("S9  滑模控制继续（预定时间收敛段）", "side"),
        ("S10  PID 位置保持（抑制抖振段）", "side"),
        ("S11 输出状态、控制量、性能指标，结束", None),
    ]
    y = 40
    boxes = []
    d = cv.draw
    for i, (label, kind) in enumerate(steps):
        if kind == "decision":
            pass
        lines = label.count("\n") + 1
        bh = 70 if lines == 1 else 70 + 30 * (lines - 1)
        if kind == "decision":
            bh = 92
        box = (cx - w / 2, y, cx + w / 2, y + bh)
        boxes.append((box, kind, label, bh))
        y += bh + 62
    total = y
    off = (1560 - total) // 2
    shift = off - 40

    prev_bottom = None
    for (box, kind, label, bh) in boxes:
        b = (box[0], box[1] + shift, box[2], box[3] + shift)
        if kind == "decision":
            cx_mid = (b[0] + b[2]) / 2
            cy_mid = (b[1] + b[3]) / 2
            d.polygon([(cx_mid, b[1]), (b[2], cy_mid), (cx_mid, b[3]), (b[0], cy_mid)],
                      fill=(250, 246, 232), outline=P.INK, width=2)
        else:
            fill = (240, 246, 252) if kind != "side" else (246, 246, 246)
            if kind == "side":
                box_w = 430
                b = (cx - box_w / 2, b[1], cx + box_w / 2, b[3])
                fill = (240, 250, 244)
            cv.rect(b, fill=fill, outline=P.INK, width=2, radius=8)
        cv.text((b[0] + b[2]) / 2, (b[1] + b[3]) / 2, label, size=21)
        if prev_bottom is not None:
            cv.arrow((cx, prev_bottom), (cx, b[1] - 2))
        prev_bottom = b[3]

    # decision branch labels
    dec = [b for (b, k, _, _) in [( (bx[0], bx[1] + shift, bx[2], bx[3] + shift), k, l, bh) for (bx, k, l, bh) in boxes] if k == "decision"][0]
    cv.text(dec[2] + 16, (dec[1] + dec[3]) / 2, "否", size=20, anchor="lm")
    d.text((cx + 10, dec[1] - 6), "是 → 转入 PID 位置保持（S10）", font=P.font(17), fill=P.INK)
    cv.save(path)
    return path


def fig_arch(path):
    cv = P.Canvas(1560, 820)
    d = cv.draw
    # blocks
    def box(x, y, w, h, text, fill=(240, 246, 252)):
        cv.rect((x, y, x + w, y + h), fill=fill, outline=P.INK, width=2, radius=10)
        cv.text(x + w / 2, y + h / 2, text, size=20)
        return (x, y, x + w, y + h)

    b1 = box(40, 90, 230, 110, "期望位置\np_d = [x_d, y_d, z_d]", (250, 246, 232))
    b2 = box(320, 70, 250, 150, "外环\n预定时间滑模\n位置控制器", (235, 243, 252))
    b3 = box(620, 70, 230, 150, "力/力矩分配\n姿态指令解算\nφ_d, θ_d, T_c", (235, 243, 252))
    b4 = box(900, 70, 200, 150, "低通滤波\nf_c = 5 Hz", (245, 240, 250))
    b5 = box(1150, 70, 230, 150, "内环\nPID 姿态控制器\nτ_φ, τ_θ, τ_ψ", (235, 243, 252))
    b6 = box(1150, 380, 230, 130, "四旋翼动力学\nI·ω̇ = τ − B·ω\nṗ = v, v̇ = a", (240, 250, 244))
    b7 = box(620, 400, 230, 110, "位置/姿态\n状态测量", (240, 250, 244))
    b8 = box(320, 560, 250, 110, "切换判据\nt ≥ t_s ?", (250, 246, 232))
    b9 = box(320, 700, 250, 90, "PID 位置保持\nK_p, K_i, K_d", (250, 246, 232))

    cv.arrow((b1[2], b1[1] + 55), (b2[0] - 4, b2[1] + 75))
    cv.arrow((b2[2], b2[1] + 75), (b3[0] - 4, b3[1] + 75))
    cv.arrow((b3[2], b3[1] + 75), (b4[0] - 4, b4[1] + 75))
    cv.arrow((b4[2], b4[1] + 75), (b5[0] - 4, b5[1] + 75))
    cv.arrow((b5[0] + 115, b5[3]), (b6[0] + 115, b6[1] - 4))
    cv.arrow((b6[0], b6[1] + 65), (b7[2] + 6, b7[1] + 55))
    cv.arrow((b7[0], b7[1] + 55), (b2[2] - 60, b2[3] + 6))
    cv.arrow((b7[0] + 40, b7[3]), (b8[0] + 125, b8[1] - 4))
    cv.arrow((b8[0] + 125, b8[3]), (b9[0] + 125, b9[1] - 4))
    cv.arrow((b9[2], b9[1] + 45), (b3[0] + 115, b3[3] + 6))
    cv.text(330, 640, "是 → 精调", size=17, anchor="lm")
    cv.text(880, 640, "否 → 滑模", size=17, anchor="lm")
    cv.arrow((b2[0] - 40, b2[1] + 20), (b2[0] - 4, b2[1] + 20))
    cv.text(150, 560, "d(t)：风扰/模型摄动\n|d| ≤ 0.5 N", size=17)
    cv.save(path)
    return path


def fig_states(path, res, title):
    cv, boxes = P.grid_canvas(1700, 1210, 3, 3, left=150, right=45, top=125, bottom=110,
                              hgap=150, vgap=80)
    names = ["X", "Y", "Z"]
    t = decimate(res.t, 5)
    for c, ax_name in enumerate(names):
        yl = "位置 (m)" if c == 0 else ""
        # position
        pn = P.Panel(cv, boxes[0][c], (0, 14), _lim(res.pos[:, c]),
                     title=f"{ax_name} 位置 (m)", ylabel=yl)
        pn.plot(t, decimate(res.pos[:, c], 5), P.COLORS[c], 3)
        pn.axhline(0, P.COLORS[6], 1, True)
        pn.render()
        # velocity
        vn = P.Panel(cv, boxes[1][c], (0, 14), _lim(res.vel[:, c]),
                     title=f"{ax_name} 方向速度 (m/s)", ylabel=("速度 (m/s)" if c == 0 else ""))
        vn.plot(t, decimate(res.vel[:, c], 5), P.COLORS[c], 3)
        vn.axhline(0, P.COLORS[6], 1, True)
        vn.render()
        # acceleration
        an = P.Panel(cv, boxes[2][c], (0, 14), _lim(res.acc[:, c]),
                     title=f"{ax_name} 方向加速度 (m/s²)", xlabel="时间 (s)", ylabel=("加速度 (m/s²)" if c == 0 else ""))
        an.plot(t, decimate(res.acc[:, c], 5), P.COLORS[c], 3)
        an.axhline(0, P.COLORS[6], 1, True)
        an.render()
    cv.text(850, 52, title, size=28, bold=True)
    cv.save(path)
    return path


def _lim(x, pad=0.08):
    lo = min(float(np.min(x)), 0.0)
    hi = max(float(np.max(x)), 0.0)
    span = hi - lo if hi > lo else 1.0
    return lo - pad * span, hi + pad * span


def fig_attitude(path, res_raw, res_flt):
    cv, boxes = P.grid_canvas(1660, 1150, 2, 2, left=125, right=55, top=80, bottom=110, hgap=140, vgap=125)
    t = decimate(res_raw.t, 5)
    p1 = P.Panel(cv, boxes[0][0], (0, 14), (-32, 32), title="滚转角指令",
                 ylabel="φ_d (°)", legend_loc="upper right")
    p1.plot(t, DEG(decimate(res_raw.roll_cmd, 5)), P.COLORS[0], 2, dash=True, label="未滤波")
    p1.plot(t, DEG(decimate(res_flt.roll_cmd, 5)), P.COLORS[2], 3, label="低通滤波")
    p1.axvline(T, P.COLORS[6], 1, True, "T")
    p1.render()

    p2 = P.Panel(cv, boxes[0][1], (0, 14), (-32, 32), title="俯仰角指令",
                 ylabel="θ_d (°)", legend_loc="upper right")
    p2.plot(t, DEG(decimate(res_raw.pitch_cmd, 5)), P.COLORS[0], 2, dash=True, label="未滤波")
    p2.plot(t, DEG(decimate(res_flt.pitch_cmd, 5)), P.COLORS[4], 3, label="低通滤波")
    p2.axvline(T, P.COLORS[6], 1, True, "T")
    p2.render()

    att_raw = run_attitude_loop(res_raw)
    att_flt = run_attitude_loop(res_flt)
    p3 = P.Panel(cv, boxes[1][0], (0, 14), (-8, 8), title="滚转角跟踪误差",
                 xlabel="时间 (s)", ylabel="e_φ (°)", legend_loc="upper right")
    p3.plot(t, DEG(decimate(att_raw["roll"]["err"], 5)), P.COLORS[0], 2, dash=True, label="未滤波")
    p3.plot(t, DEG(decimate(att_flt["roll"]["err"], 5)), P.COLORS[2], 3, label="低通滤波")
    p3.axhline(0, P.COLORS[6], 1, True)
    p3.render()

    p4 = P.Panel(cv, boxes[1][1], (0, 14), (-8, 8), title="俯仰角跟踪误差",
                 xlabel="时间 (s)", ylabel="e_θ (°)", legend_loc="upper right")
    p4.plot(t, DEG(decimate(att_raw["pitch"]["err"], 5)), P.COLORS[0], 2, dash=True, label="未滤波")
    p4.plot(t, DEG(decimate(att_flt["pitch"]["err"], 5)), P.COLORS[4], 3, label="低通滤波")
    p4.axhline(0, P.COLORS[6], 1, True)
    p4.render()
    cv.save(path)
    return path


def fig_trajectory(path, res):
    cv, boxes = P.grid_canvas(1420, 640, 1, 2, left=150, right=60, top=80, bottom=110, hgap=130)
    xs, ys, zs = res.pos[:, 0], res.pos[:, 1], res.pos[:, 2]
    left = P.Panel(cv, boxes[0][0], (-1.0, 6.0), (-4.0, 1.0),
                   title="水平面轨迹 (x–y)", xlabel="x (m)", ylabel="y (m)")
    left.plot(xs, ys, P.COLORS[2], 4)
    left.plot([xs[0]], [ys[0]], None, 0, label=None)
    left.render()
    left.cv.circle((left.px(xs[0]), left.py(ys[0])), 8, fill=(255, 255, 255), outline=P.COLORS[1], width=3)
    left.cv.circle((left.px(xs[-1]), left.py(ys[-1])), 8, fill=(255, 255, 255), outline=P.COLORS[3], width=3)
    left.cv.circle((left.px(0), left.py(0)), 10, fill=(255, 255, 255), outline=P.COLORS[0], width=3)
    left.cv.text(left.px(xs[0]) + 14, left.py(ys[0]) - 18, "起点", size=18, anchor="lm")
    left.cv.text(left.px(0) - 14, left.py(0) - 20, "目标", size=18, anchor="rm")
    left.cv.text(left.px(xs[-1]) + 18, left.py(ys[-1]) + 44, "终点（与目标重合）", size=15, anchor="lm")

    t = decimate(res.t, 5)
    right = P.Panel(cv, boxes[0][1], (0, 14), (-0.5, 10.5),
                    title="高度随时间变化", xlabel="时间 (s)", ylabel="z (m)")
    right.plot(t, decimate(zs, 5), P.COLORS[2], 4)
    right.axvline(res.meta["T"], P.COLORS[6], 1, True, "T = 9 s")
    right.render()
    cv.save(path)
    return path


def fig_compare(path, res_smc, res_full):
    cv, boxes = P.grid_canvas(1660, 1080, 2, 2, left=125, right=55, top=80, bottom=110, hgap=140, vgap=125)
    t = decimate(res_smc.t, 5)
    e_smc = np.abs(res_smc.err).max(axis=1)
    e_full = np.abs(res_full.err).max(axis=1)
    p1 = P.Panel(cv, boxes[0][0], (0, 14), (-6.2, 1.4), title="位置误差范数（对数坐标）",
                 ylabel="|e|max (m)", legend_loc="upper right",
                 fmt_y=lambda v: ("10^%d" % round(v)) if abs(v - round(v)) < 1e-6 else "")
    p1.plot(t, np.log10(np.maximum(decimate(e_smc, 5), 1e-6)), P.COLORS[0], 3, label="纯滑模")
    p1.plot(t, np.log10(np.maximum(decimate(e_full, 5), 1e-6)), P.COLORS[2], 3, label="滑模→PID 分段")
    p1.axvline(T, P.COLORS[6], 1, True, "T = 9 s")
    p1.axvline(T_SWITCH, P.COLORS[5], 1, True, "t_s")
    p1.yticks = [-6, -4, -2, 0]
    p1.render()

    p2 = P.Panel(cv, boxes[0][1], (0, 12), (-1.2, 1.2), title="X 轴滑模面 s_x",
                 ylabel="s_x (m/s)", legend_loc="upper right")
    p2.plot(t, decimate(res_smc.s[:, 0], 5), P.COLORS[0], 2, label="纯滑模")
    p2.plot(t, decimate(res_full.s[:, 0], 5), P.COLORS[2], 2, label="分段")
    p2.axhline(0, P.COLORS[6], 1, True)
    p2.render()

    p3 = P.Panel(cv, boxes[1][0], (0, 12), (-8, 8), title="X 轴控制力 u_x",
                 xlabel="时间 (s)", ylabel="u_x (N)", legend_loc="upper right")
    p3.plot(t, decimate(res_smc.u[:, 0], 5), P.COLORS[0], 2, label="纯滑模")
    p3.plot(t, decimate(res_full.u[:, 0], 5), P.COLORS[2], 2, label="分段")
    p3.render()

    p4 = P.Panel(cv, boxes[1][1], (0, 12), (-32, 32), title="滚转角指令",
                 xlabel="时间 (s)", ylabel="φ_d (°)", legend_loc="upper right")
    p4.plot(t, DEG(decimate(res_smc.roll_cmd, 5)), P.COLORS[0], 2, label="纯滑模")
    p4.plot(t, DEG(decimate(res_full.roll_cmd, 5)), P.COLORS[2], 2, label="分段")
    p4.render()
    cv.save(path)
    return path


def fig_force(path, res):
    cv, boxes = P.grid_canvas(1660, 560, 1, 2, left=125, right=55, top=80, bottom=110, hgap=140)
    t = decimate(res.t, 5)
    p1 = P.Panel(cv, boxes[0][0], (0, 14), (-8, 12), title="外环控制力",
                 xlabel="时间 (s)", ylabel="u (N)", legend_loc="upper right")
    for j, nm in enumerate(["u_x", "u_y", "u_z"]):
        p1.plot(t, decimate(res.u[:, j], 5), P.COLORS[j], 2, label=nm)
    p1.axhline(0, P.COLORS[6], 1, True)
    p1.render()
    p2 = P.Panel(cv, boxes[0][1], (0, 14), (0, 14), title="总推力指令",
                 xlabel="时间 (s)", ylabel="T_c (N)", legend_loc="upper right")
    p2.plot(t, decimate(res.thrust, 5), P.COLORS[2], 3, label="推力指令")
    p2.axhline(P_MAIN.m * P_MAIN.g, P.COLORS[0], 1, True, "mg")
    p2.render()
    cv.save(path)
    return path


def main():
    os.makedirs(OUT, exist_ok=True)
    res_smc = run_position_loop((0, 0, 0), P_MAIN)
    res_flt = run_position_loop((0, 0, 0), P_MAIN, angle_filter_hz=5.0)
    res_full = run_position_loop((0, 0, 0), P_MAIN, switch_time=T_SWITCH, angle_filter_hz=5.0)
    jobs = [
        ("fig3_states.png", lambda p: fig_states(p, res_smc, "预定时间滑模控制位置响应（T = 9 s）")),
        ("fig4_attitude.png", lambda p: fig_attitude(p, res_smc, res_flt)),
        ("fig5_trajectory.png", lambda p: fig_trajectory(p, res_smc)),
        ("fig6_compare.png", lambda p: fig_compare(p, res_smc, res_full)),
        ("fig7_force.png", lambda p: fig_force(p, res_smc)),
    ]
    for name, fn in jobs:
        path = os.path.join(OUT, name)
        fn(path)
        print("wrote", path)


if __name__ == "__main__":
    main()

