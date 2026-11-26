# -*- coding: utf-8 -*-
"""Figure 1 (flow chart) and figure 2 (control architecture) for the report."""
from __future__ import annotations

import os

import plotlib as P

INK = P.INK


def flow(path):
    cv = P.Canvas(1400, 1620)
    d = cv.draw
    cx = 470
    bw, bh = 620, 86
    seq = [
        ("S1  参数装订：m, g, k, T, h = T/3,\nα, β, ρ, φ, 采样步长 Δt", None),
        ("S2  采样位置/速度，构造误差\ne1 = p − p_d，e2 = v", None),
        ("S3  是否到达切换时刻 t ≥ t_s ?", "decision"),
        ("S4  计算核函数 R1(t)、增益 K(t) 及其导数", None),
        ("S5  构造滑模面\ns = e2 − a·e1 + K(t)·e1(t − h)", None),
        ("S6  计算三轴等效控制力 u", None),
        ("S7  力/力矩分配：解算姿态指令\n(φ_d, θ_d) 与总推力 T_c", None),
        ("S8  姿态指令一阶低通滤波（f_c = 5 Hz）", None),
        ("S9  内环 PID 姿态跟踪（输出力矩 τ），\n积分四旋翼位置/姿态动力学", None),
        ("S10  记录状态与控制量，k ← k+1，返回 S2", None),
    ]
    positions = []
    y = 26
    for label, kind in seq:
        nl = label.count("\n")
        h = 96 if kind == "decision" else bh + (28 * nl)
        positions.append((y, h, label, kind))
        y += h + 46
    total = y + 10
    shift = max(0, (1620 - total) // 2)

    boxes = []
    for (yy, h, label, kind) in positions:
        boxes.append((yy + shift, h, label, kind))

    for i, (yy, h, label, kind) in enumerate(boxes):
        if kind == "decision":
            box = (cx - bw / 2 - 30, yy, cx + bw / 2 + 30, yy + h)
            cxm, cym = (box[0] + box[2]) / 2, (box[1] + box[3]) / 2
            d.polygon([(cxm, box[1]), (box[2], cym), (cxm, box[3]), (box[0], cym)],
                      fill=(252, 246, 226), outline=INK, width=3)
        else:
            box = (cx - bw / 2, yy, cx + bw / 2, yy + h)
            cv.rect(box, fill=(238, 244, 252), outline=INK, width=3, radius=10)
        cv.text(cx, yy + h / 2, label, size=22)
        if i > 0:
            prev = boxes[i - 1]
            cv.arrow((cx, prev[0] + prev[1]), (cx, yy - 4))

    # side branch: PID position hold
    dec = boxes[2]
    sx0 = cx + bw / 2 + 130
    sbox = (sx0, dec[0] - 26, sx0 + 330, dec[0] + dec[1] + 26)
    cv.rect(sbox, fill=(238, 250, 242), outline=INK, width=3, radius=10)
    cv.text((sbox[0] + sbox[2]) / 2, (sbox[1] + sbox[3]) / 2,
            "PID 位置保持\nKp = 64.5, Ki = 20\nKd = 30 / 35", size=20)
    cv.arrow((cx + bw / 2 + 34, dec[0] + dec[1] / 2), (sbox[0] - 4, dec[0] + dec[1] / 2))
    cv.text(cx + bw / 2 + 44, dec[0] + dec[1] / 2 - 28, "是", size=20, anchor="lm")
    cv.text(cx + 30, dec[0] + dec[1] + 6, "否", size=20, anchor="lt")

    s6 = boxes[5]
    mid_y = s6[0] + s6[1] / 2
    xr = sbox[2]
    d.line([(xr, (sbox[1] + sbox[3]) / 2), (xr + 40, (sbox[1] + sbox[3]) / 2)], fill=INK, width=3)
    d.line([(xr + 40, (sbox[1] + sbox[3]) / 2), (xr + 40, mid_y)], fill=INK, width=3)
    cv.arrow((xr + 40, mid_y), (cx + bw / 2 + 6, mid_y))
    cv.text(xr + 52, mid_y - 16, "u", size=20, anchor="lb")
    cv.save(path)
    return path


def architecture(path):
    cv = P.Canvas(1460, 820)
    d = cv.draw

    def box(x, y, w, h, text, fill=(238, 244, 252), size=21):
        cv.rect((x, y, x + w, y + h), fill=fill, outline=INK, width=3, radius=12)
        cv.text(x + w / 2, y + h / 2, text, size=size)
        return (x, y, x + w, y + h)

    b1 = box(40, 90, 200, 120, "期望位置\np_d", (252, 246, 226))
    b2 = box(320, 70, 250, 160, "外环\n预定时间滑模\n位置控制器", (232, 242, 252))
    b3 = box(700, 70, 240, 160, "力/力矩分配\n姿态指令解算", (232, 242, 252))
    b4 = box(1000, 70, 190, 160, "一阶低通\n滤波\nf_c = 5 Hz", (246, 240, 252))
    b5 = box(1240, 70, 190, 160, "内环\nPID 姿态\n控制器", (232, 242, 252))
    b6 = box(1120, 380, 240, 150, "四旋翼\n刚体动力学", (236, 250, 240))
    b7 = box(560, 380, 240, 130, "位置/姿态\n状态测量", (236, 250, 240))

    cv.arrow((b1[2], 150), (b2[0] - 4, 150))
    cv.arrow((b2[2], 150), (b3[0] - 4, 150))
    cv.text(636, 116, "φ_d, θ_d, T_c", size=17)
    cv.arrow((b3[2], 150), (b4[0] - 4, 150))
    cv.arrow((b4[2], 150), (b5[0] - 4, 150))
    cv.arrow((b5[0] + 95, b5[3]), (b5[0] + 95, b6[1] - 4))
    cv.text(1348, 300, "τ", size=21, anchor="lm")
    cv.arrow((b6[0] - 4, 455), (b7[2] + 4, 455))
    cv.text(890, 428, "p, v, ω", size=19, anchor="lm")
    # feedback path: measurement -> outer loop
    d.line([(b7[0] + 120, b7[1]), (b7[0] + 120, 270)], fill=INK, width=3)
    d.line([(b7[0] + 120, 270), (b2[0] + 60, 270)], fill=INK, width=3)
    cv.arrow((b2[0] + 60, 270), (b2[0] + 60, b2[3] + 4))
    cv.text(470, 268, "状态反馈", size=18, anchor="lb")
    # disturbance enters the rigid-body dynamics
    cv.arrow((1240, 690), (1240, b6[3] + 4), (200, 60, 40), 3)
    cv.text(1256, 650, "风扰与模型摄动 d(t)\n|d| ≤ 0.5 N", size=19, anchor="lm")
    cv.save(path)
    return path


if __name__ == "__main__":
    here = os.path.dirname(os.path.abspath(__file__))
    out = os.path.join(os.path.dirname(here) if os.path.basename(here) == "tools" else here, "figures")
    os.makedirs(out, exist_ok=True)
    flow(os.path.join(out, "fig1_flowchart.png"))
    architecture(os.path.join(out, "fig2_architecture.png"))
    print("ok")
