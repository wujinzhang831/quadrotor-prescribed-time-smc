# -*- coding: utf-8 -*-
"""Run the four reported configurations and export tidy CSV + metrics.

Outputs (relative to the repository root):
    data/processed/<case>.csv     time series, 100 Hz, header in English + unit
    data/processed/metrics.csv    one row per configuration
    data/processed/metrics.json   same numbers, machine readable
"""
from __future__ import annotations

import csv
import json
import math
import os
import sys

import numpy as np

import qsmc_sim as Q
from qsmc_sim import Params

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OUT_CSV = os.path.join(ROOT, "data", "processed")

CASES = {
    "A_smc": dict(switch_time=None, angle_filter_hz=None),
    "B_smc_filter": dict(switch_time=None, angle_filter_hz=5.0),
    "C_smc_pid": dict(switch_time=None, angle_filter_hz=None, use_switch=True),
    "D_full": dict(switch_time=None, angle_filter_hz=5.0, use_switch=True),
}

DESC = {
    "A_smc": "预定时间滑模（基准）",
    "B_smc_filter": "滑模 + 姿态指令低通滤波",
    "C_smc_pid": "滑模 → PID 分段切换",
    "D_full": "滤波 + 分段切换（推荐）",
}

HEADER = ["t_s", "x_m", "y_m", "z_m", "vx_mps", "vy_mps", "vz_mps",
          "ax_mps2", "ay_mps2", "az_mps2", "ex_m", "ey_m", "ez_m",
          "ux_N", "uy_N", "uz_N", "sx_mps", "sy_mps", "sz_mps",
          "roll_cmd_deg", "pitch_cmd_deg", "thrust_cmd_N",
          "roll_actual_deg", "pitch_actual_deg", "mode_smc1_pid2"]


def run_case(name, cfg, p):
    T = p.T
    t_sw = T - 0.495 * math.exp(9 - T) - 0.405
    use_switch = cfg.pop("use_switch", False)
    kw = dict(cfg)
    if use_switch:
        kw["switch_time"] = t_sw
    res = Q.run_position_loop((0.0, 0.0, 0.0), p, **kw)
    att = Q.run_attitude_loop(res, p)
    return res, att, t_sw if use_switch else None


def metrics_row(name, res, att, t_sw, p):
    m = Q.metrics(res, thr=0.01)
    m5 = Q.metrics(res, thr=0.05)
    rms = lambda a: float(np.sqrt(np.mean(np.asarray(a) ** 2)))
    att_rms = math.degrees(max(rms(att["roll"]["err"]), rms(att["pitch"]["err"])))
    rate = math.degrees(np.abs(np.diff(res.roll_cmd)).max() / res.meta["dt"])
    return {
        "case": name,
        "description": DESC[name],
        "prescribed_time_s": m["prescribed_time"],
        "settle_1cm_s": m["settling_time"],
        "settle_5cm_s": m5["settling_time"],
        "max_abs_err_m": max(m["max_abs_err"]),
        "final_err_norm_m": float(np.abs(res.err[-1]).max()),
        "rms_err_tail_m": float(np.sqrt(np.mean(np.sum(res.err[int(0.8 * res.t.size):] ** 2, axis=1)))),
        "max_force_N": max(m["max_force"]),
        "att_rms_deg": att_rms,
        "max_att_cmd_deg": math.degrees(np.abs(np.r_[res.roll_cmd, res.pitch_cmd]).max()),
        "cmd_rate_deg_s": rate,
        "switch_time_s": t_sw if t_sw else "",
    }


def write_csv(path, res, att):
    step = int(round(0.01 / res.meta["dt"]))
    idx = list(range(0, res.t.size, step))
    with open(path, "w", newline="", encoding="utf-8") as fh:
        w = csv.writer(fh)
        w.writerow(HEADER)
        for k in idx:
            w.writerow([
                f"{res.t[k]:.3f}",
                *(f"{res.pos[k, j]:.6f}" for j in range(3)),
                *(f"{res.vel[k, j]:.6f}" for j in range(3)),
                *(f"{res.acc[k, j]:.6f}" for j in range(3)),
                *(f"{res.err[k, j]:.6f}" for j in range(3)),
                *(f"{res.u[k, j]:.6f}" for j in range(3)),
                *(f"{res.s[k, j]:.6f}" for j in range(3)),
                f"{math.degrees(res.roll_cmd[k]):.4f}",
                f"{math.degrees(res.pitch_cmd[k]):.4f}",
                f"{res.thrust[k]:.4f}",
                f"{math.degrees(att['roll']['actual'][k]):.4f}",
                f"{math.degrees(att['pitch']['actual'][k]):.4f}",
                f"{res.mode[k] + 1}",
            ])


def main():
    os.makedirs(OUT_CSV, exist_ok=True)
    p = Params()
    rows = []
    for name, cfg in CASES.items():
        res, att, t_sw = run_case(name, dict(cfg), p)
        write_csv(os.path.join(OUT_CSV, name + ".csv"), res, att)
        rows.append(metrics_row(name, res, att, t_sw, p))
        print(name, "done")
    keys = list(rows[0].keys())
    with open(os.path.join(OUT_CSV, "metrics.csv"), "w", newline="", encoding="utf-8") as fh:
        w = csv.DictWriter(fh, fieldnames=keys)
        w.writeheader()
        w.writerows(rows)
    with open(os.path.join(OUT_CSV, "metrics.json"), "w", encoding="utf-8") as fh:
        json.dump({"parameters": Q.Params().__dict__, "cases": rows}, fh,
                  ensure_ascii=False, indent=2)
    for r in rows:
        print(r["case"], "t1cm=%.3f" % r["settle_1cm_s"], "t5cm=%.3f" % r["settle_5cm_s"],
              "att_rms=%.3f" % r["att_rms_deg"], "rate=%.0f" % r["cmd_rate_deg_s"])


if __name__ == "__main__":
    main()
