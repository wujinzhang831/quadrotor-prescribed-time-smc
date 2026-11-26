# -*- coding: utf-8 -*-
"""Reference implementation of the prescribed-time sliding-mode position loop
plus inner attitude PID loop for a quadrotor (Python port of the MATLAB sources).

This module is the numerical core behind ``tools/export_results.py`` and
``tools/make_figures.py`` in the open-source package.  It mirrors
``src/quadrotor_smc.m`` (outer loop), ``src/PID_smc.m`` (inner loop) and the
position dynamics of ``scripts/run_dual_loop.m``.
"""
from __future__ import annotations

import json
import math
from dataclasses import dataclass, field, asdict

import numpy as np


# --------------------------------------------------------------------------
# vehicle / environment parameters (identical to the MATLAB sources)
# --------------------------------------------------------------------------
@dataclass
class Params:
    m: float = 0.905          # mass [kg]
    g: float = 9.8            # gravity [m/s^2]
    kx: float = 0.1           # drag coefficient x [N s/m]
    ky: float = 0.1
    kz: float = 0.1
    # initial state
    x0: float = 5.0
    y0: float = -3.0
    z0: float = 10.0
    vx0: float = 0.0
    vy0: float = 0.0
    vz0: float = 0.0
    # disturbance
    d_max: float = 0.5
    freq_x: float = 0.5
    freq_y: float = 0.6
    freq_z: float = 0.7
    # prescribed-time controller
    a: float = -0.7916        # sliding-surface delay coefficient
    alpha: float = 0.9        # alpha in (0.5, 1)
    beta: float = 1.2         # beta > 1
    phi: float = 1e-3         # boundary layer
    R_type: int = 1           # 1 = sin^2 kernel
    T: float = 9.0            # prescribed convergence time [s]
    dt: float = 1e-3
    t_end_factor: float = 2.0
    # geometric gains from the original script
    c_scale: float = 10.0     # c = c_scale / T   (xyz.m uses 300/T)
    d_scale: float = 10.0
    # inner attitude loop
    Ixx: float = 0.0081
    Iyy: float = 0.0081
    Izz: float = 0.0142
    B_roll: float = 0.05
    B_pitch: float = 0.05
    B_yaw: float = 0.03
    Kp_roll: float = 5000.0
    Ki_roll: float = 10.0
    Kd_roll: float = 150.0
    Kp_pitch: float = 5000.0
    Ki_pitch: float = 1.0
    Kd_pitch: float = 150.0
    Kp_yaw: float = 8.0
    Ki_yaw: float = 2.0
    Kd_yaw: float = 1.5
    max_torque: float = 0.5
    # position PID used after the hand-over
    Kp_p: float = 64.5
    Ki_p: float = 20.0
    Kd_x: float = 30.0
    Kd_y: float = 30.0
    Kd_z: float = 35.0
    max_force: float = 30.0


def sat(x, phi):
    return max(min(x / phi, 1.0), -1.0)


def sig(s, gamma):
    return math.copysign(abs(s) ** gamma, s)


# --------------------------------------------------------------------------
# prescribed-time kernel functions (Table 2 of the reference note)
# --------------------------------------------------------------------------
def R_h(t, h, R_type=1):
    tm = math.fmod(t, 2 * h)
    if tm < 0:
        tm += 2 * h
    if tm < h:
        return 0.0
    if R_type == 1:
        return math.sin(math.pi * (tm - h) / h) ** 2
    if R_type == 2:
        return 16.0 * (tm - h) ** 2 * (2 * h - tm) ** 2 / h ** 4
    return 64.0 * (tm - h) ** 3 * (2 * h - tm) ** 3 / h ** 6


def K_at(t, h, a, R_type=1):
    if t < h:
        return 0.0
    tau = t - h
    tm = math.fmod(tau, 2 * h)
    if tm < 0:
        tm += 2 * h
    if tm < h:
        return 0.0
    if a == 0.0:
        return (2.0 / h) * R_h(t, h, R_type)
    return (2.0 / h) * R_h(t, h, R_type) * math.exp(-2.0 * a * (t - h))


def K_func(t, h, a, R_type=1):
    """Return K(t) and its central-difference derivative."""
    K = K_at(t, h, a, R_type)
    eps = 1e-6
    Kp = K_at(t + eps, h, a, R_type)
    Km = K_at(t - eps, h, a, R_type)
    return K, (Kp - Km) / (2 * eps)


def disturbance(t, p: Params):
    dx = p.d_max * math.sin(2 * math.pi * p.freq_x * t)
    dy = p.d_max * math.cos(2 * math.pi * p.freq_y * t)
    dz = p.d_max * math.sin(2 * math.pi * p.freq_z * t + math.pi / 4)
    return dx, dy, dz


# --------------------------------------------------------------------------
# outer loop: prescribed-time sliding-mode controller
# --------------------------------------------------------------------------
def smc_force(e1, e2, e1_h, e2_h, K, Kd, axis, p: Params):
    """Return the equivalent control force u [N] for one translational axis."""
    h = p.T / 3.0
    a = p.a
    c = p.c_scale / p.T
    d = p.d_scale / p.T
    s = e2 - a * e1 + K * e1_h
    rho = p.d_max / p.m + 0.1
    g = 1.0 / p.m
    f = -(getattr(p, "k" + axis) / p.m) * e2
    if axis == "z":
        f -= p.g
    u = (1.0 / g) * (
        -c / 2 ** p.alpha * sig(s, 2 * p.alpha - 1)
        - d / 2 ** p.beta * sig(s, 2 * p.beta - 1)
        - rho * sat(s, p.phi)
        - f
        + a * e2
        - K * e2_h
        - Kd * e1_h
    )
    return u, s


def control_to_angles(ux, uy, uz, p: Params):
    """Map horizontal/vertical force commands to attitude + collective thrust.

    The original MATLAB helper evaluated ``f = m*(g - uz/m)``, i.e. it mapped the
    vertical force command with the wrong sign and produced zero collective thrust
    at hover.  The corrected allocation below uses ``T_c = u_z`` so that
    ``a_z = T_c/m - g`` and ``a_x = (T_c/m)·sinθ``, ``a_y = (T_c/m)·sinφ``.
    """
    thrust = max(uz, 1.0)                # collective thrust command [N]
    sphi = max(min(uy / thrust, 1.0), -1.0)
    phi = math.asin(sphi)
    cphi = math.cos(phi)
    if abs(cphi) < 1e-6:
        theta = 0.0
    else:
        sth = max(min(ux / (thrust * cphi), 1.0), -1.0)
        theta = math.asin(sth)
    lim = 30 * math.pi / 180
    return max(min(phi, lim), -lim), max(min(theta, lim), -lim), thrust


@dataclass
class SimResult:
    t: np.ndarray
    pos: np.ndarray            # (N,3)
    vel: np.ndarray
    acc: np.ndarray
    err: np.ndarray
    u: np.ndarray              # (N,3) force commands
    s: np.ndarray              # (N,3) sliding variables
    roll_cmd: np.ndarray
    pitch_cmd: np.ndarray
    yaw_cmd: np.ndarray
    thrust: np.ndarray
    mode: np.ndarray = field(default=None)   # 0 = SMC, 1 = PID
    meta: dict = field(default_factory=dict)


def run_position_loop(desired=(0.0, 0.0, 0.0), p: Params | None = None,
                      switch_time: float | None = None,
                      angle_filter_hz: float | None = None):
    """Simulate the dual-loop position channel.

    switch_time : if given, hand the position loop over to a PID controller
                  at that instant (segmented strategy).
    angle_filter_hz : if given, the attitude command is passed through a
                  first-order low-pass filter with this cut-off frequency.
    """
    p = p or Params()
    dt = p.dt
    T = p.T
    h = T / 3.0
    t = np.arange(0.0, T * p.t_end_factor + 1e-12, dt)
    N = t.size

    pos = np.zeros((N, 3))
    vel = np.zeros((N, 3))
    acc = np.zeros((N, 3))
    err = np.zeros((N, 3))
    u = np.zeros((N, 3))
    s = np.zeros((N, 3))
    roll = np.zeros(N)
    pitch = np.zeros(N)
    thrust = np.zeros(N)
    mode = np.zeros(N, dtype=int)

    pos[0] = [p.x0, p.y0, p.z0]
    err[0] = pos[0] - np.asarray(desired)
    vel[0] = [p.vx0, p.vy0, p.vz0]
    prev = err[0].copy()

    lag = int(round(h / dt))
    alpha_f = None
    if angle_filter_hz:
        tau = 1.0 / (2 * math.pi * angle_filter_hz)
        alpha_f = math.exp(-dt / tau)

    integ = np.zeros(3)
    kvec = np.array([p.kx, p.ky, p.kz])
    axes = ("x", "y", "z")

    for k in range(N - 1):
        tk = t[k]
        e1 = err[k]
        e2 = vel[k]
        # delayed states (zero-order hold buffer, identical to the MATLAB code)
        idx_h = k - lag
        e1_h = err[idx_h] if idx_h >= 0 else err[0]
        e2_h = vel[idx_h] if idx_h >= 0 else vel[0]

        if switch_time is not None and tk >= switch_time:
            mode[k] = 1
            # PID position controller (low.m / compute_pid_u)
            ep = np.asarray(desired) - pos[k]
            integ += ep * dt
            for j in range(3):
                kd = (p.Kd_x, p.Kd_y, p.Kd_z)[j]
                u[k, j] = (p.Kp_p * ep[j] + p.Ki_p * integ[j] - kd * vel[k, j])
                if j == 2:
                    u[k, j] += p.m * p.g
            u[k] = np.clip(u[k], -p.max_force, p.max_force)
            s[k] = s[k - 1]
        else:
            for j, ax_name in enumerate(axes):
                K, Kd = K_func(tk, h, p.a, p.R_type)
                u[k, j], s[k, j] = smc_force(e1[j], e2[j], e1_h[j], e2_h[j],
                                             K, Kd, ax_name, p)

        phi_c, theta_c, f_c = control_to_angles(u[k, 0], u[k, 1], u[k, 2], p)
        thrust[k] = f_c
        if alpha_f is None:
            roll[k], pitch[k] = phi_c, theta_c
        else:
            j0 = max(k - 1, 0)
            roll[k] = alpha_f * roll[j0] + (1 - alpha_f) * phi_c
            pitch[k] = alpha_f * pitch[j0] + (1 - alpha_f) * theta_c

        dx, dy, dz = disturbance(tk, p)
        dvec = np.array([dx, dy, dz]) / p.m
        a_vec = u[k] / p.m - kvec / p.m * vel[k] + dvec
        a_vec[2] -= p.g
        acc[k] = a_vec
        vel[k + 1] = vel[k] + dt * a_vec
        pos[k + 1] = pos[k] + dt * vel[k + 1]
        err[k + 1] = pos[k + 1] - np.asarray(desired)

    # last sample
    acc[N - 1] = acc[N - 2]
    roll[N - 1], pitch[N - 1] = roll[N - 2], pitch[N - 2]
    thrust[N - 1] = thrust[N - 2]
    mode[N - 1] = mode[N - 2]
    s[N - 1] = s[N - 2]
    u[N - 1] = u[N - 2]

    meta = dict(desired=list(map(float, desired)), T=T, dt=dt,
                switch_time=switch_time, filter_hz=angle_filter_hz,
                c=p.c_scale / T, d=p.d_scale / T, h=h,
                alpha=p.alpha, beta=p.beta, a=p.a, phi=p.phi, R_type=p.R_type)
    return SimResult(t=t, pos=pos, vel=vel, acc=acc, err=err, u=u, s=s,
                     roll_cmd=roll, pitch_cmd=pitch, yaw_cmd=np.zeros(N),
                     thrust=thrust, mode=mode, meta=meta)


# --------------------------------------------------------------------------
# inner loop: attitude PID following the command profile
# --------------------------------------------------------------------------
def run_attitude_loop(res: SimResult, p: Params | None = None):
    p = p or Params()
    dt = p.dt
    N = res.t.size
    out = {}
    for name, cmd, I, B, kp, ki, kd in (
        ("roll", res.roll_cmd, p.Ixx, p.B_roll, p.Kp_roll, p.Ki_roll, p.Kd_roll),
        ("pitch", res.pitch_cmd, p.Iyy, p.B_pitch, p.Kp_pitch, p.Ki_pitch, p.Kd_pitch),
        ("yaw", res.yaw_cmd, p.Izz, p.B_yaw, p.Kp_yaw, p.Ki_yaw, p.Kd_yaw),
    ):
        ang = np.zeros(N)
        rate = np.zeros(N)
        tau = np.zeros(N)
        integ = 0.0
        d_filt = 0.0
        for k in range(N - 1):
            e = cmd[k] - ang[k]
            integ += e * dt
            integ = max(min(integ, 2.0 / ki), -2.0 / ki)
            if k > 0:
                raw = (e - (cmd[k - 1] - ang[k - 1])) / dt
                d_filt = 0.1 * raw + 0.9 * d_filt
            tau[k] = max(min(kp * e + ki * integ + kd * d_filt, p.max_torque), -p.max_torque)
            rate[k + 1] = rate[k] + dt * (tau[k] - B * rate[k]) / I
            ang[k + 1] = ang[k] + dt * rate[k]
        tau[N - 1] = tau[N - 2]
        out[name] = dict(cmd=cmd, actual=ang, rate=rate, torque=tau,
                         err=cmd - ang)
    return out


# --------------------------------------------------------------------------
# metrics used by the report table
# --------------------------------------------------------------------------
def metrics(res: SimResult, thr=0.01):
    m = {}
    e = res.err
    norm = np.abs(e).max(axis=1)
    conv = np.where(norm < thr)[0]
    m["prescribed_time"] = float(res.meta["T"])
    m["settling_time"] = float(res.t[conv[0]]) if conv.size else float("nan")
    m["max_abs_err"] = [float(np.abs(e[:, j]).max()) for j in range(3)]
    m["final_abs_err"] = [float(abs(e[-1, j])) for j in range(3)]
    # steady-state window: last 20 % of the horizon
    tail = int(0.8 * e.shape[0])
    m["rms_err_tail"] = [float(np.sqrt(np.mean(e[tail:, j] ** 2))) for j in range(3)]
    m["max_force"] = [float(np.abs(res.u[:, j]).max()) for j in range(3)]
    m["max_angle_cmd_deg"] = float(np.degrees(np.abs(np.r_[res.roll_cmd, res.pitch_cmd]).max()))
    m["angle_cmd_rate_deg_s"] = float(np.degrees(np.abs(np.diff(res.roll_cmd)).max() / res.meta["dt"]))
    return m


if __name__ == "__main__":
    base = Params()
    r = run_position_loop((0, 0, 0), base)
    print(json.dumps(metrics(r), indent=2, ensure_ascii=False))
