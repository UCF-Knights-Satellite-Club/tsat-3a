"""Three-axis attitude model of the KAOS-3 payload hanging under a balloon line.

The payload is a rigid body (6 DOF) hung from an elastic line attached at an eye a distance d above the
centre of mass. Three PCB reaction wheels on the body axes, a two-axis trim stage that moves the battery
pack sideways, a Mahony attitude estimator (gyro + accelerometer) and a 100 Hz PID loop. Vectorized over
cases: every array has a leading case axis N. Deterministic (seeded).

Frames: world z up; body frame with z up when level. Quaternions are scalar-first, body to world.

What the model includes:
- Line: spring along the line (2 Hz bounce, ASSUMED), tension only, from a top point that can move
  (balloon or rig). The line passes through the eye; no line torsion (a swivel at the eye).
- Gravity torque: the line pulls at the eye, so a tilt away from the line direction gives a restoring
  torque M g d sin(angle). With the body held level and the line swinging by phi the wheels must supply
  M g d sin(phi).
- Wheels: torque constant, phase resistance, driver current limit, back-EMF voltage limit at 11.1 V,
  6000 rpm cap, eddy drag (from the stackup study) and an ASSUMED 0.1 mN·m bearing drag. Gyroscopic
  coupling with stored wheel momentum (omega x h) is included.
- Trim stage: the battery pack (ASSUMED 0.3 kg) moves up to ±15 mm in body x and y at 2 mm/s (lead
  screw, ASSUMED). Moving it shifts the centre of mass sideways, which sets the tilt the payload hangs at.
  With wheels, the trim stage unloads the pitch and roll wheels (second-order momentum loop); without
  wheels it levels the payload from the estimated tilt.
- Estimator: Mahony complementary filter with gyro-bias integral (30 s accelerometer time constant),
  starting with the bias estimate converged to 0.005 °/s (ground alignment before release). The
  accelerometer sees the specific force, which during a swing points along the line, not up; the slow
  gain keeps that error small. Heading is integrated gyro only (no magnetometer), so a tilted target's
  azimuth drifts slowly.
- Body rocking about the eye has ASSUMED 2% damping (air and structure); the line swing 0.5%.
- Yaw: a gust torque about the vertical (ASSUMED 1 mN·m RMS at 0.3 Hz) and wheel unloading by walking
  the heading reference, as in yaw/sim.py.
Not modelled: line torsion and swivel friction, aerodynamic torque on pitch and roll, structural modes,
IMU offset from the centre of mass, sensor misalignment.
"""
from __future__ import annotations

import json
import math
import pathlib
from dataclasses import dataclass

import numpy as np

OUT = pathlib.Path(__file__).resolve().parents[1] / "results"
DEG = math.pi / 180
G = 9.81

# ------------------------------------------------------------------ payload (ASSUMED unless noted)
M = 2.6                    # kg: the D&C mass limit (S11)
BOX = (0.20, 0.20, 0.11)   # m: KAOS flat 4U box
I_BODY = np.array([M / 12 * (BOX[1] ** 2 + BOX[2] ** 2), M / 12 * (BOX[0] ** 2 + BOX[2] ** 2),
                   M / 12 * (BOX[0] ** 2 + BOX[1] ** 2)])          # uniform box: 0.0113, 0.0113, 0.0173
D_NOM = 0.004              # m: eye above the centre of mass, design value (S8)
M_TRIM = 0.30              # kg: battery pack on the trim stage
TRIM_TRAVEL = 0.015        # m each way (S9)
TRIM_SPEED = 0.002         # m/s
TRIM_WN, TRIM_ZETA = 0.1, 0.9     # momentum-unloading loop (rad/s)
TRIM_TORQUE_LP = 5.0       # s: low-pass on motor torque in the unloading loop (rejects swing)
TRIM_ONLY_TAU = 10.0       # s: trim-only levelling time constant

# ------------------------------------------------------------------ line and environment
LINE_BOUNCE_HZ, LINE_ZETA = 2.0, 0.1
SWING_ZETA = 0.005         # aerodynamic damping of the pendulum swing
ROCK_ZETA = 0.02           # damping of the body rocking about the eye (air + structure, ASSUMED)
YAW_GUST_NM, YAW_GUST_FC = 1.0e-3, 0.3

# ------------------------------------------------------------------ sensors, estimator, control
DT = 0.0025
CTRL_EVERY = 4             # 100 Hz (S10)
GYRO_NOISE = 0.1 * DEG     # rad/s RMS per 100 Hz sample (BNO085 class, as yaw/sim.py)
GYRO_BIAS = 0.02 * DEG     # rad/s gyro bias (after factory calibration)
GYRO_BIAS_EST_ERR = 0.005 * DEG   # rad/s: estimator bias error at release (converged during ground alignment)
ACC_NOISE = 0.02           # m/s^2 RMS
AHRS_KP = 1 / 30.0         # 1/s: accelerometer correction gain
AHRS_KI = AHRS_KP ** 2 / 4
WC, ZETA_C = 15.0, 0.8     # attitude loop bandwidth (rad/s) and damping, all axes
KI_FRAC = 0.2
YAW_BLEED = 1.0 * DEG      # rad/s: heading walk used to unload the yaw wheel
SLEW = 1.0 * DEG           # rad/s: commanded tilt slew rate
V_BUS = 11.1
RPM_CAP = 6000.0
BEARING_DRAG = 0.1e-3      # N·m


@dataclass
class Wheel:
    name: str
    kt: float          # N·m per A peak (FOC)
    R: float           # ohm per phase
    J: float           # kg·m^2 rotor
    i_max: float       # A
    c_eddy: float      # N·m·s/rad (eddy drag torque = c * speed)

    def ke(self):
        return self.kt * 2 / 3

    def w_cap(self):
        w_nl = (V_BUS / math.sqrt(3)) / self.ke()
        return min(w_nl, RPM_CAP * 2 * math.pi / 60)


def wheels():
    """Wheel candidates from the stackup study (results/pcbmotor_ace_stackups.json); same 84 mm,
    3 mm-disc high-momentum rotor on all three axes."""
    rows = {r["config"]: r for r in json.loads((OUT / "pcbmotor_ace_stackups.json").read_text())}
    w2k = 2000 * 2 * math.pi / 60

    def mk(label, cfg, imax):
        r = rows[cfg]
        return Wheel(label, r["Kt_mNm_per_A"] / 1e3, r["R_phase_ohm"], r["rotor_J_kgm2"], imax,
                     r["eddy_W_at_2000rpm"] / w2k ** 2)
    return [
        mk("JLC 4L solid traces, 1 A driver (current design)", "JLC 4L, 2 oz out / 1 oz in (current design)", 1.0),
        mk("JLC 4L solid traces, 2 A driver", "JLC 4L, 2 oz out / 1 oz in (current design)", 2.0),
        mk("JLC 4L 3 strands, 2 A driver", "JLC 4L, 2 oz out / 1 oz in, 3 strands", 2.0),
        mk("ACE 6L 3 oz 3 strands, 2 A driver", "ACE 6L, 3 oz, 3 strands", 2.0),
    ]


# ------------------------------------------------------------------ quaternion helpers (N,4)
def qmul(a, b):
    aw, ax, ay, az = a.T
    bw, bx, by, bz = b.T
    return np.stack([aw * bw - ax * bx - ay * by - az * bz, aw * bx + ax * bw + ay * bz - az * by,
                     aw * by - ax * bz + ay * bw + az * bx, aw * bz + ax * by - ay * bx + az * bw], axis=1)


def qconj(q):
    return q * np.array([1.0, -1, -1, -1])


def qexp(v):
    """Quaternion of rotation vector v (N,3)."""
    a = np.linalg.norm(v, axis=1)
    s = np.where(a > 1e-12, np.sin(a / 2) / np.maximum(a, 1e-12), 0.5)
    return np.concatenate([np.cos(a / 2)[:, None], v * s[:, None]], axis=1)


def q2R(q):
    w, x, y, z = q.T
    return np.stack([
        np.stack([1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)], axis=1),
        np.stack([2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)], axis=1),
        np.stack([2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)], axis=1)], axis=1)


def tilt_quat(theta, azimuth):
    """Tilt by theta about the horizontal axis at the given azimuth (arrays (N,))."""
    ax = np.stack([np.cos(azimuth), np.sin(azimuth), np.zeros_like(azimuth)], axis=1)
    return qexp(ax * np.asarray(theta)[:, None])


def mv(R, v):
    return np.einsum("nij,nj->ni", R, v)


def mtv(R, v):
    return np.einsum("nji,nj->ni", R, v)


# ------------------------------------------------------------------ scenario
@dataclass
class Scenario:
    """Per-case inputs. Arrays have length N (or shape (N,2)/(N,3))."""
    T: float
    d: np.ndarray                  # true eye height above the centre of mass (m)
    L: np.ndarray                  # line length (m)
    c0: np.ndarray                 # (N,2) lateral centre-of-mass error at t=0 (m, body x/y)
    phi0: np.ndarray               # (N,2) initial line angle (rad) in world x and y
    phidot0: np.ndarray            # (N,2) initial line rate (rad/s)
    wheels_on: bool = True         # pitch/roll wheels (yaw wheel follows `yaw_on`)
    yaw_on: bool = True
    trim_on: bool = True
    tilt_target: np.ndarray | None = None    # (N,) rad, commanded tilt (slewed at SLEW from t_tilt)
    tilt_az: np.ndarray | None = None
    t_tilt: float = 5.0
    unbalance: np.ndarray | None = None      # (N,2) step in centre-of-mass offset (m) at t_unbal
    t_unbal: float = 10.0
    top_sigma_v: np.ndarray | None = None    # (N,) m/s RMS of random top (balloon) velocity
    top_tau: float = 4.0
    h_yaw0: np.ndarray | None = None         # (N,) initial yaw wheel momentum (N·m·s)
    yaw_gust: bool = False
    seed: int = 0


def simulate(sc: Scenario, wh: Wheel, record_every=20, t_skip=5.0):
    N = len(sc.d)
    n = int(round(sc.T / DT))
    rng = np.random.default_rng(1000 + sc.seed)
    I = I_BODY
    k_l = M * (2 * math.pi * LINE_BOUNCE_HZ) ** 2
    c_l = 2 * LINE_ZETA * math.sqrt(k_l * M)
    w_sw = np.sqrt(G / sc.L)
    b_sw = 2 * SWING_ZETA * M * w_sw                      # (N,)
    c_rock = 2 * ROCK_ZETA * np.sqrt(M * G * sc.d * (I[0] + M * sc.d ** 2))
    kp, kd = I * WC ** 2, 2 * ZETA_C * I * WC
    ki = KI_FRAC * WC * kp
    w_cap = wh.w_cap()
    va = V_BUS / math.sqrt(3)
    ke = wh.ke()
    on = np.array([sc.wheels_on, sc.wheels_on, sc.yaw_on], dtype=float)

    # ---- initial state: line straight at phi0, body level, at rest except the swing rate
    z3 = np.zeros((N, 3))
    q = np.tile([1.0, 0, 0, 0], (N, 1))
    om = z3.copy()
    hw = z3.copy()
    if sc.h_yaw0 is not None:
        hw[:, 2] = sc.h_yaw0
    c = np.zeros((N, 2)) + sc.c0
    s_trim = np.zeros((N, 2))                     # trim mass position (body x/y)
    stretch = M * G / k_l
    Lt = sc.L + stretch
    ph = sc.phi0
    line_dir = np.stack([np.sin(ph[:, 0]), np.sin(ph[:, 1]), -np.sqrt(np.clip(1 - np.sin(ph[:, 0]) ** 2 -
                                                                              np.sin(ph[:, 1]) ** 2, 0, 1))], axis=1)
    S = z3.copy()                                  # top of the line
    Sv = z3.copy()
    p = S + Lt[:, None] * line_dir
    r_att_b = np.concatenate([-c, sc.d[:, None]], axis=1)
    x = p - r_att_b
    v = np.stack([sc.phidot0[:, 0] * sc.L, sc.phidot0[:, 1] * sc.L, np.zeros(N)], axis=1)

    # ---- estimator and controller state
    qe = q.copy()
    bias = GYRO_BIAS * rng.choice([-1.0, 1.0], size=(N, 3))
    be = -bias + GYRO_BIAS_EST_ERR * rng.choice([-1.0, 1.0], size=(N, 3))
    integ = z3.copy()
    i_cmd = z3.copy()
    tau_lp = np.zeros((N, 2))
    c_fb = np.zeros((N, 2))
    e_lp = np.zeros((N, 2))
    yaw_off = np.zeros(N)
    gust = np.zeros(N)
    a_g = math.exp(-2 * math.pi * YAW_GUST_FC * DT)
    a_top = math.exp(-DT / sc.top_tau)
    dtc = DT * CTRL_EVERY
    has_tilt = sc.tilt_target is not None
    az = sc.tilt_az if has_tilt else np.zeros(N)

    rec = dict(t=[], tilt=[], tilt_est=[], yaw=[], h=[], i=[], trim=[], line=[], p_cu=[], p_eddy=[], p_in=[])
    acc = dict(tilt_sq=np.zeros(N), tilt_pk=np.zeros(N), ok2=np.zeros(N), est_pk=np.zeros(N),
               h_pk=np.zeros((N, 3)), i_pk=np.zeros((N, 3)), p_cu=np.zeros((N, 3)), p_eddy=np.zeros((N, 3)),
               p_in=np.zeros(N), trim_pk=np.zeros(N), line_pk=np.zeros(N), line_sq=np.zeros(N), sat=np.zeros(N), cnt=0)
    q_ref = q.copy()
    for k in range(n):
        t = k * DT
        # ---- reference: tilt (slewed) then heading walk
        if has_tilt:
            th = np.minimum(sc.tilt_target, SLEW * max(0.0, t - sc.t_tilt))
            q_tilt = tilt_quat(th, az)
        else:
            q_tilt = np.tile([1.0, 0, 0, 0], (N, 1))
        q_ref = qmul(qexp(np.stack([np.zeros(N), np.zeros(N), yaw_off], axis=1)), q_tilt)
        cu = c.copy()
        if sc.unbalance is not None and t >= sc.t_unbal:
            cu = cu + sc.unbalance
        cu = cu + (M_TRIM / M) * s_trim

        # ---- forces
        R = q2R(q)
        r_att_b = np.concatenate([-cu, sc.d[:, None]], axis=1)
        p = x + mv(R, r_att_b)
        if sc.top_sigma_v is not None:
            Sv = a_top * Sv + math.sqrt(1 - a_top ** 2) * sc.top_sigma_v[:, None] * rng.standard_normal((N, 3)) * [1, 1, 0]
            S = S + Sv * DT - S * (DT / 60.0)       # slow leak keeps the top near the origin
        l = S - p
        ln = np.linalg.norm(l, axis=1)
        e = l / ln[:, None]
        pdot = v + mv(R, np.cross(om, r_att_b))
        ldot = np.sum((Sv - pdot) * e, axis=1)
        T = np.maximum(0.0, k_l * (ln - sc.L) + c_l * ldot)
        F_line = T[:, None] * e
        F_aero = -b_sw[:, None] * v * [1, 1, 0]
        F = F_line + F_aero + np.array([0, 0, -M * G])
        tau_line = np.cross(r_att_b, mtv(R, F_line))
        tau_line[:, :2] -= c_rock[:, None] * om[:, :2]
        if sc.yaw_gust:
            gust = a_g * gust + math.sqrt(1 - a_g ** 2) * YAW_GUST_NM * rng.standard_normal(N)
            tau_line[:, 2] += gust

        # ---- sensors, estimator, controller (100 Hz)
        if k % CTRL_EVERY == 0:
            om_m = om + bias + GYRO_NOISE * rng.standard_normal((N, 3))
            f_b = mtv(R, F_line + F_aero) / M + ACC_NOISE * rng.standard_normal((N, 3))
            u_m = f_b / np.linalg.norm(f_b, axis=1)[:, None]
            Re = q2R(qe)
            u_e = Re[:, 2, :]                       # world up in the estimated body frame
            err = np.cross(u_m, u_e)
            be = be + AHRS_KI * err * dtc          # be is minus the bias estimate (Mahony 2008)
            om_h = om_m + be + AHRS_KP * err
            qe = qmul(qe, qexp(om_h * dtc))
            qe /= np.linalg.norm(qe, axis=1)[:, None]
            om_c = om_m + be                        # bias-corrected rate for the controller
            qerr = qmul(qconj(q_ref), qe)
            qerr *= np.sign(qerr[:, :1] + 1e-15)
            ea = 2 * qerr[:, 1:]
            W = hw / wh.J - om
            tau_body = -kp * ea - kd * om_c - ki * integ + np.cross(om_c, om_c * I + hw)
            tau_m = -tau_body
            ic = tau_m / wh.kt
            motoring = tau_m * W > 0
            i_v = np.where(motoring, (va - ke * np.abs(W)) / wh.R, (va + ke * np.abs(W)) / wh.R)
            ilim = np.clip(np.minimum(wh.i_max, i_v), 0, None)
            inew = np.clip(ic, -ilim, ilim)
            inew = np.where((np.abs(W) >= w_cap) & motoring, 0.0, inew) * on
            i_cmd = inew
            unsat = np.abs(inew - ic * on) < 1e-9 * (1 + np.abs(ic))
            integ = np.where(unsat, integ + ea * dtc, integ)
            # yaw wheel unloading: walk the heading reference toward where the wheel can slow down
            if sc.yaw_on:
                yaw_off += YAW_BLEED * np.clip(hw[:, 2] / (0.5 * wh.J * w_cap), -1, 1) * dtc
            # trim stage
            if sc.trim_on:
                u_ref = q2R(q_ref)[:, 2, :]
                c_ff = -D_NOM * u_ref[:, :2] / u_ref[:, 2:3]
                K = M * G * u_ref[:, 2]
                if sc.wheels_on:
                    tm = wh.kt * i_cmd[:, :2]
                    tau_lp += (tm - tau_lp) * dtc / TRIM_TORQUE_LP
                    a1, a0 = 2 * TRIM_ZETA * TRIM_WN, TRIM_WN ** 2
                    c_fb[:, 0] += -(a1 * tau_lp[:, 1] + a0 * hw[:, 1]) / K * dtc
                    c_fb[:, 1] += (a1 * tau_lp[:, 0] + a0 * hw[:, 0]) / K * dtc
                else:
                    e_lp += (ea[:, :2] - e_lp) * dtc / 2.0
                    c_fb[:, 0] += -D_NOM * e_lp[:, 1] / TRIM_ONLY_TAU * dtc
                    c_fb[:, 1] += D_NOM * e_lp[:, 0] / TRIM_ONLY_TAU * dtc
                s_cmd = (c_ff + c_fb) * M / M_TRIM
                s_cl = np.clip(s_cmd, -TRIM_TRAVEL, TRIM_TRAVEL)
                c_fb = np.where(s_cl != s_cmd, s_cl * M_TRIM / M - c_ff, c_fb)   # anti-windup
                s_trim = s_trim + np.clip(s_cl - s_trim, -TRIM_SPEED * dtc, TRIM_SPEED * dtc)

        # ---- wheels and body
        W = hw / wh.J - om
        tau_m = wh.kt * i_cmd
        tau_d = wh.c_eddy * W + BEARING_DRAG * np.tanh(W / 0.5)
        dom = (tau_line - tau_m + tau_d - np.cross(om, om * I + hw)) / I
        hw = hw + (tau_m - tau_d) * DT
        om = om + dom * DT
        q = qmul(q, qexp(om * DT))
        q /= np.linalg.norm(q, axis=1)[:, None]
        v = v + F / M * DT
        x = x + v * DT

        # ---- bookkeeping
        if t >= t_skip or k % record_every == 0:
            Rn = q2R(q)
            Rr = q2R(q_ref)
            tilt = np.arccos(np.clip(np.sum(Rn[:, :, 2] * Rr[:, :, 2], axis=1), -1, 1))
            Re = q2R(qe)
            t_est = np.arccos(np.clip(np.sum(Rn[:, :, 2] * Re[:, :, 2], axis=1), -1, 1))
            line_ang = np.arccos(np.clip(e[:, 2], -1, 1))
            p_cu = 1.5 * wh.R * i_cmd ** 2
            p_ed = wh.c_eddy * W ** 2
            p_in = np.sum(np.maximum(p_cu + tau_m * W, 0.0), axis=1)
        if t >= t_skip:
            acc["cnt"] += 1
            acc["tilt_sq"] += tilt ** 2
            acc["tilt_pk"] = np.maximum(acc["tilt_pk"], tilt)
            acc["ok2"] += tilt < 2 * DEG
            acc["est_pk"] = np.maximum(acc["est_pk"], t_est)
            acc["h_pk"] = np.maximum(acc["h_pk"], np.abs(hw))
            acc["i_pk"] = np.maximum(acc["i_pk"], np.abs(i_cmd))
            acc["p_cu"] += p_cu
            acc["p_eddy"] += p_ed
            acc["p_in"] += p_in
            acc["trim_pk"] = np.maximum(acc["trim_pk"], np.abs(s_trim).max(axis=1))
            acc["line_pk"] = np.maximum(acc["line_pk"], line_ang)
            acc["line_sq"] += line_ang ** 2
            acc["sat"] = np.maximum(acc["sat"], (np.abs(W) >= 0.95 * w_cap).any(axis=1))
        if k % record_every == 0:
            rec["t"].append(t)
            rec["tilt"].append(tilt)
            rec["tilt_est"].append(t_est)
            rec["h"].append(hw.copy())
            rec["i"].append(i_cmd.copy())
            rec["trim"].append(s_trim.copy())
            rec["line"].append(line_ang)
            rec["p_cu"].append(p_cu.sum(axis=1))
            rec["p_eddy"].append(p_ed.sum(axis=1))
            rec["p_in"].append(p_in)
    cnt = max(acc["cnt"], 1)
    stats = dict(tilt_rms_deg=np.sqrt(acc["tilt_sq"] / cnt) / DEG, tilt_peak_deg=acc["tilt_pk"] / DEG,
                 within2_pct=100 * acc["ok2"] / cnt, est_err_peak_deg=acc["est_pk"] / DEG,
                 h_peak_mNms=acc["h_pk"] * 1e3, i_peak_A=acc["i_pk"],
                 p_cu_W=acc["p_cu"] / cnt, p_eddy_W=acc["p_eddy"] / cnt, p_in_W=acc["p_in"] / cnt,
                 trim_peak_mm=acc["trim_pk"] * 1e3, line_peak_deg=acc["line_pk"] / DEG,
                 line_rms_deg=np.sqrt(acc["line_sq"] / cnt) / DEG,
                 saturated=acc["sat"].astype(bool))
    rec = {k: np.array(v) for k, v in rec.items()}
    return stats, rec
