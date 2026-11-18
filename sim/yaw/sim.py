"""Single-axis (yaw / Z) reaction-wheel simulation of TSat-3A hanging on a tethered balloon line.

Vectorized over Monte Carlo runs: every candidate motor is simulated for all
(rigging x disturbance level x seed) cases at once. Deterministic (seeded).

Run from sim/:  .venv/bin/python -m yaw.sim
Outputs: results/yaw_summary.csv, results/yaw_requirements.json, results/yaw_*.png
"""
from __future__ import annotations

import csv
import json
import math
import pathlib
from dataclasses import dataclass

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from .candidates import candidates, Motor

OUT = pathlib.Path(__file__).resolve().parents[1] / "results"
DEG = math.pi / 180

# ------------------------------------------------------------------ assumptions
I_SAT = 1.67e-3          # kg m^2: 1 kg, 10 x 10 cm cross-section, about the vertical axis (ASSUMED)
V_BUS = 11.1             # 3S LiPo
DT = 0.005               # plant step [s]
CTRL_EVERY = 2           # control at 100 Hz (50 Hz limits the loop to ~15 rad/s; see yaw tuning in RESULTS.md)
T_SIM = 600.0            # simulated seconds per run; energy scaled to a 30 min flight
T_FLIGHT = 1800.0
T_SKIP = 20.0            # ignore the first 20 s (initial transient) in the statistics
GYRO_NOISE = 0.1 * DEG   # rad/s RMS (BNO085 class)
GYRO_BIAS = 0.02 * DEG   # rad/s residual bias after calibration (heading = integrated gyro)
QUIESCENT_W = 0.15       # driver + sensors for the wheel subsystem
RATE_OK = 2 * DEG        # video-quality thresholds
ERR_OK = 10 * DEG
BLEED_RATE = 1.0 * DEG   # max rate at which the heading reference is walked to unload the wheel
WC, ZETA = 20.0, 0.8     # attitude loop bandwidth [rad/s] and damping (PID, 100 Hz)
KI_FRAC = 0.2            # integral gain = KI_FRAC * WC * Kp (removes steady friction / spring offsets)
TETHER_ZETA = 0.05
# set by yaw/calibrate.py so the uncontrolled, no-swivel 1U sim matches the TSat-2B video
WANDER_LINE = 2.36       # rad, balloon-end yaw wander sigma (line-driven case), from yaw.calibrate
WANDER_TAU_LINE = 8.0    # s, its correlation time
GUST_GUST = 2.72e-3      # Nm RMS gust torque (gust-driven case), from yaw.calibrate

RIGGING = {              # coupling at the payload attachment
    "no swivel": None,
    "ball-bearing swivel": 0.05e-3,   # Nm friction: ~0.002 x 10 N x 2.5 mm
    "slip ring": 1.0e-3,              # Nm: capsule slip ring datasheets list ~0.5-1 mNm starting torque
}
LEVELS = {               # stored line twist, balloon-end wander (OU sigma), gust torque RMS
    "low": dict(twist_turns=0.25, wander_rad=0.5, gust_Nm=0.1e-3),
    "medium": dict(twist_turns=1.0, wander_rad=2.0, gust_Nm=0.3e-3),
    "high": dict(twist_turns=3.0, wander_rad=6.0, gust_Nm=1.0e-3),
    # Calibrated to the TSat-2B onboard flight video (data/video_spin.py, data/video_phases.py):
    # uncontrolled, no swivel -> 172 deg/s RMS yaw rate, torsional periods 2-15 s. Two bracketing
    # explanations of the same motion (values set by yaw/calibrate.py):
    #   line-driven: the motion comes down the line (balloon yaw + line torsion); gusts as 'high'.
    #   gust-driven: the motion is aerodynamic torque on the body; line as 'medium'.
    "measured, line-driven": dict(twist_turns=3.0, wander_rad=WANDER_LINE, gust_Nm=1.0e-3,
                                  tether_T=(2.0, 15.0), wander_tau=WANDER_TAU_LINE),
    "measured, gust-driven": dict(twist_turns=1.0, wander_rad=2.0, gust_Nm=GUST_GUST,
                                  tether_T=(2.0, 15.0)),
}
WANDER_TAU = 60.0        # s, balloon-end yaw correlation time
GUST_FC = 0.3            # Hz, gust torque bandwidth (TSat-1 tension spectrum peaks at 0.29 Hz)
TETHER_T = (20.0, 120.0)  # s, uniform range of tether torsional period
SEEDS = 8
EXAMPLE_LEVEL = "measured, line-driven"


@dataclass
class Case:
    rigging: str
    level: str
    seed: int


def make_cases():
    return [Case(r, l, s) for r in RIGGING for l in LEVELS for s in range(SEEDS)]


def disturbances(cases, n_steps, seed0=1234):
    """Pre-generate balloon-end angle and gust torque for each case (same for every candidate)."""
    N = len(cases)
    rng = np.random.default_rng(seed0)
    psi_b = np.zeros((n_steps, N))
    gust = np.zeros((n_steps, N))
    a_w = np.array([math.exp(-DT / LEVELS[c.level].get("wander_tau", WANDER_TAU)) for c in cases])
    a_g = math.exp(-2 * math.pi * GUST_FC * DT)
    sig_w = np.array([LEVELS[c.level]["wander_rad"] for c in cases])
    sig_g = np.array([LEVELS[c.level]["gust_Nm"] for c in cases])
    xw = np.zeros(N)
    xg = rng.standard_normal(N) * sig_g
    for k in range(n_steps):
        xw = a_w * xw + np.sqrt(1 - a_w * a_w) * sig_w * rng.standard_normal(N)
        xg = a_g * xg + math.sqrt(1 - a_g * a_g) * sig_g * rng.standard_normal(N)
        psi_b[k] = xw
        gust[k] = xg
    # tether torsional period: uniform in TETHER_T, or log-uniform in the level's measured range
    u = rng.uniform(0, 1, N)
    t_t = np.array([math.exp(math.log(LEVELS[c.level]["tether_T"][0]) + uu * math.log(
        LEVELS[c.level]["tether_T"][1] / LEVELS[c.level]["tether_T"][0])) if "tether_T" in LEVELS[c.level]
        else TETHER_T[0] + uu * (TETHER_T[1] - TETHER_T[0]) for c, uu in zip(cases, u)])
    return psi_b, gust, t_t


def simulate(m: Motor, cases, dist, controlled=True, bleed=True, record=None):
    psi_b, gust, t_t = dist
    n_steps = psi_b.shape[0]
    N = len(cases)
    k_t = I_SAT * (2 * math.pi / t_t) ** 2
    c_t = 2 * TETHER_ZETA * np.sqrt(k_t * I_SAT)
    swivel = np.array([RIGGING[c.rigging] is not None for c in cases])
    tau_f = np.array([RIGGING[c.rigging] or 0.0 for c in cases])
    twist0 = np.array([LEVELS[c.level]["twist_turns"] * 2 * math.pi for c in cases])

    w_max = m.omega_max(V_BUS)
    w_dead = 0.0 if m.sensored else 0.15 * w_max
    w_bias = 0.0 if m.sensored else 0.5 * w_max
    ke, kt, va = m.ke(), m.kt, m.v_avail(V_BUS)
    kp, kd = I_SAT * WC ** 2, 2 * ZETA * I_SAT * WC
    ki = KI_FRAC * WC * kp
    integ = np.zeros(N)

    rng = np.random.default_rng(99)
    psi = np.where(swivel, 0.0, -twist0)       # stored twist acts through the spring when there is no swivel
    om = np.zeros(N)
    W = np.full(N, w_bias)                      # wheel speed relative to body
    psi_ref = psi.copy()
    psi_m = psi.copy()
    bias = GYRO_BIAS * np.where(np.arange(N) % 2 == 0, 1.0, -1.0)
    I = np.zeros(N)
    tau_req = np.zeros(N)
    u_cmd = np.zeros(N)                         # latched control (latency: one control period)
    t_sat = np.full(N, np.inf)
    good = np.zeros(N)
    cnt = 0
    e_sum = np.zeros(N)
    p_peak = np.zeros(N)
    i_peak = np.zeros(N)
    tau_peak = np.zeros(N)
    h_peak = np.zeros(N)
    rate_peak = np.zeros(N)
    rate_sq = np.zeros(N)
    unwind_tau = 5.0
    for k in range(n_steps):
        t = k * DT
        # ---------------- external torques
        dpsib = (psi_b[k] - psi_b[k - 1]) / DT if k else 0.0
        tq_tether_spring = -k_t * (psi - psi_b[k]) - c_t * (om - dpsib)
        top_rate = dpsib + np.where(swivel, twist0 / unwind_tau * math.exp(-t / unwind_tau), 0.0)
        tq_swivel = -tau_f * np.tanh((om - top_rate) / 0.02)
        tau_ext = np.where(swivel, tq_swivel, tq_tether_spring) + gust[k]
        # ---------------- controller (50 Hz, applied next period)
        if controlled and k % CTRL_EVERY == 0:
            I = u_cmd.copy()
            om_m = om + bias + GYRO_NOISE * rng.standard_normal(N)
            psi_m = psi_m + om_m * DT * CTRL_EVERY          # heading estimate = integrated gyro
            if bleed and np.isfinite(w_max):
                span = max(w_max - w_dead, 1e-9) * 0.5
                psi_ref += BLEED_RATE * np.clip((W - w_bias) / span, -1, 1) * DT * CTRL_EVERY
            err = psi_m - psi_ref
            tau_body = -kp * err - kd * om_m - ki * integ
            tau_req = -tau_body                                   # torque the motor must put on the wheel
            i_cmd = tau_req / kt
            motoring = tau_req * W > 0
            i_lim_v = np.where(motoring, (va - ke * np.abs(W)) / m.r, (va + ke * np.abs(W)) / m.r)
            i_lim = np.clip(np.minimum(m.i_max, i_lim_v), 0, None)
            i_new = np.clip(i_cmd, -i_lim, i_lim)
            i_new = np.where(np.abs(W) < w_dead, 0.0, i_new)       # sensorless: no commutation at low speed
            i_new = np.where((np.abs(W) >= w_max) & motoring, 0.0, i_new)
            u_cmd = i_new
            # anti-windup: integrate only while the actuator is not limited
            unsat = np.abs(i_new - i_cmd) < 1e-9 * (1 + np.abs(i_cmd))
            integ = np.where(unsat, integ + err * DT * CTRL_EVERY, integ)
        if not controlled:
            I = np.zeros(N)
        # ---------------- plant
        tau_em = kt * I
        tau_d = m.coulomb_Nm * np.tanh(W / 0.5) + m.viscous * W
        tau_w = tau_em - tau_d
        dom = (-tau_w + tau_ext) / I_SAT
        dW = tau_w / m.j_wheel - dom
        om += dom * DT
        W += dW * DT
        psi += om * DT
        # ---------------- bookkeeping
        if t >= T_SKIP:
            cnt += 1
            # heading error is judged on the controller's own estimate: slow gyro-bias drift of the
            # heading (0.02 deg/s) is harmless for video and is not a control failure
            ok = (np.abs(om) < RATE_OK) & (np.abs(psi_m - psi_ref) < ERR_OK)
            good += ok
            p = np.maximum(m.copper_loss(I) + tau_em * W, 0.0) + (QUIESCENT_W if controlled else 0.0)
            e_sum += p * DT
            p_peak = np.maximum(p_peak, p)
            i_peak = np.maximum(i_peak, np.abs(I))
            tau_peak = np.maximum(tau_peak, np.abs(tau_req))
            h_peak = np.maximum(h_peak, m.j_wheel * np.abs(W - w_bias))
            rate_peak = np.maximum(rate_peak, np.abs(om))
            rate_sq += om * om
            sat = (np.abs(W) >= 0.95 * w_max) | ((w_dead > 0) & (np.abs(W) <= 1.05 * w_dead))
            t_sat = np.where(sat & ~np.isfinite(t_sat), t, t_sat)
        if record is not None and k % 20 == 0:
            record.append((t, psi_m.copy(), om.copy(), W.copy(), I.copy(), psi_ref.copy()))
    dur = cnt * DT
    return dict(good_pct=100 * good / cnt, t_sat=t_sat, mean_P=e_sum / dur,
                energy_Wh_30min=e_sum / dur * T_FLIGHT / 3600, peak_P=p_peak, peak_I=i_peak,
                tau_req_peak_mNm=tau_peak * 1e3, h_peak_mNms=h_peak * 1e3,
                rate_peak_rpm=rate_peak * 60 / (2 * math.pi),
                rate_rms_dps=np.sqrt(rate_sq / cnt) / DEG)


def gust_step_settling(m: Motor):
    """Quiet conditions behind a ball-bearing swivel; a 3 mNm gust for 2 s at t = 5 s
    (about the peak gust torque of the 'high' level).
    Returns time from the end of the gust until rate < 2 deg/s and error < 10 deg for good."""
    cases = [Case("ball-bearing swivel", "low", 0)]
    n = int(60 / DT)
    gust = np.zeros((n, 1))
    gust[int(5 / DT):int(7 / DT)] = 3e-3
    dist = (np.zeros((n, 1)), gust, np.array([60.0]))
    rec = []
    simulate(m, cases, dist, record=rec)
    t = np.array([r[0] for r in rec])
    om = np.array([r[2][0] for r in rec])
    err = np.array([r[1][0] - r[5][0] for r in rec])
    bad = (np.abs(om) >= RATE_OK) | (np.abs(err) >= ERR_OK)
    idx = np.where(bad & (t >= 5))[0]
    if len(idx) == 0:
        return 0.0, float(np.abs(om).max() / DEG)
    return float(max(0.0, t[idx[-1]] - 7.0)), float(np.abs(om).max() / DEG)


def summarize(m, cases, res, unc):
    rows = []
    for rig in RIGGING:
        for lvl in LEVELS:
            sel = [i for i, c in enumerate(cases) if c.rigging == rig and c.level == lvl]
            ts = res["t_sat"][sel]
            rows.append(dict(
                candidate=m.name, rigging=rig, level=lvl,
                uncontrolled_peak_rpm=float(np.median(unc["rate_peak_rpm"][sel])),
                good_pct_median=float(np.median(res["good_pct"][sel])),
                good_pct_worst=float(np.min(res["good_pct"][sel])),
                saturated_runs=f"{int(np.isfinite(ts).sum())}/{len(sel)}",
                t_sat_median_s=float(np.median(ts)) if np.isfinite(ts).any() else float("inf"),
                rate_rms_dps_median=float(np.median(res["rate_rms_dps"][sel])),
                mean_P_W=float(np.median(res["mean_P"][sel])),
                peak_P_W=float(np.max(res["peak_P"][sel])),
                energy_Wh_30min=float(np.median(res["energy_Wh_30min"][sel])),
                peak_I_A=float(np.max(res["peak_I"][sel])),
                tau_req_peak_mNm=float(np.max(res["tau_req_peak_mNm"][sel])),
                h_peak_mNms=float(np.max(res["h_peak_mNms"][sel])),
            ))
    return rows


def plot_example(m, cases, dist, path):
    pick = [i for i, c in enumerate(cases) if c.level == EXAMPLE_LEVEL and c.seed == 0]
    rec = []
    simulate(m, cases, dist, record=rec)
    t = np.array([r[0] for r in rec])
    fig, axs = plt.subplots(3, 1, figsize=(9, 8), sharex=True)
    for i in pick:
        lab = cases[i].rigging
        axs[0].plot(t, np.array([r[2][i] for r in rec]) / DEG, lw=0.7, label=lab)
        axs[1].plot(t, np.array([r[3][i] for r in rec]) * 60 / (2 * math.pi), lw=0.7, label=lab)
        axs[2].plot(t, np.array([r[4][i] for r in rec]), lw=0.7, label=lab)
    axs[0].axhspan(-2, 2, color="g", alpha=0.15)
    axs[0].set_ylabel("body yaw rate [deg/s]")
    axs[0].set_ylim(-20, 20)
    axs[1].set_ylabel("wheel speed [rpm]")
    axs[2].set_ylabel("motor current [A]")
    axs[2].set_xlabel("time [s]")
    axs[0].legend(fontsize=8)
    axs[0].set_title(f"{m.name}\n{EXAMPLE_LEVEL} disturbance, seed 0")
    fig.tight_layout()
    fig.savefig(path, dpi=150)
    plt.close(fig)


def main():
    OUT.mkdir(exist_ok=True)
    cases = make_cases()
    n = int(T_SIM / DT)
    dist = disturbances(cases, n)
    all_rows, settle = [], {}
    ideal_req = None
    for m in candidates(V_BUS):
        print("simulating", m.name, flush=True)
        unc = simulate(m, cases, dist, controlled=False)
        if m.name.startswith("Ideal"):
            res = simulate(m, cases, dist, bleed=False)
            ideal_req = summarize(m, cases, res, unc)
            continue
        res = simulate(m, cases, dist)
        all_rows += summarize(m, cases, res, unc)
        settle[m.name] = gust_step_settling(m)
    with open(OUT / "yaw_summary.csv", "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(all_rows[0]))
        w.writeheader()
        w.writerows(all_rows)
    (OUT / "yaw_requirements.json").write_text(json.dumps(
        dict(ideal_actuator_hold_heading_10min=ideal_req,
             gust_step_settling_s_and_peak_rate_dps=settle), indent=2, default=str))
    # plots
    cands = [m for m in candidates(V_BUS) if not m.name.startswith("Ideal")]
    for m in cands:
        if m.name.startswith("PCB payload") or m.name.startswith("Faulhaber"):
            tag = "pcb_payload" if m.name.startswith("PCB") else "faulhaber"
            plot_example(m, cases, dist, OUT / f"yaw_example_{tag}.png")
    fig, ax = plt.subplots(figsize=(10, 5))
    names = [m.name.split(":")[0] for m in cands]
    x = np.arange(len(cands))
    for j, rig in enumerate(RIGGING):
        vals = [np.median([r["good_pct_median"] for r in all_rows if r["candidate"] == m.name and r["rigging"] == rig])
                for m in cands]
        ax.bar(x + (j - 1) * 0.27, vals, 0.27, label=rig)
    ax.set_xticks(x)
    ax.set_xticklabels(names, rotation=15, ha="right", fontsize=8)
    ax.set_ylabel("% of time video-stable\n(|rate|<2 deg/s, |err|<10 deg), median over levels")
    ax.legend()
    fig.tight_layout()
    fig.savefig(OUT / "yaw_good_pct.png", dpi=150)
    plt.close(fig)
    for r in all_rows:
        print({k: (round(v, 2) if isinstance(v, float) else v) for k, v in r.items()})
    print(json.dumps(settle, indent=1))


if __name__ == "__main__":
    main()
