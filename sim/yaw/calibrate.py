"""Calibrate the 'measured' disturbance levels to the TSat-2B onboard flight video.

Target: the uncontrolled, no-swivel 1U simulation reproduces the video's yaw-rate RMS
(results/video_phases_tsat2b_full.json, whole flight). The line-driven case tunes the
balloon-end wander; the gust-driven case tunes the gust torque. Prints the values to
put in yaw/sim.py (WANDER_LINE, WANDER_TAU_LINE, GUST_GUST) and writes
results/yaw_calibration.json.

Run from sim/:  .venv/bin/python -m yaw.calibrate
"""
from __future__ import annotations

import json
import math

import numpy as np

from . import sim
from .candidates import candidates

T_CAL = 300.0


def uncontrolled_rms(level, n_seeds=8):
    cases = [sim.Case("no swivel", level, s) for s in range(n_seeds)]
    n = int(T_CAL / sim.DT)
    dist = sim.disturbances(cases, n, seed0=777)
    m = [c for c in candidates(sim.V_BUS) if c.name.startswith("Faulhaber")][0]
    rec = []
    res = sim.simulate(m, cases, dist, controlled=False, record=rec)
    om = np.array([r[2] for r in rec])            # (time, case) rad/s, sampled every 20 steps
    t = np.array([r[0] for r in rec])
    om = om[t >= sim.T_SKIP]
    k = max(1, int(1.0 / (20 * sim.DT)))           # 1 s average
    sm = np.array([np.convolve(om[:, i], np.ones(k) / k, mode="same") for i in range(om.shape[1])]).T
    return dict(rate_rms_deg_s=float(np.median(res["rate_rms_dps"])),
                rate_1s_p95_rpm=float(np.median(np.percentile(np.abs(sm), 95, axis=0)) * 60 / (2 * math.pi)),
                rate_1s_peak_rpm=float(np.median(np.abs(sm).max(axis=0)) * 60 / (2 * math.pi)))


def solve(level, key, lo, hi, target, iters=14):
    """Bisection on log(parameter) so that uncontrolled RMS rate hits the target."""
    for _ in range(iters):
        mid = math.sqrt(lo * hi)
        sim.LEVELS[level][key] = mid
        r = uncontrolled_rms(level)["rate_rms_deg_s"]
        if r < target:
            lo = mid
        else:
            hi = mid
    sim.LEVELS[level][key] = math.sqrt(lo * hi)
    return sim.LEVELS[level][key], uncontrolled_rms(level)


def main():
    vid = json.load(open(sim.OUT / "video_phases_tsat2b_full.json"))["whole_flight"]
    target = vid["rate_rms_deg_s"]
    print("target RMS rate", target)
    w, rw = solve("measured, line-driven", "wander_rad", 0.5, 200.0, target)
    g, rg = solve("measured, gust-driven", "gust_Nm", 1e-4, 1e-1, target)
    out = dict(target_video=vid,
               line_driven=dict(wander_rad=w, wander_tau_s=sim.LEVELS["measured, line-driven"]["wander_tau"], **rw),
               gust_driven=dict(gust_Nm=g, **rg))
    (sim.OUT / "yaw_calibration.json").write_text(json.dumps(out, indent=2))
    print(json.dumps(out, indent=2))
    print(f"\nput in yaw/sim.py: WANDER_LINE = {w:.3g}; GUST_GUST = {g:.3g}")


if __name__ == "__main__":
    main()
