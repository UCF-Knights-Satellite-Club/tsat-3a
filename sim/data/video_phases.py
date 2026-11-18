"""Phase-by-phase yaw statistics from a video_spin CSV (run data.video_spin first).

Relative altitude comes from the frame-to-frame image scale of a nadir camera:
features shrink while climbing, so altitude ~ 1 / cumprod(scale). It drifts, so it
is only used to find the reel-out / hang / reel-in phases.

Usage (from sim/):
    .venv/bin/python -m data.video_phases results/video_spin_tsat2b_full.csv --name tsat2b_full
Writes results/video_phases_<name>.json and .png.
"""
from __future__ import annotations

import argparse
import json
import pathlib

import numpy as np
from scipy.ndimage import median_filter, uniform_filter1d
from scipy.signal import welch

RESULTS = pathlib.Path(__file__).resolve().parents[1] / "results"


def clean(t, rate, good):
    """Reject single-frame glitches: bad fits and points far from a 0.5 s running median."""
    fps = 1 / np.median(np.diff(t))
    r = np.where(good > 0, rate, np.nan)
    filled = np.where(np.isnan(r), 0.0, r)
    med = median_filter(filled, size=int(fps / 2) | 1, mode="nearest")
    bad = np.isnan(r) | (np.abs(r - med) > 300)
    r = np.where(bad, med, r)
    return r, fps, bad.mean()


def phase_stats(t, r, fps, yaw):
    sm = uniform_filter1d(r, int(fps))  # 1 s average
    f, P = welch(r - r.mean(), fs=fps, nperseg=min(len(r), int(120 * fps)))
    band = (f > 1 / 120) & (f < 1.0)
    fpk = f[band][np.argmax(P[band])]
    acc = np.gradient(uniform_filter1d(r, int(fps / 2)), 1 / fps)  # deg/s^2
    dur = t[-1] - t[0]
    return dict(
        duration_s=float(dur),
        rate_rms_deg_s=float(np.sqrt(np.mean(r ** 2))),
        rate_abs_median_deg_s=float(np.median(np.abs(r))),
        rate_1s_peak_deg_s=float(np.abs(sm).max()),
        rate_1s_peak_rpm=float(np.abs(sm).max() / 6),
        rate_1s_p95_rpm=float(np.percentile(np.abs(sm), 95) / 6),
        dominant_period_s=float(1 / fpk),
        accel_rms_deg_s2=float(np.sqrt(np.mean(acc ** 2))),
        net_drift_turns=float((yaw[-1] - yaw[0]) / 360),
        net_drift_rpm=float((yaw[-1] - yaw[0]) / 360 / (dur / 60)),
        yaw_excursion_turns=float((yaw.max() - yaw.min()) / 360),
        frac_rate_below_2deg_s=float(np.mean(np.abs(r) < 2)),
        frac_1s_rate_below_10deg_s=float(np.mean(np.abs(sm) < 10)),
    )


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("csv")
    ap.add_argument("--name", default="video")
    ap.add_argument("--phases", default=None,
                    help="comma list of phase boundaries in seconds (default: from altitude proxy)")
    args = ap.parse_args()

    d = np.genfromtxt(args.csv, delimiter=",", names=True)
    t, rate, scale, good = d["t_s"], d["yaw_rate_deg_s"], d["scale"], d["good"]
    r, fps, frac_bad = clean(t, rate, good)
    yaw = np.cumsum(r / fps)
    s = np.where((good > 0) & np.isfinite(scale) & (np.abs(scale - 1) < 0.1), scale, 1.0)
    alt = 1.0 / np.cumprod(s)
    alt = alt / np.percentile(alt, 99)  # relative, 1 = near the top

    if args.phases:
        edges = [t[0]] + [float(x) for x in args.phases.split(",")] + [t[-1]]
    else:
        a = uniform_filter1d(alt, int(20 * fps))
        top = a > 0.85 * a.max()
        i0, i1 = np.argmax(top), len(top) - 1 - np.argmax(top[::-1])
        edges = [t[0], t[i0], t[i1], t[-1]]
    names = ["reel-out / ascent", "top hang", "reel-in / descent"] if len(edges) == 4 else \
        [f"phase {k + 1}" for k in range(len(edges) - 1)]

    out = dict(video_csv=args.csv, fps=fps, frac_frames_replaced=float(frac_bad),
               whole_flight=phase_stats(t, r, fps, yaw), phases={})
    for k, nm in enumerate(names):
        m = (t >= edges[k]) & (t < edges[k + 1])
        if m.sum() > fps * 30:
            out["phases"][nm] = dict(t_start_s=float(edges[k]), t_end_s=float(edges[k + 1]),
                                     **phase_stats(t[m], r[m], fps, yaw[m]))

    RESULTS.mkdir(exist_ok=True)
    stem = RESULTS / f"video_phases_{args.name}"
    json.dump(out, open(f"{stem}.json", "w"), indent=2)

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(4, 1, figsize=(11, 10), sharex=True)
    ax[0].plot(t, alt, lw=0.8)
    ax[0].set_ylabel("relative altitude\n(image scale)")
    for e in edges[1:-1]:
        for a_ in ax:
            a_.axvline(e, color="r", ls="--", lw=0.8)
    ax[1].plot(t, r, lw=0.3, color="0.6", label="per frame (cleaned)")
    ax[1].plot(t, uniform_filter1d(r, int(fps)), lw=0.8, color="k", label="1 s average")
    ax[1].axhspan(-2, 2, color="g", alpha=0.3, label="+/- 2 deg/s")
    ax[1].set_ylim(-800, 800)
    ax[1].set_ylabel("yaw rate [deg/s]")
    ax[1].legend(fontsize=7, loc="upper right")
    ax[2].plot(t, uniform_filter1d(r, int(fps)) / 6, lw=0.8)
    ax[2].set_ylabel("1 s avg [rpm]")
    ax[3].plot(t, yaw / 360)
    ax[3].set_ylabel("cumulative yaw [turns]")
    ax[3].set_xlabel("flight video time [s]")
    w = out["whole_flight"]
    fig.suptitle(f"{args.name}: yaw from onboard nadir video. Whole flight: rms {w['rate_rms_deg_s']:.0f} deg/s, "
                 f"1 s peak {w['rate_1s_peak_rpm']:.0f} rpm, stable (<2 deg/s) {100 * w['frac_rate_below_2deg_s']:.1f}% of time")
    fig.tight_layout()
    fig.savefig(f"{stem}.png", dpi=120)
    print(json.dumps(out, indent=2))


if __name__ == "__main__":
    main()
