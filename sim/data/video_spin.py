"""Yaw rate from a downward-looking onboard flight video.

With a nadir camera, rotation about the payload's vertical (yaw) axis shows up as
rotation of the ground image. For each consecutive frame pair we match ORB
features and fit a similarity transform (rotation + translation + scale) with
RANSAC; the rotation angle times the frame rate is the yaw rate. Pendulum tilt
shows up mostly as translation, which the fit absorbs.

Usage (from sim/):
    .venv/bin/python -m data.video_spin PATH_TO_VIDEO [--name tsat2b] [--skip-start 3]

Outputs results/video_spin_<name>.csv, .json and .png. The video is read in place.
"""
from __future__ import annotations

import argparse
import json
import math
import pathlib

import cv2
import numpy as np

RESULTS = pathlib.Path(__file__).resolve().parents[1] / "results"


def frame_rotation(prev, cur, orb, matcher, mask):
    k1, d1 = orb.detectAndCompute(prev, mask)
    k2, d2 = orb.detectAndCompute(cur, mask)
    if d1 is None or d2 is None or len(k1) < 20 or len(k2) < 20:
        return None
    matches = matcher.knnMatch(d1, d2, k=2)
    good = [m for m, n in (p for p in matches if len(p) == 2) if m.distance < 0.75 * n.distance]
    if len(good) < 15:
        return None
    p1 = np.float32([k1[m.queryIdx].pt for m in good])
    p2 = np.float32([k2[m.trainIdx].pt for m in good])
    M, inl = cv2.estimateAffinePartial2D(p1, p2, method=cv2.RANSAC, ransacReprojThreshold=3.0,
                                         maxIters=4000, confidence=0.995)
    if M is None or inl is None:
        return None
    n_in = int(inl.sum())
    ang = math.atan2(M[1, 0], M[0, 0])
    scale = math.hypot(M[0, 0], M[1, 0])
    return ang, scale, n_in, len(good)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("video")
    ap.add_argument("--name", default="video")
    ap.add_argument("--skip-start", type=float, default=0.0, help="seconds to skip (title cards)")
    ap.add_argument("--width", type=int, default=640)
    ap.add_argument("--min-inliers", type=int, default=25)
    args = ap.parse_args()

    cap = cv2.VideoCapture(args.video)
    fps = cap.get(cv2.CAP_PROP_FPS)
    n_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    orb = cv2.ORB_create(nfeatures=1500, fastThreshold=10)
    matcher = cv2.BFMatcher(cv2.NORM_HAMMING)

    prev, mask = None, None
    rows = []
    i = -1
    while True:
        ok, frame = cap.read()
        if not ok:
            break
        i += 1
        t = i / fps
        if t < args.skip_start:
            continue
        h, w = frame.shape[:2]
        g = cv2.cvtColor(cv2.resize(frame, (args.width, int(h * args.width / w))), cv2.COLOR_BGR2GRAY)
        if mask is None:
            hh, ww = g.shape
            mask = np.full_like(g, 255)
            # the tether/attachment hardware sits at the top centre and is fixed in the body frame
            mask[: int(0.25 * hh), int(0.40 * ww): int(0.60 * ww)] = 0
        if prev is not None and g.mean() > 15:  # skip black title/transition frames
            r = frame_rotation(prev, g, orb, matcher, mask)
            if r is not None:
                ang, scale, n_in, n_good = r
                rows.append((t, ang, scale, n_in, n_good))
            else:
                rows.append((t, float("nan"), float("nan"), 0, 0))
        prev = g
    cap.release()

    a = np.array(rows, dtype=float)
    t, ang, scale, n_in = a[:, 0], a[:, 1], a[:, 2], a[:, 3]
    good = (n_in >= args.min_inliers) & (np.abs(scale - 1) < 0.15)
    rate = np.degrees(ang) * fps  # deg/s, image rotation; sign is camera-frame convention
    rate_good = np.where(good, rate, np.nan)
    # cumulative yaw angle, holding through bad frames at the local median rate
    filled = rate_good.copy()
    med = np.nanmedian(filled)
    filled[np.isnan(filled)] = med
    yaw = np.cumsum(filled / fps)

    rpm = rate_good / 6.0
    s = {
        "video": str(args.video).replace(str(pathlib.Path.home()), "~"),
        "fps": fps,
        "frames_total": n_frames,
        "pairs_analysed": int(len(t)),
        "pairs_good": int(good.sum()),
        "duration_good_s": float(good.sum() / fps),
        "yaw_rate_abs_deg_s": {
            "median": float(np.nanmedian(np.abs(rate_good))),
            "p90": float(np.nanpercentile(np.abs(rate_good), 90)),
            "p99": float(np.nanpercentile(np.abs(rate_good), 99)),
            "max": float(np.nanmax(np.abs(rate_good))),
        },
        "yaw_rate_rpm": {
            "median_abs": float(np.nanmedian(np.abs(rpm))),
            "p90_abs": float(np.nanpercentile(np.abs(rpm), 90)),
            "max_abs": float(np.nanmax(np.abs(rpm))),
            "mean_signed": float(np.nanmean(rpm)),
        },
        "net_rotation_turns": float(yaw[-1] / 360.0),
        "fraction_time_below_2deg_s": float(np.nanmean(np.abs(rate_good) < 2.0)),
        "notes": ("Image rotation of a nadir camera = payload yaw. Per-frame rotation above ~180 deg "
                  "(> 5400 deg/s at 30 fps) would alias; motion blur drops matches at high rates. "
                  "Clip may be edited for presentation (check for cuts)."),
    }

    RESULTS.mkdir(exist_ok=True)
    stem = RESULTS / f"video_spin_{args.name}"
    np.savetxt(f"{stem}.csv", np.c_[t, rate, scale, n_in, good, yaw], delimiter=",",
               header="t_s,yaw_rate_deg_s,scale,inliers,good,yaw_cum_deg", comments="", fmt="%.4f")
    json.dump(s, open(f"{stem}.json", "w"), indent=2)

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
    ax[0].plot(t, rate, ".", ms=2, color="0.75", label="all frame pairs")
    ax[0].plot(t, rate_good, ".", ms=3, label=f"good (>= {args.min_inliers} inliers)")
    k = int(fps)
    sm = np.convolve(np.nan_to_num(rate_good, nan=med), np.ones(k) / k, mode="same")
    ax[0].plot(t, sm, lw=1.5, color="k", label="1 s moving average")
    ax[0].axhspan(-2, 2, color="g", alpha=0.2, label="+/- 2 deg/s (stable video)")
    ax[0].set_ylabel("yaw rate [deg/s]")
    ax[0].legend(fontsize=7, loc="upper right")
    ax[1].plot(t, yaw / 360.0)
    ax[1].set_ylabel("cumulative yaw [turns]")
    ax[2].plot(t, n_in, lw=0.8)
    ax[2].set_ylabel("RANSAC inliers")
    ax[2].set_xlabel("video time [s]")
    fig.suptitle(f"{args.name}: yaw rate from onboard nadir video "
                 f"(median |rate| {s['yaw_rate_rpm']['median_abs']:.1f} rpm, "
                 f"p90 {s['yaw_rate_rpm']['p90_abs']:.1f} rpm)")
    fig.tight_layout()
    fig.savefig(f"{stem}.png", dpi=120)
    print(json.dumps(s, indent=2))


if __name__ == "__main__":
    main()
