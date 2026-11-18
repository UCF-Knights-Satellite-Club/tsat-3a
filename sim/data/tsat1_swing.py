"""TSat-1 flight accelerometer analysis (June 2025 tethered flight).

Reads the raw log in place (it is NOT copied into the repo):
    ~/Documents/tsat1launch/tsatlog3/data.csv
Columns: index,temp,pressure,altitude,accelx,accely,accelz,accel_filtered,ascent_velocity,state

What this data can and cannot tell us
- Accelerometer only (no gyro, no magnetometer). Yaw (spin about the tether) is NOT observable:
  for a payload hanging on a line, gravity stays on the body z axis whatever the yaw angle.
- The onboard photos are ~10 s apart, so image-based yaw rates alias above ~3 RPM.
- What it does give: the excitation spectrum (horizontal specific force), the tension modulation
  |a| at twice the swing frequency (which gives swing amplitude for an ideal pendulum), and the
  flight profile. These set the frequency content and scale of the disturbance model.

Run from sim/:  .venv/bin/python -m data.tsat1_swing
"""
from __future__ import annotations

import json
import os
import pathlib
import numpy as np
from scipy import signal
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

LOG_DIR = pathlib.Path(os.path.expanduser("~/Documents/tsat1launch/tsatlog3"))
OUT = pathlib.Path(__file__).resolve().parents[1] / "results"
G = 9.80665


def sample_rate():
    """Log has no timestamps. Use the FAT mtimes of the first photo and of data.csv (both written by
    the flight computer's clock) to get the logging span, then rows / span."""
    pics = sorted(LOG_DIR.glob("pic*.jpg"), key=lambda p: int(p.stem[3:]))
    t0 = pics[0].stat().st_mtime
    t1 = (LOG_DIR / "data.csv").stat().st_mtime
    with open(LOG_DIR / "data.csv") as f:
        n = sum(1 for _ in f) - 1
    return n / (t1 - t0), n, t1 - t0


def load():
    d = np.genfromtxt(LOG_DIR / "data.csv", delimiter=",", names=True)
    return d


def main():
    OUT.mkdir(exist_ok=True)
    fs, n, span = sample_rate()
    d = load()
    st = d["state"].astype(int)
    # flight window: first row of state 2 to last row before the long state-4 rest, trimmed
    i0 = int(np.argmax(st == 2))
    i_end = int(np.argmax(st == 4)) + int(60 * fs)       # include one minute after state 4 begins
    seg = slice(i0, i_end)
    t = (np.arange(n) / fs)[seg]
    t -= t[0]
    alt = d["altitude"][seg]
    ax_, ay_, az_ = d["accelx"][seg], d["accely"][seg], d["accelz"][seg]
    # remove static sensor tilt (mean over the segment)
    ahx, ahy = ax_ - ax_.mean(), ay_ - ay_.mean()
    ah = np.hypot(ahx, ahy)
    amag = np.sqrt(ax_ ** 2 + ay_ ** 2 + az_ ** 2)

    # hanging portion: altitude above 50% of max (skip launch handling and landing)
    hang = alt > 0.5 * np.nanmax(alt)
    nper = int(min(len(t[hang]) // 4, 120 * fs))
    f, pxx = signal.welch(ahx[hang], fs=fs, nperseg=nper)
    _, pyy = signal.welch(ahy[hang], fs=fs, nperseg=nper)
    fm, pmm = signal.welch(amag[hang] - amag[hang].mean(), fs=fs, nperseg=nper)
    ph = pxx + pyy
    band = (f > 0.02) & (f < 2.0)
    f_peak_h = float(f[band][np.argmax(ph[band])])
    bandm = (fm > 0.04) & (fm < 2.0)
    f_peak_m = float(fm[bandm][np.argmax(pmm[bandm])])
    # tension modulation amplitude at its peak -> ideal-pendulum swing amplitude: dA = 3/4 g theta0^2
    b, a = signal.butter(2, [max(0.5 * f_peak_m, 0.01), min(1.5 * f_peak_m, 0.45 * fs)], btype="band", fs=fs)
    mod = signal.filtfilt(b, a, amag[hang] - amag[hang].mean())
    amp_mod = float(np.sqrt(2) * mod.std())
    theta0 = float(np.degrees(np.sqrt(4 * amp_mod / (3 * G))))
    swing_period = 2.0 / f_peak_m          # tension modulation is at twice the swing frequency
    L_equiv = G * (swing_period / (2 * np.pi)) ** 2

    # quiet top plateau (altitude within 3 m of max, before the descent): no reel-out steps
    top = (alt > np.nanmax(alt) - 3.0) & (t < t[np.argmax(alt)] + 30)
    descent_rate = float(np.max(-np.gradient(signal.savgol_filter(alt, 41, 2), 1 / fs)))
    res = dict(
        sample_rate_hz=fs, rows=n, log_span_s=span,
        flight_rows=[i0, i_end], flight_duration_s=float(t[-1]),
        max_altitude_m=float(np.nanmax(alt)), hang_duration_s=float(hang.sum() / fs),
        horiz_accel_rms_ms2=float(ah[hang].std()), horiz_accel_p99_ms2=float(np.percentile(ah[hang], 99)),
        horiz_psd_peak_hz=f_peak_h, tension_mod_peak_hz=f_peak_m,
        tension_mod_amp_ms2=amp_mod, est_swing_period_s=swing_period, est_swing_amp_deg=theta0,
        est_equiv_pendulum_length_m=L_equiv,
        top_plateau_s=float(top.sum() / fs), top_plateau_horiz_accel_rms_ms2=float(ah[top].std()),
        max_descent_rate_ms=descent_rate,
        notes=("Accelerometer only: yaw not observable. Swing amplitude assumes an ideal point-mass pendulum "
               "(tension modulation at 2x swing frequency); the ascent was reeled out in steps and each step "
               "jerks the line, so treat the swing amplitude as an upper bound. Sensor z is vertical "
               "(az ~ 9.8 at rest)."),
    )
    (OUT / "tsat1_swing.json").write_text(json.dumps(res, indent=2))

    fig, axs = plt.subplots(3, 1, figsize=(8, 9))
    axs[0].plot(t, alt, "k")
    axs[0].set_ylabel("baro altitude [m]")
    axs[0].set_title("TSat-1 flight (tsatlog3), 20 Hz accelerometer log")
    axs[1].plot(t, ah, lw=0.4)
    axs[1].set_ylabel("|horizontal accel|\n(tilt removed) [m/s²]")
    axs[1].set_xlabel("time since state 2 [s]")
    axs[2].loglog(f[1:], ph[1:], label="horizontal x+y")
    axs[2].loglog(fm[1:], pmm[1:], label="|a| (tension)")
    axs[2].axvline(f_peak_m, color="r", ls=":", label=f"tension peak {f_peak_m:.3f} Hz")
    axs[2].set_xlabel("frequency [Hz]")
    axs[2].set_ylabel("PSD [(m/s²)²/Hz]")
    axs[2].legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(OUT / "tsat1_flight_accel.png", dpi=150)
    plt.close(fig)
    print(json.dumps(res, indent=2))


if __name__ == "__main__":
    main()
