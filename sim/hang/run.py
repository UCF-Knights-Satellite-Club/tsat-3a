"""KAOS-3 hanging-payload tests: the D&C demo specifications S1-S3 and a flight Monte Carlo.

Run from sim/:  .venv/bin/python -m hang.run        (~5 min on 8 cores)
Writes results/hang_summary.json, results/hang_tables.md, hang_s1_example.png and hang_flight_example.png.

- S1 swing: the line swings ±10° (free pendulum, periods 3-10 s, planar along a body axis, planar on
  the diagonal, and elliptical 10° × 5°); the payload holds level within ±2°.
- S2 tilt: command 15° at four azimuths (slewed at 1°/s), hold within ±2° for 60 s or more; also with
  a 10° swing at a 7 s period on top.
- S3 unbalance: a 20 g mass moves 30 mm (0.23 mm centre-of-mass step) in four directions; back within
  ±1° of level within 60 s.
- Flight Monte Carlo: random balloon (top-of-line) motion scaled to 3°, 6° and 10° RMS line swing
  (TSat-1's estimated swing reached up to 26° peak; see section 2 of RESULTS.md), line 5-20 m,
  eye height 2-6 mm, lateral centre-of-mass error up to 0.5 mm, yaw gusts; 600 s per run.
Each test runs over eye heights d = 2-6 mm (design 4 mm; the controller always assumes 4 mm).
"""
from __future__ import annotations

import json
import math
from concurrent.futures import ProcessPoolExecutor

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from . import model as hm
from .model import DEG, Scenario, simulate, wheels, OUT

D_SET = np.array([2, 3, 4, 5, 6]) * 1e-3
PERIODS = (3.0, 5.0, 7.0, 10.0)
PATTERNS = ("planar, body axis", "planar, diagonal", "elliptical 10° × 5°")
LEVELS_DEG = (3.0, 6.0, 10.0)
SEEDS = 8
T_FLIGHT = 600.0
UB = 0.020 * 0.030 / hm.M      # centre-of-mass step from 20 g moved 30 mm


def L_of(P):
    return hm.G * (P / (2 * math.pi)) ** 2


def zeros2(N):
    return np.zeros((N, 2))


# ------------------------------------------------------------------ scenario builders
def s1_cases():
    rows, d, L, phi0, phid = [], [], [], [], []
    for P in PERIODS:
        for pat in PATTERNS:
            for dd in D_SET:
                w = 2 * math.pi / P
                if pat == "planar, body axis":
                    a, b = (10 * DEG, 0.0), (0.0, 0.0)
                elif pat == "planar, diagonal":
                    a, b = (10 * DEG / math.sqrt(2),) * 2, (0.0, 0.0)
                else:
                    a, b = (10 * DEG, 0.0), (0.0, 5 * DEG * w)
                rows.append(dict(period_s=P, pattern=pat, d_mm=dd * 1e3))
                d.append(dd); L.append(L_of(P)); phi0.append(a); phid.append(b)
    N = len(rows)
    return rows, dict(T=60.0, d=np.array(d), L=np.array(L), c0=zeros2(N), phi0=np.array(phi0), phidot0=np.array(phid))


def s2_cases():
    rows, d, L, phi0, az = [], [], [], [], []
    for swing in (False, True):
        for a in (0, 45, 90, 180):
            for dd in D_SET:
                rows.append(dict(azimuth_deg=a, d_mm=dd * 1e3, swing=swing))
                d.append(dd); L.append(L_of(7.0)); az.append(a * DEG)
                phi0.append((10 * DEG, 0.0) if swing else (0.0, 0.0))
    N = len(rows)
    return rows, dict(T=120.0, d=np.array(d), L=np.array(L), c0=zeros2(N), phi0=np.array(phi0),
                      phidot0=zeros2(N), tilt_target=np.full(N, 15 * DEG), tilt_az=np.array(az), t_tilt=5.0)


def s3_cases():
    rows, d, ub = [], [], []
    for a in (0, 90, 180, 225):
        for dd in D_SET:
            rows.append(dict(direction_deg=a, d_mm=dd * 1e3))
            d.append(dd); ub.append((UB * math.cos(a * DEG), UB * math.sin(a * DEG)))
    N = len(rows)
    return rows, dict(T=120.0, d=np.array(d), L=np.full(N, L_of(7.0)), c0=zeros2(N), phi0=zeros2(N),
                      phidot0=zeros2(N), unbalance=np.array(ub), t_unbal=10.0)


def flight_cases(sigma_per_deg):
    rng = np.random.default_rng(42)
    rows, d, L, c0, sig = [], [], [], [], []
    for lvl in LEVELS_DEG:
        for s in range(SEEDS):
            dd = rng.uniform(2e-3, 6e-3)
            LL = math.exp(rng.uniform(math.log(5), math.log(20)))
            cc = rng.uniform(-0.5e-3, 0.5e-3, 2)
            rows.append(dict(level_deg=lvl, seed=s, d_mm=dd * 1e3, L_m=LL))
            d.append(dd); L.append(LL); c0.append(cc); sig.append(sigma_per_deg * lvl)
    N = len(rows)
    return rows, dict(T=T_FLIGHT, d=np.array(d), L=np.array(L), c0=np.array(c0), phi0=zeros2(N),
                      phidot0=zeros2(N), top_sigma_v=np.array(sig), yaw_gust=True, seed=7)


MODES = {
    "wheels + trim": dict(),
    "trim only (yaw wheel on)": dict(wheels_on=False),
    "uncontrolled": dict(wheels_on=False, yaw_on=False, trim_on=False),
}


# ------------------------------------------------------------------ workers
def _window(rec, t0, t1):
    t = rec["t"]
    sel = (t >= t0) & (t <= t1)
    return sel


def run_job(job):
    test, wi, mode, kw, extra = job
    wh = wheels()[wi]
    sc = Scenario(**kw, **MODES[mode])
    st, rec = simulate(sc, wh)
    out = {k: np.asarray(v).tolist() for k, v in st.items()}
    t = rec["t"]
    tilt = rec["tilt"] / DEG
    if test == "S2":
        sel = (t >= 30) & (t <= sc.T)
        out["hold_peak_deg"] = tilt[sel].max(axis=0).tolist()
        out["slew_peak_deg"] = tilt[(t >= 5) & (t < 30)].max(axis=0).tolist()
    if test == "S3":
        sel = t >= sc.t_unbal
        out["after_peak_deg"] = tilt[sel].max(axis=0).tolist()
        bad = (tilt > 1.0) & sel[:, None]
        last = np.where(bad.any(axis=0), t[np.argmax(np.cumsum(bad, axis=0) == bad.sum(axis=0), axis=0)], sc.t_unbal)
        out["settle_s"] = np.maximum(0.0, last - sc.t_unbal).tolist()
    keep = None
    if extra is not None:
        i = extra
        keep = dict(t=t.tolist(), tilt=tilt[:, i].tolist(), tilt_est=(rec["tilt_est"][:, i] / DEG).tolist(),
                    line=(rec["line"][:, i] / DEG).tolist(), h=(rec["h"][:, i] * 1e3).tolist(),
                    i=rec["i"][:, i].tolist(), trim=(rec["trim"][:, i] * 1e3).tolist())
    return test, wh.name, mode, out, keep


def calibrate_top():
    """Line-angle RMS per unit top-velocity sigma (linear), uncontrolled, 300 s."""
    N = 8
    rng = np.random.default_rng(3)
    L = np.exp(rng.uniform(math.log(5), math.log(20), N))
    kw = dict(T=300.0, d=np.full(N, 4e-3), L=L, c0=zeros2(N), phi0=zeros2(N), phidot0=zeros2(N),
              top_sigma_v=np.full(N, 0.1), seed=3, **MODES["uncontrolled"])
    st, _ = simulate(Scenario(**kw), wheels()[0], t_skip=60.0)
    deg_per_sigma = float(np.median(st["line_rms_deg"])) / 0.1
    return 1.0 / deg_per_sigma, float(np.median(st["line_rms_deg"]))


# ------------------------------------------------------------------ main
def main():
    OUT.mkdir(exist_ok=True)
    W = wheels()
    sig_per_deg, rms01 = calibrate_top()
    print(f"top velocity sigma per degree of RMS line swing: {sig_per_deg:.4f} m/s (0.1 m/s gave {rms01:.2f}°)", flush=True)
    s1r, s1k = s1_cases()
    s2r, s2k = s2_cases()
    s3r, s3k = s3_cases()
    flr, flk = flight_cases(sig_per_deg)
    ex_s1 = next(i for i, r in enumerate(s1r) if r["period_s"] == 7.0 and r["pattern"].startswith("ellip") and r["d_mm"] == 4.0)
    ex_fl = next(i for i, r in enumerate(flr) if r["level_deg"] == 10.0 and r["seed"] == 0)
    jobs = []
    for wi in range(len(W)):
        jobs.append(("S1", wi, "wheels + trim", s1k, ex_s1 if wi == 1 else None))
        jobs.append(("flight", wi, "wheels + trim", flk, ex_fl if wi == 1 else None))
    jobs.append(("S1", 0, "uncontrolled", s1k, None))
    for wi in (0, 1):
        jobs.append(("S2", wi, "wheels + trim", s2k, None))
        jobs.append(("S3", wi, "wheels + trim", s3k, None))
    for mode in ("trim only (yaw wheel on)", "uncontrolled"):
        jobs.append(("S2", 1, mode, s2k, None))
        jobs.append(("S3", 1, mode, s3k, None))
        jobs.append(("flight", 1, mode, flk, ex_fl if mode.startswith("trim") else None))
    results, examples = [], {}
    with ProcessPoolExecutor(max_workers=8) as ex:
        for test, wname, mode, out, keep in ex.map(run_job, jobs):
            print("done", test, wname, mode, flush=True)
            results.append(dict(test=test, wheel=wname, mode=mode, stats=out))
            if keep is not None:
                examples[f"{test}|{mode}"] = keep
    summary = dict(assumptions=dict(M_kg=hm.M, I_body=hm.I_BODY.tolist(), d_nom_mm=hm.D_NOM * 1e3,
                                    trim_mass_kg=hm.M_TRIM, trim_travel_mm=hm.TRIM_TRAVEL * 1e3,
                                    trim_speed_mm_s=hm.TRIM_SPEED * 1e3, loop_Hz=100, wc=hm.WC,
                                    ahrs_tau_s=1 / hm.AHRS_KP, top_sigma_per_deg=sig_per_deg),
                   cases=dict(S1=s1r, S2=s2r, S3=s3r, flight=flr), results=results)
    (OUT / "hang_summary.json").write_text(json.dumps(summary))
    tables(summary)
    plots(summary, examples)


# ------------------------------------------------------------------ reporting
def _get(summary, test, mode, wheel_prefix=None):
    for r in summary["results"]:
        if r["test"] == test and r["mode"] == mode and (wheel_prefix is None or r["wheel"].startswith(wheel_prefix)):
            return r
    return None


def tables(summary):
    C = summary["cases"]
    W = [w.name for w in wheels()]
    md = ["# KAOS-3 hanging payload: pitch/roll tests\n",
          "Generated by `hang/run.py` (model in `hang/model.py`). True attitude error unless noted; "
          "the controller assumes d = 4 mm.\n"]

    # ---- S1
    md.append("## S1: hold level while the line swings ±10°\n")
    md.append("Peak tilt error (°) after 5 s, worst over the three swing patterns. Pass is under 2°. "
              "`sat` marks a wheel that reached 95% of its speed limit; `*` marks a run where the current limit was hit.\n")
    hdr = "| Wheel | Period s | " + " | ".join(f"d = {d:g} mm" for d in D_SET * 1e3) + " |"
    md += [hdr, "|---|---|" + "---|" * len(D_SET)]
    for w in W + ["uncontrolled"]:
        r = _get(summary, "S1", "uncontrolled") if w == "uncontrolled" else _get(summary, "S1", "wheels + trim", w)
        st = r["stats"]
        for P in PERIODS:
            cells = []
            for dd in D_SET * 1e3:
                idx = [i for i, c in enumerate(C["S1"]) if c["period_s"] == P and abs(c["d_mm"] - dd) < 1e-6]
                pk = max(st["tilt_peak_deg"][i] for i in idx)
                sat = any(st["saturated"][i] for i in idx)
                ilim = w != "uncontrolled" and any(max(st["i_peak_A"][i][:2]) >= wheels()[W.index(w)].i_max * 0.999 for i in idx)
                cells.append(f"{pk:.1f}{' sat' if sat else ''}{'*' if ilim else ''}")
            md.append(f"| {w.split(' (')[0] if w != 'uncontrolled' else 'no control (free payload)'} | {P:g} | " + " | ".join(cells) + " |")
    r = _get(summary, "S1", "wheels + trim", W[1])["stats"]
    md.append("\nDemand on the pitch/roll wheels, 2 A driver (worst over patterns and periods):\n")
    md.append("| d mm | Peak wheel momentum mN·m·s | Peak current A | Mean copper + eddy W (all wheels) | Peak estimator error ° |")
    md.append("|---|---|---|---|---|")
    for dd in D_SET * 1e3:
        idx = [i for i, c in enumerate(C["S1"]) if abs(c["d_mm"] - dd) < 1e-6]
        h = max(max(r["h_peak_mNms"][i][:2]) for i in idx)
        ip = max(max(r["i_peak_A"][i][:2]) for i in idx)
        pw = max(sum(r["p_cu_W"][i]) + sum(r["p_eddy_W"][i]) for i in idx)
        es = max(r["est_err_peak_deg"][i] for i in idx)
        md.append(f"| {dd:g} | {h:.0f} | {ip:.2f} | {pw:.2f} | {es:.2f} |")

    # ---- S2
    md.append("\n## S2: hold a 15° tilt\n")
    md.append("Peak tilt error (°) during the hold (t = 30-120 s; the slew ends at 20 s), worst over four azimuths. "
              "Peak trim travel in brackets (mm, limit 15).\n")
    md += ["| Mode | Swing | " + " | ".join(f"d = {d:g} mm" for d in D_SET * 1e3) + " |", "|---|---|" + "---|" * len(D_SET)]
    rows2 = [(f"wheels + trim, {W[0].split(',')[1].strip().split(' (')[0]}", _get(summary, "S2", "wheels + trim", W[0])),
             (f"wheels + trim, {W[1].split(',')[1].strip()}", _get(summary, "S2", "wheels + trim", W[1])),
             ("trim only", _get(summary, "S2", "trim only (yaw wheel on)")),
             ("no control", _get(summary, "S2", "uncontrolled"))]
    for label, r in rows2:
        st = r["stats"]
        for swing in (False, True):
            cells = []
            for dd in D_SET * 1e3:
                idx = [i for i, c in enumerate(C["S2"]) if c["swing"] == swing and abs(c["d_mm"] - dd) < 1e-6]
                pk = max(st["hold_peak_deg"][i] for i in idx)
                tr = max(st["trim_peak_mm"][i] for i in idx)
                cells.append(f"{pk:.1f} ({tr:.0f})")
            md.append(f"| {label} | {'10°, 7 s' if swing else 'none'} | " + " | ".join(cells) + " |")

    # ---- S3
    md.append("\n## S3: recover level after a 20 g mass moves 30 mm\n")
    md.append("Peak tilt after the step (°) / time until the tilt stays within 1° (s), worst over four directions. "
              "Pass is back within 1° within 60 s.\n")
    md += ["| Mode | " + " | ".join(f"d = {d:g} mm" for d in D_SET * 1e3) + " | Peak wheel momentum mN·m·s |",
           "|---|" + "---|" * len(D_SET) + "---|"]
    for label, r in [("wheels + trim, 1 A", _get(summary, "S3", "wheels + trim", W[0])),
                     ("wheels + trim, 2 A", _get(summary, "S3", "wheels + trim", W[1])),
                     ("trim only", _get(summary, "S3", "trim only (yaw wheel on)")),
                     ("no control", _get(summary, "S3", "uncontrolled"))]:
        st = r["stats"]
        cells = []
        for dd in D_SET * 1e3:
            idx = [i for i, c in enumerate(C["S3"]) if abs(c["d_mm"] - dd) < 1e-6]
            cells.append(f"{max(st['after_peak_deg'][i] for i in idx):.1f} / {max(st['settle_s'][i] for i in idx):.0f}")
        hpk = max(max(h[:2]) for h in st["h_peak_mNms"])
        md.append(f"| {label} | " + " | ".join(cells) + f" | {hpk:.0f} |")

    # ---- flight
    md.append("\n## Flight Monte Carlo (600 s, 8 runs per level)\n")
    md.append("Random top-of-line motion scaled to the RMS line swing shown; line 5-20 m, d 2-6 mm, lateral "
              "centre-of-mass error up to 0.5 mm, 1 mN·m RMS yaw gusts. Time within 2° of level (median / worst), "
              "tilt RMS (median), runs where a wheel reached 95% of its speed limit, peak pitch/roll momentum, "
              "mean electrical input for all three wheels.\n")
    md += ["| Configuration | Line swing RMS | Within 2° % | Tilt RMS ° | Saturated runs | Peak momentum mN·m·s | Mean W | Peak current A |",
           "|---|---|---|---|---|---|---|---|"]
    fl = [(w.split(" (")[0], _get(summary, "flight", "wheels + trim", w)) for w in W]
    fl += [("Trim only (yaw wheel on)", _get(summary, "flight", "trim only (yaw wheel on)")),
           ("No control", _get(summary, "flight", "uncontrolled"))]
    for label, r in fl:
        st = r["stats"]
        for lvl in LEVELS_DEG:
            idx = [i for i, c in enumerate(C["flight"]) if c["level_deg"] == lvl]
            w2 = [st["within2_pct"][i] for i in idx]
            md.append(f"| {label} | {np.median([st['line_rms_deg'][i] for i in idx]):.1f}° | "
                      f"{np.median(w2):.0f} / {min(w2):.0f} | {np.median([st['tilt_rms_deg'][i] for i in idx]):.2f} | "
                      f"{sum(bool(st['saturated'][i]) for i in idx)}/{len(idx)} | "
                      f"{max(max(st['h_peak_mNms'][i][:2]) for i in idx):.0f} | "
                      f"{np.median([st['p_in_W'][i] for i in idx]):.2f} | "
                      f"{max(max(st['i_peak_A'][i]) for i in idx):.2f} |")
    (OUT / "hang_tables.md").write_text("\n".join(md) + "\n")
    print("\n".join(md))


def plots(summary, ex):
    e = ex.get("S1|wheels + trim")
    if e:
        t = np.array(e["t"])
        fig, axs = plt.subplots(4, 1, figsize=(9, 9), sharex=True)
        axs[0].plot(t, e["line"], label="line angle from vertical")
        axs[0].plot(t, e["tilt"], label="payload tilt error (true)")
        axs[0].plot(t, e["tilt_est"], label="estimator error", lw=0.8)
        axs[0].axhline(2, color="k", ls=":", lw=0.8)
        axs[0].set_ylabel("deg")
        axs[0].legend(fontsize=8)
        h = np.array(e["h"])
        for j, lab in enumerate("xyz"):
            axs[1].plot(t, h[:, j], label=f"wheel {lab}")
        axs[1].set_ylabel("wheel momentum\n[mN·m·s]")
        axs[1].legend(fontsize=8)
        i = np.array(e["i"])
        for j, lab in enumerate("xyz"):
            axs[2].plot(t, i[:, j], label=f"wheel {lab}")
        axs[2].set_ylabel("current [A]")
        tr = np.array(e["trim"])
        axs[3].plot(t, tr[:, 0], label="trim x")
        axs[3].plot(t, tr[:, 1], label="trim y")
        axs[3].set_ylabel("trim mass [mm]")
        axs[3].set_xlabel("time [s]")
        axs[3].legend(fontsize=8)
        axs[0].set_title("S1: elliptical 10° × 5° swing, 7 s period, d = 4 mm, JLC 4L wheels, 2 A driver")
        fig.tight_layout()
        fig.savefig(OUT / "hang_s1_example.png", dpi=150)
        plt.close(fig)
    a, b = ex.get("flight|wheels + trim"), ex.get("flight|trim only (yaw wheel on)")
    if a and b:
        fig, axs = plt.subplots(2, 1, figsize=(10, 6), sharex=True)
        t = np.array(a["t"])
        axs[0].plot(t, a["line"], lw=0.6, color="0.5", label="line angle")
        axs[0].plot(t, b["tilt"], lw=0.6, label="trim only: tilt")
        axs[0].plot(t, a["tilt"], lw=0.8, label="wheels + trim: tilt")
        axs[0].axhline(2, color="k", ls=":", lw=0.8)
        axs[0].set_ylabel("deg")
        axs[0].legend(fontsize=8)
        h = np.array(a["h"])
        for j, lab in enumerate("xyz"):
            axs[1].plot(t, h[:, j], lw=0.6, label=f"wheel {lab}")
        axs[1].set_ylabel("wheel momentum [mN·m·s]")
        axs[1].set_xlabel("time [s]")
        axs[1].legend(fontsize=8)
        axs[0].set_title("Flight Monte Carlo, 10° RMS swing, seed 0, JLC 4L wheels, 2 A driver")
        fig.tight_layout()
        fig.savefig(OUT / "hang_flight_example.png", dpi=150)
        plt.close(fig)


if __name__ == "__main__":
    main()
