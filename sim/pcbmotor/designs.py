"""PCB motor design study: quick test stator and integrated 1U payload stator.

Run from sim/:  .venv/bin/python -m pcbmotor.designs
Writes results/pcbmotor_sweep.csv, results/pcbmotor_designs.json and plots.
"""
from __future__ import annotations

import csv
import json
import math
import pathlib
from dataclasses import asdict, replace

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from .geometry import StatorDesign, RotorDesign, coil_turns, coil_angles, phase_map
from .em import analyse, foc_torque

OUT = pathlib.Path(__file__).resolve().parents[1] / "results"

V_BUS = 11.1                 # 3S LiPo nominal
H_CONV = 10.0                # W/m^2/K natural convection, both faces (conservative: ignores rotor airflow)
DT_MAX_K = 40.0              # allowed continuous board temperature rise
RPM_CAP = 6000.0             # mechanical cap for a hand-balanced rotor on small ball bearings
PWM_FREQS = (20e3, 50e3, 100e3)


def drive_metrics(kt, r_ph, l_ph, ke, inertia, board_area_m2, i_lim, v_bus=V_BUS):
    """Steady-state drive envelope for sinusoidal (FOC/SVPWM) drive.
    Phase voltage amplitude available ~ Vbus/sqrt(3). kt is per amp of PEAK phase current."""
    v_ph = v_bus / math.sqrt(3)
    i_stall = min(i_lim, v_ph / r_ph)                 # short-duration peak (driver / voltage limited)
    tau_stall = kt * i_stall
    p_stall = 1.5 * r_ph * i_stall ** 2
    p_th = DT_MAX_K * H_CONV * 2 * board_area_m2      # copper loss allowed continuously
    i_cont = min(i_stall, math.sqrt(p_th / (1.5 * r_ph)))
    tau_cont = kt * i_cont
    w_noload = v_ph / ke
    w_max = min(w_noload, RPM_CAP * 2 * math.pi / 60)
    w_half = 0.5 * w_max
    i_half = max(0.0, min(i_lim, (v_ph - ke * w_half) / r_ph))
    km = kt / math.sqrt(1.5 * r_ph)                   # Nm / sqrt(W)
    ripple = {f"ripple_pp_A_{int(f/1e3)}kHz": v_bus / (8 * l_ph * f) for f in PWM_FREQS}
    return dict(
        i_stall_A=i_stall, tau_stall_mNm=tau_stall * 1e3, p_stall_W=p_stall,
        i_cont_A=i_cont, tau_cont_mNm=tau_cont * 1e3, p_cont_W=1.5 * r_ph * i_cont ** 2,
        dT_stall_K=p_stall / (H_CONV * 2 * board_area_m2),
        noload_rpm=w_noload * 60 / (2 * math.pi), usable_max_rpm=w_max * 60 / (2 * math.pi),
        tau_at_half_speed_mNm=kt * i_half * 1e3,
        Km_mNm_per_sqrtW=km * 1e3, W_per_mNm2=1.0 / (km * 1e3) ** 2,
        H_max_mNms=inertia * w_max * 1e3, elec_tau_us=l_ph / r_ph * 1e6, **ripple)


# ---------------------------------------------------------------- design spaces
def test_space():
    """~40 mm round test stator. Cylinder magnets (cheap, stocked)."""
    out = []
    combos = [(6, 8, 12.75, [(8.0, 2.0), (8.0, 3.0)]),
              (9, 12, 12.75, [(6.0, 2.0), (6.0, 3.0)]),
              (12, 16, 12.75, [(4.0, 2.0)])]
    for layers in (4, 6):
        for tw in (0.127, 0.15, 0.2, 0.3):
            for nc, npol, rm, mags in combos:
                for md, mh in mags:
                    d = StatorDesign(f"test_{layers}L_{nc}c{npol}p_w{tw}_m{md}x{mh}", layers=layers, n_coils=nc,
                                     r_in_mm=6.5, r_out_mm=19.0, trace_mm=tw, space_mm=0.15 if tw >= 0.15 else 0.127,
                                     board_od_mm=42.0)
                    r = RotorDesign(n_poles=npol, magnet_shape="cyl", magnet_d_mm=md, magnet_h_mm=mh,
                                    magnet_r_mm=rm, airgap_mm=0.5, back_iron=True, back_iron_t_mm=1.5,
                                    disc_od_mm=40.0, disc_t_mm=0.0)
                    out.append(("test", d, r, math.pi * (21e-3) ** 2, 1.0))
    return out


PAYLOAD_STACKUPS = ("osh4", "osh6", "jlc4_1oz", "jlc6_1oz", "jlc4_2oz_out")


def payload_space():
    """1U payload stator on a ~90x90 mm board, winding OD 82 mm. Stock N42 block magnets
    (3/4 in long, 1/8 or 1/4 in thick) on a 1.5 mm steel disc that is back iron + flywheel.
    Compared across OSH Park and JLCPCB stackups; space fixed at 0.15 mm (meets every fab's minimum,
    including JLC 2 oz)."""
    out = []
    combos = [(6, 8, 12.7), (9, 12, 9.53), (12, 16, 6.35)]
    for st in PAYLOAD_STACKUPS:
        for tw in (0.2, 0.3, 0.5, 0.8, 1.2):
            for nc, npol, bw in combos:
                for mh in (3.18, 6.35):
                    d = StatorDesign(f"payload_{st}_{nc}c{npol}p_w{tw}_h{mh}", stackup=st, n_coils=nc,
                                     r_in_mm=21.0, r_out_mm=41.0, trace_mm=tw, space_mm=0.15,
                                     board_od_mm=90.0)
                    r = RotorDesign(n_poles=npol, magnet_shape="block", block_len_mm=19.05, block_w_mm=bw,
                                    magnet_h_mm=mh, magnet_r_mm=31.0, airgap_mm=0.6, back_iron=True,
                                    back_iron_t_mm=1.5, disc_od_mm=84.0, disc_t_mm=0.0)
                    out.append(("payload", d, r, 0.090 * 0.090, 1.0))
    return out


def run_sweep():
    OUT.mkdir(exist_ok=True)
    rows = []
    for kind, d, r, area, ilim in test_space() + payload_space():
        res, _ = analyse(d, r, n_theta=36)
        m = drive_metrics(res.Kt_Nm_per_A, res.R_phase_ohm, res.L_phase_uH * 1e-6,
                          res.Ke_phase_V_per_rad_s, res.rotor_inertia_kgm2, area, ilim)
        row = dict(kind=kind, name=d.name, stackup=d.stackup or f"osh{d.layers}", fab=d.fab, layers=d.layers, coils=d.n_coils, poles=r.n_poles,
                   trace_mm=d.trace_mm, turns_per_layer=res.turns_per_layer,
                   Kt_mNm_per_A=res.Kt_Nm_per_A * 1e3, R_phase_ohm=res.R_phase_ohm,
                   L_phase_uH=res.L_phase_uH, ripple_pct=res.torque_ripple_pct,
                   rotor_J_kgm2=res.rotor_inertia_kgm2, rotor_mass_g=res.rotor_mass_kg * 1e3, **m)
        rows.append(row)
        print(f"{d.name:42s} Kt={row['Kt_mNm_per_A']:6.2f} R={row['R_phase_ohm']:7.1f} "
              f"stall={row['tau_stall_mNm']:6.2f}mNm Km={row['Km_mNm_per_sqrtW']:5.2f} "
              f"n0={row['noload_rpm']:7.0f} H={row['H_max_mNms']:6.2f}", flush=True)
    with open(OUT / "pcbmotor_sweep.csv", "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0]))
        w.writeheader()
        w.writerows(rows)
    return rows


# ---------------------------------------------------------------- plots
PH_COL = ["tab:red", "tab:green", "tab:blue"]


def plot_layout(d, r, path):
    fig, ax = plt.subplots(figsize=(6, 6))
    pm = phase_map(d, r.n_poles)
    for (ph, sgn), phi in zip(pm, coil_angles(d)):
        c, s = math.cos(phi), math.sin(phi)
        for poly in coil_turns(d):
            x = poly[:, 0] * c - poly[:, 1] * s
            y = poly[:, 0] * s + poly[:, 1] * c
            ax.plot(x, y, color=PH_COL[ph], lw=0.4)
        ax.text(0.5 * (d.r_in_mm + d.r_out_mm) * c, 0.5 * (d.r_in_mm + d.r_out_mm) * s,
                "ABC"[ph] + ("+" if sgn > 0 else "-"), ha="center", va="center", fontsize=9, weight="bold")
    for k in range(r.n_poles):
        a = k * 2 * math.pi / r.n_poles
        if r.magnet_shape == "block":
            L, W = r.block_len_mm, r.block_w_mm
            pts = np.array([[-L / 2, -W / 2], [L / 2, -W / 2], [L / 2, W / 2], [-L / 2, W / 2], [-L / 2, -W / 2]])
            pts[:, 0] += r.magnet_r_mm
        else:
            t = np.linspace(0, 2 * math.pi, 60)
            pts = np.c_[r.magnet_r_mm + r.magnet_d_mm / 2 * np.cos(t), r.magnet_d_mm / 2 * np.sin(t)]
        x = pts[:, 0] * math.cos(a) - pts[:, 1] * math.sin(a)
        y = pts[:, 0] * math.sin(a) + pts[:, 1] * math.cos(a)
        ax.plot(x, y, "k--", lw=0.8)
    if d.board_od_mm >= 80:
        h = d.board_od_mm / 2
        ax.plot([-h, h, h, -h, -h], [-h, -h, h, h, -h], "k", lw=1)
    else:
        t = np.linspace(0, 2 * math.pi, 200)
        ax.plot(d.board_od_mm / 2 * np.cos(t), d.board_od_mm / 2 * np.sin(t), "k", lw=1)
    ax.set_aspect("equal")
    ax.set_title(f"{d.name}\n{d.layers} layers, {len(coil_turns(d))} turns/layer/coil, dashed = magnets (rotor)")
    ax.set_xlabel("mm")
    fig.tight_layout()
    fig.savefig(path, dpi=150)
    plt.close(fig)


def plot_waveforms(d, r, curve, path):
    th, tq = curve
    kt, ripple, thp, kph = foc_torque(d, r, th, tq)
    pp = r.n_poles // 2
    fig, ax = plt.subplots(2, 1, figsize=(7, 6), sharex=True)
    ed = np.degrees(thp * pp)
    for k in range(3):
        ax[0].plot(ed, kph[k] * 1e3, color=PH_COL[k], label=f"phase {'ABC'[k]}")
    ax[0].set_ylabel("phase torque const\n= back-EMF [mV/(rad/s)]")
    ax[0].legend()
    best = None
    for delta in np.linspace(0, 2 * math.pi, 180, endpoint=False):
        i = np.array([np.cos(pp * thp + delta - k * 2 * math.pi / 3) for k in range(3)])
        t = np.sum(kph * i, axis=0)
        if best is None or t.mean() > best.mean():
            best = t
    ax[1].plot(ed, best * 1e3, "k")
    ax[1].set_ylabel("torque at 1 A peak,\nsinusoidal drive [mNm]")
    ax[1].set_xlabel("electrical angle [deg]")
    ax[1].set_ylim(0, best.max() * 1.2 * 1e3)
    fig.suptitle(d.name)
    fig.tight_layout()
    fig.savefig(path, dpi=150)
    plt.close(fig)


def find(space, name):
    for kind, d, r, area, ilim in space:
        if d.name == name:
            return d, r, area, ilim
    raise KeyError(name)


def load_sweep():
    rows = []
    with open(OUT / "pcbmotor_sweep.csv") as f:
        for r in csv.DictReader(f):
            for k, v in r.items():
                try:
                    r[k] = float(v) if k not in ("kind", "name", "stackup", "fab") else v
                except ValueError:
                    pass
            rows.append(r)
    return rows


def main():
    import sys
    rows = load_sweep() if "--reuse" in sys.argv and (OUT / "pcbmotor_sweep.csv").exists() else run_sweep()
    # Recommended designs (rationale in results/RESULTS.md). Km (torque per sqrt(W)) is nearly
    # independent of trace width, so width is chosen for speed range, current level and PWM ripple.
    def best(stackups, min_rpm=2000, mh=3.18, km_frac=0.9):
        """Torque is never the binding requirement (yaw sim needs < 6 mNm); stored momentum is.
        So among designs within km_frac of the best Km, take the one with the most momentum."""
        c = [x for x in rows if x["kind"] == "payload" and x["stackup"] in stackups
             and x["noload_rpm"] >= min_rpm and abs(x["name_mh"] - mh) < 1e-6]
        km_best = max(x["Km_mNm_per_sqrtW"] for x in c)
        c = [x for x in c if x["Km_mNm_per_sqrtW"] >= km_frac * km_best]
        return max(c, key=lambda x: (round(x["H_max_mNms"], 1), x["Km_mNm_per_sqrtW"]))
    for x in rows:
        x["name_mh"] = float(x["name"].rsplit("_h", 1)[1]) if x["kind"] == "payload" else 0.0
    picks = {"test": next(x for x in rows if x["name"] == "test_4L_6c8p_w0.15_m8.0x3.0"),
             "payload": best(("jlc4_1oz", "jlc6_1oz", "jlc4_2oz_out")),
             "payload_osh6": best(("osh6",))}
    space = test_space() + payload_space()
    out = {}
    for key, row in picks.items():
        d, r, area, ilim = find(space, row["name"])
        res, curve = analyse(d, r, n_theta=72)
        plot_layout(d, r, OUT / f"pcbmotor_{key}_layout.png")
        plot_waveforms(d, r, curve, OUT / f"pcbmotor_{key}_waveforms.png")
        out[key] = dict(row=row, stator=asdict(d), rotor=asdict(r), result=res.as_dict())
    # High-momentum payload variant (recommended after the TSat-2B video calibration, see RESULTS.md):
    # the sweep's highest-momentum cheap-fab design with 1.5 mm of extra steel on the 84 mm disc (3 mm total).
    hm = next(x for x in rows if x["name"] == "payload_jlc4_2oz_out_6c8p_w1.2_h6.35")
    d, r, area, ilim = find(space, hm["name"])
    r.disc_t_mm = 1.5
    res, curve = analyse(d, r, n_theta=72)
    plot_layout(d, r, OUT / "pcbmotor_payload_high_momentum_layout.png")
    plot_waveforms(d, r, curve, OUT / "pcbmotor_payload_high_momentum_waveforms.png")
    out["payload_high_momentum"] = dict(row=hm, stator=asdict(d), rotor=asdict(r), result=res.as_dict())
    (OUT / "pcbmotor_designs.json").write_text(json.dumps(out, indent=2))
    for k, v in out.items():
        print(k, v["row"]["name"], {kk: round(vv, 3) if isinstance(vv, float) else vv for kk, vv in v["row"].items()})


if __name__ == "__main__":
    main()
