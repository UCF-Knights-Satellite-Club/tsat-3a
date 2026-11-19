"""PCB-motor reaction wheels sized for CubeSats, against COTS wheels in the same footprint.

Run from sim/:  .venv/bin/python -m pcbmotor.cubesat_wheels
Writes results/pcbmotor_cubesat_wheels.json and results/pcbmotor_cubesat_wheels.md.

For each footprint of a common COTS wheel (46, 58, 70 mm square) and the full 1U width (90 mm), a
PCB-motor wheel is laid out and swept over magnet thickness, flywheel disc thickness and a steel
rim (width and height), for two stators: a cheap JLCPCB 4-layer board and an ACE-style 6-layer 3 oz board, both with
stranded, transposed windings (pcbmotor/ace_stackups.py). The design that
is no taller than the COTS wheel it is compared with is reported: the most momentum that fits the same box.

Model (same EM model as pcbmotor/em.py and stacking.py):
- Geometry scales with the rotor radius: 6 coils / 8 poles, N42 blocks 0.465 R long and 0.31 R
  wide centred at 0.756 R (the proportions of the TSat-3A payload rotor), winding from 0.51 R to R.
  R is the magnet/winding outer radius; an optional steel rim sits outside it.
- Rotor: 1.5 mm steel back iron (also flywheel), optional extra steel disc, optional steel rim ring
  as tall as the magnets. Max speed 6,000 rpm (CubeSpace CubeWheels are rated at 6,000 rpm), 12 V
  bus, 2 A driver limit. Rated momentum = speed where 2 mN·m of torque is still available after eddy
  and bearing drag.
- Steady power at rated momentum = eddy loss in the traces + bearing drag (yaw/candidates.py
  defaults: 0.02 mN·m Coulomb + 2e-8 N·m/(rad/s)) + copper loss to hold that drag + 0.15 W for the
  controller (COTS figures include their electronics).
- Mass and height are ESTIMATES for a complete wheel: rotor, stator board with the driver on it,
  hub/shaft/bearings, and a 0.8 mm aluminium cover and walls. COTS numbers are datasheet values.
"""
from __future__ import annotations

import json
import math
import pathlib
from dataclasses import replace

import numpy as np

from .geometry import StatorDesign, RotorDesign, STACKUPS, OZ_TO_M
from .em import foc_torque, phase_resistance, phase_inductance, copper_mass_g
from .stacking import coil_curves
from . import ace_stackups  # noqa: F401  (registers the ace* stackups)

OUT = pathlib.Path(__file__).resolve().parents[1] / "results"
V_BUS = 12.0
I_LIM = 2.0
RPM_CAP = 6000.0
MARGIN_Nm = 2e-3
BEARING_C, BEARING_V = 2e-5, 2e-8
P_ELEC_W = 0.15
STEEL, AL, FR4, NDFEB = 7850.0, 2700.0, 1850.0, 7500.0

# COTS wheels (datasheet / vendor listing values; sources in COTS_SOURCES).
COTS = [
    dict(name="CubeSpace CW0057", H=5.7, tau=2.0, mass_g=115, size="", P_W=0.77, side_mm=None, h_mm=None),
    dict(name="CubeSpace CW0162", H=16.2, tau=7.0, mass_g=144, size="46×46×24.2", P_W=0.77, side_mm=46, h_mm=24.2),
    dict(name="Rocket Lab RW-0.01", H=10.0, tau=1.0, mass_g=122, size="", P_W=None, side_mm=None, h_mm=None),
    dict(name="Rocket Lab RW-0.03", H=30.0, tau=2.0, mass_g=183, size="50×50×40", P_W=0.3, side_mm=50, h_mm=40,
         note="0.3 W steady at 40 mN·m·s; torque 2 mN·m at 20, 0.5 at 40"),
    dict(name="BCT RWp050", H=50.0, tau=7.0, mass_g=240, size="58×58×25", P_W=1.0, side_mm=58, h_mm=25),
    dict(name="BCT RWp100", H=100.0, tau=7.0, mass_g=330, size="70×70×25", P_W=0.89, side_mm=70, h_mm=25),
]

COTS_SOURCES = [
    "CubeSpace CW0057 / CW0162: CubeSatShop and satsearch listings (checked 2026-11-18)",
    "Rocket Lab RW-0.03: Rocket Lab data sheet v3.0 (2024-08-02); RW-0.01: Rocket Lab data sheet and ICD",
    "BCT RWp050 / RWp100: satcatalog, satnow and satsearch listings (BCT's own data sheet was not reachable)",
]

ENVELOPES = [  # side mm, max height mm, compared with, trace width, strands
    dict(side=46, h_max=24.2, vs="CubeSpace CW0162", trace=0.6, strands=2),
    dict(side=50, h_max=40.0, vs="Rocket Lab RW-0.03", trace=0.6, strands=2),
    dict(side=58, h_max=25.0, vs="BCT RWp050", trace=0.8, strands=2),
    dict(side=70, h_max=25.0, vs="BCT RWp100", trace=1.0, strands=3),
    dict(side=90, h_max=25.0, vs="(1U width, no COTS match)", trace=1.2, strands=3),
]
STATORS = [("JLC 4L", "jlc4_2oz_out"), ("ACE 6L 3 oz", "ace6_3oz")]


def geometry(side, rim_w, h_mag, stackup, trace):
    Rr = side / 2 - 2.0                   # rotor radius inside 0.8 mm walls + clearance
    R = Rr - rim_w - (0.5 if rim_w else 0.0)
    st = STACKUPS[stackup]
    d = StatorDesign(f"cs_{side}_{stackup}", stackup=stackup, n_coils=6, r_in_mm=0.51 * R, r_out_mm=R,
                     trace_mm=trace, space_mm=max(0.15, st["min_space_mm"]), board_od_mm=side)
    r = RotorDesign(n_poles=8, magnet_shape="block", block_len_mm=0.465 * R, block_w_mm=0.31 * R,
                    magnet_h_mm=h_mag, magnet_r_mm=0.756 * R, airgap_mm=0.6, back_iron=True,
                    back_iron_t_mm=1.5, disc_od_mm=2 * Rr, disc_t_mm=0.0)
    return d, r, Rr, R


def rotor_mass_inertia(r: RotorDesign, Rr, extra_disc_mm, rim_w, h_rim):
    t = (r.back_iron_t_mm + extra_disc_mm) / 1e3
    rr = Rr / 1e3
    m_disc = STEEL * math.pi * rr ** 2 * t
    J = 0.5 * m_disc * rr ** 2
    m_mag = r.magnet_mass_kg()
    J += m_mag * (r.magnet_r_mm / 1e3) ** 2
    m_rim = 0.0
    if rim_w:
        ri = (Rr - rim_w) / 1e3
        m_rim = STEEL * math.pi * (rr ** 2 - ri ** 2) * h_rim / 1e3
        J += 0.5 * m_rim * (rr ** 2 + ri ** 2)
    return J, m_disc + m_mag + m_rim


def stator_em(d, r, strands):
    th, tq, c_eddy, _ = coil_curves(d, r, dual=False, n_theta=36)
    kt_s, _, _, _ = foc_torque(d, r, th, tq)
    R_s, L_s = phase_resistance(d), phase_inductance(d)
    w, g = d.trace_mm, d.stack["min_space_mm"]
    ws = (w - (strands - 1) * g) / strands
    R_s *= w / (strands * ws)
    c_eddy *= (ws / w) ** 2 * (strands * ws / w)
    return kt_s, R_s, L_s, c_eddy, ws


def best_wiring(kt_s, R_s, c_eddy, layers, J):
    """Choose parallel paths P (transposed) for the highest rated speed."""
    v_ph = V_BUS / math.sqrt(3)
    w_cap = RPM_CAP * 2 * math.pi / 60
    best = None
    for P in [p for p in range(1, layers + 1) if layers % p == 0]:
        kt, R = kt_s / P, R_s / P ** 2
        ke = kt * 2 / 3

        def spare(w):
            drag = c_eddy * w + BEARING_C + BEARING_V * w
            return kt * max(0.0, min(I_LIM, (v_ph - ke * w) / R)) - drag
        if spare(0) < MARGIN_Nm:
            continue
        lo, hi = 0.0, w_cap
        if spare(hi) >= MARGIN_Nm:
            lo = hi
        else:
            for _ in range(60):
                mid = 0.5 * (lo + hi)
                lo, hi = (mid, hi) if spare(mid) >= MARGIN_Nm else (lo, mid)
        cand = (lo, P, kt, R, ke, spare(lo))
        # highest rated speed; on a tie (speed cap), the most torque left at that speed
        if best is None or cand[0] > best[0] + 1e-6 or (abs(cand[0] - best[0]) < 1e-6 and cand[5] > best[5]):
            best = cand
    return best[:5] if best else None


def evaluate_env(env, stator_label, stackup):
    rows = []
    for rim_frac in (0.0, 0.1, 0.15):
        rim_w = rim_frac * (env["side"] / 2 - 2.0)
        for h_mag in (0.077, 0.155):  # x R, i.e. ~1/8 in and ~1/4 in magnets at the 90 mm size
            d, r, Rr, R = geometry(env["side"], rim_w, 0.0, stackup, env["trace"])
            hm = round(h_mag * R, 2)
            r = replace(r, magnet_h_mm=hm)
            kt_s, R_s, L_s, c_eddy, ws = stator_em(d, r, env["strands"])
            board_t = max(d.stack["z_mm"]) + d.stack["oz"][0] * OZ_TO_M * 1e3 + 0.2
            for extra, rim_add in [(e, a) for e in (0.0, 1.5, 3.0, 5.0) for a in ((0.0, 4.0, 8.0, 12.0, 16.0) if rim_w else (0.0,))]:
                # The rim is as tall as the magnets plus back iron plus extra disc, plus rim_add extending
                # behind the disc (away from the stator).
                rotor_h = hm + 1.5 + extra
                h_rim = rotor_h + rim_add if rim_w else 0.0
                J, m_rot = rotor_mass_inertia(r, Rr, extra, rim_w, h_rim)
                # cover, electronics, stator board, gap, rotor (with rim), clearance, cover
                height = 0.8 + 2.0 + board_t + 0.6 + max(rotor_h, h_rim) + 1.0 + 0.8
                if height > env["h_max"] + 1e-9:
                    continue
                sol = best_wiring(kt_s, R_s, c_eddy, d.layers, J)
                if sol is None:
                    continue
                w, P, kt, Rph, ke = sol
                km = kt / math.sqrt(1.5 * Rph)
                drag = c_eddy * w + BEARING_C + BEARING_V * w
                p_steady = c_eddy * w ** 2 + (BEARING_C + BEARING_V * w) * w + (drag / km) ** 2 + P_ELEC_W
                v_ph = V_BUS / math.sqrt(3)
                tau_rated = kt * max(0.0, min(I_LIM, (v_ph - ke * w) / Rph)) - drag
                tau_peak0 = kt * min(I_LIM, v_ph / Rph)
                side = env["side"] / 1e3
                m_board = side ** 2 * board_t / 1e3 * FR4 + copper_mass_g(d) / 1e3 * (env["strands"] * ws / d.trace_mm)
                m_hub = 0.010 if env["side"] <= 50 else (0.015 if env["side"] <= 70 else 0.020)
                m_house = AL * 0.8e-3 * (side ** 2 + 4 * side * height / 1e3)
                m_elec = 0.008
                m_tot = m_rot + m_board + m_hub + m_house + m_elec
                rows.append(dict(
                    envelope=f"{env['side']}×{env['side']}", vs=env["vs"], stator=stator_label, stackup=stackup,
                    rim=bool(rim_w), rim_w_mm=rim_w, rim_h_mm=h_rim, magnet_h_mm=hm, extra_disc_mm=extra,
                    height_mm=height,
                    parallel_paths=P, strands=env["strands"], Kt_mNm_per_A=kt * 1e3, R_phase_ohm=Rph,
                    Km_mNm_per_sqrtW=km * 1e3, rated_rpm=w * 60 / (2 * math.pi), H_mNms=J * w * 1e3,
                    tau_at_rated_mNm=tau_rated * 1e3, tau_stall_mNm=tau_peak0 * 1e3,
                    P_cu_for_2mNm_W=(2e-3 / km) ** 2, P_cu_for_7mNm_W=(7e-3 / km) ** 2,
                    eddy_W_at_rated=c_eddy * w ** 2, P_steady_W=p_steady,
                    rotor_g=m_rot * 1e3, mass_g=m_tot * 1e3, J_kgm2=J,
                    H_per_kg=J * w * 1e3 / m_tot))
    return rows


def best(rows):
    """Most momentum that fits the COTS wheel's box (ties broken by momentum per mass)."""
    return max(rows, key=lambda r: (round(r["H_mNms"], 1), r["H_per_kg"])) if rows else None


def main():
    allrows, picks = [], []
    for env in ENVELOPES:
        for label, st in STATORS:
            rows = evaluate_env(env, label, st)
            allrows += rows
            b = best(rows)
            if b:
                picks.append(b)
                print(f"{b['envelope']:7s} {label:12s} H={b['H_mNms']:6.1f} mass={b['mass_g']:5.0f} g "
                      f"h={b['height_mm']:4.1f} rpm={b['rated_rpm']:5.0f} tau@rated={b['tau_at_rated_mNm']:5.1f} "
                      f"Psteady={b['P_steady_W']:4.2f} eddy={b['eddy_W_at_rated']:4.2f} Km={b['Km_mNm_per_sqrtW']:5.1f}",
                      flush=True)
    (OUT / "pcbmotor_cubesat_wheels.json").write_text(json.dumps(dict(cots=COTS, picks=picks, all=allrows), indent=1))
    md = ["# PCB-motor CubeSat wheels against COTS\n",
          "Generated by `pcbmotor/cubesat_wheels.py`. PCB wheel masses and heights are estimates of a complete "
          "wheel (rotor, stator board with driver, hub and bearings, aluminium cover); COTS values are datasheet "
          f"or vendor-listing values. PCB wheels: {V_BUS:g} V, {I_LIM:g} A driver, {RPM_CAP:.0f} rpm, rated where "
          f"{MARGIN_Nm*1e3:g} mN·m is left after drag; stranded, transposed windings.\n",
          "| Wheel | Footprint mm | Height mm | Mass g | Momentum mN·m·s | mN·m·s per kg | Torque mN·m | Steady W at rated momentum |",
          "|---|---|---|---|---|---|---|---|"]
    for c in COTS:
        md.append(f"| {c['name']} | {c['size'] or '–'} | – | {c['mass_g']} | {c['H']:g} | {c['H'] / c['mass_g'] * 1e3:.0f} | "
                  f"{c['tau']:g} | {c['P_W'] if c['P_W'] is not None else '–'} |")
    for p in picks:
        md.append(f"| PCB, {p['stator']} (vs {p['vs']}) | {p['envelope']} | {p['height_mm']:.1f} | {p['mass_g']:.0f} | "
                  f"{p['H_mNms']:.0f} | {p['H_per_kg']:.0f} | {p['tau_at_rated_mNm']:.1f} at rated, "
                  f"{p['tau_stall_mNm']:.0f} at 0 rpm | {p['P_steady_W']:.2f} |")
    md.append("\nDesign details of the picks:\n")
    md.append("| Footprint | Stator | Rim w × h mm | Magnet mm | Extra disc mm | Paths | Km mN·m/√W | Rated rpm | Rotor g | "
              "Eddy W at rated | Cu W for 7 mN·m |")
    md.append("|---|---|---|---|---|---|---|---|---|---|---|")
    for p in picks:
        md.append(f"| {p['envelope']} | {p['stator']} | {(f"{p['rim_w_mm']:.1f} × {p['rim_h_mm']:.1f}") if p['rim'] else 'none'} | {p['magnet_h_mm']:.1f} | "
                  f"{p['extra_disc_mm']:.1f} | {p['parallel_paths']} | {p['Km_mNm_per_sqrtW']:.1f} | {p['rated_rpm']:.0f} | "
                  f"{p['rotor_g']:.0f} | {p['eddy_W_at_rated']:.2f} | {p['P_cu_for_7mNm_W']:.2f} |")
    md.append("\nCOTS sources: " + "; ".join(COTS_SOURCES) + ".")
    (OUT / "pcbmotor_cubesat_wheels.md").write_text("\n".join(md) + "\n")
    print("\n".join(md))


if __name__ == "__main__":
    main()
