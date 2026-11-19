"""Mechanical checks for the PCB motor designs.

Run from sim/:  .venv/bin/python -m pcbmotor.mechanical
Writes results/pcbmotor_mechanical.json and results/pcbmotor_mechanical.md.

Checks, for the test stator and the payload designs in results/pcbmotor_designs.json:
- Air gap: Kt and the eddy-loss coefficient against magnet-to-copper gap (same EM model).
- Board flatness: IPC-6012 bow and twist allowance (0.75% of the diagonal for SMT boards) against the
  design gap. A board that bows more than the gap rubs, so the stator must be clamped or bonded flat.
- Back iron: flux per pole from the mean axial field on the magnet face (magpylib, with the back-iron
  image) and the steel thickness needed to carry half of it at 1.6 T. The EM model treats the steel as
  ideal; if the rotor has less steel than this, the model overstates torque.
- Magnet retention: centrifugal load on each magnet at the speed cap and the bond shear stress it puts
  on the magnet footprint. ASSUMED structural epoxy shear strength 10 MPa at room temperature (less
  when cold), so the report gives the margin against that.
- Balance: the eccentricity from a 1% mass error on one magnet, its ISO 1940 balance grade at the top
  speed, and the shaking force at several speeds and grades.
- Temperature: Km against temperature from copper resistivity (+0.393 %/K) and NdFeB remanence
  (-0.12 %/K, typical N grade), for -50, 20 and 80 °C.
- Bearing loads: axial load from a 20 g shock (parachute opening, landing) and the gyroscopic moment
  H × Ω for a 1 rad/s body rate, as a radial load across an ASSUMED 10 mm bearing span.
Rotor bending modes (thin wide rotors such as a ring wheel) need FEA and are not computed here.
"""
from __future__ import annotations

import json
import math
import pathlib
from dataclasses import replace

import numpy as np

from .geometry import StatorDesign, RotorDesign
from .em import foc_torque, phase_resistance, rotor_magnets, ALPHA_CU
from .stacking import coil_curves
from .designs import RPM_CAP

OUT = pathlib.Path(__file__).resolve().parents[1] / "results"
DESIGNS = ("test", "payload", "payload_high_momentum")
GAPS_MM = (0.3, 0.5, 0.6, 0.8, 1.0, 1.3)
IPC_BOW = 0.0075
B_STEEL_T = 1.6
EPOXY_SHEAR_MPA = 10.0
BR_TEMPCO = -0.0012
SHOCK_G = 20.0
BEARING_SPAN_M = 0.010
BODY_RATE = 1.0


def gap_sweep(d, r):
    out = []
    for g in GAPS_MM:
        rr = replace(r, airgap_mm=g)
        th, tq, ce, bmax = coil_curves(d, rr, dual=False, n_theta=36)
        out.append(dict(gap_mm=g, Kt_mNm_per_A=foc_torque(d, rr, th, tq)[0] * 1e3, eddy_coeff=ce, Bz_peak_T=bmax))
    return out


def flux_per_pole(r: RotorDesign):
    m = rotor_magnets(r, 0.0)
    mag = m.children[0]
    x0, y0, _ = mag.position
    if r.magnet_shape == "block":
        L, W = r.block_len_mm / 1e3, r.block_w_mm / 1e3
        us, vs = np.linspace(-L / 2, L / 2, 17)[1:-1], np.linspace(-W / 2, W / 2, 17)[1:-1]
        pts = [(x0 + u, y0 + v, r.airgap_mm / 1e3 + 1e-5) for u in us for v in vs]
        area, width = L * W, L
    else:
        rad = r.magnet_d_mm / 2e3
        pts = [(x0 + rr * math.cos(a), y0 + rr * math.sin(a), r.airgap_mm / 1e3 + 1e-5)
               for rr in np.linspace(0.05, 0.95, 8) * rad for a in np.linspace(0, 2 * math.pi, 16, endpoint=False)]
        area, width = math.pi * rad ** 2, 2 * rad
    bz = float(np.abs(m.getB(np.array(pts))[:, 2]).mean())
    phi = bz * area
    return bz, phi, (phi / 2) / (B_STEEL_T * width)


def check(key, d: StatorDesign, r: RotorDesign):
    J, m_rot = r.inertia_kgm2()
    if r.magnet_shape == "block":
        m_mag = r.block_len_mm * r.block_w_mm * r.magnet_h_mm * 1e-9 * 7500
        foot = r.block_len_mm * r.block_w_mm * 1e-6
    else:
        m_mag = math.pi * (r.magnet_d_mm / 2e3) ** 2 * r.magnet_h_mm / 1e3 * 7500
        foot = math.pi * (r.magnet_d_mm / 2e3) ** 2
    th, tq, _, _ = coil_curves(d, r, dual=False, n_theta=36)
    kt = foc_torque(d, r, th, tq)[0]
    R = phase_resistance(d)
    w_nl = (11.1 / math.sqrt(3)) / (kt * 2 / 3)
    w_top = min(w_nl, RPM_CAP * 2 * math.pi / 60)
    bz, phi, t_need = flux_per_pole(r)
    steel = r.back_iron_t_mm + (r.disc_t_mm if r.disc_density > 7000 else 0.0)
    f_cent = m_mag * w_top ** 2 * r.magnet_r_mm / 1e3
    e1 = 0.01 * m_mag * (r.magnet_r_mm / 1e3) / m_rot
    temps = {}
    for t in (-50.0, 20.0, 80.0):
        kt_t = kt * (1 + BR_TEMPCO * (t - 20))
        R_t = R * (1 + ALPHA_CU * (t - 20))
        temps[f"{t:g}C"] = kt_t / math.sqrt(1.5 * R_t) * 1e3
    diag = d.board_od_mm * (math.sqrt(2) if d.board_od_mm >= 80 else 1.0)  # payload boards are square
    out = dict(
        design=key, gap_mm=r.airgap_mm, gap_sweep=gap_sweep(d, r),
        ipc_bow_mm=IPC_BOW * diag, bow_exceeds_gap=IPC_BOW * diag > r.airgap_mm,
        magnet_face_Bz_T=bz, flux_per_pole_Wb=phi, steel_needed_mm=t_need * 1e3, steel_present_mm=steel,
        back_iron_ok=steel >= t_need * 1e3,
        top_rpm=w_top * 60 / (2 * math.pi), magnet_mass_g=m_mag * 1e3, centrifugal_N=f_cent,
        bond_shear_MPa=f_cent / foot / 1e6, bond_margin=EPOXY_SHEAR_MPA / (f_cent / foot / 1e6),
        rotor_mass_g=m_rot * 1e3, J_kgm2=J,
        e_um_1pct_magnet=e1 * 1e6, G_grade_1pct_magnet=e1 * w_top * 1e3,
        shaking_N={f"G{G}@{rpm}": m_rot * (G / 1e3 / (rpm * 2 * math.pi / 60)) * (rpm * 2 * math.pi / 60) ** 2
                   for G in (2.5, 6.3, 16) for rpm in (1000, int(round(w_top * 60 / (2 * math.pi))))},
        Km_vs_temp_mNm_per_sqrtW=temps,
        shock_axial_N=m_rot * 9.81 * SHOCK_G,
        gyro_radial_N=J * w_top * BODY_RATE / BEARING_SPAN_M,
    )
    print(f"{key:22s} bow {out['ipc_bow_mm']:.2f} mm vs gap {r.airgap_mm} | steel {steel:.1f}/{t_need*1e3:.2f} mm | "
          f"magnet {f_cent:.0f} N, {out['bond_shear_MPa']:.2f} MPa | e {e1*1e6:.0f} um (G{out['G_grade_1pct_magnet']:.1f}) | "
          f"Km -50/20/80C {temps['-50C']:.1f}/{temps['20C']:.1f}/{temps['80C']:.1f}", flush=True)
    return out


def main():
    j = json.loads((OUT / "pcbmotor_designs.json").read_text())
    rows = [check(k, StatorDesign(**j[k]["stator"]), RotorDesign(**j[k]["rotor"])) for k in DESIGNS]
    (OUT / "pcbmotor_mechanical.json").write_text(json.dumps(rows, indent=2))
    md = ["# Mechanical checks\n", "Generated by `pcbmotor/mechanical.py`; assumptions in the module docstring.\n",
          "| Check | " + " | ".join(r["design"] for r in rows) + " |", "|---|" + "---|" * len(rows)]

    def line(label, fn):
        md.append(f"| {label} | " + " | ".join(fn(r) for r in rows) + " |")
    line("Design gap mm", lambda r: f"{r['gap_mm']}")
    line("Kt at gap 0.3 / design / 1.0 mm (mN·m/A)", lambda r: " / ".join(
        f"{s['Kt_mNm_per_A']:.1f}" for s in r["gap_sweep"] if s["gap_mm"] in (0.3, r["gap_mm"], 1.0)))
    line("IPC bow and twist allowance mm (exceeds gap?)", lambda r: f"{r['ipc_bow_mm']:.2f} ({'yes' if r['bow_exceeds_gap'] else 'no'})")
    line("Steel needed / present behind magnets mm", lambda r: f"{r['steel_needed_mm']:.2f} / {r['steel_present_mm']:.1f} ({'ok' if r['back_iron_ok'] else 'SATURATES'})")
    line("Top speed rpm", lambda r: f"{r['top_rpm']:.0f}")
    line("Magnet load at top speed N (bond shear MPa, margin vs 10 MPa)", lambda r: f"{r['centrifugal_N']:.0f} ({r['bond_shear_MPa']:.2f}, {r['bond_margin']:.0f}×)")
    line("1% mass error on one magnet: eccentricity µm (balance grade)", lambda r: f"{r['e_um_1pct_magnet']:.0f} (G{r['G_grade_1pct_magnet']:.1f})")
    line("Shaking force at top speed, G2.5 / G16 (N)", lambda r: " / ".join(
        f"{v:.2f}" for k, v in r["shaking_N"].items() if not k.endswith("@1000") and (k.startswith("G2.5") or k.startswith("G16"))))
    line("Km at -50 / 20 / 80 °C (mN·m/√W)", lambda r: " / ".join(f"{v:.1f}" for v in r["Km_vs_temp_mNm_per_sqrtW"].values()))
    line("Axial bearing load at 20 g shock N", lambda r: f"{r['shock_axial_N']:.0f}")
    line("Gyroscopic radial load at top speed, 1 rad/s body rate N", lambda r: f"{r['gyro_radial_N']:.1f}")
    md.append("\nRotor bending modes are not computed (they need FEA); check them for any thin, wide rotor.")
    (OUT / "pcbmotor_mechanical.md").write_text("\n".join(md) + "\n")
    print("\n".join(md))


if __name__ == "__main__":
    main()
