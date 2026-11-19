"""Custom (ACE-style) stator stackups against the JLCPCB payload board.

Run from sim/:  .venv/bin/python -m pcbmotor.ace_stackups
Writes results/pcbmotor_ace_stackups.json and results/pcbmotor_ace_stackups.md.

Question: if a fab such as Accurate Circuit Engineering (ACE) builds the stator with many thin
layers of heavy copper, and splits the wide traces into narrow transposed strands ("PCB Litz"),
how much does that buy in efficiency, eddy loss and usable stored momentum?

Everything uses the recommended high-momentum payload rotor (6 coils / 8 poles,
19.05 x 12.7 x 6.35 mm N42 blocks, 84 mm x 1.5 mm steel disc, 0.6 mm gap) unchanged, so only the
stator copper differs. The torque and eddy models are the ones in pcbmotor/em.py and
pcbmotor/stacking.py.

Model notes and assumptions:
- ACE stackups are ASSUMED builds. ACE publishes inner foil up to 4.2 mil (~3 oz) standard,
  external copper up to 8.4 mil and 2 mil minimum dielectric, but not the dielectric it can lay
  over heavy copper or the spacing it can hold at 2-3 oz. Here the dielectric between layers is
  thick enough to fill heavy copper (about 1.3x the copper), and the minimum space is 0.15 mm at
  2 oz (JLC's 2 oz rule) and 0.2 mm at 3 oz (typical heavy-copper rule). Ask ACE for real values.
- Parallel layer groups: the layers are wired as P parallel paths of layers/P layers each. Every
  path is assumed fully transposed (it visits every layer equally), so every path has the same
  back-EMF and no current circulates between them: Kt/P, R/P^2, L/P^2. P is chosen to maximise
  usable momentum. Without transposition the paths nearer the magnets drive a circulating current
  (see pcbmotor/stacking.py), which this study does not include.
- Strands: each 1.2 mm trace is split into n strands with the fab's minimum space between
  them (JLC 1 oz 0.09 mm, JLC 2 oz 0.15 mm, ACE as above), fully transposed. Copper width falls to n*w_s (R rises by w / (n*w_s)); the eddy loss per
  unit volume scales with the conductor width squared, so the eddy coefficient scales by
  (w_s/w)^2 * (n*w_s/w). Kt is taken as unchanged (the current centroid barely moves).
- Driver limit 1 A and 11.1 V bus as in the rest of the study; speed capped at 6,000 rpm for a
  hand-balanced rotor (pcbmotor/designs.py RPM_CAP).
- Copper watts are for RMS torques the three-axis balloon payload needs to cancel line swing
  (pitch/roll wheels, attachment 5 mm above the centre of mass, 1.5 kg payload, 7 s swing):
  9 mN·m RMS for a 10 degree swing, 23 mN·m RMS for 26 degrees. Mean copper loss = (tau_rms/Km)^2.
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
from .designs import drive_metrics, V_BUS, RPM_CAP

OUT = pathlib.Path(__file__).resolve().parents[1] / "results"
I_LIM = 1.0
W_CAP_RPM = RPM_CAP
MARGIN_Nm = 6e-3
TAU_RMS_mNm = {"10deg": 9.0, "26deg": 23.0}
BOARD_AREA_M2 = 0.090 * 0.090


def _stack(n, oz, diel_mm, fab, min_space):
    t = oz * OZ_TO_M * 1e3
    pitch = t + diel_mm
    return dict(fab=fab, z_mm=[round(k * pitch, 4) for k in range(n)], oz=[oz] * n,
                min_trace_mm=0.1, min_space_mm=min_space, diel_mm=diel_mm)


# ASSUMED ACE-style builds (see module docstring).
STACKUPS["ace6_3oz"] = _stack(6, 3.0, 0.13, "ACE (assumed)", 0.20)
STACKUPS["ace10_2oz"] = _stack(10, 2.0, 0.09, "ACE (assumed)", 0.15)
STACKUPS["ace12_2oz"] = _stack(12, 2.0, 0.065, "ACE (assumed, aggressive)", 0.15)

CASES = [
    # label, stackup, strands
    ("JLC 4L, 2 oz out / 1 oz in (current design)", "jlc4_2oz_out", 1),
    ("JLC 4L, 2 oz out / 1 oz in, 3 strands", "jlc4_2oz_out", 3),
    ("JLC 6L, 1 oz", "jlc6_1oz", 1),
    ("JLC 6L, 1 oz, 3 strands", "jlc6_1oz", 3),
    ("ACE 6L, 3 oz", "ace6_3oz", 1),
    ("ACE 6L, 3 oz, 3 strands", "ace6_3oz", 3),
    ("ACE 10L, 2 oz", "ace10_2oz", 1),
    ("ACE 10L, 2 oz, 3 strands", "ace10_2oz", 3),
    ("ACE 12L, 2 oz, thin dielectric", "ace12_2oz", 1),
    ("ACE 12L, 2 oz, thin dielectric, 3 strands", "ace12_2oz", 3),
]


def usable_speed(kt, ke, R, c_drag, margin_Nm, w_cap):
    """Highest speed (rad/s, <= w_cap) at which the motor still has margin_Nm of torque left
    after eddy drag, with the voltage and current limits."""
    v_ph = V_BUS / math.sqrt(3)

    def spare(w):
        return kt * max(0.0, min(I_LIM, (v_ph - ke * w) / R)) - c_drag * w
    if spare(w_cap) >= margin_Nm:
        return w_cap
    if spare(0.0) < margin_Nm:
        return 0.0
    lo, hi = 0.0, w_cap
    for _ in range(60):
        mid = 0.5 * (lo + hi)
        lo, hi = (mid, hi) if spare(mid) >= margin_Nm else (lo, mid)
    return lo


def evaluate(label, d: StatorDesign, r: RotorDesign, strands: int):
    th, tq, c_eddy_solid, bmax = coil_curves(d, r, dual=False, n_theta=48)
    kt_series, ripple, _, _ = foc_torque(d, r, th, tq)
    R_series, L_series = phase_resistance(d), phase_inductance(d)
    w = d.trace_mm
    if strands > 1:
        g = d.stack["min_space_mm"]          # strands of one conductor at the fab's minimum space
        ws = (w - (strands - 1) * g) / strands
        r_fac = w / (strands * ws)
        e_fac = (ws / w) ** 2 * (strands * ws / w)
    else:
        ws, r_fac, e_fac = w, 1.0, 1.0
    R_series *= r_fac
    c_eddy = c_eddy_solid * e_fac
    J, m_rot = r.inertia_kgm2()
    w_cap = W_CAP_RPM * 2 * math.pi / 60
    best = None
    for P in [p for p in range(1, d.layers + 1) if d.layers % p == 0]:
        kt, R, L = kt_series / P, R_series / P ** 2, L_series / P ** 2
        ke = kt * 2 / 3
        w_m = usable_speed(kt, ke, R, c_eddy, MARGIN_Nm, w_cap)
        if best is None or w_m > best[1] + 1e-6:
            best = (P, w_m, kt, R, L)
    P, w_m, kt, R, L = best
    ke = kt * 2 / 3
    m = drive_metrics(kt, R, L, ke, J, BOARD_AREA_M2, I_LIM)
    km = kt / math.sqrt(1.5 * R)
    w_nl = (V_BUS / math.sqrt(3)) / ke
    cu_g = copper_mass_g(d) / r_fac
    out = dict(
        config=label, stackup=d.stackup, fab=d.fab, layers=d.layers, strands=strands,
        strand_width_mm=ws, parallel_paths=P, layers_per_path=d.layers // P,
        Kt_mNm_per_A=kt * 1e3, R_phase_ohm=R, L_phase_uH=L * 1e6, Km_mNm_per_sqrtW=km * 1e3,
        ripple_pct=ripple * 100, noload_rpm=w_nl * 60 / (2 * math.pi),
        tau_at_1A_mNm=kt * 1e3 * min(1.0, m["i_stall_A"]),
        H_no_drag_mNms=J * min(w_nl, w_cap) * 1e3,
        rpm_usable=w_m * 60 / (2 * math.pi), H_usable_mNms=J * w_m * 1e3,
        speed_capped=w_m >= w_cap - 1e-6,
        eddy_W_at_2000rpm=c_eddy * (2000 * 2 * math.pi / 60) ** 2,
        eddy_W_at_usable=c_eddy * w_m ** 2,
        **{f"Pcu_W_swing_{k}": (v / 1e3 / km) ** 2 for k, v in TAU_RMS_mNm.items()},
        stator_thickness_mm=max(d.stack["z_mm"]) + d.stack["oz"][-1] * OZ_TO_M * 1e3,
        copper_mass_g=cu_g, Bz_peak_T=bmax, rotor_J_kgm2=J, rotor_mass_g=m_rot * 1e3,
    )
    print(f"{label:46s} P={P:2d} Kt={out['Kt_mNm_per_A']:6.2f} R={R:6.3f} Km={out['Km_mNm_per_sqrtW']:5.2f} "
          f"rpm={out['rpm_usable']:5.0f} H={out['H_usable_mNms']:6.1f} eddy2k={out['eddy_W_at_2000rpm']:5.2f} "
          f"Pcu26={out['Pcu_W_swing_26deg']:5.2f}", flush=True)
    return out


def table(rows):
    cols = [("config", "Stator", "{}"), ("parallel_paths", "Parallel paths", "{}"),
            ("Km_mNm_per_sqrtW", "Km mN·m/√W", "{:.1f}"), ("Kt_mNm_per_A", "Kt mN·m/A", "{:.1f}"),
            ("R_phase_ohm", "R Ω", "{:.3f}"),
            ("Pcu_W_swing_10deg", "Cu W, 10° swing", "{:.2f}"), ("Pcu_W_swing_26deg", "Cu W, 26° swing", "{:.2f}"),
            ("eddy_W_at_2000rpm", "Eddy W at 2,000 rpm", "{:.2f}"),
            ("rpm_usable", "Usable rpm", "{:.0f}"), ("H_usable_mNms", "Usable momentum mN·m·s", "{:.0f}"),
            ("eddy_W_at_usable", "Eddy W at usable rpm", "{:.2f}"),
            ("stator_thickness_mm", "Copper stack mm", "{:.2f}"), ("copper_mass_g", "Copper g", "{:.1f}")]
    head = "| " + " | ".join(c[1] for c in cols) + " |\n|" + "---|" * len(cols) + "\n"
    body = ""
    for r in rows:
        cells = []
        for k, _, f in cols:
            v = f.format(r[k])
            if k == "rpm_usable" and r["speed_capped"]:
                v += " (cap)"
            cells.append(v)
        body += "| " + " | ".join(cells) + " |\n"
    return head + body


def sensitivity():
    """Usable momentum and eddy loss at the usable speed with a 3 A driver and/or no speed cap."""
    global I_LIM, W_CAP_RPM
    saved = (I_LIM, W_CAP_RPM)
    cases = {"1 A, no cap": (1.0, 30000.0), "3 A, 6,000 rpm cap": (3.0, RPM_CAP), "3 A, no cap": (3.0, 30000.0)}
    res = {}
    for k, (I_LIM, W_CAP_RPM) in cases.items():
        res[k] = main(write=False)
    I_LIM, W_CAP_RPM = saved
    (OUT / "pcbmotor_ace_stackups_sensitivity.json").write_text(json.dumps(res, indent=1))
    names = [r["config"] for r in next(iter(res.values()))]
    md = ("\n## Sensitivity to the driver current limit and the speed cap\n\n"
          "Usable momentum mN·m·s / eddy W at that speed. Without the cap the motor reaches speeds a "
          "hand-balanced rotor can't run at; those columns only show where the motor stops being the limit.\n\n"
          "| Stator | " + " | ".join(cases) + " |\n|" + "---|" * (len(cases) + 1) + "\n")
    for i, n in enumerate(names):
        md += f"| {n} | " + " | ".join(
            f"{res[k][i]['H_usable_mNms']:.0f} / {res[k][i]['eddy_W_at_usable']:.2f}" for k in cases) + " |\n"
    return md


def main(write=True):
    j = json.loads((OUT / "pcbmotor_designs.json").read_text())["payload_high_momentum"]
    d0, r = StatorDesign(**j["stator"]), RotorDesign(**j["rotor"])
    rows = []
    for label, st, strands in CASES:
        space = STACKUPS[st]["min_space_mm"] if st.startswith("ace") else d0.space_mm
        d = replace(d0, name=f"{st}_s{strands}", stackup=st, space_mm=max(space, d0.space_mm))
        rows.append(evaluate(label, d, r, strands))
    if not write:
        return rows
    sens = sensitivity()
    (OUT / "pcbmotor_ace_stackups.json").write_text(json.dumps(rows, indent=2))
    md = ("# Custom stackup study (ACE-style)\n\nGenerated by `pcbmotor/ace_stackups.py`. High-momentum payload "
          "rotor unchanged; 1.2 mm traces; 11.1 V, 1 A driver limit; usable speed keeps 6 mN·m for control after "
          f"eddy drag and is capped at {RPM_CAP:.0f} rpm. ACE stackups and strand rules are assumptions; see the "
          "module docstring.\n\n" + table(rows) + sens)
    (OUT / "pcbmotor_ace_stackups.md").write_text(md)
    print(md)
    return rows


if __name__ == "__main__":
    main()
