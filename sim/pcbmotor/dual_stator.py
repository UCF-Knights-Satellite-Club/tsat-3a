"""Dual stator: one magnet rotor between two stator PCBs (coreless, no iron anywhere).

Question: does a second stator board on the other face of the rotor beat the single board with a
steel back iron? This is the reverse of stacking.py's dual rotor (one stator between two rotors).

Model (same magpylib Lorentz-force method as the rest of the sim):
- **Rotor.** The steel back iron and steel flywheel disc are replaced by a non-magnetic carrier of the
  same mass (austenitic stainless or brass, assumed) so the rotor inertia is unchanged. The magnets
  sit through the carrier with both faces exposed. Two magnet heights:
  - 1x: the current 19.05 x 12.7 x 6.35 mm N42 blocks;
  - 2x: two blocks stacked per pole (12.7 mm), which adds about 92 g of magnet.
- **Field.** With no iron, each board sees only the bare magnets (em.rotor_magnets with
  back_iron=False). A 2x magnet with no iron gives exactly the field of a 1x magnet on ideal back iron
  (the image model), so each board of the 2x dual stator sees what the current single board sees.
- **Second board.** The same PCB, flipped, at the same gap on the other face. The axial field of an
  axially magnetised block is symmetric about its mid-plane (checked in run()), and torque on radial
  copper and the eddy loss both come from that axial field, so the second board's Kt, eddy and strand
  circulating current equal the first's. Flipping the board reverses its phase sequence, so two of its
  phase leads are swapped, and it is clocked so its conductors line up with the first board's.
- **Connections.** Series: Kt and R double. Parallel: Kt unchanged, R halves. Both give the same Km;
  they bracket a rewind (bus_voltage.py: turns don't change Km), so the speed at 11.1 V can be set
  between them by the turns count.
- **Gap mismatch.** If the rotor sits off-centre by dz, one gap is 0.6 + dz and the other 0.6 - dz.
  In parallel the boards' back-EMFs differ and a current circulates between them; its loss is reported
  for dz = 0.1 mm. In series the sum of the two Kt is what matters, and it barely changes.
- **Axial force.** There is no iron, so the magnets pull on nothing outside the rotor: no bearing
  preload from the magnets and no negative axial stiffness. (The current single board has none either,
  because its only iron is the rotor's own back iron; stacking.py's dual rotor and stator_yoke.py's yoke
  carry 370-530 N.)

Not modelled: the thermal gain from a second board's faces (continuous torque would rise, but the
payload is not thermally limited), PWM ripple (bus_voltage.py; halves with series boards' doubled L).

Run from sim/:  .venv/bin/python -m pcbmotor.dual_stator
Writes results/pcbmotor_dual_stator.json and results/pcbmotor_dual_stator.md.
"""
from __future__ import annotations

from contextlib import contextmanager
from dataclasses import replace
import json
import math
import pathlib

import numpy as np

from . import em, stacking, wave, wave_asbuilt, mechanical
from .designs import V_BUS

OUT = pathlib.Path(__file__).resolve().parents[1] / "results"
RPM_CAP = 6000.0              # hand-balance cap used elsewhere
GAP_MISMATCH_MM = 0.1
FR4_G_PER_MM2 = 1.6 * 1850.0 / 1e6   # g per mm^2 of 1.6 mm FR4 (1850 kg/m^3 = 1.85e-3 g/mm^3)
RPM_COMPARE = 4000.0          # equal-speed momentum (turns can put any config at this speed)
DESIGNS = [
    # label, variant, stackup, board side mm (as laid out)
    ("spiral (current payload)", "spiral", "jlc4_2oz_out", 90.0),
    ("wave, blind vias, 2 strands swapped", "bv_swap", "jlc4_2oz", 117.0),
]
_ORIG = em.rotor_magnets


@contextmanager
def rotor_field(back_iron=True, h_mult=1.0, dgap_mm=0.0):
    """Run the existing models with the rotor's field changed: back iron on/off, magnet height scaled,
    gap shifted. The rotor's mass and inertia are handled separately (see rotor_options)."""
    def fn(r, theta=0.0):
        return _ORIG(replace(r, back_iron=back_iron, magnet_h_mm=r.magnet_h_mm * h_mult,
                             airgap_mm=r.airgap_mm + dgap_mm), theta)
    mods = (em, stacking, wave, wave_asbuilt, mechanical)
    saved = [m.rotor_magnets for m in mods]
    try:
        for m in mods:
            m.rotor_magnets = fn
        yield
    finally:
        for m, f in zip(mods, saved):
            m.rotor_magnets = f


def per_board(variant, stackup):
    """Kt, R, eddy and strand-circulating coefficients of one board in the current rotor field."""
    if variant == "spiral":
        b = wave.spiral_baseline()
        res = dict(Kt_mNm_per_A=b["Kt_mNm_per_A"], R_phase_ohm=b["R_phase_ohm"],
                   eddy_W_at_2000rpm=b["eddy_W_at_2000rpm"], circulating_W_at_2000rpm=0.0)
    else:
        res = wave_asbuilt.analyse(variant, stackup=stackup)
    return {k: float(res[k]) for k in ("Kt_mNm_per_A", "R_phase_ohm", "eddy_W_at_2000rpm",
                                       "circulating_W_at_2000rpm")}


def rotor_options():
    """Inertia and mass of the rotors compared. The dual-stator rotor swaps the 3 mm of steel (back
    iron + disc) for the same mass of non-magnetic carrier, so its inertia is unchanged."""
    rot = wave.hm_rotor()
    steel_t = rot.back_iron_t_mm + rot.disc_t_mm
    nonmag = replace(rot, back_iron=False, disc_t_mm=steel_t)
    out = {}
    for key, r in (("1x", rot), ("2x", replace(rot, magnet_h_mm=2 * rot.magnet_h_mm)),
                   ("1x_nonmag", nonmag), ("2x_nonmag", replace(nonmag, magnet_h_mm=2 * rot.magnet_h_mm))):
        J, m = r.inertia_kgm2()
        out[key] = dict(J_kgm2=J, mass_g=m * 1e3, magnet_g=r.magnet_mass_kg() * 1e3)
    return rot, out


def combine(label, design, b, n_boards, series, rotor, board_side_mm, gap_circ_W=0.0, gap_kts=None):
    """One configuration's figures from one board's figures b."""
    kt = b["Kt_mNm_per_A"] * (2 if (n_boards == 2 and series) else 1)
    R = b["R_phase_ohm"] * (2 if series else 0.5) if n_boards == 2 else b["R_phase_ohm"]
    km = kt / math.sqrt(1.5 * R)
    ke = kt / 1e3 * 2 / 3
    noload = V_BUS / math.sqrt(3) / ke * 60 / (2 * math.pi)
    w_top = min(noload, RPM_CAP) * 2 * math.pi / 60
    eddy = b["eddy_W_at_2000rpm"] * n_boards
    strand = b["circulating_W_at_2000rpm"] * n_boards
    cu = (15 / km) ** 2
    return dict(config=label, design=design, n_boards=n_boards,
                connection=("series" if series else "parallel") if n_boards == 2 else "-",
                Kt_mNm_per_A=kt, R_phase_ohm=R, Km_mNm_per_sqrtW=km, noload_rpm=noload,
                copper_W_15mNm=cu, eddy_W_2000=eddy, strand_circ_W_2000=strand,
                gap_circ_W_2000=gap_circ_W, loss_15mNm_2000_W=cu + eddy + strand,
                drag_mNm_2000=(eddy + strand) / (2000 * 2 * math.pi / 60) * 1e3,
                H_top_mNms=rotor["J_kgm2"] * w_top * 1e3, J_kgm2=rotor["J_kgm2"],
                H_at_compare_rpm_mNms=rotor["J_kgm2"] * RPM_COMPARE * 2 * math.pi / 60 * 1e3,
                rotor_g=rotor["mass_g"],
                boards_g=n_boards * board_side_mm ** 2 * FR4_G_PER_MM2,
                kt_per_board_at_gap_mismatch=gap_kts)


def check_symmetry():
    """The axial field of a bare (no back iron) magnet is the same on both faces."""
    rot = wave.hm_rotor()
    r = replace(rot, back_iron=False)
    m = _ORIG(r, 0.1)
    zc = (r.airgap_mm + r.magnet_h_mm / 2) / 1e3
    xy = np.c_[np.linspace(0.02, 0.042, 9), np.linspace(-0.005, 0.01, 9)]
    for d in (0.0008, 0.0015):          # copper depths below the near face
        top = m.getB(np.c_[xy, np.full(9, -d)])
        bot = m.getB(np.c_[xy, np.full(9, 2 * zc + d)])
        assert np.allclose(top[:, 2], bot[:, 2], rtol=1e-6, atol=1e-9)
        assert np.allclose(top[:, :2], -bot[:, :2], rtol=1e-6, atol=1e-9)


def run():
    check_symmetry()
    rot, ropt = rotor_options()
    rows = []
    for label, variant, stackup, side in DESIGNS:
        with rotor_field():
            ref = per_board(variant, stackup)
        with rotor_field(h_mult=2.0):
            ref2 = per_board(variant, stackup)
        boards = {}
        for hm in (1.0, 2.0):
            with rotor_field(back_iron=False, h_mult=hm):
                boards[hm] = per_board(variant, stackup)
            kts = []
            for dz in (+GAP_MISMATCH_MM, -GAP_MISMATCH_MM):
                with rotor_field(back_iron=False, h_mult=hm, dgap_mm=dz):
                    kts.append(per_board(variant, stackup)["Kt_mNm_per_A"])
            boards[hm]["gap_kts"] = kts
        # 2x magnets with no iron is the image of 1x on back iron: same per-board figures
        assert abs(boards[2.0]["Kt_mNm_per_A"] / ref["Kt_mNm_per_A"] - 1) < 1e-6
        rows.append(combine("single board, back iron (as now)", label, ref, 1, True, ropt["1x"], side))
        rows.append(combine("single board, back iron, 2x magnets", label, ref2, 1, True, ropt["2x"], side))
        for hm, key in ((1.0, "1x_nonmag"), (2.0, "2x_nonmag")):
            b = boards[hm]
            ka, kb = b["gap_kts"]
            w2k = 2000 * 2 * math.pi / 60
            # parallel boards, one gap 0.1 mm wider: 3 phases, peak EMF difference across 2 R_board
            circ = 0.75 * ((ka - kb) / 1e3 * 2 / 3 * w2k) ** 2 / b["R_phase_ohm"]
            tag = "1x magnets" if hm == 1.0 else "2x magnets"
            for series in (True, False):
                rows.append(combine(f"dual stator, {tag}, {'series' if series else 'parallel'}", label, b, 2,
                                    series, ropt[key], side, gap_circ_W=0.0 if series else circ,
                                    gap_kts=[ka, kb]))
        for r in rows[-6:]:
            print({k: (round(v, 3) if isinstance(v, float) else v) for k, v in r.items()}, flush=True)
    return rows, ropt


def write(rows, ropt):
    (OUT / "pcbmotor_dual_stator.json").write_text(json.dumps(dict(rows=rows, rotors=ropt), indent=2, default=float))
    L = ["# Dual stator: rotor between two stator boards", "",
         "Generated by `pcbmotor/dual_stator.py`. Rotor: the high-momentum payload rotor (8 × 19.05 × 12.7 × 6.35 mm "
         "N42, 0.6 mm gap). Dual stator: no iron; the 3 mm of steel back iron and disc become the same mass of "
         "non-magnetic carrier, so inertia is unchanged. Loss is copper + eddy + strand circulating current at "
         "15 mN·m and 2000 rpm. No-load rpm is at 11.1 V with each board's current turns; series and parallel "
         "bracket a rewind, and turns can put any configuration at any speed (`bus_voltage.py`), so momentum is "
         f"compared at an equal {RPM_COMPARE:.0f} rpm, where only inertia matters. Gap circulating: parallel boards "
         f"with the rotor {GAP_MISMATCH_MM} mm off centre, at 2000 rpm (not in the loss column). Board mass is "
         "bare 1.6 mm FR4 at the laid-out outline (spiral 90 mm, wave 117 mm).", "",
         "| Design | Config | Kt mN·m/A | R Ω | Km mN·m/√W | Loss W | Drag mN·m @2000 | Gap circ. W | No-load rpm | "
         f"H @{RPM_COMPARE:.0f} rpm mN·m·s | Rotor g | Boards g |",
         "|---|---|---|---|---|---|---|---|---|---|---|---|"]
    for r in rows:
        L.append(f"| {r['design']} | {r['config']} | {r['Kt_mNm_per_A']:.1f} | {r['R_phase_ohm']:.2f} | "
                 f"{r['Km_mNm_per_sqrtW']:.1f} | {r['loss_15mNm_2000_W']:.2f} | {r['drag_mNm_2000']:.2f} | "
                 f"{r['gap_circ_W_2000']:.3f} | {r['noload_rpm']:.0f} | {r['H_at_compare_rpm_mNms']:.0f} | "
                 f"{r['rotor_g']:.0f} | {r['boards_g']:.0f} |")
    L += ["", "Per-board Kt with the rotor 0.1 mm off centre (gaps 0.7 / 0.5 mm):", ""]
    for r in rows:
        if r["kt_per_board_at_gap_mismatch"] and r["connection"] == "parallel":
            a, b = r["kt_per_board_at_gap_mismatch"]
            L.append(f"- {r['design']}, {r['config']}: {a:.2f} / {b:.2f} mN·m/A")
    (OUT / "pcbmotor_dual_stator.md").write_text("\n".join(L) + "\n")


def main():
    write(*run())


if __name__ == "__main__":
    main()
