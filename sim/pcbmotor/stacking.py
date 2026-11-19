"""Stacking several stator PCBs, and dual-rotor (sandwich) layouts, against a single board.

Run from sim/:  .venv/bin/python -m pcbmotor.stacking
Writes results/pcbmotor_stacking.json and results/pcbmotor_stacking.md.

Configurations, for the test stator and the recommended (high-momentum) payload stator:

- single:           one board, one rotor (the design as it stands)
- stackN_series:    N identical boards bolted together on one side of the rotor, phases in series
- stackN_parallel:  the same boards with their phases in parallel
- dual1:            one board between two rotors (magnets on both sides, flux straight through)
- dual2_series / _parallel: two boards between two rotors
- more layers:      the same copper pattern on one board with more layers (a fab option, no stacking)

Model notes (same EM model as pcbmotor/em.py):
- A stacked board is just more copper layers, further from the magnets. Boards are 1.6 mm with
  BOARD_GAP_MM between them (spacer / adhesive).
- The second rotor is the first one mirrored through the middle of the stack, with the same
  magnetisation direction at each angle, so the field crosses the stator. Each rotor's back iron is
  the usual image (magnet of twice the thickness). With iron on both sides there are further images
  that this ignores, so dual-rotor torque is slightly underestimated.
- Parallel boards: torque per amp is the mean of the boards' Kt, resistance is R/N. Boards nearer the
  magnets have higher back-EMF, so a circulating current flows between parallel boards at speed. Its
  loss is reported.
- Eddy currents in the traces: a trace of width w in an axial field changing at dB/dt loses
  sigma * w^2 / 12 * (dB/dt)^2 per unit volume. This drag grows with speed and with B^2. The design
  sweep didn't include it.
- Thermal: the continuous limit uses the same convective area as one board (the outer faces only),
  so stacked boards are limited by their total copper loss.
- Axial force between the two rotors (dual rotor) from magpylib, with the back-iron images included.
  That is approximate (+-50%), but it sets the bearing and spacer loads.
"""
from __future__ import annotations

import json
import math
import pathlib
from dataclasses import replace

import numpy as np
import magpylib as magpy

from .geometry import StatorDesign, RotorDesign, STACKUPS, OSHPARK_STACKUPS, coil_turns, OZ_TO_M
from .em import (_coil_segments, rotor_magnets, foc_torque, phase_resistance, phase_inductance,
                 RHO_CU_20C)
from .designs import drive_metrics, V_BUS

OUT = pathlib.Path(__file__).resolve().parents[1] / "results"
BOARD_T_MM = 1.6
BOARD_GAP_MM = 0.3
FR4_DENSITY = 1850.0          # kg/m^3
SIGMA_CU = 1 / RHO_CU_20C
REQ_TORQUE_mNm = (6.0, 15.0)  # yaw-sim requirements behind a swivel: line-driven, gust-driven
REQ_H_mNms = (60.0, 130.0)

BASES = {
    "test": dict(board_area_m2=math.pi * 0.021 ** 2, board_outline_m2=0.050 * 0.0515,
                 more_layers="osh6"),
    "payload_high_momentum": dict(board_area_m2=0.090 * 0.090, board_outline_m2=0.090 * 0.090,
                                  more_layers="jlc6_1oz"),
}


def stack_stackup(base_key, n_boards):
    """Register (and return the name of) a stackup made of n copies of a board."""
    base = dict(STACKUPS[base_key])
    name = f"{base_key}_x{n_boards}"
    pitch = BOARD_T_MM + BOARD_GAP_MM
    STACKUPS[name] = dict(base, z_mm=[z + k * pitch for k in range(n_boards) for z in base["z_mm"]],
                          oz=base["oz"] * n_boards, fab=f"{base['fab']} x{n_boards} boards")
    return name


def board_stackup(base_key, k, n_boards):
    """Stackup holding only board k of an n-board stack (for per-board Kt in parallel)."""
    base = dict(STACKUPS[base_key])
    name = f"{base_key}_b{k}of{n_boards}"
    off = k * (BOARD_T_MM + BOARD_GAP_MM)
    STACKUPS[name] = dict(base, z_mm=[z + off for z in base["z_mm"]])
    return name


def dual_magnets(r: RotorDesign, z_bottom_mm):
    """Magnet collection of a second rotor mirrored through the stack's mid-plane. z_bottom_mm is
    the depth of the lowest copper surface below the top copper."""
    def build(theta):
        top = rotor_magnets(r, theta)
        bot = []
        for m in top.children:
            x, y, z = m.position
            bot.append(m.copy(position=(x, y, -z_bottom_mm / 1e3 - z)))
        return top, magpy.Collection(*bot)
    return build


def coil_curves(d: StatorDesign, r: RotorDesign, dual: bool, n_theta=72):
    """Torque per amp on coil 0 against rotor angle, and the eddy-loss coefficient of the whole
    stator: P_eddy = c_eddy * omega^2 [W/(rad/s)^2]."""
    mids, dls = _coil_segments(d, max_seg_mm=0.25)
    seg_len = np.linalg.norm(dls, axis=1)
    # copper thickness of each segment's layer (segments are stacked layer by layer)
    turns = coil_turns(d)
    per_layer = []
    for z, oz in zip(d.stack["z_mm"], d.stack["oz"]):
        n = 0
        for poly in turns:
            sub = np.linalg.norm(np.diff(poly, axis=0), axis=1)
            n += int(np.sum(np.maximum(1, np.ceil(sub / 0.25))))
        per_layer.append((n, oz * OZ_TO_M))
    t_cu = np.concatenate([np.full(n, t) for n, t in per_layer])
    assert len(t_cu) == len(mids)
    zmax = max(d.stack["z_mm"])
    mags = dual_magnets(r, zmax) if dual else None
    period = 2 * (2 * math.pi / r.n_poles)
    thetas = np.linspace(0, period, n_theta, endpoint=False)
    tq = np.empty(n_theta)
    Bz = np.empty((n_theta, len(mids)))
    for i, th in enumerate(thetas):
        if dual:
            top, bot = mags(th)
            B = top.getB(mids) + bot.getB(mids)
        else:
            B = rotor_magnets(r, th).getB(mids)
        F = np.cross(dls, B)
        tq[i] = np.sum(mids[:, 0] * F[:, 1] - mids[:, 1] * F[:, 0])
        Bz[i] = B[:, 2]
    dth = thetas[1] - thetas[0]
    dBdth = (np.roll(Bz, -1, axis=0) - np.roll(Bz, 1, axis=0)) / (2 * dth)
    w = d.trace_mm / 1e3
    vol = w * t_cu * seg_len
    c_eddy = d.n_coils * float(np.sum(SIGMA_CU * w ** 2 / 12 * np.mean(dBdth ** 2, axis=0) * vol))
    return thetas, -tq, c_eddy, float(np.abs(Bz).max())


def axial_force_N(r: RotorDesign, zmax_mm):
    top, bot = dual_magnets(r, zmax_mm)(0.0)
    F, _ = magpy.getFT(list(top.children), list(bot.children), meshing=200)
    return float(np.sum(np.asarray(F).reshape(-1, 3)[:, 2]))     # + = rotors pull together


def evaluate(label, base, d, r, dual=False, parallel_boards=None, n_boards=1, extra_rotor=None):
    th, tq, c_eddy, bmax = coil_curves(d, r, dual)
    kt, ripple, _, _ = foc_torque(d, r, th, tq)
    R, L = phase_resistance(d), phase_inductance(d)
    circ_W = 0.0
    if parallel_boards:
        # boards in parallel: Kt is the mean, R and L divide by N (L: tight coupling, ~ series L / N^2)
        kts = []
        for bd in parallel_boards:
            thb, tqb, _, _ = coil_curves(bd, r, dual, n_theta=36)
            kts.append(foc_torque(bd, r, thb, tqb)[0])
        kts = np.array(kts)
        n = len(kts)
        kt, R, L = float(kts.mean()), phase_resistance(parallel_boards[0]) / n, L / n ** 2
        r_b = phase_resistance(parallel_boards[0])
    J, m_rot = r.inertia_kgm2()
    if extra_rotor is not None:
        J2, m2 = extra_rotor.inertia_kgm2()
        J, m_rot = J + J2, m_rot + m2
    ke = kt * 2 / 3
    m = drive_metrics(kt, R, L, ke, J, BASES[base]["board_area_m2"], 1.0)
    w_max = m["usable_max_rpm"] * 2 * math.pi / 60
    if parallel_boards:
        # circulating current at max speed between boards with different back-EMF (phase, peak):
        # each board sees (ke_i - mean) * w across its own R_b; 3 phases, rms = peak / sqrt 2
        dke = (kts - kts.mean()) * 2 / 3
        circ_W = float(np.sum(1.5 * (dke * w_max) ** 2 / r_b))
    km = m["Km_mNm_per_sqrtW"]
    p_eddy = c_eddy * w_max ** 2
    # Speed reachable when the motor also has to overcome eddy drag (and circulating-current drag
    # between parallel boards), and the speed at which it still has a torque margin left for control.
    c_drag = c_eddy + (float(np.sum(1.5 * dke ** 2 / r_b)) if parallel_boards else 0.0)
    v_ph = V_BUS / math.sqrt(3)

    def spare_torque(w):                      # Nm left after drag, voltage- and 1 A-limited
        return kt * max(0.0, min(1.0, (v_ph - ke * w) / R)) - c_drag * w

    def top_speed(margin_Nm):
        lo, hi = 0.0, w_max
        if spare_torque(hi) >= margin_Nm:
            return hi
        if spare_torque(lo) < margin_Nm:
            return 0.0
        for _ in range(60):
            mid = 0.5 * (lo + hi)
            lo, hi = (mid, hi) if spare_torque(mid) >= margin_Nm else (lo, mid)
        return lo
    w_drag, w_margin = top_speed(0.0), top_speed(REQ_TORQUE_mNm[0] / 1e3)
    board_mass_g = n_boards * BASES[base]["board_outline_m2"] * BOARD_T_MM / 1e3 * FR4_DENSITY * 1e3
    out = dict(config=label, base=base, stackup=d.stackup, layers=d.layers, n_boards=n_boards, dual_rotor=dual,
               Kt_mNm_per_A=kt * 1e3, R_phase_ohm=R, L_phase_uH=L * 1e6, Km_mNm_per_sqrtW=km,
               ripple_pct=ripple * 100, noload_rpm=m["noload_rpm"], usable_max_rpm=m["usable_max_rpm"],
               H_max_mNms=m["H_max_mNms"], tau_at_1A_mNm=kt * 1e3 * min(1.0, m["i_stall_A"]),
               tau_cont_mNm=m["tau_cont_mNm"],
               **{f"P_cu_at_{t:g}mNm_W": (t / km) ** 2 for t in REQ_TORQUE_mNm},
               eddy_W_at_max_speed=p_eddy, eddy_drag_mNm_at_max_speed=p_eddy / w_max * 1e3 if w_max else 0,
               eddy_W_at_2000rpm=c_eddy * (2000 * 2 * math.pi / 60) ** 2,
               circulating_W_at_max_speed=circ_W, Bz_peak_T=bmax, c_eddy_W_per_rad2s2=c_eddy,
               rpm_with_drag=w_drag * 60 / (2 * math.pi), H_with_drag_mNms=J * w_drag * 1e3,
               rpm_with_6mNm_margin=w_margin * 60 / (2 * math.pi), H_with_6mNm_margin_mNms=J * w_margin * 1e3,
               rotor_J_kgm2=J, rotor_mass_g=m_rot * 1e3, board_mass_g=board_mass_g,
               moving_plus_board_g=m_rot * 1e3 + board_mass_g,
               stack_thickness_mm=max(d.stack["z_mm"]) + 0.0)
    if dual:
        out["axial_force_N"] = axial_force_N(r, max(d.stack["z_mm"]))
    print(f"{label:34s} Kt={out['Kt_mNm_per_A']:6.2f} R={R:6.2f} Km={km:5.2f} n0={m['noload_rpm']:6.0f} "
          f"H={out['H_max_mNms']:6.1f} eddy@max={p_eddy:5.2f}W H_drag={out['H_with_drag_mNms']:6.1f} "
          f"H_6mNm={out['H_with_6mNm_margin_mNms']:6.1f}", flush=True)
    return out


def run_base(base):
    j = json.loads((OUT / "pcbmotor_designs.json").read_text())[base]
    d0, r = StatorDesign(**j["stator"]), RotorDesign(**j["rotor"])
    if d0.stackup is None:
        d0 = replace(d0, stackup=f"osh{d0.layers}")
    key = d0.stackup
    # the second rotor: magnets + back iron only (no extra flywheel disc)
    r2 = replace(r, disc_t_mm=0.0)
    rows = [evaluate("single (as designed)", base, d0, r)]
    for n in (2, 3):
        ds = replace(d0, stackup=stack_stackup(key, n))
        rows.append(evaluate(f"stack{n}_series", base, ds, r, n_boards=n))
        boards = [replace(d0, stackup=board_stackup(key, k, n)) for k in range(n)]
        rows.append(evaluate(f"stack{n}_parallel", base, ds, r, n_boards=n, parallel_boards=boards))
    rows.append(evaluate("dual1 (1 board, 2 rotors)", base, d0, r, dual=True, extra_rotor=r2))
    ds = replace(d0, stackup=stack_stackup(key, 2))
    rows.append(evaluate("dual2_series", base, ds, r, dual=True, n_boards=2, extra_rotor=r2))
    boards = [replace(d0, stackup=board_stackup(key, k, 2)) for k in range(2)]
    rows.append(evaluate("dual2_parallel", base, ds, r, dual=True, n_boards=2, extra_rotor=r2,
                         parallel_boards=boards))
    ml = replace(d0, stackup=BASES[base]["more_layers"])
    rows.append(evaluate(f"more layers: {ml.stackup}", base, ml, r))
    return rows


def table(rows):
    cols = [("config", "Config", "{}"), ("Kt_mNm_per_A", "Kt mN·m/A", "{:.1f}"), ("R_phase_ohm", "R Ω", "{:.2f}"),
            ("Km_mNm_per_sqrtW", "Km mN·m/√W", "{:.2f}"), ("noload_rpm", "No-load rpm", "{:.0f}"),
            ("H_max_mNms", "H, no drag mN·m·s", "{:.1f}"),
            ("H_with_6mNm_margin_mNms", "H with eddy drag, 6 mN·m left mN·m·s", "{:.1f}"),
            ("P_cu_at_6mNm_W", "Cu W at 6 mN·m", "{:.2f}"), ("P_cu_at_15mNm_W", "Cu W at 15 mN·m", "{:.2f}"),
            ("eddy_W_at_max_speed", "Eddy W at max speed", "{:.2f}"),
            ("circulating_W_at_max_speed", "Circulating W", "{:.2f}"),
            ("moving_plus_board_g", "Rotor + boards g", "{:.0f}"), ("axial_force_N", "Rotor pull N", "{:.0f}")]
    head = "| " + " | ".join(c[1] for c in cols) + " |\n|" + "---|" * len(cols) + "\n"
    body = ""
    for r in rows:
        body += "| " + " | ".join((f.format(r[k]) if k in r else "–") for k, _, f in cols) + " |\n"
    return head + body


def main():
    out = {}
    md = ("# PCB stacking study\n\nGenerated by `pcbmotor/stacking.py` "
          f"(boards {BOARD_T_MM} mm with {BOARD_GAP_MM} mm between them; 11.1 V bus, 1 A driver limit). "
          "Copper watts are for the yaw-sim torque requirements (6 mN·m line-driven, 15 mN·m gust-driven).\n\n")
    for base in BASES:
        print(f"--- {base}")
        rows = run_base(base)
        out[base] = rows
        md += f"## {base}\n\n" + table(rows) + "\n"
    (OUT / "pcbmotor_stacking.json").write_text(json.dumps(out, indent=2))
    (OUT / "pcbmotor_stacking.md").write_text(md)
    print(md)


if __name__ == "__main__":
    main()
