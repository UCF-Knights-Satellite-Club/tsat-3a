"""Solid against stranded test stators: what the two boards in PCBMotorStrandTest/ should measure.

Run from sim/ (after `python3 PCBMotorStrandTest/gen_strand_test.py` if you want the ring tracks
included in the phase resistance):

    .venv/bin/python -m pcbmotor.strand_test

Writes results/pcbmotor_strand_test.json and results/pcbmotor_strand_test.md.

Both boards are the high-momentum payload stator (6 coils / 8 poles, 21-41 mm winding, 1.2 mm
trace, 7 turns per layer) on JLCPCB 4-layer with 2 oz copper on every layer, run with the payload rotor (19.05 x 12.7 x 6.35 mm N42 blocks, 1.5 mm steel back plate, 0.6 mm
gap). The solid board winds one 1.2 mm trace. The stranded board winds three 0.30 mm strands
with 0.15 mm between them, transposed at the layer vias (pcbmotor/bundle.py).

Model (as-built copper from pcbmotor/bundle.py, field and torque as in pcbmotor/em.py):
- Kt, back-EMF, phase resistance: every lane of every layer, with the coil current shared
  between strands by their conductance.
- Eddy drag in the traces: sigma * w^2 / 12 * (dBz/dt)^2 per unit volume (pcbmotor/stacking.py),
  with w the lane width, evaluated on the as-built lanes.
- Circulating current between strands: each strand has its own back-EMF k_s(theta) * omega. The
  strands are joined at both coil terminals, so with the phase open the difference drives a
  current round them: P = omega^2 * sum_s G_s (k_s - k_mean)^2, k_mean weighted by conductance.
  Strand inductance is ignored (it would only reduce this at speed). An untransposed bundle
  (lane s on every layer, which this layout can't build) is shown for comparison.
- Coast-down: with the phases open, drag torque = bearing + c * omega, where c is eddy plus
  circulating. The bearing part is measured separately (rotor spun with no stator), so the
  prediction to compare against is the stator's own drag c * omega, or the extra deceleration
  c * omega / J.
"""
from __future__ import annotations

import json
import math
import pathlib
from dataclasses import replace

import numpy as np

from .geometry import StatorDesign, RotorDesign, OZ_TO_M
from .bundle import strand_test_coil, strand_test_design, check_bundle, STRAND_TEST
from .layout import path_points
from .em import rotor_magnets, foc_torque, RHO_CU_20C, ALPHA_CU, mean_dbz_dtheta2, rotor_bz
from .stacking import coil_curves, SIGMA_CU

ROOT = pathlib.Path(__file__).resolve().parents[2]
OUT = ROOT / "sim" / "results"
BOARD_DIR = ROOT / "PCBMotorStrandTest"
VARIANTS = {k: v["n_strands"] for k, v in STRAND_TEST.items()}
PLATING_M = 20e-6          # via barrel plating (JLC ~18-25 um)
BOARD_T_M = 1.6e-3
RPMS = (1000, 2000, 3000, 4000)


def load_design():
    j = json.loads((OUT / "pcbmotor_designs.json").read_text())["payload_high_momentum"]
    return strand_test_design(StatorDesign(**j["stator"])), RotorDesign(**j["rotor"])


def lane_curves(bc, r: RotorDesign, n_theta=48, max_seg_mm=0.2):
    """Per (layer, lane): torque-per-amp curve on the rotor, and the eddy coefficient
    (W per (rad/s)^2) of that lane in one coil."""
    d = bc.design
    keys, mids, dls, vols, seg_owner = [], [], [], [], []
    w = bc.strand_mm / 1e3
    for L in range(4):
        z = d.stack["z_mm"][L] / 1e3
        t = d.stack["oz"][L] * OZ_TO_M
        for j, path in enumerate(bc.lanes[L]):
            poly = path_points(path, max_seg_mm) / 1e3
            p = np.c_[poly, np.full(len(poly), -z)]
            m = (p[:-1] + p[1:]) / 2
            dl = np.diff(p, axis=0)
            mids.append(m)
            dls.append(dl)
            vols.append(np.linalg.norm(dl, axis=1) * w * t)
            seg_owner.append(np.full(len(m), len(keys)))
            keys.append((L, j))
    mids, dls, vols, own = np.vstack(mids), np.vstack(dls), np.concatenate(vols), np.concatenate(seg_owner)
    period = 2 * (2 * math.pi / r.n_poles)
    thetas = np.linspace(0, period, n_theta, endpoint=False)
    tq = np.zeros((len(keys), n_theta))
    Bz = np.empty((n_theta, len(mids)))
    for i, th in enumerate(thetas):
        B = rotor_magnets(r, th).getB(mids)
        F = np.cross(dls, B)
        tz = mids[:, 0] * F[:, 1] - mids[:, 1] * F[:, 0]
        tq[:, i] = -np.bincount(own, weights=tz, minlength=len(keys))   # torque on the rotor
        Bz[i] = B[:, 2]
    bz, key = rotor_bz(rotor_magnets, r)
    pe = SIGMA_CU * w ** 2 / 12 * mean_dbz_dtheta2(bz, key, r.n_poles, mids) * vols
    eddy = np.bincount(own, weights=pe, minlength=len(keys))
    return thetas, {k: tq[i] for i, k in enumerate(keys)}, {k: eddy[i] for i, k in enumerate(keys)}


def strand_resistance(bc, lanes, temp_c=20.0):
    """Resistance of one strand that uses lane lanes[L] on layer L, with its three vias."""
    from .layout import path_length
    d = bc.design
    rho = RHO_CU_20C * (1 + ALPHA_CU * (temp_c - 20.0))
    r = sum(rho * path_length(bc.lanes[L][j]) / 1e3 / (bc.strand_mm / 1e3 * d.stack["oz"][L] * OZ_TO_M)
            for L, j in enumerate(lanes))
    a_via = math.pi * bc.via_drill_mm / 1e3 * PLATING_M
    return r + 3 * rho * BOARD_T_M / a_via


def bundle_metrics(bc, thetas, tq, eddy, wiring):
    """Coil torque curve, resistance and circulating-loss coefficient for strands that use the
    lanes in `wiring` (list per strand of the lane on each layer)."""
    G = np.array([1 / strand_resistance(bc, w) for w in wiring])
    k = np.array([sum(tq[(L, j)] for L, j in enumerate(w)) for w in wiring])     # (strands, theta)
    k_coil = (G[:, None] * k).sum(axis=0) / G.sum()
    c_circ = float(np.mean((G[:, None] * (k - k_coil) ** 2).sum(axis=0)))
    dk_rms = float(np.sqrt(np.mean((k - k_coil) ** 2)))
    return dict(k_coil=k_coil, R_coil=1 / G.sum(), c_circ_coil=c_circ, strand_R=list(1 / G),
                strand_dk_rms=dk_rms, k=k)


def jumper_predictions(m, krpm, pad_strand=None):
    """J2 open on coil 0 (stranded board): pad 2 = middle strand, pads 1 and 3 = outer strands, all
    joined at the E bus. DC resistance between pads, and the open-circuit voltage between them
    with the rotor spinning (one coil's strand-to-strand back-EMF difference)."""
    k, R = m["k"], m["strand_R"]
    n = len(R)
    mid = n // 2
    # pad number -> strand (from the generator); strand 0 is on L1's outermost lane
    ps = {int(p): s for p, s in (pad_strand or {"1": 0, "2": mid, "3": n - 1}).items()}
    sp = {s: p for p, s in ps.items()}
    out = {}
    for a, b in ((0, mid), (n - 1, mid), (0, n - 1)):
        name = f"pads {sp[a]}-{sp[b]} (strands {a}-{b})"
        dv = (k[a] - k[b]) * krpm * 1e3
        out[name] = dict(R_ohm=R[a] + R[b], V_mV_rms_per_krpm=float(np.sqrt(np.mean(dv ** 2))),
                         V_mV_peak_per_krpm=float(np.abs(dv).max()))
    return out


def ring_ohm(variant, d):
    f = BOARD_DIR / f"strandtest_{variant}_asbuilt.json"
    if not f.exists():
        return None, None
    b = json.loads(f.read_text())
    rho = RHO_CU_20C
    out = {}
    for net, x in b["ring"].items():
        oz_ring = sum(d.stack["oz"][{"In1.Cu": 1, "In2.Cu": 2}[l]] for l in b["ring_layers"])   # in parallel
        r_in = rho * x["inner_mm"] / 1e3 / (b["ring_track_mm"] / 1e3 * oz_ring * OZ_TO_M)
        r_out = rho * x["outer_mm"] / 1e3 / (b["lead_track_mm"] / 1e3 * d.stack["oz"][0] * OZ_TO_M)
        r_via = x["vias"] * rho * BOARD_T_M / (math.pi * b["via_drill_mm"] / 1e3 * PLATING_M)
        out[net] = r_in + r_out + r_via
    return out, b


def analyse(variant, d, r, n_theta=48):
    n = VARIANTS[variant]
    bc = strand_test_coil(d, variant)
    check_bundle(bc)
    thetas, tq, eddy = lane_curves(bc, r, n_theta)
    m = bundle_metrics(bc, thetas, tq, eddy, [[j for _, j in s] for s in bc.strands])
    untr = bundle_metrics(bc, thetas, tq, eddy, [[s] * 4 for s in range(n)]) if n > 1 else None
    kt, ripple, _, _ = foc_torque(d, r, thetas, m["k_coil"])
    ring, board = ring_ohm(variant, d)
    pad_strand = board.get("jumper_pad_strand") if board else None
    per_phase = d.n_coils // 3
    r_coils = m["R_coil"] * per_phase
    r_ph = r_coils + (max(ring.values()) if ring else 0.0)
    c_eddy = d.n_coils * float(sum(eddy.values()))
    c_circ = d.n_coils * m["c_circ_coil"]
    J, m_rot = r.inertia_kgm2()
    krpm = 1000 * 2 * math.pi / 60
    ke = kt * 2 / 3
    res = dict(
        variant=variant, strands=n, strand_width_mm=bc.strand_mm, strand_gap_mm=bc.strand_gap_mm,
        turns_per_layer=bc.n_turns, copper_width_mm=n * bc.strand_mm,
        strand_R_ohm=m["strand_R"],
        R_coil_ohm=m["R_coil"], R_phase_ohm=r_ph, ring_ohm_per_phase=ring,
        phase_R_bench_ohm=({f"{k[-1]}1-{k[-1]}2": r_coils + v for k, v in ring.items()} if ring else None),
        Kt_mNm_per_A=kt * 1e3, ripple_pct=ripple * 100, Km_mNm_per_sqrtW=kt / math.sqrt(1.5 * r_ph) * 1e3,
        phase_backEMF_V_peak_per_krpm=ke * krpm,
        line_backEMF_V_peak_per_krpm_star=ke * krpm * math.sqrt(3),
        electrical_Hz_per_krpm=1000 / 60 * r.n_poles / 2,
        c_eddy=c_eddy, c_circ=c_circ, c_total=c_eddy + c_circ,
        strand_backEMF_spread_mV_rms_per_krpm=m["strand_dk_rms"] * krpm * 1e3,
        rotor_J_kgm2=J,
        drag={rpm: dict(eddy_mNm=c_eddy * rpm * math.pi / 30 * 1e3,
                        circ_mNm=c_circ * rpm * math.pi / 30 * 1e3,
                        total_mNm=(c_eddy + c_circ) * rpm * math.pi / 30 * 1e3,
                        total_W=(c_eddy + c_circ) * (rpm * math.pi / 30) ** 2,
                        decel_rpm_per_s=(c_eddy + c_circ) * rpm * math.pi / 30 / J * 30 / math.pi)
              for rpm in RPMS},
        # with only stator drag (c * omega) the speed decays exponentially with this time constant
        coast_tau_s=J / (c_eddy + c_circ),
    )
    if n > 1:
        res["jumper_J2_open"] = jumper_predictions(m, krpm, pad_strand)
        res["jumper_J2_open_phaseA_R_ohm"] = r_ph + (m["strand_R"][n // 2] - m["R_coil"])
    if untr is not None:
        res["untransposed_jumper"] = jumper_predictions(untr, krpm, pad_strand)
        res["untransposed_c_circ"] = d.n_coils * untr["c_circ_coil"]
        res["untransposed_circ_mNm_at_3000"] = res["untransposed_c_circ"] * 3000 * math.pi / 30 * 1e3
        res["untransposed_strand_backEMF_spread_mV_rms_per_krpm"] = untr["strand_dk_rms"] * krpm * 1e3
    return res, bc


def sweep_estimate(d, r):
    """The design-study estimate (concentric-loop coil, ace_stackups strand factor) for comparison."""
    th, tq, c_solid, _ = coil_curves(d, r, dual=False, n_theta=48)
    w, n, g = d.trace_mm, 3, d.space_mm
    ws = (w - (n - 1) * g) / n
    return dict(c_eddy_solid=c_solid, c_eddy_stranded=c_solid * (ws / w) ** 2 * (n * ws / w))


def main():
    d, r = load_design()
    rows = {}
    for v in VARIANTS:
        rows[v], _ = analyse(v, d, r)
        x = rows[v]
        print(f"{v:9s} Kt={x['Kt_mNm_per_A']:.2f} mNm/A  R_ph={x['R_phase_ohm']:.3f}  c_eddy={x['c_eddy']:.3e}  "
              f"c_circ={x['c_circ']:.3e}  drag@3k={x['drag'][3000]['total_mNm']:.3f} mNm", flush=True)
    est = sweep_estimate(d, r)
    out = dict(design=d.name, stackup=d.stackup, rotor={k: getattr(r, k) for k in r.__dataclass_fields__},
               boards=rows, design_study_estimate=est,
               notes=["Bearing drag is not included: measure it by spinning the rotor with no stator.",
                      "JLC does not publish its 4-layer dielectric thicknesses; layer depths are the "
                      "sim's assumed 1.6 mm build (geometry.STACKUPS['jlc4_2oz']).",
                      "Resistance at 20 C; copper rises 0.39 %/K."])
    (OUT / "pcbmotor_strand_test.json").write_text(json.dumps(out, indent=2, default=float))
    s, t = rows["solid"], rows["stranded"]
    md = ["# Solid vs stranded test stators", "",
          "Generated by `pcbmotor/strand_test.py`. The payload stator geometry and rotor, on JLCPCB 4-layer "
          "(2 oz on all four layers), as laid out in `PCBMotorStrandTest/`. Drag is with the phases open; "
          "bearing drag is not included (measure it with no stator).", "",
          "| | Solid (1 x 1.2 mm) | Stranded (3 x %.2f mm, transposed) |" % t["strand_width_mm"], "|---|---|---|"]

    def row(label, key, fmt, f=lambda x: x):
        md.append(f"| {label} | {fmt.format(f(s[key]))} | {fmt.format(f(t[key]))} |")
    row("Phase resistance Ω (20 °C, incl. ring)", "R_phase_ohm", "{:.3f}")
    row("Kt mN·m/A (FOC, per A peak)", "Kt_mNm_per_A", "{:.2f}")
    row("Phase back-EMF V peak per 1000 rpm", "phase_backEMF_V_peak_per_krpm", "{:.3f}")
    row("Km mN·m/√W", "Km_mNm_per_sqrtW", "{:.2f}")
    row("Eddy coefficient µW/(rad/s)²", "c_eddy", "{:.3f}", lambda x: x * 1e6)
    row("Circulating-current coefficient µW/(rad/s)²", "c_circ", "{:.4f}", lambda x: x * 1e6)
    for rpm in RPMS:
        md.append(f"| Stator drag at {rpm} rpm, mN·m (W) | {s['drag'][rpm]['total_mNm']:.3f} ({s['drag'][rpm]['total_W']:.3f}) "
                  f"| {t['drag'][rpm]['total_mNm']:.3f} ({t['drag'][rpm]['total_W']:.3f}) |")
    md.append(f"| Extra deceleration at 3000 rpm, rpm/s | {s['drag'][3000]['decel_rpm_per_s']:.1f} | "
              f"{t['drag'][3000]['decel_rpm_per_s']:.2f} |")
    md.append(f"| Coast time constant J/c (stator drag only), s | {s['coast_tau_s']:.0f} | {t['coast_tau_s']:.0f} |")
    md += ["", f"Drag ratio solid / stranded: {s['c_total'] / t['c_total']:.1f}x. "
           f"Design-study estimate (concentric loops, strand factor from ace_stackups): "
           f"{est['c_eddy_solid'] / est['c_eddy_stranded']:.1f}x "
           f"({est['c_eddy_solid'] * 1e6:.2f} and {est['c_eddy_stranded'] * 1e6:.3f} µW/(rad/s)²).", "",
           f"Circulating current: transposed strands {t['c_circ'] / t['c_total'] * 100:.1f} % of the stranded "
           f"board's drag. Untransposed (lane s on every layer; not buildable without crossovers) it would be "
           f"{t['untransposed_c_circ'] * 1e6:.3f} µW/(rad/s)², i.e. "
           f"{t['untransposed_circ_mNm_at_3000']:.3f} mN·m at 3000 rpm. Strand-to-strand back-EMF spread: "
           f"{t['strand_backEMF_spread_mV_rms_per_krpm']:.2f} mV rms per 1000 rpm transposed, "
           f"{t['untransposed_strand_backEMF_spread_mV_rms_per_krpm']:.2f} untransposed.", "",
           "## J2 open (stranded board, coil A+ strands separated at its S end)", "",
           "| Pads | DC resistance Ω | Open-circuit V, mV rms per 1000 rpm (peak) | If untransposed |", "|---|---|---|---|"]
    for kk, v in t["jumper_J2_open"].items():
        u = t["untransposed_jumper"][kk]
        md.append(f"| {kk} | {v['R_ohm']:.2f} | {v['V_mV_rms_per_krpm']:.2f} ({v['V_mV_peak_per_krpm']:.2f}) | "
                  f"{u['V_mV_rms_per_krpm']:.1f} ({u['V_mV_peak_per_krpm']:.1f}) |")
    md += ["", f"With J2 open, phase A (A1-A2) reads {t['jumper_J2_open_phaseA_R_ohm']:.2f} Ω (coil A+ on its middle "
           f"strand only) instead of {t['R_phase_ohm']:.2f} Ω.", ""]
    (OUT / "pcbmotor_strand_test.md").write_text("\n".join(md))
    print("\n".join(md))


if __name__ == "__main__":
    main()
