"""Spiral coils with their end turns outside the magnets, against the wave winding (2026-11-20).

Why: the wave study (wave.py, RESULTS.md 5f) compared its "ends out" wave (vias at r 15 / 44 mm, active
section r 22-40) with a spiral kept inside r 21-41, so the spiral's end turns sat over the magnets
(19.05 mm blocks at r 31, covering r 21.5-40.5). The spiral was never given the same envelope. This
study gives it that envelope and asks whether the wave is still worth its vias, jumpers and board size.

Same rotor (wave.hm_rotor), drive (1 A, 11.1 V FOC) and operating point (15 mN m at 2000 rpm) as 5f.

Part 1, ideal sweep (geometry.coil_turns, em.analyse, stacking.coil_curves eddy): inner envelope radius
r_in 13-21 mm, outer r_out 41-44 mm, trace 0.5-1.6 mm, JLC 4L 2 oz outer / 1 oz inner (the 5f stackup)
and 2 oz on all four layers (the wave test boards' stackup). Each copper pattern is evaluated with the
two coils of a phase in series and in parallel. Km is the same either way; parallel halves Kt and
doubles the top speed, which is how a many-turn coil keeps the 11.1 V speed range.

Part 2, as built (layout.coil_layout: real 4-layer spirals with the two centre vias and the L2-L3 via,
JLC 0.3 / 0.56 mm vias as on the wave test boards). Torque and eddy loss are computed on exactly that
copper. Phase resistance adds the wiring ring and a lead to the pads (see RING_* below). The ring sits
outside the winding because every coil terminal and the L2-L3 via are at the outer edge (layout.py);
its radial band sets the largest r_out that fits a 90 mm square board.

Part 3, checks on the picks:
- circulating current between the two parallel coils of a phase when the rotor is tilted (the coils
  are diametrically opposite, so a tilt puts one at gap g0 + dg and the other at g0 - dg);
- PWM ripple loss with bus_voltage.ripple_rms. Ls = L_phase - M_AB from a Neumann integral over the
  as-built path of every coil (layout_inductance), the same method bus_voltage.py uses for the wave.
  em.phase_inductance (middle turn x n^2) reads about 4x high for these coils, so it is not used.

Part 4, stranded (added 2026-11-20): the same envelope wound with bundles of 1-4 strands that swap lane
order at every layer via (bundle.py, as on the strand-test board), analysed lane by lane with
strand_test.lane_curves / bundle_metrics: Kt, strand resistance, eddy on each lane's real width and the
current circulating between strands (and, for comparison, between untransposed strands). For each bundle
r_out is the largest that fits the 90 mm board, with the ring placed by the strand-test generator's
clearance rule (clear of the out-via row and the terminal bus arcs, which are as wide as the bundle).
Each bundle is wired with the two coils of a phase in parallel and in series (series: Kt x2, R ~x4),
since a wide bundle has too few turns to stay under the speed cap in parallel. Ls uses the bundle's
middle lane as a filament. Result: 2 x 0.30 mm strands in a 0.75 mm bundle, 13 turns, coils in parallel,
keeps the speed (no-load 4080 rpm, H 87) and cuts eddy 9x for 10% less Km: loss 0.52 W against 0.79 W for
the solid 0.8 mm coil from the same code, drag 0.21 against 1.9 mN m at 2000 rpm. Wide bundles wired in
series reach the speed but lose more to strand gaps (0.65 W at best).

Run from sim/:  .venv/bin/python -m pcbmotor.spiral_ends_out [--reuse]
(--reuse takes the part 1 sweep from the existing JSON and reruns parts 2-4 only, a few minutes.)
Writes results/pcbmotor_spiral_ends_out.json, results/pcbmotor_spiral_ends_out.md and layout images
results/layouts/spiral_ends_out_*.png.
"""
from __future__ import annotations

import json
import math
import pathlib
from dataclasses import asdict, replace

import numpy as np

from .geometry import StatorDesign, OZ_TO_M, phase_map, coil_angles
from .em import analyse, analyse_layout, rotor_magnets, RHO_CU_20C, MU0, mean_dbz_dtheta2, rotor_bz
from .designs import drive_metrics
from .stacking import coil_curves, SIGMA_CU
from .layout import coil_layout, check_layout, path_points
from .bus_voltage import ripple_rms, R_EXT_OHM
from .wave import hm_rotor

RES = pathlib.Path(__file__).resolve().parents[1] / "results"
V_BUS = 11.1
BOARD_MM = 90.0
BOARD_AREA_M2 = 0.090 * 0.090
TAU_OP, RPM_OP = 15.0, 2000.0
W_OP = RPM_OP * 2 * math.pi / 60
STACKUPS = ("jlc4_2oz_out", "jlc4_2oz")
R_INS = (21.0, 19.0, 17.0, 15.0, 13.0)
R_OUTS = (41.0, 42.0, 43.0, 44.0)
WIDTHS = (0.5, 0.6, 0.8, 1.0, 1.2, 1.6)
SPACE = 0.15
# JLC vias, as on the wave test boards (PCBMotorWaveTest)
VIA_DRILL, VIA_DIA = 0.3, 0.56
# Wiring ring outside the winding, on an inner layer. With the two coils of a phase in parallel and
# the star point on the board: two slots for the three 180-degree phase links (A and C share one slot,
# C hops to the other slot where it would overlap A) and one for the star ring. Track 0.5 mm; pitch
# leaves room for a 0.56 mm via on one track next to its neighbour.
RING_SLOTS = 3
RING_W = 0.5
RING_PITCH = VIA_DIA / 2 + SPACE + RING_W / 2          # 0.68 mm
EDGE_CLEAR = 0.3                                       # copper to board edge (JLC 0.3 mm)
LEAD_MM, LEAD_W = 30.0, 1.0                            # ring to the phase pads, on an outer layer
TILT_DGAP_MM = (0.05, 0.1)                             # +-gap change at opposite coils from a tilted rotor


def ring_band_mm(trace_mm):
    """Radial room needed outside r_out: the L2-L3 via (centre r_out + 0.15 + c - trace/2 with
    c from layout.coil_layout), then the ring slots, then the edge clearance."""
    c = VIA_DIA / 2 + SPACE + trace_mm / 2 + 0.02
    via_edge = -(0.15 + trace_mm / 2) + c + VIA_DIA / 2          # relative to r_out
    first = via_edge + SPACE + RING_W / 2
    last_edge = first + (RING_SLOTS - 1) * RING_PITCH + RING_W / 2
    return last_edge + EDGE_CLEAR


def r_out_max(trace_mm):
    return BOARD_MM / 2 - ring_band_mm(trace_mm)


def design(st, w, ri, ro, series=True):
    return StatorDesign(f"spiral_{st}_w{w:g}_r{ri:g}-{ro:g}_{'ser' if series else 'par'}", stackup=st, n_coils=6,
                        r_in_mm=ri, r_out_mm=ro, trace_mm=w, space_mm=SPACE, board_od_mm=BOARD_MM, series=series)


def metrics(kt, R, L, c_eddy, J, c_circ=0.0):
    """Drive envelope and losses at the operating point (kt in N m/A, R ohm, L H)."""
    m = drive_metrics(kt, R, L, kt * 2 / 3, J, BOARD_AREA_M2, 1.0, v_bus=V_BUS)
    km = m["Km_mNm_per_sqrtW"]
    w_top = m["usable_max_rpm"] * 2 * math.pi / 60
    p_cu = (TAU_OP / km) ** 2
    return dict(Kt_mNm_per_A=kt * 1e3, R_phase_ohm=R, Km_mNm_per_sqrtW=km, noload_rpm=m["noload_rpm"],
                H_max_mNms=m["H_max_mNms"], L_phase_uH=L * 1e6,
                eddy_W_at_2000rpm=c_eddy * W_OP ** 2, eddy_W_at_top=c_eddy * w_top ** 2,
                circulating_W_at_2000rpm=c_circ * W_OP ** 2,
                drag_mNm_at_2000rpm=(c_eddy + c_circ) * W_OP * 1e3,
                P_cu_15mNm_W=p_cu, loss_15mNm_2000rpm_W=p_cu + (c_eddy + c_circ) * W_OP ** 2)


# ---------------------------------------------------------------- part 1: ideal sweep
def sweep(rot, log=print):
    rows = []
    J, _ = rot.inertia_kgm2()
    for st in STACKUPS:
        for w in WIDTHS:
            for ri in R_INS:
                for ro in R_OUTS:
                    d = design(st, w, ri, ro, series=True)
                    res, _ = analyse(d, rot, n_theta=36)
                    _, _, c_eddy, _ = coil_curves(d, rot, dual=False, n_theta=36)
                    kt, R, L = res.Kt_Nm_per_A, res.R_phase_ohm, res.L_phase_uH / 1e6
                    for series in (True, False):
                        # parallel: Kt / 2, R / 4, L / 4 (em.analyse gives the same with series=False)
                        k = 1 if series else 2
                        row = dict(stackup=st, trace_mm=w, r_in_mm=ri, r_out_mm=ro, series=series,
                                   turns_per_layer=res.turns_per_layer, ripple_pct=res.torque_ripple_pct,
                                   fits_90mm=ro <= r_out_max(w) + 1e-9,
                                   **metrics(kt / k, R / k ** 2, L / k ** 2, c_eddy, J))
                        rows.append(row)
                    log(f"{st:13s} w{w:<4g} r{ri:g}-{ro:g} N{res.turns_per_layer:2d} Kt(ser) {kt*1e3:5.1f} "
                        f"Km {rows[-1]['Km_mNm_per_sqrtW']:5.2f} eddy {rows[-1]['eddy_W_at_2000rpm']:.2f} W "
                        f"loss {rows[-1]['loss_15mNm_2000rpm_W']:.2f} W")
    return rows


# ---------------------------------------------------------------- part 2: as built
def layout_curves(lay, rot, n_theta=72, max_seg_mm=0.25):
    """Torque per amp of one as-built coil against rotor angle, and the whole stator's eddy
    coefficient (P = c * omega^2), both on exactly the layout's copper (all layers, via stubs)."""
    d = lay.design
    mids, dls, t_cu = [], [], []
    for z, oz, path in zip(d.stack["z_mm"], d.stack["oz"], lay.layers):
        poly = path_points(path, max_seg_mm)
        p = np.c_[poly / 1e3, np.full(len(poly), -z / 1e3)]
        mids.append((p[:-1] + p[1:]) / 2)
        dls.append(np.diff(p, axis=0))
        t_cu.append(np.full(len(p) - 1, oz * OZ_TO_M))
    mids, dls, t_cu = np.vstack(mids), np.vstack(dls), np.concatenate(t_cu)
    period = 2 * (2 * math.pi / rot.n_poles)
    thetas = np.linspace(0, period, n_theta, endpoint=False)
    tq = np.empty(n_theta)
    Bz = np.empty((n_theta, len(mids)))
    for i, th in enumerate(thetas):
        B = rotor_magnets(rot, th).getB(mids)
        F = np.cross(dls, B)
        tq[i] = np.sum(mids[:, 0] * F[:, 1] - mids[:, 1] * F[:, 0])
        Bz[i] = B[:, 2]
    bz, key = rotor_bz(rotor_magnets, rot)
    g = mean_dbz_dtheta2(bz, key, rot.n_poles, mids)
    w = d.trace_mm / 1e3
    vol = w * t_cu * np.linalg.norm(dls, axis=1)
    c_eddy = d.n_coils * float(np.sum(SIGMA_CU * w ** 2 / 12 * g * vol))
    return thetas, -tq, c_eddy


def _coil_path_m(lay, angle=0.0, seg_mm=0.5):
    """One coil's as-built copper, all four layers in series order, as one 3-D polyline (m)."""
    d = lay.design
    pts = []
    for z, path in zip(d.stack["z_mm"], lay.layers):
        p = path_points(path, seg_mm)
        pts.append(np.c_[p, np.full(len(p), -z)])
    P = np.vstack(pts) / 1e3
    c, s = math.cos(angle), math.sin(angle)
    return P @ np.array([[c, s, 0.0], [-s, c, 0.0], [0.0, 0.0, 1.0]])


def _neumann(p1, p2, chunk=2000):
    a, da = (p1[:-1] + p1[1:]) / 2, np.diff(p1, axis=0)
    b, db = (p2[:-1] + p2[1:]) / 2, np.diff(p2, axis=0)
    tot = 0.0
    for i in range(0, len(a), chunk):
        r = np.linalg.norm(a[i:i + chunk, None, :] - b[None, :, :], axis=2)
        tot += float(np.sum((da[i:i + chunk] @ db.T) / r))
    return MU0 / (4 * math.pi) * tot


def layout_inductance(lay, rot):
    """Synchronous inductance Ls = L_A - M_AB [H] of the as-built stator, by the Neumann formula.
    Self term: the coil's centreline against a copy lifted by the GMD of the trace section,
    0.2235 * (w + t), as bus_voltage.py does for the wave. Coil-to-coil terms by symmetry: M(k) between
    coils k * 60 degrees apart. Series: phase flux = sum over both coils. Parallel: each coil carries
    half the current and the phase voltage is one branch's, so the phase sums are halved."""
    d = lay.design
    P = _coil_path_m(lay)
    gmd = 0.2235 * (d.trace_mm + d.stack["oz"][0] * OZ_TO_M * 1e3) / 1e3
    Q = P.copy()
    Q[:, 2] += gmd
    M = {0: _neumann(P, Q)}
    for k in range(1, d.n_coils // 2 + 1):
        M[k] = _neumann(P, _coil_path_m(lay, k * 2 * math.pi / d.n_coils))
    pm = phase_map(d, rot.n_poles)
    n = d.n_coils

    def m(i, j):
        k = abs(i - j) % n
        return pm[i][1] * pm[j][1] * M[min(k, n - k)]

    A = [i for i in range(n) if pm[i][0] == 0]
    B = [i for i in range(n) if pm[i][0] == 1]
    L_A = sum(m(i, j) for i in A for j in A)
    M_AB = sum(m(i, j) for i in A for j in B)
    if not d.series:
        L_A, M_AB = L_A / len(A) ** 2, M_AB / len(A) ** 2
    return L_A - M_AB, dict(L_coil_uH=M[0] * 1e6, L_phase_uH=L_A * 1e6, M_AB_uH=M_AB * 1e6,
                            M_k_uH={k: v * 1e6 for k, v in M.items() if k})


def wiring_ohm(d: StatorDesign, ro, r_ring=None):
    """Ring and lead resistance in one phase. Parallel: the far coil's branch has a 180-degree link on
    the ring; the star ring joins the coil ends close by and is ignored. Series: one 180-degree link
    in the phase path. Both: a lead to the pads on an outer layer, and 4 extra vias."""
    if r_ring is None:
        r_ring = ro + ring_band_mm(d.trace_mm) - EDGE_CLEAR - RING_W / 2 - RING_PITCH   # middle slot
    t_in = d.stack["oz"][1] * OZ_TO_M
    link = RHO_CU_20C * (math.pi * r_ring / 1e3) / (RING_W / 1e3 * t_in)
    lead = RHO_CU_20C * (LEAD_MM / 1e3) / (LEAD_W / 1e3 * d.stack["oz"][0] * OZ_TO_M)
    via = RHO_CU_20C * 1.6e-3 / (math.pi * VIA_DRILL / 1e3 * 25e-6)
    return link, lead + 4 * via


def as_built(st, w, ri, ro, series, rot, log=print):
    d = design(st, w, ri, ro, series=series)
    lay = coil_layout(d, via_dia_mm=VIA_DIA, via_drill_mm=VIA_DRILL)
    check_layout(lay)
    res = analyse_layout(lay, rot, n_theta=72)
    thetas, tq, c_eddy = layout_curves(lay, rot)
    r_coil = res["R_coil_ohm"]
    link, extra = wiring_ohm(d, ro)
    if series:
        R = 2 * r_coil + link + extra
    else:
        R = (2 * r_coil + link + extra) if series else r_coil * (r_coil + link) / (2 * r_coil + link) + extra
    kt = res["Kt_Nm_per_A"]
    L, l_parts = layout_inductance(lay, rot)
    J, _ = rot.inertia_kgm2()
    c_circ, circ = 0.0, {}
    if not series:
        for dg in TILT_DGAP_MM:
            circ[f"circulating_W_at_2000rpm_tilt_{dg}mm"] = tilt_circulating(lay, rot, dg, r_coil + link / 2) * W_OP ** 2
        c_circ = circ[f"circulating_W_at_2000rpm_tilt_{TILT_DGAP_MM[0]}mm"] / W_OP ** 2
    out = dict(stackup=st, trace_mm=w, r_in_mm=ri, r_out_mm=ro, series=series, fits_90mm=ro <= r_out_max(w) + 1e-9,
               board_mm_needed=2 * (ro + ring_band_mm(w)),
               turn_rings=lay.n_turns, turns_per_layer=[round(x, 2) for x in lay.turns_per_layer()],
               R_coil_ohm=r_coil, R_wiring_ohm=(link + extra) if series else R - r_coil / 2,
               ripple_pct=res["torque_ripple_pct"], Ls_uH=L * 1e6, inductance=l_parts,
               **metrics(kt, R, L, c_eddy, J, c_circ=c_circ), **circ)
    # PWM ripple at the operating point (Ls taken as the phase self-inductance)
    ke = kt * 2 / 3
    i_op = TAU_OP / 1e3 / kt
    v_need = math.hypot(ke * W_OP + R * i_op, rot.n_poles // 2 * W_OP * L * i_op)
    for f in (50e3, 100e3):
        x = ripple_rms(V_BUS, L, f, v_need)
        out[f"pwm_ripple_W_2000rpm_{int(f / 1e3)}kHz"] = 3 * (R + R_EXT_OHM) * x ** 2
    out["lay"] = lay
    log(f"as built {d.name}: rings {lay.n_turns}, Kt {out['Kt_mNm_per_A']:.1f}, R {R:.3f}, Km {out['Km_mNm_per_sqrtW']:.2f}, "
        f"Ls {L * 1e6:.1f} uH, eddy {out['eddy_W_at_2000rpm']:.2f} W, loss {out['loss_15mNm_2000rpm_W']:.2f} W, n0 {out['noload_rpm']:.0f}, "
        f"board {out['board_mm_needed']:.1f} mm")
    return out


def tilt_circulating(lay, rot, dgap_mm, r_branch):
    """Circulating-current coefficient (P = c * omega^2) for one tilted rotor: the two parallel coils
    of each phase see gaps g0 + dg and g0 - dg. Their EMF difference drives a current round the loop
    of the two branches (resistance 2 * r_branch; reactance ignored, which overstates it)."""
    curves = []
    for s in (+1, -1):
        r2 = replace(rot, airgap_mm=rot.airgap_mm + s * dgap_mm)
        _, tq, _ = layout_curves(lay, r2, n_theta=36)
        curves.append(tq)
    de = curves[0] - curves[1]                  # EMF difference per rad/s (V s/rad) against angle
    return 3 * float(np.mean(de ** 2)) / (2 * r_branch)


# ---------------------------------------------------------------- part 4: stranded
# (strands, bundle width). Each is run with the coils of a phase in parallel and in series. The speed
# target (no-load 4000-4300 rpm, H 85-90 mN m s) needs ~13 turns in parallel or ~6-7 in series.
# bundle.py cannot place the via row for 3 x 1.0, 3 x 1.8, 4 x 1.6 or 4 x 2.0 (the last turn's lanes
# end past the opening corner), so those are left out.
STRANDED = [(1, 0.8), (1, 1.2), (2, 0.75), (2, 0.8), (2, 0.9), (2, 1.0), (2, 1.2), (3, 1.2), (2, 1.6), (3, 1.6), (3, 2.0),
            (4, 1.8)]
NOLOAD_WINDOW = (4000.0, 4300.0)


def _bundle_ring_first(bc):
    """Centre radius of the innermost ring track: the strand-test generator's rule (gen_strand_test.py),
    clear of the out-via row and of the bus arcs (as wide as the bundle) with a via on the track."""
    d, vr = bc.design, bc.via_dia_mm / 2
    return max(bc.r_out_row + vr + d.space_mm + RING_W / 2,
               bc.r_bus + d.trace_mm / 2 + d.space_mm + vr) + 0.05


def _fit_bundle(n, W, ri=15.0, st="jlc4_2oz", series=False):
    """Bundle coil with the largest r_out (0.05 mm steps) whose ring still fits the 90 mm board.
    The ring position moves one-for-one with r_out, so this settles in one or two steps."""
    from .bundle import bundle_coil, check_bundle
    ro = 42.0
    for _ in range(8):
        d = design(st, W, ri, ro, series=series)
        bc = bundle_coil(d, n, via_dia_mm=VIA_DIA, via_drill_mm=VIA_DRILL)
        outer = _bundle_ring_first(bc) + (RING_SLOTS - 1) * RING_PITCH + RING_W / 2 + EDGE_CLEAR
        over = outer - BOARD_MM / 2
        if -0.05 < over <= 1e-9:
            break
        ro = round(math.floor((ro - over) / 0.05 + 1e-6) * 0.05, 2)
    else:
        raise AssertionError(f"r_out fit did not settle for n{n} W{W}")
    check_bundle(bc)
    return d, bc, ro


class _Filament:
    """A bundle coil seen as one filament per layer (its middle lane), for layout_inductance."""
    def __init__(self, bc):
        self.design = bc.design
        self.layers = [bc.lanes[L][bc.n_strands // 2] for L in range(4)]


_STRAND_CACHE = {}


def _stranded_field(n, W, rot):
    """Fitted bundle coil, its lane curves and the inductance-path geometry; shared by both wirings
    (the copper is the same, only the coil-to-coil wiring differs)."""
    from .strand_test import lane_curves
    key = (n, W, repr(rot))
    if key not in _STRAND_CACHE:
        d, bc, ro = _fit_bundle(n, W)
        thetas, tq, eddy = lane_curves(bc, rot, n_theta=48)
        _STRAND_CACHE[key] = (d, bc, ro, thetas, tq, eddy)
    return _STRAND_CACHE[key]


def stranded(n, W, rot, series=False, log=print):
    """One stranded design. Parallel: the two coils of a phase side by side, each branch carrying a
    180-degree ring link on the far coil (as wiring_ohm). Series: the two coils and one link in a row."""
    from .strand_test import bundle_metrics
    from .em import foc_torque
    d0, bc, ro, thetas, tq, eddy = _stranded_field(n, W, rot)
    d = replace(d0, series=series, name=design("jlc4_2oz", W, 15.0, ro, series=series).name)
    bc = replace(bc, design=d)
    m = bundle_metrics(bc, thetas, tq, eddy, [[j for _, j in st] for st in bc.strands])
    kt, ripple, _, _ = foc_torque(d, rot, thetas, m["k_coil"])          # halved when the coils are in parallel
    r_coil = m["R_coil"]
    r_ring = _bundle_ring_first(bc) + RING_PITCH                        # middle slot
    link, extra = wiring_ohm(d, ro, r_ring=r_ring)
    if series:
        R = 2 * r_coil + link + extra
    else:
        R = r_coil * (r_coil + link) / (2 * r_coil + link) + extra
    c_eddy = d.n_coils * float(sum(eddy.values()))
    c_circ = d.n_coils * m["c_circ_coil"]
    out = dict(strands=n, bundle_mm=W, series=series, strand_mm=bc.strand_mm, r_in_mm=15.0, r_out_mm=ro, turns=bc.n_turns,
               copper_fraction=n * bc.strand_mm / W, R_coil_ohm=r_coil, R_wiring_ohm=R - (2 * r_coil if series else r_coil / 2),
               ripple_pct=ripple * 100)
    if n > 1:      # what the lane swaps buy: the same strands kept on one lane on every layer
        un = bundle_metrics(bc, thetas, tq, eddy, [[s_] * 4 for s_ in range(n)])
        out["untransposed_circulating_W_at_2000rpm"] = d.n_coils * un["c_circ_coil"] * W_OP ** 2
    L, l_parts = layout_inductance(_Filament(bc), rot)
    J, _ = rot.inertia_kgm2()
    out.update(Ls_uH=L * 1e6, **metrics(kt, R, L, c_eddy, J, c_circ=c_circ))
    ke = kt * 2 / 3
    i_op = TAU_OP / 1e3 / kt
    out["I_op_A"] = i_op
    v_need = math.hypot(ke * W_OP + R * i_op, rot.n_poles // 2 * W_OP * L * i_op)
    for f in (50e3, 100e3):
        x = ripple_rms(V_BUS, L, f, v_need)
        out[f"pwm_ripple_W_2000rpm_{int(f / 1e3)}kHz"] = 3 * (R + R_EXT_OHM) * x ** 2
    log(f"stranded n{n} W{W} {'ser' if series else 'par'}: ws {bc.strand_mm:.3f}, r_out {ro:.2f}, {bc.n_turns} turns, Kt {out['Kt_mNm_per_A']:.1f}, "
        f"R {R:.3f}, Km {out['Km_mNm_per_sqrtW']:.2f}, eddy {out['eddy_W_at_2000rpm']:.3f} W, "
        f"circ {out['circulating_W_at_2000rpm']:.4f} W, loss {out['loss_15mNm_2000rpm_W']:.2f} W, n0 {out['noload_rpm']:.0f}, "
        f"Ls {L * 1e6:.1f} uH", flush=True)
    return out


def write_stranded_md(rows):
    L = ["", "## Stranded (strands swap lanes at every layer via, bundle.py), r_in 15, r_out fitted to 90 mm", "",
         "JLC 4L 2 oz on all layers, JLC 0.3 / 0.56 mm vias, ring and leads as above, strand gap 0.15 mm. The first row is "
         "the solid 0.8 mm coil built by the same code. Drag = eddy + circulating between strands, at 2000 rpm. "
         "\"Untransposed\" is the circulating loss if each strand kept its lane on every layer. Rows in **bold** land "
         f"in the {NOLOAD_WINDOW[0]:.0f}–{NOLOAD_WINDOW[1]:.0f} rpm no-load window (H about 85–90 mN·m·s). "
         "bundle.py cannot lay out 3 × 1.0, 3 × 1.8, 4 × 1.6 or 4 × 2.0 mm bundles (via row in the opening), so they are "
         "missing.", "",
         "| Strands × width | Bundle | r_out | Turns | Coils | Kt | R Ω | Km | No-load rpm / H | Eddy | Circulating (untransposed) | "
         "Drag @2000 | Loss | Ls µH | PWM ripple 50 / 100 kHz |",
         "|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|"]
    for r in rows:
        un = f" ({r['untransposed_circulating_W_at_2000rpm'] * 1e3:.0f} mW)" if r["strands"] > 1 else ""
        hit = NOLOAD_WINDOW[0] - 100 <= r["noload_rpm"] <= NOLOAD_WINDOW[1] + 100
        b = "**" if hit else ""
        L.append(f"| {b}{r['strands']} × {r['strand_mm']:.3g} mm{b} | {r['bundle_mm']:g} mm | {r['r_out_mm']:.2f} | {r['turns']} | "
                 f"{'series' if r['series'] else 'parallel'} | {r['Kt_mNm_per_A']:.1f} | {r['R_phase_ohm']:.3f} | {r['Km_mNm_per_sqrtW']:.1f} | "
                 f"{b}{r['noload_rpm']:.0f} / {r['H_max_mNms']:.0f}{b} | {r['eddy_W_at_2000rpm']:.3f} W | "
                 f"{r['circulating_W_at_2000rpm'] * 1e3:.2f} mW{un} | {r['drag_mNm_at_2000rpm']:.2f} mN·m | "
                 f"{b}{r['loss_15mNm_2000rpm_W']:.2f} W{b} | {r['Ls_uH']:.1f} | "
                 f"{r['pwm_ripple_W_2000rpm_50kHz']:.2f} / {r['pwm_ripple_W_2000rpm_100kHz']:.2f} W |")
    solid = next((r for r in rows if r["strands"] == 1 and not r["series"]), None)
    if solid:
        L += ["", "Eddy against the solid coil of the same bundle width, model vs (strand / bundle)² × copper fraction:", "",
              "| Bundle | Strands | Eddy ratio | (w_s / W)² × n w_s / W |", "|---|---|---|---|"]
        ref = {r["bundle_mm"]: r for r in rows if r["strands"] == 1 and not r["series"]}
        for r in rows:
            if r["strands"] > 1 and not r["series"] and r["bundle_mm"] in ref:
                f = (r["strand_mm"] / r["bundle_mm"]) ** 2 * r["copper_fraction"]
                L.append(f"| {r['bundle_mm']:g} mm | {r['strands']} | {r['eddy_W_at_2000rpm'] / ref[r['bundle_mm']]['eddy_W_at_2000rpm']:.3f} "
                         f"| {f:.3f} |")
    return L


# ---------------------------------------------------------------- report
def wave_rows():
    """Wave reference rows: the 5f ideal recommendation and the as-built test boards."""
    out = []
    w = json.loads((RES / "pcbmotor_wave.json").read_text())
    for r in w:
        if r["name"] == "wave_env15-44_act22-40_n48_cap1.6_s2swap":
            out.append(dict(label="Wave n48, 2 strands swapped, ideal (5f)", stackup="jlc4_2oz_out",
                            Kt=r["Kt_mNm_per_A"], Km=r["Km_mNm_per_sqrtW"], eddy=r["eddy_W_at_2000rpm"],
                            loss=r["loss_15mNm_2000rpm_W"], board="90 mm (ideal, no jumpers)"))
    for r in json.loads((RES / "pcbmotor_wave_asbuilt.json").read_text()):
        if r["name"] in ("swap", "bv_swap") and r["stackup"] == "jlc4_2oz":
            out.append(dict(label=f"Wave n48, 2 strands swapped, as built ({'blind vias' if r['buried'] else 'through vias'})",
                            stackup="jlc4_2oz", Kt=r["Kt_mNm_per_A"], Km=r["Km_mNm_per_sqrtW"],
                            eddy=r["eddy_W_at_2000rpm"] + r["circulating_W_at_2000rpm"],
                            loss=r["loss_15mNm_2000rpm_W"], board="~117 mm" if r["buried"] else "~129 mm"))
    return out


def _f(x, n=2):
    return f"{x:.{n}f}"


def write_md(rows, built, waves, path, strand_rows=()):
    L = ["# Spiral coils with the end turns outside the magnets (2026-11-20)", "",
         "`pcbmotor/spiral_ends_out.py`. Raw numbers: `pcbmotor_spiral_ends_out.json`. As-built layouts: "
         + ", ".join(f"[`{n}`](layouts/{n}.png)" for n in ("spiral_ends_out_w0.6_r15-42", "spiral_ends_out_w0.8_r15-42",
                                                         "spiral_ends_out_w1_r15-42", "spiral_ends_out_w0.8_r15-44")) + ".", "",
         "Same rotor, drive and operating point as the wave study (5f): 8 × 19.05 × 12.7 × 6.35 mm N42 blocks at "
         "r 31 mm (r 21.5–40.5), 0.6 mm gap, 3 mm steel; 1 A, 11.1 V FOC; loss = copper + eddy (+ circulating) at "
         "15 mN·m and 2000 rpm.", "",
         "## Why", "",
         "5f compared the wave, with its end turns outside the magnets (vias at r 15 / 44), against a spiral kept "
         "inside r 21–41, so the spiral's end turns sat over the magnets. Here the spiral gets the same room.", "",
         "## Ideal model: Km against envelope (6 coils / 8 poles, 2 oz outer / 1 oz inner)", ""]
    st = "jlc4_2oz_out"
    for w in (0.8, 1.2):
        L += [f"Trace {w} mm. Km (turns per layer):", "",
              "| r_in \\ r_out | " + " | ".join(f"{ro:g}" for ro in R_OUTS) + " |",
              "|---" * (len(R_OUTS) + 1) + "|"]
        for ri in R_INS:
            cells = []
            for ro in R_OUTS:
                x = next(r for r in rows if r["stackup"] == st and r["trace_mm"] == w and r["r_in_mm"] == ri
                         and r["r_out_mm"] == ro and r["series"])
                cells.append(f"{x['Km_mNm_per_sqrtW']:.1f} ({x['turns_per_layer']})")
            L.append(f"| {ri:g} | " + " | ".join(cells) + " |")
        L.append("")
    L += [f"With the wiring ring outside the winding, a 90 mm board allows r_out up to "
          f"{r_out_max(0.8):.1f} mm (0.8 mm traces) or {r_out_max(1.2):.1f} mm (1.2 mm): the L2–L3 via, "
          f"{RING_SLOTS} ring tracks at {RING_PITCH:.2f} mm pitch and {EDGE_CLEAR} mm edge clearance take "
          f"{ring_band_mm(0.8):.1f} mm.", "",
          "## Ideal model: best design per width, r_in 15, r_out 42 (fits 90 mm)", "",
          "| Stackup | Trace | Turns | Wiring | Kt | R Ω | Km | No-load rpm | H mN·m·s | Eddy @2000 | Loss |",
          "|---|---|---|---|---|---|---|---|---|---|---|"]
    for st in STACKUPS:
        for w in WIDTHS:
            for series in (True, False):
                x = next(r for r in rows if r["stackup"] == st and r["trace_mm"] == w and r["r_in_mm"] == 15
                         and r["r_out_mm"] == 42 and r["series"] == series)
                L.append(f"| {st} | {w:g} | {x['turns_per_layer']} | {'series' if series else 'parallel'} | "
                         f"{_f(x['Kt_mNm_per_A'], 1)} | {_f(x['R_phase_ohm'])} | {_f(x['Km_mNm_per_sqrtW'], 1)} | "
                         f"{x['noload_rpm']:.0f} | {_f(x['H_max_mNms'], 0)} | {_f(x['eddy_W_at_2000rpm'])} | "
                         f"{_f(x['loss_15mNm_2000rpm_W'])} |")
    L += ["", "## As built (real spirals, JLC 0.3 / 0.56 mm vias, ring and leads) against the wave", "",
          "| Design | Stackup | Kt | R Ω | Km | No-load rpm | H mN·m·s | Eddy + circ @2000 | Loss | Ls µH | PWM ripple 50 / 100 kHz | Board |",
          "|---|---|---|---|---|---|---|---|---|---|---|---|"]
    for b in built:
        lab = (f"Spiral w{b['trace_mm']:g} r{b['r_in_mm']:g}–{b['r_out_mm']:g}, {b['turn_rings']} turns, "
               f"{'series' if b['series'] else 'parallel'}")
        L.append(f"| {lab} | {b['stackup']} | {_f(b['Kt_mNm_per_A'], 1)} | {_f(b['R_phase_ohm'], 3)} | "
                 f"**{_f(b['Km_mNm_per_sqrtW'], 1)}** | {b['noload_rpm']:.0f} | {b['H_max_mNms']:.0f} | "
                 f"{_f(b['eddy_W_at_2000rpm'] + b['circulating_W_at_2000rpm'])} W | **{_f(b['loss_15mNm_2000rpm_W'])} W** | "
                 f"{b['Ls_uH']:.1f} | {_f(b['pwm_ripple_W_2000rpm_50kHz'])} / {_f(b['pwm_ripple_W_2000rpm_100kHz'])} W | "
                 f"{b['board_mm_needed']:.1f} mm |")
    for w in waves:
        L.append(f"| {w['label']} | {w['stackup']} | {_f(w['Kt'], 1)} | – | {_f(w['Km'], 1)} | ~4000 | 85 | "
                 f"{_f(w['eddy'])} W | {_f(w['loss'])} W | 8.1 (5h) | 1.05 / 0.26 W (5h) | {w['board']} |")
    L.append("")
    par = [b for b in built if not b["series"]]
    if par:
        L += ["Circulating current between the two parallel coils of a phase with the rotor tilted "
              "(gap ±dg at opposite coils), at 2000 rpm:", "",
              "| Design | " + " | ".join(f"±{dg} mm" for dg in TILT_DGAP_MM) + " |",
              "|---" * (len(TILT_DGAP_MM) + 1) + "|"]
        for b in par:
            L.append(f"| w{b['trace_mm']:g} r{b['r_in_mm']:g}–{b['r_out_mm']:g} {b['stackup']} | " + " | ".join(
                f"{b[f'circulating_W_at_2000rpm_tilt_{dg}mm'] * 1e3:.2f} mW" for dg in TILT_DGAP_MM) + " |")
        L.append("")
    if strand_rows:
        L += write_stranded_md(strand_rows)
    path.write_text("\n".join(L) + "\n")


def plot(b, rot, path):
    """As-built copper of the whole stator, all four layers, plus the ring band and the board."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from .layout_gallery import _draw_path, _magnets, _board, _setup, _save, PH_COL
    from .geometry import phase_map, coil_angles
    lay = b["lay"]
    d = lay.design
    fig, axes = plt.subplots(1, 3, figsize=(16, 5.6))
    h = BOARD_MM / 2 + 2
    _setup(axes[0], ((-h, h), (-h, h)), "L1 (L3 is the same, L2/L4 mirrored)")
    _setup(axes[1], ((-h, h), (-h, h)), "All four layers")
    tau = 2 * math.pi / d.n_coils
    half = (d.r_out_mm + 4) * math.sin(tau / 2) + 1
    _setup(axes[2], ((d.r_in_mm * math.cos(tau / 2) - 3, h), (-half, half)), "Zoom: one coil, all layers")
    pm = phase_map(d, rot.n_poles)
    ring_r = [d.r_out_mm + ring_band_mm(d.trace_mm) - EDGE_CLEAR - RING_W / 2 - k * RING_PITCH for k in range(RING_SLOTS)]
    for ai, ax in enumerate(axes):
        _board(ax, BOARD_MM, True)
        layers = [0] if ai == 0 else [0, 1, 2, 3]
        for (ph, sgn), phi in zip(pm, coil_angles(d)):
            c, s = math.cos(phi), math.sin(phi)
            for li in layers:
                p = path_points(lay.layers[li], 0.2)
                _draw_path(ax, p[:, 0] * c - p[:, 1] * s, p[:, 0] * s + p[:, 1] * c, d.trace_mm, PH_COL[ph],
                           alpha=1.0 if li == 0 else 0.35)
            for v in (lay.via_in, lay.via_out, lay.via_core):
                x, y = v[0] * c - v[1] * s, v[0] * s + v[1] * c
                ax.add_patch(plt.Circle((x, y), VIA_DIA / 2, color="k", zorder=6))
        t = np.linspace(0, 2 * math.pi, 400)
        for r in ring_r:
            ax.plot(r * np.cos(t), r * np.sin(t), color="0.6", lw=RING_W * 1.2, zorder=1)
        _magnets(ax, rot)
    km = b["Km_mNm_per_sqrtW"]
    fig.suptitle(f"{d.name} (as built: {lay.n_turns} turn rings, JLC {VIA_DRILL}/{VIA_DIA} mm vias; grey = wiring ring band)\n"
                 f"Kt {b['Kt_mNm_per_A']:.1f} mN·m/A ({'parallel' if not b['series'] else 'series'}), Km {km:.1f}, "
                 f"loss @15 mN·m/2000 rpm {b['loss_15mNm_2000rpm_W']:.2f} W, board {b['board_mm_needed']:.1f} mm needed",
                 fontsize=10)
    fig.subplots_adjust(top=0.84, wspace=0.18, left=0.04, right=0.99, bottom=0.08)
    _save(fig, path)


# as-built picks: (stackup, trace, r_in, r_out, series)
PICKS = [
    ("jlc4_2oz_out", 1.2, 21.0, 41.0, True),     # the 5f spiral baseline, as built
    ("jlc4_2oz_out", 0.8, 15.0, 42.0, False),
    ("jlc4_2oz", 1.2, 21.0, 41.0, True),
    ("jlc4_2oz", 0.6, 15.0, 42.0, False),
    ("jlc4_2oz", 0.8, 15.0, 42.0, False),
    ("jlc4_2oz", 1.0, 15.0, 42.0, False),
    ("jlc4_2oz", 0.8, 15.0, 44.0, False),        # needs the bigger board: what r_out 44 would buy
]


def main():
    import sys
    rot = hm_rotor()
    js = RES / "pcbmotor_spiral_ends_out.json"
    rows = json.loads(js.read_text())["sweep"] if "--reuse" in sys.argv and js.exists() else sweep(rot)
    built = [as_built(*p, rot) for p in PICKS]
    waves = wave_rows()
    strand_rows = [stranded(n, W, rot, series=ser) for n, W in STRANDED for ser in (False, True)]
    (RES / "layouts").mkdir(exist_ok=True)
    for b in built:
        if not b["series"] and b["stackup"] == "jlc4_2oz":
            name = f"spiral_ends_out_w{b['trace_mm']:g}_r{b['r_in_mm']:g}-{b['r_out_mm']:g}"
            plot(b, rot, RES / "layouts" / f"{name}.png")
    write_md(rows, built, waves, RES / "pcbmotor_spiral_ends_out.md", strand_rows)
    clean = [{k: v for k, v in b.items() if k != "lay"} for b in built]
    (RES / "pcbmotor_spiral_ends_out.json").write_text(json.dumps(
        dict(rotor=asdict(rot), ring=dict(slots=RING_SLOTS, width_mm=RING_W, pitch_mm=RING_PITCH,
                                          edge_clear_mm=EDGE_CLEAR, band_mm_at_0p8=ring_band_mm(0.8),
                                          r_out_max_90mm_at_0p8=r_out_max(0.8)),
             vias=dict(drill_mm=VIA_DRILL, pad_mm=VIA_DIA), sweep=rows, as_built=clean, stranded=strand_rows,
             wave=waves), indent=1))
    print((RES / "pcbmotor_spiral_ends_out.md").read_text())


if __name__ == "__main__":
    main()
