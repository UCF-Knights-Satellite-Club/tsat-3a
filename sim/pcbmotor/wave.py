"""Wave-winding variant of the high-momentum payload stator, compared with the spiral (concentrated)
coils used so far.

Why: every built PCB motor comparison we have says wave or radial windings beat spiral/concentric
coils on copper loss at equal torque (Tokgoz et al. IET EPA 2020: same torque, 20% less phase
resistance; CIOMP 2015: 26% less power at 5000 rpm; CMU chose a wave winding for the same reason).
Papers are in ~/KSC/research.

Winding modelled (a full-pitch, two-layer "radial wave", Tokgoz's "radial" winding):
- Every layer carries N identical conductors, rotated copies of one template, so nothing on a layer
  crosses. A conductor runs radially across the magnet band [r_a, r_b] and has slanted end sections
  in [r_in, r_a] and [r_b, r_out] that each shift it by half a pole pitch. On the "top" layer of a
  pair the ends slant one way, on the "bottom" layer the other way, so a top conductor at angle beta
  joins (outer via) a bottom conductor at beta + pole pitch, which joins (inner via) the next top
  conductor at beta + 2 pole pitches: a wave. Forward and return waves use the same conductor shapes.
- Within each pole pitch the conductor positions split into three 60-electrical-degree bands
  (A, -C, B), alternating sign pole to pole: q = N / (3 * poles) slots per pole per phase,
  full pitch, winding factor ~0.955.
- "series": L1/L2 form one wave pair and L3/L4 another, all in series, so every conductor carries the
  phase current. No parallel paths, so no circulating current. Layout risk: with through vias only,
  the L1-L2 vias have to fall between L3/L4 traces and vice versa (stagger by half a slot and neck
  the trace locally); that is not modelled beyond a via-pitch check.
- "stitched" (CMU style): the radial sections of all four layers are joined by vias at both ends,
  so the layers are in parallel and the end connections are tangential arcs on one layer per phase.
  Current splits by layer conductance, and because the layers sit at different depths their
  back-EMFs differ, so current circulates between layers at speed. That loss is reported.
- Trace width either uniform, or "fill": as wide as the slot pitch allows at each radius
  (perpendicular to the trace) minus the copper space, i.e. Tokgoz's "unequal width" idea.

Model: the same magpylib Lorentz-force method and rotor (back iron as an image magnet) as em.py.
Every conductor is a rotated copy of its layer's template, so the template's torque-per-amp curve
against rotor angle is computed once per layer and shifted; tests/test_wave.py checks this against
a direct sum over every conductor. Eddy loss uses the same sigma*w^2/12*(dBz/dt)^2 model as
stacking.py, with the local trace width.

Run from sim/:  .venv/bin/python -m pcbmotor.wave
"""
from __future__ import annotations

import json
import math
import pathlib
from dataclasses import dataclass, asdict, replace

import numpy as np

from .geometry import RotorDesign, StatorDesign, STACKUPS, OZ_TO_M
from .em import rotor_magnets, analyse, RHO_CU_20C, mean_dbz_dtheta2, rotor_bz
from .designs import drive_metrics
from .stacking import coil_curves

OUT = pathlib.Path(__file__).resolve().parents[1] / "results"
SIGMA_CU = 1 / RHO_CU_20C
VIA_DRILL_MM = 0.3
VIA_PLATING_M = 25e-6
VIA_PAD_MM = 0.6
BOARD_T_MM = 1.6
LINK_ALLOWANCE = 1.05        # lap jogs between wave passes and phase leads (spiral model uses 1.10)
BOARD_AREA_M2 = 0.090 * 0.090


@dataclass
class WaveDesign:
    name: str
    n_per_layer: int               # conductor positions per layer; multiple of 3 * poles
    r_in_mm: float = 21.0          # inner via radius (envelope)
    r_a_mm: float = 25.0           # radial (active) section start
    r_b_mm: float = 37.0           # radial section end
    r_out_mm: float = 41.0         # outer via radius (envelope)
    trace_mm: float | None = 1.2   # None = fill the slot pitch at every radius
    cap_mm: float | None = None    # fill only: never wider than this
    strands: int = 1               # split each conductor into this many parallel strands (gap = space)
    slit: bool = False             # strands only in the radial section (merged in the end turns)
    swap: bool = True              # full-length strands: reverse the lane order at every layer via
    space_mm: float = 0.15
    min_trace_mm: float = 0.15
    stackup: str = "jlc4_2oz_out"
    connection: str = "series"     # "series" or "stitched"
    end_arc_band_mm: float = 3.0   # stitched only: radial depth of the end-arc band at each end

    @property
    def stack(self):
        return STACKUPS[self.stackup]


# ---------------------------------------------------------------- geometry
def template(d: WaveDesign, n_poles: int, sigma: int, ds_mm=0.25):
    """Points (mm, polar) along one conductor centred on angle 0, inner via to outer via.
    sigma=+1 top-of-pair shape, -1 bottom. Returns r, theta, local width w (mm) per point."""
    tau = 2 * math.pi / n_poles
    pieces = []
    if d.connection == "stitched":
        segs = [(d.r_a_mm, d.r_b_mm, lambda r: 0.0 * r, 0.0)]
    else:
        segs = [(d.r_in_mm, d.r_a_mm, lambda r: -sigma * tau / 2 * (d.r_a_mm - r) / (d.r_a_mm - d.r_in_mm),
                 tau / 2 / (d.r_a_mm - d.r_in_mm)),
                (d.r_a_mm, d.r_b_mm, lambda r: 0.0 * r, 0.0),
                (d.r_b_mm, d.r_out_mm, lambda r: sigma * tau / 2 * (r - d.r_b_mm) / (d.r_out_mm - d.r_b_mm),
                 tau / 2 / (d.r_out_mm - d.r_b_mm))]
    for r0, r1, th, slope in segs:
        # number of samples from the slanted length
        L = (r1 - r0) * math.sqrt(1 + (r1 * slope) ** 2)
        n = max(2, int(math.ceil(L / ds_mm)) + 1)
        r = np.linspace(r0, r1, n)
        pieces.append((r, th(r), np.full(n, slope)))
    r = np.concatenate([p[0][:-1] if i < len(pieces) - 1 else p[0] for i, p in enumerate(pieces)])
    th = np.concatenate([p[1][:-1] if i < len(pieces) - 1 else p[1] for i, p in enumerate(pieces)])
    sl = np.concatenate([p[2][:-1] if i < len(pieces) - 1 else p[2] for i, p in enumerate(pieces)])
    delta = 2 * math.pi / d.n_per_layer
    cos_psi = 1 / np.sqrt(1 + (r * sl) ** 2)
    avail = r * delta * cos_psi - d.space_mm       # trace width that fits between neighbours
    w = avail if d.trace_mm is None else np.full_like(r, d.trace_mm)
    if d.trace_mm is None and d.cap_mm is not None:
        w = np.minimum(w, d.cap_mm)
    return r, th, w, avail


def copper_widths(d: WaveDesign, w, r):
    """Copper width carrying current, and the width of one strand (what sets the eddy loss).
    With slit=True the conductor is split only across the radial section."""
    split = (r >= d.r_a_mm - 1e-9) & (r < d.r_b_mm - 1e-9) if d.slit else np.ones_like(r, bool)
    w_cu = np.where(split, w - (d.strands - 1) * d.space_mm, w)
    return w_cu, np.where(split, w_cu / d.strands, w)


def slit_circulating(d: WaveDesign, rot: RotorDesign, z, oz, thetas):
    """Slit conductor: strands side by side across the radial section, joined at both ends, so a
    strand further round sees its EMF slightly earlier and current circulates between them.
    Returns the loss coefficient of ONE conductor [W/(rad/s)^2]."""
    if not d.slit or d.strands < 2:
        return 0.0
    r = np.linspace(d.r_a_mm, d.r_b_mm, max(2, int((d.r_b_mm - d.r_a_mm) / 0.25) + 1))
    _, _, w, _ = template(replace(d, r_in_mm=d.r_a_mm - 1, r_out_mm=d.r_b_mm + 1), rot.n_poles, 1)
    delta = 2 * math.pi / d.n_per_layer
    w = (r * delta - d.space_mm) if d.trace_mm is None else np.full_like(r, d.trace_mm)
    if d.cap_mm is not None and d.trace_mm is None:
        w = np.minimum(w, d.cap_mm)
    k = d.strands
    w_cu = w - (k - 1) * d.space_mm
    w_s = w_cu / k
    E = []
    for j in range(k):
        x = -w / 2 + w_s / 2 + j * (w_s + d.space_mm)        # strand centre, tangential mm
        mids, dls = _segments(r, x / r, z)
        e = np.empty(len(thetas))
        for i, a in enumerate(thetas):
            F = np.cross(dls, rotor_magnets(rot, a).getB(mids))
            e[i] = -np.sum(mids[:, 0] * F[:, 1] - mids[:, 1] * F[:, 0])
        E.append(e)
    E = np.array(E)
    ds = np.diff(r) / 1e3
    wsm = (w_s[:-1] + w_s[1:]) / 2 / 1e3
    g = 1 / float(np.sum(RHO_CU_20C * ds / (wsm * oz * OZ_TO_M)))     # one strand, equal for all
    return float(np.mean((g * (E - E.mean(0)) ** 2).sum(0)))


def positions(d: WaveDesign, n_poles: int):
    """Angle of each conductor's radial section, its phase (0=A,1=B,2=C) and sign (+ = current
    flowing outward)."""
    assert d.n_per_layer % (3 * n_poles) == 0, "n_per_layer must be a multiple of 3 * poles"
    tau = 2 * math.pi / n_poles
    phi = (np.arange(d.n_per_layer) + 0.5) * 2 * math.pi / d.n_per_layer
    band = np.floor(phi / (tau / 3) + 1e-9).astype(int)
    pole, g = band // 3, band % 3
    table = [(0, 1), (2, -1), (1, 1)]          # A+, C-, B+ within a pole
    ph = np.array([table[k][0] for k in g])
    sgn = np.array([table[k][1] for k in g]) * np.where(pole % 2 == 0, 1, -1)
    return phi, ph, sgn


def layer_shapes(d: WaveDesign):
    """sigma (end-slant direction) of each layer: pairs (L1,L2), (L3,L4)."""
    n = len(d.stack["z_mm"])
    return [1 if k % 2 == 0 else -1 for k in range(n)]


# ---------------------------------------------------------------- field / torque
def _segments(r, th, z_mm):
    p = np.c_[r * np.cos(th), r * np.sin(th), np.full(len(r), -z_mm)] / 1e3
    return (p[:-1] + p[1:]) / 2, np.diff(p, axis=0)


def template_curves(d: WaveDesign, rot: RotorDesign, n_theta=144):
    """Per layer: torque on the rotor per amp (outward current) of the template conductor against
    rotor angle over one electrical period, and its eddy coefficient c [W/(rad/s)^2]."""
    period = 2 * (2 * math.pi / rot.n_poles)
    thetas = np.linspace(0, period, n_theta, endpoint=False)
    out = []
    for z, oz, sigma in zip(d.stack["z_mm"], d.stack["oz"], layer_shapes(d)):
        r, th, w, _ = template(d, rot.n_poles, sigma)
        mids, dls = _segments(r, th, z)
        w_cu, w_s = copper_widths(d, w, r)
        wm = (w_cu[:-1] + w_cu[1:]) / 2 / 1e3
        ws = (w_s[:-1] + w_s[1:]) / 2 / 1e3
        tq = np.empty(n_theta)
        Bz = np.empty((n_theta, len(mids)))
        for i, a in enumerate(thetas):
            B = rotor_magnets(rot, a).getB(mids)
            F = np.cross(dls, B)
            tq[i] = -np.sum(mids[:, 0] * F[:, 1] - mids[:, 1] * F[:, 0])
            Bz[i] = B[:, 2]
        bz, key = rotor_bz(rotor_magnets, rot)
        vol = wm * oz * OZ_TO_M * np.linalg.norm(dls, axis=1)
        c_eddy = float(np.sum(SIGMA_CU * ws ** 2 / 12 * mean_dbz_dtheta2(bz, key, rot.n_poles, mids) * vol))
        out.append(dict(z=z, oz=oz, sigma=sigma, tq=tq, c_eddy=c_eddy,
                        c_slit=slit_circulating(d, rot, z, oz, thetas[::4])))
    return thetas, out


def _shift(thetas, curve, x):
    P = len(thetas) * (thetas[1] - thetas[0])
    return np.interp(x % P, np.r_[thetas, P], np.r_[curve, curve[0]])


def phase_curves(d: WaveDesign, rot: RotorDesign, thetas, layers, th_eval, weights):
    """k_ph(theta) [Nm/A] per phase. weights[l] = share of the phase current in layer l
    (1 for series; conductance share for stitched)."""
    phi, ph, sgn = positions(d, rot.n_poles)
    kph = np.zeros((3, len(th_eval)))
    for lay, wgt in zip(layers, weights):
        for p_, k, s in zip(phi, ph, sgn):
            kph[k] += wgt * s * _shift(thetas, lay["tq"], th_eval - p_)
    return kph


def foc(kph, th, pp):
    best = (-1e9, None)
    for order in (1, -1):
        for delta in np.linspace(0, 2 * math.pi, 180, endpoint=False):
            i = np.array([np.cos(pp * th + delta - order * k * 2 * math.pi / 3) for k in range(3)])
            t = np.sum(kph * i, axis=0)
            if t.mean() > best[0]:
                best = (t.mean(), t)
    mean, t = best
    return mean, (t.max() - t.min()) / abs(mean)


def strand_circulating(d: WaveDesign, rot: RotorDesign, thetas):
    """Full-length strands (strands > 1, not slit): every strand runs the whole phase in series and
    the strands only join at the phase terminals. Each strand's EMF is summed over every conductor
    of the phase at the strand's true lane position on each layer. With swap=True the lane order
    reverses at every via (L2 and L4 carry the reversed order, as in bundle.py and the parallel
    connection of Francois & Dehez 2022); with swap=False each strand keeps its lane. Returns the loss
    coefficient of all three phases [W/(rad/s)^2]."""
    if d.slit or d.strands < 2:
        return 0.0
    k = d.strands
    delta = 2 * math.pi / d.n_per_layer
    phi, ph, sgn = positions(d, rot.n_poles)
    E = np.zeros((k, len(thetas)))
    R = np.zeros(k)
    for li, (z, oz, sigma) in enumerate(zip(d.stack["z_mm"], d.stack["oz"], layer_shapes(d))):
        r, th, w, avail = template(d, rot.n_poles, sigma)
        cos_psi = np.clip((avail + d.space_mm) / (r * delta), 1e-3, 1.0)
        w_cu, w_s = copper_widths(d, w, r)
        for j in range(k):
            lane = (k - 1 - j) if (d.swap and li % 2 == 1) else j
            x = -w / 2 + w_s / 2 + lane * (w_s + d.space_mm)          # perpendicular offset, mm
            ths = th + x / (r * cos_psi)
            mids, dls = _segments(r, ths, z)
            curve = np.empty(len(thetas))
            for i, a in enumerate(thetas):
                F = np.cross(dls, rotor_magnets(rot, a).getB(mids))
                curve[i] = -np.sum(mids[:, 0] * F[:, 1] - mids[:, 1] * F[:, 0])
            for p_, kk, s_ in zip(phi, ph, sgn):
                if kk == 0:
                    E[j] += s_ * _shift(thetas, curve, thetas - p_)
            xs, ys = r * np.cos(ths), r * np.sin(ths)
            ds = np.hypot(np.diff(xs), np.diff(ys)) / 1e3
            wsm = (w_s[:-1] + w_s[1:]) / 2 / 1e3
            R[j] += (d.n_per_layer // 3) * float(np.sum(RHO_CU_20C * ds / (wsm * oz * OZ_TO_M)))
    g = 1 / R
    V = (g[:, None] * E).sum(0) / g.sum()
    return 3 * float(np.mean((g[:, None] * (E - V) ** 2).sum(0)))


# ---------------------------------------------------------------- resistance
def _cond_resistance(d: WaveDesign, n_poles, sigma, oz):
    r, th, w, _ = template(d, n_poles, sigma, ds_mm=0.1)
    w, _ = copper_widths(d, w, r)
    x, y = r * np.cos(th), r * np.sin(th)
    ds = np.hypot(np.diff(x), np.diff(y)) / 1e3
    wm = (w[:-1] + w[1:]) / 2 / 1e3
    return float(np.sum(RHO_CU_20C * ds / (wm * oz * OZ_TO_M))), float(np.sum(ds * wm * oz * OZ_TO_M))


def via_resistance():
    a = math.pi * (VIA_DRILL_MM / 1e3) * VIA_PLATING_M
    return RHO_CU_20C * BOARD_T_MM / 1e3 / a


def phase_resistance(d: WaveDesign, n_poles):
    """Phase resistance at 20 C (star), and copper volume of the whole stator [m^3]."""
    per_phase = d.n_per_layer // 3
    vol = 0.0
    if d.connection == "series":
        r = 0.0
        for oz, sigma in zip(d.stack["oz"], layer_shapes(d)):
            rc, v = _cond_resistance(d, n_poles, sigma, oz)
            r += per_phase * rc
            vol += d.n_per_layer * v
        r += per_phase * len(d.stack["oz"]) * via_resistance()     # one via per conductor
        return r * LINK_ALLOWANCE, vol
    # stitched: radial sections in parallel across layers, then end arcs on one layer each
    g = 0.0
    for oz in d.stack["oz"]:
        rc, v = _cond_resistance(d, n_poles, 1, oz)
        g += 1 / rc
        vol += d.n_per_layer * v
    r_rad = 1 / g
    # end arcs: each conductor needs one arc of one pole pitch, alternately at r_a and r_b. A pole's
    # bundle of q conductors nests q arcs in a band end_arc_band_mm deep; the phase's arcs use one
    # 2 oz outer layer (best case) or one 1 oz inner layer.
    q = d.n_per_layer // (3 * n_poles)
    w_arc = d.end_arc_band_mm / q - d.space_mm
    tau = 2 * math.pi / n_poles
    oz_arc = 1.5      # average of A/B/C on 2, 1, 2 oz layers would be ~1.67; take 1.5
    r_in_arc = d.r_a_mm - d.end_arc_band_mm / 2
    r_out_arc = d.r_b_mm + d.end_arc_band_mm / 2
    r_arcs = per_phase / 2 * (RHO_CU_20C * (r_in_arc + r_out_arc) / 1e3 * tau / (w_arc / 1e3 * oz_arc * OZ_TO_M))
    vol += 3 * per_phase / 2 * ((r_in_arc + r_out_arc) / 1e3 * tau * w_arc / 1e3 * oz_arc * OZ_TO_M)
    return (per_phase * r_rad + r_arcs) * LINK_ALLOWANCE, vol


# ---------------------------------------------------------------- analysis
def analyse_wave(d: WaveDesign, rot: RotorDesign, n_theta=144, v_bus=None):
    thetas, layers = template_curves(d, rot, n_theta)
    pp = rot.n_poles // 2
    th = np.linspace(0, 2 * math.pi / pp, 240, endpoint=False)
    R, vol = phase_resistance(d, rot.n_poles)
    circ_c = 0.0
    if d.connection == "series":
        weights = [1.0] * len(layers)
    else:
        G = np.array(d.stack["oz"], dtype=float)          # same width and length: G ~ copper weight
        weights = list(G / G.sum())
        # circulating current between stitched layers: per conductor, node voltage is the
        # conductance-weighted mean of the layer EMFs; loss = sum G_l (e_l - V)^2, averaged over angle.
        # Per layer, the EMF per rad/s of the template is its torque-per-amp curve.
        rc = np.array([_cond_resistance(d, rot.n_poles, 1, oz)[0] for oz in d.stack["oz"]])
        g = 1 / rc
        E = np.array([lay["tq"] for lay in layers])       # layers x angle
        V = (g[:, None] * E).sum(0) / g.sum()
        circ_c = d.n_per_layer * float(np.mean((g[:, None] * (E - V) ** 2).sum(0)))   # W/(rad/s)^2
    kph = phase_curves(d, rot, thetas, layers, th, weights)
    kt, ripple = foc(kph, th, pp)
    c_eddy = d.n_per_layer * sum(l["c_eddy"] for l in layers)
    circ_c += d.n_per_layer * sum(l["c_slit"] for l in layers)
    circ_c += strand_circulating(d, rot, thetas)
    J, m_rot = rot.inertia_kgm2()
    kw = dict(v_bus=v_bus) if v_bus else {}
    m = drive_metrics(kt, R, float("nan"), kt * 2 / 3, J, BOARD_AREA_M2, 1.0, **kw)
    w_max = m["usable_max_rpm"] * 2 * math.pi / 60
    w2k = 2000 * 2 * math.pi / 60
    # geometry checks
    widths = [template(d, rot.n_poles, s)[3] for s in (1, -1)]
    min_avail = float(min(a.min() for a in widths))
    via_pitch = 2 * math.pi * d.r_in_mm / d.n_per_layer
    min_w = float(min(copper_widths(d, template(d, rot.n_poles, s)[2], template(d, rot.n_poles, s)[0])[1].min() for s in (1, -1)))
    max_w = float(max(template(d, rot.n_poles, s)[2].max() for s in (1, -1)))
    ok = (min_avail >= max(d.min_trace_mm, d.trace_mm or 0) - 1e-9
          and via_pitch >= (1 if d.slit else d.strands) * (VIA_PAD_MM + d.space_mm) and min_w >= d.min_trace_mm - 1e-9)
    return dict(name=d.name, kind="wave", connection=d.connection, n_per_layer=d.n_per_layer,
                q=d.n_per_layer // (3 * rot.n_poles), envelope_mm=[d.r_in_mm, d.r_out_mm],
                active_mm=[d.r_a_mm, d.r_b_mm], trace="fill" if d.trace_mm is None else d.trace_mm,
                cap_mm=d.cap_mm, strands=d.strands, slit=d.slit, swap=d.swap,
                strand_min_mm=min_w, trace_max_mm=max_w, via_pitch_mm=via_pitch, layout_ok=bool(ok),
                Kt_mNm_per_A=kt * 1e3, R_phase_ohm=R, Km_mNm_per_sqrtW=m["Km_mNm_per_sqrtW"],
                ripple_pct=ripple * 100, noload_rpm=m["noload_rpm"], usable_max_rpm=m["usable_max_rpm"],
                H_max_mNms=m["H_max_mNms"], tau_at_1A_mNm=m["tau_stall_mNm"],
                copper_g=vol * 8960 * 1e3,
                eddy_W_at_2000rpm=c_eddy * w2k ** 2, eddy_W_at_max_speed=c_eddy * w_max ** 2,
                circulating_W_at_2000rpm=circ_c * w2k ** 2, circulating_W_at_max_speed=circ_c * w_max ** 2,
                P_cu_at_15mNm_W=(15 / m["Km_mNm_per_sqrtW"]) ** 2,
                loss_15mNm_2000rpm_W=(15 / m["Km_mNm_per_sqrtW"]) ** 2 + (c_eddy + circ_c) * w2k ** 2)


def spiral_baseline():
    """The recommended high-momentum spiral design, through em.analyse and the stacking.py eddy model."""
    d = StatorDesign("payload_jlc4_2oz_out_6c8p_w1.2_h6.35", stackup="jlc4_2oz_out", n_coils=6,
                     r_in_mm=21.0, r_out_mm=41.0, trace_mm=1.2, space_mm=0.15, board_od_mm=90.0)
    rot = hm_rotor()
    res, _ = analyse(d, rot)
    _, _, c_eddy, _ = coil_curves(d, rot, dual=False)
    m = drive_metrics(res.Kt_Nm_per_A, res.R_phase_ohm, res.L_phase_uH / 1e6, res.Ke_phase_V_per_rad_s,
                      res.rotor_inertia_kgm2, BOARD_AREA_M2, 1.0)
    w_max = m["usable_max_rpm"] * 2 * math.pi / 60
    w2k = 2000 * 2 * math.pi / 60
    return dict(name=d.name, kind="spiral", connection="series", n_per_layer=None, q=0.25,
                envelope_mm=[21.0, 41.0], active_mm=[21.0, 41.0], trace=1.2, strand_min_mm=1.2, slit=False, swap=False,
                trace_max_mm=1.2, cap_mm=None, strands=1, via_pitch_mm=None, layout_ok=True,
                Kt_mNm_per_A=res.Kt_Nm_per_A * 1e3, R_phase_ohm=res.R_phase_ohm,
                Km_mNm_per_sqrtW=m["Km_mNm_per_sqrtW"], ripple_pct=res.torque_ripple_pct,
                noload_rpm=m["noload_rpm"], usable_max_rpm=m["usable_max_rpm"], H_max_mNms=m["H_max_mNms"],
                tau_at_1A_mNm=m["tau_stall_mNm"], copper_g=res.copper_mass_g,
                eddy_W_at_2000rpm=c_eddy * w2k ** 2, eddy_W_at_max_speed=c_eddy * w_max ** 2,
                circulating_W_at_2000rpm=0.0, circulating_W_at_max_speed=0.0,
                P_cu_at_15mNm_W=(15 / m["Km_mNm_per_sqrtW"]) ** 2,
                loss_15mNm_2000rpm_W=(15 / m["Km_mNm_per_sqrtW"]) ** 2 + c_eddy * w2k ** 2)


def hm_rotor():
    return RotorDesign(n_poles=8, magnet_shape="block", block_len_mm=19.05, block_w_mm=12.7, magnet_h_mm=6.35,
                       magnet_r_mm=31.0, airgap_mm=0.6, back_iron=True, back_iron_t_mm=1.5, disc_od_mm=84.0,
                       disc_t_mm=1.5)


# ---------------------------------------------------------------- study
def sweep():
    """Candidate wave stators for the high-momentum rotor (magnets cover r = 21.5-40.5 mm).
    Widths: full fill, fill capped at 1.2 mm (same eddy exposure as the spiral), and full fill with
    the radial section slit into 2-4 strands that merge again in the end turns (strands all the way
    through don't fit: the slanted end turns are too narrow). The circulating current between slit
    strands is included."""
    widths = [("fill", dict(trace_mm=None)), ("cap1.2", dict(trace_mm=None, cap_mm=1.2)),
              ("fill_slit2", dict(trace_mm=None, strands=2, slit=True)),
              ("fill_slit3", dict(trace_mm=None, strands=3, slit=True)),
              ("fill_slit4", dict(trace_mm=None, strands=4, slit=True))]
    envs = [("env21-41_e4", (21.0, 25.0, 37.0, 41.0)), ("env21-41_e5", (21.0, 26.0, 36.0, 41.0)),
            ("env15-44_act21-41", (15.0, 21.0, 41.0, 44.0)), ("env15-44_act22-40", (15.0, 22.0, 40.0, 44.0))]
    out = []
    for n in (48, 72):
        for env, (ri, ra, rb, ro) in envs:
            for wn, kw in widths:
                out.append(WaveDesign(f"wave_{env}_n{n}_{wn}", n_per_layer=n, r_in_mm=ri, r_a_mm=ra,
                                      r_b_mm=rb, r_out_mm=ro, **kw))
    # width cap trade-off on the best envelopes
    for n in (48, 72):
        for env, (ri, ra, rb, ro) in envs[1:2] + envs[3:]:
            for cap in (0.8, 1.6, 2.0, 2.5):
                out.append(WaveDesign(f"wave_{env}_n{n}_cap{cap:g}", n_per_layer=n, r_in_mm=ri, r_a_mm=ra,
                                      r_b_mm=rb, r_out_mm=ro, trace_mm=None, cap_mm=cap))
    # full-length strands, lane order swapped at every via (or not), on the recommended envelope
    for n in (48,):
        for env, (ri, ra, rb, ro) in envs[1:2] + envs[3:]:
            for cap in (1.6, 2.0, 2.5, None):
                for k in (2, 3):
                    for swap in (True, False):
                        tag = "fill" if cap is None else f"cap{cap:g}"
                        out.append(WaveDesign(f"wave_{env}_n{n}_{tag}_s{k}{'swap' if swap else 'fixed'}",
                                              n_per_layer=n, r_in_mm=ri, r_a_mm=ra, r_b_mm=rb, r_out_mm=ro,
                                              trace_mm=None, cap_mm=cap, strands=k, swap=swap))
    # CMU-style stitched layers (parallel layers, tangential end arcs)
    for n in (48, 96):
        out.append(WaveDesign(f"wave_stitched_act21-41_n{n}_fill", n_per_layer=n, r_in_mm=18.0, r_a_mm=21.0,
                              r_b_mm=41.0, r_out_mm=44.0, trace_mm=None, connection="stitched"))
    return out


def table(rows):
    cols = [("name", "{}"), ("layout_ok", "{}"), ("Kt_mNm_per_A", "{:.1f}"), ("R_phase_ohm", "{:.2f}"),
            ("Km_mNm_per_sqrtW", "{:.1f}"), ("ripple_pct", "{:.1f}"), ("strands", "{}"), ("strand_min_mm", "{:.2f}"),
            ("trace_max_mm", "{:.2f}"), ("noload_rpm", "{:.0f}"), ("H_max_mNms", "{:.0f}"),
            ("P_cu_at_15mNm_W", "{:.2f}"), ("eddy_W_at_2000rpm", "{:.2f}"), ("eddy_W_at_max_speed", "{:.2f}"),
            ("circulating_W_at_max_speed", "{:.2f}"), ("loss_15mNm_2000rpm_W", "{:.2f}"), ("copper_g", "{:.1f}")]
    lines = ["| " + " | ".join(c for c, _ in cols) + " |", "|" + "---|" * len(cols)]
    for r in rows:
        lines.append("| " + " | ".join(f.format(r[c]) if r[c] is not None else "-" for c, f in cols) + " |")
    return "\n".join(lines)


def main():
    rot = hm_rotor()
    base = spiral_baseline()
    print("spiral", {k: round(v, 3) if isinstance(v, float) else v for k, v in base.items()}, flush=True)
    rows = [base]
    for d in sweep():
        res = analyse_wave(d, rot)
        rows.append(res)
        print(f"{d.name:42s} ok={res['layout_ok']!s:5s} Kt={res['Kt_mNm_per_A']:5.1f} R={res['R_phase_ohm']:5.2f} "
              f"Km={res['Km_mNm_per_sqrtW']:5.1f} rip={res['ripple_pct']:4.1f}% s={res['strands']} w={res['strand_min_mm']:.2f}-"
              f"{res['trace_max_mm']:.2f} eddy2k={res['eddy_W_at_2000rpm']:.2f} loss={res['loss_15mNm_2000rpm_W']:.2f} circ={res['circulating_W_at_max_speed']:.2f}",
              flush=True)
    (OUT / "pcbmotor_wave.json").write_text(json.dumps(rows, indent=2, default=float))
    (OUT / "pcbmotor_wave_table.md").write_text(table(rows) + "\n")
    plot_layout(RECOMMENDED, rot, OUT / "pcbmotor_wave_layout.png")



def plot_layout(d: WaveDesign, rot: RotorDesign, path):
    """One wave layer pair (L1 solid, L2 dashed) coloured by phase, with the magnet outlines."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(1, 2, figsize=(11, 5.5))
    phi, ph, sgn = positions(d, rot.n_poles)
    cols = ["tab:red", "tab:green", "tab:blue"]
    for ax, zoom in zip(axes, (False, True)):
        for sigma, ls, lw in ((1, "-", 1.4), (-1, "--", 0.9)):
            r, th, w, _ = template(d, rot.n_poles, sigma)
            for p_, k in zip(phi, ph):
                ax.plot(r * np.cos(th + p_), r * np.sin(th + p_), ls, color=cols[k], lw=lw)
        for kk in range(rot.n_poles):
            a = kk * 2 * math.pi / rot.n_poles
            L, W = rot.block_len_mm, rot.block_w_mm
            box = np.array([[-L / 2, -W / 2], [L / 2, -W / 2], [L / 2, W / 2], [-L / 2, W / 2], [-L / 2, -W / 2]])
            box[:, 0] += rot.magnet_r_mm
            rr = np.c_[box[:, 0] * math.cos(a) - box[:, 1] * math.sin(a), box[:, 0] * math.sin(a) + box[:, 1] * math.cos(a)]
            ax.plot(rr[:, 0], rr[:, 1], color="0.6", lw=0.8)
        for rad in (d.r_in_mm, d.r_out_mm):
            t = np.linspace(0, 2 * math.pi, 400)
            ax.plot(rad * np.cos(t), rad * np.sin(t), color="0.85", lw=0.6)
        ax.set_aspect("equal")
        ax.set_xlabel("mm")
        if zoom:
            ax.set_xlim(10, 46)
            ax.set_ylim(-4, 26)
            ax.set_title("Zoom: L1 solid, L2 dashed (A red, B green, C blue)")
        else:
            ax.set_title(f"{d.name}\none layer pair, magnets in grey", fontsize=10)
    fig.tight_layout()
    fig.savefig(path, dpi=130)
    plt.close(fig)


RECOMMENDED = WaveDesign("wave_env15-44_act22-40_n48_cap1.6_s2swap", n_per_layer=48, r_in_mm=15.0, r_a_mm=22.0,
                         r_b_mm=40.0, r_out_mm=44.0, trace_mm=None, cap_mm=1.6, strands=2, swap=True)


if __name__ == "__main__":
    main()
