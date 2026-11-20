"""Bus-voltage sweep for the high-momentum wave stator: what changes when the supply voltage changes,
with the winding kept and with the winding re-picked to suit the voltage.

Part 1, same winding (wave.RECOMMENDED) on 2S-8S LiPo: top speed, momentum, stored energy, magnet
bond shear, eddy drag at top speed, torque left at 3500 rpm, PWM ripple current.

Part 2, rewound: the turn count is the knob that matches a winding to a voltage. Options are
n = 24..120 conductors per layer (n must be a multiple of 3 * poles = 24; above 120 the inner vias
no longer fit at r = 15 mm) and 1 or 2 parallel paths per phase. Two paths put L1+L4 in parallel
with L2+L3 (each is a valid top/bottom wave pair and both have the same mean depth, so their
back-EMFs nearly match); the circulating current from what mismatch remains is included. Each turn
count gets its own best trace cap and strand count (lowest loss at 15 mN m and 2000 rpm, layout
checks passed). At each voltage the chosen winding is the one with the most turns that still reaches
the 11.1 V design's no-load speed (within SPEED_MARGIN).

Inductance (new for the wave model): Neumann formula over the phase's conductors. Every conductor
on a layer is a rotated copy of that layer's template, so the double sum reduces to one
template-to-template integral per layer pair and angular offset. Wide flat traces are treated as
filaments a geometric mean distance apart: hypot(layer spacing, 0.2235 * (width + thickness)).
Vias and phase leads are ignored.

PWM ripple: centre-aligned PWM with min-max (SVPWM-equivalent) injection is switched against the
carrier over one period, the phase-to-neutral voltage taken from the three leg voltages, and its
deviation from the period average integrated through the synchronous inductance Ls = L_self - M_AB.
The RMS ripple is averaged over the electrical angle at the operating point's voltage (0 and 2000
rpm at 15 mN m) and, as a worst case, at the modulation depth that maximises it. The ripple's copper
loss is 3 * (R + R_EXT) * I_rms^2 (DC resistance; at 20-100 kHz the 35-70 um copper is thinner than
the skin depth, but proximity effect in wide traces is not modelled). drive_metrics' V / (8 L f)
half-bridge bound overstates the ripple of a star-connected three-phase load at low modulation.

ASSUMED, outside the motor: R_EXT_OHM in each phase for driver MOSFETs, harness and connector, kept
the same at every voltage (in practice higher-voltage MOSFETs of the same size have more on
resistance, so this flatters the high-voltage cases a little). Driver switching loss, battery
resistance and the ripple's skin/proximity effect are not modelled.

Run from sim/:  .venv/bin/python -m pcbmotor.bus_voltage
Writes results/pcbmotor_bus_voltage.json and results/pcbmotor_bus_voltage.md.
"""
from __future__ import annotations

import json
import math
import pathlib
from dataclasses import replace

import numpy as np

from .geometry import OZ_TO_M
from .em import MU0, _neumann
from .designs import drive_metrics, PWM_FREQS, RPM_CAP
from .wave import (WaveDesign, RECOMMENDED, BOARD_AREA_M2, LINK_ALLOWANCE, hm_rotor, analyse_wave, template,
                   template_curves, phase_curves, foc, positions, layer_shapes, copper_widths, _cond_resistance,
                   via_resistance)

OUT = pathlib.Path(__file__).resolve().parents[1] / "results"
VOLTAGES = (7.4, 11.1, 14.8, 18.5, 22.2, 29.6)        # 2S .. 8S LiPo nominal
V_REF = 11.1
N_OPTIONS = (24, 48, 72, 96, 120)
CAPS = (0.6, 0.8, 1.0, 1.2, 1.6, 2.0, 2.5, 3.0, 4.0, None)
STRANDS = (1, 2, 3, 4, 5, 6)
PATHS = {1: [[0, 1, 2, 3]], 2: [[0, 3], [1, 2]]}       # layer groups in parallel
R_EXT_OHM = 0.25            # ASSUMED driver + harness resistance per phase
I_LIM_A = 1.0               # same current limit as the other studies
TAU_OP_MNM = 15.0
RPM_OP = 2000.0
RPM_CHECK = 3500.0
SPEED_MARGIN = 0.95
MAGNET_DENSITY = 7500.0
EPOXY_SHEAR_MPA = 10.0      # ASSUMED, as in mechanical.py
STEEL_DENSITY, POISSON = 7850.0, 0.29


def rpm(w):
    return w * 60 / (2 * math.pi)


def rad_s(n_rpm):
    return n_rpm * 2 * math.pi / 60


# ---------------------------------------------------------------- inductance
def _rotate(p, a):
    c, s = math.cos(a), math.sin(a)
    return np.c_[c * p[:, 0] - s * p[:, 1], s * p[:, 0] + c * p[:, 1], p[:, 2]]


def _offset_coeffs(d: WaveDesign, n_poles, pa, pb):
    """c[k] = sum of s_i s_j over conductors i of phase pa and j of phase pb with j - i = k (mod n)."""
    _, ph, sgn = positions(d, n_poles)
    ia, ib = np.nonzero(ph == pa)[0], np.nonzero(ph == pb)[0]
    c = np.zeros(d.n_per_layer)
    np.add.at(c, (ib[None, :] - ia[:, None]) % d.n_per_layer, sgn[ia][:, None] * sgn[ib][None, :])
    return c


def inductance_matrices(d: WaveDesign, n_poles, ds_mm=0.25):
    """Partial inductances between layers for one phase [H]: L[l, m] (phase A on layer l with phase A
    on layer m) and M[l, m] (phase A on l with phase B on m). Series phase self inductance = L.sum()."""
    nl = len(d.stack["z_mm"])
    delta = 2 * math.pi / d.n_per_layer
    tpl, wid = [], []
    for z, sigma in zip(d.stack["z_mm"], layer_shapes(d)):
        r, th, w, _ = template(d, n_poles, sigma, ds_mm)
        tpl.append(np.c_[r * np.cos(th), r * np.sin(th), np.full(len(r), -z)] / 1e3)
        wid.append(float(np.mean(w)))
    c_aa, c_ab = _offset_coeffs(d, n_poles, 0, 0), _offset_coeffs(d, n_poles, 0, 1)
    ks = np.nonzero((c_aa != 0) | (c_ab != 0))[0]
    L, M = np.zeros((nl, nl)), np.zeros((nl, nl))
    for l in range(nl):
        for m in range(nl):
            t = (d.stack["oz"][l] + d.stack["oz"][m]) / 2 * OZ_TO_M
            w = (wid[l] + wid[m]) / 2 / 1e3
            dz = math.hypot(abs(d.stack["z_mm"][l] - d.stack["z_mm"][m]) / 1e3, 0.2235 * (w + t))
            f = np.zeros(d.n_per_layer)
            for k in ks:
                other = _rotate(tpl[m], k * delta)
                other[:, 2] = tpl[l][0, 2] - dz
                f[k] = _neumann(tpl[l], other)
            L[l, m], M[l, m] = c_aa @ f, c_ab @ f
    return L, M


# ---------------------------------------------------------------- winding with parallel paths
def path_analysis(d: WaveDesign, rot, paths: int, n_theta=72):
    """Kt [Nm/A peak], phase resistance [ohm] and inter-path circulating loss coefficient
    [W/(rad/s)^2, all phases] with the layers grouped into parallel paths. Current splits by path
    conductance; each path's EMF per rad/s is its torque-per-amp curve."""
    thetas, layers = template_curves(d, rot, n_theta)
    pp = rot.n_poles // 2
    th = np.linspace(0, 2 * math.pi / pp, 240, endpoint=False)
    per_phase = d.n_per_layer // 3
    R_l = [per_phase * (_cond_resistance(d, rot.n_poles, s, oz)[0] + via_resistance()) * LINK_ALLOWANCE
           for oz, s in zip(d.stack["oz"], layer_shapes(d))]
    k_l = [phase_curves(d, rot, thetas, [lay], th, [1.0]) for lay in layers]
    groups = PATHS[paths]
    g = np.array([1 / sum(R_l[l] for l in grp) for grp in groups])
    K = [sum(k_l[l] for l in grp) for grp in groups]
    kph = sum(gi / g.sum() * Ki for gi, Ki in zip(g, K))
    kt, ripple = foc(kph, th, pp)
    circ = 0.0
    for p in range(3):
        E = np.array([Ki[p] for Ki in K])
        V = (g[:, None] * E).sum(0) / g.sum()
        circ += float(np.mean((g[:, None] * (E - V) ** 2).sum(0)))
    return float(kt), float(1 / g.sum()), circ, float(ripple)


def winding(d: WaveDesign, rot, paths: int, base=None):
    """Everything voltage-independent about one winding."""
    base = base or analyse_wave(d, rot, n_theta=72)
    w2k = rad_s(2000)
    c_eddy = base["eddy_W_at_2000rpm"] / w2k ** 2
    c_strand = base["circulating_W_at_2000rpm"] / w2k ** 2
    if paths == 1:
        kt, R, c_path, rip = base["Kt_mNm_per_A"] / 1e3, base["R_phase_ohm"], 0.0, base["ripple_pct"] / 100
    else:
        kt, R, c_path, rip = path_analysis(d, rot, paths)
        # strands: each path's strands see half the EMF difference and half the resistance of the series
        # case, so their circulating loss is about half (both are tiny with swapped lanes)
        c_strand *= 0.5
    L, M = inductance_matrices(d, rot.n_poles)
    L_self, M_ab = L.sum() / paths ** 2, M.sum() / paths ** 2
    J, _ = rot.inertia_kgm2()
    _, _, w, _ = template(d, rot.n_poles, 1)
    _, w_s = copper_widths(d, w, template(d, rot.n_poles, 1)[0])
    return dict(name=f"n{d.n_per_layer}_{'fill' if d.cap_mm is None else f'cap{d.cap_mm:g}'}_s{d.strands}_a{paths}",
                n_per_layer=d.n_per_layer, paths=paths, cap_mm=d.cap_mm, strands=d.strands,
                series_conductors_per_phase=len(d.stack["oz"]) * d.n_per_layer // 3 // paths,
                strand_max_mm=float(w_s.max()), layout_ok=base["layout_ok"],
                Kt_mNm_per_A=kt * 1e3, Ke_phase_V_per_krpm=kt * 2 / 3 * rad_s(1000), R_phase_ohm=R,
                Km_mNm_per_sqrtW=kt / math.sqrt(1.5 * R) * 1e3, torque_ripple_pct=rip * 100,
                L_self_uH=L_self * 1e6, M_ab_uH=M_ab * 1e6, Ls_uH=(L_self - M_ab) * 1e6,
                elec_tau_us=(L_self - M_ab) / R * 1e6,
                c_eddy=c_eddy, c_circ=c_strand + c_path, c_path=c_path, copper_g=base["copper_g"], J_kgm2=J)


def at_voltage(wd, v, rot):
    """Drive envelope and losses of winding wd on bus voltage v."""
    kt, R, Ls = wd["Kt_mNm_per_A"] / 1e3, wd["R_phase_ohm"], wd["Ls_uH"] / 1e6
    ke = kt * 2 / 3
    m = drive_metrics(kt, R, Ls, ke, wd["J_kgm2"], BOARD_AREA_M2, I_LIM_A, v_bus=v)
    v_ph = v / math.sqrt(3)
    w_nl = v_ph / ke
    w_top = min(w_nl, rad_s(RPM_CAP))
    c = wd["c_eddy"] + wd["c_circ"]
    i_op = TAU_OP_MNM / 1e3 / kt
    w_op, w_chk = rad_s(RPM_OP), rad_s(RPM_CHECK)
    m_mag = rot.block_len_mm * rot.block_w_mm * rot.magnet_h_mm * 1e-9 * MAGNET_DENSITY
    foot = rot.block_len_mm * rot.block_w_mm * 1e-6
    pp = rot.n_poles // 2

    def v_need(w):                       # phase voltage peak at 15 mN m and speed w
        return math.hypot(ke * w + R * i_op, pp * w * Ls * i_op)

    rip = {}
    for f in PWM_FREQS:
        k = int(f / 1e3)
        for tag, w in (("0rpm", 0.0), ("2000rpm", w_op)):
            x = ripple_rms(v, Ls, f, v_need(w))
            rip[f"ripple_rms_A_{tag}_{k}kHz"] = x
            rip[f"ripple_loss_W_{tag}_{k}kHz"] = 3 * (R + R_EXT_OHM) * x ** 2
        x = max(ripple_rms(v, Ls, f, mi * v_ph) for mi in np.linspace(0, 1, 11))
        rip[f"ripple_rms_A_worst_{k}kHz"] = x
        rip[f"ripple_loss_W_worst_{k}kHz"] = 3 * (R + R_EXT_OHM) * x ** 2
        rip[f"ripple_bound_pp_A_{k}kHz"] = m[f"ripple_pp_A_{k}kHz"]
    p_cu, p_ext, p_drag = 1.5 * R * i_op ** 2, 1.5 * R_EXT_OHM * i_op ** 2, c * w_op ** 2
    return dict(v_bus=v, noload_rpm=rpm(w_nl), top_rpm=rpm(w_top), over_rpm_cap=w_nl > rad_s(RPM_CAP),
                H_noload_mNms=wd["J_kgm2"] * w_nl * 1e3, H_top_mNms=wd["J_kgm2"] * w_top * 1e3,
                E_noload_J=0.5 * wd["J_kgm2"] * w_nl ** 2,
                bond_shear_noload_MPa=m_mag * w_nl ** 2 * rot.magnet_r_mm / 1e3 / foot / 1e6,
                disc_stress_noload_MPa=(3 + POISSON) / 8 * STEEL_DENSITY * w_nl ** 2 * (rot.disc_od_mm / 2e3) ** 2 / 1e6,
                drag_noload_mNm=c * w_nl * 1e3, drag_loss_noload_W=c * w_nl ** 2,
                i_stall_A=m["i_stall_A"], tau_1A_mNm=kt * 1e3 * min(I_LIM_A, m["i_stall_A"]),
                tau_at_3500rpm_mNm=kt * 1e3 * float(np.clip((v_ph - ke * w_chk) / R, 0, I_LIM_A)),
                i_op_A=i_op, p_cu_op_W=p_cu, p_ext_op_W=p_ext, p_drag_op_W=p_drag,
                p_ripple_op_W=rip["ripple_loss_W_2000rpm_50kHz"],
                p_total_op_W=p_cu + p_ext + p_drag + rip["ripple_loss_W_2000rpm_50kHz"],
                **rip)


# ---------------------------------------------------------------- PWM ripple
def ripple_rms(v_bus, Ls, f, v_peak, n_ang=72, n_t=512):
    """RMS phase ripple current [A] for centre-aligned min-max-injected PWM producing a phase voltage
    of peak v_peak, averaged over one electrical revolution."""
    th = np.linspace(0, 2 * math.pi, n_ang, endpoint=False)[:, None]
    ref = v_peak * np.cos(th - np.arange(3)[None, :] * 2 * math.pi / 3)
    ref -= (ref.max(1, keepdims=True) + ref.min(1, keepdims=True)) / 2
    d = np.clip(0.5 + ref / v_bus, 0.0, 1.0)                         # leg duty, (angle, leg)
    c = np.abs(2 * (np.arange(n_t) + 0.5) / n_t - 1)                  # carrier: 1 at the edges, 0 mid-period
    legs = (d[:, :, None] > c[None, None, :]) * v_bus
    v_an = legs[:, 0] - legs.mean(1)
    dv = v_an - v_an.mean(-1, keepdims=True)
    i = np.cumsum(dv, -1) / (f * n_t) / Ls
    i -= i.mean(-1, keepdims=True)
    return float(np.sqrt(np.mean(i ** 2)))


# ---------------------------------------------------------------- study
def best_width(n, rot, log=print):
    """Lowest-loss trace cap and strand count for n conductors per layer (layout checks passed)."""
    best = None
    for cap in CAPS:
        for k in STRANDS:
            d = replace(RECOMMENDED, name=f"n{n}_cap{cap}_s{k}", n_per_layer=n, trace_mm=None, cap_mm=cap,
                        strands=k, swap=True)
            # too many strands for the trace: skip before the model sees zero or negative copper
            if min(copper_widths(d, w, r)[1].min() for r, _, w, _ in
                   (template(d, rot.n_poles, s) for s in (1, -1))) < d.min_trace_mm - 1e-9:
                continue
            res = analyse_wave(d, rot, n_theta=72)
            if not res["layout_ok"]:
                continue
            if best is None or res["loss_15mNm_2000rpm_W"] < best[1]["loss_15mNm_2000rpm_W"]:
                best = (d, res)
    log(f"n={n}: cap {best[0].cap_mm} strands {best[0].strands} Km {best[1]['Km_mNm_per_sqrtW']:.1f} "
        f"eddy2k {best[1]['eddy_W_at_2000rpm']:.3f} W loss {best[1]['loss_15mNm_2000rpm_W']:.3f} W", flush=True)
    return best


def run():
    rot = hm_rotor()
    rec = winding(RECOMMENDED, rot, 1)
    rec["name"] = "recommended " + rec["name"]
    fixed = [at_voltage(rec, v, rot) for v in VOLTAGES]
    target = at_voltage(rec, V_REF, rot)["noload_rpm"]
    windings = []
    for n in N_OPTIONS:
        d, base = best_width(n, rot)
        for a in (1, 2):
            windings.append(winding(d, rot, a, base if a == 1 else None))
            w = windings[-1]
            print(f"  {w['name']:22s} Kt {w['Kt_mNm_per_A']:5.1f} R {w['R_phase_ohm']:.3f} Km {w['Km_mNm_per_sqrtW']:.1f} "
                  f"Ls {w['Ls_uH']:.1f} uH (L {w['L_self_uH']:.1f}, M {w['M_ab_uH']:.1f}) circ_path {w['c_path']:.2e}",
                  flush=True)
    matrix = {w["name"]: {v: at_voltage(w, v, rot) for v in VOLTAGES} for w in windings}
    matched = []
    for v in VOLTAGES:
        ok = [w for w in windings if matrix[w["name"]][v]["noload_rpm"] >= SPEED_MARGIN * target]
        pick = max(ok, key=lambda w: (w["Kt_mNm_per_A"], -matrix[w["name"]][v]["p_total_op_W"])) if ok else None
        matched.append(dict(v_bus=v, winding=pick, op=matrix[pick["name"]][v] if pick else None))
    return dict(assumptions=dict(R_EXT_OHM=R_EXT_OHM, I_LIM_A=I_LIM_A, TAU_OP_MNM=TAU_OP_MNM, RPM_OP=RPM_OP,
                                 RPM_CAP=RPM_CAP, SPEED_MARGIN=SPEED_MARGIN, target_noload_rpm=target,
                                 EPOXY_SHEAR_MPA=EPOXY_SHEAR_MPA),
                recommended=rec, fixed=fixed, windings=windings,
                matrix={k: {str(v): x for v, x in row.items()} for k, row in matrix.items()}, matched=matched)


def _table(cols, rows):
    out = ["| " + " | ".join(h for h, _ in cols) + " |", "|" + "---|" * len(cols)]
    for r in rows:
        out.append("| " + " | ".join(f(r) for _, f in cols) + " |")
    return "\n".join(out)


def write(res):
    (OUT / "pcbmotor_bus_voltage.json").write_text(json.dumps(res, indent=2, default=float))
    a = res["assumptions"]
    rec = res["recommended"]
    md = ["# Bus-voltage sweep (high-momentum wave stator)", "",
          "Generated by `sim/pcbmotor/bus_voltage.py`. Rotor: 8 x N42 3/4 x 1/2 x 1/4 in blocks on the 84 mm, "
          "1.5 mm steel disc. Supplies 2S-8S LiPo nominal. "
          f"ASSUMED {a['R_EXT_OHM']} ohm per phase outside the motor (driver + harness), the same at every voltage. "
          f"Operating point {a['TAU_OP_MNM']:g} mN m at {a['RPM_OP']:g} rpm; ripple loss at 50 kHz PWM. "
          f"Speeds over the {a['RPM_CAP']:g} rpm cap (hand-balanced rotor) are shown but flagged.", "",
          "## 1. Same winding, different supply", "",
          f"Winding `{rec['name']}`: Kt {rec['Kt_mNm_per_A']:.1f} mN m/A, R {rec['R_phase_ohm']:.2f} ohm, "
          f"Km {rec['Km_mNm_per_sqrtW']:.1f}, Ls {rec['Ls_uH']:.1f} uH (L_self {rec['L_self_uH']:.1f}, "
          f"M_AB {rec['M_ab_uH']:.1f}), L/R {rec['elec_tau_us']:.1f} us.", "",
          _table([("V bus", lambda r: f"{r['v_bus']:.1f}"),
                  ("no-load rpm", lambda r: f"{r['noload_rpm']:.0f}" + (" (over cap)" if r["over_rpm_cap"] else "")),
                  ("H at no-load mN m s", lambda r: f"{r['H_noload_mNms']:.0f}"),
                  ("energy J", lambda r: f"{r['E_noload_J']:.1f}"),
                  ("magnet bond shear MPa", lambda r: f"{r['bond_shear_noload_MPa']:.2f}"),
                  ("drag at no-load mN m", lambda r: f"{r['drag_noload_mNm']:.2f}"),
                  ("drag loss at no-load W", lambda r: f"{r['drag_loss_noload_W']:.2f}"),
                  ("torque at 3500 rpm, 1 A lim mN m", lambda r: f"{r['tau_at_3500rpm_mNm']:.1f}"),
                  ("ripple rms A at 2000 rpm, 20/50/100 kHz", lambda r: f"{r['ripple_rms_A_2000rpm_20kHz']:.2f} / {r['ripple_rms_A_2000rpm_50kHz']:.2f} / {r['ripple_rms_A_2000rpm_100kHz']:.2f}"),
                  ("ripple loss W at 2000 rpm, 20/50/100 kHz", lambda r: f"{r['ripple_loss_W_2000rpm_20kHz']:.2f} / {r['ripple_loss_W_2000rpm_50kHz']:.2f} / {r['ripple_loss_W_2000rpm_100kHz']:.2f}"),
                  ("ripple loss W at 0 rpm, 50 kHz", lambda r: f"{r['ripple_loss_W_0rpm_50kHz']:.2f}"),
                  ("worst-case ripple loss W, 50 kHz", lambda r: f"{r['ripple_loss_W_worst_50kHz']:.2f}")],
                 res["fixed"]), "",
          "## 2. Winding options (each turn count with its best trace cap and strands)", "",
          _table([("winding", lambda w: w["name"]), ("series conductors/phase", lambda w: str(w["series_conductors_per_phase"])),
                  ("strand max mm", lambda w: f"{w['strand_max_mm']:.2f}"),
                  ("Kt mN m/A", lambda w: f"{w['Kt_mNm_per_A']:.1f}"), ("R ohm", lambda w: f"{w['R_phase_ohm']:.3f}"),
                  ("Km", lambda w: f"{w['Km_mNm_per_sqrtW']:.1f}"), ("Ls uH", lambda w: f"{w['Ls_uH']:.1f}"),
                  ("L/R us", lambda w: f"{w['elec_tau_us']:.1f}"),
                  ("eddy + circ W at 2000 rpm", lambda w: f"{(w['c_eddy'] + w['c_circ']) * rad_s(2000) ** 2:.3f}"),
                  ("of which between paths", lambda w: f"{w['c_path'] * rad_s(2000) ** 2:.3f}")],
                 res["windings"]), "",
          "No-load speed (rpm) of every option on every supply:", "",
          _table([("winding", lambda w: w["name"])] +
                 [(f"{v:g} V", (lambda v: lambda w: f"{res['matrix'][w['name']][str(v)]['noload_rpm']:.0f}")(v)) for v in VOLTAGES],
                 res["windings"]), "",
          f"## 3. Rewound to keep the top speed (no-load >= {SPEED_MARGIN:.0%} of {a['target_noload_rpm']:.0f} rpm)", "",
          _table([("V bus", lambda m: f"{m['v_bus']:.1f}"), ("winding", lambda m: m["winding"]["name"] if m["winding"] else "none"),
                  ("no-load rpm", lambda m: f"{m['op']['noload_rpm']:.0f}"),
                  ("Kt", lambda m: f"{m['winding']['Kt_mNm_per_A']:.1f}"), ("Km", lambda m: f"{m['winding']['Km_mNm_per_sqrtW']:.1f}"),
                  ("I at 15 mN m A", lambda m: f"{m['op']['i_op_A']:.2f}"),
                  ("motor Cu W", lambda m: f"{m['op']['p_cu_op_W']:.2f}"), ("driver/harness W", lambda m: f"{m['op']['p_ext_op_W']:.2f}"),
                  ("eddy+circ W", lambda m: f"{m['op']['p_drag_op_W']:.2f}"),
                  ("ripple rms A @50k", lambda m: f"{m['op']['ripple_rms_A_2000rpm_50kHz']:.2f}"),
                  ("ripple W @50k", lambda m: f"{m['op']['p_ripple_op_W']:.2f}"),
                  ("total W", lambda m: f"{m['op']['p_total_op_W']:.2f}")],
                 [m for m in res["matched"] if m["winding"]]), ""]
    (OUT / "pcbmotor_bus_voltage.md").write_text("\n".join(md))


def main():
    res = run()
    write(res)
    print((OUT / "pcbmotor_bus_voltage.md").read_text())


if __name__ == "__main__":
    main()
