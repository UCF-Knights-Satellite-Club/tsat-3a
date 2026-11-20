"""Electromagnetic model of the wave test boards exactly as laid out by pcbmotor/wave_layout.py:
torque constant, phase resistance, eddy drag and the current circulating between strands.

Same method and rotor as pcbmotor/wave.py (magpylib Lorentz force on the high-momentum payload rotor,
back iron as an image magnet). Every unbroken conductor of a given (pair, layer) is the same copper
rotated, so one template conductor per layer is evaluated and shifted round the board. The 8 broken
conductors per phase (cut at the via ring instead of at the lane crossing) differ by less than a
millimetre of end turn and are treated like the rest.

Run from sim/:  .venv/bin/python -m pcbmotor.wave_asbuilt
"""
from __future__ import annotations

import json
import math
import pathlib

import numpy as np

from .em import rotor_magnets, RHO_CU_20C
from .geometry import STACKUPS, OZ_TO_M
from .designs import drive_metrics
from . import wave
from . import wave_layout as wl

OUT = pathlib.Path(__file__).resolve().parents[1] / "results"
STACKUP = "jlc4_2oz"           # the strand-test boards' stackup (2 oz on all four layers)
# ASSUMED heavy-copper build (ACE-style, not a quoted stackup): 3 oz on all four layers at the same
# layer depths, with the typical heavy-copper minimum space of 0.2 mm. ACE doesn't publish its spacing
# at heavy copper; ask before relying on it.
STACKUPS.setdefault("ace4_3oz", dict(fab="ACE (assumed)", z_mm=[0.0, 0.21, 1.39, 1.6], oz=[3.0] * 4,
                                     min_trace_mm=0.2, min_space_mm=0.2))
SIGMA_CU = 1 / RHO_CU_20C
VIA_PLATING_M = 25e-6
BOARD_T_MM = 1.6
BOARD_AREA_M2 = 0.12 * 0.12


def _res(P, w, t):
    ds = np.linalg.norm(np.diff(P, axis=0), axis=1) / 1e3
    wm = (w[:-1] + w[1:]) / 2 / 1e3
    return float(np.sum(RHO_CU_20C * ds / (wm * t)))


def _template_keys(wb):
    """One unbroken conductor per (pair, layer): its own outer site and its inner partner intact."""
    p, sh = wb.p, wb.p.shift
    broken = {(it["b"]["pair"], it["b"]["j"]) for seq in wb.chain.values() for it in seq}
    keys = {}
    for (pair, L, j), c in wb.conductors.items():
        if (pair, L) in keys:
            continue
        site = j if L == wl.top_layer(pair) else (j - sh) % p.n
        if (pair, site) not in broken:
            keys[(pair, L)] = j
    return keys


def _pieces(wb, key):
    """Copper pieces of one conductor: solid part and lanes, each oriented inner -> outer."""
    return [c for c in wb.copper if c.cond == key]


def _curves(pieces, z_mm, thetas, rot):
    """Per piece: torque on the rotor per amp of outward current vs rotor angle, and the
    eddy coefficient. Returns lists aligned with pieces."""
    tq, ce = [], []
    for c in pieces:
        P = np.c_[c.pts / 1e3, np.full(len(c.pts), -z_mm / 1e3)]
        mids, dls = (P[:-1] + P[1:]) / 2, np.diff(P, axis=0)
        wm = (c.w[:-1] + c.w[1:]) / 2 / 1e3
        t = np.empty(len(thetas))
        Bz = np.empty((len(thetas), len(mids)))
        for i, a in enumerate(thetas):
            B = rotor_magnets(rot, a).getB(mids)
            F = np.cross(dls, B)
            t[i] = -np.sum(mids[:, 0] * F[:, 1] - mids[:, 1] * F[:, 0])
            Bz[i] = B[:, 2]
        dth = thetas[1] - thetas[0]
        dB = (np.roll(Bz, -1, 0) - np.roll(Bz, 1, 0)) / (2 * dth)
        tq.append(t)
        ce.append(dB)          # eddy needs the copper thickness too: finished by the caller
        c._mids, c._len, c._wm = mids, np.linalg.norm(dls, axis=1), wm
    return tq, ce


def _shift(thetas, curve, x):
    P = len(thetas) * (thetas[1] - thetas[0])
    return np.interp(x % P, np.r_[thetas, P], np.r_[curve, curve[0]])


def analyse(name, n_theta=72, stackup=STACKUP):
    """As-built model of layout variant `name` on `stackup`. The copper is laid out at that
    stackup's minimum space (heavier copper needs wider gaps)."""
    from dataclasses import replace
    p = wl.variant(name)
    sp = STACKUPS[stackup].get("min_space_mm", p.space_mm)
    if sp > p.space_mm:
        # wider gaps leave narrower strand lanes (0.30 mm at 3 oz), and a 0.56 mm via pad would reach
        # past its lane into the gap: use 0.4 / 0.2 mm vias (blind vias in an HDI build are smaller still)
        p = replace(p, space_mm=sp, min_trace_mm=max(p.min_trace_mm, STACKUPS[stackup].get("min_trace_mm", 0)),
                    via_dia=min(p.via_dia, 0.4), via_drill=min(p.via_drill, 0.2))
    wb = wl.build(p)
    st = STACKUPS[stackup]
    t_cu = [oz * OZ_TO_M for oz in st["oz"]]
    rot = wave.hm_rotor()
    period = 2 * (2 * math.pi / rot.n_poles)
    thetas = np.linspace(0, period, n_theta, endpoint=False)
    keys = _template_keys(wb)
    tmpl = {}
    for (pair, L), j in keys.items():
        pcs = _pieces(wb, (pair, L, j))
        tq, dB = _curves(pcs, st["z_mm"][L], thetas, rot)
        eddy = sum(float(np.sum(SIGMA_CU * c._wm ** 2 / 12 * np.mean(d ** 2, axis=0) * c._wm * t_cu[L] * c._len))
                   for c, d in zip(pcs, dB))
        lanes = [(c, t) for c, t in zip(pcs, tq) if c.tag != "solid"]
        solid = [t for c, t in zip(pcs, tq) if c.tag == "solid"]
        tmpl[(pair, L)] = dict(angle=wb.conductors[(pair, L, j)]["angle"],
                               lane_tq=[t for _, t in lanes], solid_tq=(solid[0] if solid else 0 * thetas),
                               lane_R=[_res(c.pts, c.w, t_cu[L]) for c, _ in lanes],
                               solid_R=sum(_res(c.pts, c.w, t_cu[L]) for c in pcs if c.tag == "solid"),
                               eddy=eddy)
    k = p.strands
    # ---- torque: each conductor carries its sign x phase current; lanes share it equally
    pp = rot.n_poles // 2
    th = np.linspace(0, 2 * math.pi / pp, 240, endpoint=False)
    kph = np.zeros((3, len(th)))
    for (pair, L, j), c in wb.conductors.items():
        T = tmpl[(pair, L)]
        shift = c["angle"] - T["angle"]
        curve = T["solid_tq"] + sum(T["lane_tq"]) / len(T["lane_tq"])
        kph[wl.PHASES.index(c["phase"])] += c["sign"] * _shift(thetas, curve, th - shift)
    kt, ripple = wave.foc(kph, th, pp)
    # ---- resistance of one phase
    a_via = math.pi * (p.via_drill / 1e3) * VIA_PLATING_M
    r_via = RHO_CU_20C * BOARD_T_MM / 1e3 / a_via
    R = 0.0
    for (pair, L, j), c in wb.conductors.items():
        if c["phase"] != "A":
            continue
        T = tmpl[(pair, L)]
        R += T["solid_R"] + 1 / sum(1 / x for x in T["lane_R"])
    n_cond = sum(1 for c in wb.conductors.values() if c["phase"] == "A")
    R += (n_cond / 2) * r_via + (n_cond / 2) * r_via / k          # inner vias, outer strand vias
    extra = 0.0
    for c in wb.copper:
        if c.net != "PHASE_A" or c.cond is not None:
            continue
        r_ = _res(c.pts, c.w, t_cu[c.layer])
        extra += r_ / 2 if c.tag == "jumper arc" else r_       # arcs on In1 and In2 in parallel
    extra += sum(1 for v in wb.vias if v.net == "PHASE_A" and v.tag.startswith(("terminal", "jumper"))) * r_via
    R += extra
    # ---- eddy (all three phases) and strand circulating current
    c_eddy = sum(tmpl[(pr, L)]["eddy"] for (pr, L, j) in wb.conductors)
    c_circ = 0.0
    if k > 1:
        sh = p.shift
        for pair in (1, 2):
            Tt, Tb = tmpl[(pair, wl.top_layer(pair))], tmpl[(pair, wl.bot_layer(pair))]
            # one strand loop per conductor pair: top j lane i -> outer via -> bottom j + sh lane i'
            # (traversed inward). EMF per rad/s = torque per amp along the path.
            E = []
            G = []
            for i in range(k):
                ib = (k - 1 - i) if p.swap else i
                E.append((Tt["lane_tq"][i], Tb["lane_tq"][ib]))
                G.append(1 / (Tt["lane_R"][i] + Tb["lane_R"][ib] + r_via))
            G = np.array(G)
            for j in range(p.n):
                a_t = wb.conductors[(pair, wl.top_layer(pair), j)]["angle"]
                a_b = wb.conductors[(pair, wl.bot_layer(pair), (j + sh) % p.n)]["angle"]
                e = np.array([_shift(thetas, et, thetas - (a_t - Tt["angle"]))
                              - _shift(thetas, eb, thetas - (a_b - Tb["angle"])) for et, eb in E])
                V = (G[:, None] * e).sum(0) / G.sum()
                c_circ += float(np.mean((G[:, None] * (e - V) ** 2).sum(0)))
    J, _ = rot.inertia_kgm2()
    m = drive_metrics(kt, R, float("nan"), kt * 2 / 3, J, BOARD_AREA_M2, 1.0)
    w2k = 2000 * 2 * math.pi / 60
    km = m["Km_mNm_per_sqrtW"]
    return dict(name=name, strands=k, swap=p.swap, cap_mm=p.cap_mm, stackup=stackup, buried=p.buried, space_mm=p.space_mm, via_mm=[p.via_drill, p.via_dia],
                Kt_mNm_per_A=kt * 1e3, R_phase_ohm=R, R_links_ohm=extra, Km_mNm_per_sqrtW=km,
                ripple_pct=ripple * 100, noload_rpm=m["noload_rpm"],
                eddy_W_at_2000rpm=c_eddy * w2k ** 2, circulating_W_at_2000rpm=c_circ * w2k ** 2,
                drag_mNm_at_2000rpm=(c_eddy + c_circ) * w2k * 1e3,
                loss_15mNm_2000rpm_W=(15 / km) ** 2 + (c_eddy + c_circ) * w2k ** 2,
                board=dict(r_term=wb.info["r_term"], r_rings=wb.info["r_rings"], r_pad=wb.info["r_pad"],
                           vias=len(wb.vias), breaks=wb.info["n_breaks"]))


def main():
    rows = []
    runs = [(name, STACKUP) for name in wl.VARIANTS]
    runs += [(name, "ace4_3oz") for name in ("bv_solid", "bv_swap", "bv_fixed")]
    for name, st in runs:
        r = analyse(name, stackup=st)
        rows.append(r)
        print({k_: (round(v, 4) if isinstance(v, float) else v) for k_, v in r.items() if k_ != "board"}, flush=True)
    (OUT / "pcbmotor_wave_asbuilt.json").write_text(json.dumps(rows, indent=2, default=float))


if __name__ == "__main__":
    main()
