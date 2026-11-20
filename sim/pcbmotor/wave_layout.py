"""As-built copper of a 4-layer wave-wound stator: conductors, strand lanes, vias, loop breaks,
jumpers and phase pads. The KiCad generator (`PCBMotorWaveTest/gen_wave_test.py`) draws exactly this
copper, and the checks here (connectivity, current direction, spacing) run on it.

Winding (pcbmotor/wave.py has the electromagnetic model and the reasons):
- 48 conductor positions per layer, 8 poles, three phase bands of q = 2 per pole (A, -C, B).
- Two layer pairs, each a full-pitch two-layer wave: pair 1 = L1 (top shape) + L2 (bottom shape),
  pair 2 = L3 (top) + L4 (bottom), rotated by half a slot.
- A top conductor at angle phi runs from its inner via at phi - tau/2 to its outer via at
  phi + tau/2; a bottom conductor mirrors that. Top j and bottom j + 6 share an outer via; bottom b
  and top b + 6 share an inner via; and so on round the board.

Why the pairs are nested. Through vias pierce every layer, and a pair's slanted end turns are packed
too tightly to let another pair's via through. So pair 2's via rings sit at r ~18.5 and ~44.6 mm.
Pair 1 runs straight (radially) past them, midway between the vias (half a slot away), then
slants to its own via rings further in (14.5 mm) and further out (~50 mm).

Strands. With strands > 1, a conductor's radial section and outer end turn are split into k lanes,
which merge into one solid trace at r_a (inner end). At each outer via the top and bottom lanes meet
where they cross:
- "swap" pairs lane i with lane k-1-i. The crossings stack radially at the via's angle, and every
  top + bottom loop links the same flux in each strand to first order.
- "fixed" pairs lane i with lane i. The crossings sit side by side on the via ring.

Loops and breaks. With every via in place, each phase forms closed wave loops on each pair:
2 slots x 2 pole parities = 4 loops per pair, 8 per phase. Each loop is opened at one outer via near
the phase's cluster angle:
- At the break, the top lanes merge on a short bar on the top layer, and the bottom lanes on the
  bottom layer.
- Each bar runs radially out on its own layer to a terminal via.
- Jumpers join the loops in series. Each is a radial lead on F.Cu, an arc on In1.Cu and In2.Cu in
  parallel, and a radial lead back.
- The first and last loop ends run out on B.Cu to the phase pads.

Coordinates: math frame, mm, stator centre at the origin. Layers 0..3 = F.Cu, In1.Cu, In2.Cu, B.Cu.
"""
from __future__ import annotations

from dataclasses import dataclass, field
import math

import numpy as np

LAYERS = ["F.Cu", "In1.Cu", "In2.Cu", "B.Cu"]
PHASES = "ABC"
KINK_MM = 1.2      # width taken as the narrowest within this radial distance (kinks)
SMOOTH_MM = 0.8    # kinks between radial and slanted sections are rounded over +- this
FILL_MARGIN_MM = 0.05
FAN_DR = 1.2       # break fan: lane ends join at this distance outside the via ring
STUB_JOG = 1.0     # break stubs jog sideways to their terminal via over this last radial distance
TOL_MM = 0.0005    # spacing-check tolerance (the KiCad-file check uses 0.0001)
VIA_BAND_MM = 0.0


@dataclass
class WaveLayoutParams:
    name: str = "wave_test"
    n: int = 48                 # conductor positions per layer
    n_poles: int = 8
    r_a: float = 22.0           # active (radial) section of pair 2, inner end; strands merge here
    r_b: float = 40.0           # active section of pair 2, outer end
    p1_in_via: float = 14.5     # pair 1 inner via ring
    p1_in_rad: float = 17.3     # pair 1 is radial from here outward ...
    p1_out_rad: float = 46.4    # ... to here
    p1_out_via: float = 51.0    # pair 1 outer via ring (strand vias sit either side of it)
    p2_in_via: float = 18.6
    p2_out_via: float = 44.6
    strands: int = 1
    swap: bool = True
    cap_mm: float = 1.2         # maximum conductor width (all strands and gaps)
    space_mm: float = 0.15
    min_trace_mm: float = 0.15
    via_drill: float = 0.3
    via_dia: float = 0.56       # JLC: 0.3 mm drill + 0.13 mm annular ring; no wider than a strand lane
    stub_w: float = 0.6         # break bars and stubs (on the conductor's own layer)
    lead_w: float = 0.8         # jumper leads (F.Cu) and phase leads (B.Cu)
    ring_w: float = 0.8         # jumper arcs (In1.Cu + In2.Cu)
    term_gap: float = 1.4       # gap from pair 1's outermost via copper to the terminal vias
    ring_pitch: float = 1.25
    cluster_deg: tuple = (10.0, 130.0, 250.0)   # where each phase's loop breaks gather
    ext_mm: float = 2.5         # slant extension past the outer via (for finding lane crossings)

    @property
    def delta(self):
        return 2 * math.pi / self.n

    @property
    def tau(self):
        return 2 * math.pi / self.n_poles

    @property
    def shift(self):
        return self.n // self.n_poles


# ---------------------------------------------------------------- conductor geometry
def _centreline(p: WaveLayoutParams, pair: int, sigma: int, ds=0.1):
    """(r, theta, dtheta/dr) along one conductor of `pair` centred on angle 0, from the inner via to
    past the outer via (the extension is cut back once the via crossing is known)."""
    tau = p.tau
    if pair == 1:
        r_iv, r_ir, r_or, r_ov = p.p1_in_via, p.p1_in_rad, p.p1_out_rad, p.p1_out_via
    else:
        r_iv, r_ir, r_or, r_ov = p.p2_in_via, p.r_a, p.r_b, p.p2_out_via
    k_in = sigma * tau / 2 / (r_ir - r_iv)
    k_out = sigma * tau / 2 / (r_ov - r_or)
    r = r_iv + ds * np.arange(int(round((r_ov + p.ext_mm - r_iv) / ds)) + 1)
    sl = np.where(r < r_ir, k_in, np.where(r <= r_or, 0.0, k_out))
    # round the kinks: moving average of the slope over +-SMOOTH_MM. The slope is constant near both
    # ends, so the angle change from the radial section to each via ring is unchanged.
    h = int(round(SMOOTH_MM / ds))
    sl = np.convolve(np.pad(sl, h, mode="edge"), np.ones(2 * h + 1) / (2 * h + 1), mode="valid")
    th = np.r_[0.0, np.cumsum(0.5 * (sl[1:] + sl[:-1]) * ds)]
    i0 = int(np.argmin(np.abs(r - 0.5 * (r_ir + r_or))))
    th -= th[i0]                                       # radial section at angle 0
    # the sampled step loses half a cell at each kink: rescale so the ends sit exactly on +-tau/2
    iv = int(np.argmin(np.abs(r - r_ov)))
    th = np.where(r < r[i0], th * (-sigma * tau / 2) / th[0], th * (sigma * tau / 2) / th[iv])
    return r, th, sl


def _width(p: WaveLayoutParams, pair, r, sl):
    """Total conductor width (all strands and gaps): fill the slot pitch perpendicular to the trace,
    capped, and narrowed where pair 1 runs past pair 2's via rings."""
    cos_psi = 1 / np.sqrt(1 + (r * sl) ** 2)
    # r * delta * cos(psi) is the pitch of parallel lines; curved, rotated copies sit a few per cent
    # closer, so keep FILL_MARGIN_MM in hand
    fill = r * p.delta * cos_psi - p.space_mm - FILL_MARGIN_MM
    if p.strands > 1:
        # strand vias (0.6 mm) are wider than the lanes they end: room for them near the outer vias
        r_ov = p.p1_out_via if pair == 1 else p.p2_out_via
        fill = np.where(np.abs(r - r_ov) < 1.5, fill - VIA_BAND_MM, fill)
    w = np.minimum(fill, p.cap_mm)
    # at a kink between a slant and the radial section the neighbour is closer than the
    # perpendicular pitch says: use the narrowest width within KINK_MM of each point
    win = np.abs(r[:, None] - r[None, :]) <= KINK_MM
    w = np.where(win, w[None, :], np.inf).min(axis=1)
    if pair == 1:
        lim = r * p.delta - 2 * (p.via_dia / 2 + p.space_mm)
        for rv in (p.p2_in_via, p.p2_out_via):
            w = np.where(np.abs(r - rv) < 1.6, np.minimum(w, lim), w)
    return w, cos_psi


def _positions(p: WaveLayoutParams, pair: int):
    """Radial-section angle, phase (0=A, 1=B, 2=C) and sign (+ = current outward) of each conductor.
    Pair 2 sits half a slot behind pair 1 and keeps pair 1's phase pattern."""
    d, tau = p.delta, p.tau
    base = (np.arange(p.n) + 0.5) * d
    band = np.floor(base / (tau / 3) + 1e-9).astype(int)
    pole, g = band // 3, band % 3
    table = [(0, 1), (2, -1), (1, 1)]
    ph = np.array([table[k][0] for k in g])
    sg = np.array([table[k][1] for k in g]) * np.where(pole % 2 == 0, 1, -1)
    return (base if pair == 1 else base - d / 2), ph, sg


def _polar(r, th):
    return np.c_[r * np.cos(th), r * np.sin(th)]


def _cut_at_radius(P, w, rv):
    """Keep the part of polyline P (from its inner end) inside radius rv, ending exactly on rv."""
    rr = np.hypot(P[:, 0], P[:, 1])
    i = int(np.searchsorted(rr, rv))           # rr increases along the conductor
    if i >= len(rr):
        return P, w
    if i == 0:
        return P[:1], w[:1]
    t = (rv - rr[i - 1]) / (rr[i] - rr[i - 1])
    q = P[i - 1] + t * (P[i] - P[i - 1])
    return np.vstack([P[:i], q]), np.r_[w[:i], w[i - 1]]


def _first_crossing(A, B):
    """First crossing of polylines A and B: (index in A, index in B, point), or None."""
    a0, da = A[:-1], np.diff(A, axis=0)
    b0, db = B[:-1], np.diff(B, axis=0)
    bmin, bmax = np.minimum(B[:-1], B[1:]), np.maximum(B[:-1], B[1:])
    for i in range(len(a0)):
        lo, hi = np.minimum(A[i], A[i + 1]), np.maximum(A[i], A[i + 1])
        cand = np.nonzero(np.all((bmin <= hi + 1e-9) & (bmax >= lo - 1e-9), axis=1))[0]
        for jj in cand:
            den = da[i, 0] * db[jj, 1] - da[i, 1] * db[jj, 0]
            if abs(den) < 1e-12:
                continue
            w = b0[jj] - a0[i]
            t = (w[0] * db[jj, 1] - w[1] * db[jj, 0]) / den
            u = (w[0] * da[i, 1] - w[1] * da[i, 0]) / den
            if -1e-9 <= t <= 1 + 1e-9 and -1e-9 <= u <= 1 + 1e-9:
                return i, jj, a0[i] + t * da[i]
    return None


def _conductor(p: WaveLayoutParams, pair, sigma, phi):
    """Solid part (inner via to r_a) and strand lanes (r_a outward) of one conductor at angle phi,
    each as (points, widths). With one strand the whole conductor is a single lane."""
    r, th, sl = _centreline(p, pair, sigma)
    W, cos_psi = _width(p, pair, r, sl)
    k = p.strands
    if k == 1:
        return None, [(_polar(r, th + phi), W)]
    s = r <= p.r_a + 1e-9
    solid = (_polar(r[s], th[s] + phi), W[s])
    m = r >= p.r_a - 1e-9
    C = _polar(r[m], th[m] + phi)
    WW = W[m]
    ws = (WW - (k - 1) * p.space_mm) / k
    lanes = []
    for j in range(k):
        x = -WW / 2 + ws / 2 + j * (ws + p.space_mm)       # offset to the left of travel (outward) = larger angle
        lanes.append((_offset(C, x), ws))
    return solid, lanes


def _offset(P, x):
    """Polyline P offset sideways by x (per point; + = left of the direction of travel), with
    mitred corners so parallel lanes keep their spacing round a kink."""
    t = np.diff(P, axis=0)
    t /= np.linalg.norm(t, axis=1, keepdims=True)
    n = np.c_[-t[:, 1], t[:, 0]]                          # left normals of the segments
    nv = np.vstack([n[:1], n[:-1] + n[1:], n[-1:]])
    nv /= np.linalg.norm(nv, axis=1, keepdims=True)
    cos_half = np.einsum("ij,ij->i", nv, np.vstack([n[:1], n]))
    return P + nv * (x / np.maximum(cos_half, 0.2))[:, None]


# ---------------------------------------------------------------- board
@dataclass
class Copper:
    layer: int
    pts: np.ndarray            # N x 2, mm
    w: np.ndarray              # N widths, mm
    net: str
    tag: str
    cond: tuple | None = None  # (pair, layer, j) of the conductor it belongs to
    strand: int | None = None


@dataclass
class Via:
    xy: tuple
    net: str
    tag: str


@dataclass
class WaveBoard:
    p: WaveLayoutParams
    copper: list
    vias: list
    pads: list                 # (xy, name, net)
    conductors: dict           # (pair, layer, j) -> dict(phase, sign, angle, sigma)
    chain: dict                # phase -> list of dict(b=break, start, end)
    info: dict


def top_layer(pair):
    return 0 if pair == 1 else 2


def bot_layer(pair):
    return 1 if pair == 1 else 3


def build(p: WaveLayoutParams) -> WaveBoard:
    tau, sh, k = p.tau, p.shift, p.strands
    copper, vias, pads = [], [], []
    conductors, lanes, solids = {}, {}, {}
    for pair in (1, 2):
        phi, ph, sg = _positions(p, pair)
        for L in (top_layer(pair), bot_layer(pair)):
            sigma = 1 if L == top_layer(pair) else -1
            for j in range(p.n):
                solid, ln = _conductor(p, pair, sigma, phi[j])
                conductors[(pair, L, j)] = dict(phase=PHASES[ph[j]], sign=int(sg[j]), angle=float(phi[j]),
                                                sigma=sigma)
                lanes[(pair, L, j)] = [list(x) for x in ln]
                solids[(pair, L, j)] = solid

    # ---- wave loops (tops j, j + 2 sh, ...) and one break per loop near its phase's cluster
    loops, seen = [], set()
    for pair in (1, 2):
        for j in range(p.n):
            if (pair, j) in seen:
                continue
            tops, jj = [], j
            while (pair, jj) not in seen:
                seen.add((pair, jj))
                tops.append(jj)
                jj = (jj + 2 * sh) % p.n
            loops.append((pair, tops))
    breaks = []
    for pair, tops in loops:
        phase = conductors[(pair, top_layer(pair), tops[0])]["phase"]
        c = math.radians(p.cluster_deg[PHASES.index(phase)])
        ang = lambda j: conductors[(pair, top_layer(pair), j)]["angle"] + tau / 2
        jb = min(tops, key=lambda j: abs((ang(j) - c + math.pi) % (2 * math.pi) - math.pi))
        breaks.append(dict(pair=pair, j=jb, phase=phase, angle=ang(jb)))
    broken = {(b["pair"], b["j"]) for b in breaks}

    def add(layer, pts, w, net, tag, cond=None, strand=None):
        pts = np.asarray(pts, float)
        copper.append(Copper(layer, pts, np.broadcast_to(np.asarray(w, float), (len(pts),)).copy(),
                             net, tag, cond, strand))

    def add_via(xy, net, tag):
        vias.append(Via((float(xy[0]), float(xy[1])), net, tag))
        return len(vias) - 1

    # ---- outer via sites
    for pair in (1, 2):
        rv = p.p1_out_via if pair == 1 else p.p2_out_via
        for j in range(p.n):
            T = (pair, top_layer(pair), j)
            Bk = (pair, bot_layer(pair), (j + sh) % p.n)
            net = "PHASE_" + conductors[T]["phase"]
            if (pair, j) in broken:
                for key in (T, Bk):
                    for lane in lanes[key]:
                        lane[0], lane[1] = _cut_at_radius(lane[0], lane[1], rv)
                continue
            for it in range(k):
                ib = (k - 1 - it) if p.swap else it
                A, wa = lanes[T][it]
                Bp, wb = lanes[Bk][ib]
                hit = _first_crossing(A, Bp)
                assert hit is not None, f"lanes don't cross at outer site pair {pair} j {j}"
                ia, jb, xy = hit
                lanes[T][it] = [np.vstack([A[:ia + 1], xy]), np.r_[wa[:ia + 1], wa[ia]]]
                lanes[Bk][ib] = [np.vstack([Bp[:jb + 1], xy]), np.r_[wb[:jb + 1], wb[jb]]]
                add_via(xy, net, f"outer p{pair} j{j} s{it}")

    # ---- inner via sites: top j and bottom j - sh meet exactly on the inner ring
    for pair in (1, 2):
        for j in range(p.n):
            T = (pair, top_layer(pair), j)
            xy = solids[T][0][0] if solids[T] is not None else lanes[T][0][0][0]
            add_via(xy, "PHASE_" + conductors[T]["phase"], f"inner p{pair} j{j}")

    for key, c in conductors.items():
        net = "PHASE_" + c["phase"]
        if solids[key] is not None:
            add(key[1], *solids[key], net, "solid", key)
        for s_i, (P_, w) in enumerate(lanes[key]):
            add(key[1], P_, w, net, "lane" if k > 1 else "conductor", key, s_i)

    # ---- breaks: merge bars on the via ring, radial stubs to terminal vias
    vr = p.via_dia / 2
    r_term = p.p1_out_via + 0.5 * (p.cap_mm + p.space_mm) + vr + p.term_gap + vr
    dterm = (p.via_dia + p.space_mm + 0.2) / r_term
    stub_end = {}
    for b in breaks:
        pair, j, v = b["pair"], b["j"], b["angle"]
        rv = p.p1_out_via if pair == 1 else p.p2_out_via
        net = "PHASE_" + b["phase"]
        # each stub leaves on the side its conductor doesn't arrive from (a top conductor reaches
        # the ring from smaller angles, a bottom one from larger), so the stub doesn't sit on its end
        for nm, key, side in (("top", (pair, top_layer(pair), j), +1),
                              ("bot", (pair, bot_layer(pair), (j + sh) % p.n), -1)):
            a_t = v + side * dterm
            # fan: each lane end runs straight out to one root FAN_DR beyond the via ring on the
            # site's own angle (a V that can't cross a lane), then the stub runs out radially (top and
            # bottom stubs on their own layers) and jogs sideways to its terminal via. A bar along the
            # ring would lie alongside the lanes, which meet the ring almost tangentially.
            root = _polar(np.array([rv + FAN_DR]), np.array([v]))[0]
            for lane in lanes[key]:
                e = lane[0][-1]
                if np.linalg.norm(e - root) > 1e-6:
                    # as wide as the lane it continues (a wider fan would bulge towards the next lane)
                    add(key[1], np.linspace(e, root, 8), min(p.stub_w, float(lane[1][-1])), net, "break fan")
            r_jog = r_term - STUB_JOG
            leg = _polar(np.linspace(rv + FAN_DR, r_jog, 30), np.full(30, v))
            jog = np.linspace(leg[-1], _polar(np.array([r_term]), np.array([a_t]))[0], 8)
            add(key[1], np.vstack([leg, jog[1:]]), p.stub_w, net, "break stub")
            vi = add_via(_polar(np.array([r_term]), np.array([a_t]))[0], net, f"terminal {b['phase']} p{pair} j{j} {nm}")
            stub_end[(pair, j, nm)] = (vi, a_t)

    # ---- chain each phase's loops in series (sorted by angle round the cluster)
    chain, jumpers, terminals = {}, [], {}
    for phase in PHASES:
        c = math.radians(p.cluster_deg[PHASES.index(phase)])
        bs = sorted((b for b in breaks if b["phase"] == phase),
                    key=lambda b: (b["angle"] - c + math.pi) % (2 * math.pi) - math.pi)
        seq = []
        for b in bs:
            out_top = conductors[(b["pair"], top_layer(b["pair"]), b["j"])]["sign"] > 0
            seq.append(dict(b=b, start="bot" if out_top else "top", end="top" if out_top else "bot"))
        for e, s in zip(seq[:-1], seq[1:]):
            jumpers.append((phase, stub_end[(e["b"]["pair"], e["b"]["j"], e["end"])],
                            stub_end[(s["b"]["pair"], s["b"]["j"], s["start"])]))
        terminals[phase] = (stub_end[(seq[0]["b"]["pair"], seq[0]["b"]["j"], seq[0]["start"])],
                            stub_end[(seq[-1]["b"]["pair"], seq[-1]["b"]["j"], seq[-1]["end"])])
        chain[phase] = seq

    # ---- jumper arcs onto rings (interval colouring), leads on F.Cu, arcs on In1 + In2
    r_ring0 = r_term + vr + p.space_mm + p.ring_w / 2 + 0.6
    rings, jinfo = [], []
    def overlaps(lo, hi, u0, u1, m):
        return any(not (hi + m < u0 + s or lo - m > u1 + s) for s in (-2 * math.pi, 0.0, 2 * math.pi))

    for phase, (_, a0), (_, a1) in jumpers:
        a0 = a0 % (2 * math.pi)
        span = (a1 - a0 + math.pi) % (2 * math.pi) - math.pi
        lo, hi = (a0, a0 + span) if span >= 0 else (a0 + span, a0)
        for ri, used in enumerate(rings):
            R = r_ring0 + ri * p.ring_pitch
            m = (p.via_dia + p.space_mm + 0.05) / R
            if not any(overlaps(lo, hi, u0, u1, m) for u0, u1 in used):
                used.append((lo, hi))
                break
        else:
            rings.append([(lo, hi)])
            ri = len(rings) - 1
        R = r_ring0 + ri * p.ring_pitch
        net = "PHASE_" + phase
        for a in (a0, a1):
            add(0, _polar(np.linspace(r_term, R, 20), np.full(20, a)), p.lead_w, net, "jumper lead")
            add_via(_polar(np.array([R]), np.array([a]))[0], net, f"jumper {phase}")
        aa = np.linspace(lo, hi, max(8, int((hi - lo) * R / 0.4)))
        for L in (1, 2):
            add(L, _polar(np.full_like(aa, R), aa), p.ring_w, net, "jumper arc")
        jinfo.append(dict(phase=phase, ring=ri, R=R, span_deg=math.degrees(hi - lo)))
    r_rings = [r_ring0 + i * p.ring_pitch for i in range(len(rings))]
    r_pad = (r_rings[-1] if r_rings else r_term) + 2.6
    for phase, ((_, a1), (_, a2)) in terminals.items():
        net = "PHASE_" + phase
        for nm, a in ((f"{phase}1", a1), (f"{phase}2", a2)):
            add(3, _polar(np.linspace(r_term, r_pad, 30), np.full(30, a)), p.lead_w, net, "phase lead")
            pads.append((tuple(_polar(np.array([r_pad]), np.array([a]))[0]), nm, net))

    info = dict(r_term=r_term, r_rings=r_rings, r_pad=r_pad, jumpers=jinfo, n_breaks=len(breaks),
                n_loops=len(loops), n_vias=len(vias))
    return WaveBoard(p, copper, vias, pads, conductors, chain, info)


# ---------------------------------------------------------------- checks
def _dist_to_polyline(q, P):
    a, b = P[:-1], P[1:]
    ab = b - a
    t = np.clip(np.einsum("ij,ij->i", q - a, ab) / np.maximum(np.einsum("ij,ij->i", ab, ab), 1e-12), 0, 1)
    d = np.linalg.norm(a + t[:, None] * ab - q, axis=1)
    return float(d.min()) if len(d) else float(np.linalg.norm(P[0] - q))


def adjacency(wb: WaveBoard, tol=0.01):
    """Which pieces touch: ends that meet on the same layer, ends on a via, an end on another piece's
    centreline on the same layer (bars, stubs, leads), and the solid part and lanes of one conductor.
    Nodes: copper index i, via index as ('v', i)."""
    from scipy.spatial import cKDTree
    edges = set()
    ends, own = [], []
    for i, c in enumerate(wb.copper):
        for e in (c.pts[0], c.pts[-1]):
            ends.append(e)
            own.append(i)
    ends = np.array(ends)
    tree = cKDTree(ends)
    for a, b in tree.query_pairs(tol):
        ia, ib = own[a], own[b]
        if ia != ib and wb.copper[ia].layer == wb.copper[ib].layer:
            edges.add((min(ia, ib), max(ia, ib)))
    for vi, v in enumerate(wb.vias):
        for e in tree.query_ball_point(v.xy, tol):
            edges.add((own[e], ("v", vi)))
    for i, c in enumerate(wb.copper):
        if c.tag not in ("break bar", "jumper arc"):
            continue
        lo, hi = c.pts.min(0) - 0.5, c.pts.max(0) + 0.5
        for e_i in np.nonzero(np.all((ends >= lo) & (ends <= hi), axis=1))[0]:
            j = own[e_i]
            if j != i and wb.copper[j].layer == c.layer and _dist_to_polyline(ends[e_i], c.pts) < tol:
                edges.add((min(i, j), max(i, j)))
    by_cond = {}
    for i, c in enumerate(wb.copper):
        if c.cond is not None:
            by_cond.setdefault(c.cond, []).append(i)
    for ids in by_cond.values():
        for a in ids:
            for b in ids:
                if a < b and (wb.copper[a].tag == "solid" or wb.copper[b].tag == "solid"):
                    edges.add((a, b))
    # pieces meeting on one bar are one joint (the lane ends and the stub sit within a bar length)
    on_bar = {}
    for a, b in list(edges):
        for x, y in ((a, b), (b, a)):
            if not isinstance(x, tuple) and wb.copper[x].tag == "break bar" and not isinstance(y, tuple):
                on_bar.setdefault(x, set()).add(y)
    for bar, ids in on_bar.items():
        ids = sorted(ids)
        for i_, a in enumerate(ids):
            for b in ids[i_ + 1:]:
                edges.add((a, b))
    # phase pads: the lead that ends on each
    for pi_, (xy, nm, net) in enumerate(wb.pads):
        for e in tree.query_ball_point(xy, tol):
            edges.add((own[e], ("pad", pi_)))
    return edges


def check_connectivity(wb: WaveBoard):
    """Every net must be one connected piece of copper with no short to another net."""
    edges = adjacency(wb)
    parent = {}
    def find(x):
        parent.setdefault(x, x)
        while parent[x] != x:
            parent[x] = parent[parent[x]]
            x = parent[x]
        return x
    for a, b in edges:
        parent[find(a)] = find(b)
    nodes = (list(range(len(wb.copper))) + [("v", i) for i in range(len(wb.vias))]
             + [("pad", i) for i in range(len(wb.pads))])
    def net(x):
        if isinstance(x, tuple):
            return wb.vias[x[1]].net if x[0] == "v" else wb.pads[x[1]][2]
        return wb.copper[x].net
    groups = {}
    for x in nodes:
        groups.setdefault(find(x), set()).add(net(x))
    shorts = sum(1 for s in groups.values() if len(s) > 1)
    per_net = {}
    for g, s in groups.items():
        for n_ in s:
            per_net[n_] = per_net.get(n_, 0) + 1
    return dict(shorts=shorts, pieces_per_net=per_net, ok=shorts == 0 and all(v == 1 for v in per_net.values()))


def series_path(wb: WaveBoard):
    """Walk each phase's chain through its loops using the wave rules (outer site: top j <-> bottom
    j + sh; inner site: bottom b <-> top b + sh) and return the conductors in order with the current's
    direction (+1 outward), plus a list of errors: conductors missed or repeated, current against a
    conductor's sign, or a loop that doesn't close back on its break."""
    p, sh = wb.p, wb.p.shift
    res, errs = {}, []
    for phase, seq in wb.chain.items():
        vis = []
        for item in seq:
            b = item["b"]
            pair, j = b["pair"], b["j"]
            T, Bl = top_layer(pair), bot_layer(pair)
            if item["start"] == "bot":
                cur, layer, dirn = (j + sh) % p.n, Bl, -1
            else:
                cur, layer, dirn = j, T, -1
            for _ in range(p.n_poles):
                vis.append(((pair, layer, cur), dirn))
                last = (layer, cur, dirn)
                if layer == T:
                    cur, layer, dirn = ((cur - sh) % p.n, Bl, +1) if dirn == -1 else ((cur + sh) % p.n, Bl, -1)
                else:
                    cur, layer, dirn = ((cur + sh) % p.n, T, +1) if dirn == -1 else ((cur - sh) % p.n, T, -1)
            # the last conductor must leave through the break's other end (outward, at site j)
            L_, c_, d_ = last
            closes = d_ == +1 and ((L_ == T and c_ == j) or (L_ == Bl and c_ == (j + sh) % p.n))
            if not closes:
                errs.append(f"{phase}: loop at p{pair} j{j} doesn't close on its break")
        res[phase] = vis
        keys = [k_ for k_, _ in vis]
        expect = [k_ for k_, c in wb.conductors.items() if c["phase"] == phase]
        if sorted(keys) != sorted(expect):
            errs.append(f"{phase}: visits {len(keys)} ({len(set(keys))} distinct), expected {len(expect)}")
        wrong = sum(1 for k_, dn in vis if dn != wb.conductors[k_]["sign"])
        if wrong:
            errs.append(f"{phase}: {wrong} conductors carry current against their sign")
    return res, errs


PAD_D = 2.0        # phase pad copper diameter (THT wire pad)


def check_spacing(wb: WaveBoard, space=None, ds=0.1):
    """Clearance between copper pieces (tracks, vias, pads) on each layer that don't touch each
    other, ignoring nets (a whole phase is one net, so KiCad's DRC can't see turn-to-turn gaps).
    Returns a list of (layer, gap_mm, tag_a, tag_b, xy)."""
    from scipy.spatial import cKDTree
    p = wb.p
    space = p.space_mm if space is None else space
    PAD0 = -1_000_000
    def nid(x):                       # copper i -> i, via v -> -1 - v, pad q -> PAD0 - q
        if isinstance(x, tuple):
            return -1 - x[1] if x[0] == "v" else PAD0 - x[1]
        return x
    adj = set()
    for a, b in adjacency(wb):
        adj.add((nid(a), nid(b)))
        adj.add((nid(b), nid(a)))
    tag = lambda o: wb.copper[o].tag if o >= 0 else (wb.vias[-1 - o].tag if o > PAD0 else "pad")
    cxy = np.array([v.xy for v in wb.vias] + [xy for xy, _, _ in wb.pads])
    cd = np.r_[np.full(len(wb.vias), p.via_dia), np.full(len(wb.pads), PAD_D)]
    cid = np.r_[-1 - np.arange(len(wb.vias)), PAD0 - np.arange(len(wb.pads))]
    out = []
    for L in range(4):
        P, W, O = [], [], []
        for i, c in enumerate(wb.copper):
            if c.layer != L:
                continue
            seg = np.linalg.norm(np.diff(c.pts, axis=0), axis=1)
            m = np.maximum(1, np.ceil(seg / ds).astype(int))
            for a_, b_, wa, wb_, mm in zip(c.pts[:-1], c.pts[1:], c.w[:-1], c.w[1:], m):
                t = np.arange(mm)[:, None] / mm
                P.append(a_ + (b_ - a_) * t)
                W.append(wa + (wb_ - wa) * t[:, 0])
            P.append(c.pts[-1:])
            W.append(c.w[-1:])
            O.append(np.full(int(m.sum()) + 1, i))
        P, W, O = np.vstack(P), np.concatenate(W), np.concatenate(O)
        tree = cKDTree(P)
        pr = tree.query_pairs(W.max() + space, output_type="ndarray")
        i, j = pr[:, 0], pr[:, 1]
        keep = O[i] != O[j]
        i, j = i[keep], j[keep]
        gap = np.linalg.norm(P[i] - P[j], axis=1) - (W[i] + W[j]) / 2
        bad = gap < space - TOL_MM
        found = [(O[a], O[b], g, (P[a] + P[b]) / 2) for a, b, g in zip(i[bad], j[bad], gap[bad])]
        for xy, d, o in zip(cxy, cd, cid):                   # vias and pads against tracks
            nb = np.array(tree.query_ball_point(xy, d / 2 + W.max() / 2 + space), int)
            if len(nb):
                g = np.linalg.norm(P[nb] - xy, axis=1) - d / 2 - W[nb] / 2
                for q, gg in zip(nb[g < space - TOL_MM], g[g < space - TOL_MM]):
                    found.append((o, O[q], gg, (P[q] + xy) / 2))
        if L == 0:                                           # vias and pads against each other (once)
            for a, b in cKDTree(cxy).query_pairs(cd.max() + space):
                g = np.linalg.norm(cxy[a] - cxy[b]) - (cd[a] + cd[b]) / 2
                if g < space - TOL_MM:
                    found.append((cid[a], cid[b], g, (cxy[a] + cxy[b]) / 2))
        for oa, ob, g, xy in found:
            if (int(oa), int(ob)) in adj:
                continue
            out.append((LAYERS[L], round(float(g), 3), tag(int(oa)), tag(int(ob)), tuple(np.round(xy, 2))))
    return out


def check_widths(wb: WaveBoard):
    return float(min(c.w.min() for c in wb.copper))


VARIANTS = {
    "solid": dict(strands=1, cap_mm=1.2),
    "swap": dict(strands=2, cap_mm=1.6, swap=True),
    "fixed": dict(strands=2, cap_mm=1.6, swap=False),
}


def variant(name) -> WaveLayoutParams:
    return WaveLayoutParams(name=f"wave_test_{name}", **VARIANTS[name])
