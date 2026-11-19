"""As-built coils wound from a bundle of parallel strands ("PCB Litz"), or from one solid trace.

`layout.coil_layout` winds one trace per layer. Here every turn of the coil is a bundle of
`n_strands` narrow lanes of width w_s with the strand gap between them, filling the same
width as the solid trace (bundle width W = trace_mm, bundle pitch W + space_mm). n_strands = 1
is the solid coil, so the solid and stranded test boards come from the same code.

Why strands: eddy loss in a trace scales with its width squared (pcbmotor/stacking.py), so
splitting a 1.2 mm trace into three 0.3 mm strands cuts the eddy drag by about 18x. The strands
only join at the coil terminals, so if they link different amounts of rotor flux a current
circulates round them and gives some of that back. That is why the lanes swap.

Lane swaps at the layer vias. With only through vias and radial stubs, the lane next to the
coil opening has to leave a layer first, so every via set reverses the lane order. A strand
therefore takes lane s on L1, n-1-s on L2, s on L3 and n-1-s on L4. Every strand gets the same
total offset over the four layers, which cancels the flux difference to first order. What's
left comes from L1 and L2 (and L3 and L4) sitting at different depths. `strand_test.py` works
out the circulating current that remains.

Vias (all through vias):
    in    L1 -> L2, one per strand, on a row across the coil opening (right half)
    out   L2 -> L3, one per strand, just outside the winding on the coil axis
    core  L3 -> L4, one per strand, same row in the opening (left half)
The opening is a wide, shallow slot for the payload geometry, so the vias sit side by side
along it rather than one above the other as in `layout.coil_layout`.

Terminals: on L1 the strands start from a short bus arc outside the winding at -beta/2 (S);
on L4 they end on a bus at +beta/2 (E), the same terminal angles as the test stator, so the
same ring routing works.

Lane j = 0 is the outermost lane of a bundle (nearest the coil boundary), j = n-1 the innermost
(nearest the coil opening).
"""
from __future__ import annotations

from dataclasses import dataclass, replace
import math

import numpy as np

from .geometry import StatorDesign
from .layout import Arc, Line, polar, transform, path_points, path_length, _corner

# JLCPCB multilayer: 0.3 mm drill is the free minimum; 0.6 mm pad leaves a 0.15 mm ring.
VIA_DRILL_MM = 0.3
VIA_DIA_MM = 0.6


@dataclass
class BundleCoil:
    design: StatorDesign
    n_strands: int
    strand_mm: float            # lane (strand) width w_s
    strand_gap_mm: float
    n_turns: int                # bundle turns per layer
    lanes: list                 # lanes[L][j] = path (list of Arc/Line) of lane j on layer L
    strands: list               # strands[s] = [(L, j), ...] lane used on each layer, L1..L4
    vias: dict                  # {"in"|"out"|"core": [xy for strand s]}
    bus_s: tuple                # (radius, a0, a1) of the S bus arc on L1 (a0 == a1 if solid)
    bus_e: tuple                # same for the E bus on L4
    term_s: tuple               # S terminal (bus centre), L1
    term_e: tuple               # E terminal, L4
    r_row: float                # radius of the via row in the opening
    r_out_row: float            # radius of the out vias
    r_bus: float
    via_dia_mm: float
    via_drill_mm: float

    def strand_paths(self, s):
        """[(layer index, path)] of strand s, L1 to L4, in the direction of current flow."""
        return [(L, self.lanes[L][j]) for L, j in self.strands[s]]

    def turns_per_layer(self):
        cx = 0.5 * (self.design.r_in_mm + self.design.r_out_mm)
        out = []
        for L in range(4):
            p = path_points(self.lanes[L][0], 0.1) - np.array([cx, 0.0])
            a = np.unwrap(np.arctan2(p[:, 1], p[:, 0]))
            out.append(abs(a[-1] - a[0]) / (2 * math.pi))
        return out

    @property
    def via_stub_mm(self):
        return min(self.strand_mm, self.via_dia_mm)

    def widths(self, path):
        """Track width of each piece of a lane. The stubs that end on a via are no wider than the
        via pad, so a wide trace's round end doesn't reach past the pad towards the next turn."""
        out = []
        for k, seg in enumerate(path):
            w = self.strand_mm
            if isinstance(seg, Line) and k in (0, len(path) - 1):
                for q in (seg.start, seg.end):
                    r = math.hypot(*q)
                    if abs(r - self.r_row) < 1e-6 or abs(r - self.r_out_row) < 1e-6:
                        w = self.via_stub_mm
            out.append(w)
        return out

    def lane_lengths_mm(self):
        return [[path_length(p) for p in layer] for layer in self.lanes]


def bundle_coil(d: StatorDesign, n_strands: int, strand_gap_mm: float | None = None,
                via_dia_mm: float = VIA_DIA_MM, via_drill_mm: float = VIA_DRILL_MM,
                margin_mm: float = 0.02, via_pitch: float = 1.3, lane_pitch: float = 1.5,
                bus_pitch_mm: float | None = None) -> BundleCoil:
    """Build the 4-layer series coil with n_strands lanes per turn.
    via_pitch: via-to-via spacing as a multiple of the minimum (pad + space).
    lane_pitch: tangential spacing of the strand stubs at the buses, as a multiple of the minimum.
    bus_pitch_mm: if given, the stub spacing at the bus radius instead (e.g. to land on a 1.3 mm
    pitch solder jumper)."""
    assert d.layers == 4, "written for 4-layer boards"
    d.check_rules()
    n = n_strands
    beta = math.pi / d.n_coils
    W, sp = d.trace_mm, d.space_mm
    g = sp if strand_gap_mm is None else strand_gap_mm
    ws = (W - (n - 1) * g) / n
    assert ws >= d.stack["min_trace_mm"] - 1e-9, f"strand width {ws:.3f} mm below the fab minimum"
    assert g >= d.stack["min_space_mm"] - 1e-9, "strand gap below the fab minimum"
    p = W + sp
    off = [d.coil_gap_mm / 2 + j * (ws + g) + ws / 2 for j in range(n)]

    def I(k, j):
        return off[j] + k * p

    def r1(k, j):
        return d.r_in_mm + I(k, j)

    def r2(k, j):
        return d.r_out_mm - I(k, j)

    vr = via_dia_mm / 2
    clr = vr + sp + ws / 2 + margin_mm          # via centre to lane centre
    s_v = via_pitch * max(via_dia_mm + sp, clr)  # via to via (and via to a neighbouring stub), centre to centre

    def row_fits(N):
        k, j = N - 1, n - 1
        if r2(k, j) - r1(k, j) < 2 * clr:
            return None
        r_row = 0.5 * (r1(k, j) + r2(k, j))
        x = (I(k, j) + clr) / r_row
        if x >= math.sin(beta):
            return None
        a_max = beta - math.asin(x)            # via centre stays clr from the opening's straight edges
        span = (2 * n - 1) * s_v / r_row
        return r_row if span <= 2 * a_max else None

    N = 1
    while row_fits(N + 1) is not None:
        N += 1
    r_row = row_fits(N)
    assert r_row is not None and N >= 2, "winding too small for the via row"

    # via angles on the row: core set on the left (negative), in set on the right
    dv = s_v / r_row
    pos = [(-(2 * n - 1) / 2 + i) * dv for i in range(2 * n)]
    psi = [pos[n - 1 - s] for s in range(n)]        # L3 lane s ends here: psi_{n-1} < ... < psi_0
    phi = [pos[n + s] for s in range(n)]            # L1 lane s ends here: phi_0 < ... < phi_{n-1}
    r_out_row = r2(0, 0) + clr
    do = s_v / r_out_row
    gam = [(m - (n - 1) / 2) * do for m in range(n)]  # L2 lane m ends here: increasing in m
    r_bus = d.r_out_mm - d.coil_gap_mm / 2 + sp + W / 2 + margin_mm
    dt = lane_pitch * (ws + sp + margin_mm) / r2(0, n - 1)
    if bus_pitch_mm is not None:
        dt = bus_pitch_mm / r_bus
        assert dt * r2(0, n - 1) >= ws + sp + margin_mm, "bus pitch too small"
    chi = [-beta / 2 - (s - (n - 1) / 2) * dt for s in range(n)]   # S bus, L1 lane s: decreasing in s
    eang = [beta / 2 + (m - (n - 1) / 2) * dt for m in range(n)]   # E bus, L4 lane m: increasing in m

    def inward(j, a_start, r_start, end, a_end, r_end):
        """Counter-clockwise inward spiral of lane j: radial stub in from r_start at a_start,
        N turns, then a radial stub at a_end from the last inner arc (end="inner") or the last
        outer arc (end="outer") to r_end."""
        path = [Line(polar(r_start, a_start), polar(r2(0, j), a_start))]
        for k in range(N):
            a0 = a_start if k == 0 else -_corner(beta, I(k - 1, j), r2(k, j))
            last = k == N - 1
            R1, R2 = r1(k, j), r2(k, j)
            if last and end == "outer":
                assert a_end > a0
                path += [Arc(R2, a0, a_end), Line(polar(R2, a_end), polar(r_end, a_end))]
                return path
            aR2, aR1 = _corner(beta, I(k, j), R2), _corner(beta, I(k, j), R1)
            assert k > 0 or a0 < aR2
            path += [Arc(R2, a0, aR2), Line(polar(R2, aR2), polar(R1, aR1))]
            if last:
                assert a_end < aR1
                path += [Arc(R1, aR1, a_end), Line(polar(R1, a_end), polar(r_end, a_end))]
                return path
            aL1 = -_corner(beta, I(k, j), R1)
            aL2 = -_corner(beta, I(k, j), r2(k + 1, j))
            path += [Arc(R1, aR1, aL1), Line(polar(R1, aL1), polar(r2(k + 1, j), aL2))]
        raise AssertionError("unreachable")

    def outward(j, a_start, r_start, end, a_end, r_end):
        """Outward spiral (mirror image, run backwards) that starts at (r_start, a_start) in the
        opening and ends at (r_end, a_end) outside the winding."""
        return transform(inward(j, -a_end, r_end, end, -a_start, r_start), mirror=True, reverse=True)

    rev = lambda j: n - 1 - j
    L1 = [inward(s, chi[s], r_bus, "inner", phi[s], r_row) for s in range(n)]
    # L2 lane m starts at the in via of strand n-1-m and ends at out via angle gam[m]
    L2 = [outward(m, phi[rev(m)], r_row, "inner", gam[m], r_out_row) for m in range(n)]
    # L3 lane s starts at the out via where L2 lane n-1-s ended
    L3 = [inward(s, gam[rev(s)], r_out_row, "outer", psi[s], r_row) for s in range(n)]
    L4 = [outward(m, psi[rev(m)], r_row, "outer", eang[m], r_bus) for m in range(n)]
    strands = [[(0, s), (1, rev(s)), (2, s), (3, rev(s))] for s in range(n)]
    vias = {"in": [polar(r_row, phi[s]) for s in range(n)],
            "out": [polar(r_out_row, gam[rev(s)]) for s in range(n)],
            "core": [polar(r_row, psi[s]) for s in range(n)]}
    return BundleCoil(design=d, n_strands=n, strand_mm=ws, strand_gap_mm=g, n_turns=N,
                      lanes=[L1, L2, L3, L4], strands=strands, vias=vias,
                      bus_s=(r_bus, min(chi), max(chi)), bus_e=(r_bus, min(eang), max(eang)),
                      term_s=polar(r_bus, -beta / 2), term_e=polar(r_bus, beta / 2),
                      r_row=r_row, r_out_row=r_out_row, r_bus=r_bus,
                      via_dia_mm=via_dia_mm, via_drill_mm=via_drill_mm)


def check_bundle(bc: BundleCoil, space_mm: float | None = None, step_mm: float = 0.03):
    """Self-checks: every strand is one continuous path through its vias, and on every layer no
    two different strands (or a strand and a via it doesn't use) come closer than the space."""
    from scipy.spatial import cKDTree
    d = bc.design
    space = d.space_mm if space_mm is None else space_mm
    tol = 1e-6
    n = bc.n_strands
    for s in range(n):
        segs = bc.strand_paths(s)
        for L, path in segs:
            for a, b in zip(path[:-1], path[1:]):
                assert np.allclose(a.end, b.start, atol=tol), f"strand {s} L{L + 1} not continuous"
        ends = [(path[0].start, path[-1].end) for _, path in segs]
        assert abs(math.hypot(*ends[0][0]) - bc.r_bus) < tol and abs(math.hypot(*ends[3][1]) - bc.r_bus) < tol
        assert np.allclose(ends[0][1], bc.vias["in"][s]) and np.allclose(ends[1][0], bc.vias["in"][s])
        assert np.allclose(ends[1][1], bc.vias["out"][s]) and np.allclose(ends[2][0], bc.vias["out"][s])
        assert np.allclose(ends[2][1], bc.vias["core"][s]) and np.allclose(ends[3][0], bc.vias["core"][s])
    # spacing between strands, layer by layer (strands only meet at the buses, outside the winding)
    attached = {"in": (0, 1), "out": (1, 2), "core": (2, 3)}
    worst = math.inf
    hw = bc.strand_mm / 2
    vr = bc.via_dia_mm / 2
    for L in range(4):
        pts, owner, stub = [], [], []
        for s in range(n):
            j = dict(bc.strands[s])[L]
            path = bc.lanes[L][j]
            for k, seg in enumerate(path):
                pp = seg.points(step_mm)
                pts.append(pp)
                owner.append(np.full(len(pp), s))
                stub.append(np.full(len(pp), k in (0, len(path) - 1)))
        P, O, ST = np.vstack(pts), np.concatenate(owner), np.concatenate(stub)
        if n > 1:
            tree = cKDTree(P)
            pr = tree.query_pairs(2 * hw + space - 1e-4, output_type="ndarray")
            pr = pr[O[pr[:, 0]] != O[pr[:, 1]]]
            # stubs meet at the bus: ignore points beyond the winding
            rr = np.hypot(P[:, 0], P[:, 1])
            lim = bc.r_bus - d.trace_mm / 2 - 1e-6
            pr = pr[(rr[pr[:, 0]] < lim) & (rr[pr[:, 1]] < lim)]
            if len(pr):
                gap = np.linalg.norm(P[pr[:, 0]] - P[pr[:, 1]], axis=1).min() - 2 * hw
                raise AssertionError(f"L{L + 1}: strands {gap:.3f} mm apart")
        for kind, layers in attached.items():
            for s, v in enumerate(bc.vias[kind]):
                gap = np.linalg.norm(P - np.array(v), axis=1) - hw - vr
                ok = (O == s) & ST if L in layers else np.zeros(len(P), bool)   # the stub landing on it
                worst = min(worst, float(gap[~ok].min()))
    allv = [np.array(v) for k in bc.vias for v in bc.vias[k]]
    for a in range(len(allv)):
        for b in range(a + 1, len(allv)):
            worst = min(worst, float(np.linalg.norm(allv[a] - allv[b]) - 2 * vr))
    assert worst >= space - 1e-4, f"via clearance {worst:.3f} mm < {space} mm"
    return True


# The two test boards in PCBMotorStrandTest/ (shared by the KiCad generator and strand_test.py):
# the payload stator geometry on JLC's all-2 oz 4-layer board, the best JLC option in
# pcbmotor/ace_stackups.py.
STRAND_TEST_STACKUP = "jlc4_2oz"
# The stranded board's strands land 1.3 mm apart at the buses so that coil 0 can take a 3-pad
# solder jumper (SolderJumper-3_P1.3mm) in place of its S bus.
STRAND_TEST = {"solid": dict(n_strands=1), "stranded": dict(n_strands=3, bus_pitch_mm=1.3)}


def strand_test_design(d: StatorDesign) -> StatorDesign:
    return replace(d, stackup=STRAND_TEST_STACKUP, name=f"{d.name}_{STRAND_TEST_STACKUP}")


def strand_test_coil(d: StatorDesign, variant: str) -> BundleCoil:
    return bundle_coil(d, **STRAND_TEST[variant])
