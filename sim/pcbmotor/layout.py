"""As-built copper of a PCB stator: real spirals, layer vias and leads.

`geometry.coil_turns` models a coil as concentric closed loops (good for the design sweep).
A board needs one continuous spiral per layer, joined by vias. This module builds that
copper from the same `StatorDesign`, and both users read it:

- the KiCad generator (`PCBMotorTest/gen_stator.py`) draws these tracks, arcs and vias;
- `em.analyse_layout` runs the torque and resistance model on exactly this copper.

Coil shape: each turn is the coil's sector (between two radial boundary lines, one per
neighbouring coil) shrunk inward by a constant distance. So the straight edges are parallel
to the sector boundary and every gap between turns is exactly `space_mm`. That differs from
`coil_turns`, whose edges are radial lines through the origin. The two shapes touch at the
inner corners, and the as-built turns are a little wider at the outer radius.

Four-layer series coil (only through vias, since OSH Park has no blind vias), with the current
going counter-clockwise (seen from the rotor side) on every layer:

    L1  inward spiral, from the coil terminal S to via V_in in the coil centre
    L2  outward spiral, from V_in to via V_out just outside the winding
    L3  inward spiral, from V_out to via V_core in the coil centre
    L4  outward spiral, from V_core to the coil terminal E

V_in and V_core sit on the coil axis, one at each end of the centre opening. Turns are
dropped from the centre until both vias fit with full clearance on every layer.
"""
from __future__ import annotations

from dataclasses import dataclass
import math
import numpy as np

from .geometry import StatorDesign


# OSH Park 4-layer: 10 mil drill, 4 mil annular ring. We use a 5 mil ring for margin.
VIA_DRILL_MM = 0.254
VIA_DIA_MM = 0.508


@dataclass(frozen=True)
class Arc:
    r: float      # radius (mm), centred on the stator axis
    a0: float     # start angle (rad)
    a1: float     # end angle (rad); a1 > a0 is counter-clockwise

    def mirror(self):
        return Arc(self.r, -self.a0, -self.a1)

    def reverse(self):
        return Arc(self.r, self.a1, self.a0)

    def rotate(self, phi):
        return Arc(self.r, self.a0 + phi, self.a1 + phi)

    @property
    def start(self):
        return np.array([self.r * math.cos(self.a0), self.r * math.sin(self.a0)])

    @property
    def end(self):
        return np.array([self.r * math.cos(self.a1), self.r * math.sin(self.a1)])

    @property
    def mid(self):
        a = 0.5 * (self.a0 + self.a1)
        return np.array([self.r * math.cos(a), self.r * math.sin(a)])

    @property
    def length(self):
        return abs(self.a1 - self.a0) * self.r

    def points(self, max_seg_mm=0.25):
        n = max(2, int(math.ceil(self.length / max_seg_mm)) + 1)
        a = np.linspace(self.a0, self.a1, n)
        return np.c_[self.r * np.cos(a), self.r * np.sin(a)]


@dataclass(frozen=True)
class Line:
    p0: tuple
    p1: tuple

    def mirror(self):
        return Line((self.p0[0], -self.p0[1]), (self.p1[0], -self.p1[1]))

    def reverse(self):
        return Line(self.p1, self.p0)

    def rotate(self, phi):
        c, s = math.cos(phi), math.sin(phi)
        rot = lambda p: (p[0] * c - p[1] * s, p[0] * s + p[1] * c)
        return Line(rot(self.p0), rot(self.p1))

    @property
    def start(self):
        return np.array(self.p0)

    @property
    def end(self):
        return np.array(self.p1)

    @property
    def length(self):
        return float(np.hypot(self.p1[0] - self.p0[0], self.p1[1] - self.p0[1]))

    def points(self, max_seg_mm=0.25):
        n = max(2, int(math.ceil(self.length / max_seg_mm)) + 1)
        t = np.linspace(0, 1, n)[:, None]
        return np.array(self.p0) + (np.array(self.p1) - np.array(self.p0)) * t


def polar(r, a):
    return (r * math.cos(a), r * math.sin(a))


def path_points(path, max_seg_mm=0.25):
    """One polyline (N x 2, mm) through a list of Arc/Line pieces."""
    pts = [path[0].points(max_seg_mm)]
    for seg in path[1:]:
        pts.append(seg.points(max_seg_mm)[1:])
    return np.vstack(pts)


def path_length(path):
    return sum(s.length for s in path)


def transform(path, mirror=False, reverse=False, rotate=0.0):
    out = list(path)
    if mirror:
        out = [s.mirror() for s in out]
    if reverse:
        out = [s.reverse() for s in reversed(out)]
    if rotate:
        out = [s.rotate(rotate) for s in out]
    return out


@dataclass
class CoilLayout:
    """Copper of one coil at angle 0. Paths are lists of Arc/Line in order of current flow."""
    design: StatorDesign
    n_turns: int                 # turn rings used (the innermost ones are dropped to fit two vias)
    layers: list                 # [path per copper layer], L1 = rotor side
    via_in: tuple                # L1 -> L2, coil centre (inner end of the opening)
    via_out: tuple               # L2 -> L3, just outside the winding on the coil axis
    via_core: tuple              # L3 -> L4, coil centre (outer end of the opening)
    term_s: tuple                # start terminal on L1 (outer edge of the winding)
    term_e: tuple                # end terminal on L4
    clearance_c: float           # via centre to trace centre distance used
    via_dia_mm: float = VIA_DIA_MM
    via_drill_mm: float = VIA_DRILL_MM

    def turns_per_layer(self):
        """Turns actually wound on each layer (angle swept round the coil centre / 2 pi)."""
        out = []
        cx = 0.5 * (self.design.r_in_mm + self.design.r_out_mm)
        for path in self.layers:
            p = path_points(path, 0.1) - np.array([cx, 0.0])
            a = np.unwrap(np.arctan2(p[:, 1], p[:, 0]))
            out.append(abs(a[-1] - a[0]) / (2 * math.pi))
        return out


def _corner(beta, d, r):
    """Angle where the edge line (boundary at +beta shifted inward by d) meets radius r."""
    return beta - math.asin(d / r)


def _inward_spiral(d: StatorDesign, n_turns, theta_start, end, c):
    """Counter-clockwise inward spiral for the coil centred on angle 0.
    Starts on the outermost turn's outer arc at theta_start. end = "inner" stops on the last
    turn's inner arc at angle 0 and runs a stub out to the via there; end = "outer" stops on the
    last turn's outer arc at angle 0 and runs a stub in to the via there."""
    beta = math.pi / d.n_coils
    p = d.pitch_mm
    inset = [d.coil_gap_mm / 2 + d.trace_mm / 2 + k * p for k in range(n_turns + 1)]
    r1 = [d.r_in_mm + x for x in inset]
    r2 = [d.r_out_mm - x for x in inset]
    path = []
    for k in range(n_turns):
        a_start = theta_start if k == 0 else -_corner(beta, inset[k - 1], r2[k])
        last = k == n_turns - 1
        if last and end == "outer":
            path.append(Arc(r2[k], a_start, 0.0))
            path.append(Line(polar(r2[k], 0.0), polar(r2[k] - c, 0.0)))
            return path
        aR2, aR1 = _corner(beta, inset[k], r2[k]), _corner(beta, inset[k], r1[k])
        path.append(Arc(r2[k], a_start, aR2))
        path.append(Line(polar(r2[k], aR2), polar(r1[k], aR1)))
        if last:
            path.append(Arc(r1[k], aR1, 0.0))
            path.append(Line(polar(r1[k], 0.0), polar(r1[k] + c, 0.0)))
            return path
        aL1 = -_corner(beta, inset[k], r1[k])
        path.append(Arc(r1[k], aR1, aL1))
        aL2 = -_corner(beta, inset[k], r2[k + 1])
        path.append(Line(polar(r1[k], aL1), polar(r2[k + 1], aL2)))
    raise AssertionError("unreachable")


def coil_layout(d: StatorDesign, theta_s=None, margin_mm=0.02, via_dia_mm=VIA_DIA_MM,
                via_drill_mm=VIA_DRILL_MM) -> CoilLayout:
    """Build the 4-layer series coil. theta_s is the angle of the S terminal (default: halfway
    between the coil axis and its left boundary). The E terminal is its mirror image. The via
    size defaults to OSH Park's; pass the fab's own for other boards."""
    assert d.layers == 4, "this layout is written for 4-layer boards"
    d.check_rules()
    beta = math.pi / d.n_coils
    theta_s = -beta / 2 if theta_s is None else theta_s
    # via centre to neighbouring trace centre
    c = via_dia_mm / 2 + d.space_mm + d.trace_mm / 2 + margin_mm
    via_gap = via_dia_mm + d.space_mm + margin_mm      # via pad to via pad, centre to centre
    p = d.pitch_mm
    n = 0
    while True:   # largest turn count whose centre opening still holds both vias
        k = n       # candidate last turn index
        inset = d.coil_gap_mm / 2 + d.trace_mm / 2 + k * p
        r1, r2 = d.r_in_mm + inset, d.r_out_mm - inset
        if r1 >= r2 or inset >= r1 * math.sin(beta):
            break
        v_in, v_core = r1 + c, r2 - c
        ok = (v_core - v_in >= via_gap
              # both vias clear the straight edges of the last turn
              and v_in * math.sin(beta) - inset >= c and v_core * math.sin(beta) - inset >= c)
        if not ok:
            break
        n += 1
    assert n >= 2, "winding too small for a 4-layer via arrangement"
    inset0 = d.coil_gap_mm / 2 + d.trace_mm / 2
    insetL = inset0 + (n - 1) * p
    r2_0 = d.r_out_mm - inset0
    r1_L, r2_L = d.r_in_mm + insetL, d.r_out_mm - insetL

    L1 = _inward_spiral(d, n, theta_s, "inner", c)
    L2 = transform(_inward_spiral(d, n, 0.0, "inner", c), mirror=True, reverse=True)
    L2 = L2 + [Line(polar(r2_0, 0.0), polar(r2_0 + c, 0.0))]
    L3 = [Line(polar(r2_0 + c, 0.0), polar(r2_0, 0.0))] + _inward_spiral(d, n, 0.0, "outer", c)
    L4 = transform(_inward_spiral(d, n, theta_s, "outer", c), mirror=True, reverse=True)
    return CoilLayout(design=d, n_turns=n, layers=[L1, L2, L3, L4],
                      via_in=polar(r1_L + c, 0.0), via_out=polar(r2_0 + c, 0.0),
                      via_core=polar(r2_L - c, 0.0),
                      term_s=polar(r2_0, theta_s), term_e=polar(r2_0, -theta_s), clearance_c=c,
                      via_dia_mm=via_dia_mm, via_drill_mm=via_drill_mm)


def check_layout(lay: CoilLayout):
    """Basic self-checks: paths are continuous, and they start and end at the right places."""
    tol = 1e-6
    for i, path in enumerate(lay.layers):
        for a, b in zip(path[:-1], path[1:]):
            assert np.allclose(a.end, b.start, atol=tol), f"L{i + 1} not continuous"
    L1, L2, L3, L4 = lay.layers
    assert np.allclose(L1[0].start, lay.term_s) and np.allclose(L1[-1].end, lay.via_in)
    assert np.allclose(L2[0].start, lay.via_in) and np.allclose(L2[-1].end, lay.via_out)
    assert np.allclose(L3[0].start, lay.via_out) and np.allclose(L3[-1].end, lay.via_core)
    assert np.allclose(L4[0].start, lay.via_core) and np.allclose(L4[-1].end, lay.term_e)
    return True
