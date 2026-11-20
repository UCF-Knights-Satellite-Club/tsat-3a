#!/usr/bin/env python3
"""Copper spacing check that ignores nets (system Python with KiCad's pcbnew and scipy).

    python3 PCBMotorTest/check_spacing.py [board.kicad_pcb] [--space 0.15] [--dump violations.csv]

Why: every turn of a phase winding is the same net, so KiCad's DRC never checks the gap
between neighbouring turns. A turn-to-turn short would not show up in DRC, but it would
short turns of the coil. This script samples all copper on each layer (tracks, arcs, vias,
pads) and reports any two pieces closer than --space, unless they are the same piece of
copper within 1 mm along the trace or meet at a shared joint within 1 mm of it.
"""
from __future__ import annotations

import math
import sys

import numpy as np
import pcbnew
from scipy.spatial import cKDTree

STEP = 0.03     # mm between samples along a track
NEAR = 1.0      # mm: copper this close along the trace (or to a shared joint) is "connected"


def mm(v):
    return np.array([pcbnew.ToMM(v.x), pcbnew.ToMM(v.y)])


def sample_track(t):
    if t.GetClass() == "PCB_ARC":
        c = mm(t.GetCenter())
        r = pcbnew.ToMM(t.GetRadius())
        a0 = math.atan2(*(mm(t.GetStart()) - c)[::-1])
        am = math.atan2(*(mm(t.GetMid()) - c)[::-1])
        a1 = math.atan2(*(mm(t.GetEnd()) - c)[::-1])
        # choose the direction that passes through the mid point
        def ccw_span(a, b):
            return (b - a) % (2 * math.pi)
        if ccw_span(a0, am) <= ccw_span(a0, a1):
            span = ccw_span(a0, a1)
        else:
            span = -ccw_span(a1, a0)
        n = max(2, int(abs(span) * r / STEP) + 1)
        a = a0 + np.linspace(0, span, n)
        pts = c + r * np.c_[np.cos(a), np.sin(a)]
    else:
        p0, p1 = mm(t.GetStart()), mm(t.GetEnd())
        n = max(2, int(np.linalg.norm(p1 - p0) / STEP) + 1)
        pts = p0 + (p1 - p0) * np.linspace(0, 1, n)[:, None]
    s = np.r_[0, np.cumsum(np.linalg.norm(np.diff(pts, axis=0), axis=1))]
    return pts, s


def chain_tracks(tracks, blocked):
    """Join tracks that meet end to end (exactly two track ends at the point, and no via or pad
    there) into one item, with the distance along the joined trace. Curves drawn as many short
    segments are then one piece of copper. Otherwise segment 1 and segment 3 of a bend would look
    like separate copper that overlaps.
    tracks: list of (pts, along, half_width); returns the same with per-point half widths."""
    key = lambda e: (round(float(e[0]), 4), round(float(e[1]), 4))
    ends = {}
    for k, (pts, s, hw) in enumerate(tracks):
        ends.setdefault(key(pts[0]), []).append((k, 0))
        ends.setdefault(key(pts[-1]), []).append((k, 1))
    joinable = {e for e, lst in ends.items() if len(lst) == 2 and e not in blocked}
    used = [False] * len(tracks)
    out = []

    def walk(k, from_end):
        """Follow the chain from track k entered at end `from_end`."""
        pts_all, hw_all = [], []
        while True:
            used[k] = True
            pts, s, hw = tracks[k]
            seq = pts if from_end == 0 else pts[::-1]
            pts_all.append(seq if not pts_all else seq[1:])
            hw_all.append(np.full(len(seq) if len(hw_all) == 0 else len(seq) - 1, hw))
            far = key(seq[-1])
            if far not in joinable:
                break
            nxt = [(kk, ee) for kk, ee in ends[far] if kk != k]
            if not nxt or used[nxt[0][0]]:
                break
            k, from_end = nxt[0]
        P = np.vstack(pts_all)
        S = np.r_[0, np.cumsum(np.linalg.norm(np.diff(P, axis=0), axis=1))]
        return P, S, np.concatenate(hw_all)

    # start from chain ends (an end that isn't joinable), then sweep up closed loops
    for k, (pts, s, hw) in enumerate(tracks):
        if used[k]:
            continue
        for e_i, e in ((0, pts[0]), (1, pts[-1])):
            if key(e) not in joinable:
                out.append(walk(k, e_i))
                break
    for k in range(len(tracks)):
        if not used[k]:
            out.append(walk(k, 0))
    return out


def main():
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    path = args[0] if args else "PCBMotorTest/PCBMotorTest.kicad_pcb"
    space = float(sys.argv[sys.argv.index("--space") + 1]) if "--space" in sys.argv else 0.15
    dump = open(sys.argv[sys.argv.index("--dump") + 1], "w") if "--dump" in sys.argv else None
    if dump is not None:
        args = [a for a in args if a != sys.argv[sys.argv.index("--dump") + 1]]
        path = args[0] if args else path
        dump.write("layer,gap_mm,x1,y1,x2,y2\n")
    b = pcbnew.LoadBoard(path)
    layers = [b.GetLayerID(n) for n in ("F.Cu", "In1.Cu", "In2.Cu", "B.Cu")]
    total = 0
    for lid in layers:
        items = []      # (points, along, half_width (scalar or per point), endpoints)
        tracks, blocked = [], set()
        for t in b.GetTracks():
            if t.GetClass() == "PCB_VIA":
                c = mm(t.GetPosition())
                items.append((c[None], np.zeros(1), pcbnew.ToMM(t.GetWidth(lid)) / 2, [c]))
                blocked.add((round(float(c[0]), 4), round(float(c[1]), 4)))
            elif t.GetLayer() == lid:
                pts, s = sample_track(t)
                tracks.append((pts, s, pcbnew.ToMM(t.GetWidth()) / 2))
        for fp in b.GetFootprints():
            for p in fp.Pads():
                if p.IsOnLayer(lid) and p.GetSizeX() > 0:
                    c = mm(p.GetPosition())
                    # inscribed circle: KiCad's DRC already checks pads against other nets
                    r = 0.5 * min(pcbnew.ToMM(p.GetSizeX()), pcbnew.ToMM(p.GetSizeY()))
                    items.append((c[None], np.zeros(1), r, [c]))
                    blocked.add((round(float(c[0]), 4), round(float(c[1]), 4)))
        for pts, s, hw in chain_tracks(tracks, blocked):
            items.append((pts, s, hw, [pts[0], pts[-1]]))
        P = np.vstack([it[0] for it in items])
        S = np.concatenate([it[1] for it in items])
        HW = np.concatenate([np.broadcast_to(np.asarray(it[2], float), (len(it[0]),)) for it in items])
        ID = np.concatenate([np.full(len(it[0]), k) for k, it in enumerate(items)])
        # joints: item endpoints that touch another item (endpoint inside a via/pad counts)
        ends = [(k, e) for k, it in enumerate(items) for e in it[3]]
        etree = cKDTree(np.array([e for _, e in ends]))
        joints = {}
        for k, it in enumerate(items):
            for e in it[3]:
                for m in etree.query_ball_point(e, max(float(np.max(it[2])), 1e-3) + 1e-3):
                    j, ej = ends[m]
                    if j != k:
                        joints.setdefault((min(j, k), max(j, k)), []).append(0.5 * (e + ej))
        is_circle = np.concatenate([np.full(len(it[0]), len(it[0]) == 1) for it in items])
        hw_tr = HW[~is_circle].max() if (~is_circle).any() else 0.0
        tree = cKDTree(P)
        pairs = [tree.query_pairs(2 * hw_tr + space, output_type="ndarray")]
        for a in np.flatnonzero(is_circle):          # vias and pads: few, but large radius
            nb = np.array(tree.query_ball_point(P[a], HW[a] + max(hw_tr, HW[is_circle].max()) + space))
            nb = nb[nb != a]
            pairs.append(np.c_[np.full(len(nb), a), nb])
        pairs = np.vstack(pairs).astype(int)
        i, j = pairs[:, 0], pairs[:, 1]
        gap = np.linalg.norm(P[i] - P[j], axis=1) - HW[i] - HW[j]
        bad = gap < space - 1e-4
        i, j, gap = i[bad], j[bad], gap[bad]
        # along one track (or one chain of joined tracks), nearby points are the same copper. The
        # reach is twice the joint rule's radius: a bend excused within `near` of its joint on each
        # side spans 2 * near of trace.
        near = 2 * np.maximum(NEAR, HW[i] + HW[j] + space + STEP)
        same = (ID[i] == ID[j]) & (np.abs(S[i] - S[j]) <= near)
        i, j, gap = i[~same], j[~same], gap[~same]
        viol = []
        for a, c, g in zip(i, j, gap):
            key = (min(ID[a], ID[c]), max(ID[a], ID[c]))
            nr = max(NEAR, HW[a] + HW[c] + space + STEP)
            if ID[a] != ID[c] and key in joints and any(
                    np.linalg.norm(P[a] - J) <= nr + HW[a] and np.linalg.norm(P[c] - J) <= nr + HW[c]
                    for J in joints[key]):
                continue
            viol.append((g, P[a], P[c]))
        name = b.GetLayerName(lid)
        if dump is not None:
            for g, pa, pc in viol:
                dump.write(f"{name},{g:.4f},{pa[0]:.4f},{pa[1]:.4f},{pc[0]:.4f},{pc[1]:.4f}\n")
        print(f"{name:7s} {len(items):5d} items, {len(P):7d} samples: {len(viol)} spacing violations"
              + (f", worst gap {min(v[0] for v in viol):.3f} mm" if viol else ""))
        for g, pa, pc in sorted(viol, key=lambda v: v[0])[:10]:
            print(f"        gap {g:.3f} mm at ({pa[0]:.3f}, {pa[1]:.3f}) / ({pc[0]:.3f}, {pc[1]:.3f})")
        total += len(viol)
    print("OK" if total == 0 else f"{total} violations")
    return 0 if total == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
