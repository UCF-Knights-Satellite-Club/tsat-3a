#!/usr/bin/env python3
"""Generate the wave-winding test stators (KiCad 10): solid, 2 strands swapped, 2 strands fixed.

Run with the system Python that ships KiCad's `pcbnew` module (not the sim venv):

    python3 PCBMotorWaveTest/gen_wave_test.py            # all three boards
    python3 PCBMotorWaveTest/gen_wave_test.py swap       # one of them

The copper comes from sim/pcbmotor/wave_layout.py (the same code the sim checks and models):
conductors, strand lanes, vias, loop breaks, jumpers and phase pads. This script only draws it:
- every polyline is simplified to within SIMPLIFY_MM, using the narrowest width of the points it merges;
- vias, the six phase pads (THT wire pads), holes, outline, silkscreen, the magnets on F.Fab, and the
  JLCPCB rules and stackup are added.
It also writes wavetest_<variant>_asbuilt.json with the layout summary and the sim's checks.

Re-running overwrites the boards. Once you edit one by hand in the KiCad GUI, either stop using this
script for it, or change the parameters in wave_layout.py and regenerate. Don't do both.

Board coordinates: stator centre at (100, 100) mm on the sheet. KiCad's y axis points down, so the
layout's math frame is flipped in y.
"""
from __future__ import annotations

import json
import math
import pathlib
import sys

import numpy as np
import pcbnew

HERE = pathlib.Path(__file__).resolve().parent
ROOT = HERE.parent
SIM = ROOT / "sim"
sys.dont_write_bytecode = True
sys.path.insert(0, str(SIM))
sys.path.insert(0, str(ROOT / "PCBMotorTest"))
import gen_stator as gs                                                    # noqa: E402
from gen_stator import P                                                   # noqa: E402
from pcbmotor import wave_layout as wl                                     # noqa: E402
from pcbmotor.geometry import STACKUPS                                     # noqa: E402

STACKUP = "jlc4_2oz"
SIMPLIFY_MM = 0.005
EDGE_MARGIN = 2.0          # board edge beyond the phase pads' copper
CORNER_R = 3.0
HUB_HOLES_R = 11.5         # four M3 holes for a bearing housing (clear of the 14.5 mm via ring)
PAD_FP = ("TestPoint", "TestPoint_THTPad_D2.0mm_Drill1.0mm")
RULES = dict(track=0.15, clearance=0.15, drill=0.3, via=0.5, annular=0.13, hole_clear=0.25, hole_hole=0.5)
COPPER_EDGE_MM = 0.4
ROTOR = dict(n_poles=8, block_len=19.05, block_w=12.7, magnet_r=31.0, disc_od=84.0)


class Builder(gs.Builder):
    def __init__(self, path, via_drill, via_dia):
        self.b = pcbnew.NewBoard(str(path))
        self.b.SetCopperLayerCount(4)
        self.nets = {}
        self.cu = list(wl.LAYERS)
        self.via_drill, self.via_dia = via_drill, via_dia

    def via(self, xy, net):
        v = pcbnew.PCB_VIA(self.b)
        v.SetPosition(P(xy))
        v.SetDrill(pcbnew.FromMM(self.via_drill))
        v.SetWidth(pcbnew.FromMM(self.via_dia))
        v.SetNet(self.net(net))
        self.b.Add(v)

    def track(self, a, b, layer, width, net):
        t = pcbnew.PCB_TRACK(self.b)
        t.SetStart(P(a))
        t.SetEnd(P(b))
        t.SetWidth(pcbnew.FromMM(width))
        t.SetLayer(gs.layer_id(self.b, layer))
        t.SetNet(self.net(net))
        self.b.Add(t)


def simplify(pts, w, eps=SIMPLIFY_MM, pinned=None):
    """Douglas-Peucker on the centreline; each kept segment gets the narrowest width it replaces.
    Vertices where `pinned` is True (another piece ends there) are always kept, so that piece ends
    on a track end rather than part-way along one."""
    keep = np.zeros(len(pts), bool) if pinned is None else pinned.copy()
    keep[[0, -1]] = True
    stack = [(0, len(pts) - 1)]
    while stack:
        i, j = stack.pop()
        if j <= i + 1:
            continue
        a, b = pts[i], pts[j]
        ab = b - a
        L = np.linalg.norm(ab)
        q = pts[i + 1:j]
        if L < 1e-12:
            d = np.linalg.norm(q - a, axis=1)
        else:
            d = np.abs(ab[0] * (q[:, 1] - a[1]) - ab[1] * (q[:, 0] - a[0])) / L
        k = int(np.argmax(d))
        if d[k] > eps:
            m = i + 1 + k
            keep[m] = True
            stack += [(i, m), (m, j)]
    idx = np.nonzero(keep)[0]
    segs = []
    for i, j in zip(idx[:-1], idx[1:]):
        # a kept segment may sit up to eps off the true centreline, so narrow it by 2 eps: its edges
        # then stay inside the original copper and no gap gets smaller
        shave = 2 * eps if j > i + 1 else 0.0
        segs.append((pts[i], pts[j], float(w[i:j + 1].min()) - shave))
    return segs


def build(name):
    p = wl.variant(name)
    wb = wl.build(p)
    _, errs = wl.series_path(wb)
    con = wl.check_connectivity(wb)
    sp = wl.check_spacing(wb)
    assert not errs and con["ok"] and not sp, (errs, con, sp[:5])
    board = HERE / f"WaveTest_{name}.kicad_pcb"
    B = Builder(board, p.via_drill, p.via_dia)

    n_seg = 0
    end_keys = {}
    for c in wb.copper:
        for e in (c.pts[0], c.pts[-1]):
            end_keys.setdefault((c.layer, round(float(e[0]), 4), round(float(e[1]), 4)), 0)
            end_keys[(c.layer, round(float(e[0]), 4), round(float(e[1]), 4))] += 1
    for c in wb.copper:
        pinned = np.array([(c.layer, round(float(x), 4), round(float(y), 4)) in end_keys for x, y in c.pts])
        for a, b, w in simplify(c.pts, c.w, pinned=pinned):
            if np.linalg.norm(b - a) > 1e-6:
                B.track(tuple(a), tuple(b), wl.LAYERS[c.layer], w, c.net)
                n_seg += 1
    for v in wb.vias:
        B.via(v.xy, v.net)
    for xy, nm, net in wb.pads:
        fp = B.footprint(*PAD_FP, nm, xy, value=nm)
        for pad in fp.Pads():
            pad.SetNet(B.net(net))
        fp.Reference().SetVisible(False)
        fp.Value().SetVisible(False)
        a = math.atan2(xy[1], xy[0])
        lab = wl._polar(np.array([math.hypot(*xy) + 2.4]), np.array([a]))[0]
        B.text(tuple(lab), nm, "B.Silkscreen", size=1.0, mirror=True)
        B.text(tuple(lab), nm, "F.Silkscreen", size=1.0)

    # ---- outline: rounded square just outside the phase pads
    h = wb.info["r_pad"] + 1.0 + EDGE_MARGIN + 2.6
    rc = CORNER_R
    for a0, a1 in (((-h + rc, h), (h - rc, h)), ((h, h - rc), (h, -h + rc)),
                   ((h - rc, -h), (-h + rc, -h)), ((-h, -h + rc), (-h, h - rc))):
        B.shape_line(a0, a1, "Edge.Cuts")
    for cx, cy, am in ((h - rc, h - rc, 45), (-h + rc, h - rc, 135), (-h + rc, -h + rc, 225), (h - rc, -h + rc, 315)):
        m, q = math.radians(am), math.radians(45)
        B.shape_arc((cx, cy), (cx + rc * math.cos(m - q), cy + rc * math.sin(m - q)),
                    (cx + rc * math.cos(m), cy + rc * math.sin(m)),
                    (cx + rc * math.cos(m + q), cy + rc * math.sin(m + q)), "Edge.Cuts")

    # ---- holes: centre axle, hub, corners
    corner = h - 5.0
    holes = [("H1", (0, 0), "Axle M3")]
    holes += [(f"H{2 + i}", wl._polar(np.array([HUB_HOLES_R]), np.array([math.radians(45 + 90 * i)]))[0], "Hub M3")
              for i in range(4)]
    holes += [(f"H{6 + i}", (sx * corner, sy * corner), "M3") for i, (sx, sy) in enumerate(((1, 1), (-1, 1), (-1, -1), (1, -1)))]
    for ref, xy, val in holes:
        fp = B.footprint("MountingHole", "MountingHole_3.2mm_M3", ref, tuple(xy), value=val)
        fp.Reference().SetVisible(False)
        fp.Value().SetVisible(False)

    # ---- fab: rotor disc and magnets (rotor angle 0)
    B.circle((0, 0), ROTOR["disc_od"] / 2, "F.Fab", 0.1)
    for kk in range(ROTOR["n_poles"]):
        a = kk * 2 * math.pi / ROTOR["n_poles"]
        hl, hw = ROTOR["block_len"] / 2, ROTOR["block_w"] / 2
        cs = [(ROTOR["magnet_r"] + u * hl, v * hw) for u, v in ((-1, -1), (1, -1), (1, 1), (-1, 1))]
        cs = [(x * math.cos(a) - y * math.sin(a), x * math.sin(a) + y * math.cos(a)) for x, y in cs]
        for c0, c1 in zip(cs, cs[1:] + cs[:1]):
            B.shape_line(c0, c1, "F.Fab", 0.05)

    # ---- silkscreen
    k = p.strands
    label = ("SOLID 1.2 mm" if k == 1 else f"{k} STRANDS, {'SWAPPED' if p.swap else 'FIXED'} AT VIAS")
    tx = -h + 22
    B.text((tx, h - 3.0), "KSC TSat-3A wave test v0.1", "F.Silkscreen", size=1.2)
    B.text((tx, h - 5.0), label, "F.Silkscreen", size=1.2)
    B.text((-tx, h - 3.0), "48 cond/layer, 8 poles, 2 wave pairs", "F.Silkscreen", size=1.0)
    B.text((-tx, h - 5.0), "JLC 4L 2/2/2/2 oz", "F.Silkscreen", size=1.0)
    B.text((tx, -h + 3.0), "KSC TSat-3A wave test v0.1", "B.Silkscreen", size=1.2, mirror=True)
    B.text((tx, -h + 5.0), label, "B.Silkscreen", size=1.2, mirror=True)
    B.text((-tx, -h + 3.0), "pads A1-A2 B1-B2 C1-C2", "B.Silkscreen", size=1.0, mirror=True)

    # ---- rules, stackup, project
    ds = B.b.GetDesignSettings()
    ds.m_TrackMinWidth = pcbnew.FromMM(RULES["track"])
    ds.m_MinClearance = pcbnew.FromMM(RULES["clearance"])
    ds.m_MinThroughDrill = pcbnew.FromMM(RULES["drill"])
    ds.m_ViasMinSize = pcbnew.FromMM(RULES["via"])
    ds.m_ViasMinAnnularWidth = pcbnew.FromMM(RULES["annular"])
    ds.m_CopperEdgeClearance = pcbnew.FromMM(COPPER_EDGE_MM)
    ds.m_HoleClearance = pcbnew.FromMM(RULES["hole_clear"])
    ds.m_HoleToHoleMin = pcbnew.FromMM(RULES["hole_hole"])
    pcbnew.SaveBoard(str(board), B.b)
    note = write_stackup(board)
    fix_project(board, p)

    out = dict(design=p.name, variant=name, strands=k, swap=p.swap, cap_mm=p.cap_mm, stackup=note,
               board_mm=round(2 * h, 1), segments=n_seg, vias=len(wb.vias), pads=[nm for _, nm, _ in wb.pads],
               radii_mm=dict(p1_vias=[p.p1_in_via, p.p1_out_via], p2_vias=[p.p2_in_via, p.p2_out_via],
                             p1_radial=[p.p1_in_rad, p.p1_out_rad], p2_radial=[p.r_a, p.r_b],
                             terminal_vias=round(wb.info["r_term"], 2), jumper_rings=[round(r, 2) for r in wb.info["r_rings"]],
                             pads=round(wb.info["r_pad"], 2)),
               jumpers=wb.info["jumpers"],
               chain={ph: [dict(pair=it["b"]["pair"], j=it["b"]["j"], angle_deg=round(math.degrees(it["b"]["angle"]) % 360, 2),
                                start=it["start"], end=it["end"]) for it in seq] for ph, seq in wb.chain.items()},
               checks=dict(series_errors=errs, connectivity=con, spacing_violations=len(sp)))
    (HERE / f"wavetest_{name}_asbuilt.json").write_text(json.dumps(out, indent=2))
    print(f"{name}: {n_seg} track segments, {len(wb.vias)} vias, board {2 * h:.1f} mm, wrote {board.name}")


def write_stackup(board):
    """Insert the JLCPCB 4-layer stackup (copper weights from the sim's stackup; JLC doesn't publish
    the dielectric thicknesses, so they come from the layer depths the sim assumes)."""
    txt = board.read_text()
    if "(stackup" in txt:
        return "stackup already present"
    st = STACKUPS[STACKUP]
    cu = [round(oz * 0.035, 4) for oz in st["oz"]]
    z = st["z_mm"]
    diel = [round(z[i + 1] - z[i] - (cu[i] + cu[i + 1]) / 2, 4) for i in range(3)]
    kinds = ["prepreg", "core", "prepreg"]
    lines = ['\t\t(stackup',
             '\t\t\t(layer "F.SilkS" (type "Top Silk Screen"))',
             '\t\t\t(layer "F.Paste" (type "Top Solder Paste"))',
             '\t\t\t(layer "F.Mask" (type "Top Solder Mask") (thickness 0.01))']
    for i, nm in enumerate(wl.LAYERS):
        lines.append(f'\t\t\t(layer "{nm}" (type "copper") (thickness {cu[i]}))')
        if i < 3:
            lines.append(f'\t\t\t(layer "dielectric {i + 1}" (type "{kinds[i]}") (thickness {diel[i]}) '
                         f'(material "FR4") (epsilon_r 4.4))')
    lines += ['\t\t\t(layer "B.Mask" (type "Bottom Solder Mask") (thickness 0.01))',
              '\t\t\t(layer "B.Paste" (type "Bottom Solder Paste"))',
              '\t\t\t(layer "B.SilkS" (type "Bottom Silk Screen"))',
              '\t\t\t(copper_finish "HAL lead-free")',
              '\t\t\t(dielectric_constraints no)',
              '\t\t)']
    assert "\t(setup\n" in txt
    board.write_text(txt.replace("\t(setup\n", "\t(setup\n" + "\n".join(lines) + "\n", 1))
    return ("JLCPCB 4L (assumed thicknesses): copper " + " / ".join(f"{t * 1000:g}" for t in cu)
            + " um, dielectric " + " / ".join(f"{t:g}" for t in diel) + " mm")


def fix_project(board, p):
    pro = board.with_suffix(".kicad_pro")
    j = json.loads(pro.read_text())
    for c in j["net_settings"]["classes"]:
        if c["name"] == "Default":
            c.update(clearance=p.space_mm, track_width=p.min_trace_mm, via_diameter=p.via_dia, via_drill=p.via_drill)
    j["meta"]["filename"] = pro.name
    pro.write_text(json.dumps(j, indent=2))


if __name__ == "__main__":
    for v in (sys.argv[1:] or list(wl.VARIANTS)):
        build(v)
