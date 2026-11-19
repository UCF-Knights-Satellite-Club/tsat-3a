#!/usr/bin/env python3
"""Generate the PCB motor test stator board (KiCad 10) from the simulation's design.

Run with the system Python that ships KiCad's `pcbnew` module (not the sim venv):

    python3 PCBMotorTest/gen_stator.py

What it does
1. Reads the recommended test stator ("test" in sim/results/pcbmotor_designs.json), so the
   board and the simulation use one design.
2. Builds the as-built coil copper with sim/pcbmotor/layout.py (spirals, vias, terminals).
3. Writes PCBMotorTest/PCBMotorTest.kicad_pcb and .kicad_pro: six coils, the ring tracks that
   wire them into three phases, a 6-pin header, mounting holes, silkscreen, OSH Park 4-layer
   design rules and stackup.
4. Writes PCBMotorTest/stator_asbuilt.json (turns, track lengths per phase) for the as-built
   model check in sim/ (`.venv/bin/python -m pcbmotor.asbuilt`).

Re-running overwrites the board. After you start editing it by hand in the KiCad GUI, either
stop using this script or change the parameters here and regenerate, not both.

Board coordinates: stator centre at (100, 100) mm on the sheet. KiCad's y axis points down,
so the math frame of layout.py is flipped in y (connector at the bottom of the screen).
"""
from __future__ import annotations

import json
import math
import pathlib
import sys

import pcbnew

HERE = pathlib.Path(__file__).resolve().parent
SIM = HERE.parent / "sim"
sys.path.insert(0, str(SIM))
from pcbmotor.geometry import (StatorDesign, RotorDesign, phase_map, coil_angles,  # noqa: E402
                               _OSH4_CU, _OSH4_DIEL)
from pcbmotor.layout import (coil_layout, check_layout, transform, Arc, Line, polar,  # noqa: E402
                             path_length, VIA_DIA_MM, VIA_DRILL_MM)

BOARD = HERE / "PCBMotorTest.kicad_pcb"
CX, CY = 100.0, 100.0          # stator centre on the sheet (mm)
BOARD_W, BOARD_TOP, BOARD_BOT = 50.0, 25.0, 26.5   # outline: x +-25, y from +25 (top) to -26.5
CORNER_R = 2.0                 # outline corner radius
MOUNT_XY = 21.0                # M3 holes at (+-21, +-21)
CENTRE_HOLE_MM = 3.2           # M3 clearance: fixed axle screw for the rotor bearing
RING_W = 0.25                  # ring / lead track width
RING_PITCH = 0.6               # radial spacing of ring tracks (fits a via between them)
RING_LAYER = "In2.Cu"          # inner layer used for the ring arcs (L3)
HEADER_Y = -23.0               # header pin row (math frame, mm from centre)
HEADER_X = [-6.35, -3.81, -1.27, 1.27, 3.81, 6.35]
COPPER_EDGE_MM = 0.4           # OSH Park asks for 15 mil copper to edge


# ---------------------------------------------------------------- helpers
def P(xy):
    """math-frame mm (y up, origin at stator centre) -> KiCad VECTOR2I."""
    return pcbnew.VECTOR2I_MM(CX + float(xy[0]), CY - float(xy[1]))


def layer_id(board, name):
    lid = board.GetLayerID(name)
    assert lid >= 0, f"unknown layer {name}"
    return lid


class Builder:
    def __init__(self):
        self.b = pcbnew.NewBoard(str(BOARD))
        self.b.SetCopperLayerCount(4)
        self.nets = {}
        self.cu = ["F.Cu", "In1.Cu", "In2.Cu", "B.Cu"]

    def net(self, name):
        if name not in self.nets:
            n = pcbnew.NETINFO_ITEM(self.b, name)
            self.b.Add(n)
            self.nets[name] = n
        return self.nets[name]

    def seg(self, s, layer, width, net):
        if isinstance(s, Arc):
            t = pcbnew.PCB_ARC(self.b)
            t.SetStart(P(s.start))
            t.SetMid(P(s.mid))
            t.SetEnd(P(s.end))
        else:
            t = pcbnew.PCB_TRACK(self.b)
            t.SetStart(P(s.start))
            t.SetEnd(P(s.end))
        t.SetWidth(pcbnew.FromMM(width))
        t.SetLayer(layer_id(self.b, layer))
        t.SetNet(self.net(net))
        self.b.Add(t)

    def path(self, segs, layer, width, net):
        for s in segs:
            if s.length > 1e-6:
                self.seg(s, layer, width, net)

    def via(self, xy, net):
        v = pcbnew.PCB_VIA(self.b)
        v.SetPosition(P(xy))
        v.SetDrill(pcbnew.FromMM(VIA_DRILL_MM))
        v.SetWidth(pcbnew.FromMM(VIA_DIA_MM))
        v.SetNet(self.net(net))
        self.b.Add(v)

    def shape_line(self, a, b, layer, width=0.1):
        s = pcbnew.PCB_SHAPE(self.b)
        s.SetShape(pcbnew.SHAPE_T_SEGMENT)
        s.SetStart(P(a))
        s.SetEnd(P(b))
        s.SetLayer(layer_id(self.b, layer))
        s.SetWidth(pcbnew.FromMM(width))
        self.b.Add(s)

    def shape_arc(self, centre, start, mid, end, layer, width=0.1):
        s = pcbnew.PCB_SHAPE(self.b)
        s.SetShape(pcbnew.SHAPE_T_ARC)
        s.SetArcGeometry(P(start), P(mid), P(end))
        s.SetLayer(layer_id(self.b, layer))
        s.SetWidth(pcbnew.FromMM(width))
        self.b.Add(s)

    def circle(self, centre, radius, layer, width=0.1):
        s = pcbnew.PCB_SHAPE(self.b)
        s.SetShape(pcbnew.SHAPE_T_CIRCLE)
        s.SetCenter(P(centre))
        s.SetEnd(P((centre[0] + radius, centre[1])))
        s.SetLayer(layer_id(self.b, layer))
        s.SetWidth(pcbnew.FromMM(width))
        self.b.Add(s)

    def text(self, xy, s, layer, size=1.0, mirror=False, angle=0.0):
        t = pcbnew.PCB_TEXT(self.b)
        t.SetText(s)
        t.SetPosition(P(xy))
        t.SetLayer(layer_id(self.b, layer))
        t.SetTextSize(pcbnew.VECTOR2I_MM(size, size))
        t.SetTextThickness(pcbnew.FromMM(size * 0.15))
        t.SetMirrored(mirror)
        if angle:
            t.SetTextAngle(pcbnew.EDA_ANGLE(angle, pcbnew.DEGREES_T))
        self.b.Add(t)

    def footprint(self, lib, name, ref, xy, value=None, back=False, rot=0.0):
        fp = pcbnew.FootprintLoad(f"/usr/share/kicad/footprints/{lib}.pretty", name)
        fp.SetReference(ref)
        fp.SetValue(value or name)
        fp.SetFPIDAsString(f"{lib}:{name}")
        self.b.Add(fp)
        if rot:
            fp.SetOrientationDegrees(rot)
        if back:
            fp.Flip(fp.GetPosition(), pcbnew.FLIP_DIRECTION_TOP_BOTTOM)
        fp.SetPosition(P(xy))
        return fp


def load_design():
    j = json.loads((SIM / "results" / "pcbmotor_designs.json").read_text())["test"]
    return StatorDesign(**j["stator"]), RotorDesign(**j["rotor"])


# ---------------------------------------------------------------- board
def main():
    d, rot = load_design()
    lay = coil_layout(d)
    check_layout(lay)
    pm = phase_map(d, rot.n_poles)
    phis = coil_angles(d)
    assert (d.n_coils, rot.n_poles) == (6, 8) and all(s > 0 for _, s in pm), \
        "ring routing below is written for the 6-coil / 8-pole test stator"
    names = "ABC"
    B = Builder()

    # ---- coils: copper of coil 0 rotated to each coil position
    for k, (phi, (ph, _)) in enumerate(zip(phis, pm)):
        net = f"PHASE_{names[ph]}"
        for li, path in enumerate(lay.layers):
            B.path(transform(path, rotate=phi), B.cu[li], d.trace_mm, net)
        for v in (lay.via_in, lay.via_out, lay.via_core):
            B.via(polar(math.hypot(*v), math.atan2(v[1], v[0]) + phi), net)

    # ---- ring wiring
    beta = math.pi / d.n_coils
    th_s = math.atan2(lay.term_s[1], lay.term_s[0])        # S terminal angle (L1), relative to coil
    r_term = math.hypot(*lay.term_s)
    r_v2 = math.hypot(*lay.via_out)
    r0 = r_v2 + VIA_DIA_MM / 2 + d.space_mm + RING_W / 2 + 0.03
    slot_r = [round(r0 + i * RING_PITCH, 3) for i in range(4)]
    deg = math.radians

    def term(coil, end):
        """(angle, layer) of a coil terminal: S on L1, E on L4."""
        a = phis[coil] + (th_s if end == "S" else -th_s)
        return a, ("F.Cu" if end == "S" else "B.Cu")

    def pin_angle(x, r):
        return -math.acos(x / r)            # point on radius r directly above the header pin

    # (net, from terminal, to, ccw, slot). "to" is a coil terminal or ("pin", index).
    # Slots were assigned by hand so that arcs sharing a radius never overlap in angle.
    pin_of = {"B1": 0, "A2": 1, "C2": 2, "B2": 3, "A1": 4, "C1": 5}
    routes = [
        ("PHASE_A", (0, "E"), (3, "S"), True, 0),          # A: coil 0 -> coil 3
        ("PHASE_C", (1, "E"), (4, "S"), True, 1),          # C: coil 1 -> coil 4
        ("PHASE_B", (2, "E"), (5, "S"), False, 2),         # B: coil 2 -> coil 5 (clockwise, via 0 deg)
        ("PHASE_B", (2, "S"), ("pin", "B1"), True, 3),
        ("PHASE_A", (3, "E"), ("pin", "A2"), True, 2),
        ("PHASE_C", (4, "E"), ("pin", "C2"), True, 0),
        ("PHASE_B", (5, "E"), ("pin", "B2"), False, 1),
        ("PHASE_A", (0, "S"), ("pin", "A1"), False, 0),
        ("PHASE_C", (1, "S"), ("pin", "C1"), False, 3),
    ]
    ring_len = {n: {"outer_mm": 0.0, "inner_mm": 0.0, "vias": 0} for n in ("PHASE_A", "PHASE_B", "PHASE_C")}
    for net, src, dst, ccw, slot in routes:
        R = slot_r[slot]
        a0, l0 = term(*src)
        lead = Line(polar(r_term, a0), polar(R, a0))
        B.path([lead], l0, d.trace_mm, net)     # coil trace width: it starts next to the outer turns
        B.via(polar(R, a0), net)
        if dst[0] == "pin":
            x = HEADER_X[pin_of[dst[1]]]
            a1 = pin_angle(x, R)
            tail = [Line(polar(R, a1), (x, HEADER_Y))]
            l1, w1 = "B.Cu", RING_W
        else:
            a1, l1 = term(*dst)
            tail = [Line(polar(R, a1), polar(r_term, a1))]
            w1 = d.trace_mm
        # unwrap a1 to the requested direction
        span = (a1 - a0) % (2 * math.pi)                 # counter-clockwise span in [0, 2 pi)
        a1 = a0 + span if ccw else a0 - (2 * math.pi - span)
        arc = Arc(R, a0, a1)
        B.path([arc], RING_LAYER, RING_W, net)
        B.via(polar(R, a1), net)
        B.path(tail, l1, w1, net)
        ring_len[net]["inner_mm"] += arc.length
        ring_len[net]["outer_mm"] += lead.length + path_length(tail)
        ring_len[net]["vias"] += 2

    # ---- header (bottom side, so nothing sticks up next to the rotor)
    hdr = B.footprint("Connector_PinHeader_2.54mm", "PinHeader_1x06_P2.54mm_Vertical", "J1",
                      (0, 0), value="Phases", back=True, rot=90)
    pads = sorted(hdr.Pads(), key=lambda p: int(p.GetNumber()))
    # move the footprint so that pads land on HEADER_X; pad order may run either way after the flip
    p1 = pads[0].GetPosition()
    pxs = [p.GetPosition().x for p in pads]
    left_to_right = pxs[0] < pxs[-1]
    target = (HEADER_X[0] if left_to_right else HEADER_X[-1], HEADER_Y)
    off = P(target) - p1
    hdr.SetPosition(hdr.GetPosition() + off)
    by_x = sorted(pads, key=lambda p: p.GetPosition().x)
    pin_names = ["B1", "A2", "C2", "B2", "A1", "C1"]
    for p, nm in zip(by_x, pin_names):
        p.SetNet(B.net(f"PHASE_{nm[0]}"))
    for x, nm in zip(HEADER_X, pin_names):
        B.text((x, HEADER_Y - 2.3), nm, "B.Silkscreen", size=0.9, mirror=True)
    hdr.Reference().SetVisible(False)
    hdr.Value().SetVisible(False)

    # ---- mechanical
    B.footprint("MountingHole", "MountingHole_3.2mm_M3", "H5", (0, 0), value="Axle M3")
    for i, (sx, sy) in enumerate([(-1, 1), (1, 1), (1, -1), (-1, -1)]):
        B.footprint("MountingHole", "MountingHole_3.2mm_M3", f"H{i + 1}", (sx * MOUNT_XY, sy * MOUNT_XY),
                    value="M3")
    for fp in B.b.GetFootprints():
        if fp.GetReference().startswith("H"):
            fp.Reference().SetVisible(False)
            fp.Value().SetVisible(False)

    # ---- outline: rounded rectangle
    xl, xr, yt, yb, rc = -BOARD_W / 2, BOARD_W / 2, BOARD_TOP, -BOARD_BOT, CORNER_R
    B.shape_line((xl + rc, yt), (xr - rc, yt), "Edge.Cuts")
    B.shape_line((xr, yt - rc), (xr, yb + rc), "Edge.Cuts")
    B.shape_line((xr - rc, yb), (xl + rc, yb), "Edge.Cuts")
    B.shape_line((xl, yb + rc), (xl, yt - rc), "Edge.Cuts")
    s2 = math.sqrt(0.5)
    for (cx, cy, a_mid) in [(xr - rc, yt - rc, 45), (xl + rc, yt - rc, 135),
                            (xl + rc, yb + rc, 225), (xr - rc, yb + rc, 315)]:
        am = deg(a_mid)
        B.shape_arc((cx, cy), (cx + rc * math.cos(am - deg(45)), cy + rc * math.sin(am - deg(45))),
                    (cx + rc * math.cos(am), cy + rc * math.sin(am)),
                    (cx + rc * math.cos(am + deg(45)), cy + rc * math.sin(am + deg(45))), "Edge.Cuts")

    # ---- silkscreen / fab
    B.circle((0, 0), rot.disc_od_mm / 2, "F.Fab", 0.1)                     # rotor disc outline
    for k in range(rot.n_poles):                                          # magnet positions at angle 0
        a = k * 2 * math.pi / rot.n_poles
        B.circle(polar(rot.magnet_r_mm, a), rot.magnet_d_mm / 2, "F.Fab", 0.05)
    for phi, (ph, sg) in zip(phis, pm):
        B.text(polar(23.3, phi), f"{names[ph]}{'+' if sg > 0 else '-'}", "F.Silkscreen", size=1.0)
    B.text((0, 23.6), "TSat-3A PCB motor test stator v0.1", "F.Silkscreen", size=0.9)
    B.text((-14.5, -24.6), "6 coils / 8 poles", "F.Silkscreen", size=0.8)
    B.text((14.5, -24.6), f"{lay.n_turns} turns x 4L", "F.Silkscreen", size=0.8)
    B.text((0, 23.6), "KSC TSat-3A  test stator  phases: A1-A2, B1-B2, C1-C2", "B.Silkscreen", size=0.8, mirror=True)

    # ---- design rules (OSH Park 4-layer) and stackup
    ds = B.b.GetDesignSettings()
    ds.m_TrackMinWidth = pcbnew.FromMM(0.127)
    ds.m_MinClearance = pcbnew.FromMM(0.127)
    ds.m_MinThroughDrill = pcbnew.FromMM(0.254)
    ds.m_ViasMinSize = pcbnew.FromMM(0.457)
    ds.m_ViasMinAnnularWidth = pcbnew.FromMM(0.1016)
    ds.m_CopperEdgeClearance = pcbnew.FromMM(COPPER_EDGE_MM)
    ds.m_HoleClearance = pcbnew.FromMM(0.2)
    ds.m_HoleToHoleMin = pcbnew.FromMM(0.25)
    pcbnew.SaveBoard(str(BOARD), B.b)
    stack_note = write_stackup(d)
    fix_project(d)

    out = dict(design=d.name, n_turn_rings=lay.n_turns, turns_per_layer=lay.turns_per_layer(),
               coil_path_mm=[path_length(p) for p in lay.layers], trace_mm=d.trace_mm,
               ring_track_mm=RING_W, ring_layer=RING_LAYER, ring_radii_mm=slot_r, ring=ring_len,
               board_mm=[BOARD_W, BOARD_TOP + BOARD_BOT], stackup=stack_note,
               header_pins=dict(zip(range(1, 7), pin_names)))
    (HERE / "stator_asbuilt.json").write_text(json.dumps(out, indent=2))
    print(json.dumps({k: out[k] for k in ("n_turn_rings", "turns_per_layer", "ring_radii_mm")}, indent=1))
    print("wrote", BOARD)


def write_stackup(d):
    """Insert OSH Park's published 4-layer stackup into the saved board (the KiCad 10 Python API
    doesn't expose the stackup list). Thicknesses are the same ones the sim uses
    (geometry._OSH4_CU / _OSH4_DIEL, from docs.oshpark.com/services/four-layer): FR408HR, 1.7 mil
    plated outer copper, 0.5 oz inner, 7.87 mil prepreg, 39 mil core."""
    txt = BOARD.read_text()
    if "(stackup" in txt:
        return "stackup already present"
    cu = ["F.Cu", "In1.Cu", "In2.Cu", "B.Cu"]
    diel = [("prepreg", _OSH4_DIEL[0]), ("core", _OSH4_DIEL[1]), ("prepreg", _OSH4_DIEL[2])]
    lines = ['\t\t(stackup',
             '\t\t\t(layer "F.SilkS" (type "Top Silk Screen") (thickness 0.0152))',
             '\t\t\t(layer "F.Paste" (type "Top Solder Paste"))',
             '\t\t\t(layer "F.Mask" (type "Top Solder Mask") (thickness 0.0152))']
    for i, name in enumerate(cu):
        lines.append(f'\t\t\t(layer "{name}" (type "copper") (thickness {_OSH4_CU[i]}))')
        if i < 3:
            kind, t = diel[i]
            lines.append(f'\t\t\t(layer "dielectric {i + 1}" (type "{kind}") (thickness {t}) '
                         f'(material "FR408HR") (epsilon_r 3.61))')
    lines += ['\t\t\t(layer "B.Mask" (type "Bottom Solder Mask") (thickness 0.0152))',
              '\t\t\t(layer "B.Paste" (type "Bottom Solder Paste"))',
              '\t\t\t(layer "B.SilkS" (type "Bottom Silk Screen") (thickness 0.0152))',
              '\t\t\t(copper_finish "ENIG")',
              '\t\t\t(dielectric_constraints no)',
              '\t\t)']
    assert "\t(setup\n" in txt
    BOARD.write_text(txt.replace("\t(setup\n", "\t(setup\n" + "\n".join(lines) + "\n", 1))
    return ("OSH Park 4L (published): copper " + " / ".join(f"{t * 1000:g}" for t in _OSH4_CU)
            + " um, dielectric " + " / ".join(f"{t:g}" for t in _OSH4_DIEL) + " mm")


def fix_project(d):
    """Default net class to the coil trace/space (KiCad keeps net classes in the .kicad_pro)."""
    pro = BOARD.with_suffix(".kicad_pro")
    p = json.loads(pro.read_text())
    for c in p["net_settings"]["classes"]:
        if c["name"] == "Default":
            c.update(clearance=d.space_mm, track_width=d.trace_mm, via_diameter=VIA_DIA_MM,
                     via_drill=VIA_DRILL_MM)
    p["meta"]["filename"] = pro.name
    pro.write_text(json.dumps(p, indent=2))


if __name__ == "__main__":
    main()
