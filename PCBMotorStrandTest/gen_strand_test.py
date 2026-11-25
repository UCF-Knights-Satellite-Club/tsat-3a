#!/usr/bin/env python3
"""Generate the solid and stranded PCB motor test stators (KiCad 10).

Run with the system Python that ships KiCad's `pcbnew` module (not the sim venv):

    python3 PCBMotorStrandTest/gen_strand_test.py            # both boards
    python3 PCBMotorStrandTest/gen_strand_test.py stranded   # one of them

What it does
1. Reads the high-momentum payload stator ("payload_high_momentum" in
   sim/results/pcbmotor_designs.json): 6 coils / 8 poles, 21-41 mm winding, 1.2 mm trace, on JLCPCB
   4-layer with 2 oz copper on every layer (bundle.STRAND_TEST_STACKUP).
2. Builds the coil copper with sim/pcbmotor/bundle.py: one 1.2 mm trace (solid) or three
   0.30 mm strands transposed at the layer vias (stranded). Everything else is the same.
3. Writes StrandTest_<variant>.kicad_pcb / .kicad_pro: six coils, the ring tracks that wire them
   into three phases (same scheme as the test stator, on both inner layers), a 6-pin header, holes, silkscreen,
   JLCPCB rules and stackup. On the stranded board coil 0 (A+) ends its strands on a 3-pad
   solder jumper J2 instead of a bus, so its strands can be measured one at a time.
4. Writes strandtest_<variant>_asbuilt.json (turns, track lengths per phase) for
   `sim/.venv/bin/python -m pcbmotor.strand_test`.

Re-running overwrites the boards. After you start editing one by hand in the KiCad GUI, either
stop using this script for it or change the parameters here and regenerate, not both.

Board coordinates: stator centre at (100, 100) mm on the sheet. KiCad's y axis points down, so the
math frame of bundle.py is flipped in y (header at the bottom of the screen).
"""
from __future__ import annotations

import json
import math
import pathlib
import sys

import pcbnew

HERE = pathlib.Path(__file__).resolve().parent
ROOT = HERE.parent
SIM = ROOT / "sim"
sys.dont_write_bytecode = True       # don't leave __pycache__ next to PCBMotorTest/gen_stator.py
sys.path.insert(0, str(SIM))
sys.path.insert(0, str(ROOT / "PCBMotorTest"))
import gen_stator as gs                                                         # noqa: E402
from gen_stator import P, layer_id                                              # noqa: E402
from pcbmotor.geometry import StatorDesign, RotorDesign, phase_map, coil_angles, OZ_TO_M  # noqa: E402
from pcbmotor.layout import Arc, Line, polar, transform, path_length           # noqa: E402
from pcbmotor.bundle import strand_test_coil, strand_test_design, check_bundle, STRAND_TEST        # noqa: E402

BOARD_X = (-50.0, 50.0)        # outline (math frame, mm from the stator centre)
BOARD_Y = (-52.0, 48.0)        # extra room at the bottom for the header
CORNER_R = 2.0
MOUNT = [(-46, 44), (46, 44), (46, -48), (-46, -48)]     # M3 corner holes
HUB_HOLES_R = 12.0             # four M3 holes for a bearing housing, plus the M3 centre hole
LEAD_W = 1.0                   # coil terminal leads and header tails (outer layers)
RING_W = 0.9                   # ring arcs (inner copper)
RING_PITCH = 1.1               # radial spacing of the ring arcs (a via fits between them)
RING_LAYERS = ("In1.Cu", "In2.Cu")   # the same arcs on both inner layers, in parallel through the vias
JUMPER_FP = ("Jumper", "SolderJumper-3_P1.3mm_Open_Pad1.0x1.5mm")
JUMPER_R = 42.3                # radius of the jumper pad centres (outside the magnet corners, r 41.0)
JUMPER_PAD_LEN = 1.5
MAGNET_CORNER_CLEAR = 41.5     # nothing raised (solder) inside this radius on F.Cu
HEADER_Y = -49.5
HEADER_X = [-6.35, -3.81, -1.27, 1.27, 3.81, 6.35]
PIN_NAMES = ["B1", "A2", "C2", "B2", "A1", "C1"]
COPPER_EDGE_MM = 0.4
# JLCPCB multilayer: 0.3 mm via drill, 0.15 mm trace/space at 2 oz copper
RULES = dict(track=0.15, clearance=0.15, drill=0.3, via=0.5, annular=0.13, hole_clear=0.25, hole_hole=0.5)


def load_design():
    j = json.loads((SIM / "results" / "pcbmotor_designs.json").read_text())["payload_high_momentum"]
    return strand_test_design(StatorDesign(**j["stator"])), RotorDesign(**j["rotor"])


class Builder(gs.Builder):
    def __init__(self, path, via_drill, via_dia):
        self.b = pcbnew.NewBoard(str(path))
        self.b.SetCopperLayerCount(4)
        self.nets = {}
        self.cu = ["F.Cu", "In1.Cu", "In2.Cu", "B.Cu"]
        self.via_drill, self.via_dia = via_drill, via_dia

    def via(self, xy, net):
        v = pcbnew.PCB_VIA(self.b)
        v.SetPosition(P(xy))
        v.SetDrill(pcbnew.FromMM(self.via_drill))
        v.SetWidth(pcbnew.FromMM(self.via_dia))
        v.SetNet(self.net(net))
        self.b.Add(v)


def rot(xy, phi):
    return polar(math.hypot(*xy), math.atan2(xy[1], xy[0]) + phi)


def build(variant):
    d, rotor = load_design()
    bc = strand_test_coil(d, variant)
    check_bundle(bc)
    n = bc.n_strands
    pm = phase_map(d, rotor.n_poles)
    phis = coil_angles(d)
    assert (d.n_coils, rotor.n_poles) == (6, 8) and all(s > 0 for _, s in pm), \
        "ring routing below is written for 6 coils / 8 poles"
    names = "ABC"
    board = HERE / f"StrandTest_{variant}.kicad_pcb"
    B = Builder(board, bc.via_drill_mm, bc.via_dia_mm)
    W = d.trace_mm
    beta = math.pi / d.n_coils
    jumper_coil = 0 if n > 1 else None

    # ---- strand jumper on coil 0's S end (stranded board only), placed first so that coil 0's
    # strands on L1 can start from its pads
    pad_of, pad_strand = {}, {}
    if jumper_coil is not None:
        a = phis[jumper_coil] - beta / 2
        jp = B.footprint(*JUMPER_FP, "J2", (0, 0), value="Strands A+")
        # pads along the tangent (KiCad angles are counter-clockwise as seen, which is the math frame)
        jp.SetOrientationDegrees(math.degrees(a + math.pi / 2))
        jp.SetPosition(P(polar(JUMPER_R, a)))
        for p in jp.Pads():
            p.SetNet(B.net("PHASE_A"))
        mm = lambda v: (pcbnew.ToMM(v.x) - gs.CX, -(pcbnew.ToMM(v.y) - gs.CY))
        centres = {p.GetNumber(): mm(p.GetPosition()) for p in jp.Pads()}
        for s in range(n):
            st = transform(bc.lanes[0][s], rotate=phis[jumper_coil])[0].start      # (r_bus, chi_s)
            num = min(centres, key=lambda q: math.dist(centres[q], st))
            assert math.dist(centres[num], polar(JUMPER_R, math.atan2(st[1], st[0]))) < 0.1, "strand misses its pad"
            assert (num == "2") == (s == n // 2)
            pad_of[s] = centres[num]
            pad_strand[num] = s
        assert JUMPER_R - JUMPER_PAD_LEN / 2 >= MAGNET_CORNER_CLEAR
        jp.Reference().SetVisible(False)
        jp.Value().SetVisible(False)
        B.text(polar(JUMPER_R + 2.2, a), "J2", "F.Silkscreen", size=0.8)

    # ---- coils: copper of coil 0 rotated to each coil position
    for k, (phi, (ph, _)) in enumerate(zip(phis, pm)):
        net = f"PHASE_{names[ph]}"
        for L in range(4):
            for j, path in enumerate(bc.lanes[L]):
                path = transform(path, rotate=phi)
                widths = bc.widths(path)
                if k == jumper_coil and L == 0:            # L1 lane j is strand j: start it at its pad
                    path = [Line(pad_of[j], tuple(path[0].end))] + path[1:]
                for seg, w in zip(path, widths):
                    if seg.length > 1e-6:
                        B.seg(seg, B.cu[L], w, net)
        for kind in ("in", "out", "core"):
            for v in bc.vias[kind]:
                B.via(rot(v, phi), net)
        if n > 1:      # buses, split at the terminal so the phase lead lands on an arc end
            r_b, a0, a1 = bc.bus_e
            B.path([Arc(r_b, a0 + phi, beta / 2 + phi), Arc(r_b, beta / 2 + phi, a1 + phi)], "B.Cu", W, net)
            if k != jumper_coil:
                r_b, a0, a1 = bc.bus_s
                B.path([Arc(r_b, a0 + phi, -beta / 2 + phi), Arc(r_b, -beta / 2 + phi, a1 + phi)], "F.Cu", W, net)

    # ---- ring wiring (same scheme as PCBMotorTest/gen_stator.py)
    th_s = -beta / 2
    vr = bc.via_dia_mm / 2
    r_min = max(bc.r_out_row + vr + d.space_mm + RING_W / 2,                 # clear of the out vias
                bc.r_bus + W / 2 + d.space_mm + vr,                         # ring vias clear of the buses
                JUMPER_R + JUMPER_PAD_LEN / 2 + d.space_mm + vr)            # same ring on both boards
    slot_r = [round(r_min + 0.05 + i * RING_PITCH, 3) for i in range(4)]

    def term(coil, end):
        """(angle, layer, radius) of a coil terminal: S on F.Cu, E on B.Cu."""
        a = phis[coil] + (th_s if end == "S" else -th_s)
        r0 = JUMPER_R if (end == "S" and coil == jumper_coil) else bc.r_bus
        return a, ("F.Cu" if end == "S" else "B.Cu"), r0

    def pin_angle(x, r):
        return -math.acos(x / r)

    pin_of = {nm: i for i, nm in enumerate(PIN_NAMES)}
    routes = [
        ("PHASE_A", (0, "E"), (3, "S"), True, 0),
        ("PHASE_C", (1, "E"), (4, "S"), True, 1),
        ("PHASE_B", (2, "E"), (5, "S"), False, 2),
        ("PHASE_B", (2, "S"), ("pin", "B1"), True, 3),
        ("PHASE_A", (3, "E"), ("pin", "A2"), True, 2),
        ("PHASE_C", (4, "E"), ("pin", "C2"), True, 0),
        ("PHASE_B", (5, "E"), ("pin", "B2"), False, 1),
        ("PHASE_A", (0, "S"), ("pin", "A1"), False, 0),
        ("PHASE_C", (1, "S"), ("pin", "C1"), False, 3),
    ]
    ring_len = {nm: {"outer_mm": 0.0, "inner_mm": 0.0, "vias": 0} for nm in ("PHASE_A", "PHASE_B", "PHASE_C")}
    for net, src, dst, ccw, slot in routes:
        R = slot_r[slot]
        a0, l0, r0 = term(*src)
        lead = Line(polar(r0, a0), polar(R, a0))
        B.path([lead], l0, LEAD_W, net)
        B.via(polar(R, a0), net)
        if dst[0] == "pin":
            x = HEADER_X[pin_of[dst[1]]]
            a1 = pin_angle(x, R)
            tail = [Line(polar(R, a1), (x, HEADER_Y))]
            l1 = "B.Cu"
        else:
            a1, l1, r1 = term(*dst)
            tail = [Line(polar(R, a1), polar(r1, a1))]
        span = (a1 - a0) % (2 * math.pi)
        a1 = a0 + span if ccw else a0 - (2 * math.pi - span)
        arc = Arc(R, a0, a1)
        for rl in RING_LAYERS:
            B.path([arc], rl, RING_W, net)
        B.via(polar(R, a1), net)
        B.path(tail, l1, LEAD_W, net)
        ring_len[net]["inner_mm"] += arc.length
        ring_len[net]["outer_mm"] += lead.length + path_length(tail)
        ring_len[net]["vias"] += 2

    # ---- header (bottom side, so nothing sticks up next to the rotor)
    hdr = B.footprint("Connector_PinHeader_2.54mm", "PinHeader_1x06_P2.54mm_Vertical", "J1",
                      (0, 0), value="Phases", back=True, rot=90)
    pads = sorted(hdr.Pads(), key=lambda p: int(p.GetNumber()))
    pxs = [p.GetPosition().x for p in pads]
    target = (HEADER_X[0] if pxs[0] < pxs[-1] else HEADER_X[-1], HEADER_Y)
    hdr.SetPosition(hdr.GetPosition() + (P(target) - pads[0].GetPosition()))
    for p, nm in zip(sorted(pads, key=lambda p: p.GetPosition().x), PIN_NAMES):
        p.SetNet(B.net(f"PHASE_{nm[0]}"))
    for x, nm in zip(HEADER_X, PIN_NAMES):
        B.text((x, HEADER_Y - 1.8), nm, "B.Silkscreen", size=0.8, mirror=True)
    hdr.Reference().SetVisible(False)
    hdr.Value().SetVisible(False)

    # ---- mechanical
    holes = [("H5", (0, 0), "Axle M3")] + [(f"H{i + 1}", xy, "M3") for i, xy in enumerate(MOUNT)]
    holes += [(f"H{6 + i}", polar(HUB_HOLES_R, math.radians(45 + 90 * i)), "Hub M3") for i in range(4)]
    for ref, xy, val in holes:
        fp = B.footprint("MountingHole", "MountingHole_3.2mm_M3", ref, xy, value=val)
        fp.Reference().SetVisible(False)
        fp.Value().SetVisible(False)

    # ---- outline: rounded rectangle
    (xl, xr), (yb, yt), rc = BOARD_X, BOARD_Y, CORNER_R
    B.shape_line((xl + rc, yt), (xr - rc, yt), "Edge.Cuts")
    B.shape_line((xr, yt - rc), (xr, yb + rc), "Edge.Cuts")
    B.shape_line((xr - rc, yb), (xl + rc, yb), "Edge.Cuts")
    B.shape_line((xl, yb + rc), (xl, yt - rc), "Edge.Cuts")
    for (cx, cy, a_mid) in [(xr - rc, yt - rc, 45), (xl + rc, yt - rc, 135),
                            (xl + rc, yb + rc, 225), (xr - rc, yb + rc, 315)]:
        am = math.radians(a_mid)
        q = math.radians(45)
        B.shape_arc((cx, cy), (cx + rc * math.cos(am - q), cy + rc * math.sin(am - q)),
                    (cx + rc * math.cos(am), cy + rc * math.sin(am)),
                    (cx + rc * math.cos(am + q), cy + rc * math.sin(am + q)), "Edge.Cuts")

    # ---- fab / silkscreen
    B.circle((0, 0), rotor.disc_od_mm / 2, "F.Fab", 0.1)
    for k in range(rotor.n_poles):                           # magnet blocks at rotor angle 0
        a = k * 2 * math.pi / rotor.n_poles
        hl, hw = rotor.block_len_mm / 2, rotor.block_w_mm / 2
        cs = [(rotor.magnet_r_mm + u * hl, v * hw) for u, v in ((-1, -1), (1, -1), (1, 1), (-1, 1))]
        cs = [rot(c, a) for c in cs]
        for c0, c1 in zip(cs, cs[1:] + cs[:1]):
            B.shape_line(c0, c1, "F.Fab", 0.05)
    for phi, (ph, sg) in zip(phis, pm):
        B.text(polar(48.0 if abs(math.cos(phi)) < 0.9 else 47.6, phi), f"{names[ph]}{'+' if sg > 0 else '-'}",
               "F.Silkscreen", size=1.0)
    label = "SOLID 1 x 1.20 mm" if n == 1 else f"STRANDED {n} x {bc.strand_mm:.2f} mm"
    B.text((-34, 44.6), "TSat-3A strand test v0.1", "F.Silkscreen", size=0.9)
    B.text((-34, 43.0), label, "F.Silkscreen", size=0.9)
    B.text((34, 44.6), f"6 coils / 8 poles, {bc.n_turns} turns x 4L", "F.Silkscreen", size=0.8)
    B.text((34, 43.0), "JLC 4L " + "/".join(f"{oz:g}" for oz in d.stack["oz"]) + " oz", "F.Silkscreen", size=0.8)
    B.text((-34, 44.6), "KSC TSat-3A strand test", "B.Silkscreen", size=0.9, mirror=True)
    B.text((-34, 43.0), label, "B.Silkscreen", size=0.9, mirror=True)
    B.text((34, -47.0), "phases A1-A2 B1-B2 C1-C2", "B.Silkscreen", size=0.8, mirror=True)

    # ---- design rules (JLCPCB 4-layer) and stackup
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
    note = write_stackup(board, d)
    fix_project(board, d, bc)

    out = dict(design=d.name, variant=variant, strands=n, strand_mm=bc.strand_mm, strand_gap_mm=bc.strand_gap_mm,
               bundle_mm=W, n_turns=bc.n_turns, turns_per_layer=bc.turns_per_layer(),
               lane_lengths_mm=bc.lane_lengths_mm(), strand_lanes=bc.strands,
               via_drill_mm=bc.via_drill_mm, via_dia_mm=bc.via_dia_mm,
               ring_track_mm=RING_W, lead_track_mm=LEAD_W, ring_layers=list(RING_LAYERS), ring_radii_mm=slot_r,
               ring=ring_len, board_mm=[BOARD_X[1] - BOARD_X[0], BOARD_Y[1] - BOARD_Y[0]], stackup=note,
               header_pins=dict(zip(range(1, 7), PIN_NAMES)),
               jumper_pad_strand=pad_strand or None,
               jumper=("J2 on coil 0 (A+) S end: pad 2 = middle strand and the phase lead, pads 1/3 = outer "
                       "strands. Bridge 1-2 and 2-3 with solder for normal use.") if jumper_coil is not None else None)
    (HERE / f"strandtest_{variant}_asbuilt.json").write_text(json.dumps(out, indent=2))
    print(f"{variant}: {bc.n_turns} turns, strand {bc.strand_mm:.3f} mm, ring radii {slot_r}")
    print("wrote", board)


def write_stackup(board, d):
    """Insert the JLCPCB 4-layer stackup (copper weights from the design's stackup) into the saved
    board. The thicknesses come from geometry.STACKUPS, which holds JLC's published all-2 oz
    7628 build since 2026-11-23 (before that, an assumed one)."""
    txt = board.read_text()
    if "(stackup" in txt:
        return "stackup already present"
    st = d.stack
    cu = [round(oz * OZ_TO_M * 1e3, 4) for oz in st["oz"]]
    z = st["z_mm"]
    diel = [round(z[i + 1] - z[i] - (cu[i] + cu[i + 1]) / 2, 4) for i in range(3)]
    kinds = ["prepreg", "core", "prepreg"]
    names = ["F.Cu", "In1.Cu", "In2.Cu", "B.Cu"]
    lines = ['\t\t(stackup',
             '\t\t\t(layer "F.SilkS" (type "Top Silk Screen"))',
             '\t\t\t(layer "F.Paste" (type "Top Solder Paste"))',
             '\t\t\t(layer "F.Mask" (type "Top Solder Mask") (thickness 0.01))']
    for i, nm in enumerate(names):
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
    return ("JLCPCB 4L 2 oz, 7628 build (published): copper " + " / ".join(f"{t * 1000:g}" for t in cu)
            + " um, dielectric " + " / ".join(f"{t:g}" for t in diel) + " mm")


def fix_project(board, d, bc):
    """Default net class: strand width and the coil space (KiCad keeps net classes in the .kicad_pro)."""
    pro = board.with_suffix(".kicad_pro")
    p = json.loads(pro.read_text())
    for c in p["net_settings"]["classes"]:
        if c["name"] == "Default":
            c.update(clearance=d.space_mm, track_width=bc.strand_mm, via_diameter=bc.via_dia_mm,
                     via_drill=bc.via_drill_mm)
    p["meta"]["filename"] = pro.name
    pro.write_text(json.dumps(p, indent=2))


if __name__ == "__main__":
    for v in (sys.argv[1:] or list(STRAND_TEST)):
        build(v)
