"""Parametric PCB axial-flux stator and rotor geometry.

Everything here is plain numbers (mm). The same StatorDesign later drives the
KiCad footprint generator, so simulation and layout use one geometry.

Coil model: each coil is a stack of concentric "sector" loops (an annular
sector inset by one trace pitch per turn). This is the standard approximation
of a spiral; the real spiral differs by a fraction of one turn.
"""
from __future__ import annotations

from dataclasses import dataclass, field, asdict
import math
import numpy as np

OZ_TO_M = 34.79e-6  # 1 oz/ft^2 copper thickness

# OSH Park stackups (FR408, 1.6 mm). Layer z is the depth of each copper layer below the top
# layer, measured between copper mid-planes (the air gap is measured to the top layer).
# 4-layer values are OSH Park's published stackup (docs.oshpark.com/services/four-layer, checked
# 2026-11-18): 1.7 mil (0.0432 mm) outer copper "clad+plated", 7.87 mil (0.1999 mm) prepreg,
# 0.5 oz (0.0175 mm) inner copper, 39 mil (0.9906 mm) core. The finished outer copper is 1.24 oz
# equivalent, which is what sets its resistance. The 2- and 6-layer spacing is an assumption.
_OSH4_CU = (0.0432, 0.0175, 0.0175, 0.0432)
_OSH4_DIEL = (0.1999, 0.9906, 0.1999)


def _midplane_depths(cu, diel):
    z, out = 0.0, [0.0]
    for k, t in enumerate(diel):
        z += cu[k] / 2 + t + cu[k + 1] / 2
        out.append(round(z, 4))
    return out


OSHPARK_STACKUPS = {
    2: dict(z_mm=[0.0, 1.6], oz=[1.0, 1.0], min_trace_mm=0.1524, min_space_mm=0.1524),
    4: dict(z_mm=_midplane_depths(_OSH4_CU, _OSH4_DIEL), oz=[round(t / OZ_TO_M / 1e3, 3) for t in _OSH4_CU],
            min_trace_mm=0.127, min_space_mm=0.127),
    6: dict(z_mm=[0.0, 0.2, 0.7, 0.9, 1.4, 1.6], oz=[1.0, 0.5, 0.5, 0.5, 0.5, 1.0],
            min_trace_mm=0.127, min_space_mm=0.127),
}

# Named fab presets. OSH Park = quick test stator; JLCPCB = cheap integrated payload board.
# JLCPCB rules (capabilities page, 2026-11-17): outer 1 or 2 oz, inner 0.5/1/2 oz; min trace/space
# 0.09/0.09 mm at 1 oz, 0.15/0.15 mm at 2 oz; 1.6 mm standard. JLC does not publish prepreg/core
# thicknesses for JLC04161H-7628 / JLC06161H-2116 on that page, so the z positions below are an
# ASSUMED 1.6 mm build (thin prepreg under the outer layers, thick core in the middle, like the
# OSH Park stackups). min_trace is the worst case over all layers of that stackup.
STACKUPS = {
    "osh2": dict(fab="OSH Park", **OSHPARK_STACKUPS[2]),
    "osh4": dict(fab="OSH Park", **OSHPARK_STACKUPS[4]),
    "osh6": dict(fab="OSH Park", **OSHPARK_STACKUPS[6]),
    "jlc4_1oz": dict(fab="JLCPCB", z_mm=[0.0, 0.21, 1.39, 1.6], oz=[1.0, 1.0, 1.0, 1.0],
                     min_trace_mm=0.09, min_space_mm=0.09),
    "jlc6_1oz": dict(fab="JLCPCB", z_mm=[0.0, 0.2, 0.7, 0.9, 1.4, 1.6], oz=[1.0] * 6,
                     min_trace_mm=0.09, min_space_mm=0.09),
    "jlc4_2oz_out": dict(fab="JLCPCB", z_mm=[0.0, 0.21, 1.39, 1.6], oz=[2.0, 1.0, 1.0, 2.0],
                         min_trace_mm=0.15, min_space_mm=0.15),
}


@dataclass
class StatorDesign:
    name: str
    layers: int = 4
    n_coils: int = 6              # must be a multiple of 3
    r_in_mm: float = 6.0          # inner radius of the winding area
    r_out_mm: float = 19.0        # outer radius of the winding area
    trace_mm: float = 0.15
    space_mm: float = 0.15
    coil_gap_mm: float = 0.3      # copper clearance between neighbouring coils
    min_core_mm: float = 0.6      # stop adding turns when the inside gets this narrow
    board_od_mm: float = 42.0
    # Coils in a phase are wired in series (True) or parallel (False).
    series: bool = True
    # Named stackup from STACKUPS; None = OSH Park with `layers` layers.
    stackup: str | None = None

    def __post_init__(self):
        if self.stackup is not None:
            self.layers = len(STACKUPS[self.stackup]["z_mm"])

    @property
    def stack(self):
        if self.stackup is not None:
            return STACKUPS[self.stackup]
        return OSHPARK_STACKUPS[self.layers]

    @property
    def fab(self):
        return self.stack.get("fab", "OSH Park")

    @property
    def pitch_mm(self):
        return self.trace_mm + self.space_mm

    def check_rules(self):
        s = self.stack
        assert self.n_coils % 3 == 0, "n_coils must be a multiple of 3"
        assert self.trace_mm >= s["min_trace_mm"] - 1e-9, f"trace below {self.fab} minimum"
        assert self.space_mm >= s["min_space_mm"] - 1e-9, f"space below {self.fab} minimum"


@dataclass
class RotorDesign:
    n_poles: int = 8
    magnet_shape: str = "cyl"     # "cyl" (diameter magnet_d_mm) or "block"
    magnet_d_mm: float = 6.0      # cylinder diameter
    block_len_mm: float = 10.0    # block: radial length
    block_w_mm: float = 5.0       # block: tangential width
    magnet_h_mm: float = 2.0      # thickness (axial)
    magnet_r_mm: float = 12.5     # radius of magnet centres
    br_T: float = 1.32            # N42 ~1.3 T, N52 ~1.45 T
    airgap_mm: float = 0.5        # magnet face to top copper surface
    back_iron: bool = True        # steel disc behind the magnets
    back_iron_t_mm: float = 1.5
    disc_od_mm: float = 40.0      # flywheel / rotor disc diameter
    disc_t_mm: float = 2.0        # non-magnetic disc (brass/steel/printed) thickness
    disc_density: float = 7850.0  # kg/m^3 (steel)

    def magnet_mass_kg(self):
        if self.magnet_shape == "block":
            v = (self.block_len_mm * self.block_w_mm * self.magnet_h_mm) / 1e9
        else:
            v = math.pi * (self.magnet_d_mm / 2e3) ** 2 * (self.magnet_h_mm / 1e3)
        return self.n_poles * v * 7500.0

    def inertia_kgm2(self):
        r = self.disc_od_mm / 2e3
        t = self.disc_t_mm / 1e3 + (self.back_iron_t_mm / 1e3 if self.back_iron else 0.0)
        m_disc = self.disc_density * math.pi * r * r * t
        i_disc = 0.5 * m_disc * r * r
        m_mag = self.magnet_mass_kg()
        i_mag = m_mag * (self.magnet_r_mm / 1e3) ** 2
        return i_disc + i_mag, m_disc + m_mag


def _sector_loop(r1, r2, a_half, n_arc=24):
    """Closed polygon of an annular sector centred on angle 0 (mm, radians)."""
    a = np.linspace(-a_half, a_half, n_arc)
    outer = np.c_[r2 * np.cos(a), r2 * np.sin(a)]
    inner = np.c_[r1 * np.cos(a[::-1]), r1 * np.sin(a[::-1])]
    return np.vstack([outer, inner, outer[:1]])


def coil_turns(d: StatorDesign):
    """Return a list of closed polygons (mm), outermost turn first, for one coil
    centred at angle 0. Each turn is inset by one pitch on every side."""
    p = d.pitch_mm
    half_pitch_ang = math.pi / d.n_coils
    turns = []
    k = 0
    while True:
        inset = d.coil_gap_mm / 2 + d.trace_mm / 2 + k * p
        r1 = d.r_in_mm + inset
        r2 = d.r_out_mm - inset
        # angular inset: keep a constant perpendicular distance from the sector edge
        # at the inner radius (the tightest point)
        a_half = half_pitch_ang - math.asin(min(1.0, inset / r1))
        inner_width = 2 * r1 * math.sin(a_half) if a_half > 0 else -1
        if r2 - r1 < d.min_core_mm or inner_width < d.min_core_mm:
            break
        turns.append(_sector_loop(r1, r2, a_half))
        k += 1
    return turns


def coil_angles(d: StatorDesign):
    return np.arange(d.n_coils) * 2 * math.pi / d.n_coils


def phase_map(d: StatorDesign, n_poles: int):
    """Star-of-slots phase assignment. Returns list of (phase 0..2, sign ±1) per coil."""
    pp = n_poles // 2
    out = []
    for phi in coil_angles(d):
        e = (pp * phi) % (2 * math.pi)
        sector = int(round(e / (math.pi / 3))) % 6
        # sectors: 0:A+ 1:C- 2:B+ 3:A- 4:C+ 5:B-
        table = [(0, 1), (2, -1), (1, 1), (0, -1), (2, 1), (1, -1)]
        out.append(table[sector])
    counts = [sum(1 for ph, _ in out if ph == k) for k in range(3)]
    if len(set(counts)) != 1:
        raise ValueError(f"{d.n_coils} coils / {n_poles} poles is not a balanced 3-phase winding: {counts}")
    return out


def summary(d: StatorDesign, r: RotorDesign | None = None):
    out = {"stator": asdict(d), "turns_per_layer_per_coil": len(coil_turns(d))}
    if r is not None:
        out["rotor"] = asdict(r)
    return out
