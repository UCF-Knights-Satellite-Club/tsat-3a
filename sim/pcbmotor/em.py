"""Electrical and magnetic model of a PCB axial-flux motor.

- Resistance from the exact turn geometry and OSH Park copper weights.
- Torque and back-EMF from magpylib: analytic magnet fields evaluated on every
  trace segment, Lorentz force summed into torque. No mesh, no iron in the
  stator. Rotor back iron is modelled with the method of images (ideal steel
  backing ~ a magnet of twice the thickness).
- Inductance from a Neumann-integral estimate (order of magnitude; it sets the
  PWM frequency, not the torque).
"""
from __future__ import annotations

from dataclasses import dataclass, asdict
import math
import numpy as np
import magpylib as magpy
from scipy.spatial.transform import Rotation

from .geometry import (StatorDesign, RotorDesign, coil_turns, coil_angles, phase_map, OZ_TO_M)

RHO_CU_20C = 1.72e-8      # ohm*m
ALPHA_CU = 0.00393        # 1/K
MU0 = 4e-7 * math.pi


@dataclass
class MotorResult:
    name: str
    turns_per_layer: int
    coils_per_phase: int
    R_phase_ohm: float          # at 20 C, one phase (star)
    L_phase_uH: float
    Kt_Nm_per_A: float          # mean torque per amp of PEAK phase current, sinusoidal (FOC) drive
    torque_ripple_pct: float    # peak-to-peak / mean under ideal sinusoidal currents
    Ke_phase_V_per_rad_s: float  # peak phase back-EMF per rad/s (= 2/3 Kt)
    winding_ok: bool
    rotor_inertia_kgm2: float
    rotor_mass_kg: float
    copper_mass_g: float

    def as_dict(self):
        return asdict(self)


# ---------------------------------------------------------------- resistance
def _poly_length_mm(poly):
    return float(np.sum(np.linalg.norm(np.diff(poly, axis=0), axis=1)))


def phase_resistance(d: StatorDesign, temp_c=20.0):
    turns = coil_turns(d)
    rho = RHO_CU_20C * (1 + ALPHA_CU * (temp_c - 20.0))
    r_coil = 0.0
    for oz in d.stack["oz"]:
        t = oz * OZ_TO_M
        for poly in turns:
            r_coil += rho * (_poly_length_mm(poly) / 1e3) / ((d.trace_mm / 1e3) * t)
    # 10% allowance for vias, layer jumps and coil-to-coil links
    r_coil *= 1.10
    n = d.n_coils // 3
    return r_coil * n if d.series else r_coil / n


def copper_mass_g(d: StatorDesign):
    turns = coil_turns(d)
    vol = 0.0
    for oz in d.stack["oz"]:
        t = oz * OZ_TO_M
        vol += sum(_poly_length_mm(p) / 1e3 for p in turns) * (d.trace_mm / 1e3) * t
    return vol * 8960 * 1e3 * d.n_coils


# ---------------------------------------------------------------- inductance
def _neumann(p1, p2):
    """Mutual inductance of two closed polylines (m) by the Neumann formula."""
    a = (p1[:-1] + p1[1:]) / 2
    da = np.diff(p1, axis=0)
    b = (p2[:-1] + p2[1:]) / 2
    db = np.diff(p2, axis=0)
    r = np.linalg.norm(a[:, None, :] - b[None, :, :], axis=2)
    return MU0 / (4 * math.pi) * float(np.sum((da @ db.T) / r))


def phase_inductance(d: StatorDesign):
    turns = coil_turns(d)
    n = len(turns)
    mean = turns[n // 2]
    loop = np.c_[mean / 1e3, np.zeros(len(mean))]
    build = n * d.pitch_mm / 1e3
    zs = np.array(d.stack["z_mm"]) / 1e3
    L = 0.0
    for i, zi in enumerate(zs):
        for j, zj in enumerate(zs):
            dz = abs(zi - zj)
            if i == j:
                dz = 0.2235 * (build + d.stack["oz"][i] * OZ_TO_M)  # GMD of the winding cross-section
            other = loop.copy()
            other[:, 2] += dz
            L += n * n * _neumann(loop, other)
    per_phase = d.n_coils // 3
    return L * per_phase if d.series else L / per_phase


# ---------------------------------------------------------------- magnetics
def rotor_magnets(r: RotorDesign, theta=0.0):
    """magpylib Collection of the rotor magnets at mechanical angle theta.
    Coordinates in metres; top copper at z=0, rotor above (z>0)."""
    h = r.magnet_h_mm * (2 if r.back_iron else 1)
    zc = (r.airgap_mm + h / 2) / 1e3
    mags = []
    for k in range(r.n_poles):
        a = theta + k * 2 * math.pi / r.n_poles
        sgn = 1 if k % 2 == 0 else -1
        pos = (r.magnet_r_mm / 1e3 * math.cos(a), r.magnet_r_mm / 1e3 * math.sin(a), zc)
        if r.magnet_shape == "block":
            mags.append(magpy.magnet.Cuboid(
                polarization=(0, 0, sgn * r.br_T),
                dimension=(r.block_len_mm / 1e3, r.block_w_mm / 1e3, h / 1e3),
                position=pos, orientation=Rotation.from_euler("z", a)))
        else:
            mags.append(magpy.magnet.Cylinder(
                polarization=(0, 0, sgn * r.br_T),
                dimension=(r.magnet_d_mm / 1e3, h / 1e3), position=pos))
    return magpy.Collection(*mags)


def _subdivide(poly, max_len_mm):
    out = [poly[:1]]
    for a, b in zip(poly[:-1], poly[1:]):
        n = max(1, int(math.ceil(np.linalg.norm(b - a) / max_len_mm)))
        t = np.linspace(0, 1, n + 1)[1:, None]
        out.append(a + (b - a) * t)
    return np.vstack(out)


def _coil_segments(d: StatorDesign, max_seg_mm=0.25):
    """Midpoints and dl vectors (m) of every segment of one coil (angle 0), all layers.
    Segments are subdivided so the field is sampled every max_seg_mm along the trace.
    Current flows the same way round on every layer (series stack)."""
    mids, dls = [], []
    for z in d.stack["z_mm"]:
        for poly in coil_turns(d):
            poly = _subdivide(poly, max_seg_mm)
            p = np.c_[poly / 1e3, np.full(len(poly), -z / 1e3)]
            m = (p[:-1] + p[1:]) / 2
            mids.append(m)
            dls.append(np.diff(p, axis=0))
    return np.vstack(mids), np.vstack(dls)


def coil_torque_curve(d: StatorDesign, r: RotorDesign, n_theta=72):
    """Torque on ONE coil (angle 0) per amp vs rotor angle over one electrical
    period. Returns (theta array [rad mech], torque per amp [Nm/A])."""
    mids, dls = _coil_segments(d)
    period = 2 * (2 * math.pi / r.n_poles)
    thetas = np.linspace(0, period, n_theta, endpoint=False)
    tq = np.empty(n_theta)
    for i, th in enumerate(thetas):
        B = rotor_magnets(r, th).getB(mids)
        F = np.cross(dls, B)          # per amp
        tq[i] = np.sum(mids[:, 0] * F[:, 1] - mids[:, 1] * F[:, 0])
    # torque ON THE ROTOR is the reaction: minus the force on the coil
    return thetas, -tq


def foc_torque(d: StatorDesign, r: RotorDesign, thetas, tq_coil, n_eval=360):
    """Ideal sinusoidal 3-phase currents (peak 1 A) at the best current angle.
    Returns (mean torque Nm/A_peak, ripple fraction, phase torque constant curves)."""
    period = thetas[1] - thetas[0]
    P = len(thetas) * period
    pm = phase_map(d, r.n_poles)
    pp = r.n_poles // 2
    th = np.linspace(0, 2 * math.pi / pp, n_eval, endpoint=False)

    def coil_k(theta, phi):
        x = (theta - phi) % P
        return np.interp(x, np.r_[thetas, P], np.r_[tq_coil, tq_coil[0]])

    # per-phase torque constant k_ph(theta) [Nm/A] = sum of its coils with signs
    kph = np.zeros((3, n_eval))
    for (ph, sgn), phi in zip(pm, coil_angles(d)):
        kph[ph] += sgn * coil_k(th, phi)
    best = None
    for delta in np.linspace(0, 2 * math.pi, 180, endpoint=False):
        i = np.array([np.cos(pp * th + delta - k * 2 * math.pi / 3) for k in range(3)])
        t = np.sum(kph * i, axis=0)
        if best is None or t.mean() > best[0]:
            best = (t.mean(), t)
    mean, t = best
    ripple = (t.max() - t.min()) / abs(mean) if mean else float("inf")
    return mean, ripple, th, kph


def analyse(d: StatorDesign, r: RotorDesign, n_theta=72):
    d.check_rules()
    turns = coil_turns(d)
    thetas, tq = coil_torque_curve(d, r, n_theta)
    kt, ripple, _, _ = foc_torque(d, r, thetas, tq)
    J, m = r.inertia_kgm2()
    return MotorResult(
        name=d.name,
        turns_per_layer=len(turns),
        coils_per_phase=d.n_coils // 3,
        R_phase_ohm=phase_resistance(d),
        L_phase_uH=phase_inductance(d) * 1e6,
        Kt_Nm_per_A=kt,
        torque_ripple_pct=ripple * 100,
        Ke_phase_V_per_rad_s=kt * 2 / 3,
        winding_ok=True,
        rotor_inertia_kgm2=J,
        rotor_mass_kg=m,
        copper_mass_g=copper_mass_g(d),
    ), (thetas, tq)
