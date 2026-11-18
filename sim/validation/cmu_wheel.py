"""Check the PCB-motor model against a built and measured motor: the CMU PCB reaction wheel
(Tayal, Fisch, Manchester, SmallSat 2024, doi:10.26077/sr9e-tc61; design files at
github.com/SaralTayal123/PCB_Motors, read 2026-11-18).

Measured (paper + repo): 36 slots, 12 poles (firmware PP = 6), two PCB stators (one each side of
the rotor), wave winding with each radial conductor stitched across all 4 layers (1 oz, 1.2 mm board),
coil ID 53 / OD 79 mm, 0.29 mm traces + 0.172 mm space, 12 x (12.7 x 6.35 x 3.18 mm) N42 magnets on a
3 mm aluminium rotor (no back iron), phase resistance 5.5 ohm, 8 mNm peak torque, 5 W max power.

Model: the same magpylib Lorentz-force method as pcbmotor/em.py, applied to the wave-winding
conductor set. Air gap (PTFE bushing flange) is not given, so it is swept.

Run from sim/:  .venv/bin/python -m validation.cmu_wheel
"""
from __future__ import annotations

import json
import math
import pathlib

import numpy as np
import magpylib as magpy

from pcbmotor.em import RHO_CU_20C
from pcbmotor.geometry import OZ_TO_M

OUT = pathlib.Path(__file__).resolve().parents[1] / "results"

ID, OD = 53.0, 79.0              # mm, coil inner / outer diameter (notebook)
SLOTS, POLES = 36, 12
TRACE, SPACE = 0.29, 0.172       # mm
LINK_OUT_W, LINK_IN_W = 0.45, 0.40
LAYER_Z = [0.0, 0.135, 1.010, 1.145]   # mm below the rotor-facing copper surface (PCB stackup in the KiCad file)
CU_T = 1.0 * OZ_TO_M
MAG = (12.7e-3, 6.35e-3, 3.18e-3)      # radial, tangential, axial [m]
BR = 1.32                               # N42
R_MAG = (ID + OD) / 4 / 1e3             # magnets centred on the active band

# turns per bundle, as in the notebook (TRACES_NEEDED)
sub_angle = 360 / (SLOTS // 6) / 6
TURNS = math.floor(ID * math.pi * sub_angle / 360 / (TRACE + SPACE))
NEW_ID = TURNS * (TRACE + SPACE) * (SLOTS // 6) * 6 / math.pi


def phase_resistance_one_stator():
    l_rad = (OD - NEW_ID) / 2 / 1e3
    r_rad = RHO_CU_20C * l_rad / (TRACE / 1e3 * CU_T) / len(LAYER_Z)       # 4 layers in parallel
    n_bundles = POLES                                                         # one bundle per pole pitch
    l_out = 2 * math.pi * OD / 2 / n_bundles / 1e3
    l_in = 2 * math.pi * NEW_ID / 2 / n_bundles / 1e3
    r_links = (n_bundles / 2) * (RHO_CU_20C * l_out / (LINK_OUT_W / 1e3 * CU_T)
                                 + RHO_CU_20C * l_in / (LINK_IN_W / 1e3 * CU_T))
    return TURNS * (n_bundles * r_rad + r_links)


def conductors(phase):
    """Radial conductors of one phase: angle [rad] and current sign. Bundles every pole pitch,
    alternating direction (wave winding), offset 10 deg per phase."""
    out = []
    pitch = 2 * math.pi / POLES
    base = math.radians(phase * sub_angle)
    for k in range(POLES):
        for t in range(TURNS):
            a = base + k * pitch + (t + 0.5) / TURNS * math.radians(sub_angle)
            out.append((a, 1 if k % 2 == 0 else -1))
    return out


def magnets(theta, gap_mm, side):
    """Rotor magnets; side=+1 places the stator below the rotor (z<0 copper), -1 above."""
    mags = []
    zc = side * (gap_mm / 1e3 + MAG[2] / 2)
    for k in range(POLES):
        a = theta + k * 2 * math.pi / POLES
        sgn = 1 if k % 2 == 0 else -1
        m = magpy.magnet.Cuboid(polarization=(0, 0, sgn * BR), dimension=MAG,
                                position=(R_MAG * math.cos(a), R_MAG * math.sin(a), zc))
        m.rotate_from_angax(math.degrees(a), "z", anchor=(R_MAG * math.cos(a), R_MAG * math.sin(a), zc))
        mags.append(m)
    return magpy.Collection(*mags)


def phase_torque_curve(phase, gap_mm, n_theta=48, n_sub=24):
    """Torque per amp on the rotor from one phase, both stators in series, vs rotor angle."""
    cond = conductors(phase)
    r = np.linspace(NEW_ID / 2, OD / 2, n_sub + 1) / 1e3
    rm = (r[:-1] + r[1:]) / 2
    dr = np.diff(r)
    thetas = np.linspace(0, 2 * 2 * math.pi / POLES, n_theta, endpoint=False)
    tq = np.zeros(n_theta)
    for side in (+1, -1):
        pts, dls = [], []
        for z in LAYER_Z:
            zz = -side * z / 1e3
            for a, s in cond:
                c, sn = math.cos(a), math.sin(a)
                for rr, d in zip(rm, dr):
                    pts.append((rr * c, rr * sn, zz))
                    dls.append((s * d * c / len(LAYER_Z), s * d * sn / len(LAYER_Z), 0.0))   # current split over 4 layers
        pts, dls = np.array(pts), np.array(dls)
        for i, th in enumerate(thetas):
            B = magnets(th, gap_mm, side).getB(pts)
            F = np.cross(dls, B)
            tq[i] -= np.sum(pts[:, 0] * F[:, 1] - pts[:, 1] * F[:, 0])
    return thetas, tq


def foc_kt(gap_mm):
    curves = [phase_torque_curve(p, gap_mm)[1] for p in range(3)]
    thetas = phase_torque_curve(0, gap_mm, n_theta=48)[0]
    pp = POLES // 2
    best = 0.0
    for sb in (1, -1):
        for sc in (1, -1):
            k = np.array([curves[0], sb * curves[1], sc * curves[2]])
            for delta in np.linspace(0, 2 * math.pi, 72, endpoint=False):
                for order in (1, -1):
                    i = np.array([np.cos(pp * thetas + delta - order * j * 2 * math.pi / 3) for j in range(3)])
                    best = max(best, float(np.mean(np.sum(k * i, axis=0))))
    return best


def main():
    r1 = phase_resistance_one_stator()
    r_series = 2 * r1
    res = dict(turns=TURNS, new_id_mm=NEW_ID, R_one_stator_ohm=r1, R_two_series_ohm=r_series,
               R_measured_ohm=5.5, gaps={})
    for gap in (0.3, 0.5, 0.75, 1.0, 1.5):
        kt = foc_kt(gap)
        p_cu_at_8 = 1.5 * r_series * (8e-3 / kt) ** 2
        res["gaps"][gap] = dict(Kt_mNm_per_A=kt * 1e3, Km_mNm_per_sqrtW=kt * 1e3 / math.sqrt(1.5 * r_series),
                                I_for_8mNm_A=8e-3 / kt, copper_W_for_8mNm=p_cu_at_8)
        print(gap, res["gaps"][gap], flush=True)
    # measured bound: 8 mNm with <= 5 W total input -> Km >= 8/sqrt(5)
    res["measured_Km_lower_bound"] = 8 / math.sqrt(5)
    (OUT / "validation_cmu_wheel.json").write_text(json.dumps(res, indent=2))
    print(json.dumps({k: v for k, v in res.items() if k != "gaps"}, indent=2))


if __name__ == "__main__":
    main()
