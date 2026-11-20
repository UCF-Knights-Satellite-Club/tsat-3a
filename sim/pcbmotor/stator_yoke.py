"""Slotless stator yoke: a soft-magnetic disc (SMC or ferrite) bonded behind the stator PCB.

Question: how much does a magnetic plate behind the stator gain, and what does it cost? It gives the
magnet flux a low-reluctance return path straight through the copper, instead of the flux curling
between neighbouring magnets in the gap. Marignetti et al. 2018 printed a wave winding on an
Atomet SMC core this way, and TU Graz built ferrite-stator PCB motors.

Model (same magpylib Lorentz-force method as the rest of the sim):
- **Field.** Each iron surface is an ideal (mu -> infinity) plane, handled by the method of images,
  like em.py's rotor back iron (a magnet of twice the thickness).
  - Plane A is the rotor back iron, at the magnets' back face. Plane B is the yoke surface, behind
    the board.
  - A magnet reflected in an iron plane keeps its axial polarization.
  - With two planes the images repeat with period 2 x (plane spacing). Orders are kept while their
    near face is within IMAGE_REACH_MM of the copper; each order adds about 10% of the one before.
  - With no yoke this reproduces em.rotor_magnets exactly (checked in main()).
- **Axial pull** on the yoke: integral of (Bz^2 - Bt^2) / (2 mu0) over the yoke surface, which is
  the Maxwell stress just outside an ideal iron surface.
- **Yoke thickness** carries half a pole's flux sideways at the material's design flux density:
  t = Phi_pole / (2 B_design L), with L the magnet's radial length.
- **Iron loss** in the yoke at the electrical frequency (4 x rpm / 60), and the drag it causes.
  - Uses rough Steinmetz-type coefficients (ASSUMED, not from a datasheet): SMC about 43 W/kg at
    1 T and 400 Hz (Somaloy-class); MnZn power ferrite about 0.2 W/kg at 0.2 T and 270 Hz.
  - The flux density in the yoke varies round the circumference, so the mean of B^2 is taken as
    half its peak.

Not modelled:
- finite permeability and saturation (the yoke is sized to stay below saturation);
- the gaps between ferrite tiles, and the small cogging they would cause;
- the bond line's tolerance.

Run from sim/:  .venv/bin/python -m pcbmotor.stator_yoke
Writes results/pcbmotor_stator_yoke.json and results/pcbmotor_stator_yoke.md.
"""
from __future__ import annotations

from contextlib import contextmanager
import json
import math
import pathlib

import numpy as np
import magpylib as magpy
from scipy.spatial.transform import Rotation

from . import em, stacking, wave, wave_asbuilt, mechanical
from .geometry import RotorDesign, STACKUPS

OUT = pathlib.Path(__file__).resolve().parents[1] / "results"
MU0 = 4e-7 * math.pi
IMAGE_REACH_MM = 60.0     # keep images whose near face is within this distance of the copper
BOND_MM = 0.15            # adhesive + solder mask between the bottom copper and the yoke
YOKE_R = (14.0, 52.0)     # yoke annulus (covers the winding of every design here)

MATERIALS = {
    # B_design: flux density the yoke is sized for; rho kg/m^3; loss(f, B) in W/kg
    "SMC": dict(B_design=1.2, rho=7400.0, loss=lambda f, B: (0.10 * f + 2.0e-5 * f * f) * B ** 2,
                note="Somaloy/Atomet-class soft magnetic composite (assumed coefficients)"),
    "ferrite": dict(B_design=0.35, rho=4800.0, loss=lambda f, B: 0.0008 * f * (B / 0.2) ** 2.5,
                    note="MnZn power ferrite (assumed coefficients)"),
}


# ---------------------------------------------------------------- imaged rotor
def _slabs(r: RotorDesign, z_yoke_mm):
    """Axial extents (z0, z1) in mm of the magnet and all its images. Copper top at z = 0, rotor above,
    yoke surface at z = z_yoke_mm (negative), or None for no yoke."""
    a0, a1 = r.airgap_mm, r.airgap_mm + r.magnet_h_mm
    planes = []
    if r.back_iron:
        planes.append(a1)                     # rotor back iron at the magnets' back face
    if z_yoke_mm is not None:
        planes.append(z_yoke_mm)
    out = {(round(a0, 6), round(a1, 6))}
    frontier = [((a0, a1), None)]
    while frontier:
        nxt = []
        for (z0, z1), last in frontier:
            for pz in planes:
                if pz == last:
                    continue
                s = (2 * pz - z1, 2 * pz - z0)
                key = (round(s[0], 6), round(s[1], 6))
                dist = min(abs(s[0]), abs(s[1]), abs(s[0] - z_yoke_mm) if z_yoke_mm is not None else 1e9)
                if key in out or dist > IMAGE_REACH_MM:
                    continue
                out.add(key)
                nxt.append((s, pz))
        frontier = nxt
    return sorted(out)


def imaged_magnets(r: RotorDesign, theta, z_yoke_mm):
    """magpylib Collection: rotor magnets at mechanical angle theta plus their images in the rotor back
    iron and (if z_yoke_mm is not None) the stator yoke."""
    mags = []
    for z0, z1 in _slabs(r, z_yoke_mm):
        h = (z1 - z0) / 1e3
        zc = (z0 + z1) / 2e3
        for k in range(r.n_poles):
            a = theta + k * 2 * math.pi / r.n_poles
            sgn = 1 if k % 2 == 0 else -1
            pos = (r.magnet_r_mm / 1e3 * math.cos(a), r.magnet_r_mm / 1e3 * math.sin(a), zc)
            if r.magnet_shape == "block":
                mags.append(magpy.magnet.Cuboid(polarization=(0, 0, sgn * r.br_T),
                                                dimension=(r.block_len_mm / 1e3, r.block_w_mm / 1e3, h),
                                                position=pos, orientation=Rotation.from_euler("z", a)))
            else:
                mags.append(magpy.magnet.Cylinder(polarization=(0, 0, sgn * r.br_T),
                                                  dimension=(r.magnet_d_mm / 1e3, h), position=pos))
    return magpy.Collection(*mags)


def _no_back_iron(r):
    from dataclasses import replace
    return replace(r, back_iron=False)


@contextmanager
def yoke(z_yoke_mm):
    """Run the existing models with the imaged rotor (z_yoke_mm=None: rotor back iron only)."""
    def fn(r, theta=0.0):
        return imaged_magnets(r, theta, z_yoke_mm)
    mods = (em, stacking, wave, wave_asbuilt, mechanical)
    saved = [m.rotor_magnets for m in mods]
    try:
        for m in mods:
            m.rotor_magnets = fn
        yield
    finally:
        for m, f in zip(mods, saved):
            m.rotor_magnets = f


# ---------------------------------------------------------------- yoke force, flux, sizing, loss
def yoke_surface(r: RotorDesign, z_yoke_mm, n_r=90, n_a=360):
    """Axial pull on the yoke (N), and the flux of one pole into the yoke (Wb)."""
    rr = np.linspace(YOKE_R[0] - 8, YOKE_R[1] + 8, n_r) / 1e3
    aa = np.linspace(0, 2 * math.pi, n_a, endpoint=False)
    R, A = np.meshgrid(rr, aa, indexing="ij")
    pts = np.c_[(R * np.cos(A)).ravel(), (R * np.sin(A)).ravel(), np.full(R.size, (z_yoke_mm + 0.02) / 1e3)]
    B = imaged_magnets(r, 0.0, z_yoke_mm).getB(pts)
    bz = B[:, 2].reshape(R.shape)
    bt2 = (B[:, 0] ** 2 + B[:, 1] ** 2).reshape(R.shape)
    dA = R * (rr[1] - rr[0]) * (aa[1] - aa[0])
    force = float(np.sum((bz ** 2 - bt2) / (2 * MU0) * dA))
    pole = (np.abs(((A + math.pi / r.n_poles) % (2 * math.pi / r.n_poles)) - math.pi / r.n_poles) < math.pi / r.n_poles) \
        & (np.cos(r.n_poles / 2 * A) > 0)
    phi_pole = float(np.sum(np.clip(bz, 0, None) * dA * pole) / (r.n_poles / 2))
    return force, phi_pole


def yoke_options(r: RotorDesign, phi_pole, rpm_list):
    out = {}
    L = r.block_len_mm / 1e3 if r.magnet_shape == "block" else r.magnet_d_mm / 1e3
    area = math.pi * ((YOKE_R[1] / 1e3) ** 2 - (YOKE_R[0] / 1e3) ** 2)
    for name, m in MATERIALS.items():
        t = phi_pole / (2 * m["B_design"] * L)
        mass = area * t * m["rho"]
        loss = {}
        for rpm in rpm_list:
            f = (r.n_poles / 2) * rpm / 60
            p = m["loss"](f, m["B_design"]) * 0.5            # mean of B^2 is about half its peak
            w = rpm * 2 * math.pi / 60
            loss[rpm] = dict(W=mass * p, drag_mNm=mass * p / w * 1e3 if w else 0.0)
        out[name] = dict(thickness_mm=t * 1e3, mass_g=mass * 1e3, iron_loss=loss, note=m["note"])
    return out


# ---------------------------------------------------------------- study
def z_yoke_for(stackup):
    st = STACKUPS[stackup]
    return -(st["z_mm"][-1] + st["oz"][-1] * 0.035 / 2 + BOND_MM)


def run():
    rot = wave.hm_rotor()
    rows = []
    # sanity: with no yoke the imaged rotor reproduces em.rotor_magnets
    pts = np.c_[np.linspace(0.02, 0.04, 7), np.zeros(7), np.full(7, -0.0008)]
    b0 = em.rotor_magnets(rot, 0.1).getB(pts)
    b1 = imaged_magnets(rot, 0.1, None).getB(pts)
    assert np.allclose(b0, b1, rtol=1e-6, atol=1e-9), "imaged rotor with no yoke differs from em.rotor_magnets"

    cases = [("spiral (current payload)", "spiral", "jlc4_2oz_out"),
             ("wave, through vias, 2 strands swapped", "swap", "jlc4_2oz"),
             ("wave, blind vias, 2 strands swapped", "bv_swap", "jlc4_2oz"),
             ("wave, blind vias, 2 strands swapped, 3 oz (assumed)", "bv_swap", "ace4_3oz")]
    for label, variant, stackup in cases:
        zy = z_yoke_for(stackup)
        for with_yoke in (False, True):
            with yoke(zy if with_yoke else None):
                if variant == "spiral":
                    b = wave.spiral_baseline()
                    res = dict(Kt_mNm_per_A=b["Kt_mNm_per_A"], R_phase_ohm=b["R_phase_ohm"],
                               Km_mNm_per_sqrtW=b["Km_mNm_per_sqrtW"], noload_rpm=b["noload_rpm"],
                               eddy_W_at_2000rpm=b["eddy_W_at_2000rpm"], circulating_W_at_2000rpm=0.0)
                else:
                    res = wave_asbuilt.analyse(variant, stackup=stackup)
            J, _ = rot.inertia_kgm2()
            w_top = min(res["noload_rpm"], 6000) * 2 * math.pi / 60
            w2k = 2000 * 2 * math.pi / 60
            row = dict(case=label, variant=variant, stackup=stackup, yoke=with_yoke, z_yoke_mm=zy,
                       Kt_mNm_per_A=float(res["Kt_mNm_per_A"]), R_phase_ohm=float(res["R_phase_ohm"]),
                       Km_mNm_per_sqrtW=float(res["Km_mNm_per_sqrtW"]), noload_rpm=float(res["noload_rpm"]),
                       H_top_mNms=J * w_top * 1e3, copper_W_15mNm=(15 / float(res["Km_mNm_per_sqrtW"])) ** 2,
                       eddy_W_2000=float(res["eddy_W_at_2000rpm"]),
                       circ_W_2000=float(res.get("circulating_W_at_2000rpm", 0.0)))
            if with_yoke:
                F, phi = yoke_surface(rot, zy)
                opts = yoke_options(rot, phi, [2000, 4000])
                row.update(pull_N=F, phi_pole_Wb=phi, yoke=opts)
            row["loss_15mNm_2000_W"] = row["copper_W_15mNm"] + row["eddy_W_2000"] + row["circ_W_2000"]
            rows.append(row)
            print({k: (round(v, 3) if isinstance(v, float) else v) for k, v in row.items() if k != "yoke"}, flush=True)
    return rows


def write(rows):
    (OUT / "pcbmotor_stator_yoke.json").write_text(json.dumps(rows, indent=2, default=float))
    L = ["# Slotless stator yoke behind the PCB", "",
         "Generated by `pcbmotor/stator_yoke.py`. Rotor: the high-momentum payload rotor (8 × 19.05 × 12.7 × 6.35 mm "
         "N42, back iron, 0.6 mm gap). Yoke: ideal soft-magnetic plane "
         f"{BOND_MM} mm behind the bottom copper, sized below saturation; iron-loss coefficients are assumed. "
         "Loss is copper + eddy + strand circulating current at 15 mN·m and 2000 rpm (yoke iron loss listed separately).", "",
         "| Design | Yoke | Kt mN·m/A | R Ω | Km mN·m/√W | Loss W | Top rpm (11.1 V) | H at top mN·m·s |",
         "|---|---|---|---|---|---|---|---|"]
    for r in rows:
        L.append(f"| {r['case']} | {'yes' if r['yoke'] else 'no'} | {r['Kt_mNm_per_A']:.1f} | {r['R_phase_ohm']:.2f} | "
                 f"{r['Km_mNm_per_sqrtW']:.1f} | {r['loss_15mNm_2000_W']:.2f} | {r['noload_rpm']:.0f} | {r['H_top_mNms']:.0f} |")
    y = next(r for r in rows if r["yoke"])
    L += ["", "## The yoke itself (same rotor, so the same for every design)", "",
          f"- Axial pull on the yoke: **{y['pull_N']:.0f} N** (about {y['pull_N'] / 9.81:.1f} kgf), carried by the bearings / spacer.",
          f"- Flux of one pole into the yoke: {y['phi_pole_Wb'] * 1e6:.0f} µWb.", "",
          "| Material | Thickness mm | Mass g (annulus r " + f"{YOKE_R[0]:g}–{YOKE_R[1]:g} mm) | Iron loss W at 2000 / 4000 rpm | Drag mN·m at 2000 / 4000 rpm |",
          "|---|---|---|---|---|"]
    for name, o in y["yoke"].items():
        l2, l4 = o["iron_loss"][2000], o["iron_loss"][4000]
        L.append(f"| {name} | {o['thickness_mm']:.1f} | {o['mass_g']:.0f} | {l2['W']:.2f} / {l4['W']:.2f} | "
                 f"{l2['drag_mNm']:.2f} / {l4['drag_mNm']:.2f} |")
    (OUT / "pcbmotor_stator_yoke.md").write_text("\n".join(L) + "\n")


def main():
    write(run())


if __name__ == "__main__":
    main()
