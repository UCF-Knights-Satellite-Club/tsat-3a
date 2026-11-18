"""Reaction-wheel motor candidates for the yaw simulation.

Datasheet values are quoted where we have them; values marked APPROX are estimates to verify.
Torque constants: for "dc" models kt is per amp of supply-side (DC-equivalent) current and r is the
terminal (phase-to-phase) resistance, as in Faulhaber/Maxon datasheets. For "foc" models kt is per amp
of PEAK phase current under sinusoidal drive, r is per-phase, and the usable phase voltage is Vbus/sqrt(3).
"""
from __future__ import annotations

import json
import math
import pathlib
from dataclasses import dataclass

RESULTS = pathlib.Path(__file__).resolve().parents[1] / "results"
STEEL = 7850.0


def disc_inertia(od_mm, t_mm, rho=STEEL):
    r = od_mm / 2e3
    m = rho * math.pi * r * r * t_mm / 1e3
    return 0.5 * m * r * r, m


@dataclass
class Motor:
    name: str
    model: str            # "dc" or "foc"
    kt: float             # Nm/A
    r: float              # ohm (terminal for dc, per phase for foc)
    i_max: float          # A, driver / thermal current limit
    j_wheel: float        # kg m^2, rotor + flywheel
    mass_g: float         # motor + flywheel (+ PCB stator board copper/rotor for PCB designs)
    rpm_cap: float        # mechanical / rating speed cap
    sensored: bool        # Halls or encoder: controllable through zero speed
    coulomb_Nm: float = 2e-5   # bearing + seal drag
    viscous: float = 2e-8      # Nm per rad/s
    notes: str = ""
    ideal: bool = False        # unlimited voltage and speed (requirement finder)

    def ke(self):
        return self.kt if self.model == "dc" else self.kt * 2.0 / 3.0

    def v_avail(self, vbus):
        if self.ideal:
            return 1e9
        return 0.95 * vbus if self.model == "dc" else vbus / math.sqrt(3)

    def copper_loss(self, i):
        return (1.0 if self.model == "dc" else 1.5) * self.r * i * i

    def omega_max(self, vbus):
        if self.ideal:
            return math.inf
        return min(self.v_avail(vbus) / self.ke(), self.rpm_cap * 2 * math.pi / 60)


FLYWHEEL_55 = disc_inertia(55, 5)        # 55 mm x 5 mm steel disc (team's proposed 55 mm wheel)


def candidates(vbus=11.1):
    j55, m55 = FLYWHEEL_55
    out = [
        Motor("Faulhaber 2610T012B + 55mm wheel", "dc", kt=0.0176, r=28.2, i_max=0.6,
              j_wheel=7.9e-7 + j55, mass_g=20.1 + m55 * 1e3, rpm_cap=6400, sensored=True,
              notes="datasheet; digital Halls; 7.45 mNm stall at 12 V"),
        Motor("Maxon EC32 flat 226006 + 55mm wheel", "dc", kt=0.049, r=13.8, i_max=1.0,
              j_wheel=35e-7 + j55, mass_g=57 + m55 * 1e3, rpm_cap=4530, sensored=False,
              notes="datasheet; SENSORLESS; NRND; 24 V nominal run at 11.1 V"),
        Motor("mj5208 + moteus r4.11 (no added wheel)", "foc", kt=0.0314 * 1.5 / math.sqrt(3), r=0.12, i_max=3.0,
              j_wheel=8.5e-5, mass_g=195 + 14, rpm_cap=3300, sensored=True, coulomb_Nm=1e-4,
              notes="APPROX: Kv 304, R and rotor inertia estimated, iron-core cogging ignored; encoder FOC"),
    ]
    p = RESULTS / "pcbmotor_designs.json"
    if p.exists():
        d = json.loads(p.read_text())
        for key, label in (("test", "PCB test stator (40 mm, OSH Park 4L)"),
                           ("payload", "PCB payload stator (1U, cheap fab)")):
            if key not in d:
                continue
            row, res = d[key]["row"], d[key]["result"]
            i_cont = row.get("i_cont_A", row["i_stall_A"])
            out.append(Motor(f"{label}: {row['name']}", "foc", kt=res["Kt_Nm_per_A"], r=res["R_phase_ohm"],
                             i_max=i_cont, j_wheel=res["rotor_inertia_kgm2"],
                             mass_g=res["rotor_mass_kg"] * 1e3 + res["copper_mass_g"],
                             rpm_cap=row["usable_max_rpm"], sensored=True,
                             notes="simulated (magpylib); Halls assumed; current = thermal limit (40 K rise)"))
    # High-momentum PCB variant (added after the TSat-2B video raised the momentum demand): the sweep's
    # highest-momentum cheap-fab payload design, with the 84 mm steel disc doubled from 1.5 to 3 mm.
    # A thicker disc changes only the inertia and mass (the back-iron model is already ideal).
    sweep = RESULTS / "pcbmotor_sweep.csv"
    if sweep.exists():
        import csv
        rows = {r["name"]: r for r in csv.DictReader(open(sweep))}
        hm = rows.get("payload_jlc4_2oz_out_6c8p_w1.2_h6.35")
        if hm:
            dj, dm = disc_inertia(84, 1.5)
            out.append(Motor("PCB payload, high momentum (1U, cheap fab, 3 mm disc): payload_jlc4_2oz_out_6c8p_w1.2_h6.35",
                             "foc", kt=float(hm["Kt_mNm_per_A"]) / 1e3, r=float(hm["R_phase_ohm"]),
                             i_max=float(hm["i_cont_A"]), j_wheel=float(hm["rotor_J_kgm2"]) + dj,
                             mass_g=float(hm["rotor_mass_g"]) + dm * 1e3, rpm_cap=float(hm["usable_max_rpm"]),
                             sensored=True,
                             notes="sweep design + 1.5 mm extra steel on the 84 mm disc; Halls assumed"))
    out.append(Motor("Ideal actuator (requirement finder)", "foc", kt=1.0, r=1e-6, i_max=1e3,
                     j_wheel=3.5e-5, mass_g=0, rpm_cap=1e9, sensored=True, coulomb_Nm=0, viscous=0,
                     notes="unlimited torque and speed; used only to measure demand", ideal=True))
    return out
