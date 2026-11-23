"""Model of the test stator exactly as laid out, and the bench numbers to check it against.

Run from sim/ after PCBMotorTest/gen_stator.py:   .venv/bin/python -m pcbmotor.asbuilt
Reads PCBMotorTest/stator_asbuilt.json (ring track lengths) and writes
results/pcbmotor_test_asbuilt.json.
"""
from __future__ import annotations

import json
import math
import pathlib
from dataclasses import replace

from .geometry import StatorDesign, RotorDesign, OZ_TO_M
from .layout import coil_layout, check_layout
from .em import analyse_layout, RHO_CU_20C, ALPHA_CU

ROOT = pathlib.Path(__file__).resolve().parents[2]
OUT = ROOT / "sim" / "results"
# The test stator is ordered from JLCPCB on its published JLC04161H-3313 stackup (2026-11-23; it was
# designed on OSH Park's), with 0.3 mm via drills (JLC charges extra below 0.3 mm).
STACKUP = "jlc4_3313"
VIA_DRILL_MM = 0.3


def ring_resistance(ring, d, ring_w, ring_oz):
    """Ring arcs on an inner layer, leads/tails on the outer layers, plus vias. Leads are taken at
    the coil trace width (conservative: the header tails are wider)."""
    rho = RHO_CU_20C
    r_in = rho * ring["inner_mm"] / 1e3 / (ring_w / 1e3 * ring_oz * OZ_TO_M)
    r_out = rho * ring["outer_mm"] / 1e3 / (d.trace_mm / 1e3 * d.stack["oz"][0] * OZ_TO_M)
    r_via = ring["vias"] * rho * 1.6e-3 / (math.pi * VIA_DRILL_MM / 1e3 * 25e-6)
    return r_in + r_out + r_via


def main():
    designs = json.loads((OUT / "pcbmotor_designs.json").read_text())["test"]
    d, r = replace(StatorDesign(**designs["stator"]), stackup=STACKUP), RotorDesign(**designs["rotor"])
    lay = coil_layout(d)
    check_layout(lay)
    board = json.loads((ROOT / "PCBMotorTest" / "stator_asbuilt.json").read_text())
    ring_oz = d.stack["oz"][2]                       # In2.Cu
    r_ring = {k: ring_resistance(v, d, board["ring_track_mm"], ring_oz) for k, v in board["ring"].items()}
    res = analyse_layout(lay, r, n_theta=72, extra_phase_ohm=max(r_ring.values()))
    sweep = designs["result"]
    kt, ke = res["Kt_Nm_per_A"], res["Ke_phase_V_per_rad_s"]
    krpm = 1000 * 2 * math.pi / 60
    v_bus = 11.1
    out = dict(
        design=d.name,
        as_built={k: (float(v) if not isinstance(v, list) else [float(x) for x in v]) for k, v in res.items()},
        ring_ohm_per_phase=r_ring,
        sweep_model=dict(turns_per_layer=sweep["turns_per_layer"], R_phase_ohm=sweep["R_phase_ohm"],
                         Kt_Nm_per_A=sweep["Kt_Nm_per_A"]),
        bench_predictions={
            "phase_resistance_ohm_20C": {f"{k[-1]}1-{k[-1]}2": res["R_coil_ohm"] * (d.n_coils // 3) + v
                                         for k, v in r_ring.items()},
            "line_to_line_resistance_ohm_star": 2 * res["R_phase_ohm"],
            "resistance_tempco_pct_per_K": ALPHA_CU * 100,
            "phase_backEMF_V_peak_per_krpm": ke * krpm,
            "line_backEMF_V_peak_per_krpm (star)": ke * krpm * math.sqrt(3),
            "electrical_Hz_per_krpm": 1000 / 60 * r.n_poles / 2,
            "Kt_mNm_per_A_peak (FOC)": kt * 1e3,
            "Km_mNm_per_sqrtW": res["Km_Nm_per_sqrtW"] * 1e3,
            "stall_torque_mNm_at_11.1V_voltage_limited": kt * 1e3 * (v_bus / math.sqrt(3)) / res["R_phase_ohm"],
            "noload_rpm_at_11.1V": (v_bus / math.sqrt(3)) / ke * 60 / (2 * math.pi),
        },
        notes=["Back-EMF assumes the rotor in sim/results/pcbmotor_designs.json (8 x 8x3 mm N42 discs, "
               "1.5 mm steel back plate, 0.5 mm air gap). Measure the real gap; Kt falls with gap.",
               "Resistance at 20 C; copper rises 0.39 %/K, so log board temperature or measure R cold."],
    )
    # Back-EMF against air gap, with and without the steel back plate. Measuring the same rotor with
    # and without its plate checks the method-of-images back-iron model directly.
    sens = []
    for gap in (0.5, 0.75, 1.0, 1.5):
        for bi in (True, False):
            rr = replace(r, airgap_mm=gap, back_iron=bi)
            x = analyse_layout(lay, rr, n_theta=36)
            sens.append(dict(airgap_mm=gap, back_iron=bi, Kt_mNm_per_A=x["Kt_Nm_per_A"] * 1e3,
                             phase_backEMF_V_peak_per_krpm=x["Ke_phase_V_per_rad_s"] * krpm))
    out["backEMF_vs_gap"] = sens
    (OUT / "pcbmotor_test_asbuilt.json").write_text(json.dumps(out, indent=2))
    print(json.dumps(out, indent=2))


if __name__ == "__main__":
    main()
