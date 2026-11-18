"""Print Markdown tables from results/yaw_summary.csv and results/yaw_requirements.json.

Run from sim/:  .venv/bin/python -m yaw.report
"""
from __future__ import annotations

import csv
import json
import pathlib

RES = pathlib.Path(__file__).resolve().parents[1] / "results"


def short(name):
    return name.split(":")[0].replace(" + 55mm wheel", " + 55 mm wheel")


def main():
    rows = list(csv.DictReader(open(RES / "yaw_summary.csv")))
    req = json.loads((RES / "yaw_requirements.json").read_text())
    print("### Stable-video time and saturation (median / worst of 8 runs, 600 s each)\n")
    print("| Candidate | Rigging | Level | Uncontrolled peak spin [rpm] | Stable % median / worst | "
          "Runs saturated | Median time to saturation [s] | Mean / peak power [W] | Energy per 30 min [Wh] | Peak current [A] |")
    print("|---|---|---|---|---|---|---|---|---|---|")
    for r in rows:
        ts = float(r["t_sat_median_s"])
        print(f"| {short(r['candidate'])} | {r['rigging']} | {r['level']} | {float(r['uncontrolled_peak_rpm']):.0f} | "
              f"{float(r['good_pct_median']):.0f} / {float(r['good_pct_worst']):.0f} | {r['saturated_runs']} | "
              f"{'-' if ts == float('inf') else f'{ts:.0f}'} | {float(r['mean_P_W']):.2f} / {float(r['peak_P_W']):.2f} | "
              f"{float(r['energy_Wh_30min']):.3f} | {float(r['peak_I_A']):.2f} |")
    print("\n### Demand measured with an ideal actuator (no unloading, holds heading for 600 s)\n")
    print("| Rigging | Level | Stable % | Peak torque [mN·m] | Peak wheel momentum [mN·m·s] | Rate RMS [deg/s] |")
    print("|---|---|---|---|---|---|")
    for r in req["ideal_actuator_hold_heading_10min"]:
        print(f"| {r['rigging']} | {r['level']} | {float(r['good_pct_median']):.0f} | {float(r['tau_req_peak_mNm']):.1f} | "
              f"{float(r['h_peak_mNms']):.0f} | {float(r['rate_rms_dps_median']):.2f} |")
    print("\n### Settling after a 3 mN·m, 2 s gust (ball-bearing swivel, otherwise calm)\n")
    print("| Candidate | Settling time [s] | Peak rate [deg/s] |")
    print("|---|---|---|")
    for k, (ts, pk) in req["gust_step_settling_s_and_peak_rate_dps"].items():
        print(f"| {short(k)} | {float(ts):.1f} | {float(pk):.1f} |")


if __name__ == "__main__":
    main()
