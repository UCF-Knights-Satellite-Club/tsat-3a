"""Where PCB-motor wheels could beat conventional wheels: the stator scaled from 90 mm to 600 mm.

Run from sim/:  .venv/bin/python -m pcbmotor.large_wheels
Writes results/pcbmotor_large_wheels.json, .md and .png (`--plot` redraws the figure from the JSON).

Question: the KAOS-3 wheels are 90 mm. A PCB stator is an air-core (coreless) axial-flux motor
whose copper fill is low (about 11% at JLC), so at small size an iron-core motor of the same mass
gives several times the torque per root watt. This study asks how that changes as the wheel grows.

Two PCB layouts at each rotor diameter D (the winding outer diameter):
- "disc": the TSat-3A payload proportions scaled up (winding from 0.51 R to R, magnets 0.95 of the
  winding length).
- "ring": a fixed 40 mm wide winding annulus at the rim (magnets 38 mm long). The middle of the wheel
  is free, so the ring can surround a flight line, a shaft or an instrument.
Both keep the pole pitch near 25 mm (poles a multiple of 8, coils = 3/4 of the poles, the same 6/8
winding repeated), N42 blocks 12.7 mm wide and 6.35 mm thick, 3 mm steel back iron, stranded
transposed traces (1.2 mm, 3 strands; see ace_stackups.py), on a JLC 4-layer 2 oz board and an
ACE-style 6-layer 3 oz board (ACE stackup assumed).

Air gap ASSUMED to grow with size, because board flatness and rotor runout grow with size even
when the stator is bonded to a machined plate or built from arc segments:
gap = 0.6 mm + 0.1% of (D - 90 mm). A 1.5 mm gap case is run as a sensitivity.

Speed for drag numbers: rim speed held to 40 m/s (6,000 rpm cap). Magnets bonded to a face are
fine at small sizes (mechanical.py), but a large fast rim needs a retaining band; 40 m/s keeps the
comparison at a speed real large wheels use.

Conventional motors are datasheet values (COTS list below): Kollmorgen TBM frameless slotted
motors (60, 76, 129 mm) and the Parker Bayside K178200 frameless motor that drives the 2 m SPIDER
balloon-gondola reaction wheel. Their masses include rotor magnets and back iron, so the PCB
"active mass" adds magnets and back iron to the board for a like-for-like Km per kilogram.
"""
from __future__ import annotations

import json
import math
import pathlib
from concurrent.futures import ProcessPoolExecutor

import numpy as np

from .geometry import StatorDesign, RotorDesign, STACKUPS, OZ_TO_M
from .em import foc_torque, copper_mass_g
from .stacking import coil_curves
from .cubesat_wheels import stator_em
from . import ace_stackups  # noqa: F401  (registers the ace* stackups)

OUT = pathlib.Path(__file__).resolve().parents[1] / "results"
DIAMS = (90, 150, 250, 400, 600)
RING_W = 40.0
POLE_PITCH = 25.0
MAG_W, MAG_H, IRON_T = 12.7, 6.35, 3.0
TRACE, STRANDS = 1.2, 3
V_RIM, RPM_CAP = 40.0, 6000.0
STATORS = [("JLC 4L", "jlc4_2oz_out"), ("ACE 6L 3 oz", "ace6_3oz")]
STEEL, FR4, NDFEB = 7850.0, 1850.0, 7500.0

# Conventional frameless motors (motor only, rotor + stator). Km cold, 25 °C.
COTS = [
    # name, OD mm, axial mm, mass kg, Km N·m/√W, cogging N·m peak-to-peak, continuous stall N·m,
    # fi = viscous damping N·m per 1000 rpm (iron loss and windage; datasheet)
    dict(name="Kollmorgen TBM(S)-6013", od=60, ax=13, fi=0.00204, kg=0.221, km=0.097, cog=0.009, tc=0.415),
    dict(name="Kollmorgen TBM(S)-6051", od=60, ax=51, fi=0.0114, kg=0.571, km=0.230, cog=0.019, tc=1.08),
    dict(name="Kollmorgen TBM(S)-7615", od=76, ax=15, fi=0.00605, kg=0.435, km=0.167, cog=0.013, tc=0.939),
    dict(name="Kollmorgen TBM(S)-7646", od=76, ax=46, fi=0.0281, kg=1.079, km=0.360, cog=0.020, tc=2.18),
    dict(name="Kollmorgen TBM(S)-12913", od=129, ax=13, fi=0.0424, kg=1.32, km=0.443, cog=0.071, tc=2.87),
    dict(name="Kollmorgen TBM(S)-12941", od=129, ax=41, fi=0.271, kg=3.17, km=1.12, cog=0.216, tc=8.30),
    # SPIDER wheel motor: Kt 2.007 N·m/Arms, R 0.6857 ohm (taken as line to line), so
    # P = 1.5 R I_rms^2 and Km = Kt / sqrt(1.5 R) = 1.98 N·m/√W. Cogging not published.
    dict(name="Parker Bayside K178200 (SPIDER gondola wheel)", od=178, ax=None, kg=6.34,
         km=2.007 / math.sqrt(1.5 * 0.6857), cog=None, tc=25.74, fi=None),
]
COTS_SOURCES = [
    "Kollmorgen TBM(S) Frameless Motor Selection Guide (kollmorgen.com, checked 2026-11-19)",
    "Parker Bayside K178200-6Y1: Shariff et al., 'Pointing control for the SPIDER balloon-borne telescope', "
    "arXiv:1407.1880, Table 1; wheel inertia 44.77 kg·m², 50.34 kg with motor",
]


def gap_mm(D):
    return 0.6 + 0.001 * (D - 90)


def build(D, layout, stackup, gap=None):
    R = D / 2
    W = 0.49 * R if (layout == "disc" or 0.49 * R <= RING_W) else RING_W
    r_in = R - W
    r_mag = 0.756 * R if layout == "disc" else R - W / 2
    n8 = max(1, round(2 * math.pi * r_mag / (POLE_PITCH * 8)))
    poles = 8 * n8
    coils = 3 * poles // 4
    st = STACKUPS[stackup]
    d = StatorDesign(f"big_{D}_{layout}_{stackup}", stackup=stackup, n_coils=coils, r_in_mm=r_in, r_out_mm=R,
                     trace_mm=TRACE, space_mm=max(0.15, st["min_space_mm"]), board_od_mm=D + 6)
    r = RotorDesign(n_poles=poles, magnet_shape="block", block_len_mm=0.95 * W, block_w_mm=MAG_W,
                    magnet_h_mm=MAG_H, magnet_r_mm=r_mag, airgap_mm=gap if gap is not None else gap_mm(D),
                    back_iron=True, back_iron_t_mm=IRON_T, disc_od_mm=D, disc_t_mm=0.0)
    return d, r, W


def evaluate(args):
    D, layout, label, stackup, gap = args
    d, r, W = build(D, layout, stackup, gap)
    kt_s, R_s, L_s, c_eddy, ws = stator_em(d, r, STRANDS)
    th, tq, _, _ = coil_curves(d, r, dual=False, n_theta=36)
    _, ripple, _, _ = foc_torque(d, r, th, tq)
    km = kt_s / math.sqrt(1.5 * R_s)            # independent of series/parallel wiring
    R_m, Ri_m = D / 2e3, (D / 2 - W) / 1e3
    board_t = max(d.stack["z_mm"]) + d.stack["oz"][0] * OZ_TO_M * 1e3 + 0.2
    area = math.pi * ((R_m + 0.003) ** 2 - max(0.0, Ri_m - 0.005) ** 2)
    m_board = area * board_t / 1e3 * FR4 + copper_mass_g(d) / 1e3 * (STRANDS * ws / d.trace_mm)
    m_mag = r.magnet_mass_kg()
    m_iron = STEEL * math.pi * (R_m ** 2 - Ri_m ** 2) * IRON_T / 1e3
    J_active = m_mag * (r.magnet_r_mm / 1e3) ** 2 + 0.5 * m_iron * (R_m ** 2 + Ri_m ** 2)
    w = min(RPM_CAP * 2 * math.pi / 60, V_RIM / R_m)
    return dict(D_mm=D, layout=layout, stator=label, stackup=stackup, winding_w_mm=W,
                gap_mm=r.airgap_mm, poles=r.n_poles, coils=d.n_coils,
                Km_mNm_per_sqrtW=km * 1e3, ripple_pct=ripple * 100,
                board_g=m_board * 1e3, board_area_cm2=area * 1e4, magnets_g=m_mag * 1e3, iron_g=m_iron * 1e3,
                active_g=(m_board + m_mag + m_iron) * 1e3,
                Km_per_active_kg=km / (m_board + m_mag + m_iron),
                rpm_at_40ms=w * 60 / (2 * math.pi), eddy_W_at_40ms=c_eddy * w ** 2,
                eddy_drag_mNm_at_40ms=c_eddy * w * 1e3,
                eddy_drag_mNm_at_1krpm=c_eddy * (1000 * math.pi / 30) * 1e3,
                Km_per_board_kg=km / m_board,
                H_active_Nms_at_40ms=J_active * w,
                P_cu_W_for_0p1Nm=(0.1 / km) ** 2, P_cu_W_for_1Nm=(1.0 / km) ** 2,
                n_magnets=r.n_poles)


def fit(rows, layout, stator):
    pts = [(r["D_mm"], r["Km_mNm_per_sqrtW"]) for r in rows
           if r["layout"] == layout and r["stator"] == stator and r["D_mm"] >= 150 and r.get("sens") is None]
    x, y = np.log([p[0] for p in pts]), np.log([p[1] for p in pts])
    a, b = np.polyfit(x, y, 1)
    return float(a), float(math.exp(b))


def main():
    jobs = []
    for D in DIAMS:
        for label, st in STATORS:
            jobs.append((D, "disc", label, st, None))
            if D > 2 * RING_W / 0.49:   # ring differs from disc only above ~163 mm
                jobs.append((D, "ring", label, st, None))
    sens = [(D, lay, "JLC 4L", "jlc4_2oz_out", 1.5) for D in (250, 600) for lay in ("disc", "ring")]
    with ProcessPoolExecutor(max_workers=8) as ex:
        rows = list(ex.map(evaluate, jobs))
        srows = list(ex.map(evaluate, sens))
    for s in srows:
        s["sens"] = "gap 1.5 mm"
    for r in rows:
        print(f"{r['D_mm']:4d} {r['layout']:4s} {r['stator']:11s} poles={r['poles']:3d} gap={r['gap_mm']:.2f} "
              f"Km={r['Km_mNm_per_sqrtW']:8.1f} ripple={r['ripple_pct']:4.1f}% board={r['board_g']:6.0f} g "
              f"mag={r['magnets_g']:6.0f} g active={r['active_g']:6.0f} g Km/kg={r['Km_per_active_kg']:.3f} "
              f"eddy={r['eddy_W_at_40ms']:.2f} W", flush=True)
    fits = {f"{lay}, {lab}": fit(rows, lay, lab) for lay in ("disc", "ring") for lab, _ in STATORS}
    for k, (a, c) in fits.items():
        print(f"fit {k}: Km = {c:.3g} * D^{a:.2f}  (mN·m/√W, D in mm)")
    # Extrapolate the ring to the SPIDER wheel size (2 m) for comparison with its frameless motor.
    a, c = fits["ring, JLC 4L"]
    ring2m = dict(D_mm=2000, Km_mNm_per_sqrtW=c * 2000 ** a)
    big = [r for r in rows if r["layout"] == "ring" and r["stator"] == "JLC 4L" and r["D_mm"] == 600][0]
    ring2m["active_kg"] = big["active_g"] / 1e3 * 2000 / 600
    ring2m["board_kg"] = big["board_g"] / 1e3 * 2000 / 600
    (OUT / "pcbmotor_large_wheels.json").write_text(json.dumps(
        dict(rows=rows, sensitivity=srows, fits=fits, ring_2m_extrapolated=ring2m, cots=COTS,
             cots_sources=COTS_SOURCES), indent=1))

    md = ["# PCB-motor wheels scaled up, against conventional frameless motors\n",
          "Generated by `pcbmotor/large_wheels.py` (assumptions in its docstring). Km is the motor constant "
          "(torque per root watt of copper loss), independent of how the coils are wired; Km per kg is in N·m/√W per kg. Active mass = stator "
          "board + magnets + back iron; the conventional masses below also include rotor and stator. Drag is "
          f"trace eddy loss at a {V_RIM:g} m/s rim speed ({RPM_CAP:.0f} rpm cap). Active H is the momentum of the "
          "magnets and back iron alone at that speed: rim mass a wheel needs anyway.\n",
          "| D mm | Layout | Stator | Gap mm | Poles | Km mN·m/√W | Torque ripple % | Board g | Magnets g | Active g | "
          "Km per active kg | Km per board kg | Active H N·m·s at 40 m/s | rpm at 40 m/s | Eddy W at 40 m/s | "
          "Eddy drag mN·m at 1,000 rpm | Cu W for 0.1 N·m |",
          "|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|"]
    for r in rows:
        md.append(f"| {r['D_mm']} | {r['layout']} | {r['stator']} | {r['gap_mm']:.2f} | {r['poles']} | "
                  f"{r['Km_mNm_per_sqrtW']:.0f} | {r['ripple_pct']:.1f} | {r['board_g']:.0f} | {r['magnets_g']:.0f} | "
                  f"{r['active_g']:.0f} | {r['Km_per_active_kg']:.2f} | {r['Km_per_board_kg']:.2f} | "
                  f"{r['H_active_Nms_at_40ms']:.2f} | {r['rpm_at_40ms']:.0f} | "
                  f"{r['eddy_W_at_40ms']:.2f} | {r['eddy_drag_mNm_at_1krpm']:.1f} | {r['P_cu_W_for_0p1Nm']:.3g} |")
    md.append("\nGap sensitivity (JLC 4L, 1.5 mm gap instead of the scaled gap):\n")
    md.append("| D mm | Layout | Km mN·m/√W at 1.5 mm | Km at scaled gap |")
    md.append("|---|---|---|---|")
    for s in srows:
        base = [r for r in rows if r["D_mm"] == s["D_mm"] and r["layout"] == s["layout"] and r["stator"] == "JLC 4L"][0]
        md.append(f"| {s['D_mm']} | {s['layout']} | {s['Km_mNm_per_sqrtW']:.0f} | {base['Km_mNm_per_sqrtW']:.0f} "
                  f"({base['gap_mm']:.2f} mm) |")
    md.append("\nPower-law fits over 150–600 mm (Km in mN·m/√W, D in mm):\n")
    for k, (a, c) in fits.items():
        md.append(f"- {k}: Km ≈ {c:.3g} · D^{a:.2f}")
    md.append(f"- Ring (JLC) extrapolated to 2 m (SPIDER wheel size): Km ≈ {ring2m['Km_mNm_per_sqrtW']/1e3:.1f} N·m/√W, "
              f"active mass ≈ {ring2m['active_kg']:.1f} kg (board ≈ {ring2m['board_kg']:.1f} kg). Extrapolation, not modelled.")
    md.append("\nConventional frameless motors (datasheet):\n")
    md.append("| Motor | OD mm | Axial mm | Mass kg | Km N·m/√W | Km per kg | Cogging p-p N·m (% of cont. torque) | "
              "Iron and windage drag at 1,000 rpm, mN·m (W) |")
    md.append("|---|---|---|---|---|---|---|---|")
    for c in COTS:
        drag = (f"{c['fi'] * 1e3:.0f} ({c['fi'] * 1000 * math.pi / 30:.1f} W)"
                if c["fi"] else "not published")
        cog = f"{c['cog']:.3f} ({100 * c['cog'] / c['tc']:.1f}%)" if c["cog"] else "not published"
        md.append(f"| {c['name']} | {c['od']} | {c['ax'] or '–'} | {c['kg']:.2f} | {c['km']:.3f} | "
                  f"{c['km'] / c['kg']:.2f} | {cog} | {drag} |")
    md.append("\nSources: " + "; ".join(COTS_SOURCES) + ".")
    (OUT / "pcbmotor_large_wheels.md").write_text("\n".join(md) + "\n")
    print("\n".join(md))
    plot()


def plot(data=None):
    """Km against wheel size, and Km per kilogram of parasitic (non-flywheel) motor mass."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    data = data or json.loads((OUT / "pcbmotor_large_wheels.json").read_text())
    rows, cots = data["rows"], data["cots"]
    C_JLC, C_ACE, C_CONV = "#2a78d6", "#eb6834", "#1baf7a"
    INK, INK2, GRID = "#0b0b0b", "#52514e", "#e4e3df"
    fig, axs = plt.subplots(1, 2, figsize=(10, 4.2), dpi=150, facecolor="#ffffff")
    for ax in axs:
        ax.set_facecolor("#ffffff")
        for sp in ("top", "right"):
            ax.spines[sp].set_visible(False)
        for sp in ("left", "bottom"):
            ax.spines[sp].set_color(INK2)
        ax.tick_params(colors=INK2, labelsize=8)
        ax.grid(True, which="major", color=GRID, lw=0.8)
        ax.set_axisbelow(True)
        ax.set_xscale("log")
        ax.set_xlabel("Diameter, mm (PCB: rotor; conventional: motor OD)", color=INK2, fontsize=8)
    for lab, col in (("JLC 4L", C_JLC), ("ACE 6L 3 oz", C_ACE)):
        for lay, ls, mk in (("disc", "-", "o"), ("ring", "--", "s")):
            pts = sorted((r["D_mm"], r["Km_mNm_per_sqrtW"] / 1e3, r["Km_per_board_kg"])
                         for r in rows if r["stator"] == lab and (r["layout"] == lay or (lay == "ring" and r["D_mm"] < 163)))
            x = [p[0] for p in pts]
            axs[0].plot(x, [p[1] for p in pts], ls=ls, marker=mk, ms=4, lw=2, color=col, label=f"PCB {lab}, {lay}")
            axs[1].plot(x, [p[2] for p in pts], ls=ls, marker=mk, ms=4, lw=2, color=col)
    cx = [c["od"] for c in cots]
    axs[0].scatter(cx, [c["km"] for c in cots], s=40, marker="D", color=C_CONV, edgecolor="#ffffff", lw=1.5, zorder=5,
                   label="Conventional frameless (datasheet)")
    axs[1].scatter(cx, [c["km"] / c["kg"] for c in cots], s=40, marker="D", color=C_CONV, edgecolor="#ffffff", lw=1.5,
                   zorder=5)
    ex = data["ring_2m_extrapolated"]
    axs[0].plot([600, 2000], [[r for r in rows if r["D_mm"] == 600 and r["layout"] == "ring" and r["stator"] == "JLC 4L"][0]
                              ["Km_mNm_per_sqrtW"] / 1e3, ex["Km_mNm_per_sqrtW"] / 1e3], ls=":", lw=1.5, color=C_JLC)
    axs[0].annotate("JLC ring, extrapolated\nto 2 m", (2000, ex["Km_mNm_per_sqrtW"] / 1e3), xytext=(-4, -22),
                    textcoords="offset points", ha="right", fontsize=7, color=INK2)
    axs[0].annotate("SPIDER wheel motor", (178, cots[-1]["km"]), xytext=(6, 2), textcoords="offset points",
                    fontsize=7, color=INK2)
    axs[0].annotate("KAOS-3 wheel", (90, 0.016), xytext=(6, -10), textcoords="offset points", fontsize=7, color=INK2)
    axs[0].set_yscale("log")
    axs[0].set_ylabel("Km, N·m/√W", color=INK2, fontsize=8)
    axs[0].set_title("Torque per root watt", loc="left", color=INK, fontsize=10)
    axs[1].set_ylabel("Km per kg of parasitic mass, N·m/√W/kg", color=INK2, fontsize=8)
    axs[1].set_title("Per kg that is not flywheel (PCB: board only)", loc="left", color=INK, fontsize=10)
    axs[1].set_ylim(0, None)
    from matplotlib.ticker import FixedLocator, NullLocator, FuncFormatter
    axs[0].set_xticks([60, 100, 200, 400, 600, 1000, 2000])
    axs[1].set_xticks([60, 100, 200, 400, 600])
    for ax in axs:
        ax.xaxis.set_minor_locator(NullLocator())
        ax.xaxis.set_major_formatter(FuncFormatter(lambda v, _: f"{v:g}"))
    axs[0].yaxis.set_major_formatter(FuncFormatter(lambda v, _: f"{v:g}"))
    axs[0].legend(fontsize=7, frameon=False, loc="lower right", labelcolor=INK2)
    fig.tight_layout()
    fig.savefig(OUT / "pcbmotor_large_wheels.png", facecolor="#ffffff")
    plt.close(fig)


if __name__ == "__main__":
    import sys
    plot() if "--plot" in sys.argv else main()
