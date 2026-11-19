"""Comparison charts across the evaluated PCB stator stackups and layouts.

Run from sim/ after the studies it reads:   .venv/bin/python -m pcbmotor.compare_plots

Reads results/pcbmotor_ace_stackups.json, results/pcbmotor_strand_test.json and
results/pcbmotor_stacking.json, and writes:

- results/pcbmotor_compare_stackups.png  every stackup, solid against stranded, on efficiency (Km),
  copper loss for the 26 degree swing, usable momentum and eddy loss
- results/pcbmotor_compare_strand_test.png  the two strand test boards as laid out: stator drag
  against speed, strand-to-strand back-EMF (transposed against untransposed) and Km
- results/pcbmotor_compare_layouts.png  board layouts for the payload stator: one board, stacked
  boards in series / parallel, dual rotor, more layers

All three use the payload rotor (high-momentum design), so only the stator differs.
"""
from __future__ import annotations

import json
import math
import pathlib
import re

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt                       # noqa: E402
from matplotlib.lines import Line2D                   # noqa: E402
from matplotlib.ticker import FuncFormatter           # noqa: E402

PLAIN = FuncFormatter(lambda v, _: f"{v:g}")          # 0.1, 1, 10 instead of 10^-1

OUT = pathlib.Path(__file__).resolve().parents[1] / "results"

# Reference categorical palette (validated light-mode, first three slots pass all-pairs), and
# neutral text / grid inks. Aqua is below 3:1 on the surface, so every series is also labelled.
SURFACE = "#fcfcfb"
INK, INK2, MUTED, GRID = "#0b0b0b", "#52514e", "#8a8984", "#e4e3df"
BLUE, ORANGE, AQUA = "#2a78d6", "#eb6834", "#1baf7a"
SOLID_C, STRAND_C, UNTR_C = BLUE, ORANGE, AQUA

plt.rcParams.update({
    "figure.facecolor": SURFACE, "axes.facecolor": SURFACE, "savefig.facecolor": SURFACE,
    "axes.edgecolor": GRID, "axes.labelcolor": INK2, "axes.titlecolor": INK, "axes.titlesize": 11,
    "axes.titleweight": "bold", "axes.titlelocation": "left", "axes.labelsize": 9,
    "xtick.color": INK2, "ytick.color": INK2, "xtick.labelsize": 8.5, "ytick.labelsize": 8.5,
    "axes.grid": True, "grid.color": GRID, "grid.linewidth": 0.8, "axes.axisbelow": True,
    "axes.spines.top": False, "axes.spines.right": False, "font.size": 9, "legend.frameon": False,
    "legend.fontsize": 8.5, "text.color": INK,
})

CURRENT = "jlc4_2oz_out"      # the payload design today
TEST_BOARD = "jlc4_2oz"       # the strand test boards


def _save(fig, ax0, path):
    """Line the figure title, caption and legend up with the left edge of the first panel (its tick
    labels and axis label), so the cropped image has no empty strip on the left, then save."""
    fig.canvas.draw()
    x0 = ax0.get_tightbbox(fig.canvas.get_renderer()).transformed(fig.transFigure.inverted()).x0
    for txt in list(fig.texts) + ([fig._suptitle] if fig._suptitle is not None else []):
        txt.set_x(x0)
        txt.set_ha("left")
    for leg in fig.legends:
        bb = leg.get_bbox_to_anchor().transformed(fig.transFigure.inverted())
        leg.set_bbox_to_anchor((x0, bb.y0), transform=fig.transFigure)
    fig.savefig(path, dpi=150, bbox_inches="tight")


def _stackup_name(config):
    """'ACE 6L, 3 oz, 3 strands' -> 'ACE 6L, 3 oz'."""
    s = re.sub(r",\s*\d strands", "", config)
    return s.replace(" (current design)", "")


def stackup_rows():
    rows = json.loads((OUT / "pcbmotor_ace_stackups.json").read_text())
    by = {}
    for r in rows:
        st = r["stackup"]
        e = by.setdefault(st, dict(stackup=st, name=_stackup_name(r["config"]), solid=None, stranded=None))
        if r["strands"] == 1:
            e["solid"] = r
        elif e["stranded"] is None or r["Km_mNm_per_sqrtW"] > e["stranded"]["Km_mNm_per_sqrtW"]:
            e["stranded"] = r              # best stranded version (2L heavy copper: 2 strands beat 3)
    out = list(by.values())
    out.sort(key=lambda e: (e["stranded"] or e["solid"])["Km_mNm_per_sqrtW"])
    return out


def _label(e):
    lab = e["name"]
    if e["stranded"] and e["stranded"]["strands"] != 3:
        lab += f"  ({e['stranded']['strands']} strands)"
    if e["stackup"] == CURRENT:
        lab += "  ← current"
    elif e["stackup"] == TEST_BOARD:
        lab += "  ← strand test boards"
    return lab


def plot_stackups():
    rows = stackup_rows()
    metrics = [
        ("Km_mNm_per_sqrtW", "Efficiency, Km", "mN·m/√W  (higher is better)", False),
        ("Pcu_W_swing_26deg", "Copper loss, 26° swing", "W  (lower is better)", False),
        ("H_usable_mNms", "Usable momentum", "mN·m·s  (higher is better)", False),
        ("eddy_W_at_2000rpm", "Eddy loss at 2,000 rpm", "W, log scale  (lower is better)", True),
    ]
    n = len(rows)
    fig, axs = plt.subplots(1, 4, figsize=(15, 0.36 * n + 2.0), sharey=True,
                            gridspec_kw=dict(wspace=0.08))
    ys = list(range(n))
    for ax, (key, title, xlabel, logx) in zip(axs, metrics):
        for y, e in zip(ys, rows):
            a = e["solid"][key] if e["solid"] else None
            b = e["stranded"][key] if e["stranded"] else None
            if a is not None and b is not None:
                ax.plot([a, b], [y, y], color=GRID, lw=2, zorder=1, solid_capstyle="round")
            if a is not None:
                ax.scatter([a], [y], s=46, color=SOLID_C, edgecolor=SURFACE, linewidth=1.5, zorder=3)
            if b is not None:
                ax.scatter([b], [y], s=46, color=STRAND_C, edgecolor=SURFACE, linewidth=1.5, zorder=3)
        ax.set_title(title)
        ax.set_xlabel(xlabel)
        ax.grid(axis="y", visible=False)
        if logx:
            ax.set_xscale("log")
            ax.xaxis.set_major_formatter(PLAIN)
        if key == "H_usable_mNms":
            cap = max((e["stranded"] or e["solid"])[key] for e in rows)
            ax.axvline(cap, color=MUTED, lw=1, ls=(0, (3, 3)), zorder=0)
            ax.text(cap, n - 0.4, "6,000 rpm cap ", ha="right", va="bottom", fontsize=8, color=INK2)
            ax.set_xlim(0, cap * 1.12)
        if key == "Km_mNm_per_sqrtW":
            ax.set_xlim(0, None)
        if key == "Pcu_W_swing_26deg":
            ax.set_xlim(0, None)
        for y, e in zip(ys, rows):        # highlight the current design and the test-board stackup
            if e["stackup"] in (CURRENT, TEST_BOARD):
                ax.axhspan(y - 0.45, y + 0.45, color="#f0efec", zorder=0, lw=0)
    axs[0].set_yticks(ys)
    axs[0].set_yticklabels([_label(e) for e in rows])
    for t, e in zip(axs[0].get_yticklabels(), rows):
        if e["stackup"] in (CURRENT, TEST_BOARD):
            t.set_color(INK)
            t.set_fontweight("bold")
    axs[0].set_ylim(-0.7, n - 0.1)
    handles = [Line2D([], [], marker="o", ls="", color=SOLID_C, markersize=7, label="Solid 1.2 mm trace"),
               Line2D([], [], marker="o", ls="", color=STRAND_C, markersize=7,
                      label="Stranded, transposed (3 strands unless noted)")]
    fig.subplots_adjust(top=0.86)
    fig.legend(handles=handles, loc="lower left", bbox_to_anchor=(0.0, 0.905), ncol=2)
    fig.suptitle("PCB stator stackups, payload rotor: solid against stranded windings",
                 x=0.0, ha="left", y=0.985, fontsize=13, fontweight="bold")
    fig.text(0.0, -0.02, "From pcbmotor/ace_stackups.py: 1.2 mm bundle, 11.1 V and 1 A driver, usable speed keeps "
             "6 mN·m after eddy drag. ACE stackups and their spacing rules are assumptions. Sorted by "
             "stranded Km. Stackups with only an orange dot were run stranded only.", fontsize=8, color=INK2, ha="left")
    p = OUT / "pcbmotor_compare_stackups.png"
    _save(fig, axs[0], p)
    plt.close(fig)
    return p


def plot_strand_test():
    j = json.loads((OUT / "pcbmotor_strand_test.json").read_text())
    s, t = j["boards"]["solid"], j["boards"]["stranded"]
    fig, axs = plt.subplots(1, 3, figsize=(16, 4.6), gridspec_kw=dict(width_ratios=[1.5, 1.2, 0.8], wspace=0.5))

    # (a) stator drag against speed (phases open)
    ax = axs[0]
    rpm = list(range(250, 4001, 250))
    w = [r * math.pi / 30 for r in rpm]
    untr_c = t["c_eddy"] + t["untransposed_c_circ"]
    series = [("Solid 1 × 1.2 mm", s["c_total"], SOLID_C),
              (f"Stranded 3 × {t['strand_width_mm']:.2f} mm, untransposed (not built)", untr_c, UNTR_C),
              (f"Stranded 3 × {t['strand_width_mm']:.2f} mm, transposed", t["c_total"], STRAND_C)]
    for lab, c, col in series:
        ax.plot(rpm, [c * x * 1e3 for x in w], color=col, lw=2, label=lab)
    ax.set_yscale("log")
    ax.yaxis.set_major_formatter(PLAIN)
    ax.set_xlim(250, 4000)
    ax.annotate("at 4,000 rpm", (4000, s["c_total"] * w[-1] * 1e3), xytext=(4, 12), textcoords="offset points",
                fontsize=8, color=INK2)
    for lab, c, col in series:            # direct labels at the right end
        y = c * w[-1] * 1e3
        ax.annotate(f"{y:.2f} mN·m", (4000, y), xytext=(4, 0), textcoords="offset points", va="center",
                    fontsize=8.5, color=INK)
    ax.set_title("Stator drag with the phases open")
    ax.set_xlabel("Rotor speed, rpm")
    ax.set_ylabel("Drag torque, mN·m (log scale)")
    ax.legend(loc="lower right")

    # (b) strand-to-strand back-EMF with J2 open (one coil), transposed against untransposed
    ax = axs[1]
    names = list(t["jumper_J2_open"])
    tr = [t["jumper_J2_open"][k]["V_mV_rms_per_krpm"] for k in names]
    un = [t["untransposed_jumper"][k]["V_mV_rms_per_krpm"] for k in names]
    x = range(len(names))
    bw = 0.36
    b1 = ax.bar([i - bw / 2 - 0.01 for i in x], tr, bw, color=STRAND_C, label="Transposed (as built)")
    b2 = ax.bar([i + bw / 2 + 0.01 for i in x], un, bw, color=UNTR_C, label="Untransposed (not built)")
    for bars in (b1, b2):
        for b in bars:
            ax.annotate(f"{b.get_height():.1f}", (b.get_x() + b.get_width() / 2, b.get_height()),
                        xytext=(0, 2), textcoords="offset points", ha="center", fontsize=8, color=INK)
    ax.set_yscale("log")
    ax.yaxis.set_major_formatter(PLAIN)
    ax.set_xticks(list(x))
    ax.set_xticklabels([k.replace(" (strands", "\n(strands") for k in names], fontsize=8)
    ax.set_ylabel("mV rms per 1,000 rpm (log scale)")
    ax.set_title("J2 open: voltage between strands")
    ax.grid(axis="x", visible=False)
    ax.legend(loc="upper left")
    ax.set_ylim(1, 300)

    # (c) Km
    ax = axs[2]
    vals = [("Solid", s["Km_mNm_per_sqrtW"], SOLID_C), ("Stranded", t["Km_mNm_per_sqrtW"], STRAND_C)]
    bars = ax.bar([v[0] for v in vals], [v[1] for v in vals], 0.55, color=[v[2] for v in vals])
    for b in bars:
        ax.annotate(f"{b.get_height():.1f}", (b.get_x() + b.get_width() / 2, b.get_height()),
                    xytext=(0, 2), textcoords="offset points", ha="center", fontsize=8.5, color=INK)
    ax.set_title("Efficiency, Km")
    ax.set_ylabel("mN·m/√W")
    ax.grid(axis="x", visible=False)

    fig.suptitle(f"Strand test boards as laid out (JLC 4L, 2 oz all layers, payload rotor): "
                 f"{s['c_total'] / t['c_total']:.0f}× less drag stranded", x=0.0, ha="left", y=1.03,
                 fontsize=13, fontweight="bold")
    fig.text(0.0, -0.06, "From pcbmotor/strand_test.py on the as-built copper (pcbmotor/bundle.py). Drag excludes "
             "the bearing. Untransposed = each strand on the same lane on every layer, which this layout "
             "can't build; shown to size what the lane swaps buy.", fontsize=8, color=INK2, ha="left")
    p = OUT / "pcbmotor_compare_strand_test.png"
    _save(fig, axs[0], p)
    plt.close(fig)
    return p


def plot_layouts():
    rows = json.loads((OUT / "pcbmotor_stacking.json").read_text())["payload_high_momentum"]
    pretty = {
        "single (as designed)": "One board (as designed)",
        "stack2_series": "2 boards stacked, series",
        "stack2_parallel": "2 boards stacked, parallel",
        "stack3_series": "3 boards stacked, series",
        "stack3_parallel": "3 boards stacked, parallel",
        "dual1 (1 board, 2 rotors)": "1 board, 2 rotors",
        "dual2_series": "2 boards, 2 rotors, series",
        "dual2_parallel": "2 boards, 2 rotors, parallel",
    }
    lab = lambda c: pretty.get(c, c.replace("more layers: jlc6_1oz", "One 6-layer board (JLC 1 oz)"))
    rows = sorted(rows, key=lambda r: r["Km_mNm_per_sqrtW"])
    n = len(rows)
    ys = list(range(n))
    metrics = [("Km_mNm_per_sqrtW", "Efficiency, Km", "mN·m/√W", BLUE),
               ("H_with_6mNm_margin_mNms", "Usable momentum", "mN·m·s, 6 mN·m left after eddy drag", BLUE),
               ("moving_plus_board_g", "Rotor + stator mass", "g", BLUE)]
    fig, axs = plt.subplots(1, 3, figsize=(14, 0.42 * n + 1.8), sharey=True, gridspec_kw=dict(wspace=0.1))
    for ax, (key, title, xl, col) in zip(axs, metrics):
        v = [r[key] for r in rows]
        ax.barh(ys, v, height=0.6, color=col)
        for y, x in zip(ys, v):
            ax.annotate(f"{x:.0f}" if x >= 30 else f"{x:.1f}", (x, y), xytext=(3, 0), textcoords="offset points",
                        va="center", fontsize=8, color=INK)
        ax.set_title(title)
        ax.set_xlabel(xl)
        ax.grid(axis="y", visible=False)
        ax.set_xlim(0, max(v) * 1.15)
    axs[0].set_yticks(ys)
    axs[0].set_yticklabels([lab(r["config"]) for r in rows])
    fig.suptitle("Payload stator board layouts (JLC 4L 2 oz out / 1 oz in, solid 1.2 mm traces)", x=0.0, ha="left",
                 y=1.02, fontsize=13, fontweight="bold")
    fig.text(0.0, -0.04, "From pcbmotor/stacking.py: 11.1 V and 1 A driver; parallel boards include the "
             "circulating current between boards at different depths. Solid traces throughout, so eddy drag "
             "limits momentum in every layout.", fontsize=8, color=INK2, ha="left")
    p = OUT / "pcbmotor_compare_layouts.png"
    _save(fig, axs[0], p)
    plt.close(fig)
    return p


def main():
    for f in (plot_stackups, plot_strand_test, plot_layouts):
        print("wrote", f())


if __name__ == "__main__":
    main()
