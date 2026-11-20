"""Layout images of every evaluated stator: the recommended spiral designs (pcbmotor_designs.json) and
every design in the wave-winding study (pcbmotor_wave.json).

Traces are drawn at their true copper width, coloured by phase (A red, B green, C blue); full-length
strands are drawn as separate lanes, swapped on L2 where the design swaps them; slit designs split only
the radial section. Magnets are dashed grey outlines, the board edge is black.

Each image has three panels: L1 (and L3) over the whole board, L2 (and L4) over the whole board, and a
zoom on one pole with L1 solid and L2 faint. Spiral coils are the same on every layer, so their L2
panel is replaced by the coil/phase map.

Writes results/layouts/<design>.png and results/layouts/README.md (an index with each design's key
numbers). Run from sim/:  .venv/bin/python -m pcbmotor.layout_gallery
"""
from __future__ import annotations

import json
import math
import pathlib

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from .geometry import StatorDesign, RotorDesign, coil_turns, coil_angles, phase_map
from . import wave

RES = pathlib.Path(__file__).resolve().parents[1] / "results"
OUT = RES / "layouts"
PH_COL = ["tab:red", "tab:green", "tab:blue"]
DPI = 90


# ---------------------------------------------------------------- drawing helpers
def _draw_path(ax, x, y, w_mm, color, alpha=1.0, z=2):
    """Trace of physical width w_mm (scalar or per point) along a polyline, drawn as a filled ribbon:
    the centreline offset by +-w/2 along its normal."""
    p = np.c_[x, y]
    if len(p) < 2:
        return
    closed = np.allclose(p[0], p[-1])
    t = np.gradient(p, axis=0)
    if closed:
        t[0] = t[-1] = p[1] - p[-2]
    t /= np.linalg.norm(t, axis=1, keepdims=True) + 1e-12
    n = np.c_[-t[:, 1], t[:, 0]]
    h = np.broadcast_to(np.asarray(w_mm, float), (len(p),))[:, None] / 2
    ring = np.vstack([p + n * h, (p - n * h)[::-1]])
    ax.fill(ring[:, 0], ring[:, 1], color=color, alpha=alpha, lw=0, zorder=z)


def _save(fig, path):
    """Save, then reduce to a 64-colour palette PNG (the drawings use a handful of colours; ~5x smaller)."""
    from PIL import Image
    fig.savefig(path, dpi=DPI)
    plt.close(fig)
    im = Image.open(path).convert("RGB").quantize(colors=64, method=Image.Quantize.MEDIANCUT,
                                                  dither=Image.Dither.NONE)
    im.save(path, optimize=True)


def _magnets(ax, rot: RotorDesign):
    for k in range(rot.n_poles):
        a = k * 2 * math.pi / rot.n_poles
        if rot.magnet_shape == "block":
            L, W = rot.block_len_mm, rot.block_w_mm
            pts = np.array([[-L / 2, -W / 2], [L / 2, -W / 2], [L / 2, W / 2], [-L / 2, W / 2], [-L / 2, -W / 2]])
            pts[:, 0] += rot.magnet_r_mm
        else:
            t = np.linspace(0, 2 * math.pi, 60)
            pts = np.c_[rot.magnet_r_mm + rot.magnet_d_mm / 2 * np.cos(t), rot.magnet_d_mm / 2 * np.sin(t)]
        x = pts[:, 0] * math.cos(a) - pts[:, 1] * math.sin(a)
        y = pts[:, 0] * math.sin(a) + pts[:, 1] * math.cos(a)
        ax.plot(x, y, "--", color="0.45", lw=0.9, zorder=5)


def _board(ax, board_od_mm, square):
    h = board_od_mm / 2
    if square:
        ax.plot([-h, h, h, -h, -h], [-h, -h, h, h, -h], "k", lw=1)
    else:
        t = np.linspace(0, 2 * math.pi, 200)
        ax.plot(h * np.cos(t), h * np.sin(t), "k", lw=1)


def _setup(ax, lim, title):
    ax.set_aspect("equal")
    ax.set_xlim(*lim[0])
    ax.set_ylim(*lim[1])
    ax.set_title(title, fontsize=9)
    ax.tick_params(labelsize=7)
    ax.set_xlabel("mm", fontsize=7)


# ---------------------------------------------------------------- wave designs
def _wave_strand_paths(d: wave.WaveDesign, rot, sigma, layer_idx):
    """[(r, theta, width)] for one conductor template on one layer: one entry per strand (or the
    solid conductor)."""
    r, th, w, avail = wave.template(d, rot.n_poles, sigma)
    k = d.strands
    if k < 2:
        return [(r, th, w)]
    delta = 2 * math.pi / d.n_per_layer
    cos_psi = np.clip((avail + d.space_mm) / (r * delta), 1e-3, 1.0)
    w_cu, w_s = wave.copper_widths(d, w, r)
    out = []
    if d.slit:
        split = (r >= d.r_a_mm - 1e-9) & (r < d.r_b_mm - 1e-9)
        out.append((r[~split & (r < d.r_a_mm)], th[~split & (r < d.r_a_mm)], w[~split & (r < d.r_a_mm)]))
        out.append((r[~split & (r >= d.r_b_mm)], th[~split & (r >= d.r_b_mm)], w[~split & (r >= d.r_b_mm)]))
        rr, tt, ww, wss = r[split], th[split], w[split], w_s[split]
        for j in range(k):
            x = -ww / 2 + wss / 2 + j * (wss + d.space_mm)
            out.append((rr, tt + x / rr, wss))
        return out
    for j in range(k):
        lane = (k - 1 - j) if (d.swap and layer_idx % 2 == 1) else j
        x = -w / 2 + w_s / 2 + lane * (w_s + d.space_mm)
        out.append((r, th + x / (r * cos_psi), w_s))
    return out


def _draw_wave_layer(ax, d, rot, layer_idx, alpha=1.0, only_phase=None):
    sigma = wave.layer_shapes(d)[layer_idx]
    phi, ph, sgn = wave.positions(d, rot.n_poles)
    paths = _wave_strand_paths(d, rot, sigma, layer_idx)
    for p_, k in zip(phi, ph):
        if only_phase is not None and k != only_phase:
            continue
        for r, th, w in paths:
            if len(r) < 2:
                continue
            _draw_path(ax, r * np.cos(th + p_), r * np.sin(th + p_), w, PH_COL[k], alpha)


def plot_wave(row, rot, path):
    d = wave.WaveDesign(row["name"], n_per_layer=row["n_per_layer"], r_in_mm=row["envelope_mm"][0],
                        r_a_mm=row["active_mm"][0], r_b_mm=row["active_mm"][1], r_out_mm=row["envelope_mm"][1],
                        trace_mm=None if row["trace"] == "fill" else row["trace"], cap_mm=row.get("cap_mm"),
                        strands=row.get("strands", 1), slit=row.get("slit", False), swap=row.get("swap", True),
                        connection=row.get("connection", "series"))
    fig, axes = plt.subplots(1, 3, figsize=(16, 5.6))
    full = ((-47, 47), (-47, 47))
    stitched = d.connection == "stitched"
    titles = ["L1 (and L3)" + (": radial sections, all layers stitched" if stitched else ""),
              "L2 (and L4)" + (": same copper as L1" if stitched else ""), "Zoom: one pole, L1 solid, L2 faint"]
    tau = 2 * math.pi / rot.n_poles
    zx = 46 * math.cos(tau / 2)
    zoom = ((min(d.r_in_mm * math.cos(tau), 8) - 1, 47), (-2, 47 * math.sin(1.25 * tau)))
    for ax, lim, t in zip(axes, (full, full, zoom), titles):
        _setup(ax, lim, t)
    for li, ax in ((0, axes[0]), (1, axes[1])):
        _board(ax, 90.0, True)
        _draw_wave_layer(ax, d, rot, li)
        _magnets(ax, rot)
    _draw_wave_layer(axes[2], d, rot, 1, alpha=0.3)
    _draw_wave_layer(axes[2], d, rot, 0)
    _magnets(axes[2], rot)
    for rad in (d.r_in_mm, d.r_out_mm):
        t = np.linspace(0, 2 * math.pi, 300)
        for ax in axes:
            ax.plot(rad * np.cos(t), rad * np.sin(t), ":", color="0.7", lw=0.6)
    strands = (f", {d.strands} strands {'slit' if d.slit else ('swapped at vias' if d.swap else 'fixed lanes')}"
               if d.strands > 1 else "")
    width = "fill" if d.trace_mm is None and d.cap_mm is None else (
        f"width <= {d.cap_mm:g} mm" if d.cap_mm else f"{d.trace_mm:g} mm")
    ok = "" if row["layout_ok"] else "   [FAILS layout check]"
    geom = (f"radial sections r {d.r_a_mm:g}-{d.r_b_mm:g} mm, layers in parallel (tangential end arcs not drawn)"
            if stitched else f"vias r {d.r_in_mm:g}/{d.r_out_mm:g} mm, active r {d.r_a_mm:g}-{d.r_b_mm:g} mm")
    fig.suptitle(f"{row['name']}{ok}\nwave, {d.n_per_layer} conductors/layer (q={row['q']}), {geom}, {width}{strands}  |  "
                 f"Kt {row['Kt_mNm_per_A']:.1f} mN·m/A, Km {row['Km_mNm_per_sqrtW']:.1f}, "
                 f"loss @15 mN·m/2000 rpm {row['loss_15mNm_2000rpm_W']:.2f} W", fontsize=10)
    fig.subplots_adjust(top=0.84, wspace=0.18, left=0.04, right=0.99, bottom=0.08)
    _save(fig, path)


# ---------------------------------------------------------------- spiral designs
def plot_spiral(name, d: StatorDesign, rot: RotorDesign, row, path):
    fig, axes = plt.subplots(1, 3, figsize=(16, 5.6))
    h = d.board_od_mm / 2 + 2
    full = ((-h, h), (-h, h))
    tau = 2 * math.pi / d.n_coils
    half = d.r_out_mm * math.sin(tau / 2) + 2
    zoom = ((d.r_in_mm * math.cos(tau / 2) - 3, d.r_out_mm + 3), (-half, half))
    _setup(axes[0], full, f"Every layer ({d.layers}), traces at {d.trace_mm:g} mm")
    _setup(axes[1], full, "Coil / phase map")
    _setup(axes[2], zoom, "Zoom: one coil")
    pm = phase_map(d, rot.n_poles)
    turns = coil_turns(d)
    for ax in (axes[0], axes[2]):
        _board(ax, d.board_od_mm, d.board_od_mm >= 80)
        for (ph, sgn), phi in zip(pm, coil_angles(d)):
            c, s = math.cos(phi), math.sin(phi)
            for poly in turns:
                _draw_path(ax, poly[:, 0] * c - poly[:, 1] * s, poly[:, 0] * s + poly[:, 1] * c, d.trace_mm, PH_COL[ph])
        _magnets(ax, rot)
    _board(axes[1], d.board_od_mm, d.board_od_mm >= 80)
    for (ph, sgn), phi in zip(pm, coil_angles(d)):
        c, s = math.cos(phi), math.sin(phi)
        outer = turns[0]
        axes[1].fill(outer[:, 0] * c - outer[:, 1] * s, outer[:, 0] * s + outer[:, 1] * c, color=PH_COL[ph], alpha=0.35)
        rm = 0.5 * (d.r_in_mm + d.r_out_mm)
        axes[1].text(rm * c, rm * s, "ABC"[ph] + ("+" if sgn > 0 else "−"), ha="center", va="center", fontsize=11,
                     weight="bold")
    _magnets(axes[1], rot)
    extra = ""
    if row:
        extra = (f"  |  Kt {row['Kt_mNm_per_A']:.1f} mN·m/A, Km {row['Km_mNm_per_sqrtW']:.1f}"
                 + (f", loss @15 mN·m/2000 rpm {row['loss_15mNm_2000rpm_W']:.2f} W" if "loss_15mNm_2000rpm_W" in row else ""))
    fig.suptitle(f"{name}\nspiral, {d.n_coils} coils / {rot.n_poles} poles, {d.layers} layers ({d.stackup or 'OSH Park'}), "
                 f"{len(turns)} turns/layer/coil, winding r {d.r_in_mm:g}-{d.r_out_mm:g} mm{extra}", fontsize=10)
    fig.subplots_adjust(top=0.84, wspace=0.18, left=0.04, right=0.99, bottom=0.08)
    _save(fig, path)


# ---------------------------------------------------------------- main
def _group(name):
    if name.startswith("wave_stitched"):
        return "Wave, CMU-style stitched layers"
    if "swap" in name or "fixed" in name:
        return "Wave, full-length strands (swapped or fixed lanes)"
    if "slit" in name:
        return "Wave, slit radial sections"
    if "_cap" in name:
        return "Wave, width capped"
    if name.endswith("_fill"):
        return "Wave, full fill"
    return "Spiral"


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    designs = json.loads((RES / "pcbmotor_designs.json").read_text())
    rows = json.loads((RES / "pcbmotor_wave.json").read_text())
    index = []
    for key, v in designs.items():
        d = StatorDesign(**v["stator"])
        rot = RotorDesign(**v["rotor"])
        res = v["result"]
        km = res["Kt_Nm_per_A"] / math.sqrt(1.5 * res["R_phase_ohm"]) * 1e3
        row = dict(Kt_mNm_per_A=res["Kt_Nm_per_A"] * 1e3, Km_mNm_per_sqrtW=km)
        if key == "payload_high_momentum":
            row = next(r for r in rows if r["kind"] == "spiral")
        fn = f"spiral_{key}.png"
        plot_spiral(f"spiral_{key} ({v['stator']['name']})", d, rot, row, OUT / fn)
        index.append(("Spiral", fn, f"spiral_{key}", row, True))
        print(fn, flush=True)
    rot = wave.hm_rotor()
    for r in rows:
        if r["kind"] == "spiral":
            continue
        fn = f"{r['name']}.png"
        plot_wave(r, rot, OUT / fn)
        index.append((_group(r["name"]), fn, r["name"], r, r["layout_ok"]))
        print(fn, flush=True)
    lines = ["# Stator layout gallery", "",
             "Generated by `pcbmotor/layout_gallery.py` from `pcbmotor_designs.json` and `pcbmotor_wave.json`.",
             "Traces at true width, coloured by phase (A red, B green, C blue); magnets dashed grey.",
             "Loss is copper + eddy + circulating at 15 mN·m and 2000 rpm (wave study and the high-momentum",
             "spiral only). The spiral test/payload designs use other rotors; see RESULTS.md section 1.", ""]
    groups = ["Spiral", "Wave, width capped", "Wave, full-length strands (swapped or fixed lanes)",
              "Wave, full fill", "Wave, slit radial sections", "Wave, CMU-style stitched layers"]
    for g in groups:
        items = [it for it in index if it[0] == g]
        if not items:
            continue
        lines += [f"## {g}", "", "| Design | Layout OK | Kt mN·m/A | Km mN·m/√W | Loss W |", "|---|---|---|---|---|"]
        for _, fn, name, row, ok in items:
            loss = f"{row['loss_15mNm_2000rpm_W']:.2f}" if "loss_15mNm_2000rpm_W" in row else "-"
            lines.append(f"| [{name}]({fn}) | {'yes' if ok else 'no'} | {row['Kt_mNm_per_A']:.1f} | "
                         f"{row['Km_mNm_per_sqrtW']:.1f} | {loss} |")
        lines.append("")
    (OUT / "README.md").write_text("\n".join(lines))


if __name__ == "__main__":
    main()
