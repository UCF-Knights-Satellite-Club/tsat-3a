import math
import pathlib
import sys
from dataclasses import replace

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))

from pcbmotor import bus_voltage as bv
from pcbmotor.em import _neumann
from pcbmotor.geometry import OZ_TO_M
from pcbmotor.wave import RECOMMENDED, hm_rotor, analyse_wave, template, positions, layer_shapes


def test_one_path_matches_wave_model():
    rot = hm_rotor()
    d = replace(RECOMMENDED, strands=1)
    base = analyse_wave(d, rot, n_theta=72)
    kt, R, circ, _ = bv.path_analysis(d, rot, 1)
    assert math.isclose(kt * 1e3, base["Kt_mNm_per_A"], rel_tol=1e-9)
    assert math.isclose(R, base["R_phase_ohm"], rel_tol=1e-9)
    assert circ < 1e-12


def test_two_paths_quarter_resistance_half_kt():
    rot = hm_rotor()
    d = replace(RECOMMENDED, strands=1)
    kt1, R1, _, _ = bv.path_analysis(d, rot, 1)
    kt2, R2, _, _ = bv.path_analysis(d, rot, 2)
    assert abs(kt2 / kt1 - 0.5) < 0.02            # paths L1+L4 and L2+L3 carry ~half the current each
    assert abs(R2 / R1 - 0.25) < 0.05             # 2 oz pair vs 1 oz pair: unequal, so a bit off 1/4
    # current now splits by conductance (more in the 2 oz pair) instead of being forced equal: Km rises a little
    assert 1.0 <= (kt2 / math.sqrt(R2)) / (kt1 / math.sqrt(R1)) < 1.10


def test_inductance_offset_sum_matches_brute_force():
    """The rotated-template shortcut equals the full double sum over every conductor pair."""
    d = replace(RECOMMENDED, n_per_layer=24, strands=1)
    n_poles = 8
    L, M = bv.inductance_matrices(d, n_poles, ds_mm=1.0)
    phi, ph, sgn = positions(d, n_poles)
    l, m = 0, 1
    t = (d.stack["oz"][l] + d.stack["oz"][m]) / 2 * OZ_TO_M
    wl = [float(np.mean(template(d, n_poles, s, 1.0)[2])) for s in layer_shapes(d)]
    w = (wl[l] + wl[m]) / 2 / 1e3
    dz = math.hypot(abs(d.stack["z_mm"][l] - d.stack["z_mm"][m]) / 1e3, 0.2235 * (w + t))

    def conductor(layer, a, z):
        r, th, _, _ = template(d, n_poles, layer_shapes(d)[layer], 1.0)
        return np.c_[r * np.cos(th + a), r * np.sin(th + a), np.full(len(r), z)] / 1e3

    tot_aa = tot_ab = 0.0
    for i in np.nonzero(ph == 0)[0]:
        a = conductor(l, phi[i], 0.0)
        for j in np.nonzero(ph == 0)[0]:
            tot_aa += sgn[i] * sgn[j] * _neumann(a, conductor(m, phi[j], -dz * 1e3))
        for j in np.nonzero(ph == 1)[0]:
            tot_ab += sgn[i] * sgn[j] * _neumann(a, conductor(m, phi[j], -dz * 1e3))
    assert math.isclose(L[l, m], tot_aa, rel_tol=1e-6)
    assert math.isclose(M[l, m], tot_ab, rel_tol=1e-6)
    assert L.sum() > 0 and M.sum() < 0


def test_ripple_model():
    """No ripple at zero modulation (all legs switch together), 1/(L f) scaling, and below the
    half-bridge bound V / (8 L f) peak-to-peak at every modulation depth."""
    V, L, f = 11.1, 8e-6, 50e3
    v_ph = V / math.sqrt(3)
    assert bv.ripple_rms(V, L, f, 0.0) < 1e-12
    x = bv.ripple_rms(V, L, f, 0.6 * v_ph)
    assert math.isclose(bv.ripple_rms(V, L, 2 * f, 0.6 * v_ph), x / 2, rel_tol=1e-9)
    assert math.isclose(bv.ripple_rms(V, 2 * L, f, 0.6 * v_ph), x / 2, rel_tol=1e-9)
    bound_rms = V / (8 * L * f) / math.sqrt(12)
    assert all(bv.ripple_rms(V, L, f, m * v_ph) < bound_rms for m in np.linspace(0, 1, 11))


if __name__ == "__main__":
    for name, fn in list(globals().items()):
        if name.startswith("test_"):
            fn()
            print("ok", name)
