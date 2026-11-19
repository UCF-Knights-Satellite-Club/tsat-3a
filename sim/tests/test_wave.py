"""Checks for the wave-winding model (pcbmotor/wave.py)."""
import math
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))

from pcbmotor import wave
from pcbmotor.em import rotor_magnets


def small():
    d = wave.WaveDesign("t", n_per_layer=24, r_in_mm=15.0, r_a_mm=21.0, r_b_mm=41.0, r_out_mm=44.0, trace_mm=None)
    return d, wave.hm_rotor()


def test_positions_balanced():
    d, rot = small()
    phi, ph, sgn = wave.positions(d, rot.n_poles)
    for k in range(3):
        assert np.sum(ph == k) == d.n_per_layer // 3
        assert np.sum(sgn[ph == k]) == 0          # as many outward as inward conductors


def test_template_shift_matches_direct_sum():
    """Phase A torque from shifted template curves equals the torque summed over every conductor
    of phase A, placed explicitly."""
    d, rot = small()
    thetas, layers = wave.template_curves(d, rot, n_theta=72)
    th_eval = np.array([0.1, 0.37, 0.8])
    kph = wave.phase_curves(d, rot, thetas, layers, th_eval, [1.0] * len(layers))
    phi, ph, sgn = wave.positions(d, rot.n_poles)
    for i, a in enumerate(th_eval):
        mags = rotor_magnets(rot, a)
        total = 0.0
        for z, sigma in zip(d.stack["z_mm"], wave.layer_shapes(d)):
            r, th, _, _ = wave.template(d, rot.n_poles, sigma)
            for p_, k, s in zip(phi, ph, sgn):
                if k != 0:
                    continue
                mids, dls = wave._segments(r, th + p_, z)
                F = np.cross(s * dls, mags.getB(mids))
                total -= np.sum(mids[:, 0] * F[:, 1] - mids[:, 1] * F[:, 0])
        assert math.isclose(kph[0, i], total, rel_tol=0.02, abs_tol=2e-4), (kph[0, i], total)


def test_fill_width_fits_pitch():
    d, rot = small()
    for s in (1, -1):
        r, th, w, avail = wave.template(d, rot.n_poles, s)
        assert np.allclose(w, avail)
        # radial section: width + space equals the slot pitch
        mask = (r > d.r_a_mm + 0.1) & (r < d.r_b_mm - 0.1)
        assert np.allclose(w[mask] + d.space_mm, r[mask] * 2 * math.pi / d.n_per_layer)


if __name__ == "__main__":
    test_positions_balanced()
    test_fill_width_fits_pitch()
    test_template_shift_matches_direct_sum()
    print("ok")
