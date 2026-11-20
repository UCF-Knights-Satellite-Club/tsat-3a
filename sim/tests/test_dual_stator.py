import math
import pathlib
import sys
from dataclasses import replace

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))

from pcbmotor import dual_stator as ds, em, wave
from pcbmotor.wave import hm_rotor


def test_bare_magnet_field_is_symmetric_about_the_rotor():
    ds.check_symmetry()


def test_double_magnets_without_iron_match_back_iron_field():
    rot = hm_rotor()
    pts = np.c_[np.linspace(0.02, 0.042, 7), np.zeros(7), np.full(7, -0.0008)]
    b_iron = em.rotor_magnets(rot, 0.2).getB(pts)
    with ds.rotor_field(back_iron=False, h_mult=2.0):
        b_bare = wave.rotor_magnets(rot, 0.2).getB(pts)
    assert np.allclose(b_iron, b_bare, rtol=1e-9, atol=1e-12)


def test_rotor_field_is_restored_after_use():
    before = [m.rotor_magnets for m in (em, wave)]
    with ds.rotor_field(back_iron=False):
        pass
    assert [m.rotor_magnets for m in (em, wave)] == before


def test_series_and_parallel_have_the_same_km():
    b = dict(Kt_mNm_per_A=20.0, R_phase_ohm=1.0, eddy_W_at_2000rpm=0.1, circulating_W_at_2000rpm=0.0)
    rot = dict(J_kgm2=2e-4, mass_g=200.0, magnet_g=90.0)
    s = ds.combine("s", "d", b, 2, True, rot, 90.0)
    p = ds.combine("p", "d", b, 2, False, rot, 90.0)
    one = ds.combine("1", "d", b, 1, True, rot, 90.0)
    assert math.isclose(s["Km_mNm_per_sqrtW"], p["Km_mNm_per_sqrtW"])
    assert math.isclose(s["Km_mNm_per_sqrtW"], math.sqrt(2) * one["Km_mNm_per_sqrtW"])
    assert math.isclose(2 * s["noload_rpm"], p["noload_rpm"])
    assert math.isclose(s["eddy_W_2000"], 2 * one["eddy_W_2000"])
