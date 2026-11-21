"""The eddy model's field term: em.mean_dbz_dtheta2 (fine azimuthal average, 2026-11-20) must match a
brute-force central difference over many rotor angles, and must not depend on the caller's angle count."""
import math, sys, pathlib
import numpy as np
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
from pcbmotor.em import rotor_magnets, mean_dbz_dtheta2, rotor_bz
from pcbmotor.wave import hm_rotor


def _brute(rot, pts, n):
    P = 4 * math.pi / rot.n_poles
    th = np.linspace(0, P, n, endpoint=False)
    bz = np.array([rotor_magnets(rot, a).getB(pts)[:, 2] for a in th])
    d = (np.roll(bz, -1, 0) - np.roll(bz, 1, 0)) / (2 * (th[1] - th[0]))
    return np.mean(d ** 2, axis=0)


def test_matches_brute_force_at_fine_resolution():
    rot = hm_rotor()
    rng = np.random.default_rng(1)
    r = rng.uniform(16e-3, 44e-3, 40)
    a = rng.uniform(0, 2 * math.pi, 40)
    z = np.repeat([0.0, -0.21e-3], 20)
    pts = np.c_[r * np.cos(a), r * np.sin(a), z]
    bz, key = rotor_bz(rotor_magnets, rot)
    g = mean_dbz_dtheta2(bz, key, rot.n_poles, pts)
    ref = _brute(rot, pts, 2048)
    # sum over points, as the eddy loss is: within 1 %
    assert abs(g.sum() / ref.sum() - 1) < 0.01, g.sum() / ref.sum()
    # every point within 3 % of the reference or of its largest value (interpolation in r)
    assert np.all(np.abs(g - ref) <= 0.03 * np.maximum(ref, 0.05 * ref.max()))


def test_independent_of_position_round_the_circle():
    """The time average at a point depends only on (r, z): two points on one circle agree."""
    rot = hm_rotor()
    bz, key = rotor_bz(rotor_magnets, rot)
    pts = np.array([[0.031, 0.0, 0.0], [0.031 * math.cos(1.0), 0.031 * math.sin(1.0), 0.0]])
    g = mean_dbz_dtheta2(bz, key, rot.n_poles, pts)
    assert abs(g[0] - g[1]) < 1e-9 * g[0]


if __name__ == "__main__":
    test_matches_brute_force_at_fine_resolution()
    test_independent_of_position_round_the_circle()
    print("ok")
