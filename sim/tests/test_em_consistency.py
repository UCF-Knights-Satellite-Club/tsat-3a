"""Energy-conservation check: torque per amp (Lorentz force on traces) must equal
d(flux linkage)/d(theta) (Faraday). Two independent calculations of the same Kt."""
import math, sys, pathlib
import numpy as np
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
from matplotlib.path import Path
from pcbmotor.geometry import StatorDesign, RotorDesign, coil_turns
from pcbmotor.em import coil_torque_curve, rotor_magnets


def flux_linkage(d, r, theta, grid=0.15):
    lam = 0.0
    for z in d.stack["z_mm"]:
        for poly in coil_turns(d):
            xs = np.arange(poly[:, 0].min(), poly[:, 0].max(), grid)
            ys = np.arange(poly[:, 1].min(), poly[:, 1].max(), grid)
            X, Y = np.meshgrid(xs + grid / 2, ys + grid / 2)
            pts = np.c_[X.ravel(), Y.ravel()]
            inside = Path(poly).contains_points(pts)
            p = np.c_[pts[inside] / 1e3, np.full(inside.sum(), -z / 1e3)]
            lam += rotor_magnets(r, theta).getB(p)[:, 2].sum() * (grid / 1e3) ** 2
    return lam


def test_torque_equals_dlambda_dtheta():
    d = StatorDesign("t", layers=2, n_coils=6, r_in_mm=7, r_out_mm=18, trace_mm=0.3, space_mm=0.3)
    r = RotorDesign(n_poles=8, magnet_d_mm=8, magnet_h_mm=2, magnet_r_mm=12.5)
    th, tq = coil_torque_curve(d, r, n_theta=24)
    h = 1e-3
    for k in (2, 5, 9):
        dl = (flux_linkage(d, r, th[k] + h) - flux_linkage(d, r, th[k] - h)) / (2 * h)
        # torque on rotor = +/- dlambda/dtheta depending on current sign convention
        assert abs(abs(dl) - abs(tq[k])) <= 0.05 * max(abs(tq).max(), 1e-12), (k, dl, tq[k])


if __name__ == "__main__":
    test_torque_equals_dlambda_dtheta()
    print("ok")
