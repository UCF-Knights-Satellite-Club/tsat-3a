"""As-built coil layout checks: continuous paths, vias where they belong, and the current going
the same way round on all four layers (otherwise layers would cancel instead of adding)."""
import json, math, pathlib, sys
import numpy as np
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
from pcbmotor.geometry import StatorDesign
from pcbmotor.layout import coil_layout, check_layout, path_points


def winding_sense(path, centre):
    p = path_points(path, 0.1) - centre
    return float(np.sum(p[:-1, 0] * p[1:, 1] - p[1:, 0] * p[:-1, 1]))   # > 0: counter-clockwise


def test_test_stator_layout():
    j = json.loads((pathlib.Path(__file__).resolve().parents[1] / "results" / "pcbmotor_designs.json").read_text())
    d = StatorDesign(**j["test"]["stator"])
    lay = coil_layout(d)
    assert check_layout(lay)
    centre = np.array([0.5 * (d.r_in_mm + d.r_out_mm), 0.0])
    senses = [winding_sense(p, centre) for p in lay.layers]
    assert all(s > 0 for s in senses), senses
    assert all(abs(t - lay.n_turns) < 1.0 for t in lay.turns_per_layer())
    # the two centre vias are on different spots
    assert math.dist(lay.via_in, lay.via_core) > 0.5


if __name__ == "__main__":
    test_test_stator_layout()
    print("ok")
