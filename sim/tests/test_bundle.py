"""Strand-bundle coil checks (pcbmotor/bundle.py) for the two strand test boards: continuous strands,
vias where they belong, strands kept apart, current going the same way round on every lane, and
every strand taking the same lanes in total (the transposition)."""
import json, pathlib, sys
import numpy as np
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
from pcbmotor.geometry import StatorDesign
from pcbmotor.bundle import strand_test_coil, strand_test_design, check_bundle, STRAND_TEST
from pcbmotor.layout import path_points


def winding_sense(path, centre):
    p = path_points(path, 0.1) - centre
    return float(np.sum(p[:-1, 0] * p[1:, 1] - p[1:, 0] * p[:-1, 1]))   # > 0: counter-clockwise


def design():
    j = json.loads((pathlib.Path(__file__).resolve().parents[1] / "results" / "pcbmotor_designs.json").read_text())
    return strand_test_design(StatorDesign(**j["payload_high_momentum"]["stator"]))


def test_strand_test_coils():
    d = design()
    centre = np.array([0.5 * (d.r_in_mm + d.r_out_mm), 0.0])
    turns = set()
    for v in STRAND_TEST:
        bc = strand_test_coil(d, v)
        assert check_bundle(bc)
        n = bc.n_strands
        assert abs(n * bc.strand_mm + (n - 1) * bc.strand_gap_mm - d.trace_mm) < 1e-9
        for L in range(4):
            for path in bc.lanes[L]:
                assert winding_sense(path, centre) > 0, (v, L)
        # transposition: lane s, n-1-s, s, n-1-s, so every strand's lane indices add up the same
        assert len({sum(j for _, j in s) for s in bc.strands}) == 1
        assert all([L for L, _ in s] == [0, 1, 2, 3] for s in bc.strands)
        turns.add(bc.n_turns)
    assert len(turns) == 1, "both boards should have the same turn count"


if __name__ == "__main__":
    test_strand_test_coils()
    print("ok")
