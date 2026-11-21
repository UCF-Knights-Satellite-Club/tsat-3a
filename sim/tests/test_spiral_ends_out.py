"""Checks for the ends-out spiral study: parallel coils, the wiring-ring band and the JLC-via layout."""
import math, sys, pathlib
from dataclasses import replace
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
from pcbmotor.geometry import StatorDesign, RotorDesign
from pcbmotor.em import analyse
from pcbmotor.layout import coil_layout, check_layout
from pcbmotor import spiral_ends_out as s


def test_parallel_coils_halve_kt_and_keep_km():
    d = StatorDesign("t", layers=2, n_coils=6, r_in_mm=7, r_out_mm=18, trace_mm=0.3, space_mm=0.3)
    r = RotorDesign(n_poles=8, magnet_d_mm=8, magnet_h_mm=2, magnet_r_mm=12.5)
    a, _ = analyse(d, r, n_theta=24)
    b, _ = analyse(replace(d, series=False), r, n_theta=24)
    assert abs(b.Kt_Nm_per_A / a.Kt_Nm_per_A - 0.5) < 1e-9
    assert abs(b.R_phase_ohm / a.R_phase_ohm - 0.25) < 1e-9
    km = lambda x: x.Kt_Nm_per_A / math.sqrt(x.R_phase_ohm)
    assert abs(km(a) - km(b)) < 1e-9 * km(a)


def test_ring_band_fits_the_board():
    for w in (0.6, 0.8, 1.2):
        band = s.ring_band_mm(w)
        assert 2.0 < band < 4.0, band
        assert abs(s.r_out_max(w) + band - s.BOARD_MM / 2) < 1e-9
    assert s.r_out_max(0.8) >= 42.0          # the study's r_out = 42 picks must fit 90 mm


def test_jlc_via_layout():
    d = s.design("jlc4_2oz", 0.8, 15.0, 42.0, series=False)
    lay = coil_layout(d, via_dia_mm=s.VIA_DIA, via_drill_mm=s.VIA_DRILL)
    assert check_layout(lay)
    assert lay.via_dia_mm == s.VIA_DIA and lay.n_turns >= 10
    # the L2-L3 via sits outside the winding, and the innermost ring track clears it by the space rule
    via_edge = math.hypot(*lay.via_out) + s.VIA_DIA / 2
    first_track = d.r_out_mm + s.ring_band_mm(0.8) - s.EDGE_CLEAR - s.RING_W / 2 - (s.RING_SLOTS - 1) * s.RING_PITCH
    assert via_edge > d.r_out_mm
    assert first_track - s.RING_W / 2 - via_edge >= s.SPACE - 1e-6


def test_fit_bundle_fits_the_board():
    for n, W in ((1, 0.8), (2, 0.8), (3, 1.2)):
        d, bc, ro = s._fit_bundle(n, W)
        assert d.r_out_mm == ro
        outer = s._bundle_ring_first(bc) + (s.RING_SLOTS - 1) * s.RING_PITCH + s.RING_W / 2 + s.EDGE_CLEAR
        assert s.BOARD_MM / 2 - 0.05 < outer <= s.BOARD_MM / 2 + 1e-9, (n, W, outer)
        assert bc.strand_mm >= d.stack["min_trace_mm"] - 1e-9


def test_stranded_series_doubles_kt():
    from pcbmotor.wave import hm_rotor
    rot = hm_rotor()
    log = lambda *a, **k: None
    p = s.stranded(2, 1.2, rot, series=False, log=log)
    q = s.stranded(2, 1.2, rot, series=True, log=log)
    assert abs(q["Kt_mNm_per_A"] / p["Kt_mNm_per_A"] - 2) < 1e-6
    assert 3.6 < q["R_phase_ohm"] / p["R_phase_ohm"] < 4.4        # 4x coils, plus the ring and leads
    assert abs(q["eddy_W_at_2000rpm"] - p["eddy_W_at_2000rpm"]) < 1e-12
    # transposed strands: circulating loss well under the untransposed figure and under 1 mW
    assert p["circulating_W_at_2000rpm"] < 1e-3 < p["untransposed_circulating_W_at_2000rpm"]


if __name__ == "__main__":
    test_parallel_coils_halve_kt_and_keep_km()
    test_ring_band_fits_the_board()
    test_jlc_via_layout()
    test_fit_bundle_fits_the_board()
    test_stranded_series_doubles_kt()
    print("ok")
