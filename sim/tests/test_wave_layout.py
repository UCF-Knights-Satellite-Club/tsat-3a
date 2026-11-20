"""Checks for the wave test-board copper (pcbmotor/wave_layout.py) and its as-built model."""
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))

from pcbmotor import wave_layout as wl
from pcbmotor import wave_asbuilt


def test_every_variant_is_one_correct_series_path_with_clearance():
    for name in wl.VARIANTS:
        wb = wl.build(wl.variant(name))
        _, errs = wl.series_path(wb)
        assert not errs, (name, errs)
        con = wl.check_connectivity(wb)
        assert con["ok"], (name, con)
        sp = wl.check_spacing(wb)
        assert not sp, (name, sp[:5])
        assert wl.check_widths(wb) >= wb.p.min_trace_mm, name


def test_swapped_strands_cancel_the_circulating_current():
    swap = wave_asbuilt.analyse("swap", n_theta=36)
    fixed = wave_asbuilt.analyse("fixed", n_theta=36)
    assert swap["circulating_W_at_2000rpm"] < 0.02 * fixed["circulating_W_at_2000rpm"]
    assert abs(swap["Kt_mNm_per_A"] - fixed["Kt_mNm_per_A"]) < 0.01 * fixed["Kt_mNm_per_A"]


if __name__ == "__main__":
    test_every_variant_is_one_correct_series_path_with_clearance()
    test_swapped_strands_cancel_the_circulating_current()
    print("ok")
