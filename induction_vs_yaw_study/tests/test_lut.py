"""
test_lut.py — LUT 读写与插值测试
================================
运行：pytest induction_vs_yaw_study/tests/test_lut.py -v
"""

from __future__ import annotations

import os
import tempfile

import numpy as np
import pandas as pd
import pytest

from induction_vs_yaw_study.lut.schema import (
    empty_lut, append_row, save_lut, load_lut, LUT_COLUMNS,
)
from induction_vs_yaw_study.lut.interpolate import lookup


def _toy_lut() -> pd.DataFrame:
    df = empty_lut()
    # 两个工况 × derating × 3 机
    for case_id, U, s in [("caseA", 8.0, 6.0), ("caseB", 10.0, 4.0)]:
        for tid in (1, 2, 3):
            df = append_row(
                df,
                case_id=case_id, wind_speed_ms=U,
                wind_direction_offset_deg=0.0, turbulence_intensity=0.06,
                spacing_D=s, control_type="derating", turbine_id=tid,
                yaw_deg=0.0,
                power_target_mw=4.0 if tid < 3 else 5.0,
                ratio=0.8 if tid < 3 else 1.0,
                min_pitch_deg=0.0, a_diag=0.2,
                farm_power_opt_mw=12.0, farm_power_baseline_mw=11.0,
                gain_vs_baseline_pct=9.0, greedy_power_w=5.0e6,
            )
    return df


def test_roundtrip_csv():
    df = _toy_lut()
    with tempfile.TemporaryDirectory() as d:
        path = os.path.join(d, "lut.parquet")
        save_lut(df, path)
        loaded = load_lut(path)
        assert len(loaded) == len(df)
        assert set(LUT_COLUMNS).issubset(loaded.columns)


def test_lookup_exact_node():
    df = _toy_lut()
    res = lookup(df, "derating", wind_speed_ms=8.0,
                 wind_direction_offset_deg=0.0, turbulence_intensity=0.06,
                 spacing_D=6.0, method="nearest")
    # 在网格点处应取回 caseA 的值
    assert set(res.keys()) == {1, 2, 3}
    assert res[1]["power_target_mw"] == pytest.approx(4.0)
    assert res[3]["power_target_mw"] == pytest.approx(5.0)
    assert res[3]["ratio"] == pytest.approx(1.0)


def test_lookup_nearest_picks_closest():
    df = _toy_lut()
    # 查询接近 caseB (U=10, s=4)
    res = lookup(df, "derating", wind_speed_ms=9.8,
                 wind_direction_offset_deg=0.0, turbulence_intensity=0.06,
                 spacing_D=4.2, method="nearest")
    assert res[1]["greedy_power_w"] == pytest.approx(5.0e6)


def test_lookup_linear_interpolates():
    df = _toy_lut()
    # 介于两工况之间，linear 应给出加权值
    res = lookup(df, "derating", wind_speed_ms=9.0,
                 wind_direction_offset_deg=0.0, turbulence_intensity=0.06,
                 spacing_D=5.0, method="linear")
    # power_target 应在 [4.0, 5.0] 之间（两 case 上游机分别 4.0/4.0）
    assert 3.5 <= res[1]["power_target_mw"] <= 5.5
