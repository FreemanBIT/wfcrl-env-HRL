"""test_yaw_coordinate.py — 审查 P0-3：layout 旋转不改变 baseline yaw 目标
"""
from __future__ import annotations

import pytest

from induction_vs_yaw_study.cases.three_nrel5mw import (
    yaw_target_deg, ALIGN_YAW_DEG, Case, layout_for_case,
)


def test_layout_rotation_does_not_change_baseline_yaw_target():
    """WD ∈ {0,10,20}，yaw_mis=0 时对齐目标必须一致（=270）。"""
    for wd in (0.0, 10.0, 20.0):
        assert yaw_target_deg(0.0) == pytest.approx(270.0)
        xs, ys = layout_for_case(4.0, wd)
        assert len(xs) == 3 and len(ys) == 3


def test_yaw_target_only_misalignment():
    assert yaw_target_deg(0.0) == pytest.approx(270.0)
    assert yaw_target_deg(10.0) == pytest.approx(280.0)
    assert yaw_target_deg(-10.0) == pytest.approx(260.0)
    assert ALIGN_YAW_DEG == 270.0
    # case.wind_direction_deg（270+wd）不参与 yaw target 计算
    c = Case(8.0, 20.0, 0.10, 6.0, seed_id=1)
    assert yaw_target_deg(0.0) != pytest.approx(c.wind_direction_deg)
