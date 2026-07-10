"""
test_cases.py — Stage 2 算例与网格测试
======================================
运行：pytest induction_vs_yaw_study/tests/test_cases.py -v
"""

from __future__ import annotations

import numpy as np
import pytest

from induction_vs_yaw_study import constants as C
from induction_vs_yaw_study.cases.three_nrel5mw import (
    Case, layout_for_spacing, case_id, iter_cases, load_grid,
    inflow_bts_for_case, bts_name, n_cases, AVAILABLE_BTS,
)


# ---- 布局随间距正确缩放 ----

def test_layout_spacing_scales():
    xs4, ys4 = layout_for_spacing(4.0, n_turbines=3)
    xs7, ys7 = layout_for_spacing(7.0, n_turbines=3)
    assert xs4 == [0.0, 4 * C.ROTOR_DIAMETER_M, 8 * C.ROTOR_DIAMETER_M]
    assert xs7 == [0.0, 7 * C.ROTOR_DIAMETER_M, 14 * C.ROTOR_DIAMETER_M]
    assert ys4 == [0.0, 0.0, 0.0] and ys7 == [0.0, 0.0, 0.0]


def test_base_spacing_is_4D():
    # data_cases.py 的 Turb3_Row1 间距 504m == 4D
    assert C.BASE_SPACING_M == pytest.approx(504.0)
    assert C.BASE_SPACING_D == pytest.approx(4.0)


# ---- case_id 唯一且稳定 ----

def test_case_id_stable():
    c1 = Case(8.0, 0.0, 0.06, 6.0)
    c2 = Case(8.0, 0.0, 0.06, 6.0)
    assert case_id(c1) == case_id(c2)  # 稳定


def test_case_id_unique():
    grid = load_grid()
    ids = [c.id for c in iter_cases(grid, subset=False)]
    assert len(ids) == len(set(ids))  # 唯一


# ---- 风向语义 ----

def test_wind_direction_offset():
    c = Case(8.0, 0.0, 0.06, 6.0)
    assert c.wind_direction_deg == pytest.approx(270.0)  # 轴向
    c2 = Case(8.0, 20.0, 0.06, 6.0)
    assert c2.wind_direction_deg == pytest.approx(290.0)


# ---- 入流映射 (speed, TI) ----

def test_bts_naming():
    # 与用户示例一致：inflow_06ms_TI05.bts
    assert bts_name(6.0, 0.05) == "inflow_06ms_TI05.bts"
    assert bts_name(10.0, 0.15) == "inflow_10ms_TI15.bts"
    assert bts_name(8.0, 0.10) == "inflow_08ms_TI10.bts"


def test_available_bts_covers_grid():
    # 9 个组合：风速{6,8,10} × TI{0.05,0.10,0.15}
    assert len(AVAILABLE_BTS) == 9
    for s in (6.0, 8.0, 10.0):
        for ti in (0.05, 0.10, 0.15):
            assert (s, ti) in AVAILABLE_BTS


def test_inflow_match_by_speed_and_ti():
    # require_exists=False 以便在无 .bts 文件的测试环境跑逻辑
    assert inflow_bts_for_case(6.0, 0.05, require_exists=False) == "inflow_06ms_TI05.bts"
    assert inflow_bts_for_case(10.0, 0.15, require_exists=False) == "inflow_10ms_TI15.bts"


def test_inflow_no_match_returns_none():
    # 网格外的 TI（如 0.20）或风速（如 7）无匹配
    assert inflow_bts_for_case(8.0, 0.20, require_exists=False) is None
    assert inflow_bts_for_case(7.0, 0.10, require_exists=False) is None


# ---- 网格规模 ----

def test_grid_size():
    grid = load_grid()
    # 3 风速 × 5 风向 × 3 TI × 4 间距 = 180
    n_full = n_cases(grid, subset=False)
    n_sub = n_cases(grid, subset=True)
    assert n_full == 180
    assert n_sub == 1
