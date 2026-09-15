"""test_bts_reuse.py — BTS 复用断言（开发计划 §4.5 / 方案 V2 §31.2-31.4）

纯逻辑测试（require_exists=False），不启动 FAST.Farm。
验收：相同 (U,TI,seed) 下，spacing/WD/control 变化 -> bts 路径不变；
case_id 含 seed；网格规模 = 4U x 3TI x 6seed x 3WD x 4S（x3 control）。
"""
from __future__ import annotations

import csv
from pathlib import Path

from induction_vs_yaw_study.cases.three_nrel5mw import (
    bts_name, resolve_bts_file, case_id, load_grid,
    iter_seeded_cases, n_cases, n_seeded_cases, Case,
)

STUDY_ROOT = Path(__file__).resolve().parents[2]


def test_seed_name_format():
    assert bts_name(8.0, 0.10, 3) == "inflow_U08_TI10_seed03.bts"
    assert bts_name(6.0, 0.05, 1) == "inflow_U06_TI05_seed01.bts"
    assert bts_name(12.0, 0.15, 6) == "inflow_U12_TI15_seed06.bts"


def test_legacy_name_unchanged():
    # 向后兼容：seed_id=None 保留旧样式（旧测试/探针依赖）
    assert bts_name(6.0, 0.05) == "inflow_06ms_TI05.bts"


def test_resolve_identity_across_spacing_and_wd():
    """相同 (U,TI,seed)：所有 spacing/WD 的 bts 路径必须一致（约束 3 / §31.2-3）。"""
    paths = set()
    for wd in (0.0, 10.0, 20.0):
        for s in (2.0, 4.0, 6.0, 8.0):
            p = resolve_bts_file(8.0, 0.10, 3, require_exists=False)
            assert p == "coarse/inflow_U08_TI10_seed03.bts"
            paths.add(p)
    assert len(paths) == 1


def test_bts_name_has_no_layout_or_control_tokens():
    name = bts_name(8.0, 0.10, 3)
    for token in ("wd", "s2D", "s4D", "s6D", "s8D", "yaw", "induction", "baseline"):
        assert token not in name


def test_case_id_contains_seed_and_unique():
    c1 = Case(8.0, 10.0, 0.10, 6.0, seed_id=3)
    c2 = Case(8.0, 10.0, 0.10, 6.0, seed_id=3)
    c3 = Case(8.0, 10.0, 0.10, 6.0, seed_id=4)
    assert case_id(c1) == case_id(c2)
    assert case_id(c1) != case_id(c3)
    assert "_seed03_" in case_id(c1)


def test_grid_sizes_v2():
    grid = load_grid()
    # LUT 视角（不展开 seed）：4U x 3TI x 3WD x 4S = 144
    assert n_cases(grid, subset=False) == 144
    # FAST.Farm 视角（展开 seed）：144 x 6 = 864（x3 control = 2592）
    assert n_seeded_cases(grid, subset=False) == 864
    assert n_seeded_cases(grid, subset=False, seed_ids=[1]) == 144
    # 冒烟子集：1 组合 x 1 seed
    assert n_cases(grid, subset=True) == 1
    assert n_seeded_cases(grid, subset=True) == 1


def test_seed_manifest_present():
    mf = STUDY_ROOT / "config" / "seed_manifest.csv"
    assert mf.exists(), "config/seed_manifest.csv 缺失（generate_wind_boxes 会生成）"
    with open(mf, newline="", encoding="utf-8") as f:
        rows = list(csv.DictReader(f))
    assert len(rows) == 6
    # rand_seed_1 固定（11001..66006），不允许循环递增语义
    assert [int(r["rand_seed_1"]) for r in rows] == [11001, 22002, 33003,
                                                     44004, 55005, 66006]


def test_iter_seeded_uniqueness():
    grid = load_grid()
    ids = [c.id for c in iter_seeded_cases(grid, subset=True)]
    assert len(ids) == len(set(ids))
