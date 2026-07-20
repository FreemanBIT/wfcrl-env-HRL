"""
cases_steady.py — 稳态风 3 机单列算例与工况网格
==================================================
为“快速对比”路径提供稳态风(WindType=1)的工况与配置构建，**不依赖任何 .bts**。

设计要点
--------
* 复用主工程的布局/常量/配置类，仅把风类型强制为稳态：
    - FLORIS：本就用稳态风，直接复用 make_floris_config 的等价构建。
    - FAST.Farm：WindConfig.wind_type = STEADY 且 wind_file=None，于是
      create_ff_case 走 fastFarmBoxExtent 分支，自动生成自洽的低/高分辨率
      网格（无外部 .bts 盒子 → 无横向越界问题）。
* 工况维度只有 (风速, 风向偏移, 间距)，不含湍流度。
* 为复用已建好的 FLORIS LUT，每个稳态工况携带一个 lut_reference_ti，
  在 LUT 中按 (风速, 风向偏移, ti=参考值, 间距) 最近邻查询最优控制。

与主工程的关系
--------------
本模块**不修改**主工程任何文件；它 import 主工程的 cases 模块复用其
layout/常量/配置类，只是改用稳态风并提供独立的网格迭代。
"""

from __future__ import annotations

import hashlib
from dataclasses import dataclass
from itertools import product
from pathlib import Path
from typing import Iterator, Optional

import yaml

from induction_vs_yaw_study import constants as C
from induction_vs_yaw_study.cases.three_nrel5mw import layout_for_spacing, layout_for_case

from wfcrl.config import WindConfig, WindType
from wfcrl.config import FastFarmConfig, FlorisConfig


# =========================================================================
# 稳态 Case 数据结构（无 TI 维度）
# =========================================================================

@dataclass
class SteadyCase:
    """稳态风单工况（不含湍流度）。"""
    wind_speed_ms: float
    wind_direction_offset_deg: float       # 相对轴向(270°)的偏移
    spacing_D: float
    # 复用 FLORIS LUT 时的参考 TI（仅用于 LUT 查询，不进入仿真）
    lut_reference_ti: float = 0.10
    # 仿真参数（从 grid 注入）
    floris_dt: float = 60.0
    floris_max_iter: int = 3
    fastfarm_dt: float = 3.0
    fastfarm_t_settle_s: float = 360.0
    fastfarm_n_control_steps: int = 20

    @property
    def wind_direction_deg(self) -> float:
        """实际下发给仿真器的气象风向。"""
        return (C.AXIAL_WIND_DIRECTION_DEG + self.wind_direction_offset_deg) % 360.0

    @property
    def spacing_m(self) -> float:
        return self.spacing_D * C.ROTOR_DIAMETER_M

    @property
    def id(self) -> str:
        return steady_case_id(self)


def steady_case_id(case: SteadyCase) -> str:
    """稳定、唯一、可读的稳态工况 ID（产物命名用，带 'st' 前缀以区别湍流版）。"""
    base = (f"stU{case.wind_speed_ms:g}_wd{case.wind_direction_offset_deg:g}"
            f"_s{case.spacing_D:g}D")
    h = hashlib.md5(base.encode()).hexdigest()[:6]
    return f"{base}_{h}"


# =========================================================================
# 配置构建（稳态风；FLORIS / FAST.Farm 共用同一 SteadyCase）
# =========================================================================

def make_floris_config_steady(
    case: SteadyCase, output_dir: Optional[str] = None
) -> FlorisConfig:
    """构建 FLORIS 稳态配置。

    采用**风场旋转法**（与主工程湍流路径一致）：布局绕质心旋转 -offset，
    入流方向固定为轴向 270°。这样 FLORIS 与 FAST.Farm 的几何/控制口径完全一致，
    且避免 FAST.Farm 侧 PropagationDir 旋转与机舱偏航的坐标系冲突。
    """
    xs, ys = layout_for_case(case.spacing_D, case.wind_direction_offset_deg, n_turbines=3)
    wind = WindConfig(
        wind_type=WindType.STEADY,
        speed=case.wind_speed_ms,
        direction=C.AXIAL_WIND_DIRECTION_DEG,   # 270°：入流沿 +X（风场已旋转）
        turbulence_intensity=case.lut_reference_ti,   # FLORIS 回放用参考 TI
        reference_height=C.HUB_HEIGHT_M,
    )
    return FlorisConfig(
        case_name=f"flo_{case.id}",
        num_turbines=3,
        xcoords=xs, ycoords=ys,
        dt=case.floris_dt,
        max_iter=case.floris_max_iter,
        wind=wind,
        turbine_type="nrel_5MW",
        output_dir=output_dir,
    )


def make_fastfarm_config_steady(
    case: SteadyCase, output_dir: Optional[str] = None
) -> FastFarmConfig:
    """
    构建 FAST.Farm **稳态风**配置（WindType=1，无 .bts）。

    采用**风场旋转法**：布局绕质心旋转 -offset，入流方向固定 270°（PropagationDir≈0）。
    这样：
      * 转子正对 +X，baseline 机舱偏航=0，不再出现 chi>90° 的反向来流 → 不再
        "Rotor diameter must be greater than zero" / NaN 发散；
      * 与 FLORIS 几何一致，对比成立。
    关键：wind_file=None 且 wind_type=STEADY → create_ff_case 走 fastFarmBoxExtent
    分支，自动生成自洽的低/高分辨率网格。
    """
    xs, ys = layout_for_case(case.spacing_D, case.wind_direction_offset_deg, n_turbines=3)
    wind = WindConfig(
        wind_type=WindType.STEADY,
        speed=case.wind_speed_ms,
        direction=C.AXIAL_WIND_DIRECTION_DEG,   # 270°：入流沿 +X（风场已旋转）
        turbulence_intensity=case.lut_reference_ti,   # 仅记录，不进入稳态流场
        reference_height=C.HUB_HEIGHT_M,
        wind_file=None,                                # 关键：不给 .bts
    )
    max_iter = case.fastfarm_n_control_steps + int(
        round(case.fastfarm_t_settle_s / case.fastfarm_dt)
    )
    return FastFarmConfig(
        case_name=f"ff_{case.id}",
        num_turbines=3,
        xcoords=xs, ycoords=ys,
        dt=case.fastfarm_dt,
        max_iter=max_iter,
        wind=wind,
        turbine_type="nrel_5MW",
        output_dir=output_dir,
    )


# =========================================================================
# 网格加载与迭代
# =========================================================================

def load_steady_grid(grid_path: Optional[str] = None) -> dict:
    if grid_path is None:
        grid_path = str(Path(__file__).resolve().parent / "grid_steady.yaml")
    with open(grid_path, "r", encoding="utf-8") as f:
        return yaml.safe_load(f)


def iter_steady_cases(grid: dict, subset: bool = False) -> Iterator[SteadyCase]:
    """遍历稳态网格生成 SteadyCase。subset=True 用 smoke_subset。"""
    if subset and "smoke_subset" in grid:
        sub = grid["smoke_subset"]
        speeds = sub["wind_speeds_ms"]
        offsets = sub["wind_direction_offsets_deg"]
        spacings = sub["spacings_D"]
    else:
        speeds = grid["wind_speeds_ms"]
        offsets = grid["wind_direction_offsets_deg"]
        spacings = grid["spacings_D"]

    ref_ti = float(grid.get("lut_reference_ti", 0.10))
    flo = grid.get("floris", {})
    ff = grid.get("fastfarm", {})

    for U, wd, s in product(speeds, offsets, spacings):
        yield SteadyCase(
            wind_speed_ms=float(U),
            wind_direction_offset_deg=float(wd),
            spacing_D=float(s),
            lut_reference_ti=ref_ti,
            floris_dt=float(flo.get("dt", 60.0)),
            floris_max_iter=int(flo.get("max_iter", 3)),
            fastfarm_dt=float(ff.get("dt", 3.0)),
            fastfarm_t_settle_s=float(ff.get("t_settle_s", 360.0)),
            fastfarm_n_control_steps=int(ff.get("n_control_steps", 20)),
        )


def n_steady_cases(grid: dict, subset: bool = False) -> int:
    return sum(1 for _ in iter_steady_cases(grid, subset=subset))
