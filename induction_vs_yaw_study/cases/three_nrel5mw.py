"""
three_nrel5mw.py — 3 机单列算例与工况网格 (Stage 2)
===================================================
按用户要求：
  * 不修改风场模型库 / 模板。不同间距通过 **脚本内替换 xcoords** 实现
    （FLORIS 用 FlorisConfig.xcoords；FAST.Farm 用 FastFarmConfig.xcoords，
     create_ff_case 会据此重建 .fstf 布局，无需改模板）。
  * FAST.Farm 入流：工程已提供 5 个 TurbSim .bts (6/8/10/11.4 m/s，单一 TI)。
    本模块把网格风速映射到最近的可用 .bts；缺失风速则回退稳态(WindType=1)
    或提示用户用 TurbSim 生成（见 run/build_inflow.py 与 README）。

风向参考系
  FLORIS 气象惯例：270° == 来流自西向东 (+X)。单列风机沿 +X 排列，
  故 270° 即"沿列轴对齐"。网格里的 direction_offset 加到 270 上。
  FAST.Farm 端 interface 内部把 direction 转成 PropagationDir=(dir+90)%360，
  并设各机初始 NacYaw=(270-dir)%360（见 wfcrl/interface.py），保持一致。
"""

from __future__ import annotations

import hashlib
import os
from dataclasses import dataclass, field
from itertools import product
from pathlib import Path
from typing import Dict, Iterator, List, Optional

import yaml

from induction_vs_yaw_study import constants as C

from wfcrl.config import WindConfig, WindType
from wfcrl.simul_config import FastFarmConfig, FlorisConfig


# 工程根目录（用于定位 .bts 模板）
_WFCRL_ROOT = Path(__file__).resolve().parents[2]
_FF_FARMINPUTS = (
    _WFCRL_ROOT / "wfcrl" / "simulators" / "fastfarm" / "inputs" / "template" / "FarmInputs"
)

# =========================================================================
# 入流 .bts 命名与映射 —— (风速, 湍流度) 二维键
# =========================================================================
# 用户生成的 9 个 TurbSim .bts 命名规则（示例 inflow_06ms_TI05.bts）：
#     inflow_{速度:02d}ms_TI{TI*100:02d}.bts
# 覆盖：风速 {6,8,10} m/s × 湍流度 {0.05,0.10,0.15}。
# TI 现在是 FAST.Farm 的**真实**入流维度（不再只对 FLORIS 生效）。

# 本研究网格用到的标称 (风速, TI) 组合；可按需扩展。
BTS_SPEEDS = [6.0, 8.0, 10.0]
BTS_TIS = [0.05, 0.10, 0.15]


def bts_name(wind_speed_ms: float, turbulence_intensity: float,
             spacing_D: Optional[float] = None) -> str:
    """
    生成 .bts 文件名。

    * 不带间距（向后兼容旧的 9 盒命名）：inflow_06ms_TI05.bts
    * 带间距（风场旋转后按间距分宽度，推荐用于非零对风偏差）：
        inflow_06ms_TI05_s8D.bts
    """
    s = int(round(wind_speed_ms))
    ti = int(round(turbulence_intensity * 100))
    if spacing_D is None:
        return f"inflow_{s:02d}ms_TI{ti:02d}.bts"
    return f"inflow_{s:02d}ms_TI{ti:02d}_s{spacing_D:g}D.bts"


# 预构建 (速度, TI) → 文件名 映射表（旧的 9 盒，无间距维度）
AVAILABLE_BTS: Dict[tuple, str] = {
    (s, ti): bts_name(s, ti) for s in BTS_SPEEDS for ti in BTS_TIS
}


# =========================================================================
# Case 数据结构
# =========================================================================

@dataclass
class Case:
    """单个工况。"""
    wind_speed_ms: float
    wind_direction_offset_deg: float   # 相对轴向(270°)的偏移
    turbulence_intensity: float
    spacing_D: float
    # 仿真参数（从 grid.yaml 注入）
    floris_dt: float = 60.0
    floris_max_iter: int = 3
    fastfarm_dt: float = 3.0
    fastfarm_t_settle_s: float = 150.0
    fastfarm_n_control_steps: int = 50

    @property
    def wind_direction_deg(self) -> float:
        """实际下发给仿真器的气象风向。"""
        return (C.AXIAL_WIND_DIRECTION_DEG + self.wind_direction_offset_deg) % 360.0

    @property
    def spacing_m(self) -> float:
        return self.spacing_D * C.ROTOR_DIAMETER_M

    @property
    def id(self) -> str:
        return case_id(self)


def case_id(case: Case) -> str:
    """稳定、唯一、可读的工况 ID（用于产物命名）。"""
    base = (f"U{case.wind_speed_ms:g}_wd{case.wind_direction_offset_deg:g}"
            f"_TI{case.turbulence_intensity:g}_s{case.spacing_D:g}D")
    h = hashlib.md5(base.encode()).hexdigest()[:6]
    return f"{base}_{h}"


# =========================================================================
# 布局（间距替换 —— 不改模板）
# =========================================================================

def layout_for_spacing(spacing_D: float, n_turbines: int = 3):
    """
    生成单列布局坐标（沿 +X，未旋转）。间距以 D 为单位。

    Returns (xcoords, ycoords)，均为长度 n_turbines 的列表。
    """
    s_m = spacing_D * C.ROTOR_DIAMETER_M
    xcoords = [i * s_m for i in range(n_turbines)]
    ycoords = [0.0 for _ in range(n_turbines)]
    return xcoords, ycoords


def layout_for_case(
    spacing_D: float,
    wind_direction_offset_deg: float,
    n_turbines: int = 3,
):
    """
    **风场旋转法**生成布局坐标：把单列风机绕质心旋转 -offset，
    使入流始终沿 +X（PropagationDir≈0），从而：

      * FAST.Farm 的低/高分辨率网格随旋转后的 ycoords 自动正确取尺寸；
      * 尾流沿 +X 传播，不再斜向穿出 .bts 的 Y 边界（修复 -20° 越界）；
      * 转子正对 +X，不再出现 chi>90° 的反向来流导致 NaN 发散。

    这是 FAST.Farm 处理非零风向的**标准做法**（旋转风场而非转 PropagationDir）。
    已验证：旋转后(风向=270) 与 原始(风向=270+offset) 的相对几何（顺风间距、
    横风偏移）完全一致，故 FLORIS/FAST.Farm 物理等价，对比仍然成立。

    Returns (xcoords, ycoords)，已绕质心居中。
    """
    import numpy as np

    s_m = spacing_D * C.ROTOR_DIAMETER_M
    xs = np.array([i * s_m for i in range(n_turbines)], dtype=float)
    ys = np.zeros(n_turbines, dtype=float)

    # 旋转 -offset（与风向偏移相反），使风(+X)以 offset 角度入射风机列
    th = np.radians(-float(wind_direction_offset_deg))
    c, s = np.cos(th), np.sin(th)
    xr = c * xs - s * ys
    yr = s * xs + c * ys
    # 绕质心居中（最小化所需 Y 跨度，节省 .bts 宽度）
    xr -= xr.mean()
    yr -= yr.mean()
    return xr.tolist(), yr.tolist()


def required_gridwidth_m(
    spacing_D: float,
    wind_direction_offset_deg: float,
    n_turbines: int = 3,
    corridor_D: float = 2.0,
    meander_D: float = 1.0,
    rotor_halfspan_D: float = 1.0,
) -> float:
    """
    返回该 (间距, 风向偏差) 工况所需的 TurbSim 盒**最小横向宽度**（米）。

    **风场旋转法**下，外侧风机的 Y 坐标随 offset×spacing 增大（内禀的
    Y-span = (n-1)*spacing*sin|offset|），故盒子必须足够宽以：

      (a) 让每台风机的**高分辨率盒**（居中于风机，半跨≈1D 含叶尖）完整落入；
      (b) 让**低分辨率域**覆盖风机列展宽 + 尾流走廊(≈2D)，并给蜿蜒采样留 ≈1D。

    盒宽 = 2 * max( |Y|max + rotor_halfspan,  span/2 + corridor + meander )。

    与 simul_utils.create_ff_case 内部的校验口径一致；据此生成的专用盒
    （命名含 _s{spacing}D）可避免运行期 "Outside the grid bounds" 越界。
    """
    import numpy as np

    D = C.ROTOR_DIAMETER_M
    xs, ys = layout_for_case(spacing_D, wind_direction_offset_deg, n_turbines)
    yr = np.asarray(ys, dtype=float)
    ymax = float(np.abs(yr).max()) if len(yr) else 0.0
    span = float(yr.max() - yr.min()) if len(yr) else 0.0
    need_high = ymax + rotor_halfspan_D * D
    need_low = span / 2.0 + corridor_D * D + meander_D * D
    half = max(need_high, need_low)
    return 2.0 * half


def recommended_gridwidth_for_spacing(
    spacing_D: float,
    max_abs_offset_deg: float = 20.0,
    round_to_m: float = 20.0,
) -> float:
    """
    某间距下、覆盖其全部风向偏差（默认 ±20°）所需的**单一盒宽**（米，向上取整）。

    生成"每间距一个盒"时用此宽度，即可让该间距的所有 offset 工况共用一个盒。
    """
    import numpy as np

    w = required_gridwidth_m(spacing_D, max_abs_offset_deg)
    return float(np.ceil(w / round_to_m) * round_to_m)


# =========================================================================
# 入流文件映射 (FAST.Farm) —— 按 (风速, 湍流度) 联合匹配
# =========================================================================

def inflow_bts_for_case(
    wind_speed_ms: float,
    turbulence_intensity: float,
    *,
    spacing_D: Optional[float] = None,
    speed_tol: float = 0.6,
    ti_tol: float = 0.011,
    require_exists: bool = True,
) -> Optional[str]:
    """
    把 (风速, 湍流度[, 间距]) 映射到最近的可用 .bts 文件名。

    查找顺序（风场旋转法下，不同间距需要不同盒宽）：
      1) 若给了 spacing_D，先找间距专用盒 inflow_..._s{spacing}D.bts；
      2) 否则/找不到时，回退到通用盒 inflow_..ms_TI...bts（旧的 9 盒）。

    Parameters
    ----------
    require_exists : bool
        True 则要求文件实际存在于 FarmInputs/ 才返回（部署校验用）；
        False 仅按命名规则返回名字（便于在无文件环境做逻辑测试）。

    Returns 文件名，或 None（无匹配，需 TurbSim 生成）。
    """
    # 1) 间距专用盒优先
    if spacing_D is not None:
        cand = bts_name(wind_speed_ms, turbulence_intensity, spacing_D=spacing_D)
        if (not require_exists) or (_FF_FARMINPUTS / cand).exists():
            return cand

    # 2) 通用盒（最近邻 (速度, TI)）
    best, best_cost = None, 1e9
    for (spd, ti), fname in AVAILABLE_BTS.items():
        ds = abs(spd - wind_speed_ms)
        dti = abs(ti - turbulence_intensity)
        if ds <= speed_tol and dti <= ti_tol:
            cost = ds + 100.0 * dti  # TI 优先精确匹配
            if cost < best_cost:
                best, best_cost = fname, cost
    if best is None:
        return None
    if require_exists and not (_FF_FARMINPUTS / best).exists():
        return None
    return best


# 向后兼容旧名（仅按风速，忽略 TI；不建议使用）
def inflow_bts_for_speed(wind_speed_ms: float, tol: float = 0.6) -> Optional[str]:
    """[Deprecated] 仅按风速匹配（取该风速下 TI=0.10 的 .bts 作为代表）。"""
    return inflow_bts_for_case(wind_speed_ms, 0.10, speed_tol=tol)



# =========================================================================
# 配置构建（FLORIS / FAST.Farm 共用同一 Case）
# =========================================================================

def make_floris_config(case: Case, output_dir: Optional[str] = None) -> FlorisConfig:
    """
    构建 FLORIS 配置。**风场旋转法**：用旋转后的 xcoords/ycoords + 基准风向 270°。
    （与 FAST.Farm 完全一致的几何，保证对比有效。）
    """
    xs, ys = layout_for_case(case.spacing_D, case.wind_direction_offset_deg, n_turbines=3)
    wind = WindConfig(
        wind_type=WindType.STEADY,
        speed=case.wind_speed_ms,
        direction=C.AXIAL_WIND_DIRECTION_DEG,   # 270°：入流沿 +X（风场已旋转）
        turbulence_intensity=case.turbulence_intensity,
        reference_height=C.HUB_HEIGHT_M,
    )
    cfg = FlorisConfig(
        case_name=f"flo_{case.id}",
        num_turbines=3,
        xcoords=xs, ycoords=ys,
        dt=case.floris_dt,
        max_iter=case.floris_max_iter,
        wind=wind,
        turbine_type="nrel_5MW",          # FLORIS 内置
        output_dir=output_dir,
    )
    return cfg


def make_fastfarm_config(
    case: Case,
    output_dir: Optional[str] = None,
    *,
    strict_inflow: bool = True,
) -> FastFarmConfig:
    """
    构建 FAST.Farm 配置。间距通过 xcoords 注入（create_ff_case 重建布局）。

    入流：按 (风速, 湍流度) 匹配用户生成的 .bts (WindType=3, 真实湍流)。
    用户已为 风速{6,8,10} × TI{0.05,0.10,0.15} 生成全部 9 个 .bts，因此网格内
    每个工况都应能精确匹配，TI 是 FAST.Farm 的真实入流维度。

    strict_inflow=True（默认）：找不到匹配 .bts 时抛错（避免悄悄回退稳态、
    导致 TI 维度失真）。设 False 则回退稳态风(WindType=1) 并打印警告。
    """
    xs, ys = layout_for_case(case.spacing_D, case.wind_direction_offset_deg, n_turbines=3)

    bts = inflow_bts_for_case(
        case.wind_speed_ms, case.turbulence_intensity, spacing_D=case.spacing_D
    )

    # --- 盒宽充分性预检（风场旋转法下，大 offset×spacing 需要更宽的盒）---
    # 若选中的 .bts 是间距专用盒（命名含 _s{spacing}D），默认其已按本间距足够宽；
    # 若回退到通用盒（旧 9 盒，通常 800 m 宽），则对偏差工况可能太窄 —— 这里提前
    # 给出明确提示，让用户在跑之前就生成专用盒，而不是等 FAST.Farm 运行期越界。
    if bts is not None:
        need_w = required_gridwidth_m(case.spacing_D, case.wind_direction_offset_deg)
        is_dedicated = f"_s{case.spacing_D:g}D" in str(bts)
        if not is_dedicated:
            # 通用盒的常见宽度约 800 m；无法直接读盒宽，这里按经验阈值提示。
            GENERIC_BOX_WIDTH_M = 800.0
            if need_w > GENERIC_BOX_WIDTH_M + 1e-6:
                import warnings
                rec = recommended_gridwidth_for_spacing(case.spacing_D)
                warnings.warn(
                    f"[make_fastfarm_config] 工况 U={case.wind_speed_ms} "
                    f"wd={case.wind_direction_offset_deg:g} s={case.spacing_D:g}D "
                    f"需要盒宽 >= {need_w:.0f} m，但当前回退到通用盒"
                    f"（约 {GENERIC_BOX_WIDTH_M:.0f} m），运行期会因旋转风机列越出"
                    f" .bts 边界而报 'Outside the grid bounds'。\n"
                    f"  请为该间距生成专用盒（覆盖全部 ±offset）：GridWidth≈{rec:.0f} m，"
                    f"命名 inflow_{int(round(case.wind_speed_ms)):02d}ms_"
                    f"TI{int(round(case.turbulence_intensity*100)):02d}_"
                    f"s{case.spacing_D:g}D.bts。\n"
                    f"  生成器：python -m induction_vs_yaw_study.turbsim_inp."
                    f"generate_turbsim_inputs --out <FarmInputs 目录>"
                )

    if bts is not None:
        wind = WindConfig(
            wind_type=WindType.TURBSIM_BTS,
            speed=case.wind_speed_ms,
            direction=C.AXIAL_WIND_DIRECTION_DEG,   # 270°：入流沿 +X（风场已旋转）
            turbulence_intensity=case.turbulence_intensity,
            reference_height=C.HUB_HEIGHT_M,
            wind_file=bts,
        )
    else:
        expected = bts_name(case.wind_speed_ms, case.turbulence_intensity)
        msg = (f"No matching .bts for U={case.wind_speed_ms} TI={case.turbulence_intensity} "
               f"(expected '{expected}' in {_FF_FARMINPUTS}). "
               f"Generate it with TurbSim (见 README) 或将 strict_inflow=False 回退稳态。")
        if strict_inflow:
            raise FileNotFoundError(msg)
        import warnings
        warnings.warn(msg)
        wind = WindConfig(
            wind_type=WindType.STEADY,
            speed=case.wind_speed_ms,
            direction=C.AXIAL_WIND_DIRECTION_DEG,   # 270°：入流沿 +X（风场已旋转）
            turbulence_intensity=case.turbulence_intensity,
            reference_height=C.HUB_HEIGHT_M,
        )

    max_iter = case.fastfarm_n_control_steps + int(
        round(case.fastfarm_t_settle_s / case.fastfarm_dt)
    )
    cfg = FastFarmConfig(
        case_name=f"ff_{case.id}",
        num_turbines=3,
        xcoords=xs, ycoords=ys,
        dt=case.fastfarm_dt,
        max_iter=max_iter,
        wind=wind,
        turbine_type="nrel_5MW",
        output_dir=output_dir,
    )
    return cfg


# =========================================================================
# 网格加载与迭代
# =========================================================================

def load_grid(grid_path: Optional[str] = None) -> dict:
    if grid_path is None:
        grid_path = str(Path(__file__).resolve().parent / "grid.yaml")
    with open(grid_path, "r", encoding="utf-8") as f:
        return yaml.safe_load(f)


def iter_cases(grid: dict, subset: bool = False) -> Iterator[Case]:
    """
    遍历网格生成 Case。

    subset=True 时用 grid['smoke_subset'] 的小范围（先跑通用）。
    """
    if subset and "smoke_subset" in grid:
        sub = grid["smoke_subset"]
        speeds = sub["wind_speeds_ms"]
        offsets = sub["wind_direction_offsets_deg"]
        tis = sub["turbulence_intensities"]
        spacings = sub["spacings_D"]
    else:
        speeds = grid["wind_speeds_ms"]
        offsets = grid["wind_direction_offsets_deg"]
        tis = grid["turbulence_intensities"]
        spacings = grid["spacings_D"]

    flo = grid.get("floris", {})
    ff = grid.get("fastfarm", {})

    for U, wd, ti, s in product(speeds, offsets, tis, spacings):
        yield Case(
            wind_speed_ms=float(U),
            wind_direction_offset_deg=float(wd),
            turbulence_intensity=float(ti),
            spacing_D=float(s),
            floris_dt=float(flo.get("dt", 60.0)),
            floris_max_iter=int(flo.get("max_iter", 3)),
            fastfarm_dt=float(ff.get("dt", 3.0)),
            fastfarm_t_settle_s=float(ff.get("t_settle_s", 150.0)),
            fastfarm_n_control_steps=int(ff.get("n_control_steps", 50)),
        )


def n_cases(grid: dict, subset: bool = False) -> int:
    return sum(1 for _ in iter_cases(grid, subset=subset))
