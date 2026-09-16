"""
three_nrel5mw.py — 3 机单列算例与工况网格 (V2 重构)
====================================================
按用户约束（开发计划 §0/§1）：
  * 约束 1：风向仅 {0, 10, 20}（不做负风向——与正风向对称）。
  * 约束 3：入流 .bts 按 (风速, TI, seed) 唯一键（config/seed_manifest.csv），
    命名 inflow_U{U:02d}_TI{TI:02d}_seed{seed:02d}.bts；文件名不含 spacing/WD/control。
  * 约束 4：风速集 {6, 8, 10, 12}。
  * 约束 5：只读使用 wfcrl/closed-loop-control；本模块属于 study，允许修改。
  * 风场旋转法不变：不同间距/风向通过替换 xcoords/ycoords 实现，不重生成 .bts。

迭代语义：
  * iter_cases()           不展开 seed（FLORIS LUT 与 seed 无关，见开发计划 §5.2）；
  * iter_seeded_cases()    展开 seed（FAST.Farm 全 DOE / manifest 用）。
"""
from __future__ import annotations

import os
from dataclasses import dataclass
from itertools import product
from pathlib import Path
from typing import Dict, Iterator, List, Optional

import yaml

from induction_vs_yaw_study import constants as C

from wfcrl.config import WindConfig, WindType
from wfcrl.config import FastFarmConfig, FlorisConfig


# 工程根目录（study 自身）：用于定位 study/FarmInputs
_STUDY_ROOT = Path(__file__).resolve().parents[1]
_FF_FARMINPUTS = Path(os.environ.get(
    "WFCRL_FARMINPUTS_DIR",
    str(_STUDY_ROOT / "FarmInputs"),
))
# 本次研究所需的风盒（coarse：study/FarmInputs/coarse；可选覆盖：WFCRL_FARMINPUTS_DIR）
_COARSE_DIR = _FF_FARMINPUTS / "coarse"
_FINE_DIR = _FF_FARMINPUTS / "fine"


# =========================================================================
# 入流 .bts 命名与映射 —— (风速, 湍流度, seed) 三维唯一键
# =========================================================================
BTS_SPEEDS = [6.0, 8.0, 10.0, 12.0]      # 约束 4
BTS_TIS = [0.05, 0.10, 0.15]
SEED_IDS = [1, 2, 3, 4, 5, 6]            # config/seed_manifest.csv：11001..66006


def bts_name(wind_speed_ms: float, turbulence_intensity: float,
             seed_id: Optional[int] = None) -> str:
    """
    生成 .bts 文件名。

    * seed_id=None（旧样式，兼容）：inflow_06ms_TI05.bts
    * seed_id 给定（V2 唯一键）：inflow_U06_TI05_seed01.bts
    """
    s = int(round(wind_speed_ms))
    ti = int(round(turbulence_intensity * 100))
    if seed_id is None:
        return f"inflow_{s:02d}ms_TI{ti:02d}.bts"
    return f"inflow_U{s:02d}_TI{ti:02d}_seed{int(seed_id):02d}.bts"


# 预构建 (速度, TI) -> 文件名（旧样式 12 盒，仅兼容/探针用；**新代码用 resolve_bts_file**）
AVAILABLE_BTS: Dict[tuple, str] = {
    (s, ti): bts_name(s, ti) for s in BTS_SPEEDS for ti in BTS_TIS
}


def resolve_bts_file(
    wind_speed_ms: float,
    turbulence_intensity: float,
    seed_id: int,
    *,
    require_exists: bool = True,
    fine: bool = False,
) -> Optional[str]:
    """
    按 (风速, TI, seed) 唯一键解析 .bts 相对文件名（相对 study/FarmInputs）。

    - 不允许 spacing / WD / control 参与解析（约束/开发计划 §4.4）；
    - require_exists=True 时要求文件实际存在（找不到返回 None）；
    - fine=True 时查 FarmInputs/fine/（DEL 阶段精细盒）。
    """
    # bts_name 已含 .bts 后缀（seed 键新样式）；勿再拼接
    name = bts_name(wind_speed_ms, turbulence_intensity, seed_id)
    base = _FINE_DIR if fine else _COARSE_DIR
    if require_exists and not (base / name).exists():
        return None
    rel = ("fine" if fine else "coarse") + "/" + name
    return rel


# 向后兼容旧名（仅按风速，忽略 TI/seed；不建议使用）
def inflow_bts_for_speed(wind_speed_ms: float, tol: float = 0.6) -> Optional[str]:
    """[Deprecated] 仅按风速匹配（取该风速下 TI=0.10、seed=1 的 .bts 作为代表）。"""
    return resolve_bts_file(
        wind_speed_ms, 0.10, 1, require_exists=False,
    )


def inflow_bts_for_case(
    wind_speed_ms: float,
    turbulence_intensity: float,
    *,
    spacing_D: Optional[float] = None,
    seed_id: Optional[int] = None,
    speed_tol: float = 0.6,
    ti_tol: float = 0.011,
    require_exists: bool = True,
) -> Optional[str]:
    """
    [Deprecated / 兼容] 旧版 (U, TI[, spacing]) -> .bts 映射。

    V2 起 .bts 唯一键 = (U, TI, seed)；spacing_D 不再影响文件名（忽略并提示）。
    * seed_id 给定 -> 走 resolve_bts_file；
    * seed_id=None -> 旧样式 AVAILABLE_BTS 最近邻（保持旧测试/探针可用）。
    """
    if seed_id is not None:
        return resolve_bts_file(wind_speed_ms, turbulence_intensity, seed_id,
                                require_exists=require_exists)
    if spacing_D is not None:
        import warnings
        warnings.warn("spacing_D no longer affects bts identity (V2); ignored.",
                      DeprecationWarning, stacklevel=2)
    best, best_cost = None, 1e9
    for (spd, ti), fname in AVAILABLE_BTS.items():
        ds = abs(spd - wind_speed_ms)
        dti = abs(ti - turbulence_intensity)
        if ds <= speed_tol and dti <= ti_tol:
            cost = ds + 100.0 * dti
            if cost < best_cost:
                best, best_cost = fname, cost
    if best is None:
        return None
    if require_exists and not (_FF_FARMINPUTS / best).exists():
        return None
    return best


# =========================================================================
# Case 数据结构
# =========================================================================

@dataclass
class Case:
    """单个工况（含 seed 身份；seed 是 FAST.Farm 侧 realization 键）。"""
    wind_speed_ms: float
    wind_direction_offset_deg: float   # 相对轴向(270°)的偏移；仅 {0, 10, 20}
    turbulence_intensity: float
    spacing_D: float
    seed_id: int = 1
    # 仿真参数（从 grid.yaml 注入；审查 P0-5：时长只存 t_sim/t_settle/control_dt，计数派生）
    floris_dt: float = 60.0
    floris_max_iter: int = 3
    control_dt_s: float = 3.0
    t_sim_s: float = 1200.0
    t_settle_s: float = 600.0

    @property
    def n_total_control_steps(self) -> int:
        """总控制步数 = t_sim / control_dt（派生，不独立配置）。"""
        return int(round(self.t_sim_s / self.control_dt_s)) if self.control_dt_s else 0

    @property
    def n_statistics_steps(self) -> int:
        """统计窗口控制步数 = (t_sim - t_settle) / control_dt（派生）。"""
        return int(round((self.t_sim_s - self.t_settle_s) / self.control_dt_s)) \
            if self.control_dt_s else 0

    # -- 旧字段兼容别名（Deprecated；steady/ 子包与历史脚本可用）--
    @property
    def fastfarm_dt(self) -> float:
        return self.control_dt_s

    @property
    def fastfarm_t_settle_s(self) -> float:
        return self.t_settle_s

    @property
    def fastfarm_n_control_steps(self) -> int:
        return self.n_statistics_steps

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
    """稳定、唯一、可读的工况 ID（含 seed）。

    示例：U08_TI10_seed03_wd10_s6D
    """
    s = int(round(case.wind_speed_ms))
    ti = int(round(case.turbulence_intensity * 100))
    return (f"U{s:02d}_TI{ti:02d}_seed{int(case.seed_id):02d}"
            f"_wd{case.wind_direction_offset_deg:g}_s{case.spacing_D:g}D")


def with_seed(case: Case, seed_id: int) -> Case:
    """同工况、换 seed（dataclasses.replace 语义）。"""
    from dataclasses import replace
    return replace(case, seed_id=int(seed_id))


# 风场旋转法下的 yaw 目标约定（审查 P0-3，identity 实测校准）：
#   FAST.Farm inflow 气象风向恒为 270°（AXIAL），WD offset 由 layout 旋转表达；
#   风机初始 NacYaw_i = (270 - direction) = 0°（OpenFAST 惯例：NacYaw=0 == 机舱对齐 +X/迎风）。
#   因此**对齐参考 = NacYaw 系的 0°**，与 case WD 无关；yaw 命令只表达失准角。
#   绝对目标（NacYaw 系）= 0 + misalignment（%360）。
#   警示：身份测试实测“对齐=270°”的版本会把 yaw0 命令错算成 ~270°，导致
#   机舱大幅偏航、farm 功率下降 ~3.9% 越限 —— 必须用 NacYaw 系（identity 先于 DOE 的价值）。
ALIGN_YAW_DEG = 0.0  # NacYaw 系的对齐参考（等价于气象 270° 来风、机舱正对 +X）


def yaw_target_deg(yaw_misalignment_deg: float) -> float:
    """由失准角求绝对 yaw 目标（OpenFAST NacYaw 系，deg）。

    仅用于旋转法 inflow=270°（气象）的坐标系；禁止把 (270 - wd) 再计入
    （wd 已由 layout 旋转表达，重复计入会造成 WD 双计）。
    """
    return float((ALIGN_YAW_DEG + float(yaw_misalignment_deg)) % 360.0)


# =========================================================================
# 布局（间距替换 —— 不改模板；风场旋转法不变）
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
    **风场旋转法**：把单列风机绕质心旋转 -offset，使入流始终沿 +X。

    已验证：旋转后(风向=270) 与 原始(风向=270+offset) 的相对几何完全一致，
    FLORIS/FAST.Farm 物理等价，对比仍然成立。
    """
    import numpy as np

    s_m = spacing_D * C.ROTOR_DIAMETER_M
    xs = np.array([i * s_m for i in range(n_turbines)], dtype=float)
    ys = np.zeros(n_turbines, dtype=float)

    th = np.radians(-float(wind_direction_offset_deg))
    c, s = np.cos(th), np.sin(th)
    xr = c * xs - s * ys
    yr = s * xs + c * ys
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

    V2 统一盒宽 1500 m 覆盖全部工况（8D/20°：列展宽 ~690 m，需 half ~723 m < 750 m ✓）。
    本函数保留为 QA / 充分性校验口径。
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
    """某间距下、覆盖其全部风向偏差所需的单一盒宽（米，向上取整）。"""
    import numpy as np

    w = required_gridwidth_m(spacing_D, max_abs_offset_deg)
    return float(np.ceil(w / round_to_m) * round_to_m)


# =========================================================================
# 配置构建（FLORIS / FAST.Farm 共用同一 Case）
# =========================================================================

def make_floris_config(case: Case, output_dir: Optional[str] = None) -> FlorisConfig:
    """
    构建 FLORIS 配置。**风场旋转法**：旋转后坐标 + 基准风向 270°。
    FLORIS 与 seed 无关（稳态模型），case.seed_id 仅作身份记录。
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
        turbine_type="nrel_5MW",
        output_dir=output_dir,
    )
    return cfg


def make_fastfarm_config(
    case: Case,
    output_dir: Optional[str] = None,
    *,
    strict_inflow: bool = True,
    fine: bool = False,
) -> FastFarmConfig:
    """
    构建 FAST.Farm 配置。入流按 (风速, TI, seed) 唯一键解析（resolve_bts_file）。

    约束 3/5：新盒只位于 study/FarmInputs/（WFCRL_FARMINPUTS_DIR 注入解析）；
    无匹配盒时 strict_inflow=True 抛错（拒绝悄悄回退稳态，TI 维度失真）。
    """
    xs, ys = layout_for_case(case.spacing_D, case.wind_direction_offset_deg, n_turbines=3)

    bts = resolve_bts_file(
        case.wind_speed_ms, case.turbulence_intensity, case.seed_id,
        require_exists=True, fine=fine,
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
        # 充分性预检：统一盒宽须覆盖该工况（1500 m 覆盖 8D/20°；此处兜底校验）
        need_w = required_gridwidth_m(case.spacing_D, case.wind_direction_offset_deg)
        if need_w > 1500.0 + 1e-6:
            raise ValueError(
                f"[make_fastfarm_config] 工况 U={case.wind_speed_ms} "
                f"wd={case.wind_direction_offset_deg:g} s={case.spacing_D:g}D 需要盒宽 "
                f"{need_w:.0f} m > 1500 m（统一盒），请检查 wind_conditions.yaml 的 grid_width_m。"
            )
    else:
        expected = bts_name(case.wind_speed_ms, case.turbulence_intensity, case.seed_id)
        msg = (f"No matching .bts for U={case.wind_speed_ms} TI={case.turbulence_intensity} "
               f"seed={case.seed_id} (expected '{expected}' in {_COARSE_DIR}). "
               f"Generate it with generate_wind_boxes.py (或设 strict_inflow=False 回退稳态，不推荐)。")
        if strict_inflow:
            raise FileNotFoundError(msg)
        import warnings
        warnings.warn(msg)
        wind = WindConfig(
            wind_type=WindType.STEADY,
            speed=case.wind_speed_ms,
            direction=C.AXIAL_WIND_DIRECTION_DEG,
            turbulence_intensity=case.turbulence_intensity,
            reference_height=C.HUB_HEIGHT_M,
        )

    # 总仿真时长 = t_sim（= t_settle + 统计窗口）；总控制步数派生（审查 P0-5）
    max_iter = case.n_total_control_steps
    cfg = FastFarmConfig(
        case_name=f"ff_{case.id}",
        num_turbines=3,
        xcoords=xs, ycoords=ys,
        dt=case.control_dt_s,
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


def _grid_lists(grid: dict, subset: bool):
    """返回 (speeds, offsets, tis, spacings, seeds)；subset 时用 smoke_subset。"""
    src = grid["smoke_subset"] if (subset and "smoke_subset" in grid) else grid
    return (
        src["wind_speeds_ms"],
        src["wind_direction_offsets_deg"],
        src["turbulence_intensities"],
        src["spacings_D"],
        src.get("seeds", [1]),
    )


def iter_cases(grid: dict, subset: bool = False) -> Iterator[Case]:
    """
    遍历网格生成 Case（**不展开 seed**，seed_id=1；FLORIS/LUT 用）。
    """
    speeds, offsets, tis, spacings, seeds0 = _grid_lists(grid, subset)
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
            control_dt_s=float(ff.get("control_dt_s", ff.get("dt", 3.0))),
            t_sim_s=float(ff.get("t_sim_s", 1200.0)),
            t_settle_s=float(ff.get("t_settle_s", 600.0)),
        )


def iter_seeded_cases(grid: dict, subset: bool = False,
                      seed_ids: Optional[List[int]] = None) -> Iterator[Case]:
    """
    遍历网格 x seed（FAST.Farm 全 DOE / case_manifest 用）。

    顺序：seed 外层 -> (U, wd, TI, S) 内层（与 build_case_manifest 一致）。
    """
    seeds0 = _grid_lists(grid, subset)[4] if seed_ids is None else seed_ids
    for seed_id in seeds0:
        for case in iter_cases(grid, subset=subset):
            yield with_seed(case, seed_id)


def n_cases(grid: dict, subset: bool = False) -> int:
    """LUT 视角的工况数（不展开 seed）。"""
    return sum(1 for _ in iter_cases(grid, subset=subset))


def n_seeded_cases(grid: dict, subset: bool = False,
                   seed_ids: Optional[List[int]] = None) -> int:
    """FAST.Farm 视角的工况数（展开 seed）；再乘 controls 得 case 总数。"""
    return sum(1 for _ in iter_seeded_cases(grid, subset=subset, seed_ids=seed_ids))


__all__ = [
    "Case", "case_id", "with_seed", "iter_cases", "iter_seeded_cases",
    "n_cases", "n_seeded_cases", "load_grid",
    "layout_for_spacing", "layout_for_case",
    "required_gridwidth_m", "recommended_gridwidth_for_spacing",
    "bts_name", "resolve_bts_file", "inflow_bts_for_case", "inflow_bts_for_speed",
    "AVAILABLE_BTS", "BTS_SPEEDS", "BTS_TIS", "SEED_IDS",
    "make_floris_config", "make_fastfarm_config",
    "_FF_FARMINPUTS", "_COARSE_DIR", "_FINE_DIR",
]
