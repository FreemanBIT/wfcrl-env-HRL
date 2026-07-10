"""
power_setpoint_tools.py — 转换层核心 (Stage 1)
==============================================
本模块解决整个对比研究中最关键的"口径统一"问题。

背景（来自工程源码 wfcrl/interface.py 的实测语义）
---------------------------------------------------
两个仿真器对 `ControlInput.power` 的解释 **不同**：

* FLORIS (`FlorisInterface.step`, mode 1/2)：
    `controls.power` 被当作 **限功率比 ratio ∈ (0.01, 1.0]**。
    内部对 power_thrust_table 的 power/Ct 曲线整体缩放：
        power_new = power_base * ratio
    并用一维制动盘关系由缩放后的 Cp 反解新的轴向诱导 a，
    再写回 Ct = 4a(1-a)。即 FLORIS 端是"按比例降额"。

* FAST.Farm (DISCON_bridge.f90, mode 1/4)：
    `controls.power` 是 **绝对目标功率 (MW)**。
    bridge 用前馈 T_ff = P_target(W)/ω + PI 修正跟踪该功率。

因此：要让"同一个最优工作点"在两个仿真器里物理一致，必须建立
    绝对功率 P_target(MW)  ⇄  限功率比 ratio
的双向映射，并约定 **以绝对功率 (MW) 作为 LUT 的标准存储量**，
下发时各自转换：
    - FAST.Farm：直接用 P_target(MW)        → mode 1 / mode 4
    - FLORIS：  ratio = P_target / P_greedy(U) → mode 1 / mode 4

其中 P_greedy(U) 是该风速下不降额(贪婪)时的功率，可由 FLORIS 基准跑或由
power_thrust_table 查得。本模块提供解析近似 greedy_power_w()，但 **推荐**
在优化脚本中用 FLORIS 实跑的基准功率来标定 ratio（见 optimize/，更精确）。

诱导因子 a 在本研究中是 **诊断/汇报量**（由 P_target 与 U 反算），不直接下发。

依赖：numpy、scipy。复用 wfcrl.config.ControlInput 的工厂方法（5-mode）。
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional, Sequence

import numpy as np

from induction_vs_yaw_study import constants as C

# wfcrl 的控制输入数据结构（5-mode）。本模块只调用其公开工厂方法。
from wfcrl.config import ControlInput


# =========================================================================
# 数据结构
# =========================================================================

@dataclass
class Setpoint:
    """单台风机的降额工作点（诊断 + 下发所需全部量）。"""
    power_target_mw: float      # 绝对目标功率 (MW) —— LUT 标准存储量
    ratio: float                # 相对贪婪功率的限功率比 (FLORIS 下发用)
    induction: float            # 诊断轴向诱导因子 a
    ct: float                   # 诊断推力系数
    cp: float                   # 诊断功率系数（气动）
    min_pitch_deg: float = 0.0  # 最小桨距约束（默认 0，交由 ROSCO 处理）


# =========================================================================
# 一维制动盘关系（仅用于诊断 a / Ct）
# =========================================================================

def cp_of_a(a: float) -> float:
    """Cp = 4a(1-a)^2."""
    return 4.0 * a * (1.0 - a) ** 2


def ct_of_a(a: float) -> float:
    """Ct = 4a(1-a)."""
    return 4.0 * a * (1.0 - a)


def a_of_ct(ct: float) -> float:
    """由 Ct 反解物理根 a (取 a<0.5 的小根)。Ct=4a(1-a)。"""
    ct = float(np.clip(ct, 0.0, 0.999))
    return 0.5 * (1.0 - np.sqrt(1.0 - ct))


def greedy_power_w(wind_speed_ms: float) -> float:
    """
    该风速下贪婪(不降额)电功率的 **解析近似** (W)。

    用 Betz 上限 Cp=16/27 估算，并封顶到额定功率。
    注意：这是粗略近似，真实贪婪功率应由 FLORIS / power_thrust_table 查得。
    优化脚本会用 FLORIS 实跑基准覆盖该值；此函数仅作回退与单测参考。
    """
    p = C.power_from_cp(C.cp_max_betz(), wind_speed_ms, electrical=True)
    return float(min(p, C.P_RATED_W))


# =========================================================================
# 绝对功率 ⇄ 诱导因子（诊断）
# =========================================================================

def induction_from_power(
    power_target_w: float,
    wind_speed_ms: float,
    *,
    electrical: bool = True,
) -> float:
    """
    由绝对目标功率与风速反算诊断轴向诱导因子 a（取 a∈[0,1/3] 物理根）。

    P = 0.5 ρ A Cp(a) U^3 (· η)   →  Cp_target  →  解 4a(1-a)^2 = Cp_target。
    """
    ws = max(float(wind_speed_ms), 1e-6)
    denom = 0.5 * C.AIR_DENSITY * C.ROTOR_AREA_M2 * ws ** 3
    if electrical:
        denom *= C.DRIVETRAIN_EFFICIENCY
    cp_target = float(power_target_w) / denom if denom > 0 else 0.0
    cp_target = float(np.clip(cp_target, 0.0, C.cp_max_betz()))

    # 在 a ∈ [0, 1/3] 上 Cp 单调增，用二分求解
    lo, hi = 0.0, C.A_GREEDY
    for _ in range(80):
        mid = 0.5 * (lo + hi)
        if cp_of_a(mid) < cp_target:
            lo = mid
        else:
            hi = mid
    return 0.5 * (lo + hi)


def power_from_induction(
    a: float,
    wind_speed_ms: float,
    *,
    electrical: bool = True,
) -> float:
    """由轴向诱导 a 与风速计算电(或气动)功率 (W)。互逆校验用。"""
    return C.power_from_cp(cp_of_a(float(a)), float(wind_speed_ms), electrical=electrical)


def ct_from_power(power_target_w: float, wind_speed_ms: float, **kw) -> float:
    """由绝对目标功率反算诊断推力系数 Ct。"""
    a = induction_from_power(power_target_w, wind_speed_ms, **kw)
    return ct_of_a(a)


# =========================================================================
# 绝对功率(MW) ⇄ 限功率比 ratio  —— FLORIS 下发桥梁
# =========================================================================

def power_mw_to_ratio(
    power_target_mw: float,
    wind_speed_ms: float,
    *,
    greedy_power_w_value: Optional[float] = None,
) -> float:
    """
    绝对目标功率(MW) → FLORIS 限功率比 ratio。

    ratio = P_target / P_greedy(U)，裁剪到 (0.01, 1.0]。

    Parameters
    ----------
    greedy_power_w_value : Optional[float]
        该风速下贪婪功率 (W)。若提供（推荐由 FLORIS 实跑得到），用之；
        否则回退到 greedy_power_w(U) 解析近似。
    """
    p_greedy = greedy_power_w_value if greedy_power_w_value is not None else greedy_power_w(wind_speed_ms)
    if p_greedy <= 0:
        return 1.0
    ratio = (float(power_target_mw) * 1e6) / p_greedy
    return float(np.clip(ratio, 0.01, 1.0))


def ratio_to_power_mw(
    ratio: float,
    wind_speed_ms: float,
    *,
    greedy_power_w_value: Optional[float] = None,
) -> float:
    """FLORIS 限功率比 ratio → 绝对目标功率(MW)。power_mw_to_ratio 的逆。"""
    p_greedy = greedy_power_w_value if greedy_power_w_value is not None else greedy_power_w(wind_speed_ms)
    return float(np.clip(ratio, 0.01, 1.0)) * p_greedy / 1e6


# =========================================================================
# 构造 Setpoint
# =========================================================================

def make_setpoint(
    power_target_mw: float,
    wind_speed_ms: float,
    *,
    greedy_power_w_value: Optional[float] = None,
    min_pitch_deg: float = 0.0,
) -> Setpoint:
    """由绝对目标功率(MW)与风速构造完整 Setpoint（含诊断 a/Ct/Cp 与 FLORIS ratio）。"""
    p_w = float(power_target_mw) * 1e6
    a = induction_from_power(p_w, wind_speed_ms)
    ratio = power_mw_to_ratio(power_target_mw, wind_speed_ms,
                              greedy_power_w_value=greedy_power_w_value)
    return Setpoint(
        power_target_mw=float(power_target_mw),
        ratio=ratio,
        induction=a,
        ct=ct_of_a(a),
        cp=cp_of_a(a),
        min_pitch_deg=float(min_pitch_deg),
    )


# =========================================================================
# ControlInput 构造（单一来源）—— 两个仿真器都从这里拿控制输入
# =========================================================================

def build_yaw_control(yaw_deg: Sequence[float]) -> ControlInput:
    """
    构造纯偏航控制 (mode=0)。FLORIS 与 FAST.Farm 通用。

    yaw_deg : 每台风机的绝对偏航角 (deg, OpenFAST 坐标: 0°=+X东, +逆时针)。
    """
    yaw = np.asarray(yaw_deg, dtype=np.float64).ravel()
    n = len(yaw)
    return ControlInput(
        mode=np.zeros(n, dtype=np.int32),
        yaw=yaw,
        pitch=np.zeros(n, dtype=np.float64),
        power=np.zeros(n, dtype=np.float64),
        min_pitch=np.zeros(n, dtype=np.float64),
    )


def build_derating_control(
    sim_kind: str,
    *,
    power_target_mw: Sequence[float],
    wind_speed_ms: float,
    yaw_deg: Optional[Sequence[float]] = None,
    min_pitch_deg: Optional[Sequence[float]] = None,
    greedy_power_w_per_turbine: Optional[Sequence[float]] = None,
) -> ControlInput:
    """
    构造降额(诱导)控制输入 —— **核心单一来源函数**。

    LUT 中存的是每台机的绝对目标功率 (MW)。本函数据此为指定仿真器构造
    正确语义的 ControlInput：

    * sim_kind == "fastfarm":
        mode=1 (或 mode=4 若给 yaw)，`power` 字段 = 绝对功率 (MW)。
    * sim_kind == "floris":
        mode=1 (或 mode=4 若给 yaw)，`power` 字段 = 限功率比 ratio，
        ratio = P_target / P_greedy(U)。

    Parameters
    ----------
    sim_kind : {"fastfarm", "floris"}
    power_target_mw : 每台机绝对目标功率 (MW)。
    wind_speed_ms : 来流风速 (m/s)，用于 FLORIS 的 ratio 换算。
    yaw_deg : 可选，每台机偏航角 (deg)。给定则用 mode=4（功率+偏航）。
    min_pitch_deg : 可选，每台机最小桨距 (deg)，默认 0。
    greedy_power_w_per_turbine : 可选，每台机贪婪功率 (W)，用于精确 ratio。
        推荐由 FLORIS 基准跑提供（同一 U 下三机贪婪功率可能因尾流而不同，
        但降额 LUT 通常以自由来流贪婪功率为基准，使 ratio 物理含义清晰）。
    """
    sim_kind = sim_kind.lower()
    p_mw = np.asarray(power_target_mw, dtype=np.float64).ravel()
    n = len(p_mw)

    if min_pitch_deg is None:
        mp = np.zeros(n, dtype=np.float64)
    else:
        mp = np.asarray(min_pitch_deg, dtype=np.float64).ravel()

    if yaw_deg is None:
        yaw = np.zeros(n, dtype=np.float64)
        mode_val = 1
    else:
        yaw = np.asarray(yaw_deg, dtype=np.float64).ravel()
        mode_val = 4

    if sim_kind == "fastfarm":
        power_field = p_mw.copy()                      # 绝对 MW
    elif sim_kind == "floris":
        gp = (None if greedy_power_w_per_turbine is None
              else np.asarray(greedy_power_w_per_turbine, dtype=np.float64).ravel())
        power_field = np.array([
            power_mw_to_ratio(
                p_mw[i], wind_speed_ms,
                greedy_power_w_value=(gp[i] if gp is not None else None),
            )
            for i in range(n)
        ], dtype=np.float64)                            # 限功率比
    else:
        raise ValueError(f"sim_kind must be 'fastfarm' or 'floris', got {sim_kind!r}")

    return ControlInput(
        mode=np.full(n, mode_val, dtype=np.int32),
        yaw=yaw,
        pitch=np.zeros(n, dtype=np.float64),
        power=power_field,
        min_pitch=mp,
    )
