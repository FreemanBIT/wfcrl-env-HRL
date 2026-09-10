"""
Core data types shared across the closed-loop control package.

These are intentionally plain dataclasses (no heavy deps) so every module —
bridge, sensing, surrogate, observers, controllers, demos — speaks the same
vocabulary.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Optional
import math


@dataclass
class TurbineMeas:
    """One turbine's raw measurement, parsed from ``measurements_T<id>.txt``.

    Fields mirror the FAST.Farm bridge output (``DISCON_bridge.f90``). Angles are
    in degrees, power in kW (as written by the bridge), speeds in rpm.
    """
    turbine_id: int
    step: int = -1
    t: float = 0.0
    genpwr_kw: float = 0.0        # generator power (kW)
    genspd_rpm: float = 0.0       # generator speed (rpm)
    gentq_nm: float = 0.0         # generator torque (Nm)
    rotspd_rpm: float = 0.0       # rotor speed (rpm)
    wind_x: float = 0.0           # HorWindV — nacelle horizontal wind speed (m/s)
    blpitch_deg: float = 0.0      # blade pitch (deg)
    nacyaw_deg: float = 0.0       # nacelle yaw / heading (deg)
    mip1_knm: float = 0.0         # blade-root in-plane moment (kNm)
    moop1_knm: float = 0.0        # blade-root out-of-plane moment (kNm)
    mzb1_knm: float = 0.0         # blade-root Mz (kNm)

    @property
    def genpwr_w(self) -> float:
        return self.genpwr_kw * 1.0e3

    @property
    def genspd_rads(self) -> float:
        return self.genspd_rpm * math.pi / 30.0


@dataclass
class Cmd:
    """One turbine's control command (current ZMQ architecture).

    Primary channels (SCADA/RCP semantics):
      yaw_delta_deg : 偏航动作指令——转动多少度（顺时针为正，deg）；直通 yaw_delta 通道
      yaw_deg       : 绝对机舱方位目标（deg，OpenFAST 坐标）——控制器内部表示；
                       桥内换算为动作指令（target - heading），语义等价
      induction_a   : 诱导因子（无量纲，a=1/3 为贪婪）——直通 induction_ref 通道

    Legacy fields (file-bridge era; deprecated, removed in refactor step D):
      power_mw      : 功率目标（MW）——兼容入口，桥内先转 induction_a
      pitch_deg     : 桨距（deg）——legacy 近似映射入口
      torque_nm     : 扭矩（Nm）——新架构无直接通道，忽略并告警
    """
    turbine_id: int
    yaw_delta_deg: Optional[float] = None   # 偏航动作指令（CW+ 增量，deg）
    yaw_deg: Optional[float] = None         # 绝对机舱方位目标（deg）
    induction_a: Optional[float] = None     # 诱导因子（无量纲）
    power_mw: Optional[float] = None        # (legacy) 功率目标 MW
    pitch_deg: Optional[float] = None       # (legacy) 桨距 deg
    torque_nm: Optional[float] = None       # (legacy) 扭矩 Nm

    def to_line(self) -> str:
        """(legacy) 文件桥行渲染——已弃用，仅测试引用保留。"""
        parts = [f"T{self.turbine_id}"]
        if self.yaw_deg is not None:
            parts.append(f"yaw={self.yaw_deg:.4f}")
        if self.pitch_deg is not None:
            parts.append(f"pitch={self.pitch_deg:.4f}")
        if self.torque_nm is not None:
            parts.append(f"torque={self.torque_nm:.2f}")
        if self.power_mw is not None:
            parts.append(f"power={self.power_mw:.6f}")
        return " ".join(parts)



@dataclass
class TurbineFlow:
    """Per-turbine flow estimate produced by the sensing layer."""
    turbine_id: int
    U: float = 0.0        # local inflow speed estimate (m/s)
    phi: float = 270.0    # local inflow direction estimate (deg)
    P: float = 0.0        # (filtered) power (kW)
    yaw: float = 0.0      # current nacelle yaw (deg)


@dataclass
class FlowEstimate:
    """Farm-level + per-turbine flow estimate consumed by observers/controllers."""
    U_inf: float = 8.0                       # free-stream wind speed (m/s)
    phi: float = 270.0                       # free-stream wind direction (deg)
    TI_est: float = 0.06                     # estimated turbulence intensity (-)
    per_turbine: dict[int, TurbineFlow] = field(default_factory=dict)
    t: float = 0.0
    step: int = -1

    def U_eff(self, turbine_id: int) -> float:
        tf = self.per_turbine.get(turbine_id)
        return tf.U if tf is not None else self.U_inf