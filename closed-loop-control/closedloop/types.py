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
    """One turbine's control command（新架构双通道）。

    yaw_delta_deg : 偏航动作指令（转动多少度，顺时针为正，deg）——SCADA/RCP 语义，直通 yaw_delta
    yaw_deg       : 绝对机舱方位目标（deg，OpenFAST 坐标）——控制器内部表示，桥内换算为动作指令
    induction_a   : 诱导因子（无量纲，a=1/3 为贪婪）——直通 induction_ref 通道
    """

    turbine_id: int
    yaw_delta_deg: Optional[float] = None
    yaw_deg: Optional[float] = None
    induction_a: Optional[float] = None

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