"""
仿真器状态与操作结果数据结构
===========================
定义 SimulatorState（单步测量值快照）和 ApplyResult（控制下发结果）。

SimulatorState 是 ContinuousFastFarmInterface 中 read_measurements()
的返回类型，用于 MPC / 非 RL 场景：
    state = sim.read_measurements()    # 获取当前测量值快照
    controls = solver.solve(state)      # 外部算法计算控制
    sim.apply(controls)                 # 下发控制

用法
----
from wfcrl.engine import SimulatorState, ApplyResult

state = SimulatorState(
    step_idx=5, sim_time=15.0,
    power_mw=np.array([4.8, 4.9, 5.0]),
    yaw_deg=np.array([0.0, 5.0, -3.0]),
)
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, Optional

import numpy as np


# =========================================================================
# SimulatorState — 测量值快照
# =========================================================================

@dataclass
class SimulatorState:
    """
    一次 read_measurements() 的快照，仿真器层的数据结构。

    与 SimulationOutput 的区别：
    - SimulationOutput   : 批量时序数据（n_steps, n_turbines）
    - SimulatorState     : 单步快照（n_turbines,），每次 read_measurements() 返回一个

    Attributes
    ----------
    step_idx : int
        当前步序号（从 0 开始）。
    sim_time : float
        当前仿真时间 (s)。
    power_mw : np.ndarray
        每台风机发电功率，shape (n_turbines,)。
    wind_speed : Optional[np.ndarray]
        每台风机处风速 (m/s)，shape (n_turbines,)。
    yaw_deg : Optional[np.ndarray]
        偏航角（相对来流的失准角，deg），shape (n_turbines,)。
    pitch_deg : Optional[np.ndarray]
        变桨角 (deg)，shape (n_turbines,)。
    torque_nm : Optional[np.ndarray]
        发电机转矩 (Nm)，shape (n_turbines,)。
    rotor_speed_rpm : Optional[np.ndarray]
        风轮转速 (RPM)，shape (n_turbines,)。
    blade_loads : Optional[np.ndarray]
        叶片载荷（根部弯矩），shape (n_turbines, 3)。
        [RootMIP1, RootMOoP1, RootMzb1]。
    n_turbines_measured : int
        实际读取到测量值的风机数量。若小于总风机数，说明部分风机测量缺失。
    metadata : dict
        额外元数据（如 {"warning": "only 2/3 turbines measured"}）。
    """

    step_idx: int
    sim_time: float
    power_mw: np.ndarray                                  # (n_turbines,)
    wind_speed: Optional[np.ndarray] = None               # (n_turbines,)
    yaw_deg: Optional[np.ndarray] = None                  # (n_turbines,) 相对来流失准角
    pitch_deg: Optional[np.ndarray] = None                # (n_turbines,)
    torque_nm: Optional[np.ndarray] = None                # (n_turbines,)
    rotor_speed_rpm: Optional[np.ndarray] = None          # (n_turbines,)
    blade_loads: Optional[np.ndarray] = None              # (n_turbines, 3)
    n_turbines_measured: int = 0
    metadata: dict = field(default_factory=dict)

    @property
    def n_turbines(self) -> int:
        """风机数量（从 power_mw 推断）。"""
        return len(self.power_mw) if self.power_mw is not None else 0

    @property
    def farm_power_mw(self) -> float:
        """风场总功率 (MW)。"""
        return float(np.sum(self.power_mw)) if self.power_mw is not None else 0.0


# =========================================================================
# ApplyResult — 控制下发结果
# =========================================================================

@dataclass
class ApplyResult:
    """
    一次 apply() 调用的结果。

    记录下发的控制命令、是否经过约束裁剪、以及约束状态。

    Attributes
    ----------
    step_idx : int
        下发时的步序号。
    controls_applied : object
        实际下发的 ControlInput（可能经过约束裁剪）。
    controls_requested : object
        请求的原始 ControlInput（裁剪前的值）。
    constraints_active : bool
        是否启用了执行器约束（ActuatorConstraints）。
    clipped_axes : Dict[str, bool]
        各控制维度是否被裁剪。如 {"yaw": True, "pitch": False}。
    constraints_reset : bool
        本步是否重置了约束追踪状态（例如 warmup 结束后）。
    metadata : dict
        额外元数据。
    """

    step_idx: int
    controls_applied: object
    controls_requested: object
    constraints_active: bool = False
    clipped_axes: Dict[str, bool] = field(default_factory=dict)
    constraints_reset: bool = False
    metadata: dict = field(default_factory=dict)
