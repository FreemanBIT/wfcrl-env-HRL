"""
执行器物理约束
==============
定义 ActuatorConstraints — 仿真器层的可选物理约束功能。

若 FastFarmConfig.actuator_constraints 为 None，不施加任何约束。
若设置，apply() 会在写 controls.txt 前自动裁剪。

约束类型
--------
1. 偏航速率约束 (yaw_rate_max)    : 偏航角变化率上限 (deg/s)
2. 变桨速率约束 (pitch_rate_max)  : 变桨角变化率上限 (deg/s)
3. 执行时间占比约束 (max_actuation_fraction) : 单控制维度最大执行时间占比，
   当 actuating_frac >= max_actuation_fraction 时将该维度指令回零。

用法
----
from wfcrl.engine import ActuatorConstraints

constraints = ActuatorConstraints(
    yaw_rate_max=0.3,           # deg/s
    pitch_rate_max=8.0,         # deg/s
    max_actuation_fraction=0.1,
)

# 在每一步 apply() 前检查
clipped = constraints.check(
    prev_controls=prev_cmd,
    new_controls=new_cmd,
    dt=3.0,
    accumulated=accumulator,
    num_steps=5,
)
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, Optional

import numpy as np

from wfcrl.config.control import ControlInput


# =========================================================================
# ActuatorConstraints — 执行器物理约束
# =========================================================================

@dataclass
class ActuatorConstraints:
    """
    执行器物理约束 — 仿真器层的可选功能。

    施加在 apply() 写 controls.txt 之前的裁剪逻辑。
    若为 None，不施加任何约束。

    Attributes
    ----------
    yaw_rate_max : float
        偏航最大速率 (deg/s)。默认 0.3 deg/s（典型 5MW 风机）。
    pitch_rate_max : float
        变桨最大速率 (deg/s)。默认 8.0 deg/s。
    max_actuation_fraction : float
        单控制维度最大执行时间占比。
        累积执行时间占比 >= 此值时，该维度指令回零。
        默认 0.1（即累计执行超过仿真总时长的 10% 时归零）。
    """

    yaw_rate_max: float = 0.3
    pitch_rate_max: float = 8.0
    max_actuation_fraction: float = 0.1

    def check(
        self,
        prev_controls: ControlInput,
        new_controls: ControlInput,
        dt: float,
        accumulated: Dict[str, np.ndarray],
        num_steps: int,
    ) -> ControlInput:
        """
        对 new_controls 施加约束，返回裁剪后的 ControlInput。

        约束逻辑
        --------
        1. 速率约束：yaw_rate_max / pitch_rate_max 限制单步最大变化量
        2. 执行时间占比约束：对各维度累计执行时间计数，
           当 actuating_frac >= max_actuation_fraction 时该维度归零

        Parameters
        ----------
        prev_controls : ControlInput
            上一步的控制输入。
        new_controls : ControlInput
            当前步请求的控制输入（将被裁剪）。
        dt : float
            仿真步长 (s)。
        accumulated : Dict[str, np.ndarray]
            各维度累计执行时间（由调用方维护，在 reset() 时清空）。
            键: "yaw", "pitch", "power"
            值: shape (n_turbines,) 的累计执行时间数组。
        num_steps : int
            到当前步为止的总步数。

        Returns
        -------
        ControlInput
            裁剪后的控制输入。
        """
        n = len(new_controls.mode)
        yaw_out = new_controls.yaw.copy()
        pitch_out = new_controls.pitch.copy()
        power_out = new_controls.power.copy() if new_controls.power is not None else None

        # 1. 偏航速率约束
        yaw_delta = np.abs(new_controls.yaw - prev_controls.yaw)
        max_yaw_delta = self.yaw_rate_max * dt
        overshoot = yaw_delta > max_yaw_delta
        if np.any(overshoot):
            direction = np.sign(new_controls.yaw - prev_controls.yaw)
            yaw_out[overshoot] = prev_controls.yaw[overshoot] + direction[overshoot] * max_yaw_delta

        # 2. 变桨速率约束
        pitch_delta = np.abs(new_controls.pitch - prev_controls.pitch)
        max_pitch_delta = self.pitch_rate_max * dt
        overshoot_p = pitch_delta > max_pitch_delta
        if np.any(overshoot_p):
            direction = np.sign(new_controls.pitch - prev_controls.pitch)
            pitch_out[overshoot_p] = prev_controls.pitch[overshoot_p] + direction[overshoot_p] * max_pitch_delta

        # 3. 执行时间占比约束
        total_time = num_steps * dt
        for dim_name, dim_arr in [("yaw", yaw_out), ("pitch", pitch_out), ("power", power_out)]:
            if dim_arr is None:
                continue
            acc = accumulated.get(dim_name, np.zeros(n))
            # 本步是否执行了该维度（非零变化）
            if dim_name == "yaw":
                actuating = np.abs(dim_arr - prev_controls.yaw) > 1e-6
            elif dim_name == "pitch":
                actuating = np.abs(dim_arr - prev_controls.pitch) > 1e-6
            elif dim_name == "power":
                prev_power = prev_controls.power if prev_controls.power is not None else np.zeros(n)
                actuating = np.abs(dim_arr - prev_power) > 1e-6
            else:
                actuating = np.zeros(n, dtype=bool)

            acc_new = acc.copy()
            acc_new[actuating] += dt
            accumulated[dim_name] = acc_new

            # 超限 → 归零
            exceed = acc_new > self.max_actuation_fraction * total_time
            if np.any(exceed):
                if dim_name == "yaw":
                    yaw_out[exceed] = 0.0
                elif dim_name == "pitch":
                    pitch_out[exceed] = 0.0
                elif dim_name == "power":
                    if power_out is not None:
                        power_out[exceed] = 0.0

        return ControlInput(
            mode=new_controls.mode.copy(),
            yaw=yaw_out,
            pitch=pitch_out,
            power=power_out,
            min_pitch=new_controls.min_pitch.copy() if new_controls.min_pitch is not None else None,
        )

    def reset(self) -> None:
        """重置所有累计执行量（本约束对象无状态，由调用方维护 accumulated dict）。"""
        # ActuatorConstraints 本身无状态，状态由调用方维护的 accumulated dict 持有。
        # 调用方在重置仿真时应清空其 accumulated dict。
        pass
