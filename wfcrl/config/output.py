"""
仿真输出数据结构
================
定义 SimulationOutput：统一的单步或批量仿真输出，所有接口共用。
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, Optional

import numpy as np
import pandas as pd


@dataclass
class SimulationOutput:
    """
    单步或批量仿真输出，所有接口共用。

    Attributes
    ----------
    time : np.ndarray
        时间向量 (s)，shape (n_steps,)。
    power_mw : np.ndarray
        每台风机功率 (MW)，shape (n_steps, n_turbines)。
    wind_speed : Optional[np.ndarray]
        每台风机处风速 (m/s)，shape (n_steps, n_turbines)。None 表示不可用。
    wind_direction : Optional[np.ndarray]
        每台风机处风向 (度)，shape (n_steps, n_turbines)。
    yaw_deg : Optional[np.ndarray]
        偏航角 (度)，shape (n_steps, n_turbines)。
    pitch_deg : Optional[np.ndarray]
        变桨角 (度)，shape (n_steps, n_turbines)。
    torque_nm : Optional[np.ndarray]
        转矩 (Nm)，shape (n_steps, n_turbines)。
    rotor_speed_rpm : Optional[np.ndarray]
        风轮转速 (RPM)，shape (n_steps, n_turbines)。
    generator_torque_nm : Optional[np.ndarray]
        发电机转矩 (Nm)，shape (n_steps, n_turbines)。
    thrust_n : Optional[np.ndarray]
        推力 (N)，shape (n_steps, n_turbines)。FLORIS 提供；FAST.Farm 可选。
    blade_loads : Optional[np.ndarray]
        叶片载荷 (根部弯矩)，shape (n_steps, n_turbines, 6)。
        FAST.Farm 提供 (RootMIP1, RootMOoP1, RootMzb1 ×2? 等)。
    farm_power_mw : np.ndarray
        风场总功率 (MW)，shape (n_steps,)。
    yaw_cmd_deg : Optional[np.ndarray]
        偏航控制指令（相对来流的失准角，deg），shape (n_steps, n_turbines)。
    ratio : Optional[np.ndarray]
        限电比例 = 目标功率 / 贪婪功率（1.0 = 不限电），shape (n_steps, n_turbines)。
    power_target_mw : Optional[np.ndarray]
        目标功率（MW），降额控制时有意义，shape (n_steps, n_turbines)。
    metadata : dict
        额外元数据。

    Methods
    -------
    to_dataframe() -> pd.DataFrame
        转为包含所有通道的 DataFrame。
    to_csv(path) -> None
        保存为 CSV。
    """

    time: np.ndarray
    power_mw: np.ndarray
    wind_speed: Optional[np.ndarray] = None
    wind_direction: Optional[np.ndarray] = None
    yaw_deg: Optional[np.ndarray] = None
    pitch_deg: Optional[np.ndarray] = None
    torque_nm: Optional[np.ndarray] = None
    rotor_speed_rpm: Optional[np.ndarray] = None
    generator_torque_nm: Optional[np.ndarray] = None
    thrust_n: Optional[np.ndarray] = None
    blade_loads: Optional[np.ndarray] = None
    farm_power_mw: Optional[np.ndarray] = None
    # ---- 控制输入（下发给仿真器的指令，便于在 timeseries 中核对）----
    yaw_cmd_deg: Optional[np.ndarray] = None
    ratio: Optional[np.ndarray] = None
    power_target_mw: Optional[np.ndarray] = None
    metadata: dict = field(default_factory=dict)

    def __post_init__(self):
        # 自动计算 farm_power_mw
        if self.farm_power_mw is None and self.power_mw is not None:
            self.farm_power_mw = np.sum(self.power_mw, axis=-1)

        # 确保至少 1d
        for attr in ("time", "power_mw", "farm_power_mw"):
            val = getattr(self, attr)
            if val is not None and val.ndim == 0:
                setattr(self, attr, np.atleast_1d(val))

        # 确保 2d
        for attr in ("power_mw", "wind_speed", "wind_direction", "yaw_deg",
                      "pitch_deg", "torque_nm", "rotor_speed_rpm",
                      "generator_torque_nm", "thrust_n",
                      "yaw_cmd_deg", "ratio", "power_target_mw"):
            val = getattr(self, attr)
            if val is not None and val.ndim == 1:
                setattr(self, attr, val.reshape(1, -1))

    @property
    def n_steps(self) -> int:
        return len(self.time) if self.time is not None else 0

    @property
    def n_turbines(self) -> int:
        if self.power_mw is not None:
            return self.power_mw.shape[-1]
        return 0

    def to_dataframe(self) -> pd.DataFrame:
        """转为包含所有可用通道的 DataFrame。"""
        rows: Dict[str, np.ndarray] = {"time_s": self.time}

        pw = self.power_mw
        if pw is not None:
            for t in range(pw.shape[-1]):
                rows[f"T{t+1}_power_MW"] = pw[:, t]
            rows["farm_power_MW"] = self.farm_power_mw if self.farm_power_mw is not None else np.sum(pw, axis=-1)

        for name, arr in [
            ("wind_speed", self.wind_speed),
            ("wind_direction", self.wind_direction),
            ("yaw_deg", self.yaw_deg),
            ("pitch_deg", self.pitch_deg),
            ("torque_nm", self.torque_nm),
            ("rotor_speed_rpm", self.rotor_speed_rpm),
            ("thrust_n", self.thrust_n),
            # ---- 控制输入列 ----
            ("yaw_cmd_deg", self.yaw_cmd_deg),
            ("ratio", self.ratio),
            ("power_target_mw", self.power_target_mw),
        ]:
            if arr is not None:
                for t in range(arr.shape[-1]):
                    rows[f"T{t+1}_{name}"] = arr[:, t]

        return pd.DataFrame(rows)

    def to_csv(self, path: str, **kwargs) -> None:
        """保存为 CSV 文件。"""
        self.to_dataframe().to_csv(path, index=False, **kwargs)

    @classmethod
    def single_step(cls, time: float, power_mw: np.ndarray, **kwargs) -> "SimulationOutput":
        """快捷构造单步输出。"""
        return cls(
            time=np.array([time]),
            power_mw=power_mw.reshape(1, -1),
            **kwargs,
        )
