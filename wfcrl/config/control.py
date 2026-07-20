"""
控制输入数据结构
================
定义 ControlInput：单步控制输入，所有接口共用。

5-Mode Farm Control Protocol (DISCON.F90 WFCRL Bridge):
    mode=0: yaw only
    mode=1: power target + min pitch constraint
    mode=2: pitch absolute
    mode=3: pitch absolute + yaw
    mode=4: power target + min pitch + yaw
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

import numpy as np


@dataclass
class ControlInput:
    """
    单步控制输入，所有接口共用。
    5-Mode Farm Control Protocol (DISCON.F90 WFCRL Bridge):
        mode=0: yaw delta only
        mode=1: power target + min pitch constraint
        mode=2: pitch absolute
        mode=3: pitch absolute + yaw delta
        mode=4: power target + min pitch + yaw delta

    Attributes
    ----------
    mode : np.ndarray
        控制模式 (0-4)，shape (n_turbines,)，整数。
    yaw : np.ndarray
        偏航角绝对值 (deg)，shape (n_turbines,)。OpenFAST 坐标 (0°=东+X, +逆时针)。
    pitch : np.ndarray
        变桨指令 (deg, 绝对值)，shape (n_turbines,)。
    power : Optional[np.ndarray]
        功率目标 (MW)，shape (n_turbines,)。mode=1/4 时使用。
    min_pitch : Optional[np.ndarray]
        最小变桨约束 (deg)，shape (n_turbines,)。mode=1/4 时使用。
    """
    mode: np.ndarray
    yaw: np.ndarray
    pitch: np.ndarray
    power: Optional[np.ndarray] = None
    min_pitch: Optional[np.ndarray] = None

    def __post_init__(self):
        self.mode = np.asarray(self.mode, dtype=np.int32)
        self.yaw = np.asarray(self.yaw, dtype=np.float64)
        self.pitch = np.asarray(self.pitch, dtype=np.float64)
        if self.power is not None:
            self.power = np.asarray(self.power, dtype=np.float64)
        if self.min_pitch is not None:
            self.min_pitch = np.asarray(self.min_pitch, dtype=np.float64)
        if self.mode.ndim == 0:
            self.mode = np.atleast_1d(self.mode)
        if self.yaw.ndim == 0:
            self.yaw = np.atleast_1d(self.yaw)
        if self.pitch.ndim == 0:
            self.pitch = np.atleast_1d(self.pitch)
        if self.power is not None and self.power.ndim == 0:
            self.power = np.atleast_1d(self.power)
        if self.min_pitch is not None and self.min_pitch.ndim == 0:
            self.min_pitch = np.atleast_1d(self.min_pitch)

    @classmethod
    def zero(cls, n_turbines: int) -> "ControlInput":
        """创建全零控制输入 (mode=0, 零指令)。"""
        return cls(
            mode=np.zeros(n_turbines, dtype=np.int32),
            yaw=np.zeros(n_turbines, dtype=np.float64),
            pitch=np.zeros(n_turbines, dtype=np.float64),
            power=np.zeros(n_turbines, dtype=np.float64),
            min_pitch=np.zeros(n_turbines, dtype=np.float64),
        )

    @classmethod
    def mode0_yaw(cls, n_turbines: int, yaw_deg: float) -> "ControlInput":
        """mode=0: 纯偏航绝对值控制。OpenFAST 坐标。"""
        return cls(
            mode=np.zeros(n_turbines, dtype=np.int32),
            yaw=np.full(n_turbines, yaw_deg, dtype=np.float64),
            pitch=np.zeros(n_turbines, dtype=np.float64),
        )

    @classmethod
    def mode1_power(cls, n_turbines: int, power_mw: float, min_pitch_deg: float = 0.0) -> "ControlInput":
        """mode=1: 功率目标 + 最小变桨约束。"""
        return cls(
            mode=np.ones(n_turbines, dtype=np.int32),
            yaw=np.zeros(n_turbines, dtype=np.float64),
            pitch=np.zeros(n_turbines, dtype=np.float64),
            power=np.full(n_turbines, power_mw, dtype=np.float64),
            min_pitch=np.full(n_turbines, min_pitch_deg, dtype=np.float64),
        )

    @classmethod
    def mode2_pitch(cls, n_turbines: int, pitch_deg: float) -> "ControlInput":
        """mode=2: 纯变桨绝对值控制。"""
        return cls(
            mode=np.full(n_turbines, 2, dtype=np.int32),
            yaw=np.zeros(n_turbines, dtype=np.float64),
            pitch=np.full(n_turbines, pitch_deg, dtype=np.float64),
        )

    @classmethod
    def mode3_pitch_yaw(cls, n_turbines: int, pitch_deg: float, yaw_deg: float) -> "ControlInput":
        """mode=3: 变桨绝对值 + 偏航绝对值。OpenFAST 坐标。"""
        return cls(
            mode=np.full(n_turbines, 3, dtype=np.int32),
            yaw=np.full(n_turbines, yaw_deg, dtype=np.float64),
            pitch=np.full(n_turbines, pitch_deg, dtype=np.float64),
        )

    @classmethod
    def mode4_power_yaw(cls, n_turbines: int, power_mw: float, min_pitch_deg: float, yaw_deg: float) -> "ControlInput":
        """mode=4: 功率目标 + 最小变桨约束 + 偏航绝对值。OpenFAST 坐标。"""
        return cls(
            mode=np.full(n_turbines, 4, dtype=np.int32),
            yaw=np.full(n_turbines, yaw_deg, dtype=np.float64),
            pitch=np.zeros(n_turbines, dtype=np.float64),
            power=np.full(n_turbines, power_mw, dtype=np.float64),
            min_pitch=np.full(n_turbines, min_pitch_deg, dtype=np.float64),
        )
