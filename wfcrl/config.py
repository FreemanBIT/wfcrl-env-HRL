"""
统一配置与输入/输出数据结构
===========================
定义所有仿真器（FAST.Farm / FLORIS）共用的：
- WindType / WindConfig / WindSegment：风况配置
- ControlInput：控制输入
- SimulationOutput：仿真输出

所有接口和调用方都依赖此模块的数据结构。
"""

from __future__ import annotations

import os
from dataclasses import dataclass, field
from enum import IntEnum
from typing import Dict, List, Optional, Union

import numpy as np
import pandas as pd


# =========================================================================
# WindType — 映射 InflowWind.dat 的 WindType 参数 (1-7)
# =========================================================================

class WindType(IntEnum):
    """
    FAST.Farm / InflowWind.dat 支持的风类型。

    FLORIS 兼容性：
    - STEADY (1): 完全支持
    - UNIFORM (2): FLORIS 用稳态近似
    - TURBSIM_BTS (3): FLORIS 用稳态近似 + 湍流强度
    - BLADED_BIN (4): FLORIS 用稳态近似
    - HAWC (5): FLORIS 用稳态近似
    - USER (6): FLORIS 用稳态近似
    - BLADED_NATIVE (7): FLORIS 用稳态近似
    """
    STEADY = 1          # 稳态风
    UNIFORM = 2         # 均匀风文件 (.dat)
    TURBSIM_BTS = 3     # 二进制 TurbSim 全流场 (.bts)
    BLADED_BIN = 4      # 二进制 Bladed 全流场 (.wnd/.sum)
    HAWC = 5            # HAWC 格式二进制 (.bin)
    USER = 6            # 用户自定义
    BLADED_NATIVE = 7   # 原生 Bladed 全流场


# FAST.Farm 中 FLORIS 不支持的风类型
_FASTFARM_ONLY_WINDTYPES = {
    WindType.UNIFORM, WindType.TURBSIM_BTS, WindType.BLADED_BIN,
    WindType.HAWC, WindType.USER, WindType.BLADED_NATIVE,
}


# =========================================================================
# WindSegment — 单段风况
# =========================================================================

@dataclass
class WindSegment:
    """
    一段时域风况。

    Attributes
    ----------
    duration : float
        持续时间 (秒)。使用 np.inf 表示无穷。
    speed : float
        风速 (m/s)。
    direction : float
        风向 (度，气象惯例，0=北风，顺时针)。
    turbulence : float
        湍流强度 (-)，默认 0.1。
    """
    duration: float
    speed: float
    direction: float
    turbulence: float = 0.1


# =========================================================================
# WindConfig — 统一风况配置
# =========================================================================

@dataclass
class WindConfig:
    """
    统一风况配置，可映射到 FAST.Farm InflowWind.dat 或 FLORIS flow_field。

    使用示例
    --------
    # 稳态风
    w1 = WindConfig(wind_type=WindType.STEADY, speed=10, direction=270)

    # TurbSim 风文件
    w2 = WindConfig(wind_type=WindType.TURBSIM_BTS, wind_file="90m_08mps.bts")

    # 多段风况
    w3 = WindConfig(
        wind_type=WindType.STEADY,
        segments=[
            WindSegment(duration=60, speed=8, direction=270),
            WindSegment(duration=60, speed=12, direction=300),
        ],
    )

    # 时序风 (从数组)
    w4 = WindConfig(
        wind_type=WindType.STEADY,
        wind_time_series=np.array([[8,270],[9,275],[10,280]]),
    )

    Attributes
    ----------
    wind_type : WindType
        风类型，对应 InflowWind.dat 的 WindType。
    speed : float
        稳态风速 (m/s)，WindType=1 时使用。
    direction : float
        风向 (度)，气象惯例。
    turbulence_intensity : float
        湍流强度 (-)，FLORIS 直接使用；FAST.Farm WindType=1 时忽略。
    shear_exponent : float
        风切变指数 (-)，默认 0.2。
    reference_height : float
        参考高度 (m)，默认 90。
    wind_file : Optional[str]
        风文件路径/名称 (WindType=2/3/4/5 时使用，如 "90m_08mps.bts")。
        相对路径相对于 FAST.Farm FarmInputs/ 目录。
    wind_time_series : Optional[np.ndarray]
        时序风速风向数组，shape (n, 2)，第一列为 speed，第二列为 direction。
        与 segments 互斥。
    segments : Optional[List[WindSegment]]
        多段风况列表。与 wind_time_series 互斥。
    """

    wind_type: WindType = WindType.STEADY
    speed: float = 8.0
    direction: float = 270.0
    turbulence_intensity: float = 0.1
    shear_exponent: float = 0.2
    reference_height: float = 90.0
    wind_file: Optional[str] = None
    wind_time_series: Optional[np.ndarray] = field(default=None, repr=False)
    segments: Optional[List[WindSegment]] = None

    # ---- 验证 ----

    def __post_init__(self):
        if self.wind_time_series is not None and self.segments is not None:
            raise ValueError("wind_time_series and segments are mutually exclusive")

    # ---- FLORIS 兼容性 ----

    @property
    def is_floris_supported(self) -> bool:
        """FLORIS 是否原生支持此 wind_type。仅 WindType.STEADY 原生支持。"""
        return self.wind_type == WindType.STEADY

    # ---- 转换为字典（用于 FLORIS / FAST.Farm 配置） ----

    def to_floris_dict(self) -> dict:
        """
        转为 FLORIS flow_field 参数字典。

        对于 FLORIS 不支持的风类型，使用稳态近似 + warning。
        """
        if self.wind_type not in (WindType.STEADY,):
            import warnings
            warnings.warn(
                f"FLORIS does not natively support WindType={self.wind_type.name}. "
                f"Using steady wind approximation (speed={self.speed}, direction={self.direction})."
            )

        result: dict = {
            "wind_speeds": [self.speed],
            "wind_directions": [self.direction],
            "turbulence_intensities": [self.turbulence_intensity],
            "wind_shear": self.shear_exponent,
        }

        # 多段风况 → 多条件 (FLORIS 多 findex)
        if self.segments:
            result["wind_speeds"] = [s.speed for s in self.segments]
            result["wind_directions"] = [s.direction for s in self.segments]
            result["turbulence_intensities"] = [s.turbulence for s in self.segments]

        return result

    def to_fastfarm_inflow_dict(self, farm_inputs_dir: str = "") -> dict:
        """
        转为 FAST.Farm InflowWind 参数字典。

        返回的字典可直接传给 FASTInputFile 写入 InflowWind.dat。

        Parameters
        ----------
        farm_inputs_dir : str
            FAST.Farm FarmInputs/ 目录路径。用于解析 wind_file 相对路径。
        """
        result: dict = {
            "WindType": int(self.wind_type),
            "HWindSpeed": self.speed,
            "RefHt": self.reference_height,
            "PLExp": self.shear_exponent,
        }

        if self.wind_type == WindType.STEADY:
            pass  # 使用上面的默认值

        elif self.wind_type == WindType.UNIFORM:
            if self.wind_file:
                result["Filename_Uni"] = f'"{self.wind_file}"'
            result["RefHt_Uni"] = self.reference_height

        elif self.wind_type == WindType.TURBSIM_BTS:
            if self.wind_file:
                result["FileName_BTS"] = f'"{self.wind_file}"'

        elif self.wind_type == WindType.BLADED_BIN:
            if self.wind_file:
                result["FileNameRoot"] = f'"{self.wind_file}"'

        elif self.wind_type == WindType.HAWC:
            if self.wind_file:
                result["FileName_u"] = f'"{self.wind_file}"'
            result["RefHt_Hawc"] = self.reference_height
            result["URef"] = self.speed
            result["PLExp_Hawc"] = self.shear_exponent

        elif self.wind_type in (WindType.BLADED_NATIVE, WindType.USER):
            if self.wind_file:
                result["FileNameRoot"] = f'"{self.wind_file}"'

        return result

    @classmethod
    def from_dict(cls, d: dict) -> "WindConfig":
        """从字典构建 WindConfig（向后兼容旧 config dict）。"""
        wind_type = d.get("wind_type", WindType.STEADY)
        if isinstance(wind_type, int):
            wind_type = WindType(wind_type)

        segments = None
        if "segments" in d and d["segments"]:
            segments = [
                WindSegment(**s) if isinstance(s, dict) else s
                for s in d["segments"]
            ]

        return cls(
            wind_type=wind_type,
            speed=d.get("speed", d.get("wind_speed", 8.0)),
            direction=d.get("direction", d.get("wind_direction", 270.0)),
            turbulence_intensity=d.get("turbulence_intensity", 0.1),
            shear_exponent=d.get("shear_exponent", 0.2),
            reference_height=d.get("reference_height", 90.0),
            wind_file=d.get("wind_file"),
            wind_time_series=d.get("wind_time_series"),
            segments=segments,
        )


# =========================================================================
# ControlInput — 统一控制输入
# =========================================================================

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


# =========================================================================
# SimulationOutput — 统一仿真输出
# =========================================================================

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
                      "generator_torque_nm", "thrust_n"):
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
