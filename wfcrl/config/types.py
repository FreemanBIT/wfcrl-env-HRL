"""
风况类型与配置数据结构
====================
定义 WindType / WindSegment / WindConfig：

- WindType：FAST.Farm InflowWind.dat 的风类型枚举
- WindSegment：单段时域风况
- WindConfig：统一风况配置，可映射到 FAST.Farm / FLORIS
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import IntEnum
from typing import Dict, List, Optional, Union

import numpy as np


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
