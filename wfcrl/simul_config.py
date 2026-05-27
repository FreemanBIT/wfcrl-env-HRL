"""
仿真器配置
==========
定义 SimulationConfig（通用）、FastFarmConfig、FlorisConfig。
同一套 SimulationConfig 可以分别生成 FAST.Farm 和 FLORIS 的专有配置，
自动映射通用参数，丢弃不支持的参数。
"""

from __future__ import annotations

import os
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Optional

import numpy as np

from wfcrl.config import ControlInput, WindConfig, WindType


# FAST.Farm 可执行文件路径：优先使用环境变量
def _get_fastfarm_exe() -> str:
    _DEFAULT = str(
        Path(__file__).resolve().parent
        / "simulators/fastfarm/bin/FAST.Farm_x64_OMP.exe"
    )
    return os.environ.get("FAST_FARM_EXE", _DEFAULT)


# =========================================================================
# SimulationConfig — 仿真器无关的通用配置
# =========================================================================

@dataclass
class SimulationConfig:
    """
    仿真器无关的通用仿真配置。

    同一套配置可以通过 `.create_fastfarm()` / `.create_floris()` 生成
    模拟器专用配置，自动处理参数映射。

    Attributes
    ----------
    case_name : str
        案例名称。
    num_turbines : int
        风机数量。
    xcoords : List[float]
        风机 X 坐标 (m)。
    ycoords : List[float]
        风机 Y 坐标 (m)。
    dt : float
        控制步长 (s)。FAST.Farm 用 3s，FLORIS 用 60s。
    max_iter : int
        最大仿真步数。
    t_init : float
        初始化时间 (s)，仿真前运行以消除瞬态。
    turbine_type : str
        风机型号 (如 "nrel_5MW", "goldwind_8_5mw")。
    wind : WindConfig
        风况配置。
    output_channels : List[str]
        需要提取的输出通道名称列表。
        默认包含：power, wind_speed, wind_direction, yaw, pitch, torque,
                 rotor_speed, blade_loads
    output_dir : Optional[str]
        输出目录。None 则自动生成。
    """
    case_name: str
    num_turbines: int
    xcoords: List[float]
    ycoords: List[float]
    dt: float
    max_iter: int
    wind: WindConfig = field(default_factory=WindConfig)
    t_init: float = 0.0
    turbine_type: str = "nrel_5MW"
    output_channels: List[str] = field(default_factory=lambda: [
        "power", "wind_speed", "wind_direction",
        "yaw", "pitch", "torque", "rotor_speed", "blade_loads",
    ])
    output_dir: Optional[str] = None

    def __post_init__(self):
        if len(self.xcoords) != len(self.ycoords):
            raise ValueError("xcoords and ycoords must have same length")
        if len(self.xcoords) != self.num_turbines:
            self.num_turbines = len(self.xcoords)

    # ---- 工厂方法 ----

    def create_fastfarm(self, output_dir: Optional[str] = None, **overrides) -> "FastFarmConfig":
        """
        创建 FAST.Farm 专有配置。

        自动映射通用参数，丢弃 FLORIS 专有参数。
        通过 **overrides 可覆盖任何参数。
        """
        ff = FastFarmConfig(
            case_name=self.case_name,
            num_turbines=self.num_turbines,
            xcoords=list(self.xcoords),
            ycoords=list(self.ycoords),
            dt=self.dt,
            max_iter=self.max_iter,
            wind=self.wind,
            t_init=self.t_init,
            turbine_type=self.turbine_type,
            output_channels=list(self.output_channels),
            output_dir=output_dir or self._auto_output_dir("fastfarm"),
        )
        # 应用覆盖
        for k, v in overrides.items():
            if hasattr(ff, k):
                setattr(ff, k, v)
        return ff

    def create_floris(self, output_dir: Optional[str] = None, **overrides) -> "FlorisConfig":
        """
        创建 FLORIS 专有配置。

        自动映射通用参数，丢弃 FAST.Farm 专有参数。
        注意：FLORIS 不支持 WindType ≠ 1 的原始风类型，会自动降级。
        """
        # FLORIS 的 dt 通常比 FAST.Farm 大
        floris_dt = self.dt if self.dt >= 30 else 60.0

        fl = FlorisConfig(
            case_name=self.case_name,
            num_turbines=self.num_turbines,
            xcoords=list(self.xcoords),
            ycoords=list(self.ycoords),
            dt=floris_dt,
            max_iter=self.max_iter,
            wind=self.wind,
            t_init=self.t_init,
            turbine_type=self.turbine_type,
            output_channels=[
                ch for ch in self.output_channels
                if ch not in ("torque",)  # FLORIS 无转矩输出
            ],
            output_dir=output_dir or self._auto_output_dir("floris"),
        )
        for k, v in overrides.items():
            if hasattr(fl, k):
                setattr(fl, k, v)
        return fl

    def _auto_output_dir(self, simulator: str) -> str:
        base = Path(__file__).resolve().parent.parent
        name = f"{simulator}__{self.t_init + self.max_iter * self.dt:.0f}s"
        name += f"__{self.num_turbines}T_{time.time():.0f}"
        return str(base / "__simul__" / simulator / name)

    # ---- 从旧版 dict 构建（向后兼容） ----

    @classmethod
    def from_dict(cls, d: dict, case_name: str = "custom") -> "SimulationConfig":
        """从旧版 config dict 构建 SimulationConfig。"""
        wind = WindConfig.from_dict(d)
        return cls(
            case_name=d.get("case_name", case_name),
            num_turbines=d.get("num_turbines", len(d.get("xcoords", []))),
            xcoords=d.get("xcoords", []),
            ycoords=d.get("ycoords", []),
            dt=d.get("dt", 3.0),
            max_iter=d.get("max_iter", 100),
            wind=wind,
            t_init=d.get("t_init", 0.0),
            turbine_type=d.get("turbine_type", "nrel_5MW"),
        )

    def to_legacy_dict(self) -> dict:
        """转为旧版 create_ff_case / create_floris_case 兼容的 dict。"""
        d: dict = {
            "num_turbines": self.num_turbines,
            "xcoords": self.xcoords,
            "ycoords": self.ycoords,
            "dt": self.dt,
            "max_iter": self.max_iter,
            "speed": self.wind.speed,
            "direction": self.wind.direction,
            "wind_time_series": None,
        }
        if self.wind.wind_type == WindType.TURBSIM_BTS and self.wind.wind_file:
            d["wind_time_series"] = self.wind.wind_file
        if self.turbine_type != "nrel_5MW":
            d["turbine_type"] = self.turbine_type
        return d


# =========================================================================
# FastFarmConfig — FAST.Farm 专有配置
# =========================================================================

@dataclass
class FastFarmConfig(SimulationConfig):
    """
    FAST.Farm 专有仿真配置。

    在 SimulationConfig 基础上增加 FAST.Farm 特有参数。

    Attributes
    ----------
    fastfarm_exe : Optional[str]
        FAST.Farm 可执行文件路径。None 使用默认路径。
    template_dir : Optional[str]
        模板目录。None 使用默认模板。
    dt_low : Optional[float]
        FAST.Farm 底层时间步长。None 使用模板默认值。
    fstf_overrides : dict
        .fstf 文件中需要覆盖的额外参数。
    wind_time_series_file : Optional[str]
        风时序文件路径 (TurbSim .bts 等)，WindType≠1 时使用。
        会自动从 WindConfig.wind_file 设置。
    """
    fastfarm_exe: Optional[str] = field(default_factory=_get_fastfarm_exe)
    template_dir: Optional[str] = None
    dt_low: Optional[float] = None
    fstf_overrides: dict = field(default_factory=dict)
    wind_time_series_file: Optional[str] = None

    def __post_init__(self):
        super().__post_init__()
        # 从 WindConfig 同步 wind_file
        if self.wind.wind_file and not self.wind_time_series_file:
            self.wind_time_series_file = self.wind.wind_file


# =========================================================================
# FlorisConfig — FLORIS 专有配置
# =========================================================================

@dataclass
class FlorisConfig(SimulationConfig):
    """
    FLORIS 专有仿真配置。

    Attributes
    ----------
    turbine_library_path : Optional[str]
        风机库路径。None 使用默认路径。
    wake_model : str
        尾流模型，默认 "gauss_curl"。
    solver_grid_points : int
        求解器网格点，默认 3。
    enable_active_wake_mixing : bool
        是否启用主动尾流混合。
    enable_secondary_steering : bool
        是否启用二次转向。
    enable_yaw_added_recovery : bool
        是否启用偏航恢复。
    enable_transverse_velocities : bool
        是否启用横向速度。
    """
    turbine_library_path: Optional[str] = None
    wake_model: str = "gauss_curl"
    solver_grid_points: int = 3
    enable_active_wake_mixing: bool = False
    enable_secondary_steering: bool = True
    enable_yaw_added_recovery: bool = True
    enable_transverse_velocities: bool = True

    def __post_init__(self):
        super().__post_init__()
        if self.turbine_library_path is None:
            self.turbine_library_path = str(
                Path(__file__).resolve().parent
                / "simulators/floris/inputs/turbine_library/"
            )
