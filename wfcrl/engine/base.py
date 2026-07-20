"""
仿真器统一抽象基类
==================
定义 SimulatorInterface（ABC）和 FastFarmAborted 异常。

所有仿真器实现（FAST.Farm / FLORIS）继承此 ABC。

用法
----
from wfcrl.engine import SimulatorInterface

class MySim(SimulatorInterface):
    def setup(self) -> None: ...
    def reset(self, wind) -> None: ...
    def step(self, controls) -> SimulationOutput: ...
    def close(self) -> None: ...
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from typing import List, Optional, Sequence

import numpy as np

from wfcrl.config.control import ControlInput
from wfcrl.config.output import SimulationOutput
from wfcrl.config.types import WindConfig


# =========================================================================
# FastFarmAborted — FAST.Farm 进程意外退出异常
# =========================================================================

class FastFarmAborted(RuntimeError):
    """FAST.Farm 进程在仿真过程中意外退出/崩溃/挂起时抛出。

    上层（回放驱动）捕获它后可立即：(1) 停止本工况，(2) 保存已推进出的部分结果，
    (3) 快速切换到下一个工况，而不必逐步空等超时。
    """
    pass


# =========================================================================
# SimulatorInterface — 统一接口抽象基类
# =========================================================================

class SimulatorInterface(ABC):
    """
    所有仿真器接口的抽象基类 — 零 RL 依赖。

    子类必须实现：
    - setup()         : 生成仿真输入文件
    - reset(wind)     : 根据风况初始化/重置仿真器
    - step(controls)  : 执行一步仿真，返回 SimulationOutput
    - close()         : 清理资源

    可选覆盖：
    - run()           : 批量运行多步（默认用 step() 循环）
    """

    n_turbines: int
    config: object  # SimulationConfig 子类

    @abstractmethod
    def setup(self) -> None:
        """生成仿真输入文件，准备仿真环境。"""

    @abstractmethod
    def reset(self, wind: WindConfig) -> None:
        """
        根据风况初始化/重置仿真器。

        在 setup() 之后调用，或每次改变风况时调用。

        Parameters
        ----------
        wind : WindConfig
            风况配置（风速/风向/风类型等）。
        """

    @abstractmethod
    def step(self, controls: ControlInput) -> SimulationOutput:
        """
        执行一步仿真。

        Parameters
        ----------
        controls : ControlInput
            当前步的控制输入（偏航/变桨/转矩/功率目标）。

        Returns
        -------
        SimulationOutput
            当前步的输出（包含所有通道的时序数据）。
        """

    def run(self, controls_list: Sequence[ControlInput]) -> SimulationOutput:
        """
        批量运行多步仿真（step() 的循环语法糖）。

        Parameters
        ----------
        controls_list : Sequence[ControlInput]
            每一步的控制输入序列。

        Returns
        -------
        SimulationOutput
            汇总所有步的输出。
        """
        outputs: List[SimulationOutput] = []
        for controls in controls_list:
            outputs.append(self.step(controls))
        return self._merge_outputs(outputs)

    @abstractmethod
    def close(self) -> None:
        """清理资源（终止子进程、释放文件句柄等）。"""

    # ---- 内部工具 ----

    @staticmethod
    def _merge_outputs(outputs: List[SimulationOutput]) -> SimulationOutput:
        """合并多个单步输出为一个批量输出。

        Parameters
        ----------
        outputs : List[SimulationOutput]
            单步输出列表，至少包含一个元素。

        Returns
        -------
        SimulationOutput
            合并后的输出，每个字段沿时间轴拼接。
        """
        if not outputs:
            raise ValueError("Empty outputs list")

        def _concat(attr: str) -> Optional[np.ndarray]:
            vals = [getattr(o, attr) for o in outputs if getattr(o, attr) is not None]
            return np.concatenate(vals, axis=0) if vals else None

        return SimulationOutput(
            time=_concat("time"),
            power_mw=_concat("power_mw"),
            wind_speed=_concat("wind_speed"),
            wind_direction=_concat("wind_direction"),
            yaw_deg=_concat("yaw_deg"),
            pitch_deg=_concat("pitch_deg"),
            torque_nm=_concat("torque_nm"),
            rotor_speed_rpm=_concat("rotor_speed_rpm"),
            generator_torque_nm=_concat("generator_torque_nm"),
            thrust_n=_concat("thrust_n"),
            blade_loads=_concat("blade_loads"),
            metadata=outputs[0].metadata if outputs else {},
        )
