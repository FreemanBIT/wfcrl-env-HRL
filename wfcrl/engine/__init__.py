"""
WFCRL 仿真器包
=============
提供统一 SimulatorInterface 抽象基类和所有具体实现。

设计原则
--------
1. 零 RL 依赖：不 import gymnasium / pettingzoo / 任何 RL 库
2. MPC 等非 RL 算法可直接使用仿真器
3. 所有仿真器实现共享 SimulatorInterface ABC

子包结构
--------
- base.py            : SimulatorInterface ABC, FastFarmAborted
- state.py           : SimulatorState, ApplyResult
- constraints.py     : ActuatorConstraints
- angle_utils.py     : _wrap180, nacyaw_from_misalignment, misalignment_from_nacyaw
- _outb.py           : _parse_outb_file, _parse_all_outb (内部)
- _outlist.py        : _inject_outlist_channels (内部)
- fastfarm_step.py   : FastFarmInterface
- fastfarm_continuous.py : ContinuousFastFarmInterface
- floris.py          : FlorisInterface

用法
----
from wfcrl.engine import SimulatorInterface, SimulatorState
from wfcrl.engine.base import FastFarmAborted
from wfcrl.engine.state import ApplyResult
"""

from wfcrl.engine.base import (
    SimulatorInterface,
    FastFarmAborted,
)
from wfcrl.engine.state import (
    SimulatorState,
    ApplyResult,
)
from wfcrl.engine.constraints import (
    ActuatorConstraints,
)
from wfcrl.engine.angle_utils import (
    wrap180,
    nacyaw_face_wind,
    nacyaw_from_misalignment,
    misalignment_from_nacyaw,
)

# 具体仿真器实现
from wfcrl.engine.fastfarm_step import FastFarmInterface
from wfcrl.engine.fastfarm_continuous import ContinuousFastFarmInterface
from wfcrl.engine.floris import FlorisInterface

__all__ = [
    # Base
    "SimulatorInterface",
    "FastFarmAborted",
    # State
    "SimulatorState",
    "ApplyResult",
    # Constraints
    "ActuatorConstraints",
    # Angle utils
    "wrap180",
    "nacyaw_face_wind",
    "nacyaw_from_misalignment",
    "misalignment_from_nacyaw",
    # Concrete implementations
    "FastFarmInterface",
    "ContinuousFastFarmInterface",
    "FlorisInterface",
]
