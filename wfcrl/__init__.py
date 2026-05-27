"""
WFCRL — Wind Farm Control Reinforcement Learning

接口和配置：
- config.py        : WindConfig, ControlInput, SimulationOutput, WindType
- simul_config.py  : SimulationConfig, FastFarmConfig, FlorisConfig
- interface.py     : FastFarmInterface, FlorisInterface, SimulatorInterface

RL 环境（可选依赖：gymnasium, pettingzoo）：
- environments/    : FarmCase, data_cases, registration
- mdp.py           : WindFarmMDP
- multiagent_env.py: MAWindFarmEnv
- simple_env.py    : WindFarmEnv

工具：
- simul_utils.py   : FAST.Farm/FLORIS 输入文件生成
"""

from wfcrl.config import (
    ControlInput,
    SimulationOutput,
    WindConfig,
    WindSegment,
    WindType,
)
from wfcrl.simul_config import (
    FastFarmConfig,
    FlorisConfig,
    SimulationConfig,
)
from wfcrl.interface import (
    FastFarmInterface,
    FlorisInterface,
    SimulatorInterface,
)

__all__ = [
    # Config
    "WindType",
    "WindConfig",
    "WindSegment",
    "ControlInput",
    "SimulationOutput",
    # Simulation config
    "SimulationConfig",
    "FastFarmConfig",
    "FlorisConfig",
    # Interface
    "SimulatorInterface",
    "FastFarmInterface",
    "FlorisInterface",
]
