"""
WFCRL RL 环境包
==============
提供 Gymnasium (single-agent) 和 PettingZoo (multi-agent) 两种风场控制 RL 环境。

子模块
------
- base.py         : BaseWindFarmEnv — 公共基类（奖励计算、约束检查）
- centralized.py  : WindFarmEnv — Gymnasium 单智能体环境
- multiagent.py   : MAWindFarmEnv — PettingZoo AEC 多智能体环境
- wrappers.py     : LogWrapper, AECLogWrapper, RandomSimulator
- rewards.py      : RewardShaper, DoNothingReward, ReferencePercentage, StepPercentage
"""

from wfcrl.envs.centralized import WindFarmEnv
from wfcrl.envs.multiagent import MAWindFarmEnv
from wfcrl.envs.wrappers import (
    LogWrapper,
    AECLogWrapper,
    RandomSimulator,
)
from wfcrl.envs.rewards import (
    RewardShaper,
    DoNothingReward,
    ReferencePercentage,
    StepPercentage,
)

__all__ = [
    "WindFarmEnv",
    "MAWindFarmEnv",
    "LogWrapper",
    "AECLogWrapper",
    "RandomSimulator",
    "RewardShaper",
    "DoNothingReward",
    "ReferencePercentage",
    "StepPercentage",
]
