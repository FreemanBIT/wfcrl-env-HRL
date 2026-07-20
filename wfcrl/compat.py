"""Backward compatibility layer for WFCRL refactoring.

This module re-exports all public types from their new locations so that
existing code using `from wfcrl.config import ControlInput` etc. continues
to work without changes.

Deprecated: New code should import directly from the new subpackages:
    from wfcrl.config.control import ControlInput
    from wfcrl.engine import ContinuousFastFarmInterface
"""

from __future__ import annotations

# ---- Legacy compatibility shims ----
from wfcrl.compat_data import (
    FarmCase, FastFarmCase, FlorisCase,
    DefaultControl, FarmRowFastfarm, FarmRowFloris,
)
from wfcrl.config.types import WindType, WindConfig, WindSegment
from wfcrl.config.control import ControlInput
from wfcrl.config.output import SimulationOutput
from wfcrl.config.simulator import SimulationConfig, FastFarmConfig, FlorisConfig
from wfcrl.config.layout import FarmLayout, LayoutRegistry

# ---- Simulator interfaces ----
from wfcrl.engine.base import SimulatorInterface, FastFarmAborted
from wfcrl.engine.angle_utils import (
    nacyaw_face_wind,
    nacyaw_from_misalignment,
    misalignment_from_nacyaw,
)
from wfcrl.engine.state import SimulatorState
from wfcrl.engine.constraints import ActuatorConstraints
from wfcrl.engine.fastfarm_step import FastFarmInterface
from wfcrl.engine.fastfarm_continuous import ContinuousFastFarmInterface
from wfcrl.engine.floris import FlorisInterface

# ---- Env types ----
try:
    from wfcrl.envs.centralized import WindFarmEnv
    from wfcrl.envs.multiagent import MAWindFarmEnv
    from wfcrl.envs.wrappers import LogWrapper, AECLogWrapper
    from wfcrl.envs.rewards import RewardShaper, DoNothingReward
except ImportError:
    pass  # gymnasium/pettingzoo may not be installed

# ---- MDP ----
try:
    from wfcrl.mdp.mdp import WindFarmMDP
except ImportError:
    pass

__all__ = [
    # Config
    "WindType", "WindConfig", "WindSegment",
    "ControlInput", "SimulationOutput",
    "SimulationConfig", "FastFarmConfig", "FlorisConfig",
    "FarmLayout", "LayoutRegistry",
    # Legacy shims
    "FarmCase", "FastFarmCase", "FlorisCase", "DefaultControl",
    # Simulator
    "SimulatorInterface",
    "FastFarmInterface", "ContinuousFastFarmInterface", "FlorisInterface",
    "SimulatorState", "FastFarmAborted",
    "ActuatorConstraints",
    "nacyaw_face_wind", "nacyaw_from_misalignment", "misalignment_from_nacyaw",
    # Env (optional)
    "WindFarmEnv", "MAWindFarmEnv",
    "LogWrapper", "AECLogWrapper",
    "RewardShaper", "DoNothingReward",
    # MDP
    "WindFarmMDP",
]
