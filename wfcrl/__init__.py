"""WFCRL — Wind Farm Control Reinforcement Learning.

Unified interface for wind farm simulation (FAST.Farm / FLORIS) and
RL environment creation (Gymnasium / PettingZoo).

Re-exported via wfcrl.compat for backward compatibility.
New code should import from subpackages directly:

    from wfcrl.config import WindConfig, ControlInput
    from wfcrl.engine import ContinuousFastFarmInterface
    from wfcrl.envs import WindFarmEnv
"""

from wfcrl.compat import *  # noqa: F401, F403
from wfcrl.compat import __all__  # noqa: F401
