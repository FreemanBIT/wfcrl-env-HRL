"""
closedloop — Data-driven closed-loop wind-farm control for the WFCRL / FAST.Farm project.

Three schemes share one skeleton (Phase 0). Each scheme swaps only its
observer + controller (+ dynamic model):

    Scheme A : FLORIS re-calibration + steady re-optimization
    Scheme B : FLORIDyn + EnKF + MPC          (recommended main scheme)
    Scheme C : grey-box / digital-twin augmentation

Integration surface with FAST.Farm is the file bridge implemented in
``wfcrl/simulators/fastfarm/src/DISCON_bridge.f90``:

    controller  ->  controls.txt            (yaw / pitch / torque / power per turbine)
    FAST.Farm   ->  measurements_T<i>.txt   (power, wind_x, nacyaw, loads, ...)

See ``开发方案.md`` (development plan) for the module map and equation references.
"""

from .types import TurbineMeas, Cmd, FlowEstimate, TurbineFlow
from .control_mode import ControlMode, ControlModeSpec
from .bridge import FarmBridge
from .sensing import Sensing
from .induction import (
    induction_to_power,
    power_to_induction,
    cp_from_induction,
    ct_from_induction,
)
from .surrogate import SurrogateModel
from .base_controller import BaseController, CommandArbiter
from .evaluate import Evaluator, Trajectory
from .case_config import FarmCase, default_case
from .runner import (
    ClosedLoopRunner,
    Plant,
    FastFarmPlant,
    MockPlant,
    RunResult,
)
from . import wind_schedule

__all__ = [
    "TurbineMeas",
    "Cmd",
    "FlowEstimate",
    "TurbineFlow",
    "ControlMode",
    "ControlModeSpec",
    "FarmBridge",
    "Sensing",
    "induction_to_power",
    "power_to_induction",
    "cp_from_induction",
    "ct_from_induction",
    "SurrogateModel",
    "BaseController",
    "CommandArbiter",
    "Evaluator",
    "Trajectory",
    "FarmCase",
    "default_case",
    "ClosedLoopRunner",
    "Plant",
    "FastFarmPlant",
    "MockPlant",
    "RunResult",
    "wind_schedule",
]

__version__ = "0.1.0"
