"""
closedloop — Data-driven closed-loop wind-farm control for the WFCRL / FAST.Farm project.

Three schemes share one skeleton (Phase 0). Each scheme swaps only its
observer + controller (+ dynamic model):

    Scheme A : FLORIS re-calibration + steady re-optimization
    Scheme B : FLORIDyn + EnKF + MPC          (recommended main scheme)
    Scheme C : grey-box / digital-twin augmentation

Integration surface with FAST.Farm (current, ZeroMQ architecture):
    controller -> Cmd -> ZmqFarmBridge -> FastFarmZmqEngine(FarmZmqClient)
               -> ZeroMQ -> 集成 DLL 内 farm_control_runtime -> ROSCO
    FAST.Farm  -> FarmStateFrame(~3s) -> client 合并 -> TurbineMeas

Legacy file bridge (controls.txt / measurements_T*.txt, DISCON_bridge.f90)
is retained only for historical reference and is non-functional with the
current FCR_FARM_CONTROL DLL — see docs/farm_control/PROJECT_REVIEW.md.
"""

from .types import TurbineMeas, Cmd, FlowEstimate, TurbineFlow
from .control_mode import ControlMode, ControlModeSpec
from .zmq_farm import (ZmqFarmBridge, FastFarmZmqPlant, ZmqPlant,
                       launch_farm, build_farm_case)
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
    MockPlant,
    RunResult,
    ZmqPlant,
)
from . import wind_schedule

__all__ = [
    "TurbineMeas",
    "Cmd",
    "FlowEstimate",
    "TurbineFlow",
    "ControlMode",
    "ControlModeSpec",
    "ZmqFarmBridge",
    "ZmqPlant",
    "FastFarmZmqPlant",
    "launch_farm",
    "build_farm_case",
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
    "MockPlant",
    "RunResult",
    "wind_schedule",
]

__version__ = "0.1.0"