"""Scheme C: grey-box learning augmentation / digital twin.

Adds a data-driven residual on top of the physical surrogate (A/B) to correct
structural model error, with joint state-parameter-residual estimation and a
safety-projected policy option.
"""
from .residual_model import ResidualModel, GreyBoxSurrogate
from .joint_estimator import JointEstimator
from .controller_c import ControllerC

__all__ = [
    "ResidualModel",
    "GreyBoxSurrogate",
    "JointEstimator",
    "ControllerC",
]
