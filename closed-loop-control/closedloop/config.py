"""
Central configuration for the closed-loop control package.

Collects the knobs a user is most likely to change in one place. Import
``CONFIG`` and override fields, or construct your own dataclasses per run. The
demo/runner accept explicit configs too; this is the convenient default.
"""
from __future__ import annotations

from dataclasses import dataclass, field


@dataclass
class PathsConfig:
    """Where FAST.Farm / FLORIS files live. Adjust for your machine."""
    # FLORIS case template — points to the pre-generated 2x3@4D case
    floris_case_yaml: str | None = "cases/farm_2x3_4D/floris_case.yaml"
    # FAST.Farm run directory (where controls.txt / measurements_T*.txt live)
    fastfarm_run_dir: str | None = None
    # Path to the WFCRL project root (for FAST.Farm template / binary discovery)
    wfcrl_root: str | None = r"D:\HR_Project\wfcrl-env-HRL"
    # Optional .fstf file if driving FAST.Farm through the WFCRL interface
    fstf_file: str | None = None
    # Optional FAST.Farm executable (WFCRL interface); None auto-discovers from wfcrl_root
    fastfarm_executable: str | None = None
    # Optional farm-level wind-direction file (scanning-lidar channel)
    direction_file: str | None = None


@dataclass
class TimingConfig:
    dt: float = 2.0              # control step (s); MUST equal FAST.Farm DT_low
    t_ctrl: float = 20.0         # controller re-plan interval (s)
    t_cal: float = 600.0         # Scheme A calibration interval (s)
    warmup_s: float = 60.0       # transient skipped before scoring
    duration_s: float = 600.0    # episode length (s)


@dataclass
class Config:
    paths: PathsConfig = field(default_factory=PathsConfig)
    timing: TimingConfig = field(default_factory=TimingConfig)
    seed: int = 0
    # default control mode (1=YAW 2=TORQUE 3=PITCH 4=YAW_TORQUE 5=YAW_PITCH)
    control_mode: int = 1


# a shared default instance callers can import and tweak
CONFIG = Config()
