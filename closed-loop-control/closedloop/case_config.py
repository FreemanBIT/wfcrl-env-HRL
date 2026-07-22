"""
M1 — Farm case configuration.

Fixes the 2x3 @ 4D NREL 5MW layout used by every scheme, and provides helpers to
(a) generate the per-turbine DISCON input files the bridge needs, and (b) emit a
FLORIS ``case.yaml`` with the matching layout.

Layout (westerly 270 deg -> x is streamwise; y is spanwise across rows):

    row 0:  T1(0,0)     T2(504,0)     T3(1008,0)
    row 1:  T4(0,504)   T5(504,504)   T6(1008,504)

D = 126 m (NREL 5MW), spacing 4D = 504 m in both directions.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional

from .induction import D_ROTOR


@dataclass
class FarmCase:
    """Farm layout + reference conditions.

    Can be constructed from a ``wfcrl.config.layout.FarmLayout`` via
    :meth:`from_wfcrl_layout`, or built directly for the 2x3@4D default.
    """
    name: str = "farm_2x3_4D"
    turbine_type: str = "nrel_5MW"
    D: float = D_ROTOR
    spacing_D: float = 4.0
    n_rows: int = 2
    n_cols: int = 3
    # reference conditions (baseline)
    wind_speed: float = 8.0
    wind_direction: float = 270.0
    turbulence_intensity: float = 0.06

    layout_x: list[float] = field(default_factory=list)
    layout_y: list[float] = field(default_factory=list)

    def __post_init__(self):
        if not self.layout_x:
            self._build_layout()

    def _build_layout(self) -> None:
        s = self.spacing_D * self.D
        xs, ys = [], []
        for r in range(self.n_rows):
            for c in range(self.n_cols):
                xs.append(c * s)
                ys.append(r * s)
        self.layout_x = xs
        self.layout_y = ys

    @classmethod
    def from_wfcrl_layout(cls, wfcrl_layout, **overrides) -> "FarmCase":
        """Create a FarmCase from a ``wfcrl.config.layout.FarmLayout``.

        The WFCRL layout provides ``name, num_turbines, xcoords, ycoords,
        turbine_type``. You can override any FarmCase field via ``**overrides``
        (e.g. ``wind_speed=10.0, n_rows=2``).

        Note: WFCRL's built-in ``"6T"`` layout has y coords centered at 0
        (``[-252, 252]``), while the development plan uses ``[0, 504]``. Both
        are 2x3@4D, just with a different y offset. Pass
        ``layout_y=[0,0,0,504,504,504]`` as an override if you need exact plan
        coordinates.
        """
        import wfcrl.config.layout as _wlay
        if isinstance(wfcrl_layout, str):
            reg = _wlay.LayoutRegistry.from_builtin()
            wfcrl_layout = reg.get(wfcrl_layout)
        if not isinstance(wfcrl_layout, _wlay.FarmLayout):
            raise TypeError(f"Expected FarmLayout, got {type(wfcrl_layout)}")
        kw = dict(
            name=wfcrl_layout.name,
            turbine_type=wfcrl_layout.turbine_type,
            D=D_ROTOR,
            spacing_D=4.0,
            layout_x=list(wfcrl_layout.xcoords),
            layout_y=list(wfcrl_layout.ycoords),
        )
        kw.update(overrides)
        return cls(**kw)

    @property
    def n_turbines(self) -> int:
        return self.n_rows * self.n_cols

    @property
    def upstream_ids(self) -> list[int]:
        """First column (streamwise-most upstream) turbine ids under westerly
        inflow: T1 and T4 for the 2x3 layout."""
        ids = []
        for r in range(self.n_rows):
            ids.append(r * self.n_cols + 1)  # 1-based first column of each row
        return ids

    # -- file generation -------------------------------------------------------
    def write_discon_inputs(self, run_dir: Path) -> list[Path]:
        """Write per-turbine DISCON input files with full ROSCO v2.9 parameters.

        The first line is the turbine ID (read by DISCON_bridge.f90). The
        remaining lines are ROSCO NREL 5MW controller parameters from the
        project template ``DISCON_ROSCO_TEMPLATE.IN``.
        """
        import shutil
        from pathlib import Path as _Path

        run_dir = _Path(run_dir)
        run_dir.mkdir(parents=True, exist_ok=True)

        # locate the ROSCO template shipped with the WFCRL project
        import closedloop.config as _cfg
        wfcrl_root = _cfg.CONFIG.paths.wfcrl_root or r"D:\HR_Project\wfcrl-env-HRL"
        template = _Path(wfcrl_root) / "wfcrl/simulators/fastfarm/servo_dll/DISCON_ROSCO_TEMPLATE.IN"
        if not template.exists():
            # fallback: search relative to this package
            alt = _Path(__file__).resolve().parents[3] / "wfcrl/simulators/fastfarm/servo_dll/DISCON_ROSCO_TEMPLATE.IN"
            if alt.exists():
                template = alt
        
        if not template.exists():
            raise FileNotFoundError(
                f"ROSCO template not found at {template}. Set CONFIG.paths.wfcrl_root "
                f"to the WFCRL project directory."
            )

        paths = []
        for tid in range(1, self.n_turbines + 1):
            p = run_dir / f"DISCON_T{tid}.IN"
            with open(template) as ft:
                lines = ft.readlines()
            # first line must be the turbine ID
            lines[0] = f"{tid}\n"
            p.write_text("".join(lines))
            paths.append(p)
        return paths

    def to_floris_yaml(self) -> str:
        """Emit a FLORIS case.yaml (GCH) with this layout via yaml.dump."""
        import yaml
        cfg = dict(
            name="GCH_2x3_4D",
            description=f"NREL 5MW 2x3 at 4D spacing (generated by closedloop.case_config)",
            floris_version="v4.6",
            logging=dict(console=dict(enable=True, level="WARNING"),
                         file=dict(enable=False, level="WARNING")),
            solver=dict(type="turbine_grid", turbine_grid_points=3),
            farm=dict(
                layout_x=list(self.layout_x),
                layout_y=list(self.layout_y),
                turbine_type=[self.turbine_type],
            ),
            flow_field=dict(
                air_density=1.225, reference_wind_height=-1,
                turbulence_intensities=[self.turbulence_intensity],
                wind_directions=[self.wind_direction],
                wind_shear=0.12, wind_speeds=[self.wind_speed], wind_veer=0.0,
            ),
            wake=dict(
                enable_active_wake_mixing=False,
                enable_secondary_steering=True,
                enable_yaw_added_recovery=True,
                enable_transverse_velocities=True,
                model_strings=dict(combination_model="sosfs", deflection_model="gauss",
                                   turbulence_model="crespo_hernandez", velocity_model="gauss"),
                wake_deflection_parameters=dict(
                    gauss=dict(ad=0.0, alpha=0.58, bd=0.0, beta=0.077, ka=0.38, kb=0.004)),
                wake_velocity_parameters=dict(
                    gauss=dict(alpha=0.58, beta=0.077, ka=0.38, kb=0.004)),
                wake_turbulence_parameters=dict(
                    crespo_hernandez=dict(initial=0.1, constant=0.5, ai=0.8, downstream=-0.32)),
            ),
        )
        return yaml.dump(cfg, default_flow_style=False, sort_keys=False)
    def write_floris_yaml(self, path: Path) -> Path:
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(self.to_floris_yaml())
        return path


def default_case() -> FarmCase:
    """The canonical 2x3 @ 4D NREL 5MW case used across all schemes/demos."""
    return FarmCase()
