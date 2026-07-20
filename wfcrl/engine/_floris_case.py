"""FLORIS case file generation (internal).

Creates FLORIS input YAML from a configuration dictionary.
Migrated from wfcrl/simul_utils.py.
"""

from __future__ import annotations

from pathlib import Path
from typing import Dict

import yaml

LOCAL_DIR = Path(__file__).resolve().parent.parent  # wfcrl/simulator/ → wfcrl/

TEMPLATE_DIR = str(LOCAL_DIR / "simulators/{}/inputs/template/") + "/"
CASE_DIR = str(LOCAL_DIR / "simulators/{}/inputs/") + "/"


def create_floris_case(case: Dict, output_dir=None):
    template_dir = TEMPLATE_DIR.format("floris")
    output_dir = Path(CASE_DIR.format("floris") if output_dir is None else output_dir)
    with open(f"{template_dir}case.yaml", "r") as fp:
        config = yaml.safe_load(fp)
    config["farm"]["layout_x"] = case["xcoords"]
    config["farm"]["layout_y"] = case["ycoords"]
    if case.get("direction") is not None:
        config["flow_field"]["wind_directions"] = [case["direction"]]
    if case.get("speed") is not None:
        config["flow_field"]["wind_speeds"] = [case["speed"]]
    if case.get("turbine_type") is not None:
        config["farm"]["turbine_type"] = case["turbine_type"]
        turbine_lib = str(LOCAL_DIR / "simulators/floris/inputs/turbine_library/")
        config["farm"]["turbine_library_path"] = case.get("turbine_library_path", turbine_lib)
    elif case.get("turbine_library_path") is not None:
        config["farm"]["turbine_library_path"] = case["turbine_library_path"]
    output_dir.mkdir(parents=True, exist_ok=True)
    with open(output_dir / "case.yaml", "w") as fp:
        yaml.safe_dump(config, fp)
    return str(output_dir / "case.yaml")
