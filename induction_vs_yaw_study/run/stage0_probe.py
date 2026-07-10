"""
stage0_probe.py — 接口探针与可行性自检 (Stage 0)
================================================
在写/跑任何实验前，确认：
  1. wfcrl 的 ControlInput / 接口签名与本研究假设一致；
  2. ControlInput 原生支持 5-mode（mode 0 偏航、mode 1 功率目标），
     即 **无需修改控制接口** 即可同时做偏航与降额控制；
  3. （可选）FLORIS 能 import 且 nrel_5MW 可解析；
  4. （可选）FAST.Farm 可执行文件存在。

用法：
    python -m induction_vs_yaw_study.run.stage0_probe
结果打印到终端，并写 docs/PROBE_RESULTS.md。
"""

from __future__ import annotations

import inspect
import os
import sys
from pathlib import Path


def _section(title: str) -> str:
    return f"\n{'='*70}\n{title}\n{'='*70}"


def main():
    lines = []

    def log(s=""):
        print(s)
        lines.append(s)

    log(_section("Stage 0 Probe — induction_vs_yaw_study"))

    # ---- 1. ControlInput 签名与 5-mode ----
    log(_section("1. wfcrl.config.ControlInput"))
    try:
        from wfcrl.config import ControlInput, WindConfig, WindType, SimulationOutput
        fields = list(getattr(ControlInput, "__dataclass_fields__", {}).keys())
        log(f"ControlInput fields: {fields}")
        factories = [m for m in dir(ControlInput) if m.startswith("mode")]
        log(f"ControlInput mode factories: {factories}")
        has_power = "power" in fields
        has_minpitch = "min_pitch" in fields
        log(f"  -> supports absolute power target: {has_power}")
        log(f"  -> supports min_pitch constraint : {has_minpitch}")
        if has_power and has_minpitch and "mode1_power" in factories:
            log("  VERDICT: 控制接口已原生支持功率目标 + 偏航 → 无需修改接口。")
        else:
            log("  VERDICT: 缺少功率通道，需检查（本研究假设其存在）。")
    except Exception as e:  # noqa
        log(f"FAILED to import wfcrl.config: {e}")

    # ---- 2. 接口签名 ----
    log(_section("2. wfcrl.interface 接口"))
    try:
        from wfcrl.interface import (
            FlorisInterface, FastFarmInterface, ContinuousFastFarmInterface,
        )
        for cls in (FlorisInterface, ContinuousFastFarmInterface):
            methods = [m for m in ("setup", "reset", "step", "start",
                                   "wait_step", "stop", "close")
                       if hasattr(cls, m)]
            log(f"{cls.__name__}: {methods}")
    except Exception as e:  # noqa
        log(f"FAILED to import wfcrl.interface: {e}")

    # ---- 3. simul_config ----
    log(_section("3. wfcrl.simul_config"))
    try:
        from wfcrl.simul_config import FlorisConfig, FastFarmConfig
        log(f"FlorisConfig fields: {list(FlorisConfig.__dataclass_fields__.keys())}")
        log(f"FastFarmConfig fields: {list(FastFarmConfig.__dataclass_fields__.keys())}")
    except Exception as e:  # noqa
        log(f"FAILED: {e}")

    # ---- 4. FLORIS 可用性 ----
    log(_section("4. FLORIS 可用性"))
    try:
        import floris
        log(f"floris version: {floris.__version__}")
        from floris import FlorisModel
        try:
            fm = FlorisModel("defaults")
            fm.set(turbine_type=["nrel_5MW"], layout_x=[0, 504, 1008],
                   layout_y=[0, 0, 0], wind_speeds=[8.0],
                   wind_directions=[270.0], turbulence_intensities=[0.06])
            fm.run()
            p = fm.get_turbine_powers().flatten() / 1e6
            log(f"  nrel_5MW 3T greedy powers (MW): {p}")
            log("  FLORIS OK.")
        except Exception as e:  # noqa
            log(f"  FLORIS model build failed: {e}")
    except Exception as e:  # noqa
        log(f"  FLORIS not importable: {e}")

    # ---- 5. FAST.Farm exe ----
    log(_section("5. FAST.Farm 可执行文件"))
    exe = os.environ.get("FAST_FARM_EXE")
    if not exe:
        root = Path(__file__).resolve().parents[2]
        exe = str(root / "wfcrl" / "simulators" / "fastfarm" / "bin" /
                  "FAST.Farm_x64_OMP.exe")
    log(f"FAST_FARM_EXE: {exe}")
    log(f"  exists: {os.path.exists(exe)}")

    # ---- 6. 入流 .bts ----
    log(_section("6. 入流 .bts 模板 (按 风速×TI 匹配)"))
    try:
        from induction_vs_yaw_study.cases.three_nrel5mw import (
            AVAILABLE_BTS, inflow_bts_for_case, bts_name, _FF_FARMINPUTS,
        )
        from induction_vs_yaw_study.cases.three_nrel5mw import (
            BTS_SPEEDS, BTS_TIS,
        )
        log(f"FarmInputs dir: {_FF_FARMINPUTS}")
        missing = []
        for s in BTS_SPEEDS:
            for ti in BTS_TIS:
                name = bts_name(s, ti)
                exists = (_FF_FARMINPUTS / name).exists()
                log(f"  U={s} TI={ti}: {name}  {'OK' if exists else 'MISSING'}")
                if not exists:
                    missing.append(name)
        if missing:
            log(f"  WARNING: {len(missing)} .bts 缺失，需 TurbSim 生成: {missing}")
        else:
            log("  全部 9 个 .bts 就位 → FAST.Farm 可在全网格的真实 TI 下回放。")
    except Exception as e:  # noqa
        log(f"FAILED: {e}")

    # ---- 写报告 ----
    docs = Path(__file__).resolve().parents[1] / "docs"
    docs.mkdir(parents=True, exist_ok=True)
    with open(docs / "PROBE_RESULTS.md", "w", encoding="utf-8") as f:
        f.write("# Stage 0 Probe Results\n\n```\n")
        f.write("\n".join(lines))
        f.write("\n```\n")
    log(_section(f"Report written to {docs / 'PROBE_RESULTS.md'}"))


if __name__ == "__main__":
    main()
