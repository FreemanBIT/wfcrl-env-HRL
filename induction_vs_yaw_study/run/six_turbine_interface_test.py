"""six_turbine_interface_test.py — 2×3 六机风电场控制接口测试（稳态风 11 m/s，270°）

在 FAST.Farm 稳态风（周期性均匀 .bts，物理等效稳态入流）下验证场控接口：

  布局：LayoutRegistry "6T"（2×3）：
    T1(0,-252) SW | T2(504,-252) S中 | T3(1008,-252) SE
    T4(0,+252)  NW | T5(504,+252)  N中 | T6(1008,+252) NE
  风向 270°（西风，吹向 +X）；西北角=T4、东南角=T3（对角受控）。

两个场景：
  --control yaw       ：T3/T4 在 200 s 施加 yaw_delta=+10° 阶跃，其余/其余时间 0°；
  --control induction ：T3/T4 在 200 s 施加 induction_a=0.2 阶跃，其余/其余时间 1/3（贪婪）。

每个控制步（dt=3s）落盘全场状态帧时序（功率/机舱方位/seq 回读/盘面风），
用于判断接口是否真正生效（seq_applied 回读 + 物理量变化）。

用法：
    python -m induction_vs_yaw_study.run.six_turbine_interface_test --control yaw
    python -m induction_vs_yaw_study.run.six_turbine_interface_test --control induction
输出：results/_six_turbine_{control}/{ts}/interface_timeseries.csv + scenario.json
"""
from __future__ import annotations

import argparse
import json
import math as _math
import os as _os
import sys as _sys
import time as _time
from pathlib import Path

_STUDY_ROOT = Path(__file__).resolve().parents[1]
_os.environ.setdefault("WFCRL_FARMINPUTS_DIR", str(_STUDY_ROOT / "FarmInputs"))
_REPO = _STUDY_ROOT.parent
if str(_REPO) not in _sys.path:
    _sys.path.insert(0, str(_REPO))
_ccc = _os.path.abspath(_os.path.join(_STUDY_ROOT, "..", "closed-loop-control"))
if _ccc not in _sys.path:
    _sys.path.insert(0, _ccc)

import numpy as np

from wfcrl.config import WindConfig
from wfcrl.config.types import WindType
from wfcrl.config.layout import LayoutRegistry
from wfcrl.config.simulator import FastFarmConfig
from wfcrl.engine.fastfarm_zmq import FastFarmZmqEngine
from closedloop.types import Cmd
from closedloop.bridge import ZmqFarmBridge
from induction_vs_yaw_study.replay.fastfarm_zmq_replay import _send_cmds

# 场景参数
WIND_SPEED = 11.0
WIND_DIR = 270.0
UNIFORM_BTS = "coarse/uniform11_380s.bts"
DURATION = 380.0
DT = 3.0
STEP_AT = 200.0          # 阶跃时刻
STEP_DEG = 10.0          # yaw 阶跃（CW+）
STEP_A = 0.2             # induction 阶跃
CTRL_TURBINES = [3, 4]   # 东南角(T3) + 西北角(T4)
ALL_TURBINES = [1, 2, 3, 4, 5, 6]


def configure_verification_outputs(engine) -> None:
    """engine.setup() 后启用验证输出（就地修改物化的 FarmInputs）：

    1) 逐机 OpenFAST：OutFileFmt 2(二进制 .outb) -> 1(文本 .out)，
       ElastoDyn OutList 追加 NacYaw/GenPwr/RotSpeed/NacYawF（权威机组状态，
       用于判定偏航是否真正执行——不依赖 ZMQ 状态帧）。
    2) 农场 FAST.Farm：WrDisWind=True + NOutDisWindXY=1（轮毂高度 90 m XY 平面），
       产出 Case.Low.DisXY1.t<n>.vtk（扰动风场，含真实尾流结构）。
    """
    import glob
    fi = getattr(engine._ff, "_farm_base", None)
    if not fi:
        return

    # ---- 1) 逐机 .fst：文本输出 + 关键通道 ----
    # 仅用该 NREL 5MW 模型**有效**的输出通道（NacYawF/GenPwr/RotPwr 会 abort）。
    # NacYaw = 机舱偏航角（权威偏航执行判定）。
    ed_verify_channels = ["NacYaw"]
    for fst in glob.glob(_os.path.join(fi, "FFTest_WT*.fst")):
        txt = Path(fst).read_bytes().decode("ascii", errors="replace").replace("\r\n", "\n")
        lines = txt.split("\n")
        for i, l in enumerate(lines):
            toks = l.split()
            if len(toks) >= 2 and toks[1] == "OutFileFmt":
                lines[i] = l.replace(toks[0], "1", 1)   # 文本 .out
            elif len(toks) >= 2 and toks[1] == "EDFile":
                ed = toks[0].strip('"')
                ed_path = Path(fi) / ed
                if ed_path.exists():
                    from induction_vs_yaw_study.run.add_load_channels import _append_outlist
                    _append_outlist(str(ed_path), ed_verify_channels)
        Path(fst).write_bytes(("\n".join(lines) + "\n").encode("ascii", errors="replace"))

    # ---- 2) Case.fstf：VTK 扰动风 XY 平面（轮毂高度；仅平面，避免全 3D 大文件）----
    fstf = Path(fi) / "Case.fstf"
    if fstf.exists():
        txt = fstf.read_bytes().decode("ascii", errors="replace").replace("\r\n", "\n")
        lines = txt.split("\n")
        for i, l in enumerate(lines):
            toks = l.split()
            if len(toks) >= 2 and toks[1] == "NOutDisWindXY":
                lines[i] = l.replace(toks[0], "1", 1)
        fstf.write_bytes(("\n".join(lines) + "\n").encode("ascii", errors="replace"))


def build_config(out_dir: Path) -> FastFarmConfig:
    reg = LayoutRegistry.from_builtin()
    layout = reg.get("6T")
    wind = WindConfig(speed=WIND_SPEED, direction=WIND_DIR,
                      wind_type=WindType.TURBSIM_BTS, wind_file=UNIFORM_BTS)
    cfg = FastFarmConfig(
        case_name=layout.name, num_turbines=layout.num_turbines,
        xcoords=layout.xcoords, ycoords=layout.ycoords,
        dt=DT, max_iter=int(DURATION / DT), wind=wind, output_dir=str(out_dir),
    )
    return cfg


def baseline_cmds(control: str) -> dict:
    """阶跃前的基线命令：yaw=0° / induction=1/3（贪婪），全部风机。"""
    cmds = {}
    for tid in ALL_TURBINES:
        if control == "yaw":
            cmds[tid] = Cmd(turbine_id=tid, yaw_delta_deg=0.0, induction_a=None)
        else:
            cmds[tid] = Cmd(turbine_id=tid, yaw_delta_deg=None, induction_a=1.0 / 3.0)
    return cmds


def step_cmds(control: str) -> dict:
    """阶跃命令：仅受控机（T3/T4）施加；其余仍基线。"""
    cmds = baseline_cmds(control)
    for tid in CTRL_TURBINES:
        if control == "yaw":
            cmds[tid] = Cmd(turbine_id=tid, yaw_delta_deg=STEP_DEG, induction_a=None)
        else:
            cmds[tid] = Cmd(turbine_id=tid, yaw_delta_deg=None, induction_a=STEP_A)
    return cmds


def extract_row(st, commanded) -> dict:
    """从状态帧提取一行时序（全场 + 各机）。"""
    tb = (st or {}).get("turbines", [])
    row = {"sim_time_s": float((st or {}).get("sim_time_s", float("nan"))),
           "low_step": int((st or {}).get("low_step", -1))}
    for tid in ALL_TURBINES:
        t = tb[tid - 1] if len(tb) >= tid else {}
        p = t.get("gen_power_w")
        if isinstance(p, dict):
            p_inst, p_mean = p.get("instant", float("nan")), p.get("mean", float("nan"))
        else:
            p_inst = p_mean = float(p if p is not None else float("nan"))
        row[f"T{tid}_power_kw"] = p_inst / 1e3 if p_inst == p_inst else float("nan")
        row[f"T{tid}_power_mean_kw"] = p_mean / 1e3 if p_mean == p_mean else float("nan")
        row[f"T{tid}_heading_rad"] = float(t.get("nacelle_heading_rad", float("nan")))
        row[f"T{tid}_yaw_seq"] = int(t.get("yaw_seq_applied", 0) or 0)
        row[f"T{tid}_ind_seq"] = int(t.get("induction_seq_applied", 0) or 0)
        row[f"T{tid}_disk_ambient"] = float(t.get("disk_ambient_wind_mps", float("nan")))
        row[f"T{tid}_disk_disturbed"] = float(t.get("disk_disturbed_wind_mps", float("nan")))
    # 命令侧（回读用）
    for tid in ALL_TURBINES:
        c = commanded.get(tid)
        row[f"T{tid}_cmd_yaw_deg"] = (float(c.yaw_delta_deg) if c and c.yaw_delta_deg is not None else 0.0)
        row[f"T{tid}_cmd_a"] = (float(c.induction_a) if c and c.induction_a is not None else 1.0 / 3.0)
    return row


def main() -> int:
    ap = argparse.ArgumentParser(description="2x3 six-turbine farm control interface test (steady 11 m/s)")
    ap.add_argument("--control", choices=["yaw", "induction"], required=True)
    ap.add_argument("--duration", type=float, default=DURATION)
    ap.add_argument("--step-at", type=float, default=STEP_AT)
    ap.add_argument("--out-root", type=Path, default=_STUDY_ROOT / "results")
    args = ap.parse_args()

    from induction_vs_yaw_study.run._proc_utils import cleanup_stale_fastfarm
    cleanup_stale_fastfarm(min_age_s=300.0)

    n_total = max(1, int(round(args.duration / DT)))
    step_step = max(1, int(round(args.step_at / DT)))

    out_dir = args.out_root / f"_six_turbine_{args.control}" / _time.strftime("%Y%m%d_%H%M%S")
    out_dir.mkdir(parents=True, exist_ok=True)

    cfg = build_config(out_dir)
    _base = 28000 + (_os.getpid() % 1500) * 2
    print(f"== six-turbine {args.control} test: 6T(2x3) / 270deg / {WIND_SPEED} m/s steady / "
          f"step@{args.step_at}s on T{CTRL_TURBINES} / {args.duration:.0f}s / ports {_base}+1")

    engine = FastFarmZmqEngine(cfg, cmd_port=_base, state_port=_base + 1)
    engine.setup()
    # 启用验证输出：逐机文本 .out（NacYaw 等）+ 农场 VTK 扰动风 XY 平面
    configure_verification_outputs(engine)
    try:
        from induction_vs_yaw_study.run.add_load_channels import add_load_channels
        _ff_base = getattr(engine._ff, "_farm_base", None)
        if _ff_base:
            add_load_channels(_ff_base, verbose=False)
    except Exception as _e:
        print(f"[warn] load channels: {_e}")

    t0 = _time.time()
    engine.start()
    bridge = ZmqFarmBridge(engine, len(ALL_TURBINES), wind_direction=WIND_DIR)
    rows = []
    seq_ctr = {"yaw": {}, "ind": {}}
    commanded = {}
    try:
        br_ok = False
        while _time.time() - t0 < 90:
            st = bridge.get_state(timeout_s=5.0)
            if st is not None and st.get("fast_step", 0) > 0:
                br_ok = True
                break
        if not br_ok:
            raise TimeoutError("FAST.Farm 未在 90s 内发出状态帧")

        for k in range(1, n_total + 1):
            bridge.wait_step(k, timeout=240.0)
            if k == 1:
                commanded = baseline_cmds(args.control)
                _send_cmds(engine, commanded, seq_ctr)
            elif k == step_step:
                commanded = step_cmds(args.control)
                _send_cmds(engine, commanded, seq_ctr)
            try:
                st = engine.get_state(timeout_s=5.0)
            except Exception:
                st = None
            rows.append(extract_row(st, commanded))

        _proc = getattr(engine, "_proc", None)
        if _proc is not None:
            try:
                _proc.wait(timeout=600)
            except Exception:
                pass
    finally:
        try:
            engine.stop()
        except Exception:
            pass
        try:
            engine.close()
        except Exception:
            pass

    wall = _time.time() - t0

    # 落盘 CSV
    import csv
    cols = list(rows[0].keys()) if rows else []
    with open(out_dir / "interface_timeseries.csv", "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=cols)
        w.writeheader()
        for r in rows:
            w.writerow(r)

    summary = {
        "control": args.control, "layout": "6T_2x3", "wind_speed": WIND_SPEED,
        "wind_dir": WIND_DIR, "bts": UNIFORM_BTS, "duration_s": args.duration,
        "dt_s": DT, "step_at_s": args.step_at, "ctrl_turbines": CTRL_TURBINES,
        "step_yaw_deg": STEP_DEG, "step_a": STEP_A, "wall_s": wall,
        "n_steps": len(rows),
    }
    (out_dir / "scenario.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")

    print(f"wall={wall:.0f}s  steps={len(rows)}  -> {out_dir / 'interface_timeseries.csv'}")
    return 0


if __name__ == "__main__":
    _sys.exit(main())
