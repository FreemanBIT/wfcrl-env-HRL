"""generate_wind_boxes.py — 按 (U, TI, seed) 生成统一 TurbSim 风盒 .inp（V2 重构）

替换旧 generate_turbsim_inputs.py（旧：U x TI x spacing -> 每间距一盒 + seed 循环递增）。
新规则（开发计划 §1/§4.2，约束 1/3/4）：
  * 唯一键 = (风速, TI, seed)；固定盒宽 1500 m（覆盖 8D/20° 最不利几何）；
  * 命名 inflow_U{U:02d}_TI{TI:02d}_seed{seed:02d}.inp|.bts，不含 spacing/WD/control；
  * RandSeed1 取 config/seed_manifest.csv 的固定 rand_seed_1（禁止循环 seed+=1）；
  * 默认 AnalysisTime 覆盖全部查询时刻（规避 FAST.Farm v5 Grid4D 时间墙）。

用法：
    python -m induction_vs_yaw_study.turbsim_inp.generate_wind_boxes            # 72 个 .inp
    python -m induction_vs_yaw_study.turbsim_inp.generate_wind_boxes \
        --seeds 1 --speeds 8 --tis 0.10 --run --turbsim <exe>                  # 引导盒（Phase 1）
"""
from __future__ import annotations

import argparse
import csv
import math
import os
import subprocess
import sys
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

import yaml

STUDY_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_CONFIG = STUDY_ROOT / "config" / "wind_conditions.yaml"
DEFAULT_OUT = STUDY_ROOT / "FarmInputs" / "coarse"
DEFAULT_MANIFEST = STUDY_ROOT / "config" / "seed_manifest.csv"

# TurbSim .inp 模板（逐字段替换；数值全部来自 config/wind_conditions.yaml）
TEMPLATE = """---------TurbSim v2.00.* Input File------------------------
3xNREL-5MW rotated-layout Yaw-vs-Induction study inflow (V2 seed-key).
---------Runtime Options-----------------------------------
False        Echo            - Echo input data to <RootName>.ech (flag)
{seed:d}   RandSeed1       - First random seed  (-2147483648 to 2147483647)
{seed2:d}   RandSeed2       - Second random seed (-2147483648 to 2147483647) for intrinsic pRNG, or an alternative pRNG: "RanLux" or "RNSNLW"
False        WrBHHTP         - Output hub-height turbulence parameters in binary form?  (Generates RootName.bin)
False        WrFHHTP         - Output hub-height turbulence parameters in formatted form?  (Generates RootName.dat)
False        WrADHH          - Output hub-height time-series data in AeroDyn form?  (Generates RootName.hh)
True         WrADFF          - Output full-field time-series data in TurbSim/AeroDyn form? (Generates RootName.bts)
False        WrBLFF          - Output full-field time-series data in BLADED/AeroDyn form?  (Generates RootName.wnd)
False        WrADTWR         - Output tower time-series data? (Generates RootName.twr)
False        WrFMTFF         - Output full-field time-series data in formatted (readable) form?  (Generates RootName.u, RootName.v, RootName.w)
False        WrACT           - Output coherent turbulence time steps in AeroDyn form? (Generates RootName.cts)
True         Clockwise       - Clockwise rotation looking downwind? (used only for full-field binary files - not necessary for AeroDyn)
         1   ScaleIEC        - Scale IEC turbulence models to exact target standard deviation? [0=no additional scaling; 1=use hub scale uniformly; 2=use individual scales]

--------Turbine/Model Specifications-----------------------
       {numgrid_z:d}   NumGrid_Z       - Vertical grid-point matrix dimension
      {numgrid_y:d}   NumGrid_Y       - Horizontal grid-point matrix dimension
      {time_step:.4g}   TimeStep        - Time step [seconds]
    {analysis_time:.1f}   AnalysisTime    - Length of analysis time series [seconds] (program will add time if necessary: AnalysisTime = MAX(AnalysisTime, UsableTime+GridWidth/MeanHHWS) )
    "{usable_time}"   UsableTime      - Usable length of output time series [seconds] (program will add GridWidth/MeanHHWS seconds unless UsableTime is "ALL")
      {hub_ht:g}   HubHt           - Hub height [m] (should be > 0.5*GridHeight)
      {grid_height:g}   GridHeight      - Grid height [m]
     {grid_width:g}   GridWidth       - Grid width [m] (should be >= 2*(RotorRadius+ShaftLength))
        0   VFlowAng        - Vertical mean flow (uptilt) angle [degrees]
        0   HFlowAng        - Horizontal mean flow (skew) angle [degrees]

--------Meteorological Boundary Conditions-------------------
"{turb_model}"    TurbModel       - Turbulence model ("IECKAI","IECVKM","GP_LLJ","NWTCUP","SMOOTH","WF_UPW","WF_07D","WF_14D","TIDAL","API","USRINP","TIMESR", or "NONE")
"None"      UserFile        - Name of the file that contains inputs for user-defined spectra or time series inputs (used only for "USRINP" and "TIMESR" models)
        1   IECstandard     - Number of IEC 61400-x standard (x=1,2, or 3 with optional 61400.1 edition number (i.e. "1-Ed2") )
"{iecturbc:.1f}"      IECturbc        - IEC turbulence characteristic ("A", "B", "C" or the turbulence intensity in percent) ("KHTEST" option with NWTCUP model, not used for other models)
"{iec_wind_type}"       IEC_WindType    - IEC turbulence type ("NTM"=normal, "xETM"=extreme turbulence, "xEWM1"=extreme 1-year wind, "xEWM50"=extreme 50-year wind, where x=wind turbine class 1, 2, or 3)
"default"   ETMc            - IEC Extreme Turbulence Model "c" parameter [m/s]
"PL"        WindProfileType - Velocity profile type ("LOG";"PL"=power law;"JET";"H2L"=Log law for TIDAL model;"API";"USR";"TS";"IEC"=PL on rotor disk, LOG elsewhere; or "default")
"None"      ProfileFile     - Name of the file that contains input profiles for WindProfileType="USR" and/or TurbModel="USRVKM" [-]
     {ref_ht:g}   RefHt           - Height of the reference velocity (URef) [m]
      {uref:g}   URef            - Mean (total) velocity at the reference height [m/s] (or "default" for JET velocity profile) [must be 1-hr mean for API model; otherwise is the mean over AnalysisTime seconds]
      350   ZJetMax         - Jet height [m] (used only for JET velocity profile, valid 70-490 m)
"{pl_exp}"       PLExp           - Power law exponent [-] (or "default")
"default"   Z0              - Surface roughness length [m] (or "default")

--------Non-IEC Meteorological Boundary Conditions------------
"default"   Latitude        - Site latitude [degrees] (or "default")
     0.05   RICH_NO         - Gradient Richardson number [-]
"default"   UStar           - Friction or shear velocity [m/s] (or "default")
"default"   ZI              - Mixing layer depth [m] (or "default")
"default"   PC_UW           - Hub mean u'w' Reynolds stress [m^2/s^2] (or "default" or "none")
"default"   PC_UV           - Hub mean v'w' Reynolds stress [m^2/s^2] (or "default" or "none")
"default"   PC_VW           - Hub mean w'w' Reynolds stress [m^2/s^2] (or "default" or "none")

--------Spatial Coherence Parameters----------------------------
"{scmod1}"       SCMod1           - u-component coherence model ("GENERAL","IEC","API","NONE", or "default")
"{scmod2}"       SCMod2           - v-component coherence model ("GENERAL","IEC","NONE", or "default")
"{scmod3}"       SCMod3           - w-component coherence model ("GENERAL","IEC","NONE", or "default")
"{incdec1}"     InCDec1          - u-component coherence parameters for general or IEC models [-, m^-1] (e.g. "10.0  0.3e-3" in quotes) (or "default")
"{incdec2}"     InCDec2          - v-component coherence parameters for general or IEC models [-, m^-1] (e.g. "10.0  0.3e-3" in quotes) (or "default")
"{incdec3}"     InCDec3          - w-component coherence parameters for general or IEC models [-, m^-1] (e.g. "10.0  0.3e-3" in quotes) (or "default")
{coh_exp:g}         CohExp           - Coherence exponent for general model [-] (or "default")

--------Coherent Turbulence Scaling Parameters-------------------
".\\EventData"    CTEventPath     - Name of the path where event data files are located
"random"      CTEventFile     - Type of event files ("LES", "DNS", or "RANDOM")
true          Randomize       - Randomize the disturbance scale and locations? (true/false)
          1   DistScl         - Disturbance scale [-] (ratio of event dataset height to rotor disk). (Ignored when Randomize = true.)
        0.5   CTLy            - Fractional location of tower centerline from right [-] (looking downwind) to left side of the dataset. (Ignored when Randomize = true.)
        0.5   CTLz            - Fractional location of hub height from the bottom of the dataset. [-] (Ignored when Randomize = true.)
         30   CTStartTime     - Minimum start time for coherent structures in RootName.cts [seconds]

====================================================
! NOTE: Do not add or remove any lines in this file!
====================================================
"""


def load_config(path: Path) -> dict:
    with open(path, "r", encoding="utf-8") as f:
        return yaml.safe_load(f)


def numgrid_y_for(width_m: float, target_spacing_m: float) -> int:
    """NumGrid_Y 使 dY = W/(NY-1) <= target（~9 m）。"""
    ny = math.ceil(width_m / target_spacing_m) + 1
    dy = width_m / (ny - 1)
    assert dy <= target_spacing_m * 1.05, f"dY={dy:.2f} > target {target_spacing_m}"
    return ny


def seed2_of(rand_seed_1: int) -> int:
    # 与旧模板一致：RandSeed2 = RandSeed1 + 1000000
    return rand_seed_1 + 1000000


def read_seed_manifest(path: Path) -> dict:
    """返回 {seed_id: rand_seed_1}。"""
    out = {}
    with open(path, "r", newline="", encoding="utf-8") as f:
        for row in csv.DictReader(f):
            out[int(row["seed_id"])] = int(row["rand_seed_1"])
    return out


def write_seed_manifest(path: Path, seeds) -> None:
    """按约定 id -> 11001/22002/... 写 config/seed_manifest.csv。"""
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["seed_id", "rand_seed_1"])
        for sid in seeds:
            w.writerow([int(sid), int(sid) * 11001])
    print(f"seed manifest -> {path}")


def bts_name(speed: float, ti: float, seed_id: int) -> str:
    s = int(round(speed))
    t = int(round(ti * 100))
    return f"inflow_U{s:02d}_TI{t:02d}_seed{int(seed_id):02d}"


def build_inp_text(cfg: dict, speed: float, ti: float, seed_id: int) -> str:
    tb = cfg["turbsim"]
    width = float(tb["grid_width_m"])
    ny = numgrid_y_for(width, float(tb["target_grid_spacing_m"]))
    rs1 = read_seed_manifest(DEFAULT_MANIFEST)[int(seed_id)]
    return TEMPLATE.format(
        seed=rs1,
        seed2=seed2_of(rs1),
        numgrid_z=int(tb["num_grid_z"]),
        numgrid_y=ny,
        time_step=float(tb["time_step_s"]),
        analysis_time=float(tb["analysis_time_s"]),
        usable_time=tb["usable_time"],
        hub_ht=float(tb["artificial_hub_height_m"]),
        grid_height=float(tb["grid_height_m"]),
        grid_width=width,
        turb_model=tb["turbulence_model"],
        iecturbc=round(ti * 100.0, 1),
        iec_wind_type=tb["iec_wind_type"],
        ref_ht=float(tb["hub_height_ref_m"]),
        uref=speed,
        pl_exp=float(tb["shear_power_law_exponent"]),
        scmod1=tb["coherence"]["scmod_1"],
        scmod2=tb["coherence"]["scmod_2"],
        scmod3=tb["coherence"]["scmod_3"],
        incdec1=tb["coherence"]["incdec_1"],
        incdec2=tb["coherence"]["incdec_2"],
        incdec3=tb["coherence"]["incdec_3"],
        coh_exp=float(tb["coherence"]["coh_exp"]),
    )


def run_turbsim(exe: Path, inp_path: Path, timeout_s: int = 7200) -> int:
    """在 inp 所在目录运行 TurbSim（输出 .bts 与 .inp 同名同目录）。"""
    return subprocess.run(
        [str(exe), inp_path.name], cwd=str(inp_path.parent),
        capture_output=True, text=True, timeout=timeout_s,
    ).returncode


def main():
    ap = argparse.ArgumentParser(description="Generate unified (U,TI,seed) TurbSim .inp boxes")
    ap.add_argument("--out", type=Path, default=DEFAULT_OUT)
    ap.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    ap.add_argument("--speeds", nargs="+", type=float, default=None)
    ap.add_argument("--tis", nargs="+", type=float, default=None)
    ap.add_argument("--seeds", nargs="+", type=int, default=None)
    ap.add_argument("--run", action="store_true", help="生成后立即调用 TurbSim 逐个跑")
    ap.add_argument("--turbsim", type=Path, default=None,
                    help="TurbSim 可执行文件路径（--run 时必需）")
    ap.add_argument("--jobs", type=int, default=1, help="--run 并行实例数")
    args = ap.parse_args()

    cfg = load_config(args.config)
    speeds = args.speeds or cfg["wind_speeds_ms"]
    tis = args.tis or cfg["turbulence_intensities"]
    seeds = args.seeds or cfg["seeds"]

    out: Path = args.out
    out.mkdir(parents=True, exist_ok=True)

    # 确保 seed manifest 存在（幂等）
    if not DEFAULT_MANIFEST.exists():
        write_seed_manifest(DEFAULT_MANIFEST, cfg["seeds"])

    ny = numgrid_y_for(float(cfg["turbsim"]["grid_width_m"]),
                       float(cfg["turbsim"]["target_grid_spacing_m"]))
    dy = float(cfg["turbsim"]["grid_width_m"]) / (ny - 1)
    print(f"Box: W={cfg['turbsim']['grid_width_m']} m, NumGrid_Y={ny}, dY={dy:.2f} m")
    print(f"Generating {len(speeds)}U x {len(tis)}TI x {len(seeds)}seed "
          f"= {len(speeds)*len(tis)*len(seeds)} .inp -> {out.resolve()}")

    inps = []
    for seed_id in seeds:
        for speed in speeds:
            for ti in tis:
                name = bts_name(speed, ti, seed_id)
                txt = build_inp_text(cfg, speed, ti, seed_id)
                p = out / f"{name}.inp"
                p.write_text(txt, encoding="ascii")
                inps.append(p)
    print(f"Wrote {len(inps)} .inp")

    if args.run:
        exe = args.turbsim or (STUDY_ROOT.parent / "FarmInputs" / "TurbSim_x64.exe")
        if not Path(exe).exists():
            ap.error(f"TurbSim exe not found: {exe}")
        def _one(p: Path):
            print(f"== TurbSim: {p.name} ==", flush=True)
            rc = run_turbsim(Path(exe), p)
            print(f"   rc={rc}", flush=True)
            return p.name, rc
        with ThreadPoolExecutor(max_workers=max(1, args.jobs)) as pool:
            results = list(pool.map(_one, inps))
        bad = [(n, rc) for n, rc in results if rc != 0]
        print(f"TurbSim done: {len(results)-len(bad)} ok, {len(bad)} failed")
        for n, rc in bad:
            print(f"  FAIL {n}: rc={rc}")


if __name__ == "__main__":
    main()
