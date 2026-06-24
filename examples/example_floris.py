"""
FLORIS 在线控制示例 (3-Mode Farm Control)
==========================================
使用 FlorisInterface 进行稳态风场在线控制。

控制模式 (--mode):
  0: 偏航控制 (yaw)
  1: 诱导因子控制 (curtailment ratio)
  2: 诱导因子 + 偏航控制

用法:
    python examples/example_floris.py --mode 0
    python examples/example_floris.py --mode 1 --ratio 0.7
    python examples/example_floris.py --mode 2 --ratio 0.5 --yaw_amp 25
"""
import argparse, os, sys, time
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from wfcrl.config import WindConfig, ControlInput
from wfcrl.interface import FlorisInterface, SimulatorInterface
from wfcrl.environments.data_cases import named_cases_dictionary
from wfcrl.simul_config import FlorisConfig

parser = argparse.ArgumentParser()
parser.add_argument("--case", default="Turb3_Row1")
parser.add_argument("--steps", type=int, default=100)
parser.add_argument("--wind_speed", type=float, default=10.0)
parser.add_argument("--wind_direction", type=float, default=270.0)
parser.add_argument("--mode", type=int, default=0, choices=[0, 1, 2],
                    help="0=yaw, 1=induction(curtail), 2=induction+yaw")
parser.add_argument("--ratio", type=float, default=1.0,
                    help="Curtailment ratio 0~1 (modes 1,2). 1.0=full power")
parser.add_argument("--yaw_amp", type=float, default=25.0,
                    help="Yaw amplitude in deg (modes 0,2)")
args = parser.parse_args()

key = args.case + "_"
if key not in named_cases_dictionary:
    avail = [k.rstrip("_") for k in named_cases_dictionary]
    print(f"Unknown case. Available: {avail}"); sys.exit(1)

# FLORIS 案例（data_cases 第二个元素）
base_case = named_cases_dictionary[key][1]
n_turbs = base_case.num_turbines

# FLORIS dt 通常 >= 60s，对齐 FAST.Farm 示例的总仿真时长
FLORIS_DT = max(60.0, float(base_case.dt))
total_sim_time = args.steps * 3.0  # 对齐 FASTFarm 的 dt=3.0 × steps
floris_steps = max(1, int(np.round(total_sim_time / FLORIS_DT)))

wind = WindConfig(speed=args.wind_speed, direction=args.wind_direction)
config = FlorisConfig(
    case_name=args.case, num_turbines=n_turbs,
    xcoords=base_case.xcoords, ycoords=base_case.ycoords,
    dt=FLORIS_DT, max_iter=floris_steps, wind=wind,
)

ts = time.time()
mode_names = ["Yaw Only", "Induction", "Induction+Yaw"]
out_dir = os.path.join(os.path.dirname(__file__), "..",
    "__simul__", "floris", f"{args.case}_Mode{args.mode}_{ts:.0f}")
os.makedirs(out_dir, exist_ok=True)
config.output_dir = out_dir

print("=" * 70)
print(f"FLORIS 3-Mode Control — Mode {args.mode}: {mode_names[args.mode]}")
print("=" * 70)
print(f"Case: {args.case} | Turbines: {n_turbs}")
print(f"Wind: {args.wind_speed} m/s, {args.wind_direction} deg")
print(f"FLORIS DT: {FLORIS_DT}s | Steps: {floris_steps}")
print(f"Total simulated time: {total_sim_time:.0f}s")
print(f"Mode: {args.mode} ({mode_names[args.mode]})")
if args.mode in (1, 2):
    print(f"Curtail ratio: {args.ratio:.2f}")
print(f"Output: {out_dir}")
print("=" * 70)

# 创建 FLORIS 接口
fl = FlorisInterface(config)
fl.setup()
fl.reset(wind)

# 在线控制循环
all_outputs = []
step_yaws = []
step_ratios = []

for step in range(floris_steps):
    # Yaw: sinusoidal sweep
    yaw_val = args.yaw_amp * np.sin(2 * np.pi * step / max(1, floris_steps / 4))
    ratio_val = args.ratio

    if args.mode == 0:
        controls = ControlInput.mode0_yaw(n_turbs, yaw_val)
    elif args.mode == 1:
        controls = ControlInput.mode1_power(n_turbs, ratio_val)
    elif args.mode == 2:
        controls = ControlInput.mode4_power_yaw(n_turbs, ratio_val, 0.0, yaw_val)

    output = fl.step(controls)

    farm_pw = float(output.farm_power_mw[-1]) if output.farm_power_mw is not None else 0.0
    all_outputs.append(output)
    step_yaws.append(yaw_val)
    step_ratios.append(ratio_val)

    parts = [f"Step {step+1:3d}/{floris_steps}"]
    if args.mode in (0, 2):
        parts.append(f"yaw={yaw_val:+.1f}°")
    if args.mode in (1, 2):
        parts.append(f"ratio={ratio_val:.2f}")
    parts.append(f"farm_pwr={farm_pw:.2f}MW")
    print(" | ".join(parts))

fl.close()

# 合并输出
final_output = SimulatorInterface._merge_outputs(all_outputs)

if final_output is not None and final_output.power_mw is not None and final_output.power_mw.size > 0:
    out_csv = os.path.join(out_dir, "full_output.csv")
    final_output.to_csv(out_csv)
    print(f"\nResults saved to {out_csv}")
    ffp = final_output.farm_power_mw
    print(f"Farm power range: {ffp.min():.2f} - {ffp.max():.2f} MW")

    # ====== 功率时序画图 ======
    t_arr = final_output.time
    fig, axes = plt.subplots(2, 1, figsize=(14, 10), sharex=True)

    # 全场功率
    ax = axes[0]
    ax.plot(t_arr, ffp, "r-o", lw=2, ms=5, label="Farm Power")
    ax.set_ylabel("Farm Power (MW)", fontsize=13)
    ax.set_title(f"{args.case} — FLORIS Control (Wind: {wind.speed} m/s, {wind.direction}°)",
                 fontsize=14, fontweight="bold")
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize=11)

    # 单机功率（前 6 台）
    ax = axes[1]
    pw = final_output.power_mw
    n_plot = min(6, n_turbs)
    colors = plt.cm.tab10(np.linspace(0, 1, n_plot))
    for i in range(n_plot):
        ax.plot(t_arr, pw[:, i], color=colors[i], lw=1.5, marker="o", ms=4,
                label=f"T{i+1} (mean={pw[:, i].mean():.2f} MW)")
    ax.set_xlabel("Time (s)", fontsize=13)
    ax.set_ylabel("Per-Turbine Power (MW)", fontsize=13)
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize=9, ncol=2)

    png_path = os.path.join(out_dir, "power_timeseries.png")
    fig.savefig(png_path, dpi=150, bbox_inches="tight")
    plt.close(fig)
    print(f"Plot saved to {png_path}")
else:
    print("Warning: No power data collected.")
print("Done.")
