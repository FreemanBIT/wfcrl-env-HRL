"""
FAST.Farm 连续闭环控制示例 (5-Mode Farm Protocol)
=================================================
使用 ContinuousFastFarmInterface + ROSCO WFCRL Bridge 实现场控。

控制模式 (--mode):
  0: 纯偏航增量控制 (yaw delta)
  1: 功率目标 + 最小变桨约束 (power + min pitch)
  2: 纯变桨绝对值控制 (pitch absolute)
  3: 变桨绝对值 + 偏航增量 (pitch + yaw)
  4: 功率目标 + 最小变桨约束 + 偏航增量 (power + min pitch + yaw)

用法:
    python examples/example_FASTFarm.py --mode 0 --steps 10 --wind_speed 10
    python examples/example_FASTFarm.py --mode 1 --power 4.0 --min_pitch 5.0 --steps 20
    python examples/example_FASTFarm.py --mode 2 --pitch 10 --steps 20
    python examples/example_FASTFarm.py --mode 3 --pitch 10 --yaw_amp 20 --steps 20
    python examples/example_FASTFarm.py --mode 4 --power 3.0 --min_pitch 3.0 --yaw_amp 15 --steps 20
"""
import argparse, os, sys, time
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from wfcrl.config import WindConfig, ControlInput
from wfcrl.interface import ContinuousFastFarmInterface
from wfcrl.environments.data_cases import named_cases_dictionary
from wfcrl.simul_config import FastFarmConfig

# --- CLI ---
parser = argparse.ArgumentParser(
    description="FAST.Farm 5-Mode Farm Control Example",
    formatter_class=argparse.RawDescriptionHelpFormatter,
    epilog="""
Mode details:
  0: yaw delta sweep (sinusoidal)
  1: power target + min pitch constraint (step test)
  2: pitch absolute (sinusoidal sweep)
  3: pitch absolute + yaw delta
  4: power + min pitch + yaw delta
""")
parser.add_argument("--case", default="Turb3_Row1")
parser.add_argument("--steps", type=int, default=20, help="Number of control steps")
parser.add_argument("--wind_speed", type=float, default=10.0)
parser.add_argument("--wind_direction", type=float, default=270.0)
parser.add_argument("--mode", type=int, default=0, choices=[0,1,2,3,4],
                    help="Farm control mode (0-4)")
parser.add_argument("--power", type=float, default=4.0,
                    help="Power target in MW (modes 1,4)")
parser.add_argument("--min_pitch", type=float, default=0.0,
                    help="Min pitch constraint in deg (modes 1,4)")
parser.add_argument("--pitch", type=float, default=5.0,
                    help="Pitch command in deg (modes 2,3)")
parser.add_argument("--yaw_amp", type=float, default=15.0,
                    help="Yaw delta amplitude in deg (modes 0,3,4)")
parser.add_argument("--yaw_period", type=int, default=8,
                    help="Yaw delta period in steps")
args = parser.parse_args()

# --- Case setup ---
key = args.case + "_"
if key not in named_cases_dictionary:
    avail = [k.rstrip("_") for k in named_cases_dictionary]
    print(f"Unknown case. Available: {avail}"); sys.exit(1)

base_case = named_cases_dictionary[key][0]
n_turbs = base_case.num_turbines

wind = WindConfig(speed=args.wind_speed, direction=args.wind_direction)
config = FastFarmConfig(
    case_name=args.case, num_turbines=n_turbs,
    xcoords=base_case.xcoords, ycoords=base_case.ycoords,
    dt=3.0, max_iter=args.steps, wind=wind,
)

ts = time.time()
out_dir = os.path.join(os.path.dirname(__file__), "..",
    "__simul__", "fastfarm", "continuous",
    f"{args.case}_Mode{args.mode}_{ts:.0f}")
os.makedirs(out_dir, exist_ok=True)
config.output_dir = out_dir

mode_names = ["Yaw Only", "Power+MinPitch", "Pitch Only",
              "Pitch+Yaw", "Power+MinPitch+Yaw"]
print("=" * 70)
print(f"FAST.Farm 5-Mode Farm Control — Mode {args.mode}: {mode_names[args.mode]}")
print("=" * 70)
print(f"Case: {args.case} | Turbines: {n_turbs} | Steps: {args.steps}")
print(f"Wind: {args.wind_speed} m/s, {args.wind_direction} deg")
print(f"Mode: {args.mode} ({mode_names[args.mode]})")
print(f"Output: {out_dir}")
print("=" * 70)

# --- Create interface ---
ff = ContinuousFastFarmInterface(config)
ff.setup()
ff.reset(wind)

print(f"\nStarting FAST.Farm...")
ff.start()

# --- Control loop ---
step_powers = []
step_yaws = []
step_pitches = []
optimal_yaw = (270 - wind.direction) % 360  # OpenFAST: 0°=East(+X), +CCW

for step in range(args.steps):
    yaw_cmd = optimal_yaw  # default: face wind
    pitch_cmd = 0.0
    power_cmd = 0.0
    min_pitch_cmd = 0.0

    if args.mode in (0, 3, 4):
        # Sinusoidal yaw sweep around optimal (absolute yaw, OpenFAST coords)
        yaw_offset = args.yaw_amp * np.sin(2 * np.pi * step / args.yaw_period)
        yaw_offset = np.clip(yaw_offset, -25.0, 25.0)
        yaw_cmd = optimal_yaw + yaw_offset

    if args.mode == 1:
        power_cmd = args.power
        min_pitch_cmd = args.min_pitch

    if args.mode == 2:
        # Sinusoidal pitch sweep
        pitch_cmd = args.pitch * (1.0 + 0.5 * np.sin(2 * np.pi * step / 10))
        pitch_cmd = np.clip(pitch_cmd, 0.0, 30.0)

    if args.mode == 3:
        pitch_cmd = args.pitch

    if args.mode == 4:
        power_cmd = args.power
        min_pitch_cmd = args.min_pitch

    # Dispatch to factory methods
    if args.mode == 0:
        controls = ControlInput.mode0_yaw(n_turbs, yaw_cmd)
    elif args.mode == 1:
        controls = ControlInput.mode1_power(n_turbs, power_cmd, min_pitch_cmd)
    elif args.mode == 2:
        controls = ControlInput.mode2_pitch(n_turbs, pitch_cmd)
    elif args.mode == 3:
        controls = ControlInput.mode3_pitch_yaw(n_turbs, pitch_cmd, yaw_cmd)
    elif args.mode == 4:
        controls = ControlInput.mode4_power_yaw(n_turbs, power_cmd, min_pitch_cmd, yaw_cmd)

    output = ff.wait_step(controls)

    farm_pw = float(output.farm_power_mw[-1]) if output.farm_power_mw is not None else 0.0
    step_powers.append(farm_pw)
    step_yaws.append(yaw_cmd)
    step_pitches.append(pitch_cmd)

    # Status line
    parts = [f"Step {step+1:3d}/{args.steps}"]
    if args.mode in (0, 3, 4):
        parts.append(f"yaw={yaw_cmd:+.1f}°")
    if args.mode in (1, 4):
        parts.append(f"P_tgt={power_cmd:.1f}MW")
        if min_pitch_cmd > 0:
            parts.append(f"minPit={min_pitch_cmd:.1f}°")
    if args.mode in (2, 3):
        parts.append(f"pitch={pitch_cmd:.1f}°")
    parts.append(f"farm_pwr={farm_pw:.2f}MW")
    print(" | ".join(parts))

# --- Stop ---
print("\nWaiting for FAST.Farm to finish...")
final_output = ff.stop()
ff.close()

if final_output is not None and final_output.power_mw is not None and final_output.power_mw.size > 0:
    out_csv = os.path.join(out_dir, "full_output.csv")
    final_output.to_csv(out_csv)
    print(f"\nResults saved to {out_csv}")
    ffp = final_output.farm_power_mw
    print(f"Farm power range: {ffp.min():.2f} - {ffp.max():.2f} MW")

    # --- Plot ---
    t_arr = final_output.time
    fig, axes = plt.subplots(2, 1, figsize=(14, 10), sharex=True)

    ax = axes[0]
    ax.plot(t_arr, ffp, "b-", lw=2, label="Farm Power")
    ax.set_ylabel("Farm Power (MW)", fontsize=13)
    ax.set_title(f"{args.case} — Mode {args.mode}: {mode_names[args.mode]} "
                 f"(Wind: {wind.speed} m/s, {wind.direction}°)",
                 fontsize=14, fontweight="bold")
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize=11)

    ax = axes[1]
    pw = final_output.power_mw
    n_plot = min(6, n_turbs)
    colors = plt.cm.tab10(np.linspace(0, 1, n_plot))
    for i in range(n_plot):
        ax.plot(t_arr, pw[:, i], color=colors[i], lw=1.5,
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
    print(f"\nWarning: No power data in final output.")
    print(f"Check FAST.Farm log: {os.path.join(out_dir, 'fastfarm_continuous.log')}")
print("Done.")
