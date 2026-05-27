"""
FAST.Farm 连续闭环控制示例
===========================
使用 ContinuousFastFarmInterface 实现真正的连续流场在线控制。
FAST.Farm 启动一次，通过 DISCON bridge DLL 每个 timestep 交换控制/测量。

相比 FastFarmInterface（每步重启 FAST.Farm，流场均重置）：
- ContinuousFastFarmInterface：一次启动，流场持续演化（物理正确）

要求:
    - DISCON_WT1.dll 已编译（WFCRL Bridge）
    - FAST.Farm v5.0.0 可执行文件

用法:
    python examples/example_FASTFarm.py
    python examples/example_FASTFarm.py --steps 10 --wind_speed 12
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

parser = argparse.ArgumentParser()
parser.add_argument("--case", default="DafengH1")
parser.add_argument("--steps", type=int, default=3)
parser.add_argument("--wind_speed", type=float, default=10.0)
parser.add_argument("--wind_direction", type=float, default=270.0)
args = parser.parse_args()

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
    "__simul__", "fastfarm", "continuous", f"{args.case}_Continuous_{ts:.0f}")
os.makedirs(out_dir, exist_ok=True)
config.output_dir = out_dir

print("=" * 70)
print("FAST.Farm Continuous Closed-Loop Control")
print("=" * 70)
print(f"Case: {args.case} | Turbines: {n_turbs} | Steps: {args.steps}")
print(f"Wind: {args.wind_speed} m/s, {args.wind_direction} deg")
print(f"Output: {out_dir}")
print("=" * 70)

# 创建连续接口
ff = ContinuousFastFarmInterface(config)
ff.setup()       # 生成完整 .fstf + 部署 DISCON bridge DLL
ff.reset(wind)

print(f"\nStarting FAST.Farm (single process, continuous flow)...")
ff.start()       # 后台启动 FAST.Farm

# 在线控制循环
power_at_step = 0.0
prev_power = 0.0
yaw_val = 0.0
direction = 1.0
step_powers = []
step_yaws = []

for step in range(args.steps):
    # --- 控制算法（可替换为 RL / MPC / 查表）---
    if prev_power > 0 and step > 0:
        if power_at_step < prev_power * 0.98:
            direction *= -1
    yaw_val += direction * 5.0
    yaw_val = np.clip(yaw_val, -25, 25)
    pitch_cmd = 0.0

    controls = ControlInput.scalar(n_turbs, yaw_deg=yaw_val, pitch_deg=pitch_cmd)
    output = ff.wait_step(controls)

    power_at_step = float(output.farm_power_mw[-1]) if output.farm_power_mw is not None else 0.0
    prev_power = power_at_step
    step_powers.append(power_at_step)
    step_yaws.append(yaw_val)

    pitch_display = output.pitch_deg[-1, 0] if output.pitch_deg is not None else 0.0

    print(f"  Step {step+1:3d}/{args.steps}: yaw={yaw_val:+.1f} deg | "
          f"farm_power={power_at_step:.2f} MW | "
          f"pitch={pitch_display:.1f} deg")

# 停止并获取完整输出
print("\nWaiting for FAST.Farm to finish...")
final_output = ff.stop()
ff.close()

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
    ax.plot(t_arr, ffp, "b-", lw=2, label="Farm Power")
    ax.set_ylabel("Farm Power (MW)", fontsize=13)
    ax.set_title(f"{args.case} — FAST.Farm Control (Wind: {wind.speed} m/s, {wind.direction}°)",
                 fontsize=14, fontweight="bold")
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize=11)

    # 单机功率（前 6 台）
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
