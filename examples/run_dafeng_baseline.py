"""
DafengH1 FAST.Farm Baseline Simulation
========================================
Fixed yaw and pitch inputs for 24 turbines.
Runs FAST.Farm standalone (no MPI interface),
then parses each turbine's output file for power and loads.

Usage:
    python examples/run_dafeng_baseline.py

Outputs (in __simul__/fastfarm/):
    - powers_timeseries.csv  : 24-turbine power (MW) per time step
    - loads_timeseries.csv   : 6 load components per turbine per time step
    - total_farm_power.png   : time-series plot of total farm power
"""

import argparse
import os
import re
import sys
import time

import numpy as np
import pandas as pd

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

# ---------------------------------------------------------------------------
# Parse arguments
# ---------------------------------------------------------------------------
parser = argparse.ArgumentParser(description="DafengH1 FAST.Farm Baseline Simulation")
parser.add_argument("--case", type=str, default="DafengH1",
                    help="Case name from data_cases (e.g. DafengH1, Ablaincourt)")
parser.add_argument("--wind_speed", type=float, default=10.0, help="Wind speed (m/s)")
parser.add_argument("--wind_direction", type=float, default=270.0,
                    help="Wind direction in meteorological degrees (0=North, 90=East)")
parser.add_argument("--sim_time", type=float, default=10.0,
                    help="Total simulation time (s)")
parser.add_argument("--dt", type=float, default=3.0,
                    help="FAST.Farm time step (s)")
parser.add_argument("--yaw", type=float, default=None,
                    help="Fixed yaw angle for all turbines (deg). Default: faces into wind.")
parser.add_argument("--pitch", type=float, default=0.0,
                    help="Fixed pitch angle for all turbines (deg)")
args = parser.parse_args()

# ---------------------------------------------------------------------------
# Derived parameters
# ---------------------------------------------------------------------------
dt = args.dt
max_iter = int(np.ceil(args.sim_time / dt))
total_time = max_iter * dt

# FAST.Farm PropagationDir: 0=wind from West(270°), 90=from North(0°),
# 180=from East(90°), 270=from South(180°)
propagation_dir = (args.wind_direction + 90) % 360
yaw_angle = float(propagation_dir) if args.yaw is None else float(args.yaw)
pitch_angle = args.pitch

print("=" * 60)
print("DafengH1 Baseline — Standalone FAST.Farm Simulation")
print("=" * 60)
print(f"  Wind:       {args.wind_speed} m/s from {args.wind_direction}°")
print(f"  Yaw:        {yaw_angle:.0f}° for all turbines")
print(f"  Pitch:      {pitch_angle:.0f}° for all turbines")
print(f"  Sim time:   {total_time:.0f}s ({max_iter} steps × {dt}s)")
print(f"  Turbines:   24")

# ---------------------------------------------------------------------------
# Imports (after sys.path so wfcrl is importable)
# ---------------------------------------------------------------------------
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import seaborn as sns

from wfcrl.environments.data_cases import named_cases_dictionary
from openfast_toolbox.io import FASTOutputFile

sns.set_theme(style="darkgrid")

# ===========================================================================
# 1. Create DafengH1 case configuration
# ===========================================================================
# 2-6. Use FastFarmStandaloneInterface (from wfcrl.interface)
# ===========================================================================
from wfcrl.interface import FastFarmStandaloneInterface

case_key = args.case + "_"
if case_key not in named_cases_dictionary:
    avail = [k.rstrip("_") for k in named_cases_dictionary]
    print(f"Unknown case '{args.case}'. Available: {avail}"); sys.exit(1)
base_case = named_cases_dictionary[case_key][0]
n_turbines = base_case.num_turbines

config_dict = base_case.dict()
config_dict["max_iter"] = max_iter
config_dict["dt"] = dt
config_dict["t_init"] = 0
config_dict["speed"] = args.wind_speed
config_dict["direction"] = args.wind_direction

timestamp = time.time()
output_dir = os.path.join(
    os.path.dirname(__file__), "..",
    "__simul__", "fastfarm",
    f"DafengH1_Baseline_{args.wind_speed}ms_{args.wind_direction}deg_{timestamp:.0f}"
)
os.makedirs(output_dir, exist_ok=True)

ff_io = FastFarmStandaloneInterface(config_dict, output_dir)
ff_io.setup()
print(f"  [1/5] Input files ready | {output_dir}")
print(f"  [2/5] Wind: {args.wind_speed} m/s, {args.wind_direction} deg (PropagationDir={propagation_dir})")

ff_io.set_yaw_pitch(yaw_angle, pitch_angle)
print(f"  [3/5] Set NacYaw={yaw_angle:.0f} deg, BlPitch={pitch_angle:.0f} deg for {n_turbines} turbines")

print(f"  [4/5] Running FAST.Farm...")
measurements = ff_io.run()
result_returncode = 0
print(f"  [4/5] FAST.Farm completed")

print(f"\n  [5/5] Parsing output files...")
turbine_powers = []; turbine_loads = []; time_vector = None
_out_prefix = os.path.splitext(os.path.basename(ff_io.fstf_file))[0]
farm_base = ff_io.farm_base

for i in range(1, n_turbines + 1):
    ob = os.path.join(farm_base, f"{_out_prefix}.T{i}.outb")
    o = os.path.join(farm_base, f"{_out_prefix}.T{i}.out")
    if os.path.exists(ob): df = FASTOutputFile(ob).toDataFrame()
    elif os.path.exists(o): df = FASTOutputFile(o).toDataFrame()
    else: continue
    if time_vector is None: time_vector = df.iloc[:, 0].values
    for col in df.columns:
        base = col.split('[')[0].strip('_ ')
        if base == 'GenPwr':
            unit = col.split('[')[1].split(']')[0] if '[' in col else 'W'
            p = df[col].values
            p_mw = p / 1e3 if unit.upper() in ('KW','KILOWATT','KWATT') else p / 1e6
            turbine_powers.append(p_mw); break
    for col in df.columns:
        b = col.split('[')[0].strip('_ ')
        if b.startswith('RootMIP') or b.startswith('RootMOoP') or b.startswith('RootMzb'):
            turbine_loads.append(df[col].values)

n_parsed = len(turbine_powers)
print(f"  Turbine outputs: {n_parsed}/{n_turbines} parsed")

# If we have power data from measurements, use it
power_array = np.array(turbine_powers).T if turbine_powers else np.zeros((0, n_turbines))
time_array = time_vector if time_vector is not None else np.arange(max_iter) * dt
print(f"\nSaving results...")

if turbine_powers:
    power_array = np.column_stack(turbine_powers)
    n_steps = power_array.shape[0]
    if time_vector is None or len(time_vector) != n_steps:
        time_vector = np.arange(n_steps) * dt
    tv = time_vector[:n_steps]

    cols = [f"T{i+1}_power_MW" for i in range(n_turbines)]
    pdf = pd.DataFrame(power_array, columns=cols)
    pdf.insert(0, "time_s", tv)
    pdf.insert(1, "total_power_MW", power_array.sum(axis=1))
    p_csv = os.path.join(output_dir, "powers_timeseries.csv")
    pdf.to_csv(p_csv, index=False, float_format="%.6f")
    print(f"  ✓ Powers CSV: {p_csv}")
    mean_power = power_array.sum(axis=1).mean()
else:
    mean_power = 0.0
    print(f"  ⚠ No power data saved (check turbine output channels)")

if turbine_loads:
    larr = np.column_stack(turbine_loads)
    lcols = [f"load_{j}" for j in range(larr.shape[1])]
    ldf = pd.DataFrame(larr, columns=lcols)
    ldf.insert(0, "time_s", tv[:len(larr)])
    l_csv = os.path.join(output_dir, "loads_timeseries.csv")
    ldf.to_csv(l_csv, index=False, float_format="%.6e")
    print(f"  ✓ Loads CSV: {l_csv}")

# ===========================================================================
# 8. Plot total farm power
# ===========================================================================
print(f"\nGenerating plots...")
fig, axes = plt.subplots(nrows=2, figsize=(12, 10))

ax = axes[0]
if turbine_powers:
    tp = power_array.sum(axis=1)
    ax.plot(tv, tp, "b-", linewidth=2)
    ax.axhline(mean_power, color="r", linestyle="--", alpha=0.7,
               label=f"Mean: {mean_power:.2f} MW")
    ax.legend()
else:
    ax.text(0.5, 0.5, "No power data", transform=ax.transAxes, ha="center")
ax.set_xlabel("Time (s)"); ax.set_ylabel("Total Farm Power (MW)")
ax.set_title(f"DafengH1 — Total Farm Power\n"
             f"(Wind: {args.wind_speed} m/s, {args.wind_direction}° | "
             f"Yaw={yaw_angle:.0f}°, Pitch={pitch_angle:.0f}°)")
ax.grid(True, alpha=0.3)

ax = axes[1]
if turbine_powers:
    n_plot = min(6, power_array.shape[1])
    for t in range(n_plot):
        ax.plot(tv, power_array[:, t], label=f"T{t+1}", lw=1.5, alpha=0.8)
    ax.set_ylabel("Turbine Power (MW)")
    ax.legend(ncol=3, fontsize=8)
else:
    ax.text(0.5, 0.5, "No power data", transform=ax.transAxes, ha="center")
ax.set_xlabel("Time (s)")
ax.set_title(f"Individual Turbine Power (first {min(6, n_turbines)})")
ax.grid(True, alpha=0.3)

plt.tight_layout()
plot_path = os.path.join(output_dir, "total_farm_power.png")
fig.savefig(plot_path, dpi=150, bbox_inches="tight")
plt.close(fig)
print(f"  ✓ Plot: {plot_path}")

# ===========================================================================
# 9. Summary
# ===========================================================================
print("\n" + "=" * 60)
print("SUMMARY")
print("=" * 60)
print(f"  Turbines:         {n_turbines}")
print(f"  Wind speed:       {args.wind_speed} m/s")
print(f"  Wind direction:   {args.wind_direction}°")
print(f"  Yaw:              {yaw_angle:.0f}°")
print(f"  Pitch:            {pitch_angle:.0f}°")
print(f"  Simulation:       {total_time:.0f}s ({max_iter} steps × {dt}s)")
print(f"  Mean farm power:  {mean_power:.2f} MW")
print(f"  FAST.Farm:        {'OK' if result_returncode == 0 else f'code {result_returncode}'}")
print(f"  Output:           {output_dir}")
print("=" * 60)
