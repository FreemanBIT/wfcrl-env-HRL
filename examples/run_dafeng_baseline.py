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

from wfcrl.environments.data_cases import fastfarm_DafengH1
from wfcrl.interface import FastFarmInterface
from wfcrl.simul_utils import create_ff_case, write_inflow_info
from openfast_toolbox.io.fast_input_file import FASTInputFile
from openfast_toolbox.io import FASTOutputFile

sns.set_theme(style="darkgrid")

# ===========================================================================
# 1. Create DafengH1 case configuration
# ===========================================================================
base_case = fastfarm_DafengH1
n_turbines = base_case.num_turbines

config_dict = base_case.dict()
config_dict["max_iter"] = max_iter
config_dict["dt"] = dt
config_dict["t_init"] = 0
config_dict["speed"] = args.wind_speed

# ===========================================================================
# 2. Generate FAST.Farm input files
# ===========================================================================
timestamp = time.time()
output_dir = os.path.join(
    os.path.dirname(__file__), "..",
    "__simul__", "fastfarm",
    f"DafengH1_Baseline_{args.wind_speed}ms_{args.wind_direction}deg_{timestamp:.0f}"
)
os.makedirs(output_dir, exist_ok=True)

fstf_file = create_ff_case(config_dict, output_dir=output_dir)
fstf_dir = os.path.dirname(fstf_file)
farm_base = os.path.join(output_dir, "FarmInputs")
print(f"\n[1/5] Input files: {output_dir}")

# ===========================================================================
# 2b. Add OutList to generated .fst files (after create_ff_case, so FASTInputFile
#     in create_ff_case won't see the END line)
# ===========================================================================
# Read the generated fstf to find turbine .fst file references
fstf_tmp = FASTInputFile(fstf_file)
wt_refs_tmp = [row[3].replace('"', "") for row in fstf_tmp["WindTurbines"]]

out_channels = [
    "GenPwr", "GenTq", "RotSpeed",
    "RootMyb1", "RootMyb2", "RootMyb3",
    "RootMxb1", "RootMxb2", "RootMxb3",
    "YawPzn",
    "BldPitch1", "BldPitch2", "BldPitch3",
    "HSShftP",
]

for wt_ref in wt_refs_tmp:
    wt_path = os.path.join(farm_base, wt_ref)
    with open(wt_path, 'rb') as f:
        raw = f.read()
    text = raw.decode('ascii', errors='replace')

    # Find the OutFmt line and insert OutList after it
    # Don't insert if OutList already exists
    if 'OutList' in text:
        continue

    # Find a line starting with a quoted string near the end of OUTPUT section
    lines = text.split('\n')
    new_lines = []
    inserted = False
    for line in lines:
        new_lines.append(line)
        if not inserted and line.strip().startswith('"G0"'):
            # Insert OutList channels after OutFmt line
            new_lines.append('')  # blank line separator
            for ch in out_channels:
                new_lines.append(f'"{ch}"    {ch}')
            new_lines.append('END of input file')
            inserted = True

    if inserted:
        out_text = '\n'.join(new_lines)
        out_bytes = out_text.encode('ascii', errors='replace')
        out_bytes = out_bytes.replace(b'\r\n', b'\n').replace(b'\n', b'\r\n')
        with open(wt_path, 'wb') as f:
            f.write(out_bytes)

print(f"  Added OutList ({len(out_channels)} channels) to {len(wt_refs_tmp)} turbine .fst files")

# ===========================================================================
# 3. Set wind direction in InflowWind.dat
# ===========================================================================
inflow_path = os.path.join(farm_base, "InflowWind.dat")
if os.path.exists(inflow_path):
    inflow = FASTInputFile(inflow_path)
    inflow["PropagationDir"] = propagation_dir
    inflow.write(inflow_path)
    with open(inflow_path, 'rb') as f:
        iw_raw = f.read()
    iw_raw = iw_raw.replace(b'\r\n', b'\n').replace(b'\n', b'\r\n')
    iw_text = iw_raw.decode('ascii')
    iw_text = re.sub(
        r'^(\s*)[0-9.+-]+\s+RotorApexOffsetPos',
        r'\1 0.0, 0.0, 0.0   RotorApexOffsetPos',
        iw_text, flags=re.MULTILINE
    )
    with open(inflow_path, 'wb') as f:
        f.write(iw_text.encode('ascii'))
write_inflow_info(inflow_path, float(args.wind_speed))
print(f"  [2/5] Inflow: {args.wind_speed} m/s, direction {args.wind_direction}° "
      f"(PropagationDir={propagation_dir})")

# ===========================================================================
# 4. Set fixed yaw & pitch in each turbine's input files
# ===========================================================================
fstf = FASTInputFile(fstf_file)
wt_refs = [row[3].replace('"', "") for row in fstf["WindTurbines"]]

for i, wt_ref in enumerate(wt_refs):
    wt_path = os.path.join(farm_base, wt_ref)
    wt = FASTInputFile(wt_path)

    # Set yaw in ElastoDyn
    ed_file = wt["EDFile"].replace('"', "")
    ed_path = os.path.join(farm_base, ed_file)
    ed = FASTInputFile(ed_path)
    ed["NacYaw"] = yaw_angle
    ed["BlPitch(1)"] = pitch_angle
    ed["BlPitch(2)"] = pitch_angle
    ed["BlPitch(3)"] = pitch_angle
    ed.write(ed_path)
    with open(ed_path, 'rb') as f:
        raw = f.read()
    raw = raw.replace(b'\r\n', b'\n').replace(b'\n', b'\r\n')
    with open(ed_path, 'wb') as f:
        f.write(raw)

print(f"  [3/5] Set NacYaw={yaw_angle:.0f}°, BlPitch={pitch_angle:.0f}° "
      f"for {n_turbines} turbines")

# ===========================================================================
# 5. Run FAST.Farm standalone via subprocess
# ===========================================================================
ff_exe = FastFarmInterface.default_exe_path
if not os.path.exists(ff_exe):
    raise FileNotFoundError(f"FAST.Farm executable not found: {ff_exe}")

print(f"  [4/5] Launching FAST.Farm from {farm_base} ...")
print(f"  Executable: {ff_exe}")
# Run FAST.Farm directly (no mpiexec — SC_DLL removed, no MPI needed)
# Stream stdout so user sees progress
import subprocess as _sp
proc = _sp.Popen(
    [ff_exe, fstf_file],
    cwd=farm_base,
    stdout=_sp.PIPE, stderr=_sp.STDOUT,
    text=True, bufsize=1,
)
stdout_lines = []
for line in proc.stdout:
    stdout_lines.append(line)
    print(line, end='', flush=True)
proc.wait()
result_text = ''.join(stdout_lines)
result_returncode = proc.returncode

if result_returncode != 0:
    print(f"  WARNING: FAST.Farm exit code {result_returncode}")
else:
    print(f"  [4/5] FAST.Farm completed (exit code 0)")

# ===========================================================================
# 6. Parse output files
# ===========================================================================
print(f"\n  [5/5] Parsing output files...")

all_turbine_data = []
turbine_powers = []
turbine_loads = []
time_vector = None

# Derive output file prefix from fstf filename (e.g., "Case" from "Case.fstf")
_out_prefix = os.path.splitext(os.path.basename(fstf_file))[0]

for i in range(1, n_turbines + 1):
    outb_path = os.path.join(farm_base, f"{_out_prefix}.T{i}.outb")
    out_path = os.path.join(farm_base, f"{_out_prefix}.T{i}.out")

    if os.path.exists(outb_path):
        df = FASTOutputFile(outb_path).toDataFrame()
    elif os.path.exists(out_path):
        df = FASTOutputFile(out_path).toDataFrame()
    else:
        continue

    if time_vector is None:
        time_vector = df.iloc[:, 0].values  # first column = Time

    all_turbine_data.append(df)

    # Generator power — try various channel names
    # FASTOutputFile suffixes units in brackets, e.g. GenPwr_[kW], GenPwr_[W]
    power_found = False
    pw_keywords = ['GenPwr', 'HSShftP']
    for col in df.columns:
        base = col.split('[')[0].strip('_ ')  # strip unit suffix and trailing underscore
        if base in pw_keywords:
            val = df[col].values
            unit = col.split('[')[1].split(']')[0] if '[' in col else 'W'
            # Convert to MW
            if unit.upper() in ('W', 'WATT'):
                turbine_powers.append(val / 1e6)
            elif unit.upper() in ('KW', 'KILOWATT', 'kW'):
                turbine_powers.append(val / 1e3)  # kW → MW
            elif unit.upper() in ('MW', 'MEGAWATT', 'mW'):
                turbine_powers.append(val)
            else:
                turbine_powers.append(val / 1e6)  # assume W
            power_found = True
            break
    if not power_found:
        cols_sample = [c for c in df.columns if any(k in c for k in ['Pwr','pwr','Pow','Shft'])][:8]
        print(f"    T{i}: GenPwr not found. Power-related channels: {cols_sample}")

    # Blade root loads — match by base name, ignoring unit suffix
    load_keys = ['RootMIP', 'RootMOoP', 'RootMzb']
    row = []
    for base_key in load_keys:
        for col in df.columns:
            b = col.split('[')[0].strip('_ ')
            if b.startswith(base_key):
                row.append(df[col].values)
                break
    if row:
        turbine_loads.append(np.column_stack(row))

n_parsed = len(all_turbine_data)
print(f"  Turbine outputs: {n_parsed}/{n_turbines} parsed")

# If no individual turbine files, try farm-level output
if n_parsed == 0:
    farm_out_path = os.path.join(farm_base, "Case.out")
    if os.path.exists(farm_out_path):
        farm_df = FASTOutputFile(farm_out_path).toDataFrame()
        time_vector = farm_df.iloc[:, 0].values
        print(f"  Using farm-level Case.out ({len(farm_df)} rows)")
    else:
        time_vector = np.arange(max_iter) * dt
        print(f"  No output files found — using fallback time vector")

# ===========================================================================
# 7. Save results to CSV
# ===========================================================================
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
    lcols = []
    for t in range(len(turbine_loads)):
        for l in range(turbine_loads[t].shape[1]):
            lcols.append(f"T{t+1}_load_{l}")
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
