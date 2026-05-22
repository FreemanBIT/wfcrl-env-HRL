"""
FAST.Farm Standalone Simulation Example
=========================================
Runs any FAST.Farm case from data_cases using the standalone interface.
Usage:
    python examples/example_fastfarm.py --case DafengH1 --steps 5
    python examples/example_fastfarm.py --case Turb6_Row2 --steps 3
"""
import argparse, numpy as np, os, sys, time
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

parser = argparse.ArgumentParser()
parser.add_argument("--case", type=str, default="DafengH1",
                    help="Case name from data_cases (e.g. DafengH1, Ablaincourt)")
parser.add_argument("--steps", type=int, default=5)
parser.add_argument("--wind_speed", type=float, default=10.0)
parser.add_argument("--yaw", type=float, default=0.0)
parser.add_argument("--pitch", type=float, default=0.0)
args = parser.parse_args()

from wfcrl.environments.data_cases import named_cases_dictionary

key = args.case + "_"
if key not in named_cases_dictionary:
    avail = [k.rstrip("_") for k in named_cases_dictionary]
    print(f"Unknown case '{args.case}'. Available: {avail}"); sys.exit(1)

farm_case = named_cases_dictionary[key][0]  # [0]=FAST.Farm case
print(f"Case: {args.case} ({farm_case.num_turbines} turbines)")

from wfcrl.interface import FastFarmStandaloneInterface

config = farm_case.dict()
config["max_iter"] = args.steps
config["speed"] = args.wind_speed
config["dt"] = getattr(farm_case, 'dt', 3)
config["t_init"] = 0

ts = time.time()
out_dir = os.path.join(os.path.dirname(__file__), "..",
    "__simul__", "fastfarm", f"{args.case}_{ts:.0f}")
os.makedirs(out_dir, exist_ok=True)

ff = FastFarmStandaloneInterface(config, out_dir)
ff.setup()
ff.set_yaw_pitch(args.yaw, args.pitch)
print(f"Running {args.steps} steps...")
meas = ff.run()

if meas['power_mw'] is not None:
    fp = meas['power_mw'].sum(axis=1)
    print(f"Farm power: mean={fp.mean():.2f} MW, range=[{fp.min():.2f}, {fp.max():.2f}] MW")
else:
    print("No power data")
