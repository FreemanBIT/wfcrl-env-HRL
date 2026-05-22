"""
DafengH1 FAST.Farm Simulation
==============================
Comprehensive demo of FAST.Farm integration with WFCRL.

Usage:
    mpiexec -n 1 python examples/run_dafeng_fastfarm.py --steps 3

This script demonstrates:
    1. Creating a Decentralized FastFarm environment (yaw control)
    2. Fixing InflowWind template (RotorApexOffsetPos comma format)
    3. Running a multi-agent step policy
    4. Plotting and saving results (yaw angles, farm power)
    5. Custom controls (yaw + pitch) with action/observation spaces
"""

import argparse
import numpy as np
import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

parser = argparse.ArgumentParser(description="DafengH1 FAST.Farm Simulation")
parser.add_argument("--steps", type=int, default=3, help="Number of RL steps")
parser.add_argument("--t_init", type=int, default=0, help="Initialization time before control (seconds; 0 for quick test)")
args = parser.parse_args()

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import pandas as pd
import seaborn as sns

from wfcrl import environments as envs

sns.set_theme(style="darkgrid")


# ---------------------------------------------------------------------------
# Helper: multi-agent step routine (same as demo.ipynb)
# ---------------------------------------------------------------------------
def multi_agent_step_routine(env, policy, max_cycles=None):
    """Run the multi-agent environment until all agents are done."""
    if max_cycles is None:
        max_cycles = getattr(env, 'max_num_steps', 500)
    from collections import deque
    agent_queue = deque(env.possible_agents)
    r = {agent: 0.0 for agent in env.possible_agents}
    num_steps = {agent: 0 for agent in env.possible_agents}
    cycles = 0
    while cycles < max_cycles:
        for agent in list(agent_queue):
            env.agent_selection = agent
            observation, reward, termination, truncation, info = env.last()
            if termination or truncation:
                r[agent] += reward
                agent_queue.remove(agent)
                action = None
            else:
                r[agent] += reward
                action = policy(num_steps[agent], env.agent_name_mapping[agent])
                num_steps[agent] += 1
            env.step(action)
            if not agent_queue:
                break
        cycles += 1
        if not agent_queue:
            break
    return r


# ---------------------------------------------------------------------------
# Output directory for plots
# ---------------------------------------------------------------------------
_output_dir = os.path.join(os.path.dirname(__file__), "..", "__simul__", "fastfarm")
os.makedirs(_output_dir, exist_ok=True)


# ============================================================================
# 1. Fix RotorApexOffsetPos in InflowWind template
# ============================================================================
print("=" * 60)
print("DafengH1 FAST.Farm - Comprehensive Demo")
print("=" * 60)

_iw_path = os.path.join(
    os.path.dirname(__file__), "..",
    "wfcrl", "simulators", "fastfarm", "inputs", "template", "FarmInputs", "InflowWind.dat"
)
if os.path.exists(_iw_path):
    with open(_iw_path) as f:
        _content = f.read()
    if "0.0, 0.0, 0.0   RotorApexOffsetPos" not in _content:
        _content = _content.replace(
            "0.0 0.0 0.0   RotorApexOffsetPos",
            "0.0, 0.0, 0.0   RotorApexOffsetPos"
        )
        with open(_iw_path, 'w') as f:
            f.write(_content)
        print("[OK] Fixed RotorApexOffsetPos to comma-separated format")
    else:
        print("[OK] RotorApexOffsetPos already fixed")
else:
    print(f"[WARN] InflowWind template not found at {_iw_path}")


# ============================================================================
# 2. FastFarm environment with default yaw control
# ============================================================================
print("\n--- Default Yaw Control ---")
ff_env = envs.make("Dec_DafengH1_Fastfarm", max_num_steps=args.steps, t_init=args.t_init)
print(f"Turbines: {ff_env.num_turbines}")
print(f"Default controls: {ff_env.controls}")

ff_env.reset()
print(f"Agents: {ff_env.agents}")
print("FAST.Farm spawned!")


def step_policy(i, j):
    """Simple yaw policy: yaw turbine 0 at step 30, others always 0."""
    return {"yaw": np.float32(-10.0)} if (i == 30 and j == 0) else {"yaw": np.float32(0.0)}


rewards = multi_agent_step_routine(ff_env, step_policy)
env.close()
print(f"Rewards per turbine: {rewards}")
print(f"Total reward (sum): {sum(rewards.values()):.2f}")

# --- Plot results ---
columns = [f"T{i+1}" for i in range(ff_env.num_turbines)]
yaws = np.c_[[[h["yaw"] for h in ff_env.history[agent]["observation"]]
              for agent in ff_env.possible_agents]].T
powers = np.c_[[ff_env.history[agent]["power"] for agent in ff_env.possible_agents]].T
yaws_df = pd.DataFrame(yaws, columns=columns)
powers_df = pd.DataFrame(powers, columns=columns)

fig, axes = plt.subplots(ncols=2, figsize=(15, 5))
sns.lineplot(yaws_df, ax=axes[0])
axes[0].set(ylabel="Yaw (°)", xlabel="Iterations", title="Yaw Angles per Turbine")
sns.lineplot(powers_df.sum(1), ax=axes[1])
axes[1].set(ylabel="Power (MW)", xlabel="Iterations", title="Total Farm Power Output")

fig.savefig(os.path.join(_output_dir, "default_yaw_control.png"), dpi=150, bbox_inches="tight")
plt.close(fig)
print(f"✓ Plot saved to {_output_dir}\\default_yaw_control.png")


# ============================================================================
# 3. Custom controls: yaw + pitch
# ============================================================================
print("\n--- Custom Controls: Yaw + Pitch ---")
controls = {"yaw": (-20, 20, 15), "pitch": (0, 45, 1)}
ff_env2 = envs.make("Dec_DafengH1_Fastfarm", max_num_steps=args.steps, controls=controls, t_init=args.t_init)
ff_env2.reset()
print(f"Custom controls: {ff_env2.controls}")
print(f"Action space (agent 0): {ff_env2.action_space(ff_env2.possible_agents[0])}")
print(f"Observation space (agent 0): {ff_env2.observation_space(ff_env2.possible_agents[0])}")


def step_policy_pitch(i, j):
    """Policy with yaw and pitch control."""
    return (
        {"yaw": np.float32(-5.0), "pitch": np.float32(2.0)}
        if i == 5
        else {"yaw": np.float32(0.0), "pitch": np.float32(0.0)}
    )


rewards2 = multi_agent_step_routine(ff_env2, step_policy_pitch)
print(f"Total reward (yaw+pitch): {sum(rewards2.values()):.2f}")

print("\n" + "=" * 60)
print("Simulation complete!")
print("=" * 60)

import subprocess
subprocess.run(["taskkill", "/f", "/im", "FAST.Farm_x64_OMP.exe"], capture_output=True)
