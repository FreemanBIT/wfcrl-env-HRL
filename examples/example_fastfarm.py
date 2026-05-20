import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import numpy as np

from wfcrl import environments as envs
from wfcrl.rewards import StepPercentage

env = envs.make(
    "Dec_DafengH1_Fastfarm",
    max_num_steps=2, #100
    controls=["yaw", "pitch"],
    reward_shaper=StepPercentage(),
    load_coef=1,
    # UNCOMMENT TO ADD CUSTOM PATH TO FAST.Farm
    # path_to_simulator="path_to_simulator"
)


def dummy_policy(agent, i):
    if (agent == "turbine_1") and (i == 20):
        return {
            "yaw": np.array([15.0]),
            "pitch": np.array([3.0]),
        }
    return {"yaw": np.array([0]), "pitch": np.array([0.0])}


env.reset()
r = {agent: 0.0 for agent in env.possible_agents}
num_steps = {agent: 0 for agent in env.possible_agents}
loads = {agent: 0.0 for agent in env.possible_agents}
powers = {agent: 0.0 for agent in env.possible_agents}

for agent in env.agent_iter():
    observation, reward, termination, truncation, info = env.last()
    r[agent] += reward
    if termination or truncation:
        action = None
    else:
        action = dummy_policy(agent, num_steps[agent])
        num_steps[agent] += 1
        loads[agent] += float(np.mean(np.abs(info["load"]))) if "load" in info else 0.0
        powers[agent] += float(info["power"]) if "power" in info else 0.0
    env.step(action)

print(f"\nTotal reward = {r}\n")
print(f"Powers = {powers}\n")
print(f"Loads = {loads}\n")
