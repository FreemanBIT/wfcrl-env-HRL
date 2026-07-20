"""
Gymnasium / PettingZoo Wrapper
===============================
提供日志记录和仿真器随机化包装器。
"""

from gymnasium import Env, Wrapper
from pettingzoo import AECEnv
from pettingzoo.utils.wrappers import BaseWrapper


class RandomSimulator(BaseWrapper):
    """
    PettingZoo AEC wrapper — 在 reset 时对仿真器参数进行随机化采样。
    """

    def __init__(self, env: AECEnv):
        super().__init__(env)

        # Retrieve variables
        self.continuous_control = self.env.continuous_control
        self.max_num_steps = self.env.max_num_steps
        self._state = self.env.mdp.start_state
        self.num_turbines = self.env.mdp.num_turbines
        self.mdp = self.env.mdp
        self.controls = self.env.controls
        # get_parameters() not yet implemented on new SimulatorInterface
        self.parameters_vector = getattr(self.env.mdp.interface, 'get_parameters', lambda: None)()

    def reset(self, seed=None, options=None):
        # sample_parameters() not yet implemented on new SimulatorInterface
        if hasattr(self.env.mdp.interface, 'sample_parameters'):
            self.parameters_vector = self.env.mdp.interface.sample_parameters()
        self.env.reset(seed, options)


class AECLogWrapper(BaseWrapper):
    """
    PettingZoo AEC wrapper — 记录 observation / reward / load / power 历史。
    """

    def __init__(self, env: AECEnv):
        super().__init__(env)

        # Init empty lists to store history
        self.history = {
            agent: {"observation": [], "reward": [], "load": [], "power": []}
            for agent in self.env.possible_agents
        }

        # Retrieve variables
        self.continuous_control = self.env.continuous_control
        self.max_num_steps = self.env.max_num_steps
        self._state = self.env.mdp.start_state
        self.num_turbines = self.env.mdp.num_turbines
        self.mdp = self.env.mdp
        self.controls = self.env.controls

    def last(self):
        agent = self.env.agent_selection
        observation, reward, termination, truncation, info = self.env.last()
        self.history[agent]["observation"].append(observation)
        self.history[agent]["reward"].append(reward)
        if "power" in info:
            self.history[agent]["power"].append(info["power"])
        if "load" in info:
            self.history[agent]["load"].append(info["load"])
        return observation, reward, termination, truncation, info

    def reset(self, seed=None, options=None):
        self.history = {
            agent: {"observation": [], "reward": [], "load": [], "power": []}
            for agent in self.env.possible_agents
        }
        return self.env.reset(seed, options)


class LogWrapper(Wrapper):
    """
    Gymnasium wrapper — 记录 observation / reward / load / power 历史。
    """

    def __init__(self, env: Env):
        super().__init__(env)

        # Init empty lists to store history
        self.history = {"observation": [], "reward": [], "load": [], "power": []}

        # Retrieve variables
        self.continuous_control = self.env.continuous_control
        self.max_num_steps = self.env.max_num_steps
        self._state = self.env.mdp.start_state
        self.num_turbines = self.env.mdp.num_turbines
        self.mdp = self.env.mdp
        self.controls = self.env.controls

    def step(self, action):
        observation, reward, terminated, truncated, info = self.env.step(action)
        self.history["observation"].append(observation)
        self.history["reward"].append(reward)
        if "power" in info:
            self.history["power"].append(info["power"])
        if "load" in info:
            self.history["load"].append(info["load"])
        return observation, reward, terminated, truncated, info

    def reset(self, seed=None, options=None):
        self.history = {"observation": [], "reward": [], "load": [], "power": []}
        return self.env.reset(seed, options)
