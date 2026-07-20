"""
WindFarmEnv — 集中式风场控制 Gymnasium 环境
============================================
单智能体 RL 环境，使用 Gymnasium API。

职责
----
1.  封装 WindFarmMDP 为 Gymnasium Env
2.  状态/动作空间直接透传 MDP 定义
3.  奖励计算：归一化功率 - 载荷惩罚
4.  执行器约束：基于累计执行时间裁剪动作
"""

from __future__ import annotations

import copy
from typing import Dict, Optional

import gymnasium as gym
import numpy as np

from wfcrl.envs.base import BaseWindFarmEnv
from wfcrl.envs.rewards import DoNothingReward, RewardShaper
from wfcrl.compat_data import FarmCase
from wfcrl.engine.base import SimulatorInterface
from wfcrl.mdp.mdp import WindFarmMDP


class WindFarmEnv(BaseWindFarmEnv, gym.Env):
    """
    集中式风场控制环境 (Gymnasium API)。

    Parameters
    ----------
    interface : SimulatorInterface
        仿真器接口实例。
    farm_case : FarmCase
        风场用例描述（布局、时间步长等）。
    controls : dict
        控制配置字典，如 {"yaw": (-20, 20, 5), "pitch": (0, 45, 1)}。
    continuous_control : bool
        是否使用连续控制空间。默认 True。
    reward_shaper : RewardShaper
        奖励塑形函数。默认 DoNothingReward。
    start_iter : int
        初始预热步数。默认 0。
    max_num_steps : int
        每个 episode 的最大步数。默认 500。
    load_coef : float
        载荷惩罚系数。默认 0.1。
    """

    metadata = {"name": "centralized-windfarm"}

    def __init__(
        self,
        interface: SimulatorInterface,
        farm_case: FarmCase,
        controls: dict,
        continuous_control: bool = True,
        reward_shaper: RewardShaper = DoNothingReward(),
        start_iter: int = 0,
        max_num_steps: int = 500,
        load_coef: float = 0.1,
    ):
        self.mdp = WindFarmMDP(
            interface=interface,
            farm_case=farm_case,
            controls=controls,
            continuous_control=continuous_control,
            start_iter=start_iter,
            horizon=start_iter + max_num_steps,
        )
        self.continuous_control = continuous_control
        self.action_space = self.mdp.action_space
        self.observation_space = self.mdp.state_space
        self._state: Optional[dict] = self.mdp.start_state
        self.num_turbines = self.mdp.num_turbines
        self.max_num_steps = max_num_steps
        self.reward_shaper = reward_shaper
        self.controls = controls
        self.dt = farm_case.dt
        self.farm_case = farm_case
        self.accumulated_actions = self.mdp.get_accumulated_actions()
        self.num_moves = 0
        self.load_coef = load_coef

    # ========== Gymnasium API ==========

    def reset(self, seed=None, options=None):
        """重置环境到初始状态。"""
        self.mdp.reset(seed, options)
        self._state = self.mdp.start_state
        self.reward_shaper.reset()
        observation = copy.deepcopy(self._state)
        self.accumulated_actions = self.mdp.get_accumulated_actions()
        self.num_moves = 0
        return observation

    def step(self, actions: Dict):
        """
        执行一步动作。

        Parameters
        ----------
        actions : dict
            动作字典，每个值为 shape (num_turbines,) 的 np.ndarray。

        Returns
        -------
        observation, reward, terminated, truncated, info
        """
        assert self._state is not None, "Call reset before `step`"

        self.num_moves += 1

        # 执行器约束检查
        actions = self.clamp_actions_by_actuation(
            actions, self.accumulated_actions, self.num_moves, self.farm_case.dt,
        )

        next_state, powers, loads, truncated = self.mdp.take_action(
            self._state, actions
        )

        # 奖励计算
        freewind_speed = self._state["freewind_measurements"][0]
        reward = self.compute_reward(powers, loads, freewind_speed)

        self._state = next_state
        terminated = False
        truncated = truncated
        info = {"power": powers}
        if loads is not None:
            info["load"] = loads
        observation = copy.deepcopy(self._state)

        # 累计动作追踪
        self.accumulated_actions = self.mdp.get_accumulated_actions()

        return observation, reward, terminated, truncated, info

    def close(self):
        """清理仿真器资源。"""
        self.close_simulator()
