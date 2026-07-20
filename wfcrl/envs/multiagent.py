"""
MAWindFarmEnv — 多智能体风场控制 PettingZoo AEC 环境
======================================================
基于 PettingZoo AECEnv 的多智能体 RL 环境。

职责
----
1.  每个风机作为一个独立智能体
2.  合作式奖励（所有智能体共享全局奖励）
3.  支持连续/离散动作空间
4.  执行器约束：基于累计执行时间裁剪动作
5.  遵循 PettingZoo AEC 协议
"""

from __future__ import annotations

import functools
from collections import OrderedDict
from typing import Dict, Optional

import numpy as np
from gymnasium import spaces
from pettingzoo import AECEnv
from pettingzoo.utils import agent_selector

from wfcrl.envs.base import BaseWindFarmEnv
from wfcrl.envs.rewards import DoNothingReward, RewardShaper
from wfcrl.compat_data import FarmCase
from wfcrl.engine.base import SimulatorInterface
from wfcrl.mdp.mdp import WindFarmMDP


class MAWindFarmEnv(BaseWindFarmEnv, AECEnv):
    """
    多智能体风场控制环境 (PettingZoo AEC API)。

    Parameters
    ----------
    interface : SimulatorInterface
        仿真器接口实例。
    farm_case : FarmCase
        风场用例描述（布局、时间步长等）。
    controls : dict
        控制配置字典。
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

    metadata = {
        "name": "multiagent-windfarm",
        "is_parallelizable": True,
    }

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
        self.max_num_steps = max_num_steps
        self._state: Optional[dict] = None
        self.num_turbines = self.mdp.num_turbines
        self.reward_shaper = reward_shaper
        self.controls = controls
        self.farm_case = farm_case
        self.state_space = self.mdp.state_space
        self.load_coef = load_coef

        # Init AEC properties
        self.possible_agents = [
            "turbine_" + str(r + 1) for r in range(self.num_turbines)
        ]
        self.agent_name_mapping = dict(
            zip(self.possible_agents, list(range(len(self.possible_agents))))
        )
        self._build_agent_spaces()

    # ========== PettingZoo AEC API ==========

    @functools.lru_cache(maxsize=None)
    def observation_space(self, agent):
        return self._obs_spaces[agent]

    @functools.lru_cache(maxsize=None)
    def action_space(self, agent):
        return self._action_spaces[agent]

    def state(self):
        return self._state

    def _build_agent_spaces(self):
        """构建每个智能体的观察和动作空间。"""
        self._obs_spaces = {}
        self._action_spaces = {}
        for i, agent in enumerate(self.possible_agents):
            self._obs_spaces[agent] = {
                key: spaces.Box(space.low[i], space.high[i])
                for key, space in self.mdp.state_space.items()
                if key != "freewind_measurements"
            }
            if self.continuous_control:
                self._action_spaces[agent] = {
                    key: spaces.Box(space.low[i], space.high[i])
                    for key, space in self.mdp.action_space.items()
                }
            else:
                self._action_spaces[agent] = {
                    key: space[i] for key, space in self.mdp.action_space.items()
                }

    def observe(self, agent):
        """
        返回指定智能体的局部观察。

        局部观察 = 全局状态中移去 freewind_measurements 后，
        每个数组取对应该智能体的元素。
        """
        global_state = self.state()
        agent_state = OrderedDict()
        for key, partial_state in global_state.items():
            if key != "freewind_measurements":
                agent_state[key] = partial_state[self.agent_name_mapping[agent]]
        return agent_state

    def reset(self, seed=None, options=None):
        """
        重置环境。

        设置 AEC 协议要求的 agents / rewards / dones / infos / agent_selection。
        """
        self.mdp.reset(seed, options)
        self._state = self.mdp.start_state
        self.reward_shaper.reset()

        self.agents = self.possible_agents[:]
        self._num_steps = {agent: 0 for agent in self.agents}
        self.rewards = {agent: np.array([0.0]) for agent in self.agents}
        self._cumulative_rewards = {agent: np.array([0.0]) for agent in self.agents}
        self.terminations = {agent: False for agent in self.agents}
        self.truncations = {agent: False for agent in self.agents}
        self.infos = {agent: {} for agent in self.agents}
        self.actions = {agent: None for agent in self.agents}
        self.observations = {agent: self.observe(agent) for agent in self.agents}
        self.constrained = {agent: self.observe(agent) for agent in self.agents}
        accumulated_actions = self.mdp.get_accumulated_actions()
        self.accumulated_actions = {
            agent: {
                control: accumulated_actions[control][id_agent]
                for control in accumulated_actions
            }
            for id_agent, agent in enumerate(self.agents)
        }
        self.num_moves = 0
        self._agent_selector = agent_selector(self.agents)
        self.agent_selection = self._agent_selector.next()

    def step(self, action):
        """
        AEC step：当前智能体提交动作。
        当所有智能体都提交后，执行一步仿真并分配奖励。
        """
        assert self._state is not None, "Call reset before `step`"

        agent = self.agent_selection

        if self.truncations[agent] or self.terminations[agent]:
            self._was_dead_step(action)
            return

        self._num_steps[agent] += 1

        # 验证动作完整性
        for control in action:
            if control not in self.mdp.controls:
                raise ValueError(
                    f"Control `{control}` for agent {agent} is not activated."
                    f" List of activated controls: {list(self.mdp.controls.keys())}"
                )
        if any([not (control in action) for control in self.mdp.controls]):
            raise ValueError(
                f"Action {action} for agent {agent} is incomplete."
                f" List of needed controls: {self.mdp.controls.keys()}"
            )

        # 执行器约束检查（agent 级别）
        agent_accumulator = self.accumulated_actions[agent]
        for control in action:
            if control not in WindFarmMDP.ACTUATORS_RATE:
                continue
            actuating_time = (
                agent_accumulator[control] / WindFarmMDP.ACTUATORS_RATE[control]
            )
            actuating_frac = actuating_time / self._num_steps[agent] / self.farm_case.dt
            if actuating_frac >= 0.1:
                if isinstance(action[control], np.ndarray) and action[control].ndim >= 1:
                    action[control][:] = 0.0
                else:
                    action[control] = np.float32(0.0)

        # 重启奖励累积
        self._cumulative_rewards[agent] = 0
        self.actions[self.agent_selection] = action

        # 所有智能体提交完毕时，执行一步仿真
        if self._agent_selector.is_last():
            joint_action = self._join_actions(self.actions)
            next_state, powers, loads, truncated = self.mdp.take_action(
                self._state, joint_action
            )

            # 全局奖励计算
            freewind_speed = self.state()["freewind_measurements"][0]
            reward = self.compute_reward(powers, loads, freewind_speed)

            self._state = next_state
            for ag in self.agents:
                if loads is not None:
                    self.infos[ag]["load"] = loads[self.agent_name_mapping[ag]]
                self.rewards[ag] = reward
                self.observations[ag] = self.observe(ag)
                self.truncations[ag] = truncated
                self.terminations[ag] = False
                self.infos[ag]["power"] = powers[self.agent_name_mapping[ag]]

            self.num_moves += 1
            if self.num_moves >= self.max_num_steps:
                for a in self.agents:
                    self.truncations[a] = True
        else:
            self._clear_rewards()

        # 更新动作累积
        accumulator = self.mdp.get_accumulated_actions()
        for control in action:
            acc = accumulator[control][self.agent_name_mapping[agent]]
            self.accumulated_actions[agent][control] = acc

        self.agent_selection = self._agent_selector.next()
        self._accumulate_rewards()

    # ========== 内部辅助 ==========

    def _join_actions(self, agent_actions: dict) -> dict:
        """将各智能体的动作合并为联合动作字典。"""
        joint_action = {
            control: np.zeros(self.num_turbines, dtype=np.float32)
            for control in self.mdp.controls
        }
        for j, (ag, action) in enumerate(agent_actions.items()):
            for control in action:
                val = action[control]
                if isinstance(val, np.ndarray):
                    joint_action[control][j] = val.item() if val.ndim >= 1 else val
                else:
                    joint_action[control][j] = val
        return joint_action

    def close(self):
        """清理仿真器资源。"""
        self.close_simulator()
