"""
BaseWindFarmEnv — 风场控制 RL 环境公共基类
==========================================
提取 WindFarmEnv (Gymnasium) 和 MAWindFarmEnv (PettingZoo) 的公共逻辑：

1.  奖励计算：归一化功率 + 载荷惩罚
2.  执行器约束检查：根据 ACTUATORS_RATE 限制动作频率
3.  通用属性存储

用法
----
class WindFarmEnv(BaseWindFarmEnv, gym.Env):
    ...
"""

from __future__ import annotations

from typing import Dict, Optional

import numpy as np

from wfcrl.mdp import WindFarmMDP
from wfcrl.envs.rewards import DoNothingReward, RewardShaper


class BaseWindFarmEnv:
    """
    风场控制环境的混入基类。

    子类必须定义 / 注入以下属性：
    - self.mdp : WindFarmMDP
    - self.reward_shaper : RewardShaper
    - self.load_coef : float
    - self.num_moves : int
    - self.farm_case : FarmCase（提供 .dt）
    """

    # 子类需覆盖或 __init__ 中设置
    mdp: WindFarmMDP
    reward_shaper: RewardShaper
    load_coef: float
    num_moves: int
    controls: dict
    num_turbines: int
    continuous_control: bool

    # ========== 奖励计算 ==========

    def compute_reward(
        self,
        powers: np.ndarray,
        loads: Optional[np.ndarray],
        freewind_speed: float,
    ) -> np.ndarray:
        """
        计算归一化奖励。

        Parameters
        ----------
        powers : np.ndarray
            shape (num_turbines,)，各风机功率 (MW)。
        loads : np.ndarray or None
            shape (num_turbines, 3) 或 None，叶根载荷。
        freewind_speed : float
            自由流风速 (m/s)。

        Returns
        -------
        np.ndarray
            shape (1,) 的奖励标量。
        """
        normalized_powers = powers * 1e3 / (freewind_speed ** 3)
        load_penalty = 0.0
        if loads is not None:
            load_penalty = float(np.mean(np.abs(loads)))
        reward = float(normalized_powers.mean()) - self.load_coef * load_penalty
        return np.array([self.reward_shaper(reward)])

    # ========== 执行器约束 ==========

    def clamp_actions_by_actuation(
        self,
        actions: Dict[str, np.ndarray],
        accumulated: Dict[str, np.ndarray],
        num_moves: int,
        dt: float,
    ) -> Dict[str, np.ndarray]:
        """
        根据历史执行时间占比裁剪动作。

        当某控制维度的累计执行时间占比 >= 10% 时，将该维度的动作置零。

        Parameters
        ----------
        actions : dict
            动作字典 {control_name: np.ndarray}。
        accumulated : dict
            累计执行量 {control_name: np.ndarray}。
        num_moves : int
            当前已执行步数。
        dt : float
            仿真步长 (s)。

        Returns
        -------
        dict
            裁剪后的动作字典（原地修改）。
        """
        for control in actions:
            if control not in WindFarmMDP.ACTUATORS_RATE:
                continue
            actuating_time = (
                accumulated[control] / WindFarmMDP.ACTUATORS_RATE[control]
            )
            actuating_frac = actuating_time / num_moves / dt
            action_val = np.asarray(actions[control])
            if action_val.ndim >= 1:
                action_val[actuating_frac >= 0.1] = 0.0
            elif actuating_frac >= 0.1:
                action_val = np.float32(0.0)
            actions[control] = action_val
        return actions

    # ========== 资源清理 ==========

    def close_simulator(self) -> None:
        """清理仿真器 MPI 资源（如果有）。"""
        if (
            hasattr(self, 'mdp')
            and hasattr(self.mdp, 'interface')
            and hasattr(self.mdp.interface, '_finalize_mpi_comm')
        ):
            self.mdp.interface._finalize_mpi_comm()
