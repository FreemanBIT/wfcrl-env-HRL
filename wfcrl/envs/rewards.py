"""
奖励塑形工具
============
定义 RewardShaper ABC 及其具体实现：

- DoNothingReward     : 透传原始奖励，不做任何修改
- ReferencePercentage : (reward - reference) / reference
- StepPercentage      : 相对于上一步的百分比变化
"""

from abc import ABC, abstractmethod


class RewardShaper(ABC):
    """奖励塑形抽象基类。"""

    @abstractmethod
    def __call__(self, reward: float):
        pass

    def update(self):
        pass

    def reset(self):
        pass


class DoNothingReward(RewardShaper):
    """
    Dummy class. Returns the same reward.
    """

    def __call__(self, reward):
        return reward


class ReferencePercentage(RewardShaper):
    def __init__(self, reference: float):
        self.reference = reference

    def __call__(self, reward):
        return (reward - self.reference) / self.reference


class StepPercentage(RewardShaper):
    def __init__(self, reference: float = 0.0):
        self.reference = reference

    def __call__(self, reward):
        if self.reference == 0:
            shaped_reward = 0.0
        else:
            shaped_reward = (reward - self.reference) / self.reference
        self.reference = reward
        return shaped_reward

    def reset(self, reference: float = 0.0):
        self.reference = reference
