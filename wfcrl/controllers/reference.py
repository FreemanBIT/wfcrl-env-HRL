"""Small reference controllers for integration tests and examples."""

from __future__ import annotations

from typing import Optional

import numpy as np

from .base import WindFarmController
from .types import ControlIntent, ControllerObservation


class GreedyController(WindFarmController):
    def compute(self, observation: Optional[ControllerObservation]) -> ControlIntent:
        return ControlIntent(yaw_misalignment_deg=np.zeros(self.context.n_turbines))


class FixedYawController(WindFarmController):
    def __init__(self, yaw_misalignment_deg):
        self.yaw = np.atleast_1d(np.asarray(yaw_misalignment_deg, dtype=float))

    def compute(self, observation: Optional[ControllerObservation]) -> ControlIntent:
        return ControlIntent(yaw_misalignment_deg=self.yaw)


class FixedDeratingController(WindFarmController):
    def __init__(self, derating_ratio, yaw_misalignment_deg=None):
        self.ratio = np.atleast_1d(np.asarray(derating_ratio, dtype=float))
        self.yaw = yaw_misalignment_deg

    def compute(self, observation: Optional[ControllerObservation]) -> ControlIntent:
        return ControlIntent(
            derating_ratio=self.ratio,
            yaw_misalignment_deg=self.yaw,
        )


class FastFarmYawController(WindFarmController):
    """FAST.Farm 偏航控制器 — 上游风机主动偏航偏转尾流。

    自动识别上游风机（x坐标最小的行），施加固定偏航角。
    下游风机保持零偏航。专为 FAST.Farm 6机2行布局设计。

    用法:
        ctrl = FastFarmYawController(yaw_angle=-17.0)
    """

    def __init__(self, yaw_angle: float = -17.0):
        """
        参数:
            yaw_angle: 上游风机偏航角 (°)，负值=尾流向下偏转
        """
        self.yaw_angle = float(yaw_angle)
        self._n = 0
        self._upstream_mask = None

    def reset(self, context):
        """实验开始时确定上游风机。"""
        super().reset(context)
        self._n = context.n_turbines

        # 通过 capabilities 获取布局信息（如果有）
        # 默认：假设6机2行布局，x最小的为上游
        if hasattr(context, '_layout_x') and context._layout_x is not None:
            xs = np.array(context._layout_x)
        else:
            # 默认 6T 布局
            xs = np.array([0.0, 504.0, 1008.0, 0.0, 504.0, 1008.0])[:self._n]

        min_x = xs.min()
        self._upstream_mask = (xs == min_x)

    def compute(self, observation: Optional[ControllerObservation]) -> ControlIntent:
        yaw = np.zeros(self._n)
        if self._upstream_mask is not None:
            yaw[self._upstream_mask] = self.yaw_angle
        return ControlIntent(yaw_misalignment_deg=yaw)

