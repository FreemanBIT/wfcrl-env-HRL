"""M9 — Control-mode interface（3 模式，双通道：偏航动作指令 + 诱导因子）。

REFACTOR_PLAN 第 4 部分：新架构 FCR 仅两条控制通道（yaw_delta / induction_ref），
原 TORQUE/PITCH 通道（文件桥时代）删除。控制模式收敛为：

    mode 1  YAW           仅偏航（a=1/3 贪婪）
    mode 2  INDUCTION     仅诱导（yaw 对齐来流）
    mode 3  YAW_INDUCTION 偏航 + 诱导（推荐主模式）

控制器（A/B/C）查询 ControlModeSpec 决定优化变量与命令组装。
"""
from __future__ import annotations

from enum import IntEnum

from .types import Cmd


A_GREEDY = 1.0 / 3.0
A_MIN = 0.0
A_MAX = 1.0 / 3.0
YAW_MAX = 30.0


class ControlMode(IntEnum):
    YAW = 1
    INDUCTION = 2
    YAW_INDUCTION = 3


_DECISION_VARS = {
    ControlMode.YAW:           ["yaw"],
    ControlMode.INDUCTION:     ["a"],
    ControlMode.YAW_INDUCTION: ["yaw", "a"],
}


class ControlModeSpec:
    """从 :class:`ControlMode` 派生决策变量、默认值、边界与命令组装。"""

    def __init__(self, mode: ControlMode):
        self.mode = ControlMode(mode)

    def decision_vars(self) -> list:
        return list(_DECISION_VARS[self.mode])

    def optimizes_yaw(self) -> bool:
        return "yaw" in _DECISION_VARS[self.mode]

    def optimizes_induction(self) -> bool:
        return "a" in _DECISION_VARS[self.mode]

    def fixed_defaults(self) -> dict:
        d: dict = {}
        if not self.optimizes_yaw():
            d["yaw"] = 0.0
        if not self.optimizes_induction():
            d["a"] = A_GREEDY
        return d

    def bounds(self) -> dict:
        b: dict = {}
        if self.optimizes_yaw():
            b["yaw"] = (-YAW_MAX, YAW_MAX)
        if self.optimizes_induction():
            b["a"] = (A_MIN, A_MAX)
        return b

    # -- 命令组装（双通道：绝对偏航 + 诱导因子）--------------------
    def to_controls(self, turbine_id: int, yaw_deg: float, a: float,
                    U_eff: float, rho: float = 1.225) -> Cmd:
        """由（已限幅的）优化解 (yaw_deg, a) 组装单机 Cmd。

        输出字段：yaw_deg（绝对机舱方位 deg，桥内换算为动作指令）与
        induction_a（无量纲诱导因子）。YAW_INDUCTION 为推荐双通道模式。
        """
        cmd = Cmd(turbine_id=turbine_id)
        cmd.yaw_deg = float(yaw_deg) if self.optimizes_yaw() else 0.0
        if self.optimizes_induction():
            cmd.induction_a = float(a)
        return cmd


def greedy_cmd(turbine_id: int, U_eff: float, rho: float = 1.225) -> Cmd:
    """贪婪基线命令：yaw=0（对齐来流），a=1/3（不降载）。

    新架构下不设功率目标字段——greedy 即“不下发诱导命令”（Runtime 默认 a=1/3）。
    """
    return Cmd(turbine_id=turbine_id, yaw_deg=0.0, induction_a=A_GREEDY)


__all__ = ["ControlMode", "ControlModeSpec", "greedy_cmd",
           "A_GREEDY", "A_MIN", "A_MAX", "YAW_MAX"]
