"""
WFCRL MDP 桥接层
===============
提供 WindFarmMDP — RL 动作/状态空间定义与仿真器之间的桥接。

职责
----
1.  RL 动作空间 / 状态空间定义
2.  RL action → ControlInput 转换
3.  SimulationOutput → RL observation 转换
4.  动作累积追踪（配合 ActuatorConstraints）

使用惰性导入避免与 wfcrl.environments → wfcrl.multiagent_env 之间的循环依赖。
"""

__all__ = [
    "WindFarmMDP",
    "clip_to_dict_space",
]


def __getattr__(name):
    """惰性加载，避免循环导入。"""
    if name in __all__:
        import wfcrl.mdp.mdp as _mod
        return getattr(_mod, name)
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")


def __dir__():
    return __all__
