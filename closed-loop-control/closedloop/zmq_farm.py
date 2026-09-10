"""zmq_farm.py — 新架构（ZeroMQ 场控）的统一入口模块。

REFACTOR_PLAN 第 3 部分：把分散在 bridge.py / runner.py / zmq_pipeline.py 的
ZMQ 相关对象在此聚合，供外部脚本（examples、研究回放）以单一入口使用：

    from closedloop.zmq_farm import launch_farm, ZmqFarmBridge, ZmqPlant

实现定义仍在各自源文件（行为不变），本模块只做命名空间收敛。
"""
from __future__ import annotations

from .bridge import ZmqFarmBridge, FastFarmZmqPlant
from .runner import ZmqPlant
from .zmq_pipeline import launch_farm, build_farm_case, resolve_layout, wait_first_state

__all__ = [
    "ZmqFarmBridge",
    "FastFarmZmqPlant",
    "ZmqPlant",
    "launch_farm",
    "build_farm_case",
    "resolve_layout",
    "wait_first_state",
]
