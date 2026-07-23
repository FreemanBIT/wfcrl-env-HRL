"""
自定义控制器加载器
==================
从 custom_controllers/ 目录安全加载用户编写的控制器插件。
"""

import os
import sys
import importlib
import inspect
import traceback
from pathlib import Path
from typing import Optional

# 项目根目录
PROJECT_ROOT = Path(__file__).resolve().parent.parent
CUSTOM_DIR = PROJECT_ROOT / "custom_controllers"

# 确保项目根目录在 sys.path 中（Streamlit 环境下有时会缺失）
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

# 提前导入 WFCRL 基类，带错误处理
try:
    from wfcrl.controllers import (
        WindFarmController as _WFC,
        ControlIntent as _CI,
        ControllerContext as _CC,
        ControllerObservation as _CO,
    )
    _HAVE_WFCRL = True
except Exception as _e:
    _HAVE_WFCRL = False
    _WFC = type("WindFarmController", (), {"__doc__": "Placeholder - wfcrl not available"})
    _CI = type("ControlIntent", (), {})
    _CC = type("ControllerContext", (), {})
    _CO = type("ControllerObservation", (), {})
BUILTIN_REGISTRY = {
    "greedy": "贪婪基线 — 零偏航、全额发电",
    "fixed_yaw": "固定偏航 — 预设偏航角控制",
    "fixed_derating": "固定降额 — 预设降额比例控制",
    "fastfarm_yaw": "FAST.Farm偏航 — 上游风机主动偏航(-17°)，专为FAST.Farm设计",
}


def get_custom_controllers():
    """扫描 custom_controllers/ 目录，返回所有自定义控制器类。"""
    if not CUSTOM_DIR.exists():
        return []
    if not _HAVE_WFCRL:
        return []  # wfcrl 不可用，返回空列表

    import numpy as np

    controllers = []

    for fpath in sorted(CUSTOM_DIR.glob("*.py")):
        if fpath.name == "__init__.py":
            continue

        try:
            source = fpath.read_text(encoding="utf-8")
        except Exception as e:
            controllers.append({"name": f"(读文件失败) {fpath.name}", "filename": fpath.name,
                                "description": str(e), "source": "", "class": None, "error": str(e)})
            continue

        # 快速跳过不含 WindFarmController 的文件
        if "WindFarmController" not in source:
            continue

        try:
            # 在受控命名空间中执行
            namespace = {
                "__name__": f"ctrl_{fpath.stem}",
                "__file__": str(fpath),
                "__builtins__": __builtins__,
                "WindFarmController": _WFC,
                "ControlIntent": _CI,
                "ControllerContext": _CC,
                "ControllerObservation": _CO,
                "np": np,
                "np_array": np.array,
                "np_zeros": np.zeros,
            }
            exec(compile(source, str(fpath), "exec"), namespace)

            # 查找 WindFarmController 子类
            for name, obj in namespace.items():
                if not isinstance(obj, type) or name == "WindFarmController":
                    continue
                if issubclass(obj, _WFC) and obj is not _WFC:
                    desc = (obj.__doc__ or "").strip().split("\n")[0]
                    try:
                        class_source = inspect.getsource(obj)
                    except Exception:
                        class_source = "(无法获取源码)"

                    controllers.append({
                        "name": name,
                        "filename": fpath.name,
                        "description": desc,
                        "source": class_source,
                        "class": obj,
                        "error": None,
                    })

        except Exception as e:
            controllers.append({
                "name": f"(加载失败) {fpath.name}",
                "filename": fpath.name,
                "description": f"❌ {type(e).__name__}: {e}",
                "source": "",
                "class": None,
                "error": traceback.format_exc(),
            })

    return controllers


def get_controller_by_name(name: str):
    """按控制器名称查找控制器类。

    优先查找自定义控制器，再查内置控制器。
    """
    # 先在自定义控制器中查找
    for ctrl in get_custom_controllers():
        if ctrl["name"] == name and ctrl["class"] is not None:
            return ctrl["class"]
    return None


def get_all_controllers():
    """获取所有可用控制器（内置 + 自定义）。"""
    controllers = []

    # 内置控制器
    for key, desc in BUILTIN_REGISTRY.items():
        controllers.append({
            "name": key,
            "type": "builtin",
            "description": desc,
            "filename": "",
            "source": "",
            "class": None,
            "error": None,
        })

    # 自定义控制器
    for ctrl in get_custom_controllers():
        ctrl["type"] = "custom"
        controllers.append(ctrl)

    return controllers


def remove_controller(filename: str) -> bool:
    """删除自定义控制器文件。"""
    fpath = CUSTOM_DIR / filename
    if fpath.exists() and fpath.suffix == ".py":
        fpath.unlink()
        return True
    return False


def save_controller(filename: str, content: str) -> Optional[str]:
    """保存控制器文件。返回 None 成功，返回 str 为错误信息。"""
    if not filename.endswith(".py"):
        filename += ".py"
    # 安全检查：只允许在 custom_controllers 目录中
    fpath = CUSTOM_DIR / filename
    try:
        fpath.write_text(content, encoding="utf-8")
        return None
    except Exception as e:
        return str(e)
