"""
风向与机舱朝向角度换算工具
==========================
统一的风向 ↔ 机舱朝向换算函数集。

关键约定
--------
与 InflowWind PropagationDir = (风向 + 90) % 360 一致：
- 转子对准来流时，机舱绝对朝向 NacYaw = wrap180(wd + 90)
- FLORIS 的偏航角是 "相对来流的失准角"，不是绝对朝向
- FAST.Farm ElastoDyn 的 NacYaw 是绝对机舱朝向，需换算

换算关系
--------
绝对 NacYaw = wrap180(wind_direction + 90 + yaw_misalignment)
相对失准角 = misalignment_from_nacyaw(wind_direction, absolute_nacyaw)
           = wrap180(absolute_nacyaw - (wind_direction + 90))

用法
----
from wfcrl.engine.angle_utils import (
    wrap180,
    nacyaw_face_wind,
    nacyaw_from_misalignment,
    misalignment_from_nacyaw,
)

abs_yaw = nacyaw_from_misalignment(wdir=270, yaw_misalign=10)
# → -170 (即 190° 映射到 (-180, 180])
misalign = misalignment_from_nacyaw(wdir=270, nacyaw_abs=190)
# → 10.0
"""

from __future__ import annotations

from typing import Union

import numpy as np


# =========================================================================
# 角度归一化
# =========================================================================

def wrap180(x: Union[float, np.ndarray]) -> Union[float, np.ndarray]:
    """归一化角度到 (-180, 180]。

    Parameters
    ----------
    x : float or np.ndarray
        输入角度（度）。

    Returns
    -------
    float or np.ndarray
        归一化到 (-180, 180] 的角度。
    """
    return ((float(x) + 180.0) % 360.0) - 180.0


# =========================================================================
# 风向 → 机舱朝向 换算
# =========================================================================

def nacyaw_face_wind(wind_direction_deg: float) -> float:
    """转子对准来流时的绝对机舱朝向（度，∈(-180,180]）。

    与 InflowWind PropagationDir=(wd+90) 约定一致：NacYaw_base = wrap180(wd+90)。

    当前实现返回 0.0（风场旋转法下风始终沿 +X 方向，机舱朝向 0° 即对准来流）。

    Parameters
    ----------
    wind_direction_deg : float
        气象风向（度，0=北风，顺时针）。

    Returns
    -------
    float
        绝对 NacYaw（度，∈(-180,180]）。
    """
    return 0.0


def nacyaw_from_misalignment(
    wind_direction_deg: float,
    yaw_misalign_deg: float,
) -> float:
    """由"相对来流的偏航失准角"换算成绝对 NacYaw（度，∈(-180,180]）。

    FLORIS/LUT 的 yaw 是相对来流的失准角；FAST.Farm/ED 需要绝对机舱朝向。
    风场旋转法下：

        绝对 NacYaw = wrap180(yaw_misalign)

    因为风场已旋转使来流沿 +X，机舱 0°=来流方向，失准角直接等于绝对朝向。

    Parameters
    ----------
    wind_direction_deg : float
        气象风向（度），仅用于保持接口对称，风场旋转法下不影响结果。
    yaw_misalign_deg : float
        相对来流的偏航失准角（度）。

    Returns
    -------
    float
        绝对 NacYaw（度，∈(-180,180]）。
    """
    return wrap180(float(yaw_misalign_deg))


def misalignment_from_nacyaw(
    wind_direction_deg: float,
    nacyaw_abs_deg: float,
) -> float:
    """由绝对 NacYaw 反算"相对来流的偏航失准角"（度，∈(-180,180]）。

    用于把 FAST.Farm 回读的绝对机舱角换算回与 FLORIS 一致的失准角口径。
    风场旋转法下：失准角 = wrap180(绝对 NacYaw)。

    Parameters
    ----------
    wind_direction_deg : float
        气象风向（度），仅用于保持接口对称，风场旋转法下不影响结果。
    nacyaw_abs_deg : float
        绝对 NacYaw（度，∈(-180,180]）。

    Returns
    -------
    float
        相对来流的偏航失准角（度，∈(-180,180]）。
    """
    return wrap180(float(nacyaw_abs_deg))
