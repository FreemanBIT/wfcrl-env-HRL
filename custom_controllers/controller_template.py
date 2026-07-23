"""
自定义控制器模板
================
继承 WindFarmController 基类，实现 reset() 和 compute() 两个方法即可。

输入: observation (ControllerObservation)
  - time_s: float             当前仿真时间 (s)
  - power_mw: array[6]        各风机功率 (MW)
  - farm_power_mw: float      风场总功率 (MW)
  - wind_speed: array[6]      各风机局部风速 (m/s)
  - yaw_misalignment_deg: array[6]  当前偏航失准角 (°)
  - pitch_deg: array[6]       当前变桨角 (°)

输出: ControlIntent
  - yaw_misalignment_deg: array[6]   偏航失准角 (°)，范围 [-25, 25]
  - power_setpoint_mw: array[6]      功率设定点 (MW)，或 None
  - derating_ratio: array[6]         降额比例 (0, 1]，或 None
  - pitch_deg: array[6]              变桨角 (°)，或 None

约束（平台自动处理，控制器不需要管）:
  - 偏航速率限制、死区、范围
  - 功率变化率、范围
  - 降额比例变化率

使用方式:
  1. 复制此文件，改个名字（如 my_awesome_controller.py）
  2. 实现你的控制逻辑（参考下面的示例）
  3. 在 Dashboard 的「控制器管理」页面上传
  4. 在快速实验中选择你的控制器运行
"""

import numpy as np
from wfcrl.controllers import (
    WindFarmController, ControlIntent, ControllerContext, ControllerObservation,
)


# ══════════════════════════════════════════════════════════════════════════
# 示例 1: 贪婪基线（无偏航、无降额）
# ══════════════════════════════════════════════════════════════════════════

class GreedyController(WindFarmController):
    """贪婪基线控制器——所有风机零偏航、全额发电。

    这是最简单的控制器，用作对比基准。
    """

    def reset(self, context: ControllerContext):
        """实验开始时调用，初始化参数。"""
        self.context = context
        self.n = context.n_turbines
        self.step = 0

    def compute(self, observation):
        """每一步调用，返回控制指令。

        输入 observation 为 None 表示第一步（还没有测量数据）。
        """
        self.step += 1
        return ControlIntent(
            yaw_misalignment_deg=np.zeros(self.n),  # 零偏航
            # 其他参数默认 None = 不控制
        )


# ══════════════════════════════════════════════════════════════════════════
# 示例 2: 固定偏航控制器
# ══════════════════════════════════════════════════════════════════════════

class FixedYawController(WindFarmController):
    """固定偏航控制器——给指定的风机设置固定偏航角。

    演示如何配置每台风机的偏航角。
    """

    def __init__(self, yaw_angles_deg=None):
        """
        参数:
            yaw_angles_deg: list[float] 每台风机的偏航角，默认 T1=10°, 其余 0°
        """
        self.yaw_angles = yaw_angles_deg or [10.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    def reset(self, context: ControllerContext):
        self.context = context
        self.n = context.n_turbines
        # 确保偏航角数组长度匹配风机数
        if len(self.yaw_angles) != self.n:
            self.yaw_angles = self.yaw_angles[:self.n] + [0.0] * (self.n - len(self.yaw_angles))

    def compute(self, observation):
        return ControlIntent(
            yaw_misalignment_deg=np.array(self.yaw_angles),
        )


# ══════════════════════════════════════════════════════════════════════════
# 示例 3: 尾流感知偏航控制器（进阶）
# ══════════════════════════════════════════════════════════════════════════

class WakeAwareYawController(WindFarmController):
    """尾流感知偏航控制器。

    根据当前风速和风向，动态调整上游风机的偏航角
    以偏转尾流，提升下游风机发电量。

    控制逻辑:
    - 上游风机（x坐标最小的一行）根据风速调节偏航角
    - 风速高时偏航角大，风速低时偏航角小
    - 下游风机保持零偏航
    """

    def __init__(self, max_yaw=20.0):
        self.max_yaw = max_yaw

    def reset(self, context: ControllerContext):
        self.context = context
        self.n = context.n_turbines
        self.step = 0

    def compute(self, observation):
        self.step += 1

        yaw = np.zeros(self.n)

        # 第一步没有观测数据，返回零偏航
        if observation is None:
            return ControlIntent(yaw_misalignment_deg=yaw)

        # 判断哪些是上游风机（这里假设第一行是上游）
        # 实际应用中需要根据风向和布局确定
        upstream = [0, 3]  # T1, T4 (6T布局中的上游风机)

        for i in upstream:
            if i < self.n:
                # 根据风速调节偏航：风速高 → 偏航大
                ws = observation.wind_speed[i] if observation.wind_speed is not None else 8.0
                angle = self.max_yaw * min(ws / 12.0, 1.0)
                # 交替左右偏航使尾流偏向两侧
                if i % 2 == 0:
                    yaw[i] = -angle
                else:
                    yaw[i] = angle

        return ControlIntent(yaw_misalignment_deg=yaw)


# ══════════════════════════════════════════════════════════════════════════
# 示例 4: 降额控制器
# ══════════════════════════════════════════════════════════════════════════

class DeratingController(WindFarmController):
    """降额控制器——降低上游风机功率以减少尾流影响。

    同时使用偏航和降额两种手段。
    """

    def __init__(self, derating=0.8, yaw_deg=5.0):
        self.derating = derating
        self.yaw_deg = yaw_deg

    def reset(self, context: ControllerContext):
        self.context = context
        self.n = context.n_turbines

    def compute(self, observation):
        yaw = np.zeros(self.n)
        derating = np.ones(self.n)

        # 上游风机（6T布局中的T1, T4）
        if self.n >= 1:
            yaw[0] = self.yaw_deg
            derating[0] = self.derating
        if self.n >= 4:
            yaw[3] = self.yaw_deg
            derating[3] = self.derating

        return ControlIntent(
            yaw_misalignment_deg=yaw,
            derating_ratio=derating,
        )


# ══════════════════════════════════════════════════════════════════════════
# 编写你自己的控制器
# ══════════════════════════════════════════════════════════════════════════
#
# 复制这个文件，把上面的 class 改成你的逻辑：
#
# class MyController(WindFarmController):
#     def reset(self, context):
#         self.context = context
#         self.n = context.n_turbines
#         # 你的初始化代码
#
#     def compute(self, observation):
#         # observation 可能为 None（第一步）
#         # 返回 ControlIntent 对象
#         return ControlIntent(
#             yaw_misalignment_deg=np.zeros(self.n),
#         )
#
# ⚠️ 注意事项:
#   - 不要修改文件名中的 class 名称对应关系（上传时按文件名识别）
#   - 所有数值用 float 或 numpy array
#   - compute() 每步调用一次，要足够快（< 0.1s）
#   - 约束（偏航限幅/速率等）由平台自动处理，控制器不用管
#   - 如果不需要控制某个通道，设为 None 即可
