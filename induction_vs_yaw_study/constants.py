"""
constants.py — NREL 5MW 物理常数与研究默认参数
================================================
集中存放所有物理常数，禁止在其它模块散落硬编码。

数值来源：
- NREL 5MW 参考机组 (Jonkman et al. 2009, NREL/TP-500-38060)
- DISCON_bridge.f90 内部参数（P_RATED=5.2966e6 等，与本文件保持一致）

注意 FAST.Farm 端 DISCON bridge 使用的额定功率为 5.2966 MW（含传动效率折算后
的发电机额定电功率），与教科书的 5.0 MW 气动额定略有差异。本文件同时提供两者，
转换层默认采用与 bridge 一致的 P_RATED_ELEC，确保 FLORIS 与 FAST.Farm 口径一致。
"""

from __future__ import annotations

import math

# ---- 几何 ----
ROTOR_DIAMETER_M: float = 126.0          # NREL 5MW 风轮直径 D (m)
ROTOR_RADIUS_M: float = ROTOR_DIAMETER_M / 2.0   # 63.0 m
ROTOR_AREA_M2: float = math.pi * ROTOR_RADIUS_M ** 2
HUB_HEIGHT_M: float = 90.0               # 轮毂高度 (m)

# ---- 空气与额定 ----
AIR_DENSITY: float = 1.225               # rho (kg/m^3)，与 FLORIS case.yaml ref_air_density 一致

# 气动额定功率（教科书值）
P_RATED_AERO_W: float = 5.0e6            # 5.0 MW
# 发电机额定电功率（与 DISCON_bridge.f90 P_RATED 一致）
P_RATED_ELEC_W: float = 5.2966e6         # 5.2966 MW
# 转换层默认采用电功率额定（与 FAST.Farm bridge 对齐）
P_RATED_W: float = P_RATED_ELEC_W

# 传动+发电效率（气动→电）。用于由 Cp 估算电功率时的折算。
DRIVETRAIN_EFFICIENCY: float = 0.944

# ---- 控制相关 ----
RATED_WIND_SPEED_MS: float = 11.4        # 额定风速 (m/s)
CUTIN_WIND_SPEED_MS: float = 3.0
CUTOUT_WIND_SPEED_MS: float = 25.0
A_GREEDY: float = 1.0 / 3.0              # 贪婪(Betz)轴向诱导因子
FINE_PITCH_DEG: float = 0.0              # Region II 精调桨距角

# DISCON_bridge.f90 ROSCO 等效参数（仅供参考/校验，勿用于覆盖 bridge）
GEN_SPD_RATED_RADS: float = 122.9        # rad/s (高速轴)
T_RATED_NM: float = 43093.55             # 额定发电机转矩 (Nm)
K_OPT: float = 2.853                     # Region II K*omega^2 系数

# ---- 研究网格默认值（可在 cases/grid.yaml 覆盖）----
DEFAULT_WIND_SPEEDS_MS = [6.0, 8.0, 10.0]
DEFAULT_WIND_DIRECTIONS_DEG = [0.0, 10.0, 20.0]   # 相对来流轴向的偏移（见 cases）
DEFAULT_TURBULENCE_INTENSITIES = [0.06, 0.10]
DEFAULT_SPACINGS_D = [4.0, 6.0, 7.0]              # 以 D 为单位的机间距

# Turb3_Row1 基准间距：504 m == 4D（data_cases.py 中定义）
BASE_SPACING_M: float = 504.0
BASE_SPACING_D: float = BASE_SPACING_M / ROTOR_DIAMETER_M   # 4.0

# 风向参考系：FLORIS 气象惯例 270° == 来流自西向东(+X)。
# 单列风机沿 +X 排布时，270° 即为"沿列轴对齐"来流。
AXIAL_WIND_DIRECTION_DEG: float = 270.0


def cp_max_betz() -> float:
    """Betz 极限 Cp = 16/27."""
    return 16.0 / 27.0


def power_from_cp(cp: float, wind_speed_ms: float, *, electrical: bool = True) -> float:
    """
    由功率系数 Cp 与来流风速估算功率 (W)。

    Parameters
    ----------
    cp : float
        功率系数（气动）。
    wind_speed_ms : float
        来流风速 (m/s)。
    electrical : bool
        True 则乘以传动效率得到电功率；False 返回气动功率。
    """
    p_aero = 0.5 * AIR_DENSITY * ROTOR_AREA_M2 * cp * wind_speed_ms ** 3
    return p_aero * DRIVETRAIN_EFFICIENCY if electrical else p_aero
