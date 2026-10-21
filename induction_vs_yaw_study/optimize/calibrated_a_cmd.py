"""calibrated_a_cmd.py — 基于 FAST.Farm 标定 map 的 a_cmd 直优支撑（Phase 4 / gate 10）
================================================================================

ICD v1.5 接口升级后（2026-10-20），诱导执行语义变化（W-01/W-02：r_pitch 纯桨距 +
PRC/PS/VS 全量 save-restore），旧理论 Betz ratio 不再忠实刻画 a_cmd→功率 关系。
本模块加载 FAST.Farm 实测 operating map（_calib_worker 产出），建立
    a_cmd  →  FAST.Farm 可实现功率比 ratio(a) = P_map(a) / P_map(a_ref)
的校准曲线，供 floris_derating_opt 在 a_cmd 决策变量上做标定搜索。

约束（用户指令 2026-10-20）：不修改 farm_control_runtime/ 与任何场控接口代码，
本模块只消费标定数据（results/_calib_ind/*.csv，研究侧产物）。

用法：
    from induction_vs_yaw_study.optimize.calibrated_a_cmd import CalibrationCurve
    curve = CalibrationCurve.load(u_ms=8.0)      # 无 map 时返回 None
    ratio = curve.ratio_of(float(a_cmd))          # 0.10..0.33 -> 0.63..1.0
    a_min = curve.a_min_eff                        # U8: 0.10
"""
from __future__ import annotations

import os
from pathlib import Path
from typing import Optional

import numpy as np

_STUDY_ROOT = Path(__file__).resolve().parents[1]

# a_cmd 物理上限（动量理论贪婪根）；标定 map 的末点应≈此值
A_GREEDY = 1.0 / 3.0
# 允许 map 末点与贪婪的偏差（P 比值），超出则警告（map 可能未覆盖）
_GREEDY_TOL = 0.05


class CalibrationCurve:
    """由 FAST.Farm 标定 map 构造的 a_cmd → 功率比 校准曲线。

    ratio(a) = P_map(a) / P_map(a_ref)，其中 a_ref 取 map 中最大 a_cmd 点
    （期望≈A_GREEDY=1/3）。线性插值（a_cmd 轴），外插钳制到端点。
    """

    def __init__(self, u_ms: float, a_vals: np.ndarray, p_w: np.ndarray,
                 source: str):
        self.u_ms = float(u_ms)
        self.a_vals = np.asarray(a_vals, float)
        self.p_w = np.asarray(p_w, float)
        self.source = source
        # 排序（map 应已按 a 升序，防御性排序）
        order = np.argsort(self.a_vals)
        self.a_vals = self.a_vals[order]
        self.p_w = self.p_w[order]
        # 贪婪参考 = 最大 a 点
        self.a_ref = float(self.a_vals[-1])
        self.p_ref = float(self.p_w[-1])
        # 有效下界 = map 最小 a（ICD v1.5 U8 实测 0.10，旧 DLL 曾在此崩溃）
        self.a_min_eff = float(self.a_vals[0])
        if self.p_ref <= 0.0:
            raise ValueError(f"calibration map {source}: P_ref<=0")

    @classmethod
    def load(cls, u_ms: float,
             map_path: Optional[Path] = None) -> Optional["CalibrationCurve"]:
        """从标定 map csv 加载 U 风速曲线；文件缺失/该 U 无数据 -> None。"""
        path = map_path or (_STUDY_ROOT / "results" / "_calib_ind" /
                            "induction_operating_map_icd15.csv")
        if not Path(path).exists():
            return None
        import csv
        rows = []
        with open(path, newline="", encoding="utf-8") as f:
            for r in csv.DictReader(f):
                try:
                    u = float(r["U"])
                    a = float(r["a_cmd"])
                    p = float(r["P_kW"]) * 1e3
                except (KeyError, ValueError):
                    continue
                if abs(u - float(u_ms)) < 1e-9:
                    rows.append((a, p))
        if not rows:
            return None
        a_vals = np.array([r[0] for r in rows], float)
        p_vals = np.array([r[1] for r in rows], float)
        return cls(u_ms, a_vals, p_vals, source=str(path))

    def ratio_of(self, a_cmd: float) -> float:
        """a_cmd → 标定功率比 ratio∈[ratio(a_min_eff), 1.0]。钳制到曲线端点。"""
        a = float(np.clip(a_cmd, self.a_min_eff, self.a_ref))
        p = float(np.interp(a, self.a_vals, self.p_w))
        return p / self.p_ref

    def a_of_ratio(self, ratio: float) -> float:
        """标定功率比 → a_cmd（反插值；优化结果换算用）。"""
        ratio = float(np.clip(ratio, 0.01, 1.0))
        ratios = self.p_w / self.p_ref
        # 单调段内插值；端点外钳制
        return float(np.interp(ratio, ratios, self.a_vals))

    def greedy_check(self) -> Optional[str]:
        """末点（a_ref）与理论贪婪功率的偏差提示；None=正常。"""
        from induction_vs_yaw_study import constants as C
        p_theory = C.power_from_cp(C.cp_max_betz(), self.u_ms, electrical=False)
        dev = abs(self.p_ref - p_theory) / p_theory if p_theory > 0 else 1.0
        if dev > _GREEDY_TOL:
            return (f"map 末点 a={self.a_ref:.2f} P={self.p_ref/1e3:.0f}kW "
                    f"与理论贪婪 {p_theory/1e3:.0f}kW 偏差 {dev*100:.1f}%")
        return None

    def describe(self) -> str:
        return (f"CalibrationCurve U={self.u_ms}m/s [{self.source}] "
                f"a∈[{self.a_min_eff:.2f},{self.a_ref:.2f}] "
                f"ratio∈[{self.ratio_of(self.a_min_eff):.3f},1.0] "
                f"({len(self.a_vals)} pts)")
