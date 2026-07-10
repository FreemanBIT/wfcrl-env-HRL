"""
induction_vs_yaw_study
======================
诱导因子(降额)控制 vs 偏航控制 对比研究包。

针对 3 × NREL 5MW 单列风场，在 FLORIS 中分别求解最优偏航 LUT 和最优功率设定点
(降额/诱导)LUT，再在 FAST.Farm 中用相同工况回放，最后交叉对比。

模块组织（按开发阶段命名）：
- stage0_probe.py            : 接口探针与可行性自检
- constants.py              : 物理常数（NREL 5MW）
- conversion/               : 功率设定点 ↔ 诱导因子/最小桨距 转换层 (Stage 1)
- cases/                    : 3 机单列算例与工况网格 (Stage 2)
- optimize/                 : FLORIS 偏航 + 降额 LUT 求解 (Stage 3)
- lut/                      : LUT 数据结构与多维插值
- replay/                  : FLORIS / FAST.Farm 统一回放 (Stage 4-6)
- analysis/                 : 指标计算与对比图 (Stage 7)
- run/                      : 各阶段命令行入口

设计原则：
1. 不修改 wfcrl/ 既有实现（控制接口已支持 5-mode，无需改动）。
2. 不修改风场模型库 / 模板；不同间距通过脚本内替换 xcoords 实现。
3. 控制量单一来源：FLORIS 与 FAST.Farm 喂入的 yaw / power 由同一函数产出。
"""

__version__ = "1.0.0"
