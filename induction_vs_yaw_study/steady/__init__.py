"""
induction_vs_yaw_study.steady — 稳态风快速对比子包
===================================================
在湍流盒(.bts)仿真较慢时，用稳态风(WindType=1)快速得到 FAST.Farm vs FLORIS
的诱导(降额)/偏航 控制对比。复用主工程的布局、转换层、调度、LUT 与分析。

入口：
  * run_replay_steady  —— 稳态回放（FAST.Farm 走 WindType=1，无 .bts）
  * run_compare_steady —— 汇总对比（速度×风向×间距）
"""
