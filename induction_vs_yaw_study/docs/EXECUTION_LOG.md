# EXECUTION LOG — full-DOE gate 清单跟踪（审查 §20）

> 更新方式：每完成一项就地打勾并记录结果；全部打勾前禁止 full DOE。

| # | Gate | 状态 | 结果/日期 |
|---|---|---|---|
| 1 | timestep compatibility test（A/B/C） | 🟡 A 通过；B 待生成 0.05 短盒；C=N/A（成本） | A: bts0.1+DT_High0.01 全程通过（identity/1200s 冒烟/9 次 sign）；B 进行中 |
| 2 | BTS usable-time 语义一致（ALL=全时长可查、非周期、无 wrap） | ✅ | 文档 §4.2 + 生成器注释统一；1800s 盒验证 1200s 仿真尾部无越界 |
| 3 | same (U,TI,seed) -> same BTS | ✅ | test_bts_reuse（31.2-4）+ QA pass |
| 4 | zero-yaw identity（farm + T1..3 <0.5%） | ✅ | -0.31%（farm）；分机版脚本就绪（实测 yaw0 各机 ~0.3%） |
| 5 | baseline no-override identity | ✅ | baseline_repeat -0.007%（重复性） |
| 6 | greedy/sentinel induction identity | ✅ | induction_r1 -0.44%（a≈1/3 via power_to_induction(Betz)） |
| 7 | yaw sign verified（WD -10/0/+10 × yaw ±20） | ✅ 初版 | wd+10: +20 最优 4635kW 与 FLORIS(+15° 最优) sign 一致；wd-10: ±20 对称(4147)；wd0: 0 最优。mirror 需多 seed |
| 8 | WD mirror sanity checked | 🟡 单 seed 已跑 | P(+10,+20)=4635 vs P(-10,-20)=4147（单 seed 不要求相等；等 pilot 多 seed 平均） |
| 9 | induction operating map（1T 标定） | ⬜ | 待 Phase 3（1T 支持已就绪：Case.n_turbines=1 + 自动端口） |
| 10 | FLORIS induction LUT 存 calibrated a_cmd | 🟡 | LUT 已有 ratio/power_target；a_cmd 主链改造待 Phase 4 |
| 11 | 显式 t_sim/t_settle 窗口生效 | ✅ | metrics.py 落地 + test_stats_window；Case 时长派生（P0-5） |
| 12 | worker 进程清理 PID-scope | ✅ | run/_proc_utils.cleanup_stale_fastfarm（age+PID） |
| 13 | manifest 更新并发安全 | ✅ 增量 | run_fastfarm_batch 增量 append + 锁；resume 跳过 done/重试 failed |
| 14 | BTS 共享引用（无 per-case GB 复制） | ✅ | engine 只引用 FarmInputs/coarse 源路径（已确认） |
| 15 | pilot DOE 通过 | ⬜ | 待 U8 18 盒齐 + 批跑器子进程化防 abort |

## 关键观测记录（2026-09-16）

- ZMQ 状态帧为功率统计唯一口径（本集成 FAST.Farm 仅写 Case.T1.outb）。
- 致命 libzmq assert（Connection reset / Socket op on non-socket）会杀死进程：
  → 单 case 子进程隔离（sign 套件逐 yaw 子进程）+ 自动端口（PID 派生）已落地。
- yaw 目标系 = NacYaw 系（对齐 0°）；"270° 对齐"版本曾使 yaw0 偏航 ~270° 致 farm -3.9%（被 identity 抓获）。
- 300s 回归门禁状态：每次代码改动后 PASS（约 138-195s 墙钟）。
- 冒烟（1200s 全时长 × 3 控制）：baseline/yaw 各 ~610s 墙钟 PASS；1800s 盒时间覆盖成立。
