# EXECUTION LOG — full-DOE gate 清单跟踪（审查 §20）— 更新 2026-09-16 13:15

| # | Gate | 状态 | 结果/日期 |
|---|---|---|---|
| 1 | timestep compatibility test | 🟡 A ✅；B 待 0.05 短盒（生成器已支持 --time-step/--analysis-time） | A：bts0.1+DT_High0.01 全程通过 |
| 2 | BTS usable-time 语义一致 | ✅ | 非周期 + 足长 AnalysisTime + ALL（全时长可查）唯一语义；1800s 盒覆盖 1200s 仿真验证 |
| 3 | same (U,TI,seed) -> same BTS | ✅ | test_bts_reuse + QA |
| 4 | zero-yaw identity | ✅ | -0.31%（farm）；分机版脚本（T1..3 <0.5%） |
| 5 | baseline no-override identity | ✅ | baseline_repeat -0.007% |
| 6 | greedy/sentinel induction identity | ✅ | induction_r1 -0.44% |
| 7 | yaw sign verified | ✅ | wd+10: +20 最优 4635kW 与 FLORIS(+15°) sign 一致；wd-10: ±20 对称；wd0: 0 最优 |
| 8 | WD mirror sanity | 🟡 单 seed 已跑 | P(+10,+20)=4635 vs P(-10,-20)=4147；多 seed 平均待 pilot |
| 9 | induction operating map（1T） | 🟡 U8 六点完成 | a 0.10→1456…0.33→1588kW（单调）；Ct/Cp 运行时字段为 0（待验证）；U6/10/12 待盒 |
| 10 | FLORIS induction LUT 存 calibrated a_cmd | ⬜ | Phase 4（LUT 现含 ratio/power_target；a_cmd 主链待接） |
| 11 | 显式窗口 + 时长派生 | ✅ | metrics.py + Case 派生计数（P0-5） |
| 12 | worker 清理 PID-scope | ✅ | run/_proc_utils（age+PID） |
| 13 | manifest 并发安全 | ✅ | 批跑器增量 append+锁；resume 重试 failed |
| 14 | BTS 共享引用 | ✅ | engine 只引用源盒 |
| 15 | pilot DOE 通过 | ⬜ | 缺 U8 盒（5/18）+ 全精度 LUT 可跑（quick LUT 已有） |

## 工程里程碑（今日）：
- 批跑器子进程化（_case_worker）：libzmq 致命 assert 只杀单 case；quick smoke 验证（baseline/yaw ok）
- 自动随机端口：TIME_WAIT "Address in use" 根治
- yaw target = NacYaw 系（对齐 0°）；"270°" 版本被 identity 抓获（-3.9% → -0.31%）
- induction 标定链路成型（1T Case + _calib_worker + zmq_state_rows.json）
