# EXECUTION LOG — full-DOE gate 清单跟踪（审查 §20）— 更新 2026-09-16 13:15

| # | Gate | 状态 | 结果/日期 |
|---|---|---|---|
| 1 | timestep compatibility test | ✅（用户指令） | dt 对齐：bts 0.1 == DT_High 0.1（工具箱 dT_High=dT_Box；模板默认同）——全部门禁/identity/yaw-sign/1200s 冒烟均在此配置 PASS；B/C 档 N/A/可选 |
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


## DEL 多机 .outb 通道（2026-09-16 定位解决）
- 根因实证：ZMQ 客户端在线（引擎轮询状态）时，集成 FAST.Farm 只写 Case.T1.outb（T2/T3 缺失；
  先关客户端再收尾还会触发 FAST.Farm 0xC0000005 崩溃）。
- 解决：**乙模式 = 无客户端 direct exe 运行** -> 正常收尾且 Case.T1/T2/T3.outb 全部写出。
- 落地：run/outb_only_run.py（direct exe + 解析多机 outb -> 原始高频 CSV + summary），
  无须修改 wfcrl 源码（case 由 ContinuousFastFarmInterface 生成后直接 exe）。
- 载荷通道：turbine .fst OutList 现缺 blade/tower 载荷通道（RootMyb/RotThrust 等）——
  Phase 7 时对生成 case 的 WT*.fst 追加载荷 OutList 即可（仍在 study 侧）。
- 分工：功率统计=ZMQ 模式（全机状态）；DEL/载荷=乙模式（全机 outb，代表 case）。
## 工程里程碑（今日）：
- 批跑器子进程化（_case_worker）：libzmq 致命 assert 只杀单 case；quick smoke 验证（baseline/yaw ok）
- 自动随机端口：TIME_WAIT "Address in use" 根治
- yaw target = NacYaw 系（对齐 0°）；"270°" 版本被 identity 抓获（-3.9% → -0.31%）
- induction 标定链路成型（1T Case + _calib_worker + zmq_state_rows.json）

## P1 yaw 有效性闭环 + 偶发顺桨发现（2026-09-21）
- **P1 根因**：bridge `_send_yaw_delta` 硬编码 ttl=60s → 静态命令 63s 后过期回退，机舱转回 0°（历史全部 yaw 无效结论均由此产生）。修复：replay `_send_cmds` 直调 engine，`ttl_s=1e6`（研究侧，未动 wfcrl/closed-loop-control）。
- **验证**：180s/600s/1200s 三次带命令运行 seq=1 保持、机舱恒 −30.1°、T1 P≈1133 kW（−31%≈cos²30°）、T2 尾流偏转收益 1646 kW。300s 门禁真盒 4195 / uniform 3863 kW PASS。
- **新发现（转 P2）**：111818 次 1200s 运行 t≈524.2s T1 非命令性永久顺桨（pitch→90°@8°/s、GenTq→0）。排除 SD_Mode/运行时 fault/wfcrl 桥接/命令链。条件：`VS_TSRopt=0` 使转速被转矩 PI 钉在 `VS_MinOMSpd` 贴边（±0.005 rad/s 极限环）。同配置 600s 复跑未复发，ROSCO 内部态（dbg2 126 通道）全程健康 → 判非确定性数值故障。1200s+roscolog 复现运行进行中。
- **影响**：Induction U8 标定 map 作废重做（旧口径含未生效 yaw + 全窗均值）；DOE 批跑器需加顺桨检测剔除。
