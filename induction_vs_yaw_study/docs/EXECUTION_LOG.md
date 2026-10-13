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

## ROSCO Bridge 开发计划 M0 进度（BRIDGE_DEV_PLAN_v2，2026-10-13）
- 上游输入：FASTFarm_WFC_Bridge_Development_Plan_v1_1.md + FASTFarm_WFC_Interface_ICD_v1_1.csv；
  决策（7 条 DR）已全部纳入 BRIDGE_DEV_PLAN_v2（docs/farm_control/，commit a55a68d）。
- M0-0.2 可行性结论：供应商 RT（fastfarm_rt_v1.1.0）ServoDyn 走 Bladed-DLL 接口
  （PCMode=5/VSContrl=5，DLL_FileName=./DISCON_WT1.so），实际 .so 为 21KB ELF 桩
  → ROSCO 保留在 OF 内 ServoDyn/DISCON 路径成立（DR-1），需从本 fork 编 Linux .so 挂入。
- M0-0.4 300s 基线回归：**PASS**（20261013_163723，U8/TI10/seed01，WD0/S6D，300s，steps=100，
  mean_farm_power=4232 kW，wall=525.9s；注：load-ch 已加 TwrBsMxt/TwrBsFxt 等通道，
  与旧基线 3860~4195kW 口径差异待 M0-0.3 多机隔离验证时统一）。
- **M0 全部子任务完成**（2026-10-13 更新）：
  - M0-0.1 ROSCO fork diff 盘点 → docs/farm_control/M0_ROSCO_FORK_DIFF.md（commit 49cd61b）：
    A 组场控集成仅 DISCON.F90(+212/-3 FCR 宏) + Controllers.f90(+17/-1 偏航钩子)；
    B 组健壮性（AlreadyInitialized/WE_Vw 除零守卫/SysGnuWin）其余文件原版。
  - M0-0.3 多机实例隔离验证 → verify_multiturbine_isolation.py（commit ffb1d4a）**PASS**：
    单次 300s T1=诱导0.2(仅)/T2=偏航+10(仅)/T3=无命令；
    seq 回显：T1 ind=1/yaw=0、T2 yaw=1/ind=0、T3 全0（无串扰）；
    功率：T1=1418kW(-14.3% vs greedy 1655，诱导生效)、T2=1271kW(偏航生效)、T3=1366kW。
    → 确认 B 组 AlreadyInitialized 守卫多实例无冲突。
  - M0-0.5 实时核心拆分设计 → docs/farm_control/M0_RTCORE_SPLIT_DESIGN.md（commit b603f7c）：
    DR-4 无锁快照协议（双缓冲 + seq），10ms 禁锁/socket/malloc/等待；rt_core + comm_adapter 新建。
- **门禁 G0 达成条件核对**：0.1/0.2 结论落库 ✅、0.4 300s 基线 PASS ✅（4232kW）、
  0.3 多机隔离 PASS ✅、0.5 设计落库 ✅ → **G0 满足，可进入 M1**。
- **M1 Bridge 实时核心首版**（2026-10-13，commit acb9057）：
  - bridge/fcr_bridge_core.h/.c：独立于通信的可测核心（DR-3/DR-4/DR-5）
    * Mode0 旁路 / Mode1 功率比例开环+控制分配（R_Pitch/R_Speed/R_Torque 权重，禁未分配叠加）/
      Mode2 Ct 前馈（Ct(lambda,beta) 气动表反解 min_pitch_ref）/ Mode3 桩
    * Ct 估计 DR-7 模型修正（扰动盘面风归一化修正，不直接经典归一化）
    * 偏航 DR-5 绝对目标锁存 + TTL/fallback 安全；10ms step 纯内存（无锁/socket/malloc/等待）
  - bridge/yaw_adapter.c：旧 delta 协议 → 绝对目标适配器
  - tests/unit/test_bridge_core.c 8 用例；**单测 47/47 PASS**
  - **Bridge 接口设计已提交供审查**（fcr_bridge_core.h，commit acb9057/fc7e997）。
  - **G1 进行中**（2026-10-13）：
    * Bridge 审查点默认决策落地（fc7e997）：aero 表改共享指针省内存（320KB/机→共享）、
      DR-7 四类风速语义（ambient_upstream/disk_background/disk_disturbed）；单测仍 47/47。
    * 离线模式验证脚本 verify_bridge_modes.py（2f74889）**PASS**：ctypes 加载编译后 bridge DLL，
      校验 Mode0 旁路 / Mode1 R_power 单调+控制分配守恒 / Mode2 Ct 表反解 min_pitch 方向 /
      Ct 估计 DR-7 上游参考风优先 / yaw_adapter DR-5 / TTL 安全。
    * 300s 回归（G1 剩余项）运行中。
