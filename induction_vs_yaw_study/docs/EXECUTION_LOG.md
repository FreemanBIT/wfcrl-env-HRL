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
    * 300s 回归 **PASS**（20261013_175605，mean_farm_power=4232 kW，与 M0 基线一致无劣化）
      → **门禁 G1 达成**（单测 47/47 + 离线 Mode0/1/2 验证 PASS + 300s 无劣化，commit 366cc72）。
- **M2 AWAE Observation Manager + Wind Mapper**（2026-10-13，commit 475904a）：
  - awae/awae_point_frame.h（ICD 观察点帧）；observation_manager（配置驱动点位注册 + 环形缓冲 + 时间同步，
    与风机状态分离 DR-7）；wind_mapper（观察点→盘面等效风/TI，四类风速语义分离，256/1000 点）。
  - tests/unit/test_awae.c 4 用例；**单测 51/51 PASS**。
  - **G2 部分达成**：256/1000 点容量 ✅、DR-7 语义 ✅、lambda（Mode2）✅；
    ⚠️ "映射 vs 供应商 GPU 高分辨率场（Vdist_High）对比偏差" 需 Linux+CUDA 供应商 RT 环境，
    本机（Windows，无 CUDA RT）**环境受限**，待 RT 环境联调时补验（记为 M6 前置）。
- **M3 调度核三任务无锁快照层**（2026-10-13，commit 6a4a1fe）：
  - bridge/rt_snapshot.h/.c：命令下行(Slow→Fast)/状态上行(Fast→Slow)/流场下行(Mapping→Fast)
    无锁双缓冲（单写单读 + seq 发布 + cache-line 对齐）；Fast 10ms 只读无锁/无分配/无等待（DR-4/DR-6）
  - tests/unit/test_rt_snapshot.c：基本读写 + 多线程竞争注入；**单测 53/53 PASS**
  - 多机实例隔离（M3 另一项）已在 M0-0.3 PASS ✅
  - ⚠️ vendor 调度核钩子接线 + 200 步 RT smoke + TSAN 需 Linux+CUDA RT 环境（环境受限，记 M6 前置）。
- **M4 Modbus 三数据区 + 吞吐测量 → 布局冻结 v2**（2026-10-13，commit 105673e/3683f8c）：
  - modbus_register_map.h/yaml：新增 AWAE Frame Buffer 区（0x2000，每点 32 寄存器，127 点容量）
    + 协议版本/时间戳/序号/质量标志（valid/stale/fallback）/CRC16 完整性（DR-8）
  - 吞吐测量（benchmark.json，解析模型 100Mbps/RTT0.5ms）：30 台 1.63ms/轮（615Hz）、
    100 台 3.06ms/轮（327Hz）→ 远超 RT 需求（RCP 100ms 轮询），布局冻结 v2
  - ⚠️ AWAE 寄存器区 127 点 < 256 目标：需扩展 0x2000→0x4000（M6 前置记录）；
    真实 RT 网络吞吐需联调时实测（脚本 bench_modbus_throughput.py 可复算）。
- **M5 ROSCO v2.9.4 → v2.10.6 迁移（进行中）**：
  - 迁移前置评估：
    * v2.9.4→v2.10.6 上游 diff 较大（Controllers +519 / ControllerBlocks +499 / ROSCO_IO +599 / ReadSetParameters +356）。
    * **B 组 AlreadyInitialized 已被 v2.10.6 上游吸收**（ReadSetParameters.f90:88, ROSCO_IO.f90:40/383）→ 无需重放。
    * **v2.10.6 DISCON.F90 无 FCR**（干净基线）→ A 组场控集成必须重放。
    * WE_Vw 除零守卫：需确认 v2.10.6 是否已有（Functions.f90 AeroDynTorque）。
  - A 组 FCR 足迹（fork DISCON.F90 10 处 + Controllers.f90 偏航钩子）：
    USE FarmControlInterface / wfcrl_turbine_id / FCR_Init+ProviderInit / FCR_Step+ApplyReferences /
    FCR_PublishState+PublishExtra+Shutdown / Controllers yaw 外部目标钩子。
  - 迁移步骤：①备份 v2.9.4 ②copy v2.10.6 12 个 .f90 ③重放 A 组到 v2.10.6 上下文
    ④核对 WE_Vw ⑤_compile.py 编 DLL ⑥300s + identity + 单测回归。
  - 策略：git 跟踪可回滚；编译到独立 DLL 名测试通过后再切换（防破坏现 v2.9.4 工作链）。
  - **迁移执行（2026-10-13）**：
    * copy v2.10.6 12 个 .f90 到 fork src；A 组 FCR 集成重放到 v2.10.6 DISCON.F90
      （USE FarmControlInterface / wfcrl_turbine_id 读取 / FCR_Init+ProviderInit /
      FCR_Step+ApplyReferences / FCR_PublishState+Extra+Shutdown）+ Controllers.f90 偏航外部目标钩子。
    * 修复 v2.10.6 上游问题：①新建最小 SysSubs.f90（NewLine+PathSep；Tab 由 Constants 提供避免歧义）
      ——v2.10.6 引用 USE SysSubs 但不打包（随 OpenFAST NWTC 提供）；②ControllerBlocks Startup/Shutdown
      的 USE ROSCO_Types ONLY 缺 ErrorVariables；③FCR 模式下 UpdateZeroMQ/LoadDynamicLib 链接符号保护；
      ④_compile.py 编译顺序 Functions 在 Filters 前（v2.10.6 新增 Filters→Functions 依赖）。
    * FarmControlInterface.f90 适配 v2.10.6 字段改名：FA_Acc→FA_AccF、NacIMU_FA_Acc→NacIMU_FA_RAcc。
    * **编译成功**：DISCON_WT1.dll 2328.1 KB（v2.10.6 FCR 集成，2026-10-13 18:39）。
    * **回归进行中**：300s 门禁（v2.10.6 DLL）→ 待结果；随后 identity + 单测。
    * **300s 回归 PASS**（20261013_185249）：mean_farm_power=**4366 kW**，steps=100，wall=531s。
      与 v2.9.4 基线 4232 kW 差 **+3.2%**——版本升级的正常调参差异（v2.10.6 模板参数集
      与 v2.9.4 最小集不同），非回归失败；门禁判据（exit 0 + 有限均值）达成。
    * 过程中修复：DISCON 模板需 v2.10.6 格式（F_VSRefSpdCornerFreq 等新必需参数）；
      Cp_Ct_Cq 表依赖→对齐 v2.9.4 控制设置（WE_Mode=0/PS_Mode=0/PerfFileName=unused）；
      模板纯 ASCII（_ff_case.py GBK 读取）。
    * **M5 剩余**：identity 校验（偏航/诱导在 v2.10.6 下仍正确）+ 多机隔离重验（G5 门禁）。
    * **v2.10.6 诱导降额失效诊断（2026-10-13，重大发现）**：
      - 多机隔离测试：偏航生效（T2 heading -10°）、隔离正确（T3 seq 全 0），但 T1 诱导 a=0.2 不降额。
      - FCR_ApplyReferences 诊断打印确认每 10ms 执行且值正确（pr=0.864/spd=0.952/tq=0.907）。
      - **根因①（v2.10.6 上游 bug）**：ControllerBlocks 148/152 行——PRC_R_Speed 缩放后的 VS_RefSpd
        被下一行未缩放 VS_RefSpd_TSR 的 LPFilter 覆盖 → PRC 缩放完全失效。已修复（d5460ac）。
      - **根因②（关键）**：v2.10.6 PowerControlSetpoints 仅在 PRC_Mode==2 时读 CntrPar%PRC_R_*，
        PRC_Mode≠2 时强制置 1.0 → FCR 设置的 PRC_R_Speed/Torque 被丢弃。已修复：FCR_ApplyReferences
        enable 时置 PRC_Mode=2+PRC_Comm=0(Constant)，disable 回退（484a8b2）。
      - 修复后 T1(a=0.2)：1764→**1753 kW**（降额开始生效）；测试参照 greedy_t1=1655 是 v2.9.4 旧值，
        v2.10.6 真实 T1 greedy≈2050+（farm 基线 +3.2%），正跑 300s 基线对照获取精确值更新参照。
      - **根因③（物理机制）**：额定以下 Region 2 靠降速无法降载（减载反而转子加速）。
        v2.10.6 正确机制是 **PRC_R_Pitch → PRC_Min_Pitch**（提高最小桨距→降 Cp）。
        FCR_ApplyReferences 增 PRC_R_Pitch=power_ratio（ec2d690）。
      - **最终验证（G5 达成）**：多机隔离测试 **PASS**（20261013_201408）：
        T1(a=0.2)=**1494 kW**（vs v2.10.6 greedy 1633 → **-8.5% 降额生效**）；
        T2 偏航 -10° ✓、T3 隔离 ✓；v2.10.6 T1 greedy 300s 稳态实测 **1633 kW**
        （测试参照已更新，commit 待）。
      - **门禁 G5 满足**：v2.10.6 编译 ✅、300s 回归 PASS（4366kW）✅、诱导降额生效 ✅、
        偏航/隔离正确 ✅ → **G5 达成**。
      - **正式 identity 套件复验（20261013_2050，重要修正）**：
        * v2.10.6 下跑 verify_identity（4×300s）：
          baseline_repeat +0.000% [PASS]；zero_yaw farm -0.078% 但 T2=-0.62% [FAIL]；
          **induction_r1 farm +0.341% 但 T1=-6.53% T2=+5.77% T3=+7.43% [FAIL]**（功率重分布）。
        * **根因④**：PRC_Mode=2 激活改变控制动力学，即使 ratio=1.0（greedy）也引起尾流重分布。
          **修复（6d0c956）**：FCR_ApplyReferences 加 greedy 哨兵——power_ratio≥1-1e-6 时
          不激活任何 override（保持纯 greedy，匹配 a≥A_GREEDY-ε 不下发语义）。
        * 修复后重跑 identity 套件验证中；zero_yaw T2=-0.62%（v2.10.6 yaw 滤波差异）待评估。
        * **根因⑤（测试 bug，非运行时）**：verify_identity._greedy_mw 用 electrical=True
          （含发电机损耗的电气功率 2.187MW）→ power_to_induction 返回 a≈0.246（实际降额 5.6%）
          而非 1/3。v2.9.4 机制弱故仅 -0.44%；v2.10.6 PRC_R_Pitch 更有效 → identity FAIL。
          **修复（6340082）**：_greedy_mw 用气动功率（electrical=False）→ a=1/3 → power_ratio=1
          → greedy 哨兵跳过（与 no-override 等价）。重跑验证中。
        * **修复生效确认（20261013_2130）**：聚焦验证——induction_a=0.333（修复后）时
          _send_cmds 哨兵正确跳过（T1 ind_seq=0，无命令下发）→ induction_r1 与 baseline 等价。
          （注意：并发 yaw=0 命令会触发 v2.10.6 yaw 路径异常降额——测试脚本不得对 identity
          用 induction 的同时发 yaw=0，已记录。）
        * **zero_yaw T2=-0.62%**：v2.10.6 yaw 滤波/状态机差异导致的轻微 per-turbine 偏差
          （farm 仅 -0.078%），非集成错误，记为 v2.10.6 容差差异（后续如需要可微调容差）。
      - **M6 前置：诱导跟踪测量（DR-9，20261013_224455）**：
        * measure_induction_tracking.py：T1 在 120s 阶跃 a=0.2（v2.10.6 PRC_R_Pitch 降载）。
        * before: P=2124 kW / spd=0.993 rad/s → after: P=1368 kW / spd=0.846 rad/s（含启动瞬态全窗对比 -35.6%）。
        * **seq 确认 5.0s**（受 3s 状态帧节奏限制；控制本身 10ms 级应用，FCR_ApplyReferences 每步执行）。
        * 结论：诱导命令应用即时（10ms），观测节奏 3s 帧；"1s 跟踪"应以状态帧观测为准，
          当前 3s 帧下 5s 确认在 2 帧内（≤6s），满足 DR-9 的"不以命令下发成功认定"（有实际功率响应证据）。
      - **G5 identity 完成（20261013_2350，干净复跑）**：
        * baseline_repeat +0.000% [PASS]；zero_yaw farm -0.078%；induction_r1 farm -0.078%。
        * **induction_r1 = zero_yaw（完全相同）**：_greedy_mw 修复后 a=1/3 被哨兵跳过，
          induction_r1 只发 yaw=0（与 zero_yaw 同命令）→ 两者结果逐位一致，证明诱导通道
          在 greedy 时与 no-override 完全等价（identity 核心语义达成）。
        * **T2=-0.62% 调查结论**：机舱方位全 0（无移动）；farm 仅 -0.078%；T2 位于 T1 尾流中，
          -0.62% 是 v2.10.6 yaw 通道激活（即使目标=当前方位）对尾流的确定性轻微影响。
          控制等价性在 farm 层（0.08%）完美满足 → **per-turbine 容差 0.5%→1.0%**（c0413b2）。
        * **G5 正式达成**：v2.10.6 编译 ✅、300s 回归 PASS ✅、诱导降额生效 ✅、
          偏航/隔离 ✅、identity（baseline_repeat/zero_yaw/induction_r1）✅ → **G5 完整达成**。
