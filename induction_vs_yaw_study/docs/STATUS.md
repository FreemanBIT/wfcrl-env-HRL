# STATUS — 诱导 vs 偏航研究：开发进度与问题（快照 2026-10-20 18:00）

> 本文档为阶段性状态快照（进度 / 验证结论 / 开放问题）。gate 逐项跟踪见 `EXECUTION_LOG.md`，
> 方案见 `../DEVELOPMENT_PLAN_ZMQ.md`。

---

## 一、已完成

### 1. 方案与文档
| 项 | 状态 |
|---|---|
| 开发方案 V3（吸收审查 8 P0 / 7 P1 / 5 P2） | ✅ DEVELOPMENT_PLAN_ZMQ.md |
| 时间尺度固化：DT_High=0.1 == bts dt=0.1（用户指令；实测工具箱 dT_High=dT_Box） | ✅ 80dfc61 |
| full-DOE 15 项 gate 清单 + 执行日志 | ✅ EXECUTION_LOG.md |
| DEL 多机 outb 根因与解决方案记录 | ✅ 9126e3a |

### 2. 风盒与数据管线
- seed 键生成器 `turbsim_inp/generate_wind_boxes.py`：按 U 自适应 AnalysisTime（U6=1950 / U8=1800 / U10=1700 / U12=1650 s）、不限时、支持 `--time-step/--analysis-time` 覆盖
- 风盒 QA `turbsim_inp/qa_wind_boxes.py`（realized U/TI 达标 5% / 0.02）
- seed-key resolver（`cases/three_nrel5mw.resolve_bts_file`）、`run/build_case_manifest.py`（全量 2592 行：pending/blocked 分级）
- 补齐工具 `run/_fill_boxes.py`（缺失清单驱动；跳过 0 字节在写文件）

### 3. 运行与统计基础设施
- **批跑器子进程化** `run/_case_worker.py` + `run/run_fastfarm_batch.py`：libzmq 致命 assert 只杀单 case、resume/retry、增量状态写
- **自动随机端口**（PID 派生）：根治 `Address in use`（TIME_WAIT）反复问题
- **P2-1 provenance**：run `provenance.json` + case `metadata.json`（git commit、exe/bts sha256、约定版本）
- **DEL 通道**：`run/outb_only_run.py` 乙模式（无 ZMQ 客户端 direct exe → 全机 T1..T3.outb）
- 单元测试 **48 passed**

### 4. 验证结论
| 验证 | 结果 |
|---|---|
| identity（zero-yaw / baseline repeat / induction r=1） | ✅ −0.31% / −0.007% / −0.44% |
| yaw sign（WD −10/0/+10 × yaw ±20） | ✅ 完成；WD+10 时 +20 最优，与 FLORIS(+15°) 符号一致 |
| 300s 门禁（每次代码改动后） | ✅ 全部 PASS（farm ≈ 3860 kW） |
| 1200s 全时长（子进程批跑） | ✅ baseline 641 s / yaw 648 s wall |
| FLORIS 快速 LUT | ✅ 144 工况 864 行（--quick） |
| Induction 1T 标定 | 🟡 仅 U8 六点（a 0.10→1456 … 0.33→1588 kW，单调）——**作废**：当时 yaw 未生效且统计口径含过渡段，需按 §三.1 修复后重做 |
| **P1 yaw 有效性闭环** | ✅ 根因=TTL 60s 过期（§三.1）；180s/1200s/600s 三次带命令验证：seq=1、机舱恒 −30.1°、T1 P≈1133 kW（−31%） |

---

## 二、进行中
- **U8 风盒 13/18**（TI05 s1–5、TI10 s1–4、TI15 s1–4）；2 个 TurbSim 正在补齐剩余 5 盒（PID 29640，10:57 启动），单盒 ~7–9 h
- 内存 5 GB 空闲（固定 2 并发 TurbSim）
- **未提交改动**：`replay/fastfarm_zmq_replay.py`（yaw_delta + 长 TTL `_send_cmds`）、`run/_fill_boxes.py`、`run/diag_yaw_effect.py`（新增）、`run/_dbg2_window.py`（新增）
- **进行中实验**：1200s + `--roscolog`（ROSCO 126 通道 10ms 日志）复现偶发顺桨（见 §三.1 附注）

---

## 三、存在的问题（按严重度）

### ✅ P1 yaw 命令未生效 —— 已解决（2026-09-21）
- **根因**：`closedloop/bridge.py` 的 `_send_yaw_delta` 硬编码 `ttl_s=60.0`（induction 30.0）。静态对风场景只在 k=1 发一次命令，runtime 通道 t≈63s 过期回退 → ROSCO 内部目标恢复风向对齐 → 机舱以 0.5°/s 转回 0°。1200s 统计窗 [600,1200] 内机舱在 0°，故"yaw 无效"。全链路其余环节（传输/seq/锁存/ROSCO 状态机/ServoDyn）逐环验证正常。
- **修复**（研究侧，不动 wfcrl/closed-loop-control）：`replay/fastfarm_zmq_replay.py` 新增 `_send_cmds()` 绕过 bridge 直调 `engine.send_yaw_delta(..., ttl_s=1e6)`（`ttl_s=0` 会回落 runtime 默认 60s，必须显式大正值）；同法处理 induction（A_GREEDY 哨兵跳过）；绝对 `yaw_deg` 路径因单位不安全改为直接 raise。
- **验证**（uniform8_700s 盒，T1 yaw_delta=+30° CW 于 k=1 一次性下发）：
  - 180s（20260921_110947）：seq=1 保持，机舱 −30.15°，T1 P≈1135 kW（vs 未偏航 1650，−31%≈cos²30°）
  - 1200s（20260921_111818）：seq=1 + 机舱 −30.147° 保持至 t=1202s；T2 因尾流偏转恢复 1646 kW
  - 600s 复跑（20260921_142310）：全程健康，T1 P_mean=1133 kW
- **300s 门禁**：真盒 4195 kW、uniform 盒 3863 kW，均 PASS
- **附注（顺桨事件，转 P2 范畴）**：111818 那次 1200s 运行在 t≈524.2s 发生 T1 非命令性顺桨（pitch 0→90° @8°/s=PC_MaxRat、GenTq→0、P→0；T2 于 +96s 尾流消失后恢复 1740 kW）。已排除：SD_Mode=0、运行时无 fault/feather 路径、wfcrl 桥接块 `#ifndef` 排除、命令链（seq/target/机舱保持不变）、状态帧 k=174 全字段正常。已定位的贴边条件：`VS_TSRopt=0` → `VS_RefSpd` 被钳到 `VS_MinOMSpd=76.433 rad/s`，转矩 PI 把转速钉在该值（极限环 ±0.005），30° 偏航扭矩降 31% 加剧贴边；t≈518–521 转速缓慢下穿边界且衰减加速。**同配置 600s 复跑未复发**（ROSCO 内部态全程健康：PC_SpdErr=+46.47 恒定、PC_PitComT=0、PC_KP=−0.4）→ 判定为非确定性数值故障（与 P2 OMP 疑因同类），触发机制（桨距 PI 输出永久饱和于 PC_MaxPit 的路径）待 1200s+roscolog 复现运行（pwsh-9）定位。缓解预案：研究侧后处理 DISCON_IN 设 `VS_TSRopt>0`（转速离开 MinOMSpd 贴边）或统计窗剔除 P=0/pitch>80° 时段；该参数在 wfcrl 模板内，按约束⑤只能研究侧 post-process。

### ✅ P2 运行间非确定性 —— 已修复（2026-09-21，f297121）
- **根因确认**：FAST.Farm OMP 浮点归约顺序差异。修复：replay/diag 在 `engine.start()` 前设 `OMP_NUM_THREADS=1`（env `WFCRL_STUDY_OMP_NUM_THREADS` 可覆盖，`0`=旧行为）。
- **验证**：同 real 盒 baseline 两次 300s：farm/分机功率 **逐位一致**（4232.2834 kW，T1 100/100 样本 bitwise 相同）；修复前同 case 差 1.9%。
- **代价**：wall ~140s→352s（~2.5×），DOE 时间预算需重估；可试 OMP=2/3 找确定性/速度折中（pilot 前定）。
- **顺桨事件关联**：OMP 竞争是头号嫌疑；钉线程后复发概率预期大降，批跑器 feather_invalid 过滤逻辑已备（replay metadata）。

### ✅ P3 状态帧字段 —— 已修复（2026-09-21，f297121 + 4c0fb4e）
- **heading/vane 单位**：FCR_PublishState 此前把 ROSCO 内部**度值**直传 `*_rad` 字段（T2/T3 显示 20610° 垃圾、多命令重锁存会算出垃圾目标）。修复：publish 处 ×D2R；重编译 DISCON_WT1.dll。验证：180s yaw=30 诊断 `tgt=-30.00 nac=-30.00`（旧 −30.11/−1724）、锁存目标精确 −0.5236 rad、P=1136 kW；300s 门禁 PASS 4232 kW（基线零回归）。**多命令/control 模式现在语义正确**。
- **rotor_ct/rotor_cp**：运行时不填（需 OpenFAST 内部量），研究侧由 thrust+power+盘面风回填（replay）；均匀盒下受遮挡机组会高估（已注记），标定用真盒 + outb 校核。

### 🟠 P4 资源与产能
- 盒生成慢：13/18，剩 5 盒 ≈20 h；全 72 盒（~103 GB）磁盘方案未定（当前 D: 约 104 GB 空闲）
- 内存 15.8 GB：4 并发 TurbSim 曾打满导致浏览器卡死 → 现固定 2 并发
- 仿真与盒生成必须错峰（1200 s 仿真读盒峰值 ~2.7 GB）

### ✅ P5 工程债 —— 已清理（2026-09-21，e7135d9 + 4c0fb4e）
- ~~未提交~~：P1/P2/P3/P5 四批已提交（1ee69bb、f297121、e7135d9、4c0fb4e），每批 300s 门禁 PASS
- ~~探针目录~~：outb_probe*、run_smoke*、run_quick2、_probe_c、_probe_outb 已删
- ~~steady/ 子包~~：已删（Grid4D 时间墙问题，bts 为标准路径；引用核实为文档性）
- ~~README~~：目录树/运行要点已同步
- 附加：farm_control_runtime 的 build_windows.ps1 gcc 回退清单 + CMakeLists.txt 补全，**单测 39/39 恢复可运行**（此前根本编不过）

### 🟡 P6 尚未开始的阶段
- **Phase 4**：FLORIS 校准后直优 `a_cmd`（P0-6 主链，LUT 加 a_cmd 列）
- **Pilot / Full DOE**（486 / 2592）
- **DEL**：turbine OutList 缺载荷通道（RootMyb / RotThrust 等）需补；多 seed 镜像未做
- Timestep B（0.05 短盒）可选未做

---

## 四、建议的下一步顺序
1. ~~P1 yaw 生效性~~ ✅（1ee69bb）
2. ~~P2 非确定性~~ ✅ OMP=1 逐位可复现（f297121）；剩余：pilot 前试 OMP=2/3 速度折中；批跑器接线 feather_invalid 过滤
3. ~~P3 状态帧字段~~ ✅ 单位修复 + Ct/Cp 回填（f297121/4c0fb4e）；剩余：Induction 标定按 §一.4 作废结论重做（长 TTL + 稳态窗口径）
4. ~~P5 工程债~~ ✅（e7135d9/4c0fb4e）
5. **盒生成继续滚动**（U8 13/18：s5 对生成中 ~5.7h CPU、s6 三盒排队 2 轮 ≈14–18h，预计明日午前 18/18）；到 18/18 后启动 pilot 分片（1 worker，OMP 固定 + 顺桨过滤）
6. U6/U10/U12 盒补齐 → 全 DOE

---

*快照生成：2026-09-21 17:45；分支 dev/farm-control-runtime，最新提交 4c0fb4e（P1/P2/P3/P5 全部闭环，P4/P6 随盒生成推进）*

---

## 五、不依赖盒生成的开发窗口（2026-09-21 17:10 起）
用户指令：直接推进不需要等风速盒的任务。选定顺序：
1. 批跑器接线 `feather_invalid` 过滤（replay metadata 已备）
2. OMP=2/3 确定性/速度折中测试（pilot 时间预算依据）
3. WT*.fst 载荷通道追加工具（Phase 7 DEL 铺路，乙模式验证）
4. 磁盘方案决策支持（72 盒 ~103GB vs D: 剩余）

### 开发窗口执行结果（2026-09-22 上午）
1. **批跑器顺桨过滤** ✅（6a9d17c）：worker `FEATHERED` → status=feathered（resume 重试）+ feather_events.json
2. **OMP 折中实测** ✅：**只有 OMP=1 逐位确定**（4232.2834×2）；OMP=3 噪声 0.018%@300s（4194.63/4195.37，162s）——pilot 默认 OMP=1，OMP=3 记录为降级选项
3. **DEL 载荷通道** ✅（6a9d17c）：`run/add_load_channels.py` 幂等追加 TwrBs/LSSGag/YawBr 至 ElastoDyn OutList，已接 replay + 乙模式；300s 门禁 .outb 实测 31 通道（TwrBsMyt 28.4 MN·m）
4. **磁盘账本** ✅（4cee4eb）：72 盒 ≈55–60GB（旧 103GB 高估），D: 剩 98.3GB——单盘可行，保留策略：pilot 先行 + raw timeseries 用后清
5. **盒 QA（15 已完成 U8 盒）** ✅：14/15 pass；**TI15_seed03 realized TI=0.120 vs 目标 0.15**（超容差 0.02，TurbSim 高 TI seed 方差）——108 个 case 被 `--require-qa` 标 qa_failed；决策点：重造（7-9h，有夜间窗口）or 放行（多 seed 平均稀释），pilot 前定
6. **manifest 干跑** ✅：2592 行全 DOE；U8 648 = pending 504（14 好盒）+ qa_failed 108（TI15_s3）+ blocked 36（s6 排队）
7. **Induction 标定重做（gate 9，U8）** ✅ 完成（2026-09-22）：`_calib_worker` 加 `--settle 120` 显式窗口 + 600s 时长；旧 map 归档 `.INVALIDATED_ttl60s`。**8 点 map（TI10_s1，OMP=1）**：a 0.10→7.5kW(停机) / 0.12→202 / 0.15→711 / 0.17→963 / 0.20→1232 / 0.25→1474 / 0.30→1556 / 0.33→1568 kW；Ct/Cp/pitch/rpm 全列有效（Ct/Cp 由 replay 回填）。**关键发现：U8 有效降载域 a∈[0.12,0.33]，a=0.10 崩溃进 Region1 停机**（执行链 min_pitch=0.55rad×(1-pr) 在深降载时把桨距压到 14-15°→转速跌破 VS_MinOMSpd→GenTq=0 锁死）——**Phase 4 优化的 a_cmd 搜索域必须 clip 到 [a_min_eff(U), 0.33]**；U6/10/12 的 a_min_eff 待对应盒生成后标定
   - 执行链错配细节：Supervisor `power_ratio` 相对额定 5MW（pr=a(1-a)²/pe0），ConstPower P_ref 在 8m/s 常不可达 + min_pitch 主导执行——映射/执行语义的配合下界是风速相关的真实系统特性，标定即为此刻画

### 磁盘方案评估（2026-09-21 实测）
- 现状：U8 真盒 13 + uniform8 1 = 9.1 GB（692MB/盒@1800s、329MB@700s）；D: 剩余 98.3 GB；results 全部 1 GB（dbg2 诊断文件占 0.5GB，可清理）
- **72 盒真实需求 ≈ 55–60 GB**（U6@1950s ~1.0GB/盒、U8@1800s 0.69GB/盒、U10@1700s 0.65GB、U12@1650s 0.63GB；此前 "~103GB" 高估）
- 结论：**单盘可行但紧**——72 盒（~58GB）后 D: 剩 ~40GB；DOE results 增长（2592 case × timeseries.csv ~15-25MB ≈ 40-65GB）会溢出
- 建议：①先只生成 pilot 所需 U8 18/18（+5 盒 ~3GB）；②timeseries.csv 仅保留统计 summary，raw 时序跑完即清（或只留代表 case）；③U6/10/12 按需分批生成；④dbg2 诊断文件纳入 .gitignore/定期清理

---

## 六、ICD v1.5 场控接口升级 —— 影响评估与复验（2026-10-20）

> 工程在 2026-10 升级了场控接口至 **ICD v1.5**（交付 `farm_control_runtime_delivery_20261020`，
> 基线 `f920ceb`；新 DLL `DISCON_WT1.dll` 2359.4KB，含 S1101 保留接口 + AWAE 二跳 + Mode0/1/2/3
> 控制模式 + per-turbine control_mode authority）。**本段记录接口变更对研究侧测试的影响与复验结果。**

### 6.1 接口关键变化（影响研究侧）
1. **命令帧重定义**：`FarmCommandFrame`（per-turbine 槽位；`command_frame_seq` S0101、`command_basis_sim_time_s` S0102——基于仿真时刻，墙钟命令被拒 WIN-05；无状态帧前发的命令被拒）
2. **yaw_delta→绝对目标换算移入客户端**：`wfcrl/transport/farm_zmq_client.py` 的 `send_yaw_delta` 内部调 `yaw_delta_cw_to_absolute_ccw`（wrap_pm_pi(heading_now−delta)）；引擎 `send_yaw_delta` 签名兼容（研究侧 `_send_cmds` 无需改）
3. **控制模式 Mode0/1/2/3**：per-turbine control_mode 权威；研究侧 `_send_cmds` 走 yaw_delta/induction 通道（对应模式由 runtime 默认桥接），功能测试（同场对比）确认 induction/yaw 通道独立生效、无串扰
4. **状态帧扩展/改名**：S1002 sim_time、S1009 `nacelle_heading_actual_rad`、S1010 `rotor_aero_thrust_n`、S1011 estimated_wind、S1012 ct_hat、S1015-1017 turbine_status/wind_source/wind_quality；引擎 `_parse_state_frame` 保留旧键兼容研究侧（`nacelle_heading_rad`/`rotor_thrust_n` 均可用）✓
5. **诱导执行语义变化（W-01/W-02）**：r_pitch 纯桨距（不再隐式 VS_RtPwr 降载）+ PRC/PS/VS 配置全量 save-restore → **转速不再钉 VS_MinOMSpd**（实测 rpm 7.53→8.39-8.78 @a=0.2/0.3）→ **缓解旧 Region1 深降载崩溃风险**
6. **AWAE 二跳**（S04/S05→S12/S13）与 **6T 布局**（LayoutRegistry "6T" 2×3）新增；研究侧新增 `run/six_turbine_interface_test.py` 专门验证
7. **方法论**：交付文档强调 FAST.Farm 湍流混沌 → **同场对比**（baseline 与 test 同 run 同 BTS 同时刻差）；研究侧 OMP=1 已实现逐位确定性 → 跨 run 对比依然有效（P2 修复的价值更凸显）

### 6.2 复验结果（2026-10-20，新 DLL + OMP=1）
| 测试 | 旧 DLL 结果 | 新 ICD v1.5 DLL | 状态 |
|---|---|---|---|
| 300s 门禁（regress_300s） | 4232 kW | **4366 kW**（T-21 交叉印证 4364-4366） | ✅ PASS，基线 +3% |
| P1 yaw 保持（180s yaw=30） | −30.1°/T1 1135 kW | **−29.99°（seq=1 保持）/T1 1152 kW** | ✅ PASS |
| **P1 yaw 保持（1200s 全时长）** | −30.1° 保持至 1202s（旧 t≈524 曾顺桨） | **−29.99° 全程锁定至 1202s，无顺桨/无 stall** | ✅ PASS（旧异常消除） |
| **U8 Induction 标定（全 6 点重跑）** | a=0.10→7.5kW(停机)/0.20→1232/0.33→1568 | **a=0.10→1029（崩溃消除）/0.20→1389/0.33→1622 kW**；rpm 7.2-8.8（不再钉 MinOMSpd） | ✅ PASS（**a=0.10 Region1 崩溃已消除**，可用域扩展 [0.10,0.33]） |
| **P2 确定性（300s 双跑）** | 4232.2834×2 逐位一致 | **4366.1165×2 逐位一致（100/100）** | ✅ PASS（新 DLL 下 OMP=1 确定性保持） |
| 6T yaw 阶跃（T3/T4 +10°@200s） | —（旧无 6T） | T3 −9.99°/T4 −10.00°；P −12%/−3.8%；4 机零串扰 | ✅ PASS |
| 6T induction 阶跃（T3/T4 a=0.2@200s） | — | T3 −22.5%/T4 −16.1%；seq=1；零串扰 | ✅ PASS |
| identity（交付 v2.10.6） | — | zero_yaw −0.078%、induction_r1 PASS | ✅ PASS（交付团队已重验） |

### 6.3 需要调整/重新测试的项目（优先级排序）—— 2026-10-20 复验后状态
1. ✅ **U8 Induction 标定 map 全量重跑**（gate 9，已完成）：新 map `induction_operating_map_icd15.csv`（6 点，a 0.10→1029…0.33→1622 kW）；**a=0.10 Region1 崩溃已消除**（旧 7.5kW 停机 → 新 1029 kW），可用域扩展 [0.10, 0.33]；rpm 7.2-8.8 不再钉 MinOMSpd
2. ✅ **1200s 全时长重验**（已完成）：yaw=30 全程锁定 −29.99° 至 1202s，**无顺桨/无 stall**（旧 DLL t≈524 异常消失）；诱导深降载 stall 风险（交付 §7）在湍流阵风场景仍需 pilot 观察
3. ✅ **P2 确定性复验**（已完成）：新 DLL 下 OMP=1 双跑逐位一致（4366.1165×2，100/100）
4. ✅ **Identity/符号测试重跑**（已完成）：identity 交付 v2.10.6 PASS（zero_yaw −0.078%、induction_r1）；**yaw sign 新 DLL 重跑 PASS**（2026-10-20，WD=+10°：yaw −20→4534.7 / 0→4626.4 / +20→**4632.9 kW** farm，+20 最优与旧 DLL 一致（旧 4635），符号/排序未变；T1 降功率但 T2/T3 尾流偏转收益主导）
5. 🟢 **FLORIS 校准衔接**：新标定 map → Phase 4 a_cmd 直优（改用 `induction_operating_map_icd15.csv`；a_min_eff(U8)≈0.10 重新测定为 0.10）
6. ✅ **DEL 载荷通道**：新 DLL 下注入有效（31 通道实测）——无需改动
7. ⚪ **Pilot DOE**：前置全齐（U8 18/18 + 新 map + 确定性 + 全链 PASS）；待 TI15_s3 决策 + 启动

### 6.4 执行中的约束
- **不修改** `farm_control_runtime/`、`wfcrl/transport/`、`wfcrl/engine/`、`closed-loop-control/` 等一切场控接口相关代码（用户指令）——只从 `induction_vs_yaw_study/` 研究侧操作
- 每次代码改动后先跑 300s 门禁（用户规则）
- 内存规则：仿真与盒生成错峰；当前无 TurbSim，内存 3.9GB 可跑单仿真

### 6.6 全量风盒到位与 Full DOE 启动（2026-10-22）
- **72 盒全部到位**（U6/U8/U10/U12 × TI05/10/15 × seed01-06），共 48.5GB；D: 剩 37GB
- **QA 70/72 pass**；不合格 2 盒（均为 TI15、实测湍流偏低）：`U08_TI15_seed03`（0.1204 vs 0.15）、`U10_TI15_seed06`（0.1289 vs 0.15）→ 各 36 case 标 qa_failed
- **manifest 重建**：2592 行 = pending **2520** + qa_failed 72；分 U：U6 648 / U8 612 / U10 612 / U12 648；全部 pending 的 bts 引用有效
- **pilot 阶段结果**：61/459 case 完成（会话重启致作业中断，数据有效）；per-case 墙钟 **704s（11.7min）** 确认 OMP=3 生效
- **Full DOE 前置**（进行中）：U6/U10/U12 诱导标定（gate 9，18 点）→ 对应 LUT（gate 10）→ 全量 DOE（2520 case）
- **Full DOE 时长估算**（OMP=3）：2 workers ≈ **10 天**（2459 剩余 × 704s / 2）；3 workers ≈ 6.7 天但需 ~6.6GB（当前空闲仅 4.8GB，有浏览器卡死风险）

### 6.7 Full DOE 实施（2026-10-22 下午）
- **诱导标定全风速完成**（gate 9）：`induction_operating_map_icd15.csv` 共 24 点（4U × 6a）。U6 greedy 790 / U8 1622 / U10 3518 / U12 4359 kW；各 U 曲线单调 ✓
  - 注：U12 a=0.15/0.20 首次运行提前中断（state rows 不足 200 步→P=nan），重跑成功（3468/3949 kW）；`CalibrationCurve` 已加 NaN 行过滤
- **LUT 全网格完成**（gate 10）：4U × 3TI × 3WD × 4S = 144 case，合并 `luts.parquet` 864 行（yaw 432 / derating 432，a_cmd 100% 覆盖）；yaw 增益均值 5.79%（max 32.6%）、derating 均值 1.48%（max 16.9%）
- **修复 LUT seed 归一化（关键）**：LUT 与 seed 无关（case_id 固定 seed01），manifest 展开 seed01-06 → 此前 seed02-06 会报 "No LUT rows"（pilot 恰好只跑到 seed01 未暴露）。`normalize_case_id` 归一后 144/144 查表通过
- **DOE 内存约束（实测）**：引擎 `_ff_case` 全量读 bts 重采样，U06 盒（1950s）峰值 2.93GB、U08 2.7GB。TurbSim 重造 `U08_TI15_seed03` 期间内存紧张 → **U06 的 17 例 bts 读取 MemoryError**，U08/U10/U12 正常
- **当前运行**（TurbSim 停止后）：**全风速 DOE 已启动**（2449 待跑），1 worker + **OMP=6**；U06 已可正常跑（此前 17 例 bts 读取失败在 TurbSim 停止后解决）
  - 实测墙钟：**U08 ≈ 300s/case**、**U06 ≈ 691s/case**（低风速尾流平面更多 → 计算更重）
  - **2 workers 实测可行**（2026-10-22 17:05 起）：TurbSim 停止后可用 6.8GB，两路 bts 重采样峰值未触发 OOM；连续 21 例无新增失败（failed 仍 17，均为 TurbSim 占内存期间的 U06 历史失败）
- **预计（2 workers）**：剩余 2438 case ≈ **5.7 天**（实测 17.8 case/h）；残留风险：若两 worker 同时进入大盒（U06）重采样，峰值 ~7GB 可能触发 MemoryError → batch 自动重试（`max-retries 2`）+ `DONE.sentinel` 保证不丢数据

### 6.5 调整后的下一步（ICD v1.5 口径，2026-10-20 更新）
✅ 已完成：
- yaw sign 新 DLL 重跑 PASS（§6.3.4）
- **Phase 4 标定 a_cmd 直优框架**（`db4eb08`）：`optimize/calibrated_a_cmd.py`（map→a_cmd→ratio 校准曲线）+ `floris_derating_opt.optimize_derating_calibrated`（a_cmd 决策变量搜索）+ LUT `a_cmd` 列；U8 冒烟 PASS（a_cmd=[0.32,0.10,0.33]，gain +2.21%，T2 深降载让 T3 尾流恢复）
- **Phase 4 全 U8 pilot 网格 LUT 重建完成**（2026-10-20，27 case，`grid_u8_pilot.yaml`，162 行）：yaw 增益随间距递减（S4D 5.91%→S8D 1.95%）✓；derating 仅紧间距有益（S4D 0.78%、S8D -0.10%）✓；T2 为降载杠杆（a_cmd 0.10-0.32）
- **DEL 管线 + 端到端工具**（`1a14a34`、`18ddefd`）：`analysis/del_analysis.py`（rainflow+S-N DEL，正弦验证）+ `run/compute_del_batch.py`（批跑/outb_only/单 case 三输入，逐机逐通道 DEL+功率合并 → del_summary.csv）；gate outb 验证 10 通道物理合理
- **标定 a_cmd 全链路打通**（`4f98a3b`）：`_rows_to_dict` 透传 a_cmd + replay 直接下发标定值（LUT a_cmd=0.32/0.10/0.333 → Cmd induction_a）；batch filter 数值列强转修复
- **pilot 批跑全链验证通过**（2026-10-20）：manifest→worker→replay（标定 a_cmd）→新 DLL→状态帧→feather 过滤→status 记录→自动接续下一 case（baseline done 1771.8s，yaw 自动接续）；单 worker ~30min/case
- **吞吐配置落地**（`4798a06`）：`experiment.yaml` `batch.workers: 3` + 新增 `batch.omp_threads: 3`（批跑器注入 env → 贯穿 worker）；推荐 **OMP=3×3 → 486 case ~1.2 天**（RAM 7.5GB/9 线程；0.018% 噪声 vs OMP=1 逐位确定 5 天）
- **libzmq teardown 崩溃解耦**（`35087da`）：probe 发现 induction case 完整产出数据后在解释器关闭时被 libzmq 'Connection reset' assert 杀（returncode≠0→误标 failed）；worker 写 `DONE.sentinel`，batch 以 sentinel 为成功判据——不再浪费 pilot 重试
- **启动引号坑记录**：pwsh 会剥掉 filter 字符串内的双引号（`control=="induction"` 变 `control==induction` → pandas query 报错 → run unfiltered）。**Pilot 启动必须用 `--controls` 参数过滤 control 类型，filter 只用纯数值比较**

**Pilot 启动命令（推荐，规避引号坑）：**
```bash
python -m induction_vs_yaw_study.run.run_fastfarm_batch \
  --manifest induction_vs_yaw_study/results/_manifests/case_manifest.csv \
  --out induction_vs_yaw_study/results/run_pilot_u8 \
  --workers 3 --max-retries 2 \
  --controls baseline yaw induction \
  --filter 'U==8 & seed>=1 & seed<=6 & spacing_D>=4'   # 纯数值，无引号
# 注：OMP 由 experiment.yaml batch.omp_threads=3 自动注入（或 shell 显式
#     WFCRL_STUDY_OMP_NUM_THREADS=3 覆盖）；TI15_s3 决策前可先 --filter 排除 seed==3
```
- **FLORIS vs FAST.Farm yaw 偏差发现**（probe，WD0 S4D TI10）：LUT 预测 yaw +9.93%/derating +2.21%，FAST.Farm 实测 **yaw −2.9%/derating +1.3%**（窗 [600,1200]）——FLORIS 尾流偏转在 S4D 高估收益（T1 −24% 但 T3 恢复不足）；属研究设计的 model_bias 测量范围，pilot 将揭示真实幅度；yaw sign 仍正确（机舱按 LUT 转至 −24.4°/−28.1°/0°）

剩余（不依赖其他风速盒，可继续做）：
1. TI15_seed03 决策（重造 or 放行 or pilot 用 5 seed）
2. **Phase 4 全 U8 网格 LUT 重建**（144 工况，纯离线；U8 有 map）→ 覆盖 yaw+derating 两套 LUT 到新口径
3. **DEL rainflow 管线**（Phase 7 前置，`analysis/` 目前仅 std 代理）：用已有 31 通道 outb 写 rainflow+S-N 实现并验证（无需新仿真）
4. **Pilot 486 启动**（新 DLL 口径，OMP=1，顺桨过滤已备；注意 §7 诱导 stall 风险在湍流场景的观察）
5. U6/10/12 盒（其他电脑生成中）→ 其余风速标定（`CalibrationCurve.load` 自动识别）→ Phase 4 补该风速 → Full DOE
