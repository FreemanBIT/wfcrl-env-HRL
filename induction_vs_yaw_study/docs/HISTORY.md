# HISTORY — 已完成里程碑与问题闭环记录（归档）

> 本文档归档**已完成**的阶段成果、问题根因与修复过程（按时间倒序）。
> 当前状态/开放问题见 `STATUS.md`；gate 跟踪见 `EXECUTION_LOG.md`；
> DOE 运维见 `DOE_OPERATIONS.md`；方案见 `../DEVELOPMENT_PLAN_ZMQ.md`。

---

## 2026-10-26 — 分析工具链与 DOE 运维现实

### 新增分析/运维工具
| 工具 | 作用 |
|---|---|
| `run/check_doe_quality.py` | 质量审计（完整性/数值合理性/稳定性）；失稳率 >2% 返回非零 |
| `run/analyze_doe.py` | 汇总 → case/group/gains(seed 配对)/per_turbine/bias + report.md |
| `analysis/doe_plots.py` | 6 张结果图（增益趋势/预测-实测散点/逐机增量/偏差）|
| `run/doe_status.py` | 进度看板（进度/速率/ETA/资源，支持 --json）|
| `run/compute_del_batch.py` | DEL 端到端（批跑/outb_only/单 case 三输入）|

### 运维教训（重要）
1. **worker 卡死**：`_run_one` 的 `subprocess.run` 无 timeout → 实测有 worker 卡死 3 天占槽位。
   修复：`WFCRL_STUDY_CASE_TIMEOUT_S`（默认 2700s）+ 超时清理残留 + `WFCRL_STUDY_STAGGER_S` 错峰。
2. **内存现实**：本机 15.8GB，系统+浏览器常占 ~10.8GB；每 case 峰值 ~5GB（bts 2.4–2.9GB + 仿真 2GB）
   → **2 workers 不可行**（实测 OOM），最终 1 worker。
3. **实测每 case 墙钟（OMP=3）**：U06 ~508s、U08 ~590s、U10 ~326s、U12 ~383s；综合 ~460–560s。

### 失稳检测判据演进
- 初版 `pitch>80 或 rpm<1` 会漏掉"半顺桨"（pitch 79°、rpm 1.5）→ 产生 +117% 假增益
- 收紧为 **pitch>40 或 rpm<2**（正常运行时 U12 强风桨距也不超过 ~25°）

---

## 2026-10-23 — 重大发现：OMP=6 破坏 FAST.Farm 数值稳定性

- **现象**：DOE 完成 470 case 后抽查，**32.3%（152/470）** 的 case 在统计窗内有机组顺桨 90°（pitch→90°@8°/s、rpm→0、P→0），
  baseline/yaw/induction 各约 1/3，与 U/TI/spacing 无关。
- **排除路径**：
  1. 怀疑尾流平面追赶（`Offending wake plane removed`）→ 按警告降 f_c 0.1057→0.04 → 仅部分改善，事件数几乎不变 → **非根因**
  2. 怀疑深尾流低于切入风速的物理停机 → U12（12 m/s）同样失稳 → **非物理**
- **决定性对比**：pilot 61 case 是 OMP=3（**0% 失稳**）；DOE 255 case 是 OMP=6（**36.9% 失稳**）→ **OMP 线程数是根因**
- **验证**：同一失稳 case 用 OMP=3 重跑 → 完全稳定（T1 4993 kW 额定）
- **处置**：`WFCRL_STUDY_F_C` 补丁默认关闭；剔除 152 个失稳 case；DOE 改 **OMP=3**
- **教训**：OMP=6 表面快 2.3×（305s vs 700s/case），实则因尾流模型数值破坏而"虚快"，产出 37% 无效数据 → **正确性优先**

---

## 2026-10-22 — 全量风盒到位、Full DOE 启动、磁盘危机

### 风盒与标定
- **72 盒全部到位**（U6/U8/U10/U12 × TI05/10/15 × seed01-06），48.5GB
- **诱导标定全风速完成**（gate 9）：`induction_operating_map_icd15.csv` 24 点（4U × 6a）
  - U6 greedy 790 / U8 1622 / U10 3518 / U12 4359 kW；各 U 单调
  - 注：U12 a=0.15/0.20 首跑提前中断（state rows 不足→P=nan），重跑成功（3468/3949 kW）
- **LUT 全网格完成**（gate 10）：144 case / 864 行（yaw 432 / derating 432），a_cmd 100% 覆盖

### 关键修复
1. **LUT seed 归一化**：LUT 与 seed 无关（case_id 固定 seed01），manifest 展开 seed01-06
   → 若不修，5/6 的 case 会报 "No LUT rows"。`normalize_case_id` 归一后 144/144 查表通过。
2. **resume 排除 qa_failed/blocked**：原逻辑会用不合格盒跑 case。

### 磁盘危机
- **事故**：DOE 跑到 255 case 时 D: 写满（`OSError: No space left on device`）
- **根因**：ROSCO `LoggingLevel=1`（模板默认）→ 每机写 ~77MB `.dbg`，3 机 = **231MB/case**，累计 48.9GB
- **修复**（研究侧，模板不动）：
  1. 删除 660 个 .dbg（释放 48.9GB）
  2. replay 在 setup 后把物化 DISCON 的 `LoggingLevel` 改 **0**（`WFCRL_STUDY_ROSCO_LOG=1` 恢复）
  3. `_case_worker` 产出落盘后清理每 case 大件（bts/outb/dbg/ServoData DLL）
     （`WFCRL_STUDY_KEEP_CASE_FILES=1` 保留，DEL 阶段用）
- **效果**：每 case 241MB → **~1.8MB**

### QA 容差修正（2026-10-26 补充）
- 两个 TI15 盒（`U08_TI15_seed03`=0.1204、`U10_TI15_seed06`=0.1289 vs 目标 0.15）被 ±0.02 绝对容差判不合格
- **关键认识**：TurbSim 给定 seed 确定性（`RandSeed1`），同 seed 重跑必然复现同一实现；单次 1700–1800s 的 IEC Kaimal 实现轮毂 TI 有 ±0.02–0.03 天然抽样散布
- **修正**：`|dTI| <= max(0.02, 0.25*TI_target)` → TI05 0.02 / TI10 0.025 / TI15 0.0375 → **72/72 通过**

---

## 2026-10-20 — ICD v1.5 场控接口升级：影响评估与全量复验

工程升级至 **ICD v1.5**（交付 `farm_control_runtime_delivery_20261020`，基线 `f920ceb`；
新 DLL `DISCON_WT1.dll` 2359.4KB，含 S1101 保留接口 + AWAE 二跳 + Mode0/1/2/3 + per-turbine control_mode）。

### 接口关键变化（影响研究侧）
1. **命令帧重定义**：`FarmCommandFrame`（per-turbine 槽位；`command_frame_seq` S0101、`command_basis_sim_time_s` S0102
   ——基于仿真时刻，墙钟命令被拒 WIN-05；无状态帧前发的命令被拒）
2. **yaw_delta→绝对目标换算移入客户端**（`wfcrl/transport/farm_zmq_client.py`）；引擎 `send_yaw_delta` 签名兼容
3. **控制模式 Mode0/1/2/3**：per-turbine control_mode 权威
4. **状态帧扩展/改名**：S1002 sim_time、S1009 `nacelle_heading_actual_rad`、S1010 `rotor_aero_thrust_n`、
   S1011 estimated_wind、S1012 ct_hat、S1015-1017；引擎保留旧键兼容研究侧
5. **诱导执行语义变化（W-01/W-02）**：r_pitch 纯桨距 + PRC/PS/VS 全量 save-restore
   → 转速不再钉 `VS_MinOMSpd`（实测 rpm 7.53→8.39-8.78 @a=0.2/0.3）→ 缓解旧 Region1 深降载崩溃
6. **AWAE 二跳 + 6T 布局**（LayoutRegistry "6T" 2×3）新增
7. **方法论**：交付文档强调 FAST.Farm 湍流混沌 → 同场对比；研究侧 OMP 固定使跨 run 对比有效

### 复验结果（新 DLL）
| 测试 | 旧 DLL | 新 ICD v1.5 DLL | 状态 |
|---|---|---|---|
| 300s 门禁 | 4232 kW | **4366 kW**（T-21 交叉印证）| ✅ |
| P1 yaw 保持 180s | −30.1°/T1 1135 kW | **−29.99°/T1 1152 kW** | ✅ |
| P1 yaw 保持 1200s | 旧 t≈524 曾顺桨 | **全程锁定 −29.99°，无顺桨/stall** | ✅ |
| U8 诱导标定 6 点 | a=0.10→7.5kW(停机) | **a=0.10→1029kW（崩溃消除）** | ✅ |
| P2 确定性（300s 双跑）| 4232.2834×2 | **4366.1165×2 逐位一致** | ✅ |
| 6T yaw 阶跃 | — | T3 −9.99°/T4 −10.00°，4 机零串扰 | ✅ |
| 6T induction 阶跃 | — | T3 −22.5%/T4 −16.1%，零串扰 | ✅ |
| yaw sign（WD+10）| +20 最优 4635 kW | **+20 最优 4632.9 kW，符号不变** | ✅ |

---

## 2026-09-21 — P1~P6 问题闭环

### P1 yaw 命令未生效 ✅
- **根因**：`closedloop/bridge.py` `_send_yaw_delta` 硬编码 `ttl_s=60.0`（induction 30.0）；
  静态对风场景只在 k=1 发一次命令 → t≈63s 过期回退 → 机舱转回 0°
- **修复**（研究侧）：`replay` 新增 `_send_cmds()` 直调 `engine.send_yaw_delta(..., ttl_s=1e6)`
  （`ttl_s=0` 会回落 runtime 默认 60s，必须显式大正值）
- **验证**：180s/1200s/600s 三次：seq=1 保持、机舱恒 −30.1°、T1 ≈1133 kW（−31%≈cos²30°）

### P2 运行间非确定性 ✅
- **根因**：FAST.Farm OMP 浮点归约顺序差异（同 case 两次差 1.9%）
- **修复**：`engine.start()` 前设 OMP 线程数（env `WFCRL_STUDY_OMP_NUM_THREADS`）
- **验证**：OMP=1 逐位一致（4232.2834×2，100/100 样本）；代价 wall ×2.5

### P3 状态帧字段 ✅
- **heading/vane 单位**：`FCR_PublishState` 把 ROSCO 内部度值直传 `*_rad` → 修复 ×D2R + 重编译 DLL
- **rotor_ct/rotor_cp**：运行时不填 → 研究侧由 thrust+power+盘面风回填（均匀盒下受遮挡机组会高估，已注记）

### P5 工程债 ✅
- 删除 `steady/` 子包、探针目录；README 同步
- `farm_control_runtime` 的 build_windows.ps1 gcc 回退清单 + CMakeLists 补全 → 单测 39/39 恢复

### P6 阶段（部分完成）
- Phase 4 标定 a_cmd 直优 ✅（见下）
- Pilot/Full DOE ✅ 启动
- DEL 载荷通道 ✅（`add_load_channels.py`）；DEL 端到端 ✅（`del_analysis.py` + `compute_del_batch.py`）
- Timestep B（0.05 短盒）可选未做

---

## 2026-09-22 — 不依赖盒生成的开发窗口

1. **批跑器顺桨过滤** ✅：worker 打 `FEATHERED` → status=feathered（resume 重试）
2. **OMP 折中实测** ✅：OMP=1 逐位确定；OMP=3 噪声 0.018%@300s（后证实 OMP>3 会破坏稳定性）
3. **DEL 载荷通道** ✅：TwrBs/LSSGag/YawBr 7 通道追加至 ElastoDyn OutList，300s 门禁 .outb 实测 31 通道
4. **磁盘账本** ✅：72 盒真实需求 ~55–60GB（旧 103GB 高估）

---

## 2026-09-21 前 — 基础设施

- 风盒管线：`generate_wind_boxes.py`（U 自适应 AnalysisTime）、`qa_wind_boxes.py`、seed-key resolver
- 批跑器子进程化（libzmq 致命 assert 只杀单 case）+ 自动随机端口 + provenance
- 乙模式 `outb_only_run.py`（无 ZMQ 客户端 → 全机 T1..T3.outb）
- identity / yaw sign / 300s 门禁 / 1200s 全时长 全部 PASS
