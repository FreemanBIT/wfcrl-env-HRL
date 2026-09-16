# FAST.Farm 三机串列 · Yaw vs Induction 测试开发方案（V3，按审查修订）

> 输入：方案 V2 + 用户约束（1 风向 0/10/20；2 产出全在 study；3 旧 bts 删除；4 加 12m/s；5 不改 wfcrl）
> 审查：FASTFarm_Development_Plan_Review_Issues.md（8 P0 / 7 P1 / 5 P2）
> **核心原则（审查 §24）：先证明控制语义 -> 再证明单 case identity -> 再做多 seed pilot -> 最后才 full DOE。**

---

## 0. 修订记录（相对 V2 的变化）

| 审查项 | 修订 |
|---|---|
| P0-1 时间尺度矛盾 | 明确三时间尺度；新增 Timestep Compatibility Test 为 full-DOE gate；纠正 "DT_TurbSim<=DT_High" 错误表述（§4） |
| P0-2 UsableTime/periodicity | 统一为"非周期 + 足长 AnalysisTime + UsableTime=ALL（全时长可查询，不依赖 wrap）"唯一语义（§4.2） |
| P0-3 yaw 坐标双计 | 确立坐标原则：旋转法下 inflow 恒 270°，baseline 对齐目标不随 WD 变；新增 layout-rotation 测试与 yaw-sign gate（§7） |
| P0-4 baseline 语义 | baseline = no supervisory override（唯一）；identity 分"no-override vs yaw0"与"no-override vs greedy/sentinel"两个测试（§8） |
| P0-5 n_control_steps | 删除独立配置；t_sim_s/t_settle_s/control_dt_s 派生计数（§9） |
| P0-6 induction 迁移 | 主链路改为 1T 标定 a_cmd -> FLORIS 优化 a_cmd* -> LUT 存 a_cmd -> FAST.Farm 直接下发 a_cmd；ratio/MW 链降级为诊断（§10） |
| P0-7 负 WD | full DOE 保留 0/10/20；sanity 必须含 −10/0/+10 mirror 验证（多 seed 统计镜像对称）（§11） |
| P0-8 direct optimization | 分层搜索（7×7 粗 -> ±10°/5° 局部 -> 可选 surrogate）；新增 Phase 5b 代表 case 直优；Regret 仅 5b 后可算（§12/§17） |
| P1-1 12m/s | pilot 增加 above-rated mini-pilot（U12/TI10/WD{0,10}/S{4D,6D}/seed1..3）（§16） |
| P1-2 YAML 机器可读 | wind_conditions.yaml 全字段显式（含 coherence 数值、turbulence_model 命名）（§4.1） |
| P1-3 BTS 复制 | 全部 shared reference（现有 engine 已只引用源路径，不复制）；批跑记录 reference_mode（§13） |
| P1-4 进程清理 | 一律 PID/process-tree scoped，禁止全局 kill FAST.Farm（§13） |
| P1-5 manifest 并发 | 增量 append + 锁；主 manifest 由协调者独占（§13） |
| P1-6 weighted energy | 新增 Phase 5c；无风玫瑰时只报 equal-weight case-average，每个 spacing 单独报告（§18） |
| P2-1 provenance | run/case metadata 记录 git commit、exe/bts/config hash、版本、约定版本（§19） |
| P2-2 turbine-level 诊断 | 每 case 保存 P1..3、Ct、a_cmd、actual yaw、effective inflow（ZMQ 已含） |

---

## 1. 现状（截至 V3 修订日，已实现并验证）

- **架构**：ZeroMQ 闭环（Controller -> Cmd -> ZmqFarmBridge -> FastFarmZmqEngine -> FAST.Farm v5.0.0）；状态帧（全机）经 ZMQ 回传。
- **统计口径**：ZMQ 状态帧（每控制步，全 3 机）——因本集成 FAST.Farm 仅写 Case.T1.outb（T2/T3 无 .outb），已弃用 .outb 作为功率统计（313d0e0）；.outb 仅诊断；DEL 阶段需另行解决多机载荷通道（开放项）。
- **门禁**：300s 回归 PASS（uniform 与真实湍流盒两轮；真实盒 3 机 farm ≈ 3.86 MW）。
- **风盒**：seed 键生成器（(U,TI,seed) 唯一；按 U 自适应 AnalysisTime：U6=1950/U8=1800/U10=1700/U12=1650s；dt=0.1s；不限时 TurbSim）；72 个 .inp；QA 达标（TI05/TI10 seed1 realized U/TI 通过）。
- **FLORIS**：快速 LUT（144 工况 × 2 控制，864 行，--quick：粗网格 0.1 + 无细化）。
- **测试**：43 passed（含显式统计窗口、BTS 复用断言）。
- **批跑器**：manifest 驱动、增量状态、断点续跑、并发 worker（端口对隔离）、induction<=>derating 映射（run_fastfarm_batch.py）。
- **冒烟验证**：9 case（U8/TI10/seeds1-3/WD0/S4D × 3 控制）1200s 全时长仿真约 610-620s 墙/个跑通 —— 1800s 盒对 1200s 仿真时间覆盖验证通过；1800s AnalysisTime 无需再加长。

---

## 2. 用户约束（不变）

1. 风向 full DOE 仅 {0, 10, 20}（负侧保留于 sanity/校验，见 P0-7）。
2. 测试产出与配置全在 D:\\HR_Project\\wfcrl-env-HRL\\induction_vs_yaw_study\\ 下。
3. 不依赖任何旧 .bts（用户已删除）；新盒按 (U,TI,seed) 生成。
4. 风速 {6, 8, 10, 12}。
5. 不修改 wfcrl/、closed-loop-control/（只读依赖；注入点 WFCRL_FARMINPUTS_DIR + 显式 output_dir）。

---

## 3. 目录与命名（不变，+ provenance）

```
induction_vs_yaw_study/
├── config/            # wind_conditions.yaml / experiment.yaml / seed_manifest.csv
├── FarmInputs/        # coarse/（NEW: seed 键盒；shared reference，不复制进 case） fine/ qa/
├── modules/           # 生成器/QA/resolver/manifest/batch/calib/identity/regress/analyze
├── results/
│   └── run_<id>/
│       ├── run_config.yaml + git_commit.txt + provenance.json（exe/bts/config hash、版本）
│       ├── case_manifest.csv（协调者独占更新）
│       └── cases/<case_id>/metadata.json（P2-1 字段）
└── tests/
```

---

## 4. 时间尺度与风盒语义（P0-1/P0-2/P1-2）

### 4.1 三个时间尺度（用户 2026-09-16 指令：DT_High=0.1 与 TurbSim timestep=0.1 对齐）

| 尺度 | 值 | 说明/约束 |
|---|---|---|
| FAST.Farm DT_High | **0.1 s**（= bts dt） | openfast_toolbox 行为 dT_High = dT_Box：**DT_High 恒等于 .bts 时间步**（模板默认亦 0.1）；DT_Low=3.0（控制步）|
| Turbine FAST 步/ROSCO/FCR 内部 | 0.01 s（10ms） | 集成 DLL 内部；不在本研究触碰 |
| TurbSim .bts dt | **0.1 s** | 与 DT_High 完全一致：InflowWind 无插值、逐帧对应（用户指令） |

**结论**：当前配置已满足用户指令（3T/6T 全部门禁、1200s 冒烟、identity、yaw-sign 均在此配置下通过）。
- 审查 P0-1 的"DT_TurbSim <= DT_High 矛盾"不再成立：0.1 == 0.1，且 DT_High 由 dT_Box 决定（改 bts dt 会自动跟随）。
- 历史"DT_High=0.01/301 帧窗口"分析基于旧假设，作废；v5 稳态风 ~260s 硬墙现象与 bts 无关（前者仅影响 WindType=1，本研究全部用 bts）。
- 若未来需要 bts dt=0.05（DEL 精化候选），DT_High 自动变为 0.05（工具箱行为），无需手工改。

### 4.2 TurbSim UsableTime 唯一语义

采用**一种**一致语义（P0-2）：
```text
UsableTime = "ALL"  ->  TurbSim 输出整个 AnalysisTime（按 U 自适应计算，
                        >= 查询上限 = Tsim(1200) + max_downstream_span/U + margin）；
文件为非周期全时长盒；FAST.Farm 查询时刻全部落在文件内，不依赖 MODULO wrap。
periodic 仅作为未来多周期方案的备选（RandSeed1=0），本研究不使用。
```
（数值 usable=1200 方案可作为可选替代，但需确认 TurbSim 输出长度语义后再采用。）

### 4.3 wind_conditions.yaml（完整机器可读，P1-2）

```yaml
wind_speeds_ms: [6.0, 8.0, 10.0, 12.0]
turbulence_intensities: [0.05, 0.10, 0.15]
seeds: [1, 2, 3, 4, 5, 6]

doe:
  t_sim_s: 1200.0
  t_settle_s: 600.0
  control_dt_s: 3.0
  margin_s: 120.0
  max_spacing_D: 8.0

turbsim:
  turbulence_model: IECKAI
  iec_wind_type: NTM
  scale_iec: 1
  hub_height_ref_m: 90.0
  artificial_hub_height_m: 175.0
  grid_width_m: 1500.0
  grid_height_m: 340.0
  grid_bottom_m: 5.0
  target_grid_spacing_m: 9.0
  num_grid_z: 40
  analysis_time_s: 1600.0       # 下限；生成器按 doe + U 自适应（U6=1950/8=1800/10=1700/12=1650）
  usable_time: "ALL"            # 语义见 §4.2
  time_step_s: 0.10             # 实测 .bts dt=0.1s（10Hz）
  vertical_flow_angle_deg: 0.0
  horizontal_flow_angle_deg: 0.0
  shear_power_law_exponent: 0.0
  coherence:
    scmod_1: IEC
    scmod_2: IEC
    scmod_3: IEC
    incdec_1: "12.0 0.00035273"
    incdec_2: "12.0 0.00035273"
    incdec_3: "12.0 0.00035273"
    coh_exp: 0.0
```

### 4.4 Timestep Compatibility（P0-1 已满足；保留记录）

- 默认档（A）：bts dt = DT_High = 0.1（用户指令）——**已全程验证**（identity/1200s 冒烟/yaw-sign/门禁全部 PASS）。
- B 档（bts dt=0.05）：工具箱会让 DT_High 自动跟随 0.05；仅作为 DEL 精化候选，本 DOE 不需要（跨越网格 @0.1 对功率统计充分）。
- C 档（0.01）：TurbSim 成本不可行，N/A。
- 验收记录：A 档为 coarse 默认；若做 B，用 timestep_test.py（1T/300s）对比 mean P 差异并记录。

---

## 5. 风盒管线（P0-2 语义 + P1-3 共享引用）

- (U,TI,seed) 唯一键；命名 inflow_U{U:02d}_TI{TI:02d}_seed{seed:02d}.bts；不含 spacing/WD/control。
- 固定 seed manifest（11001..66006）。
- **只生成 pilot 所需盒先行**（U8 全 TI×seed 18 盒），不一次性生成 72 盒（P1-3/compute 预算）。
- QA：轮毂位 (z=90,y=0) 实测 U/TI/σ（验收 5%/0.02）。
- **BTS 共享引用**：engine 只引用 FarmInputs/coarse 源路径（已验证不复制）；批跑记录 bts_source_path + reference_mode=shared（P1-3）。

---

## 6. 控制通道与坐标约定（P0-3/P0-4）

### 6.1 坐标原则（P0-3）

```text
风场旋转法：FAST.Farm inflow 方向恒为 270 度；WD offset 通过旋转 layout 体现。
因此 baseline nacelle 对齐目标 = 270 度（不随 case WD 变化）。
yaw 控制统一为失准角语义：
    yaw_delta（直接动作通道，CW+）= yaw_misalignment        （首选）
    或绝对目标 = 270 + yaw_misalignment（%360）            （等价，需保持基准恒 270）
禁止出现 (270 - wd) + mis 之类把 WD 再计入的写法（wd 已由 layout 旋转表达）。
```

新增测试：
- test_layout_rotation_does_not_change_baseline_yaw_target：WD∈{0,10,20}，yaw_mis=0 时发送的 aligned yaw target 必须一致（=270 度）。
- **yaw sign gate**：WD ∈ {−10, 0, +10} × yaw ∈ {−20, 0, +20}；输出 requested/actual yaw、farm P、WT2/WT3 有效入流、（可用时）尾流横向位移；记录 positive yaw → 转子法向/尾流偏向，与 FLORIS convention 对齐；不 PASS 不跑 yaw DOE。

### 6.2 Baseline 定义（P0-4，唯一）

```text
baseline = no supervisory override：
    yaw_override   = None（机组按初始对齐=270 度自由运行）
    induction_override = None（ROSCO 贪婪）
```
- 禁止把 "induction_a=1/3" 或 "ratio=1" 作为 baseline 定义（那是协议 sentinel/贪婪工作点，不是无监督）。
- identity 两个测试：
  A. baseline(no-override) vs yaw=[0,0,0]     （bridge 层等价性）
  B. baseline(no-override) vs induction greedy/sentinel 命令（通道激活等价性）
  验收：farm 均值功率差 < 0.5%，且 **T1/T2/T3 分别** < 0.5%。

### 6.3 时长参数派生（P0-5）

删除独立 n_control_steps 配置；只保留 t_sim_s / t_settle_s / control_dt_s：
```python
n_total_control_steps = round(t_sim_s / control_dt_s)                # 1200/3 = 400
n_statistics_steps    = round((t_sim_s - t_settle_s) / control_dt_s)  # 200
```
Case/metadata 记录 t_sim_s、t_settle_s、control_dt_s 及两个派生计数（n_* 不得独立配置）。

### 6.4 Cmd 通道（不变）

Cmd{turbine_id, yaw_delta_deg, yaw_deg, induction_a}；bridge 换算 yaw_delta（CW+）、_send_induction（a，>=A_GREEDY-1e-3 跳过=贪婪哨兵）。

---

## 7. 控制标定与跨模型迁移（P0-6）

### 7.1 Induction 主链路（修订后）

```text
OpenFAST/FAST.Farm 1T 标定：(U, a_cmd) -> (P, Ct, Cp, Pitch, RotorSpeed)
        |
        v
FLORIS 校准 turbine control surface（用该 map，不再用 Betz 反演）
        |
        v
FLORIS 优化 a_cmd*（T1,T2[,T3=1/3]）
        |
        v
LUT 存 a_cmd_T1/T2/T3（+诊断 Ct/P）
        |
        v
FAST.Farm replay 直接下发同一 a_cmd（bridge._send_induction）
```
- 控制变量迁移结构对称：yaw γ* -> γ_FF；induction a* -> a_FF。
- ratio -> P_target -> a 的旧链降级为 diagnostic/legacy（不用于主实验结论）。
- LUT schema 增加 a_cmd 列；a_cmd=1/3（贪婪 sentinel）必须可恢复 no-override（identity B 验证）。

### 7.2 Yaw 主链路（不变）

FLORIS 优化失准角 γ* -> LUT 存 γ* -> FAST.Farm replay 同 γ*（yaw_mode static）。

---

## 8. 指标与统计（P1-6/P2-2 + 既有）

- 显式窗口 [t_settle, t_sim]（已落地，metrics.py）；paired-seed 聚合（先逐 seed ΔP 再聚合 mean/std/SE/95%CI）。
- η_wake、ΔP_farm + WT1/2/3 分机增益（上游牺牲/下游恢复）。
- **turbine-level 诊断（P2-2）**：每 case 保存 P1..3、Ct1..3（state 有 rotor_ct）、a_cmd、actual yaw、effective inflow（disk_ambient/nacelle）——ZMQ 状态均已含。
- **迁移指标**：η_transfer（ΔP_FF/ΔP_FLO，仅 ΔP_FLO>=0.5% 时）。
- **能量（P1-6/Phase 5c）**：有 site 风玫瑰 -> AEP(S)=Σ p(U,WD,TI)·P̄ 分 spacing 报告；无 -> 只称 "equal-weight case-average power gain"，禁止称 AEP。
- Regret：仅 Phase 5b（FAST.Farm 直优）后可算。

---

## 9. Sanity 与镜像（P0-7）

- full DOE：WD ∈ {0, 10, 20}。
- **Sanity 必须含 WD ∈ {−10, 0, +10}**（yaw sign + mirror symmetry validation）。
- Mirror 指标：P(+10, γ) vs P(−10, −γ) 多 seed 统计平均（不要求单 seed 相等）。
- 措辞（替代"负风向与正风向对称……"）：当前对称三机布局 + 横向统计均匀入流下，正负 WD 统计上镜像对称；full DOE 仅保留正侧，负侧用于 yaw-sign 与 mirror validation。

---

## 10. 开发阶段（审查 §19 修订版）

### Phase 0 — Configuration semantics（先做，不跑大仿真）
1. 时间尺度/UsableTime 语义确认（含 Timestep Compatibility Test A/B/C，1T）。
2. yaw 坐标约定固化 + layout-rotation baseline yaw 测试。
3. baseline no-override 语义 + identity 两测试设计。
4. 时长参数派生改造（t_sim/t_settle/control_dt）。
> 里程碑：P0-1..P0-5 gate 对应项落地。

### Phase 1 — Wind pipeline（只生成 pilot 盒）
seed manifest / 统一盒（先 U8 18 盒）/ resolver / QA / manifest；BTS 共享引用确认。

### Phase 2 — Control sanity（身份/符号/镜像）
- zero-yaw identity、no-induction-override baseline identity、greedy/sentinel identity（均 T1..3 分机 <0.5%）
- yaw sign gate（WD −10/0/+10 × yaw −20/0/+20）
- mirror symmetry sanity
> 在 identity/yaw-sign 全 PASS 前不得跑完整 yaw/induction DOE（硬 gate）。

### Phase 3 — Operating-point calibration
- Induction：1T × U{6,8,10,12} × a_cmd sweep -> (P, Ct, Cp, Pitch, RotorSpeed) operating map（校准）。
- Yaw：确认 sign/机舱实际响应/尾流横向方向。

### Phase 4 — Calibrated FLORIS optimization
- Yaw：优化 γ*；Induction：**直接优化校准后的 a_cmd***；生成两类 LUT（含 a_cmd 列）。

### Phase 5 — Pilot DOE
- 主 pilot：U=8 × TI{5,10,15}% × WD{0,10,20} × S{4,6,8}D × seed1..6 × 3 controls（486）。
- 额外 above-rated mini-pilot（P1-1）：U=12 × TI10% × WD{0,10} × S{4,6}D × seed1..3 × 3 controls（36）：验证 pitch 调级区行为。

### Phase 5b — Representative FAST.Farm Direct Optimization（P0-8）
- 12–20 代表 case（aligned/partial wake、低/高 TI、4D/8D、低于/高于额定）。
- 分层搜索：粗 7×7（yaw ∈ {−30..30}/10°） -> 局部 ±10° step5°（约 5²） -> 可选 surrogate。
- Induction 用校准 a_cmd 网格同样 coarse->local。
- 得到 u*_FF；此后才计算 Regret。

### Phase 5c — Weighted Energy（P1-6）
有 wind rose -> AEP 分 spacing；无 -> equal-weight case-average（不称 AEP）。

### Phase 6 — Full DOE
仅当 §11 gate 全部 PASS。

### Phase 7 — Fine-grid DEL
代表 case；raw .outb 高频 + rainflow + component-specific S-N；Power-Load Pareto。
> 开放项：多机 .outb 缺失导致 DEL 通道受限（T1 only）——fine 阶段需解决（可能需在许可范围内改变案例输出设置或改用可用载荷通道），未解前 DEL 结论不可下。

---

## 11. Full DOE 前硬性 Gate（审查 §20）

```text
[ ] timestep compatibility test（A/B/C）通过且结论记录
[ ] BTS usable-time 语义一致（ALL=全时长可查、非周期、无 wrap 依赖）
[ ] same (U,TI,seed) -> same BTS（复用断言）
[ ] zero-yaw identity（farm + T1..3 < 0.5%）
[ ] baseline no-override identity（T1..3 < 0.5%）
[ ] greedy/sentinel induction identity（T1..3 < 0.5%）
[ ] yaw sign verified（WD -10/0/+10 x yaw ±20）
[ ] WD mirror sanity checked
[ ] induction operating map generated（1T 标定）
[ ] FLORIS induction LUT 存 calibrated a_cmd（不是 ratio 主链）
[ ] 显式 t_sim/t_settle 窗口生效
[ ] worker 进程清理为 PID-scope（不禁全局 kill）
[ ] manifest 更新并发安全（append+锁 / 协调者独占）
[ ] BTS 共享引用（无 per-case GB 复制）
[ ] pilot DOE 通过
```
任一项未过：STOP，不 launch full DOE。

---

## 12. 工作量/存储/并行预算（更新）

| 项 | 量 | 估算 |
|---|---|---|
| pilot U8 盒 | 18 × ~0.7GB | ~12.6GB（已按实测 dt=0.1） |
| 全 DOE 盒 | 72（按需生成，非一次性） | ~50-60GB 峰值 |
| 1200s 仿真 | ~610-620s 墙/个（3T 实测） | pilot 486 ≈ 4 worker × ~2.5 天 |
| above-rated mini-pilot | 36 | +1/8 pilot |
| Timestep test | 3×1T×300s | ~10 min |
| 标定 | 1T 4U×6a + yaw sign 9 次 | ~1.5-2h |
| Direct opt（5b） | 12-20 case × (49+25) runs | 代表 case 直优量级 ~10^3 runs（分层） |

---

## 13. 开放问题 / 风险

1. 多机 .outb 缺失：功率已经 ZMQ 测量解决；DEL 载荷通道受限（T1 only）——fine 阶段须解决，否则 DEL 结论受限（记录在案）。
2. TurbSim 单盒时长 ~4h+ CPU：并行策略 + 按需生成（先 pilot）已纳入。
3. 内存 15.8GB：仿真实例与盒生成需错峰调度（已实践：暂停盒生成跑批）。
4. 12m/s Region III：mini-pilot 先验证再决定权重。
5. FLORIS 全精度 LUT（step0.05+细化）成本高：以校准 a_cmd 直优 + --quick 策略分阶段。

---

## 14. 执行顺序（审查 §23 修订）

1. 修正 yaw 坐标（WD 双计风险）——含测试
2. baseline no-override 定义 + identity 两测试
3. 时长参数派生（去 n_control_steps）
4. 统一 UsableTime 语义（文档+配置注释）
5. Timestep Compatibility Test（1T A/B/C）
6. Induction 改 FLORIS 直接优化 calibrated a_cmd（LUT 加 a_cmd）
7. Sanity 增加 WD=−10（mirror + yaw sign）
8. Direct optimization 改分层搜索（Phase 5b）
9. above-rated 12m/s mini pilot
10. BTS 共享引用 + 批跑 reference_mode 记录
11. 进程清理 PID-scope + manifest 并发安全
12. Phase 5b（直优）-> 5c（weighted energy）
13. 更新 full-DOE gate 并按序执行

sanity（zero-yaw identity / no-override identity / greedy identity / yaw sign / mirror）全 PASS 前，不跑大 DOE。
