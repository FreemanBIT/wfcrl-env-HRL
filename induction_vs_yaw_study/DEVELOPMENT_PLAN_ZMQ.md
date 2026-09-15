# FAST.Farm 三机串列 · Yaw vs Induction 测试开发方案（基于 wfcrl 新 ZMQ 架构重规划）

> 输入：用户提供的 FASTFarm_Yaw_Induction_Test_Development_Plan.md（下称"方案V2"）
> 用户约束（必须遵守）：
>   1. 风向测试只做 **0° / 10° / 20°**（不做负风向——与正风向对称，效果一致）；
>   2. 测试产出与配置文件**全部**放在 D:\HR_Project\wfcrl-env-HRL\induction_vs_yaw_study\ 下；
>   3. 用户将删除原有 .bts 文件（本计划**不得依赖**旧盒文件名/路径）；
>   4. 风速增加 **12 m/s**（风速集 {6, 8, 10, 12}）；
>   5. 测试工程开发过程中**不得修改现有 wfcrl 工程文件**（wfcrl/*、closed-loop-control/* 只读依赖）。
> 本计划**不包含** FAST.Farm v5.0.0 源码重编译（独立工作线，见 §13.3）。

---

## 0. 文档目的

- 解决旧 induction_vs_yaw_study 对比流程的 5 类问题（见 §1）；
- 全部复用于新 ZeroMQ 闭环链路（Controller -> Cmd -> ZmqFarmBridge -> FastFarmZmqEngine -> FAST.Farm v5）；
- 保留可复用件：FLORIS 优化层、LUT 存储、ZMQ 回放驱动；
- 新增/改造：seed 化风盒管线、case manifest、结果隔离、显式统计窗口、控制 identity 测试、
  yaw sign / induction 标定、paired-seed 统计、DEL/Pareto 分析；
- **隔离原则（约束 5）**：所有新增/修改的代码与产物只能落在 induction_vs_yaw_study/ 之下；
  wfcrl/、closed-loop-control/、docs/farm_control/ 一律只读使用，不提交任何改动。

---

## 1. 新旧方案差异总览

| 维度 | 旧 study（现状） | 方案 V2 + 约束要求 | 本计划落点 |
|---|---|---|---|
| BTS 生成键 | U × TI × spacing（36 盒，将删除） | U × TI × seed -> **72** 盒（4U×3TI×6seed） | 重写生成器（§4.2） |
| 风向维度 | ±20 / ±10 / 0 | **0 / 10 / 20**（对称性裁剪，约束 1） | grid.yaml（§5.1） |
| 风速维度 | 6 / 8 / 10 | **6 / 8 / 10 / 12**（约束 4） | wind_conditions.yaml（§4.1） |
| 随机种子 | seed 循环递增 | 固定 seed manifest（11001..66006） | seed_manifest.csv（§3/§4） |
| WD 处理 | 风场旋转法（已实现） | 旋转 layout，不重生成 BTS | 复用 layout_for_case（§5） |
| 统计窗口 | 后 50%（last frac） | 显式 [t_settle, Tsim]（600-1200 s） | 重写 analysis/metrics.py（§8.1） |
| 结果目录 | results/timeseries 混放 + glob | results/{run_id}/cases/{case_id} 隔离 | 全部在 study 下（§3/§9） |
| FAST.Farm 驱动 | 文件桥（已删除）-> ZMQ 静态回放 | 复用 ZMQ 链路 | fastfarm_zmq_replay.py 增强（§6） |
| 仿真时长 | 510 s | 1200 s（600 结算 + 600 统计） | grid.yaml（§5.1） |
| 控制标定 | Betz 解析近似 | yaw sign + induction operating map | §7 标定脚本 |
| 多 seed 统计 | 无 | 6 seed paired 聚合 | case_id 含 seed（§5.3）+ 统计（§8.3） |
| 载荷 | blade_loads 代理 std | 原始 .outb 高频 rainflow DEL | §8.4 |
| case 驱动 | 运行时遍历 | case_manifest.csv + resume | build_case_manifest.py（§5.5） |
| 依赖边界 | 直接依赖 wfcrl 内部行为 | 只读依赖 + env 注入（约束 5） | §5.4 / §6 / §13.2 |

---

## 2. 现状架构盘点

### 2.1 ZeroMQ 场控链路（不改动，只读使用）

\`\`\`
ClosedLoopRunner / replay 脚本
   |  Cmd{turbine_id, yaw_delta_deg, yaw_deg, induction_a}
   v
ZmqFarmBridge (closed-loop-control/closedloop/bridge.py)
   |  _send_yaw_delta（CW+ 动作语义，rad+seq） / _send_induction（a，>=A_GREEDY-1e-3 跳过）
   v  ZeroMQ PUB 5556 / SUB 5557
FastFarmZmqEngine (wfcrl/engine/fastfarm_zmq.py)
   |  setup(): case 生成 + FCR 集成 DLL + YawDOF=True + TMax=用户时长+一个低步
   |  start(): 启动 FAST.Farm_x64_OMP.exe（FCR 10ms 闭环）
   v
FAST.Farm v5.0.0（FarmStateFrame <- 各机 FcrFarmTurbineState）
   v
get_state() -> turbines[]（gen_power_w*/rotor_ct/rotor_cp/nacelle_heading_rad/...）
\`\`\`

### 2.2 只读复用清单（不改任何 wfcrl / closed-loop-control 文件）

| 模块 | 用途 | 使用方式 |
|---|---|---|
| wfcrl/engine/fastfarm_zmq.py FastFarmZmqEngine | ZMQ 引擎（cmd/state 端口可配） | 只读；传参使用 |
| wfcrl/engine/_ff_case.py | 盒充分性校验、低分辨率重建、f_c 自适应 | 只读；WFCRL_FARMINPUTS_DIR 注入 |
| wfcrl/transport/farm_zmq_client.py / farm_protocol.py | ZMQ 客户端/帧协议 | 由引擎内部使用 |
| closedloop.bridge.ZmqFarmBridge / closedloop.types.Cmd | 命令通道（yaw_deg / induction_a） | 只读导入 |
| closedloop.induction.power_to_induction | 回放下发 a 换算 | 只读导入 |
| replay/fastfarm_zmq_replay.py | ZMQ 静态回放驱动 | 增强（watchdog/窗口，§6） |
| replay/schedule.py / fastfarm_replay.py | 控制输入记录 / 兼容入口 | 复用 |
| replay/floris_replay.py | FLORIS 回放 | 复用 |
| optimize/floris_yaw_opt.py / floris_derating_opt.py | FLORIS 最优偏航/降额 | 复用（case 键去 seed） |
| lut/schema.py | LUT 读写（绝对 MW 标准） | 复用 |
| conversion/power_setpoint_tools.py | P <-> ratio <-> a（诊断层） | 复用 |

### 2.3 需要改造/废弃（只允许在 study 内改动）

| 现状 | 问题 | 处置 |
|---|---|---|
| turbsim_inp/generate_turbsim_inputs.py | spacing->BTS、seed 递增、无 12m/s | 重写为 seed 键生成器（§4.2） |
| cases/three_nrel5mw.py bts_name(spacing) 分支 | 文件名含 spacing | 改 seed 键解析（§4.4） |
| analysis/metrics.py steady_window_mean | last 50% | 显式窗口（§8.1） |
| run/run_replay.py / run_compare.py | results/timeseries 混放 + glob | run_id 隔离、manifest 驱动（§9） |
| 回放驱动无 fast_step 进度看门狗 | 假完成缺陷 | 批跑器 watchdog（§6.1） |
| steady/ 子包 | 稳态风 v5 ~260s 硬墙 | 标注废弃（全走 .bts） |
| 旧 FarmInputs 36 盒 | 将删除（约束 3） | 不归档、不依赖；新盒入 study/FarmInputs |

---

## 3. 目录与命名规范（约束 2：全部在 study 下）

\`\`\`
induction_vs_yaw_study/
├── config/
│   ├── wind_conditions.yaml        # 新（§4.1）
│   ├── experiment.yaml             # 新（DOE 组合/批跑/并行参数）
│   └── seed_manifest.csv           # 新（seed_id,rand_seed_1）
├── FarmInputs/                     # 新（约束 3：不依赖旧盒）
│   ├── coarse/                     # 72 盒（4U×3TI×6seed）
│   ├── fine/                       # 代表工况精细盒（DEL 阶段）
│   └── qa/                         # wind_box_metrics.csv / failures / plots
├── scripts/                        # 新：generate_wind_boxes / qa_wind_boxes /
│                                   #      build_case_manifest / run_fastfarm_batch /
│                                   #      calibrate_yaw_sign / calibrate_induction /
│                                   #      regress_300s / analyze_run
├── results/
│   └── run_YYYYMMDD_HHMMSS/
│       ├── run_config.yaml / git_commit.txt / seed_manifest.csv
│       ├── case_manifest.csv
│       ├── cases/<case_id>/         # 每 case 独立（fastfarm_zmq.log / trajectory）
│       ├── metrics/                 # case_metrics / paired_metrics / aggregated_metrics
│       └── plots/
├── tests/                           # identity / reuse / stats-window 测试
└── docs/                            # 研究报告、标定记录
\`\`\`

### 3.1 路径注入（约束 5 的关键机制）

- wfcrl 引擎通过环境变量 **WFCRL_FARMINPUTS_DIR** 解析风盒目录（
  wfcrl/engine/_ff_case.py:26 已确认），批跑器/回放启动时设为
  induction_vs_yaw_study/FarmInputs 即可，**无需改 wfcrl 源码**；
- 所有 case 的 output_dir 显式传入 induction_vs_yaw_study/results/{run_id}/cases/{case_id}；
- 若某能力 wfcrl 未提供，在 study 内自建（resolver、批跑、watchdog、指标等）。

### 3.2 Case ID（含 seed）

\`\`\`
U08_TI10_seed03_wd10_s6D_yaw
U08_TI10_seed03_wd10_s6D_baseline
U08_TI10_seed03_wd10_s6D_induction
\`\`\`
- WD 仅 {0, 10, 20}（wd0 / wd10 / wd20）；
- 随 case 保存完整 metadata（含 bts_file 相对路径）。

### 3.3 硬性约束（自动化断言）

1. 相同 U/TI/seed 的 BTS 路径唯一（跨 WD、spacing、control）；
2. case_id 必须携带 seed；LUT（FLORIS 侧）不携带 seed（§5.2）；
3. 文件名禁止出现 spacing / WD / control；
4. 所有输出路径根必须在 induction_vs_yaw_study/ 下（约束 2）。

---

## 4. 风工质流水线（方案 V2 Task A/B/C/G 映射）

### 4.1 config/wind_conditions.yaml（新）

\`\`\`yaml
wind_speeds_ms: [6.0, 8.0, 10.0, 12.0]     # 约束 4
turbulence_intensities: [0.05, 0.10, 0.15]
seeds: [1, 2, 3, 4, 5, 6]                  # 对应 seed_manifest rand_seed_1 11001..66006

turbsim:
  hub_height_ref_m: 90.0
  artificial_hub_height_m: 175.0           # 仅 TurbSim 网格定位；绝不改 turbine 轮毂高度
  grid_width_m: 1500.0
  grid_height_m: 340.0
  grid_bottom_m: 5.0
  target_grid_spacing_m: 9.0               # coarse 验收 8<=dy,dz<=10
  analysis_time_s: 1600.0                  # >= Tsim + GridWidth/min_U + 余量（6m/s 最严）
  usable_time: "ALL"
  dt_s: 0.10
  turbine_model: IECKAI
  iec_wind_type: NTM
  shear_power_law_exponent: 0.0            # 主实验；0.2 作为剪切敏感性（后续）
  coherence: (沿用方案V2 §6 的 SCMod/InCDec/CohExp)

# 12 m/s 说明：高于额定风速 11.4 m/s（Region III），
# 机组进入额定功率平台（桨距控制）+ Ct 下降 -> 尾流减弱、可控增益变小。
# 分析时需按 Region II / III 分层解释，且 12m/s 的 derating LUT 可能退回 ratio=1。
\`\`\`

### 4.2 生成器重写（turbsim_inp/generate_wind_boxes.py，新，替换旧脚本）

- 循环：for seed in seeds: for U in speeds: for TI in tis -> 72 个 .inp；
- 命名：inflow_U{U:02d}_TI{TI:02d}_seed{seed:02d}.inp|.bts（无 spacing/WD/control）；
- GridWidth 固定 1500.0；NumGrid_Y ~ 168（dy~9.0）、NumGrid_Z = 40（dz~8.72）；
- RandSeed1 = manifest.rand_seed_1；RandSeed2 = rand_seed_1 + 1000000；
- 默认非周期 + 足长 AnalysisTime（单次直读，查询时刻全部在文件内，规避 v5 时间墙）；
  如需多周期复用再启用 TurbSim periodic（RandSeed1=0，seed 映射 RandSeed2）；
- 生成 seed_manifest.csv 到 config/；
- 支持 --speeds/--tis/--seeds/--only 便于分片与"引导盒"先行（§11 Phase 1）。

### 4.3 风盒 QA（turbsim_inp/qa_wind_boxes.py，新）

- openfast_toolbox.io.turbsim_file.TurbSimFile 读盒（_ff_case 同款）；
- (z=90, y=0) 插值 u/v/w 时序 -> realized U、TI_u、sigma_u/v/w；
- 输出 FarmInputs/qa/wind_box_metrics.csv：
  file,seed_id,target_U,target_TI,realized_U,realized_TI,sigma_u,sigma_v,sigma_w,pass；
- 验收：|U_real-U_tar|/U_tar <= 5% 且 |TI_real-TI_tar| <= 0.02；失败阻止入 manifest。

### 4.4 BTS resolver（改造 cases/three_nrel5mw.py —— study 内允许）

\`\`\`python
def resolve_bts_file(wind_speed_ms, turbulence_intensity, seed_id) -> Path | None:
    # 仅 (U, TI, seed) 三维；删除 spacing 后缀与通用盒回退分支
\`\`\`
- 默认 FarmInputs 根改为 study/FarmInputs（仍可用 WFCRL_FARMINPUTS_DIR 覆盖）；
- 找不到 -> raise（strict_inflow=True 保持）。

### 4.5 复用断言（方案 V2 §18）

同一 (U,TI,seed) 的全部 WD x spacing x control case：断言 len(unique(bts_file)) == 1。

---

## 5. Case / Manifest 层（方案 V2 Task D 映射）

### 5.1 cases/grid.yaml 改造

\`\`\`yaml
wind_speeds_ms: [6.0, 8.0, 10.0, 12.0]        # 约束 4
wind_direction_offsets_deg: [0.0, 10.0, 20.0] # 约束 1（对称性裁剪）
turbulence_intensities: [0.05, 0.10, 0.15]
spacings_D: [2.0, 4.0, 6.0, 8.0]
seeds: [1, 2, 3, 4, 5, 6]

floris: { dt: 60.0, max_iter: 3 }
fastfarm:
  dt: 3.0
  t_settle_s: 600.0             # 显式结算窗口起点
  n_control_steps: 200          # 600..1200 s 统计窗口（dt=3）
smoke_subset:                   # 快速冒烟
  wind_speeds_ms: [8.0]
  wind_direction_offsets_deg: [0.0]
  turbulence_intensities: [0.10]
  spacings_D: [4.0]
  seeds: [1]
\`\`\`
Case 增加 seed_id:int=1；case_id 公式加入 seed。

### 5.2 FLORIS LUT 的 seed 无关性（关键效率决策）

- FLORIS 稳态模型无湍流 realization -> 最优 setpoint 与 seed 无关；
- LUT 计算量 = 4U x 3TI x 3WD x 4S = **144**（不是 216 的 6 倍）；
- optimize/* 的 case 键去掉 seed；按 (U,TI,WD,S) 布局实跑 greedy 基准得 greedy_power_w；
- 回放时 settings 仅与 (U,TI,WD,S) 绑定，6 个 seed 复用同一 setpoint。

### 5.3 全 DOE 规模（含约束 1/4）

- FAST.Farm case 总数 = 4U x 3TI x 6seed x 3WD x 4S x 3control = **2592**；
- 方案 V2 的 3240（5WD）缩减为 2592（3WD）；风速 +1 抵消部分裁剪；
- 结果隔离与分片执行见 §9/§11/§12。

### 5.4 build_case_manifest.py（新）

枚举 (U x TI x seed x WD x S x control) -> case_manifest.csv：
case_id,U,TI,seed,WD,spacing_D,control,bts_file,status（pending/running/done/failed）
- 仅 QA 通过盒入 manifest；支持 --filter 分片（§6.2）。

---

## 6. FAST.Farm ZMQ 批跑器（方案 V2 Task E / §32 核心）

### 6.1 单 case 执行单元（增强 fastfarm_zmq_replay.py，保持签名兼容）

1. case 构造：make_fastfarm_config(case) + wind_time_series=resolve_bts_file(...)；
2. 命令下发：
   - baseline：不写命令（或 induction_a=1/3 显式贪婪）；
   - yaw：Cmd(yaw_deg=abs_yaw)，yaw_mode 默认 static（与旧研究口径一致）；
   - induction：Cmd(induction_a=a_calib)，a 来自标定 map（§7.2），弃 Betz 反演；
3. watchdog（修假完成）：每控制步校验 state['fast_step'] 单调推进；连续 N 个轮询不推进
   -> 判停滞，记录 abort 并保留部分 outb；
4. 结算窗口：t >= 600 数据单独落 statistics/，窗口参数写 metadata；
5. 自然完成等待：控制步完成后 _proc.wait(timeout) 确保 .outb 完整落盘（沿用现有）；
6. 产物与清理：case 目录 = results/{run_id}/cases/{case_id}/；完成后删除 _work 中
   复制入的 .bts（源盒保留在 study/FarmInputs/coarse/）。

### 6.2 批跑 CLI（run_fastfarm_batch.py，新）

\`\`\`bash
python -m induction_vs_yaw_study.run.run_fastfarm_batch \\
  --manifest induction_vs_yaw_study/results/RUN/case_manifest.csv \\
  --run-id RUN \\
  --workers 4 \\
  --max-retries 2 \\
  --filter 'U==8 & TI==0.10 & seed<=3 & WD==0' \\
  --resume
\`\`\`
- 启动前设置 WFCRL_FARMINPUTS_DIR=study/FarmInputs（约束 5 注入）；
- 使用现有"清理 FAST.Farm 进程与端口占用"惯例；
- 日志只写 run 目录 / 工作区，绝不写 D:\ 根或 C:\。

### 6.3 并行调度

- 每 worker 独立端口对（FastFarmZmqEngine cmd_port=5556+2k / state_port=5557+2k）；
- 端口/并发可配置（experiment.yaml）；
- 失败重试 <=2，仍失败 -> status=failed + error 列。

### 6.4 已知约束（本计划内不重编译，不改 wfcrl）

- **DT_High 保持 0.01 s**：E6a 实验证明 DT_High=0.05 在集成 DLL 链路启动崩溃，
  方案 V2 §9.1"粗跑 DT_High=0.10"不采纳（记录为偏差，§13.1）；
- control_dt=3 s 不变；.bts dt=0.10 满足 DT_TurbSim <= DT_High；
- 单机 1200s 6T 墙钟 ~20-30 min（300s 回归 ~5 min 线性外推）；1T 标定 ~4-6 min。

---

## 7. 控制通道与标定（方案 V2 §20/21/22 映射）

### 7.1 三控制类型 Cmd 语义

| 控制 | Cmd 构造 | 说明 |
|---|---|---|
| baseline | Cmd(yaw_deg=abs(270-wd), induction_a=1/3) | 贪婪，yaw=0 失准 |
| yaw | Cmd(yaw_deg=abs_yaw)；abs_yaw=(270-wd)+yaw_mis mod 360 | bridge 换算 yaw_delta（CW+），yaw_mode static/control |
| induction | Cmd(induction_a=a_calib) | _send_induction；a>=A_GREEDY-1e-3 跳过（=baseline） |

### 7.2 Induction 标定（calibrate_induction.py，新）

1T（经 ZMQ 链路），U in {6,8,10,12} x a in {0.10,0.15,0.20,0.25,0.30,0.33}；
- 输出（.outb 高频）：a_cmd、P、Ct（或 RotThrust）、Pitch、RotorSpeed（通道名按
  OutListParameters.xlsx 核对，必要时在 study 侧扩展 OutList 注入）；
- 产出 induction_operating_map.csv：a -> P/P_greedy -> Ct（方案 V2 §21.1）；
- 标定 run 时长可缩短（300s：60 结算 + 240 统计）降成本。

### 7.3 Yaw sign 标定（calibrate_yaw_sign.py，新）

固定 (U=8, TI=0.05 最低盒, S=4D, WD=10°)，yaw in {-20, 0, +20}；
- 输出：YawErr、farm P、WT2 入流（nacelle/disk_ambient_wind）、（可用时）尾流中心；
- 判据：与 FLORIS convention 对齐——明确 positive yaw 在 FAST.Farm 中尾流偏向哪一侧；
- 产出 yaw_sign_calibration.csv；
- 注意：方案 V2 的"确定性测试"因 v5 稳态风硬墙不能走 WindType=1，以 TI=0.05 盒近似（§13.1 偏差）。

### 7.4 Identity 测试（tests/，新）

- test_zero_yaw_equals_baseline：yaw=[0,0,0] vs baseline -> farm P 差 < 0.5%；
- test_derating_ratio_one_equals_baseline：a=1/3 vs baseline -> 差 < 0.5%；
- test_same_environment_uses_same_bts：execution 断言；
- 同一 (U=8,TI=0.10,seed=1,WD=0,S=4D) 短配置（300s）执行，全 PASS 解锁 Phase 4+。

---

## 8. 指标与统计（方案 V2 §23-27 映射）

### 8.1 统计层重写（analysis/metrics.py）

删除 steady_window_mean(frac=0.5)；改为：
\`\`\`python
def steady(df, t_settle_s, t_sim_s):
    return df[(df["time_s"] >= t_settle_s) & (df["time_s"] <= t_sim_s)]
\`\`\`

### 8.2 Power 指标

- Pbar_i（窗口均值）、Pbar_farm；
- DeltaP_farm_% + WT1/2/3 分机增益（看上游牺牲/下游恢复）；
- eta_wake = DeltaP_downstream / (-DeltaP_controlled)（仅受控机功率下降时计算）。

### 8.3 Paired-seed 聚合（方案 V2 §26）

- 先逐 seed 算 DeltaP_s = P_ctrl,s - P_base,s，再跨 seed 求 mean/std/SE/95%CI；
- 禁止先跨 seed 平均绝对功率再算增益；
- paired_metrics.csv 每行 = (U,TI,WD,S,control) + 聚合统计。

### 8.4 迁移性 / Regret（方案 V2 §22/23）

- eta_transfer = DeltaP_FASTFarm / DeltaP_FLORIS（仅当 DeltaP_FLORIS >= 0.5%）；
- Regret_% = [P_FF(u*_FF) - P_FF(u*_FLO)] / P_FF(u*_FF) x 100%；
- FF 侧近最优由 §7 / Phase 3 coarse 扫描近似（yaw in [-30..30] 整数步；induction 用标定 map）。

### 8.5 Load / DEL（方案 V2 §27）

- 功率分析用 3s 降采样 CSV；**DEL 一律从原始高频 .outb**（禁 3s CSV）；
- 通道（按 OutListParameters.xlsx 核实）：blade-root flap/edge、tower FA/SS、
  yaw-bearing moment、RotThrust、LSS torque；
- 每机每通道独立 rainflow（analysis/del.py，新）；
- DeltaDEL_i,k %、DeltaDEL_max = max(...)；
- Power-Load Pareto（analysis/pareto.py，新）：X=farm power gain %，Y=DeltaDEL_max %，
  约束 max_allowed_del_increase_pct in {0,5,10}；
- Open Item：fine 阶段 .fst OutFreq（建议 5-10 Hz）实测验证对墙钟的影响。

---

## 9. 结果隔离与分析（方案 V2 §16 / Task E）

- 每次 batch 创建唯一 run_id；所有产物只写 study/results/{run_id}/（约束 2）；
- case 目录内附 metadata.json（U/TI/seed/WD/S/control/bts_file/统计窗口/控制指令）；
- run_compare.py 改为 manifest+run_id 驱动，删除对历史目录的 glob 聚合；
- 新分析入口 analyze_run.py --run-id RUN_ID，输出
  case_metrics / paired_metrics / aggregated_metrics / plots / REPORT.md。

---

## 10. 自动化测试与验收（方案 V2 §31/§33，含约束 1/4 更新）

| 编号 | 测试 | 验收 |
|---|---|---|
| 31.1 | BTS 数量 | n_bts == 72（4U x 3TI x 6seed） |
| 31.2-31.4 | BTS 身份 | 相同 U/TI/seed，spacing/WD/control 变化 -> bts 路径不变 |
| 31.5 | 统计窗口 | 所有分析行 time >= t_settle |
| 31.6 | zero-yaw identity | farm P 差 < 0.5% |
| 31.7 | induction identity (r=1) | farm P 差 < 0.5% |
| 33 | 验收清单 | 逐条勾选写入 docs/REPORT |

---

## 11. 开发阶段与执行步骤

> 工程红线（用户惯例）：任何一次仓库代码修改后，先跑一次 300s 仿真回归再进入下一步。
> 旧回归命令依赖的 inflow_08ms_TI10_s8D.bts 将被删除 -> **回归门禁改用新生成的
> 引导盒 inflow_U08_TI10_seed01.bts**（Phase 1 第一步先生成它）；
> 回归入口改为 study 内 run/regress_300s.py（约等命令为 6T greedy、
> U=8/TI=0.10/seed=1、mode 3 双通道等价、300s），输出入 study/results/（约束 2）。

### Phase 0 — 骨架与配置（无仿真）
1. 新建 config/{wind_conditions,experiment}.yaml、seed_manifest.csv；
2. grid.yaml 加 seeds/WD{0,10,20}/U12/显式统计窗口；
3. 目录骨架：study/FarmInputs/{coarse,fine,qa}、study/results 模板；
4. 确认不依赖旧 36 盒（用户即将删除）。
   - 门禁：本次仅配置/目录，无代码 -> 可延后到 Phase 1 引导盒就绪后补跑。

### Phase 1 — 风工质管线
1. 重写生成器；**先生成引导盒 inflow_U08_TI10_seed01.bts**；
2. 引导盒 QA 通过 -> 立即跑一次 regress_300s 门禁（恢复红线能力）；
3. 批量生成其余 71 盒（后台作业，分片）；
4. 实现 qa_wind_boxes.py 全量 QA；
5. 改造 resolver（seed 键）+ 复用断言测试；
6. 实现 build_case_manifest.py。
   - 门禁：regress_300s 通过 + pytest（reuse/stats-window）。

### Phase 2 — Sanity 验证
范围：U=8, TI=0.10, S=4D, WD in {0, 10, 20}, seed=1，三控制。
1. baseline 重复性（同 case 2 次，P 差 < 0.5%）；
2. yaw 三档（-20/0/+20）符号测试；
3. zero-yaw identity + induction r=1 identity；
4. 31.5 统计窗口断言。
   - 门禁：identity 全 PASS 才进入更大 DOE（方案 V2 明文要求）；
   - 每步代码改动后先跑 regress_300s。

### Phase 3 — 控制标定
1. calibrate_induction.py：1T x (4U x 6a) -> operating map；
2. calibrate_yaw_sign.py：3T 三档 -> sign calibration；
3. 标定结果接入 schedule/replay（induction 用 map，弃 Betz 反演）。
   - 门禁：regress_300s + identity 复跑 PASS。

### Phase 4 — 试点多 seed DOE
U=8, TI in {0.05,0.10,0.15}, WD in {0,10,20}, S in {4D,6D,8D}, seed 1..6, 3 controls
（= 3x3x3x6x3 = 486；预算紧可先 seed 1..3 = 243）。
1. 批跑器并行跑通；2. paired 统计；3. 趋势/CI 检查（含 Region II/III 分层）。
   - 门禁：试点报告评审通过后才进全 DOE。

### Phase 5 — 全功率 DOE（4U x 3TI x 6seed x 3WD x 4S x 3ctrl = 2592）
分片执行（按 U 或 seed 切片 + resume），多周级后台；产出
paired gains / CI / eta_transfer / eta_wake / Regret。

### Phase 6 — Fine-grid 载荷研究
1. 选代表工况（strong/partial wake、低/高 TI、短/长间距、含 12m/s）生成 fine BTS；
2. run_fastfarm_batch --fine（OutFreq 调高）-> 原始 .outb；
3. analysis/del.py rainflow -> DeltaDEL；analysis/pareto.py -> Power-Load Pareto；
4. 回答方案 V2 Q1-Q3，写研究报告。

---

## 12. 工作量 / 存储 / 并行预算（含约束 1/4 更新）

| 项 | 量 | 估算 |
|---|---|---|
| coarse BTS | 72 x ~1.2 GB | ~86 GB（1600s 盒） |
| fine BTS | 6-12（代表工况） | 40-80 GB（dt=0.05 单盒极大；可按 (S,WD) 缩窄） |
| TurbSim 生成 | 72 盒 | 每盒数分钟-数十分钟（后台分批） |
| 单 case 墙钟 | 1200s 仿真 | ~20-30 min（6T）；1T 标定 ~4-6 min |
| Phase 4 试点 | 486 case | 单 worker ~200h；4 worker ~2-3 天 |
| Phase 5 全 DOE | 2592 case | 4 worker ~11 天（建议分片 + 里程碑评审） |
| LUT（FLORIS） | 144 | 分钟级（seed 无关复用） |
| 标定 | 24 x 1T + 3 x 3T | ~3-4 h |

> 结论：全 DOE 是多周级战役 -> 按 Phase 4->5 里程碑门禁；先以 seed 1..3 输出现象与 CI。

---

## 13. 风险 / 红线 / 开放问题

### 13.1 与方案 V2 的偏差（记录在案）
1. DT_High 保持 0.01（E6a 约束），不采纳 V2 §9.1 粗跑 DT_High=0.10；
2. 确定性 yaw sign 测试以 TI=0.05 盒替代 WindType=1（v5 稳态硬墙）；
3. 旧盒删除（约束 3）-> 不提供 legacy 归档，回归门禁改用新引导盒。

### 13.2 工程红线
- ROSCO 10ms 路径不引入 socket/文件 I/O；闭环状态只来自 Provider push；
- 角度/单位换算只在传输层统一；yaw_delta（CW+ 动作）与 induction_ref 是唯二规范通道；
- 离线研究用"静态 setpoint 回放"而非 10ms 多循环（与历史可比）；
- **wfcrl/、closed-loop-control/、docs/farm_control/ 零修改**（约束 5）；
  注入点 = WFCRL_FARMINPUTS_DIR env + 显式 output_dir；
- 日志只落 run 目录 / 工作区，绝不写 D:\ 根或 C:\。

### 13.3 交叉影响：v5 重编译工作线
- 若 P1 补丁（每高步刷新 TimeStart）落地并替换 FAST.Farm_x64_OMP.exe，稳态风解锁，
  可增加稳态确定性测试并复核 §13.1-2；不影响本 DOE 结构。

### 13.4 开放问题（实现阶段确认）
1. fine 盒按 (S,WD) 缩窄宽度的规格（省盘 vs 统一 1500 的取舍）；
2. OutFreq 与 DEL 通道数在 fine 阶段的实测折衷；
3. 并行 worker 数与机器核数的实际配置（experiment.yaml 暴露；4 个 6T OMP 实例前 CPU 压测）；
4. 旧 steady/ 子包废弃删除时机；
5. 12 m/s（Region III）下 yaw/induction 收益很小 -> 是否需要从 pilot 先行验证再决定
   全 DOE 中 12m/s 的权重（不删除该工况，仅排优先级）。

---

## 14. 建议执行顺序

1. seed manifest 固定 -> 2. 删除 spacing->BTS 依赖（不依赖旧盒）-> 3. 统一 1500x340 盒
（引导盒先行）-> 4. BTS resolver（seed 键）-> 5. wind QA -> 6. case manifest ->
7. 显式统计窗口 -> 8. run_id 结果隔离 -> 9. zero-yaw identity -> 10. induction r=1 identity ->
11. yaw sign 标定 -> 12. induction operating map 标定 -> 13. pilot DOE（486）-> 14. full DOE（2592）
-> 15. fine-grid DEL/Pareto。
sanity 三项（zero-yaw identity / induction identity / yaw sign）全 PASS 前，不跑大 DOE。
