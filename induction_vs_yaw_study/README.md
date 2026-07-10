# induction_vs_yaw_study

诱导因子(降额)控制 vs 偏航控制 对比研究 — 针对 3 × NREL 5MW 单列风场。

在 **FLORIS** 中针对参数化工况（风速 / 风向 / 湍流度 / 间距）离线求解
**最优偏航 LUT** 和 **最优功率设定点(降额)LUT**；在 **FAST.Farm** 中用相同工况
回放两套 LUT；最后交叉对比 FLORIS 与 FAST.Farm、偏航与降额四个组合的功率与载荷。

> 本包是 `wfcrl-env-HRL` 工程的**叠加层**，放在工程根目录下，与 `wfcrl/` 并列。
> 不修改 `wfcrl/` 任何既有代码，也不修改风场模型库 / 模板。

---

## 0. 关键设计结论（读代码后确认）

1. **控制接口无需修改。** 工程的 `ControlInput` 已实现 5-mode 协议
   （`mode0_yaw` 偏航、`mode1_power` 功率目标+最小桨距、`mode4_power_yaw`），
   `DISCON_bridge.f90` 已实现功率闭环跟踪（前馈 `T_ff=P_target/ω` + PI 修正）。
   偏航与降额两种控制都能直接下发。

2. **两个仿真器对 `ControlInput.power` 的语义不同，已在转换层统一：**
   - FLORIS (`FlorisInterface.step` mode 1/2)：`power` = **限功率比 ratio ∈ (0.01,1]**，
     缩放 `power_thrust_table` 并由制动盘关系反解新 a→Ct。
   - FAST.Farm (`DISCON_bridge.f90` mode 1/4)：`power` = **绝对目标功率 (MW)**。

   → **LUT 以绝对功率(MW)为标准存储量**；`conversion.build_derating_control(sim_kind,…)`
   是唯一的控制输入来源，给 FAST.Farm 绝对 MW、给 FLORIS `ratio=P/P_greedy(U)`。
   保证"同一物理工作点"在两个仿真器里一致。

3. **诱导因子 a 是诊断/汇报量**（由 P_target 与 U 反算写入 LUT），不直接下发。

4. **不同间距在脚本内通过 `xcoords` 替换实现**（`cases.layout_for_spacing`），
   FLORIS 用 `FlorisConfig.xcoords`，FAST.Farm 由 `create_ff_case` 据 `xcoords`
   重建 `.fstf` 布局。**不改模板、不改模型库。**

---

## 1. 安装与前置

本包依赖工程本身的环境（`pip install -e .` 已装好 FLORIS、openfast_toolbox 等）。

额外无新依赖（仅用 numpy / scipy / pandas / matplotlib / pyyaml，均已在工程内）。

FAST.Farm 可执行文件与 `DISCON_WT1.dll` 按工程 README 准备好：
- `wfcrl/simulators/fastfarm/bin/FAST.Farm_x64_OMP.exe`
- 已编译的 `DISCON_WT1.dll`（`wfcrl/simulators/fastfarm/servo_dll/`）

---

## 2. 目录结构

```
induction_vs_yaw_study/
├── constants.py                 NREL 5MW 物理常数（集中，禁止散落硬编码）
├── conversion/
│   └── power_setpoint_tools.py  功率(MW)↔ratio↔诱导a 转换 + ControlInput 单一来源
├── cases/
│   ├── three_nrel5mw.py         3 机单列算例、间距替换、入流映射、config 构建
│   └── grid.yaml                工况网格定义（可改）
├── optimize/
│   ├── _floris_model.py         构建用于优化的 FlorisModel + 贪婪基准
│   ├── floris_yaw_opt.py        偏航 LUT 求解 (Stage 3A)
│   └── floris_derating_opt.py   降额 LUT 求解 (Stage 3B)，复用 interface 限功率逻辑
├── lut/
│   ├── schema.py                LUT 列定义、parquet/csv 读写
│   └── interpolate.py           多维 (U,wd,TI,spacing) 插值查询
├── replay/
│   ├── schedule.py              控制输入单一来源（FLORIS/FAST.Farm 同源）
│   ├── floris_replay.py         FLORIS 回放 (Stage 5)
│   └── fastfarm_replay.py       FAST.Farm 连续回放 (Stage 6)
├── analysis/
│   ├── metrics.py               稳态指标 + summary 表
│   └── plots.py                 核心对比图
├── run/
│   ├── stage0_probe.py          接口探针与可行性自检 (Stage 0)
│   ├── build_luts.py            构建两套 LUT (Stage 3 入口)
│   ├── run_replay.py            回放 LUT (Stage 5/6 入口)
│   └── run_compare.py           交叉对比 + 报告 (Stage 7 入口)
├── tests/                       单元测试（conversion / cases / lut）
└── docs/                        PROBE_RESULTS.md, LUT_SANITY.md（运行时生成）

产物（写入工程根 results/，建议 git 忽略）：
results/
├── luts/luts.parquet            两套 LUT
├── timeseries/{sim}_{control}_{case_id}.csv
├── figures/                     对比图
├── run_log_{sim}.csv            回放运行日志
├── summary.parquet / summary.csv
└── REPORT.md                    一页式结论
```

---

## 3. 运行流程（按阶段）

```bash
# 把本包放在工程根目录（与 wfcrl/ 并列）后，从工程根运行：

# Stage 0 — 探针：确认接口、FLORIS、FAST.Farm exe、入流 .bts 都就绪
python -m induction_vs_yaw_study.run.stage0_probe

# 单元测试（纯逻辑，不需仿真器）
pytest induction_vs_yaw_study/tests/ -v

# Stage 3 — 构建 LUT（FLORIS 离线优化）
python -m induction_vs_yaw_study.run.build_luts --subset      # 先冒烟 1 工况
python -m induction_vs_yaw_study.run.build_luts               # 全网格 180 工况（先 --subset 跑通）

# Stage 5 — FLORIS 回放（快，自洽校验）
python -m induction_vs_yaw_study.run.run_replay --sim floris --subset
python -m induction_vs_yaw_study.run.run_replay --sim floris

# Stage 6 — FAST.Farm 回放（慢，高保真）
python -m induction_vs_yaw_study.run.run_replay --sim fastfarm --subset
python -m induction_vs_yaw_study.run.run_replay --sim fastfarm   # 全网格，耗时

# Stage 7 — 交叉对比 + 报告
python -m induction_vs_yaw_study.run.run_compare
```

`run_replay` 支持断点续跑：已存在的 `results/timeseries/*.csv` 自动跳过
（用 `--overwrite` 强制重跑）。可用 `--controls baseline yaw derating` 选择子集。

> Windows 上 FAST.Farm 需要 MPI；按工程 README，必要时用
> `mpiexec -n 1 python -m induction_vs_yaw_study.run.run_replay --sim fastfarm`。

---

## 4. 入流风文件（已就绪） ★

FAST.Farm 用 TurbSim `.bts` 全流场湍流盒做入流。**你已生成 9 个 `.bts`**，
覆盖 风速 {6,8,10} m/s × 湍流度 {0.05,0.10,0.15}，放在
`wfcrl/simulators/fastfarm/inputs/template/FarmInputs/`：

| 文件名 | 风速 | 湍流度 |
|--------|------|--------|
| `inflow_06ms_TI05.bts` | 6 m/s | 5% |
| `inflow_06ms_TI10.bts` | 6 m/s | 10% |
| `inflow_06ms_TI15.bts` | 6 m/s | 15% |
| `inflow_08ms_TI05.bts` | 8 m/s | 5% |
| `inflow_08ms_TI10.bts` | 8 m/s | 10% |
| `inflow_08ms_TI15.bts` | 8 m/s | 15% |
| `inflow_10ms_TI05.bts` | 10 m/s | 5% |
| `inflow_10ms_TI10.bts` | 10 m/s | 10% |
| `inflow_10ms_TI15.bts` | 10 m/s | 15% |

命名规则（`cases.bts_name`）：`inflow_{速度:02d}ms_TI{TI*100:02d}.bts`。

**关键改进：TI 现在是 FAST.Farm 的真实入流维度。**
`cases.inflow_bts_for_case(speed, ti)` 按 (风速, 湍流度) **联合**匹配 `.bts`，
所以 FLORIS 扫的 TI 和 FAST.Farm 回放的 TI 是同一个物理湍流度——
不再有"TI 只对 FLORIS 生效"的折中。网格的 3 风速 × 3 TI = 9 个 (速度,TI) 组合
与你的 9 个 `.bts` 一一对应，**全网格直接可跑**。

`make_fastfarm_config` 默认 `strict_inflow=True`：若某工况找不到匹配 `.bts`
会**直接报错**（而非悄悄回退稳态导致 TI 维度失真）。Stage 0 探针会逐一检查
9 个文件是否就位。

### 对风偏差 / 间距为何不需要额外 `.bts`

- **对风偏差**（wind_direction_offset）：由 `InflowWind` 的 `PropagationDir`
  实现（interface 内部设 `PropagationDir=(direction+90)%360`），**复用同一个
  `.bts`**，无需为每个偏差角生成新文件。
- **间距**（spacing_D）：FAST.Farm 的高分辨率盒按每台机位置从低分辨率盒自动
  裁剪（见 `simul_utils.create_ff_case` 的 `fastFarmTurbSimExtent`），所以
  2D~8D 各间距**共用同一个 `.bts`**，无需为间距生成新文件。

### 若以后要扩展网格（需新 `.bts` 时）

只有当你给 `grid.yaml` 新增**当前 9 个之外的 (风速, TI) 组合**（如 12 m/s 或
TI=0.20）时，才需要用 TurbSim 生成新 `.bts`。生成要求（基于工程模板
`FarmInputs/90m_08mps.inp` 修改）：

- **风速**：`URef`（如 `6.0`）。
- **湍流度**：`IECturbc`（直接给百分比，如 `"5"`/`"10"`/`"15"`）。
- **参考高度**：`RefHt = 90`、`HubHt = 90`（NREL 5MW 轮毂高度）。
- **网格**：`GridHeight`/`GridWidth` ≥ 转子直径 126m 覆盖（模板 340/680 足够）。
- **时长**：`AnalysisTime ≥ t_settle_s + n_control_steps*dt`。本网格
  `t_settle_s=360`、`n_control_steps=50`、`dt=3` → 总时长 510s，故
  **`AnalysisTime ≥ 520`**（你已生成的 9 个若按更短时长生成，跑长回放前请确认
  够长，否则 FAST.Farm 会循环复用湍流盒，物理上可接受但需知悉）。
- **时间步**：`TimeStep = 0.1`（与 `.fstf` 的 `DT_High=0.1` 对齐）。
- 输出 `WrADFF = True`。
- 文件名严格按 `inflow_{速度:02d}ms_TI{TI*100:02d}.bts`，放入 `FarmInputs/`，
  并在 `cases/three_nrel5mw.py` 的 `BTS_SPEEDS`/`BTS_TIS` 列表里登记新值。

---

## 5. 工况网格（`cases/grid.yaml`）

与你的最终验证目标一致：

```yaml
wind_speeds_ms: [6.0, 8.0, 10.0]
wind_direction_offsets_deg: [-20.0, -10.0, 0.0, 10.0, 20.0]
turbulence_intensities: [0.05, 0.10, 0.15]
spacings_D: [2.0, 4.0, 6.0, 8.0]
```

笛卡尔积 = 3 × 5 × 3 × 4 = **180 工况**。FLORIS 与 FAST.Farm 用**同一套工况**回放。

- **风向偏移(对风偏差)**：相对来流轴向(270°，沿 +X 列轴)的偏移。
  0° = 完全对齐（尾流最强）；±10/±20° 为对风偏差。
- **间距**：以 D=126m 为单位（2D=252m … 8D=1008m）。
- **湍流度**：5%/10%/15%，FAST.Farm 端用对应 `.bts` 真实复现。
- 改网格直接编辑 `grid.yaml`；`smoke_subset` 是冒烟用的最小子集（U=8,off=0,TI=0.10,4D）。

> FAST.Farm 成本随工况数线性增长（180 工况 × 3 控制 = 540 次 FAST.Farm 运行）。
> **务必先 `--subset` 跑通，再分批上全网格**（用 `run_replay` 的断点续跑能力）。
> 间距 8D + 风速 6m/s 是最慢工况（尾流传播 ~336s），`t_settle_s=360` 已据此设定。


---

## 6. 结果与验收

- **Stage 3 自检**（写 `docs/LUT_SANITY.md`，建议 agent 补充脚本）：
  上游机偏航非零、随 TI↑间距↑减小；上游机降额 ratio<1、增益≥0；末机≈贪婪。
- **Stage 5 自检**：FLORIS 回放稳态总功率 ≈ 优化记录的 `farm_power_opt_mw`（<1%）。
  否则说明 schedule/口径不一致。
- **Stage 7 核心图**：
  - A 功率增益对比（yaw vs derating，FLORIS vs FAST.Farm）
  - **B FLORIS-vs-FAST.Farm 增益散点**（对角线=一致）——主结论图
  - C 增益随间距趋势 / D 载荷-功率 trade-off
- **REPORT.md** 自动汇总：哪些工况降额优于偏航、FLORIS 是否系统性高估增益。

---

## 7. 已知限制 / 给后续的提示

- FLORIS 降额用工程 `FlorisInterface` 的"按比例缩放 power_thrust_table"机制，
  非 FLORIS 原生 `simple-derating`。两者都基于制动盘，但若想对齐 FLORIS 官方
  `set(power_setpoints=...)`，可在 `optimize/_floris_model.py` 增一条可选路径。
- 载荷对比用叶根弯矩标准差作为 DEL 的简化代理；需要正式 DEL 可在
  `analysis/metrics.py` 接入 rainflow 计数。
- FAST.Farm 的 TI 复现见第 4 节折中方案。
- 风向/偏航角参考系：FLORIS 气象 270°=轴向；FAST.Farm 端 interface 内部转
  `PropagationDir=(dir+90)%360` 并设初始 `NacYaw=(270-dir)%360`。已对齐，但
  建议 agent 补一个"已知偏航增益标准算例"做端到端符号校验（见开发计划 Stage 8）。
```
