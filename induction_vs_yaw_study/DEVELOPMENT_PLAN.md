# 开发计划：诱导(降额) vs 偏航 控制对比

> 工程：`wfcrl-env-HRL`（FLORIS + FAST.Farm 统一接口）
> 风场：3 × NREL 5MW 单列
> 本计划已基于**实际源码**（config.py / interface.py / simul_config.py /
> simul_utils.py / DISCON_bridge.f90 / 模板 / data_cases.py）核对，并已附带可运行
> 脚本（`induction_vs_yaw_study/`）。每个阶段写明：交付物（已生成的脚本）、coding
> agent 需在真实环境验证/补全的点、验收标准。

---

## 读码确认的两条关键事实（决定了整体设计）

1. **控制接口已足够，无需修改。** `ControlInput` 实现 5-mode：
   `mode0_yaw` / `mode1_power`(功率目标+最小桨距) / `mode2_pitch` / `mode3` / `mode4`。
   `DISCON_bridge.f90` 已实现功率闭环（前馈 `T_ff=P_target/ω` + PI 修正），并通过
   `controls.txt` 的 `T{id} mode= yaw= pitch= power= minpitch=` 协议接收命令。
   → 原计划里"改 Fortran 加功率通道"的条件阶段**取消**。

2. **`ControlInput.power` 在两个仿真器里语义不同，必须在转换层统一：**
   - FLORIS：`power` = 限功率比 ratio(0.01~1)，缩放 power_thrust_table。
   - FAST.Farm：`power` = 绝对目标功率(MW)。
   → LUT 存绝对 MW；下发由 `conversion.build_derating_control(sim_kind,…)` 单一收口。
   这是全项目最关键的正确性约束，**任何回放都不得绕过此函数**。

---

## 命名与约束（agent 全程遵守）

- 新代码全在 `induction_vs_yaw_study/`（与 `wfcrl/` 并列），**不改 `wfcrl/`**。
- **不改风场模型库 / 模板**；不同间距用 `cases.layout_for_spacing` 在脚本内替换 `xcoords`。
- 物理常数集中 `constants.py`（R=63,D=126,rho=1.225,rated,η,a_greedy）。
- 控制量单一来源：FLORIS / FAST.Farm 的 yaw / power 必由
  `conversion.build_*` + `replay.schedule.build_control_input` 产出。
- 功率单位统一 W↔MW 明确（LUT/下发/日志），禁止混用。
- 长耗时步骤支持断点续跑、子集、`case_id` 命名产物。
- 每阶段末更新 `docs/DEVELOPMENT_LOG.md`：实际签名、坑、与计划偏差。

---

## Stage 0 — 探针与可行性自检 ✅ 已生成

**交付物**：`run/stage0_probe.py`
打印 `ControlInput` 字段与 mode 工厂、接口方法、`FlorisConfig/FastFarmConfig` 字段、
FLORIS 可用性（nrel_5MW 3 机贪婪功率）、FAST.Farm exe 是否存在、入流 `.bts` 映射；
写 `docs/PROBE_RESULTS.md`。

**agent 验证**：在真实环境运行，确认：
- ControlInput 含 `power`/`min_pitch` 且有 `mode1_power`（应为真 → 接口无需改）。
- FLORIS import OK、nrel_5MW 可解析。
- FAST.Farm exe 与 DISCON_WT1.dll 就位。

**验收**：`PROBE_RESULTS.md` 明确写出"接口无需修改"的判定与各项 OK/缺失。

---

## Stage 1 — 功率设定点转换层 ✅ 已生成并通过单测

**交付物**：
- `conversion/power_setpoint_tools.py`：
  - 制动盘诊断：`cp_of_a/ct_of_a/a_of_ct`、`induction_from_power`、`power_from_induction`、`ct_from_power`。
  - 口径桥：`power_mw_to_ratio` / `ratio_to_power_mw`（绝对 MW ↔ FLORIS ratio）。
  - **单一来源**：`build_yaw_control`(mode0) / `build_derating_control(sim_kind,…)`。
- `tests/test_conversion.py`（已本地验证 15 项全过：Betz、Ct 往返、功率↔诱导往返、
  功率↔ratio 往返、单调性、FLORIS/FAST.Farm 同物理点一致）。

**agent 验证/补全**：
- 用 `pytest tests/test_conversion.py` 复跑。
- 若 Stage 0 显示 ControlInput 字段名有出入，仅需改 `build_*` 里字段名映射。

**验收**：单测全绿；`build_derating_control("fastfarm",…)` 给绝对 MW、
`("floris",…)` 给 ratio，且二者换算回的物理功率一致。

---

## Stage 2 — 3 机单列算例与工况网格 ✅ 已生成并通过逻辑测试

**交付物**：
- `cases/three_nrel5mw.py`：`Case`、`layout_for_spacing`(间距替换不改模板)、
  `inflow_bts_for_case`(按 风速×TI 联合匹配用户的 9 个 .bts)、
  `make_floris_config`/`make_fastfarm_config`、`iter_cases`/`case_id`。
- `cases/grid.yaml`：180 工况网格（3 风速 × 5 对风偏差 × 3 TI × 4 间距）
  + smoke_subset + 优化搜索范围。
- `tests/test_cases.py`（布局缩放、case_id 唯一稳定、风向语义、.bts 命名/匹配、网格规模=180）。

**agent 验证/补全**：
- 真实环境跑 `make_floris_config`/`make_fastfarm_config` 各 1 步，确认能被
  `FlorisInterface.setup()` / `ContinuousFastFarmInterface.setup()` 接受。
- 确认风向语义：FLORIS 270°=轴向，FAST.Farm 端 interface 自动转 PropagationDir。
- **入流**：你已生成 风速{6,8,10} × TI{0.05,0.10,0.15} 共 9 个 .bts，
  网格内每个 (风速,TI) 组合都精确匹配，TI 是 FAST.Farm 的真实入流维度。
  `make_fastfarm_config(strict_inflow=True)` 找不到匹配会报错（防止悄悄回退稳态）。
  Stage 0 探针逐一检查 9 个文件是否就位。仅当网格新增 9 个之外的 (风速,TI)
  组合时才需 TurbSim 生成（见 README 第 4 节）。

**验收**：任取 case 两仿真器各跑 1 步不报错；TI/风向确实写进输入（给字段证据）。

---

## Stage 3 — FLORIS 两套 LUT ✅ 已生成（待真实 FLORIS 验证）

**交付物**：
- `optimize/_floris_model.py`：`build_model`(布局/风况注入)、`greedy_farm_and_turbine_power`、
  `freestream_greedy_power_w`(ratio 分母)。
- `optimize/floris_yaw_opt.py`：`optimize_yaw`（FLORIS `YawOptimizationSR`，回退 scipy SLSQP）。
- `optimize/floris_derating_opt.py`：`optimize_derating`（**复用 interface 的限功率逻辑**
  ——缩放 power_thrust_table + 制动盘反解 a→Ct——粗网格扫描 + Nelder-Mead 细化）。
- `lut/schema.py` + `lut/interpolate.py`（已通过测试）。
- `run/build_luts.py`（遍历网格，写 `results/luts/luts.parquet`）。

**agent 验证/补全**：
- 真实 FLORIS 跑 `build_luts --subset`，确认无 NaN、ratio<1 的上游降额、gain≥0。
- 若 `YawOptimizationSR` 的 import 路径在你的 FLORIS 版本不同（v4.6），按报错调整
  （已写 try/except 回退，但 agent 应确认走的是 SR 而非回退）。
- `_build_curtailed_model` 用 per-turbine `turbine_type` dict 注入 FlorisModel；若你的
  FLORIS 版本不接受 dict 列表，改用 interface 同款"写临时 turbine yaml + turbine_library_path"
  方式（可直接照搬 `FlorisInterface._apply_turbine_curtailment`）。
- 补 `docs/LUT_SANITY.md` 自检脚本（物理合理性）。

**验收**：全网格生成 yaw + derating 两套 LUT；物理自检通过；
`power_from_induction(a_diag,U)≈power_target`（往返 <1e-2）。

---

## Stage 4 — 统一回放驱动 ✅ 已生成

**交付物**：
- `replay/schedule.py`：`build_control_input(control_type, sim_kind, settings,…)` 单一收口。
- `replay/floris_replay.py`：用 `FlorisInterface` 逐步 step。
- `replay/fastfarm_replay.py`：用 `ContinuousFastFarmInterface`
  （setup→reset→start→wait_step×N→stop），收集逐步在线测量。

**agent 验证**：同一 case 两 replay 返回结构一致的 `SimulationOutput`（字段/形状对齐）。

---

## Stage 5 — FLORIS 回放（自洽基线，快） ✅ 入口已生成

**交付物**：`run/run_replay.py --sim floris`，写 `results/timeseries/floris_*.csv`。

**验收**：FLORIS 回放稳态总功率 ≈ Stage 3 `farm_power_opt_mw`（<1%）。
若不符 → schedule 与优化口径不一致，重点查 `build_derating_control` 的 ratio 标定
（`greedy_power_w` 是否用了同一自由来流基准）。

---

## Stage 6 — FAST.Farm 回放（高保真，慢） ✅ 入口已生成

**交付物**：`run/run_replay.py --sim fastfarm`：
- 断点续跑（已存在 csv 跳过）、`--controls` 子集、`results/run_log_fastfarm.csv`。
- 时序含功率/桨距/转矩/转速/叶根载荷。

**agent 验证/补全**：
- 先 `--subset` 跑通（单工况三控制）。给足 `t_settle_s`（≥ 间距*D/U）。
- 入流：每个 (风速,TI) 工况用对应 .bts，TI 真实复现（9 个 .bts 已就位）。
- Windows 用 `mpiexec -n 1 …`。
- 关注 `wait_step` 轮询超时（默认 120s）；长 dt 或慢机器可调大。

**验收**：derating 回放上游机稳态功率 < 其 baseline（降额生效）；末机功率升高
（尾流恢复）；run_log 完整。

---

## Stage 7 — 交叉对比与报告 ✅ 已生成

**交付物**：
- `analysis/metrics.py`：稳态窗口均值、各机/全场功率、相对 baseline 增益、
  模型偏差 `(P_ff-P_flo)/P_flo`、载荷 std 代理；`build_summary`。
- `analysis/plots.py`：A 增益对比 / **B FLORIS-vs-FAST.Farm 散点** / C 间距趋势 / D 载荷 trade-off。
- `run/run_compare.py`：summary + 注入 case 元数据 + 出图 + `results/REPORT.md`。

**agent 验证/补全**：
- summary 覆盖所有完成 case；至少 3 张核心图产出。
- 需要正式 DEL 时把 std 代理换成 rainflow（`analysis/metrics.py`）。

**验收**：REPORT.md 能回答——哪些工况降额优于偏航、FLORIS 是否系统性高估增益。

---

## Stage 8 —（可选增强 P2）

- **风向/偏航符号端到端校验**：用一个已知偏航增益的标准算例，确认 FLORIS 与
  FAST.Farm 的角度约定完全一致（最重要的补强项，避免结论符号错误）。
- **FLORIS 原生 simple-derating 对照路径**：在 `_floris_model.py` 增
  `set(power_setpoints=...)` 路径，与"缩放 power_thrust_table"路径对比，量化差异。
- **扩展工况范围**：当前 9 个 (风速,TI) .bts 已支持全网格；若加新风速/TI，
  按 README 第 4 节生成 .bts 并在 `BTS_SPEEDS`/`BTS_TIS` 登记。
- **aeromap 升级**：OpenFAST 稳态扫描生成真实 Cp/Ct(λ,θ)，提升诊断精度。
- **时变工况 / 双列布局**：复用同一框架。

---

## 阶段依赖

```
Stage 0 探针
  ├─ Stage 1 转换层 ✅(测试通过)
  ├─ Stage 2 算例/网格 ✅(测试通过)
  │    └─ Stage 3 FLORIS 两套 LUT ✅(待真实 FLORIS 验证)
  │         └─ Stage 4 统一回放 ✅
  │              ├─ Stage 5 FLORIS 回放(快) ✅
  │              └─ Stage 6 FAST.Farm 回放(慢) ✅
  │                   └─ Stage 7 对比+报告 ✅
```

**里程碑**
- M1 = Stage 0+1：转换层单测全绿 + 接口确认无需改 → 可行性确认 ✅（本地已验证 M1 核心）。
- M2 = Stage 3：两套 LUT 物理合理 → 优化层可信（需真实 FLORIS）。
- M3 = Stage 5：FLORIS 回放自洽 → 管道贯通。
- M4 = Stage 6 冒烟：FAST.Farm 回放降额生效 → 高保真验证可行。
- M5 = Stage 7：核心对比图 + REPORT → 结题。

---

## 本地已验证（在无 FLORIS/openfast_toolbox 的环境）

- `conversion`：15 项数学/语义检查全过（含 FLORIS/FAST.Farm 同物理点一致）。
- `cases`：布局缩放、case_id 唯一稳定、风向、入流映射、网格规模、config 构建全过。
- `lut`：读写往返、最近邻/线性插值全过。
- `optimize` / `replay` / `analysis`：因依赖 FLORIS / openfast_toolbox / 实际仿真，
  未能在本环境执行，需 agent 在工程环境验证（逻辑已对齐真实接口）。
```
