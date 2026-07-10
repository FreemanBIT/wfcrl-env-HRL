# 稳态风快速对比模块（steady/）

在湍流盒（`.bts`）生成与 FAST.Farm 仿真都较慢时，本模块让你**先用稳态风**
（InflowWind `WindType=1`，均匀来流）快速跑出一版 FAST.Farm vs FLORIS 的
诱导（降额）/ 偏航 控制对比，覆盖**不同风速 × 不同对风偏差 × 不同风机间距**。
湍流维度（TI）暂不考虑。

它**不修改你工程里的任何现有文件**，只是新增一个 `steady/` 子包，复用主工程的
布局、转换层、调度、LUT 与分析代码。

---

## 为什么稳态风能直接跑、且不会再遇到 .bts 越界

FAST.Farm 用 `WindType=1` 时**不需要任何 `.bts`**：`create_ff_case` 走
`fastFarmBoxExtent` 分支，由它**先**生成自洽的低/高分辨率网格，高分辨率盒再据此
派生——不存在“外部 TurbSim 盒子边界”这个约束，因此你之前遇到的
`Grid too small in Y direction` / `dY too large` **在稳态路径上根本不会出现**。

代价：稳态风没有真实湍流，所以这版结果用于**快速看趋势和量级**、验证全链路与
控制逻辑；湍流版（你正在准备的 9 个 `.bts`）才是最终高保真结果。

---

## 复用已建好的 FLORIS LUT

你已经在 FLORIS 中建好了按 (风速, 风向偏移, **TI**, 间距) 优化的 yaw/derating LUT。
稳态工况没有真实 TI，所以本模块在 LUT 中按
`(风速, 风向偏移, ti = lut_reference_ti, 间距)` 做**最近邻查询**取最优控制下发。
`lut_reference_ti` 在 `grid_steady.yaml` 里设定，默认 `0.10`（居中、较具代表性）。

> 控制量与下发口径与主工程**完全同源**：仍然走 `replay/schedule.build_control_input`
> → `conversion.build_derating_control`，FAST.Farm 拿绝对功率（MW）、FLORIS 拿
> `ratio = P_target / P_greedy(U)`，两边物理一致。

如果你想看 LUT 对 TI 假设的敏感性，把 `lut_reference_ti` 分别设成 0.05 / 0.10 / 0.15
各跑一轮，对比稳态结果即可。

---

## 安装：把 steady/ 放进工程

把整个 `steady/` 目录放到主包下，与 `cases/`、`replay/`、`run/` 并列：

```
induction_vs_yaw_study/
├── cases/
├── conversion/
├── lut/
├── replay/
├── analysis/
├── run/
└── steady/          ← 放这里
    ├── __init__.py
    ├── grid_steady.yaml
    ├── cases_steady.py
    ├── replay_steady.py
    ├── run_replay_steady.py
    └── run_compare_steady.py
```

> 注意：本模块用 `from induction_vs_yaw_study.steady...` 的绝对导入，因此必须放在
> 主包内部、作为子包使用（与主工程现有模块一致）。

---

## 用法（三步）

所有命令从**工程根目录**（含 `induction_vs_yaw_study/` 的那一层）运行。

### 1) 冒烟单工况跑通（强烈建议先做）

```bash
python -m induction_vs_yaw_study.steady.run_replay_steady --sim fastfarm --subset
python -m induction_vs_yaw_study.steady.run_replay_steady --sim floris   --subset
```

`--subset` 只跑 `grid_steady.yaml` 里 `smoke_subset` 的 1 个工况
（U=8, wd_offset=0, spacing=4D）× {baseline, yaw, derating}。确认无报错、能出
`results/timeseries_steady/*.csv` 再继续。

### 2) 全 60 工况回放

```bash
# FLORIS（很快）
python -m induction_vs_yaw_study.steady.run_replay_steady --sim floris

# FAST.Farm（稳态，单算例比湍流快不少）
python -m induction_vs_yaw_study.steady.run_replay_steady --sim fastfarm
```

支持**断点续跑**（已存在的产物自动跳过）；想只跑部分控制：
```bash
python -m induction_vs_yaw_study.steady.run_replay_steady --sim fastfarm --controls baseline derating
```
想覆盖重跑加 `--overwrite`。

### 3) 汇总对比

```bash
python -m induction_vs_yaw_study.steady.run_compare_steady
```

产物：
```
results/steady_summary.csv         # 每 (case, sim, control) 一行：全场功率、各机功率、
                                   #   相对 baseline 增益、FAST.Farm vs FLORIS 模型偏差
results/steady_summary_by_dim.csv  # 按 (风速, 风向偏移, 间距) × (sim, control) 的功率透视
results/steady_gain_by_dim.csv     # 同上，但值是相对 baseline 的增益(%)
results/figs_steady/
├── power_vs_spacing.png           # wd=0 各风速下，功率随间距变化（yaw/derating/baseline，两仿真器）
└── model_bias_vs_direction.png    # spacing=4D 下模型偏差随风向偏移
```

---

## 工况与参数（grid_steady.yaml）

| 维度 | 取值 | 说明 |
|---|---|---|
| `wind_speeds_ms` | 6, 8, 10 | 与湍流版一致 |
| `wind_direction_offsets_deg` | -20,-10,0,10,20 | 相对 270°（沿列轴）的对风偏差 |
| `spacings_D` | 2, 4, 6, 8 | 脚本内经 `xcoords` 注入，不改模型库 |
| `lut_reference_ti` | 0.10 | 复用 FLORIS LUT 时的 TI 切片 |
| **合计** | **60 工况** | 3 × 5 × 4 |

时长（稳态可比湍流短）：`t_settle_s=360`、`n_control_steps=20` →
单算例 `TMax = 360 + 20×3 = 420 s`。

> **想再提速？** 稳态尾流平衡只取决于尾流“走过”整列的时间 ≈ 列长/风速。
> 小间距、大风速工况其实远不需要 360 s。可在 `grid_steady.yaml` 把 `t_settle_s`
> 调小（例如 240 s）先观察；或后续给 `cases_steady` 加按
> `列长/风速` 自适应的 `t_settle`（本版未做，保持简单稳妥）。

---

## 与湍流版的关系

| | 稳态版（本模块） | 湍流版（主工程） |
|---|---|---|
| 入流 | `WindType=1` 均匀风，**无 .bts** | 9 个 TurbSim `.bts`（风速×TI） |
| 维度 | 风速×风向×间距（60） | 风速×风向×TI×间距（180） |
| LUT | **复用**主工程已建 LUT（按参考 TI 查） | 同一 LUT |
| 速度 | 快（无湍流盒、窗口短） | 慢（湍流盒生成+长仿真） |
| 用途 | 快速看趋势、验证链路 | 最终高保真对比 |
| 产物目录 | `results/timeseries_steady/`、`results/steady_*` | `results/timeseries/` 等 |

两者产物目录分开，互不覆盖，可并存。

---

## 故障排查

* **`No rows for control_type=...` / LUT 找不到**：确认 `results/luts/luts.parquet`
  （或 `.csv`）已由湍流版 `build_luts` 生成；或用 `--lut` 指定路径。
* **`未找到稳态时序目录`**：先跑 `run_replay_steady` 再跑 `run_compare_steady`。
* **FAST.Farm 仍报横向越界**：稳态路径不应出现；若出现，说明 `wind_file` 被意外
  设置成了 `.bts`。检查 `make_fastfarm_config_steady` 确为 `wind_file=None`、
  `wind_type=STEADY`。
* **matplotlib 缺失**：作图会被自动跳过，CSV 仍正常产出；需要图就 `pip install matplotlib`。
