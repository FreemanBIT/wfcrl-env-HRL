# STATUS — 诱导 vs 偏航研究：当前状态快照

**快照时间**：2026-10-26 22:30 · 分支 `dev/farm-control-runtime` · 最新提交 `fdbd49b`

> 文档索引
> - **本文档**：当前状态 / 开放问题 / 下一步
> - `HISTORY.md`：已完成里程碑与问题根因（P1~P6、ICD v1.5 迁移、磁盘与 OMP 事故）
> - `DOE_OPERATIONS.md`：DOE 运行 / 监控 / 排障手册
> - `EXECUTION_LOG.md`：full-DOE 15 项 gate 逐项跟踪
> - `../DEVELOPMENT_PLAN_ZMQ.md`：开发方案 V3

---

## 一、当前阶段：Full DOE 运行中

| 指标 | 值 |
|---|---|
| 进度 | **593 / 2592（22.9%）** |
| 分布 | U06 216、U08 161、U10 108、U12 108 |
| 失败 | 101（`--resume` 自动重试中）|
| 运行配置 | **1 worker × OMP=3**（内存约束，见下）|
| 速率 | 6.16 case/h（平均 584s/case）|
| ETA | **~308 小时（约 12.8 天）** |
| 资源 | 磁盘 53.0GB 空闲（~1.8MB/case）；内存 3.1GB 空闲 |
| 数据质量 | 失稳 0（OMP=3 修复后），质量审计通过 |

**为什么只能用 1 worker**：本机 15.8GB，系统+浏览器常占 ~10.8GB；每 case 峰值 ~5GB
（bts 读取 2.4–2.9GB + 仿真 2GB）→ 2 workers 实测 OOM。详见 `DOE_OPERATIONS.md` §4.3。

---

## 二、阶段性研究结论（503 case，部分 seed 已完成）

### 2.1 逐机机制（尾流偏转成立）
| 控制 | T1 | T2 | T3 | farm |
|---|---|---|---|---|
| yaw | **−145 kW** | −8 | **+267 kW** | **+114 kW** |
| induction | −25 | −81 | +120 | +14 |

偏航让上游机让路、下游机受益（净收益来自尾流偏转）；诱导降载较温和。

### 2.2 FLORIS 系统性高估偏航收益（关键 model-bias）
| 风速 | FLORIS 预测 | FAST.Farm 实测 | 偏差 |
|---|---|---|---|
| U6 | +7.27% | +3.46% | **−3.82pp** |
| U8 | +5.64% | +2.10% | **−3.55pp** |
| U10 | +6.36% | +3.73% | **−2.63pp** |
| U12 | +4.75% | +0.53% | **−4.22pp** |

→ **直接采用 FLORIS 优化的偏航角会高估收益约 3–4 个百分点**（RMSE 7–10%）；
诱导降载的偏差小（RMSE 4–6%）。

> 注：以上基于 1 seed 为主的初步样本，完整 6-seed 数据到齐后需复核。

---

## 三、工具链（研究侧，全部可用）

| 工具 | 作用 |
|---|---|
| `run/run_fastfarm_batch.py` + `run/_case_worker.py` | 批跑（子进程隔离 / resume / 超时 / 顺桨过滤）|
| `run/doe_status.py` | 进度看板 |
| `run/check_doe_quality.py` | 质量审计（完整性/数值/稳定性）|
| `run/analyze_doe.py` | 汇总 → case/group/gains(seed 配对)/per_turbine/bias + report.md |
| `analysis/doe_plots.py` | 6 张结果图 |
| `analysis/del_analysis.py` + `run/compute_del_batch.py` | DEL（rainflow + S-N）|
| `run/add_load_channels.py` | 载荷通道注入（TwrBs/LSSGag/YawBr）|
| `optimize/calibrated_a_cmd.py` | 标定 a_cmd 校准曲线（map → ratio）|
| `turbsim_inp/generate_wind_boxes.py` / `qa_wind_boxes.py` | 风盒生成 / QA |
| `run/build_case_manifest.py` | manifest 构建（QA 门禁）|

**单元测试：58 passed**

---

## 四、开放问题

### 🟡 P4 资源与产能
- DOE 单 worker 需 ~12.8 天；若需提速只能释放内存开 2 workers（约 6 天）
- 若要缩短设计：seed 6→4 可省 1/3 时间（统计上 4 seed 仍可给可靠均值/标准差）

### 🟡 P7 残余数值失稳（~2.5%）
- OMP=3 下仍有约 2.5% 的 case 出现机组顺桨（正常风速下，非物理）
- 处理：`check_doe_quality` 检出 → 剔除后重跑；若重跑仍失稳则排除并记录
- 根因指向 ROSCO 桨距控制的边界行为（wfcrl 代码，约束⑤不可改）

### 🟡 P8 DEL 需代表工况重跑
- 为控磁盘，DOE case 完成后清理了 outb；DEL 需用
  `WFCRL_STUDY_KEEP_CASE_FILES=1` 对代表工况重跑（每 case ~10–20 组）

### 🟢 P9 未开始
- **Phase 5b**：FAST.Farm 直优（12–20 代表 case，分层搜索）→ 之后才有 Regret
- **Phase 5c**：加权能量（无风玫瑰 → equal-weight case-average）
- Timestep B（0.05 短盒）可选
- U6/U10/U12 的 `a_min_eff` 已在标定中覆盖（各 U 曲线单调）

---

## 五、下一步

1. **等 DOE 跑完**（~12.8 天）→ 补跑 101 failed + 检出失稳 case
2. 全量 6-seed 分析 → 复核 §2 的阶段性结论；出最终 `report.md` + 6 张图
3. DEL 代表工况（keep-files 模式）→ Power-Load Pareto
4. Phase 5b 直优 → Regret 分析
5. Phase 5c equal-weight 能量聚合

---

## 六、关键约束（贯穿全程）

- **不修改** `farm_control_runtime/`、`wfcrl/transport/`、`wfcrl/engine/`、`closed-loop-control/`
  等场控接口相关代码；只从 `induction_vs_yaw_study/` 研究侧操作
- 每次代码改动后跑 300s 门禁（**例外**：纯分析/文档工具不触发仿真）
- 内存规则：仿真与盒生成错峰；DOE 单 worker
- 风向仅 {0,10,20}；风速 {6,8,10,12}；OBE 输出全部在 `induction_vs_yaw_study/` 下
