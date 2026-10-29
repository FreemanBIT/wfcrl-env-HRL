# STATUS — 诱导 vs 偏航研究：当前状态快照

**快照时间**：2026-10-28 17:10 · 分支 `dev/farm-control-runtime` · 最新提交 `86b4eba`

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
| 进度 | **624 / 2592（24.1%）** |
| 分布 | U06 216、U08 189、U10 111、U12 108 |
| 失败 | 160（`--resume` 自动重试中；均为内存抢占导致，见 G-22）|
| 运行配置 | **1 worker × OMP=3**（内存约束，见下）|
| 速率 | 6.1 case/h（平均 590s/case）|
| ETA | **~296 小时（约 12.3 天）** |
| 资源 | 磁盘 53.1GB 空闲（~1.8MB/case）；内存 4.6GB 空闲 |
| 数据质量 | 失稳 0（OMP=3 修复后），质量审计通过 |
| 控制器 DLL | 624 done 中 507 个用旧 DLL、116 个用新 DLL、1 个带 sha 溯源（G-21） |

**为什么只能用 1 worker**：本机 15.8GB，系统+浏览器常占 ~10.8GB；每 case 峰值 ~5GB
（bts 读取 2.4–2.9GB + 仿真 2GB）→ 2 workers 实测 OOM。详见 `DOE_OPERATIONS.md` §4.3。

---

## 二、阶段性研究结论（625 case，DOE 24%；6 seed 仍在补齐）

### 2.1 逐机机制（尾流偏转成立）
| 控制 | T1 | T2 | T3 | farm |
|---|---|---|---|---|
| yaw | **−153 kW** | −26 | **+289 kW** | **+110 kW** |
| induction | −42 | −98 | +132 | −7 |

偏航让上游机让路、下游机受益（净收益来自尾流偏转）；诱导降载较温和，
625 case 下全场净收益已收敛到 0 附近（−7 kW）。

**等权口径（Phase 5c，n=该 spacing 参与统计的工况数）**：

| scope | yaw | induction |
|---|---|---|
| ALL（n≈144） | **+2.36% ± 0.44** | **+0.76% ± 0.38** |
| S2D | +4.15% | +2.55% |
| S4D | +3.18% | +1.62% |
| S6D | +1.46% | −0.06% |
| S8D | +0.65% | **−1.01%** |

→ 两种控制都随间距增大而收益衰减；**8D 时诱导已转为负收益**。

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

| `run/audit_provenance.py` | 溯源审计（DLL sha / git commit / bts sha 一致性）|

**单元测试：66 passed**

---

## 四、开放问题

### 🟡 P4 资源与产能
- DOE 单 worker 需 ~12.8 天；若需提速只能释放内存开 2 workers（约 6 天）
- 若要缩短设计：seed 6→4 可省 1/3 时间（统计上 4 seed 仍可给可靠均值/标准差）

### ✅ P10 外部重建控制器 DLL 的数据一致性（已处置，G-21）

**发现**：运行团队在同一分支（`7607e38` + `5436678`，2026-10-27 15:54/16:17）重建了
`DISCON_WT1.dll`（2359.4 → 2371.6 KB）。DOE 跨越该时间点 → 507 个 case 用旧 DLL、
116 个用新 DLL，而旧 case 没有记录 DLL 版本，事后无法判定是否同源。

**判定：不影响本研究的数据一致性**，依据三条：
1. 两次提交的代码改动全部被门控在 **Windows FAST.Farm 路径不会进入**的分支里
   —— `state_store.c` 的字段合并需 `of.valid`（仅 fastfarmRT 的 S02 输入路径置位），
   `rosco_api.c` 的 RT 共运行需 `g_rt_rt_mode`（仅 `fcr_rt_init` 置位）；
2. `FarmStateFrame` 布局未变（25664 B）、`FCR_MAX_TURBINES` 仍 64，仅 AWAE 帧容量
   64→1000，与 3 机场控回放无关；提交说明亦自述"只影响 AWAE 帧几何/缓冲区"；
3. 实测：新 DLL 年代的 case 失稳率与旧年代一致（post 1/97 vs pre 0/100），无异常。

**已落地**：case `metadata.json` 与 run `provenance.json` 现记录 `discon_dll_sha256`
（按 mtime/size 进程内缓存，避免每 case 重读 2.4MB）；新增 `run/audit_provenance.py`
输出 DLL 年代分布 / git commit 分布 / bts sha 冲突（当前 19 个风盒 0 冲突）。

**遗留**：旧 507 个 case 无 sha 字段，只能按 `timeseries.csv` mtime 划年代。若后续要
做严格的跨年代 A/B，需在空槽位重跑同一工况（单 case ~10min），当前不阻塞。

### ✅ P11 内存不足导致 case 批量 MemoryError（已处置，G-22）

**现象**：failed 从 101 涨到 160（+59），全部是 `openfast_toolbox/io/turbsim_file.py:85`
的 `MemoryError`，每次失败仅 2s。

**根因（2026-10-28 实测更正，勿再沿用旧说法）**：**不是**外部工作流在抢内存。
运行团队提交说明里写到"55 个并发 fastfarmRT/DOE 进程"，但主机普查表明那是
**64 个已退出的 FAST.Farm 残留进程条目**（0 线程 / 0 CPU / 状态 `stopped`、
63/64 的父进程已消失），**不占任何内存**；普查时刻也没有外部工作流在跑。
真实占用来自桌面环境本身：

| 组成 | GB |
|---|---|
| memcompression（Windows 压缩内存） | 2.06 |
| windows services | 1.77 |
| Edge 浏览器 | 1.32 |
| 微信 | 0.96 |
| node（DSH 宿主） | 0.54 |
| 其余（explorer/腾讯会议/Defender/OneDrive…） | ~1.8 |
| **合计（非本研究）** | **~10–11** |

本机 15.8GB，可用长期只有 **3.5–4.6GB**；而单 case 峰值 = bts 重采样 ~3.3GB
（U06 2.93GB + 解释器）→ 可用一旦跌破 3.3GB，bts 读取必然 MemoryError。

**处置**：批跑器在每个 case 启动前轮询可用内存（`WFCRL_STUDY_MIN_FREE_GB`=4.0GB，
上限 `WFCRL_STUDY_MEM_WAIT_MAX_S`=600s，超时照常启动以免长期停滞），把"失败→重试"
换成"短暂等待"。2026-10-28 17:00 重启批次后，done 增长而 failed 冻结在 160。

**若想提速**：关掉 Edge/微信/腾讯会议可释放约 2.6GB（→ 可用 6GB 以上），才有机会
恢复 2 workers（约 6 天完成）；否则维持单 worker ~12.3 天。

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

1. **等 DOE 跑完**（~12.3 天）→ 补跑 160 failed + 检出失稳 case
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
