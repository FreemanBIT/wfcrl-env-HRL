# DOE 运维手册 — Full DOE 运行 / 监控 / 排障

> 适用：`run_full_doe`（2592 case = 4U × 3TI × 6seed × 3WD × 4S × 3controls）。
> 历史事故与根因见 `HISTORY.md`；当前进度见 `STATUS.md`。

---

## 1. 启动 / 续跑

```powershell
# 进入仓库根
cd D:\HR_Project\wfcrl-env-HRL
$env:WFCRL_STUDY_OMP_NUM_THREADS = '3'      # 必须 <=3（见 §4.1）

# 首次或续跑（--resume 跳过 status=done 的 case；failed 会自动重试）
.\.venv\Scripts\python.exe -u -m induction_vs_yaw_study.run.run_fastfarm_batch `
  --manifest induction_vs_yaw_study/results/_manifests/case_manifest.csv `
  --out induction_vs_yaw_study/results/run_full_doe `
  --workers 1 --max-retries 3 --resume `
  --controls baseline yaw induction
```

**注意（PowerShell 引号坑）**：`--filter` 字符串里的双引号会被 pwsh 剥掉。
过滤 control 请用 `--controls`；`--filter` 只用纯数值比较，例如
`--filter 'U==8 & seed>=1 & spacing_D>=4'`。

---

## 2. 配置项（环境变量）

| 变量 | 默认 | 说明 |
|---|---|---|
| `WFCRL_STUDY_OMP_NUM_THREADS` | 1 | FAST.Farm OMP 线程数。**必须 ≤3**（>3 会破坏数值稳定性）|
| `WFCRL_STUDY_CASE_TIMEOUT_S` | 2700 | 单 case 超时（秒）；超时杀 worker 并判失败 |
| `WFCRL_STUDY_STAGGER_S` | 180 | 多 worker 错峰启动（workers=1 时无效）|
| `WFCRL_STUDY_ROSCO_LOG` | 0 | 设 1 恢复 ROSCO `.dbg` 日志（每机 77MB，诊断用）|
| `WFCRL_STUDY_KEEP_CASE_FILES` | 0 | 设 1 保留 case 的 bts/outb/DLL（DEL 阶段需要）|
| `WFCRL_STUDY_F_C` | 0 | 覆盖 Case.fstf 的 wake `f_c`（0=保持模板值；诊断用）|

`experiment.yaml` 的 `batch.omp_threads` 会被批跑器读取并注入上述 OMP 变量。

---

## 3. 监控

```powershell
# 进度看板（进度/速率/ETA/资源）
.\.venv\Scripts\python.exe -m induction_vs_yaw_study.run.doe_status `
  --run induction_vs_yaw_study/results/run_full_doe

# 质量审计（完整性/数值/稳定性；失稳率 >2% 时退出码非零）
.\.venv\Scripts\python.exe -m induction_vs_yaw_study.run.check_doe_quality `
  --run induction_vs_yaw_study/results/run_full_doe
```

---

## 4. 已知陷阱（务必遵守）

### 4.1 OMP > 3 会破坏数值稳定性
- OMP=6 时 **37%** 的 case 出现机组顺桨 90°/停机（正常工作风速下的数值失效）
- OMP=3 残余失稳率 ~2.5%；OMP=1 逐位确定但慢 2.5×
- 判据：统计窗内 `pitch>40°` 或 `rpm<2rpm`（正常运行时 U12 强风桨距也不超过 ~25°）

### 4.2 磁盘：必须关闭 ROSCO 调试日志
- 模板 `LoggingLevel=1` → 每机 77MB `.dbg`，3 机 = 231MB/case
- replay 已自动改 `LoggingLevel=0`；`_case_worker` 完成后清理大件（→ ~1.8MB/case）
- 若手工跑单 case 忘记清理，2592 case 可写爆 600GB

### 4.3 内存：只能用 1 worker
- 本机 15.8GB，系统+浏览器常占 ~10.8GB
- 每 case 峰值 ~5GB（bts 读取 2.4–2.9GB + 仿真 2GB）
- 2 workers 实测 OOM（`Unable to allocate 2.93 GiB`）

**内存闸门（G-22）**：同机还有另一条工作流（其并发进程实测可达 55 个），可用内存
会被挤到 3GB 以下，worker 随即在 `openfast_toolbox/io/turbsim_file.py:85` 抛
`MemoryError`。批跑器现在每个 case 启动前轮询可用内存：

| 环境变量 | 默认 | 作用 |
|---|---|---|
| `WFCRL_STUDY_MIN_FREE_GB` | 4.0 | 低于此值先等待 |
| `WFCRL_STUDY_MEM_WAIT_MAX_S` | 600 | 等待上限；超时照常启动，避免长期停滞 |

等待会打印 `[mem] free X GB < Y GB -> waiting` / `[mem] resumed`。
大量 `attemptN FAILED ... turbsim_file.py` 且每个只耗 2s，就是内存被抢占的特征。

### 4.3b 控制器 DLL 会被外部重建（G-21）
- `wfcrl/simulators/fastfarm/servo_dll/DISCON_WT1.dll` 由运行团队在同一分支上重建
  （2026-10-27 16:14 一次：2359.4→2371.6KB，对应 `7607e38`+`5436678`）
- 重建后**必须**核对是否影响控制律：那两次提交的改动全部被门控在 Windows
  FAST.Farm 路径不会进入的分支里（`state_store.c` 需 `of.valid`、
  `rosco_api.c` 需 `g_rt_rt_mode`），且 `FarmStateFrame` 布局未变（25664B）
- 现在 case `metadata.json` 与 run `provenance.json` 都记 `discon_dll_sha256`，
  任何"这批数据用哪个 DLL 跑的"问题都能直接查，不必靠文件 mtime 推断
- 追溯旧数据：DOE 已完成的 623 case 中 **507** 个在 10-27 16:14 前（旧 DLL）、
  **116** 个之后（新 DLL）；旧数据无 sha 字段，只能按 `timeseries.csv` mtime 划分

### 4.4 TurbSim 给定 seed 是确定性的
- 同 seed + 同 .inp 重跑 → **完全相同的湍流场**（实测 TI 相同到小数点后 4 位）
- 要让盒"换一个实现"，必须换 `RandSeed1` 或改 `.inp` 参数

### 4.5 QA 容差
- `|dU|/U <= 5%`，`|dTI| <= max(0.02, 0.25*TI_target)`
- 单次 1700–1800s 实现的轮毂 TI 天然有 ±0.02–0.03 散布，绝对容差 0.02 对 TI15 过严

---

## 5. 故障恢复

| 症状 | 处理 |
|---|---|
| 批跑进程消失、case 不推进 | 检查 `manifest_status.csv`；直接重跑同一命令（`--resume`）|
| worker 卡住（0 CPU 长期不退） | 超时机制会自动杀（2700s）；急用可 `Stop-Process` 后重跑 |
| 大量 `MemoryError: Unable to allocate ... bts` | 确认只有 1 worker；确认无其他大内存进程（TurbSim/浏览器）|
| 大量失稳（pitch→90°） | 确认 `WFCRL_STUDY_OMP_NUM_THREADS<=3` |
| 磁盘满 | 删除 `results/<run>/cases/*/FarmInputs/*.dbg`；确认 LoggingLevel=0 生效 |
| 案例判失败但数据完整 | `DONE.sentinel` 会在输出落盘后写入；有 sentinel 即视为成功 |

**剔除失稳 case 并重跑**（不删 `timeseries.csv` 的话 resume 会跳过）：

```python
# 扫描 status 文件，把失稳 case 从 done 集合移除并删目录，然后 --resume 重跑
# 判据：窗内 pitch>40 或 rpm<2（见 check_doe_quality.py）
```

---

## 6. 结果分析

```powershell
# 1) 汇总（case/group/gains/per_turbine/bias + report.md）
.\.venv\Scripts\python.exe -m induction_vs_yaw_study.run.analyze_doe `
  --run induction_vs_yaw_study/results/run_full_doe

# 2) 图（6 张）
.\.venv\Scripts\python.exe -m induction_vs_yaw_study.analysis.doe_plots `
  --analysis induction_vs_yaw_study/results/run_full_doe/analysis

# 3) DEL（代表工况，需 WFCRL_STUDY_KEEP_CASE_FILES=1 跑出的 outb）
.\.venv\Scripts\python.exe -m induction_vs_yaw_study.run.compute_del_batch `
  --run induction_vs_yaw_study/results/run_full_doe --t-settle 60
```

**统计口径**：显式窗口 `[t_settle=600, t_end=1200]`；增益按 **seed 配对**（同 seed 的
test/baseline 比值，再跨 seed 平均）；失稳 case 默认剔除（`--include-unstable` 可纳入）。

---

## 7. 预算参考（实测）

| 项 | 值 |
|---|---|
| 每 case 墙钟（OMP=3, 1 worker） | U06 ~508s、U08 ~590s、U10 ~326s、U12 ~383s |
| 综合速率 | ~6.4 case/h |
| 2592 case 总时长 | ~400 h（约 17 天；已完成的会随 resume 跳过）|
| 磁盘 | ~1.8MB/case → 全 DOE ~4.6GB |
| 内存 | 1 worker 峰值 ~5GB |
