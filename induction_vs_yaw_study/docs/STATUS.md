# STATUS — 诱导 vs 偏航研究：开发进度与问题（快照 2026-09-21 10:29）

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
| Induction 1T 标定 | 🟡 仅 U8 六点（a 0.10→1456 … 0.33→1588 kW，单调） |

---

## 二、进行中
- **U8 风盒 13/18**（TI05 s1–5、TI10 s1–4、TI15 s1–4）；2 个 TurbSim 正在补齐剩余 5 盒，单盒 ~7–9 h，预计 ~20 h
- 内存 5 GB 空闲（固定 2 并发 TurbSim）
- **未提交改动**：`replay/fastfarm_zmq_replay.py`（yaw_delta 通道）、`run/_fill_boxes.py`

---

## 三、存在的问题（按严重度）

### 🔴 P1 yaw 命令实际未生效 —— 阻塞 yaw 侧全部结论
- **现象**：yaw=30° 时 WT1 功率仅 −0.06%（几何预期 −35%）；LUT yaw=[24.4, 28.1, 0] 在 FAST.Farm 中增益 −0.13%（FLORIS 预测 +9.93%）
- **链路已确认正常**：`YawDOF=True`、`YCMode=5`（FCR DLL 偏航速率指令）、ROSCO `Y_ControlMode=1`
- **根因假设**：运行时偏航控制器依赖状态帧 `nacelle_heading_rad` 反馈；`farm_control_runtime/src/rosco_api.c:170` 在 `valid=0` 时取其 NaN → 速率指令失效
- **已尝试**：绝对 yaw → `yaw_delta` 直通通道；实测 T1 仍 −0.06%（**未解决**）
- **待做**：扩展状态导出 `valid / yaw_target_heading_rad / yaw_seq_applied / nacelle_vane_rad`，判定"命令是否到达 runtime"；若到达而机舱不动 → 需动 C 侧 DLL 并重编译

### 🔴 P2 运行间非确定性 ~2%
- 同一 yaw=0 工况两次运行 farm 功率 3275 vs 3338 kW（**1.9%**），T2/T3 分机差异可达 ~3%
- 疑因 FAST.Farm OMP 20 线程的浮点归约顺序差异
- **影响**：单 seed 配对比较的噪声底；需固定 OMP 线程数（或确认多 seed 平均足以压噪）

### 🟠 P3 状态帧部分字段不可靠
- `rotor_ct / rotor_cp = 0`（runtime 未填充）→ Ct/Cp 诊断不可用（标定 map 该列无效）
- `nacelle_heading_rad` 出现垃圾值（T2/T3 达 20610°）→ 依赖它的换算/诊断不可信

### 🟠 P4 资源与产能
- 盒生成慢：13/18，剩 5 盒 ≈20 h；全 72 盒（~103 GB）磁盘方案未定（当前 D: 约 104 GB 空闲）
- 内存 15.8 GB：4 并发 TurbSim 曾打满导致浏览器卡死 → 现固定 2 并发
- 仿真与盒生成必须错峰（1200 s 仿真读盒峰值 ~2.7 GB）

### 🟡 P5 工程债
- 最新改动未提交、未跑 300 s 门禁
- 临时/探针目录未清理（`_probe_c`、`run_smoke*`、`run_quick2` 等）
- 旧 `steady/` 子包未删；README 与 V3 未核对

### 🟡 P6 尚未开始的阶段
- **Phase 4**：FLORIS 校准后直优 `a_cmd`（P0-6 主链，LUT 加 a_cmd 列）
- **Pilot / Full DOE**（486 / 2592）
- **DEL**：turbine OutList 缺载荷通道（RootMyb / RotThrust 等）需补；多 seed 镜像未做
- Timestep B（0.05 短盒）可选未做

---

## 四、建议的下一步顺序
1. **P1 yaw 生效性**（最高优先，决定 yaw 实验能否成立）：导出 `valid / yaw_target / yaw_seq_applied` 定位断点
2. **P2 非确定性**：固定 OMP 线程数并复测（单次对比即可）
3. 提交 `yaw_delta` 改动 + `_fill_boxes.py`，跑一次 300 s 门禁
4. 盒生成继续滚动（2 并发）；到 18/18 后启动 pilot 分片（1 worker）

---

*快照生成：2026-09-21 10:29；分支 dev/farm-control-runtime，最新提交 be47cbe*
