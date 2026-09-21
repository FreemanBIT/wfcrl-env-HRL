# STATUS — 诱导 vs 偏航研究：开发进度与问题（快照 2026-09-21 17:10）

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
| Induction 1T 标定 | 🟡 仅 U8 六点（a 0.10→1456 … 0.33→1588 kW，单调）——**作废**：当时 yaw 未生效且统计口径含过渡段，需按 §三.1 修复后重做 |
| **P1 yaw 有效性闭环** | ✅ 根因=TTL 60s 过期（§三.1）；180s/1200s/600s 三次带命令验证：seq=1、机舱恒 −30.1°、T1 P≈1133 kW（−31%） |

---

## 二、进行中
- **U8 风盒 13/18**（TI05 s1–5、TI10 s1–4、TI15 s1–4）；2 个 TurbSim 正在补齐剩余 5 盒（PID 29640，10:57 启动），单盒 ~7–9 h
- 内存 5 GB 空闲（固定 2 并发 TurbSim）
- **未提交改动**：`replay/fastfarm_zmq_replay.py`（yaw_delta + 长 TTL `_send_cmds`）、`run/_fill_boxes.py`、`run/diag_yaw_effect.py`（新增）、`run/_dbg2_window.py`（新增）
- **进行中实验**：1200s + `--roscolog`（ROSCO 126 通道 10ms 日志）复现偶发顺桨（见 §三.1 附注）

---

## 三、存在的问题（按严重度）

### ✅ P1 yaw 命令未生效 —— 已解决（2026-09-21）
- **根因**：`closedloop/bridge.py` 的 `_send_yaw_delta` 硬编码 `ttl_s=60.0`（induction 30.0）。静态对风场景只在 k=1 发一次命令，runtime 通道 t≈63s 过期回退 → ROSCO 内部目标恢复风向对齐 → 机舱以 0.5°/s 转回 0°。1200s 统计窗 [600,1200] 内机舱在 0°，故"yaw 无效"。全链路其余环节（传输/seq/锁存/ROSCO 状态机/ServoDyn）逐环验证正常。
- **修复**（研究侧，不动 wfcrl/closed-loop-control）：`replay/fastfarm_zmq_replay.py` 新增 `_send_cmds()` 绕过 bridge 直调 `engine.send_yaw_delta(..., ttl_s=1e6)`（`ttl_s=0` 会回落 runtime 默认 60s，必须显式大正值）；同法处理 induction（A_GREEDY 哨兵跳过）；绝对 `yaw_deg` 路径因单位不安全改为直接 raise。
- **验证**（uniform8_700s 盒，T1 yaw_delta=+30° CW 于 k=1 一次性下发）：
  - 180s（20260921_110947）：seq=1 保持，机舱 −30.15°，T1 P≈1135 kW（vs 未偏航 1650，−31%≈cos²30°）
  - 1200s（20260921_111818）：seq=1 + 机舱 −30.147° 保持至 t=1202s；T2 因尾流偏转恢复 1646 kW
  - 600s 复跑（20260921_142310）：全程健康，T1 P_mean=1133 kW
- **300s 门禁**：真盒 4195 kW、uniform 盒 3863 kW，均 PASS
- **附注（顺桨事件，转 P2 范畴）**：111818 那次 1200s 运行在 t≈524.2s 发生 T1 非命令性顺桨（pitch 0→90° @8°/s=PC_MaxRat、GenTq→0、P→0；T2 于 +96s 尾流消失后恢复 1740 kW）。已排除：SD_Mode=0、运行时无 fault/feather 路径、wfcrl 桥接块 `#ifndef` 排除、命令链（seq/target/机舱保持不变）、状态帧 k=174 全字段正常。已定位的贴边条件：`VS_TSRopt=0` → `VS_RefSpd` 被钳到 `VS_MinOMSpd=76.433 rad/s`，转矩 PI 把转速钉在该值（极限环 ±0.005），30° 偏航扭矩降 31% 加剧贴边；t≈518–521 转速缓慢下穿边界且衰减加速。**同配置 600s 复跑未复发**（ROSCO 内部态全程健康：PC_SpdErr=+46.47 恒定、PC_PitComT=0、PC_KP=−0.4）→ 判定为非确定性数值故障（与 P2 OMP 疑因同类），触发机制（桨距 PI 输出永久饱和于 PC_MaxPit 的路径）待 1200s+roscolog 复现运行（pwsh-9）定位。缓解预案：研究侧后处理 DISCON_IN 设 `VS_TSRopt>0`（转速离开 MinOMSpd 贴边）或统计窗剔除 P=0/pitch>80° 时段；该参数在 wfcrl 模板内，按约束⑤只能研究侧 post-process。

### ✅ P2 运行间非确定性 —— 已修复（2026-09-21，f297121）
- **根因确认**：FAST.Farm OMP 浮点归约顺序差异。修复：replay/diag 在 `engine.start()` 前设 `OMP_NUM_THREADS=1`（env `WFCRL_STUDY_OMP_NUM_THREADS` 可覆盖，`0`=旧行为）。
- **验证**：同 real 盒 baseline 两次 300s：farm/分机功率 **逐位一致**（4232.2834 kW，T1 100/100 样本 bitwise 相同）；修复前同 case 差 1.9%。
- **代价**：wall ~140s→352s（~2.5×），DOE 时间预算需重估；可试 OMP=2/3 找确定性/速度折中（pilot 前定）。
- **顺桨事件关联**：OMP 竞争是头号嫌疑；钉线程后复发概率预期大降，批跑器 feather_invalid 过滤逻辑已备（replay metadata）。

### ✅ P3 状态帧字段 —— 已修复（2026-09-21，f297121 + 4c0fb4e）
- **heading/vane 单位**：FCR_PublishState 此前把 ROSCO 内部**度值**直传 `*_rad` 字段（T2/T3 显示 20610° 垃圾、多命令重锁存会算出垃圾目标）。修复：publish 处 ×D2R；重编译 DISCON_WT1.dll。验证：180s yaw=30 诊断 `tgt=-30.00 nac=-30.00`（旧 −30.11/−1724）、锁存目标精确 −0.5236 rad、P=1136 kW；300s 门禁 PASS 4232 kW（基线零回归）。**多命令/control 模式现在语义正确**。
- **rotor_ct/rotor_cp**：运行时不填（需 OpenFAST 内部量），研究侧由 thrust+power+盘面风回填（replay）；均匀盒下受遮挡机组会高估（已注记），标定用真盒 + outb 校核。

### 🟠 P3 状态帧部分字段不可靠
- `rotor_ct / rotor_cp = 0`（runtime 未填充）→ Ct/Cp 诊断不可用（标定 map 该列无效）
- `nacelle_heading_rad` 出现垃圾值（T2/T3 达 20610°）→ 依赖它的换算/诊断不可信

### 🟠 P4 资源与产能
- 盒生成慢：13/18，剩 5 盒 ≈20 h；全 72 盒（~103 GB）磁盘方案未定（当前 D: 约 104 GB 空闲）
- 内存 15.8 GB：4 并发 TurbSim 曾打满导致浏览器卡死 → 现固定 2 并发
- 仿真与盒生成必须错峰（1200 s 仿真读盒峰值 ~2.7 GB）

### ✅ P5 工程债 —— 已清理（2026-09-21，e7135d9 + 4c0fb4e）
- ~~未提交~~：P1/P2/P3/P5 四批已提交（1ee69bb、f297121、e7135d9、4c0fb4e），每批 300s 门禁 PASS
- ~~探针目录~~：outb_probe*、run_smoke*、run_quick2、_probe_c、_probe_outb 已删
- ~~steady/ 子包~~：已删（Grid4D 时间墙问题，bts 为标准路径；引用核实为文档性）
- ~~README~~：目录树/运行要点已同步
- 附加：farm_control_runtime 的 build_windows.ps1 gcc 回退清单 + CMakeLists.txt 补全，**单测 39/39 恢复可运行**（此前根本编不过）

### 🟡 P6 尚未开始的阶段
- **Phase 4**：FLORIS 校准后直优 `a_cmd`（P0-6 主链，LUT 加 a_cmd 列）
- **Pilot / Full DOE**（486 / 2592）
- **DEL**：turbine OutList 缺载荷通道（RootMyb / RotThrust 等）需补；多 seed 镜像未做
- Timestep B（0.05 短盒）可选未做

---

## 四、建议的下一步顺序
1. ~~P1 yaw 生效性~~ ✅（1ee69bb）
2. ~~P2 非确定性~~ ✅ OMP=1 逐位可复现（f297121）；剩余：pilot 前试 OMP=2/3 速度折中；批跑器接线 feather_invalid 过滤
3. ~~P3 状态帧字段~~ ✅ 单位修复 + Ct/Cp 回填（f297121/4c0fb4e）；剩余：Induction 标定按 §一.4 作废结论重做（长 TTL + 稳态窗口径）
4. ~~P5 工程债~~ ✅（e7135d9/4c0fb4e）
5. **盒生成继续滚动**（U8 15/18，s5 对生成中、s6 三盒排队，预计明日）；到 18/18 后启动 pilot 分片（1 worker，OMP 固定 + 顺桨过滤）
6. U6/U10/U12 盒补齐 → 全 DOE

---

*快照生成：2026-09-21 17:10；分支 dev/farm-control-runtime，最新提交 4c0fb4e（P1/P2/P3/P5 全部闭环，P4/P6 随盒生成推进）*
