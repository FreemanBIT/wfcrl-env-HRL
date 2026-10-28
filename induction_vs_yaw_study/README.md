# induction_vs_yaw_study — 诱导 vs 偏航 对比研究

针对 3 × NREL 5MW 单列风场的 **诱导因子（降载）控制 vs 偏航控制** 对比研究：
在 **FLORIS** 中针对参数化工况（风速/风向/湍流/间距）离线求解 **最优偏航 LUT** 与
**最优功率设定点（降载）LUT**；在 **FAST.Farm** 中回放两组 LUT；最后交叉对比
FLORIS 与 FAST.Farm、偏航与降载四个组合的功率与载荷。

> 本包是 `wfcrl-env-HRL` 工程的叠加层，与 `wfcrl/` 并列；不修改 `wfcrl/` 既有代码，
> 也不修改风场模型库/模板。

## 架构变更（REFACTOR_PLAN 附录 A 已落地）

```text
旧：ContinuousFastFarmInterface + ControlInput 5-mode + controls.txt 文件桥  ->  已移除
新：replay/fastfarm_zmq_replay.py（ZMQ 驱动）
     仿真：FastFarmZmqEngine + ZmqFarmBridge（复用 closedloop.zmq_pipeline 案例/风况配置）
     下发：偏航动作指令（yaw_delta，CW+；yaw_mode=static: 首步锁存 / control: 逐步重发）
           + 诱导因子 a = power_to_induction(LUT 的 power_target_mw)
     测量：仿真结束后解析 .outb（统计口径与旧实现一致，可与历史结果对比）
入口：run/run_replay.py --sim fastfarm（其余参数不变；新增 --yaw_mode/--wind_time_series）
```

注意：旧设计结论中“诱导因子 a 仅作诊断量、不下发”已过时——新架构下 a 是**直接下发通道**
（induction_ref），LUT 的 P_target(MW) 在回放驱动中换算为 a。

## 运行环境要点

- **OMP 线程数必须 ≤3**（`WFCRL_STUDY_OMP_NUM_THREADS`）：OMP=1 逐位确定但慢；
  OMP=3 是速度/稳定的折中；**OMP>3 会破坏 FAST.Farm 尾流模型数值稳定性
  （实测 OMP=6 时 37% 的 case 出现机组顺桨停机）**。详见 `docs/DOE_OPERATIONS.md`。
- **DOE 只能单 worker**：本机内存 15.8GB，系统常占 ~10.8GB，每 case 峰值 ~5GB
  （bts 读取 2.4–2.9GB + 仿真 2GB）。
- **命令 TTL**：研究侧 `_send_cmds` 直调 engine 下发 yaw_delta/induction，
  `ttl_s=1e6`（bridge 默认 60s 会在静态对风场景过期回退，P1 根因）。
- **Ct/Cp 诊断列**：replay 由 thrust/power/盘面风回填 `rotor_ct/rotor_cp`
  （运行时不填；均匀盒下受遮挡机组会高估，标定用真盒并 outb 校核）。

## 目录

```text
cases/       3 机案例与工况网格（grid.yaml 全网格 + grid_full_u*.yaml 按风速）
conversion/  LUT 存储口径（绝对功率 MW）与 power<->ratio<->a 换算
optimize/    FLORIS 偏航/降载优化 + calibrated_a_cmd（标定 a_cmd 校准曲线）
lut/         LUT schema/插值（normalize_case_id：LUT 与 seed 无关）
replay/      fastfarm_zmq_replay.py（★ ZMQ 回放驱动）/ floris_replay.py / schedule.py
run/         批跑与运维：
  run_fastfarm_batch / _case_worker   批跑（子进程隔离/resume/超时/顺桨过滤）
  build_case_manifest                  manifest 构建（QA 门禁）
  build_luts / calibrate_induction    LUT 与诱导标定
  analyze_doe / check_doe_quality     结果汇总与质量审计
  doe_status                          进度看板
  compute_del_batch / outb_only_run   DEL 端到端 / 乙模式 outb
  add_load_channels                   载荷通道注入
  regress_300s / diag_yaw_effect      门禁与偏航诊断
  _fill_boxes                         风盒补齐
analysis/    metrics / plots / doe_plots（6 张结果图）/ del_analysis（rainflow）
tests/       单测（58 passed）
docs/        STATUS / HISTORY / DOE_OPERATIONS / EXECUTION_LOG / PROBE_RESULTS
```

> 注：`steady/` 子包（WindType=1 稳态回放）已于 2026-09-21 移除——预编译
> Grid4D 时间墙问题使其不再实用，统一使用 TurbSim bts 风盒（见 docs/STATUS.md）。

## 运行

```bash
# FLORIS 回放
python -m induction_vs_yaw_study.run.run_replay --sim floris --controls yaw derating baseline

# FAST.Farm ZMQ 回放（单 case）
python -m induction_vs_yaw_study.run.run_replay --sim fastfarm --controls yaw derating baseline \
  --wind_time_series inflow_08ms_TI10_s8D.bts

# Full DOE（详见 docs/DOE_OPERATIONS.md）
$env:WFCRL_STUDY_OMP_NUM_THREADS = '3'
python -m induction_vs_yaw_study.run.run_fastfarm_batch \
  --manifest induction_vs_yaw_study/results/_manifests/case_manifest.csv \
  --out induction_vs_yaw_study/results/run_full_doe \
  --workers 1 --max-retries 3 --resume --controls baseline yaw induction

# 进度 / 质量 / 分析
python -m induction_vs_yaw_study.run.doe_status --run .../run_full_doe
python -m induction_vs_yaw_study.run.check_doe_quality --run .../run_full_doe
python -m induction_vs_yaw_study.run.analyze_doe --run .../run_full_doe
python -m induction_vs_yaw_study.analysis.doe_plots --analysis .../run_full_doe/analysis
```

> 断点续跑：`--resume` 跳过 status=done；`failed` 自动重试。
