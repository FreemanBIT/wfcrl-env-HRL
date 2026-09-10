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

## 目录

```text
cases/       3 机案例与工况网格（布局/风速/TI/间距）
conversion/  LUT 存储口径（绝对功率 MW）与 power<->ratio<->a 换算
optimize/    FLORIS 侧最优偏航/降载 LUT 求解
lut/         LUT 模式与插值
replay/      fastfarm_zmq_replay.py（★ ZMQ 回放驱动）/ floris_replay.py / schedule.py
run/         run_replay / run_compare / build_luts / stage0_probe
steady/      稳态工况回放（WindType=1；注意 >260s 有预编译 Grid4D 时间墙，建议 bts）
analysis/    指标与图
tests/       单测（conversion/lut/cases）
docs/        PROBE_RESULTS 等
```

## 运行

```bash
# FLORIS 回放
python -m induction_vs_yaw_study.run.run_replay --sim floris --controls yaw derating baseline
# FAST.Farm ZMQ 回放（默认；--wind_time_series 可指定 TurbSim .bts）
python -m induction_vs_yaw_study.run.run_replay --sim fastfarm --controls yaw derating baseline --wind_time_series inflow_08ms_TI10_s8D.bts
# 断点续跑：已存在产物自动跳过；--overwrite 强制重跑
```
