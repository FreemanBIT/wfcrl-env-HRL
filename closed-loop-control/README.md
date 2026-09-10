# closed-loop-control — ZMQ 场控闭环控制器层

面向 **WFCRL / FAST.Farm** 的数据驱动闭环控制器代码库（站点 NREL 5MW × 6 机，
2 排 × 3 列，行间距 4D=504m）。实现开发方案中 A/B/C 三套方案与全部共用模块，
配套完整测试与 demo（含贪婪基线对比）。

## 架构（当前：ZeroMQ 双通道）

```text
Controller(A/B/C/greedy)
  | Cmd{ yaw_delta_deg / yaw_deg / induction_a }
  v
ZmqFarmBridge + FastFarmZmqPlant + ZmqPlant（closedloop/zmq_farm.py 统一入口）
  v
FastFarmZmqEngine -> FarmZmqClient -> ZeroMQ(FarmCommandFrame 5672B)
  -> 集成 DLL 内 farm_control_runtime -> ROSCO -> 状态帧(23096B) 回传

命令通道（每机独立 seq/TTL）:
  yaw_delta_rad（偏航动作指令，CW+，绝对目标由 runtime 锁存）
  induction_ref（诱导因子 a，无量纲；a=1/3 贪婪=不下发）
控制模式（closedloop/control_mode.py，旧 5-mode 已收敛为 3 个）：
  1=YAW         仅偏航（a=1/3 贪婪）
  2=INDUCTION   仅诱导（yaw 对齐来流）
  3=YAW_INDUCTION 偏航+诱导（双通道，推荐主模式）
```

## 目录结构（当前）

```text
closed-loop-control/closedloop/
  zmq_farm.py              # ★ 新架构统一入口（ZmqFarmBridge/FastFarmZmqPlant/ZmqPlant/launch_farm）
  controller_factory.py    # build_controller 统一构建（含 --mpc_t_ctrl 透传）
  bridge.py                # ZmqFarmBridge（量纲/协议翻译；历史 FarmBridge 已删除）
  runner.py                # ClosedLoopRunner + Plant/MockPlant/ZmqPlant
  types.py                 # TurbineMeas / Cmd（yaw_delta_deg,yaw_deg,induction_a）/ FlowEstimate
  control_mode.py          # M9 3 模式（YAW/INDUCTION/YAW_INDUCTION）
  sensing.py / induction.py / surrogate.py / evaluate.py
  case_config.py / config.py / wind_schedule.py / base_controller.py
  scheme_a/ (FLORIS 再标定+稳态重优化)  scheme_b/ (FLORIDyn+EnKF+MPC 主方案)  scheme_c/ (灰箱)
  demos/ (greedy 基线 / run_demo(Mock) / compare_report / test_smoke)
  tests/                   # 单测（Mock/桥单元）+ _run_offline 离线执行器
closed-loop-control/examples/
  example_fastfarm_launcher.py   # ★ 唯一 FAST.Farm 闭环示例（--case/--controller/--mode/--wind_time_series/--mpc_t_ctrl）
  plot_simul.py _print_control_diagrams.py  # 分析工具

（历史 example_fastfarm_bridge.py / example_fastfarm_wfcrl.py 已合并/删除）
```

## 快速开始

### 1) Mock 闭环（无需 FAST.Farm，几秒出结果）

```bash
python -m closedloop.demos.run_demo --controller B --mode 3 --wind steady_8ms --duration 400 --out results/
python -m closedloop.demos.compare_report --controller B --mode 3 --duration 400 --out report.html
```

### 2) FAST.Farm 真机闭环（ZMQ 链路；300s+ 建议 --wind_time_series 用 TurbSim .bts）

```bash
python examples\example_fastfarm_launcher.py --duration 300 --controller greedy --case 6T --wind_speed 8 --mode 3
python examples\example_fastfarm_launcher.py --duration 300 --controller B --case 6T --wind_speed 8 --mode 3 --wind_time_series inflow_08ms_TI10_s8D.bts
```

参数：`--case`（3T/6T/...）、`--mode`（1/2/3）、`--wind_time_series`（.bts 文件名，相对 FarmInputs/）、
`--cmd_port/--state_port`（默认 5556/5557）、`--mpc_t_ctrl`（ControllerB 重规划周期 s，默认 20）。

### 3) 测试

```bash
python -m pytest closed-loop-control/tests closed-loop-control/closedloop/demos/test_smoke.py
# 无 pytest 环境：python tests\_run_offline.py
```

## 集成说明

- FAST.Farm 必须使用 FCR_FARM_CONTROL 构建的 DISCON_WT1.dll（旧文件桥 DLL 不兼容）；
- 传输端口 5556/5557 可通过环境变量 FCR_ZMQ_CMD_PORT/FCR_ZMQ_STATE_PORT 或 CLI 指定；
- 详细 ICD 与 RT 集成见 `docs/farm_control/FASTFARM_RT_PROVIDER_ICD.md` 与 `RT_INTEGRATION_GUIDE.md`。
