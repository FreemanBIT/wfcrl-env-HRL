# WFCRL — 风电场控制仿真与场控闭环（ZeroMQ 架构）

风电场景控闭环控制框架：底层驱动 **FAST.Farm**（高保真）与 **FLORIS**（工程尾流模型），
场控指令经 **ZeroMQ** 传输到集成在 DISCON DLL 内的 `farm_control_runtime` 中间层，
翻译为 ROSCO 外部接口变量执行；状态（功率/载荷/方位/流场）按 ~1s 统计帧回传场控。

## 仓库组成（当前架构）

```text
wfcrl/                       # 仿真库（无 RL 依赖）
  engine/fastfarm_zmq.py     # ★ 唯一 FAST.Farm 运行引擎（FastFarmZmqEngine，ZMQ 双通道）
  transport/                 # farm_protocol / farm_zmq_client（帧 ABI：命令 5672B / 状态 23096B）
  config/ mdp/               # 配置 / RL 桥接（保留，非场控主链）
  compat.py                  # 旧 import 路径兼容层（Deprecated）

closed-loop-control/         # 闭环控制器层（A/B/C/greedy，Mock 可离线跑）
  closedloop/zmq_farm.py     # ★ 新架构统一入口（ZmqFarmBridge / ZmqPlant / launch_farm）
  closedloop/controller_factory.py   # 控制器统一构建
  closedloop/control_mode.py # 3 模式：YAW=1 / INDUCTION=2 / YAW_INDUCTION=3（双通道）
  examples/example_fastfarm_launcher.py   # ★ 唯一 FAST.Farm 闭环示例入口

farm_control_runtime/        # 场控中间层 C 库（离线集成 DLL / RT 独立链接，同一份代码）
  src/ include/ rosco/       # CommandStore / YawActionManager / InductionSupervisor / 聚合器
  transport/                 # zmq / modbus 双传输（modbus_map.yaml 冻结）
  offline/                   # 离线 Provider（avrSWAP 探针）+ FCR_Step 10ms 链路
  tests/                     # 单元 + 集成（ZMQ 端到端）

induction_vs_yaw_study/      # 诱导 vs 偏航 对比研究（LUT + FAST.Farm ZMQ 回放）
docs/farm_control/           # 协议决策/ICD/交付/集成指南/重构计划与复盘
```

## 信息流（离线闭环）

```text
场控 Python（Controller A/B/C/greedy / 研究回放）
  Cmd{ yaw_delta_deg（偏航动作指令，CW+ 增量 · SCADA 语义）
       yaw_deg（绝对目标，控制器内部，桥内换算为动作指令）
       induction_a（诱导因子，无量纲，a=1/3 贪婪）}
  -> ZmqFarmBridge（量纲/协议翻译 + seq/TTL）
  -> FastFarmZmqEngine -> FarmZmqClient（PUB/SUB，bind 5556/5557）
  -> ZeroMQ FarmCommandFrame -> DISCON_WT1.dll 内 farm_control_runtime
       CommandStore（可靠性）-> YawActionManager（增量->绝对目标锁存）
                              -> InductionSupervisor（a->ct_ref/power_ratio/min_pitch）
  -> FcrRoscoExternalSetpoint -> ROSCO（10ms 外部偏航目标+变桨/扭矩参考）-> OpenFAST
  -> Offline Provider 探针 + FCR_PublishState -> state_store(10ms) -> 1s 统计聚合
  -> FarmStateFrame(~3s) -> ZeroMQ -> 客户端按 turbine_id 合并 -> TurbineMeas -> 场控

时序：内部 10ms；闭环控制步 dt=3s；状态帧约 3s（1s 统计窗口）。
可靠性：命令帧每机独立 seq、TTL 过期回退、乱序拒绝（CommandStore）。
```

## 快速开始（FAST.Farm 闭环 300s）

```bash
cd D:\HR_Project\wfcrl-env-HRL

# 湍流入流（300s+ 建议使用 FarmInputs/ 下 TurbSim .bts，如 inflow_08ms_TI10_s8D.bts；
#  稳态 WindType=1 在 prebuilt 二进制下存在 Grid4D 时间墙 ~260s）

# 闭环（greedy 冒烟 / Controller B 完整）
python closed-loop-control\examples\example_fastfarm_launcher.py --duration 300 \
  --controller B --mode 3 --case 6T --wind_speed 8 --wind_time_series inflow_08ms_TI10_s8D.bts

# 直连通道示例（不经闭环控制器）
python examples\example_FASTFarm_zmq.py --mode 3 --a 0.20 --steps 12

# Mock 闭环（无需 FAST.Farm）
python -m closedloop.demos.run_demo --controller B --mode 3 --duration 400
```

## 文档

- `wfcrl/simulators/fastfarm/src_legacy_bridge/` —— 旧文件桥 Fortran（DISCON_bridge/ZeroMQInterface）已移此处
- `docs/farm_control/FASTFARM_RT_PROVIDER_ICD.md` —— 实时状态提供 ICD
- `docs/farm_control/REALTIME_DELIVERY.md` / `RT_INTEGRATION_GUIDE.md` —— RT 集成（runtime 在调度器）
- `docs/farm_control/PROTOCOL_DECISIONS.md` —— 通道语义/单位约定
- `docs/farm_control/REFACTOR_PLAN.md` —— ZMQ 收敛重构计划与执行记录
- `docs/farm_control/PROJECT_REVIEW.md` —— 工程现状 review

## 分支与发布

- 主开发分支：`dev/farm-control-runtime`（勿修改 main/master）
- 里程碑 tag：`phase11-rt-delivery`（ZMQ 传输 + RT Provider ICD）
