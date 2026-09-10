# PROJECT_REVIEW — 场控机制更新后的工程现状与重构建议


> 更新：本 review 中的重构建议（P0-P3）已由 REFACTOR_PLAN 执行（F/A/B/E/C/D 全部完成，
> 每步 300s 仿真回归通过；详见 REFACTOR_PLAN.md 执行记录）。遗留项仅在 P3（测试加固）与文档收尾。
生成日期：2026-09；分支：dev/farm-control-runtime（fab966c+）。

## 1. 当前架构（已实现）

```text
closed-loop-control（控制器层，Python）
  Controller(A/B/C/greedy) -> CommandArbiter -> ZmqFarmBridge(Cmd 映射)
    -> FastFarmZmqEngine -> FarmZmqClient(PUB/SUB, bind 5556/5557)
    -> ZeroMQ -> 每 WT 进程 DISCON_WT1.dll 内 farm_control_runtime
    （CommandStore/YawActionManager/InductionSupervisor/聚合器/Provider 探针）
    -> ROSCO 10ms 执行 -> 状态帧(~3s) 回传 -> 合并/解析 -> TurbineMeas -> 控制循环
```

连接：命令帧(5672B)/状态帧(23096B) ABI 冻结；控制步 dt=3s、内部 10ms、
状态帧约 3s；每机独立 seq/TTL；风机数 <=64 动态。

## 2. 双轨/遗留清单（旧机制仍存活的位置）

| # | 位置 | 旧机制 | 状态 | 处置建议 |
|---|---|---|---|---|
| L1 | wfcrl/engine/fastfarm_continuous.py | 文件桥 wait_step/_write_controls_file/_read_measurements_with_poll | 类被 FastFarmZmqEngine 复用（setup/reset/_farm_base），I/O 方法在新 DLL 下失效 | 拆分：保留案例生成/启动设施，I/O 方法标注 deprecated |
| L2 | examples/example_FASTFarm.py | ContinuousFastFarmInterface + ControlInput 5-mode（controls.txt） | 当前 FCR DLL 下不可运行 | 顶部加 Deprecation 说明，指向 examples/example_FASTFarm_zmq.py；或改造为调新引擎 |
| L3 | closedloop/bridge.py::FarmBridge | 文件桥（controls.txt/measurements_T*） | 保留但新 DLL 下不可用 | 标注 legacy；ZmqFarmBridge 已实现（二者并存，文档分离） |
| L4 | closedloop/runner.py::FastFarmPlant | 文件桥 Plant | 同上 | 标注 legacy，ZmqPlant 已实现 |
| L5 | closedloop/wfcrl_plant.py::WFCRLPlant | 绑定旧包 wfcrl.interface（已不存在） | 不可用 | 标记 deprecated 或删除；FastFarmZmqPlant 已实现并存 |
| L6 | closedloop/__init__.py | docstring/__all__ 仍讲文件桥（DISCON_bridge.f90） | 过时且未导出新类 | 更新 docstring；__all__ 增加 Zmq 导出（本 review 一并提交） |
| L7 | farm_control_runtime/offline/harness/（p3/p6 脚本） | shm 注入时代 harness | 仍可跑（DLL 保留 shm 通道） | 标注历史 harness（shm），说明当前主链路为 ZMQ |
| L8 | induction_vs_yaw_study/ | 独立研究（旧接口） | 存档性目录 | 保留，不入主链；README 标注归档 |
| L9 | wfcrl/compat.py / compat_data.py | 兼容 shims | 有用（旧 import 路径） | 保留，头注释已标 Deprecated |
| L10 | closedloop/examples/ 三个脚本 | 已重写为新架构 | 基本一致 | build_controller 等重复代码 -> 抽取 controller_factory |
| L11 | wfcrl/engine/fastfarm_step.py / floris.py / mdp/ | 其它仿真器（非场控链路） | 独立功能 | 保留 |
| L12 | wfcrl/engine/__init__.py | 中文注释为 GBK 编码（读作乱码） | 功能不受影响 | 后续统一转 UTF-8（一次性 diff） |

## 3. 其它发现

- 安装/入口分散：closed-loop-control 通过 sys.path 注入使用，未 pip install；
  建议保持现状，把唯一入口脚本固化到 examples（已做 launcher）。
- Controller B 计算成本：EnKF 每步 6~17s，MPC 每 20s；300s/100 步 wall 约 15~25min；
  建议把 EnKF 间隔与 MPC t_ctrl 做成 CLI 参数（--enkf_every / --mpc_t_ctrl）。
- 状态帧合并：修复后（帧头偏移 56）多机正常；仍建议补充合并器单元测试。
- 集成测试：farm_control_runtime/tests/integration 已面向新架构；
  closed-loop-control 尚无 FAST.Farm 冒烟测试，建议把 6T/60s greedy 固化为可选 smoke。
- git 卫生：FarmInputs/*、*.bts、__simul__、results 等已忽略；
  模板目录 5 个约 28MB bts 已入库（历史），如介意可后续移除或转 LFS。

## 4. 重构路线（建议按序执行）

### P0 文档与标注对齐（低风险，可立即做）
- [x] closedloop/__init__.py docstring + __all__（本 review 一并提交）
- [ ] README.md / closed-loop-control/README.md 顶部增加新架构指针（ZeroMQ 链路图）
- [ ] 文件桥类（FarmBridge/FastFarmPlant/WFCRLPlant/example_FASTFarm.py）加
      DeprecationWarning/docstring 标注，注明 FCR DLL 下不可用、仅供历史参考
- [ ] harness p3/p6 与 induction_vs_yaw_study 顶部加归档说明

### P1 结构收敛（中风险，纯移动/合并 + 兼容别名）
- [ ] 新建 closedloop/zmq_farm.py：集中 ZmqFarmBridge + ZmqPlant + FastFarmZmqPlant
      + launch_farm（从 bridge.py/runner.py/wfcrl_plant.py/zmq_pipeline.py 迁移）；
      旧位置保留兼容 import
- [ ] 抽取 closedloop/controller_factory.py：三份 example 的 build_controller 合并
- [ ] wfcrl/engine/fastfarm_continuous.py：controls/measurements 相关方法统一标注
      deprecated，类 docstring 写明仅供 FastFarmZmqEngine 复用设施
- [ ] wfcrl/engine/__init__.py 转 UTF-8（一次性）

### P2 移除（需评审确认后执行）
- [ ] 删除 closedloop/wfcrl_plant.py 的 WFCRLPlant（绑定已删除的 wfcrl.interface）
- [ ] 删除/归档 examples/example_FASTFarm.py（旧 5-mode 文件桥版）
- [ ] 若仓库内无任何调用：删除 FarmBridge / FastFarmPlant（文件桥），保留 Zmq 系
      （修改前 grep 确认无引用，含 tests/demos）

### P3 加固与测试
- [ ] 合并器单元测试（farm_zmq_client：构造多机帧 -> merged 正确）
- [ ] closed-loop-control 增加可选 smoke：6T/60s greedy + user bts（pytest marker fastfarm_smoke）
- [ ] Controller B 计算预算文档化 + --enkf_every/--mpc_t_ctrl CLI
- [ ] Modbus 传输端到端（python client -> modbus server -> poll）固化脚本

## 5. 现状健康度小结

- 主链路（ZMQ 场控闭环）：已可用并通过 6T/300s 实测；
- 主要乱点在：历史文件桥代码与说明残留（L1-L9）、三份 example 重复代码（L10）、
  编码/文档旧页（L6/L11/L12）；功能上无冲突，维护面需按 P0/P1 收敛。