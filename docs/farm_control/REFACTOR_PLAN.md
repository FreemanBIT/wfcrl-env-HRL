# REFACTOR_PLAN — ZMQ 场控链路收敛与架构重构（四部分修改计划）

状态：主体已执行完成（2026-09，dev/farm-control-runtime @ bc742ba，全部步骤经 300s 仿真回归）。
原则：先删后验、每步可回滚（git revert / 独立 commit）、保持 wfcrl.compat 导入兼容。

================================================================
第 1 部分：删除旧机制文件与脚本（只保留 ZMQ 场控链路）
================================================================

### 1.1 Fortran 旧桥（FCR 宏下不再编译）
- 删除 wfcrl/simulators/fastfarm/src/DISCON_bridge.f90（文件桥）
- 删除 wfcrl/simulators/fastfarm/src/ZeroMQInterface.f90（legacy ZMQ 接口）
- 删除 compile.bat / compile_rosco.bat（旧编译入口），统一用 _compile.py
- 验证：_compile.py --mode farmcontrol 产物与 fab966c 一致（git diff DLL 无变化）

### 1.2 Python 旧文件桥类/脚本
- closedloop/bridge.py：删除 FarmBridge 及文件解析正则，仅保留 ZmqFarmBridge
  （文件保留原名，以保证既有 import 兼容）
- closedloop/runner.py：删除 FastFarmPlant（文件桥 Plant），保留
  Plant/MockPlant/ZmqPlant/ClosedLoopRunner/RunResult
- 删除 closedloop/wfcrl_plant.py（WFCRLPlant 绑定的 wfcrl.interface 已不存在；
  FastFarmZmqPlant 迁入第 3 部分新建的 zmq_farm.py）
- 删除 examples/example_FASTFarm.py（旧 5-mode 文件桥示例；替代示例见第 2/4 部分）
- closedloop/demos/test_smoke.py：删除文件桥 round-trip 用例；
  run_demo.py 明确只使用 MockPlant（docstring 已声明）
- closed-loop-control/tests/test_units.py：删除基于 measurements_T 解析的用例
  （如 mip1/moop1 解析断言），保留 Mock 与 ZMQ 桥相关用例
- 删除 farm_control_runtime/offline/harness/ 下的 p3/p6 脚本（shm 时代历史 harness）；
  shm_command_source.c 保留（DLL 内调试通道，FCR_DISABLE_SHM 开关仍可用）

### 1.3 待确认删除（需你拍板）
- induction_vs_yaw_study/：独立研究目录（旧接口），建议整目录移出主链
  （git rm 后可从历史恢复；或移到 docs/archive/ 指针）
- 顶层非代码目录（Case File/ results/ __simul__/ dist/）：不动（多为产物/样例）
- 模板目录 5 个约 28MB 的 bts：保留（wfcrl demo 资源）

================================================================
第 2 部分：仿真脚本统一为 ZMQ 链路
================================================================

### 2.1 三份 closed-loop 示例合并为唯一入口
- 删除 closed-loop-control/examples/example_fastfarm_bridge.py 与
  example_fastfarm_wfcrl.py（功能与 launcher 重复）
- example_fastfarm_launcher.py 作为唯一示例入口，统一参数：
  --case --wind_speed --wind_direction --dt --duration --controller --mode
  --wind_time_series --cmd_port --state_port --seed --plot --mpc_t_ctrl --enkf_every
- build_controller 逻辑抽到 closedloop/controller_factory.py（三份去重）

### 2.2 顶层 examples 对齐
- examples/example_FASTFarm_zmq.py：保留并改造（详见第 4 部分模式简化）；
  成为唯一的直连通道（不走 closedloop 控制器）示例
- examples/example_FASTFarm.py：删除（1.2 已列）
- examples/example_floris.py：FLORIS 独立示例，保留不动

### 2.3 引擎层收敛
- wfcrl/engine/fastfarm_continuous.py：删除文件桥 I/O 方法
  （wait_step/_write_controls_file/_read_measurements_with_poll/_write_initial_controls/
  _fix_initial_yaw/set_fixed_yaw），保留 setup/reset/_farm_base 等案例设施
  （FastFarmZmqEngine 复用点）；类 docstring 更新
- wfcrl/engine/fastfarm_zmq.py 为唯一仿真运行入口
- wfcrl/compat.py 导出保持（ContinuousFastFarmInterface 仅剩设施 API）

### 2.4 集成测试与工具
- farm_control_runtime/tests/integration/（已 ZMQ）不动，补充合并器单元测试
- 新增 closed-loop-control/tests/test_zmq_smoke.py（pytest marker fastfarm_smoke）：
  6T/60s greedy + 用户 bts，验证闭环冒烟（可选执行）

================================================================
第 3 部分：工程架构归类与重构
================================================================

### 3.1 目标结构（Python 侧）
```text
wfcrl/
  engine/fastfarm_core.py        # 案例生成/启动设施（自 continuous 拆出）
  engine/fastfarm_zmq.py         # 唯一运行引擎（组合 core + ZMQ）
  engine/fastfarm_step.py floris.py mdp/ ...   # 其它仿真器，保留
  transport/                     # farm_protocol / farm_zmq_client（不变）
  compat.py                      # 兼容导出（不变，Deprecated 标注）

closed-loop-control/closedloop/
  zmq_farm.py                    # 【新】ZmqFarmBridge + ZmqPlant + FastFarmZmqPlant
                                 #       + launch_farm/build_case（行为不变，集中存放）
  controller_factory.py          # 【新】build_controller（三示例共用）
  bridge.py                      # 只留 from .zmq_farm import * 兼容行
  runner.py                      # Plant/MockPlant/ZmqPlant/ClosedLoopRunner
  types.py control_mode.py sensing.py evaluate.py induction.py surrogate.py
  case_config.py config.py wind_schedule.py base_controller.py
  scheme_a/ scheme_b/ scheme_c/  # 控制器（第 4 部分统一接口）
  demos/ tests/

closed-loop-control/examples/
  example_fastfarm_launcher.py   # 唯一 closedloop 示例
  plot_simul.py _print_*.py      # 分析工具（保留）

farm_control_runtime/
  src/ include/ rosco/ transport/{zmq,modbus}/ offline/provider/ config/ tests/
  offline/harness/               # 删除（1.2）

wfcrl/simulators/fastfarm/src/
  DISCON.F90 Controllers.f90 Constants.f90 ControllerBlocks.f90 Filters.f90
  Functions.f90 ReadSetParameters.f90 ROSCO_*.f90 SysGnuWin.f90 ExtControl.f90
  _compile.py                    # 唯一编译入口；DISCON_bridge/ZeroMQInterface 已删
```

### 3.2 操作要点
- 纯移动/合并，不改行为；旧位置保留一行兼容 import，git grep 逐项确认
- closedloop/__init__.py 导出随 3.1 更新（已具备 Zmq 导出）
- wfcrl/engine/__init__.py 转 UTF-8（一次性，仅注释编码）
- README / closed-loop-control/README.md / INTEGRATION_GUIDE.md 更新为 ZMQ 链路说明

================================================================
第 4 部分：5-Mode 收敛为“偏航 + 诱导因子”双通道
================================================================

### 4.1 现状问题
- 5 个 mode 源于文件桥时代（torque/pitch 直接通道）；新架构下
  FCR 仅两条控制通道：yaw_delta（偏航）与 induction_ref（诱导因子 a）
- 现有 ControlMode：YAW/TORQUE/PITCH/YAW_TORQUE/YAW_PITCH —— TORQUE/PITCH
  实为同一条 induction 通道的不同投递口径，且 ZmqFarmBridge 中 pitch 是近似映射、
  torque 被忽略——语义冗余且易误导

### 4.2 新模式设计（3 个）
```text
ControlMode  (新)         决策变量           下发通道
  YAW        = 1          yaw_deg            yaw_delta（CW+ 增量，绝对值换算）
  INDUCTION  = 2          a ∈ [0,1/3]        induction_ref
  YAW_INDUCTION = 3       yaw_deg + a        双通道（推荐主模式，即 Scheme B 默认）
```
- types.Cmd 收敛为两个业务字段：yaw_deg（绝对机舱方位，deg）与
  induction_a（无量纲）。power_mw/pitch_deg/torque_nm 保留为兼容只读属性：
  power_mw := induction_to_power(a, U)（仅诊断用）；pitch/torque 置 None
- ControlModeSpec.to_controls(turbine_id, yaw_deg, a)：直接生成 yaw+a 命令；
  删除 use_power_for_torque / _DERATE_CHANNEL 逻辑
- schemes A/B/C：优化器接口不变（本就输出 yaw_sol/a_sol）；内部删除 mode=2/4/5
  相关分支；fallback/安全默认（a=1/3）保留
- ZmqFarmBridge.write：仅 yaw->delta、a->induction_ref 两条映射；
  power/pitch/torque 兼容入口置为 DeprecationWarning + 换算提示（不传输）
- 示例 CLI：--mode 取值 1/2/3；旧 4/5 传入时映射为 3 并告警；
  删除 --power/--min_pitch/--pitch 参数，保留 --yaw_amp/--yaw_period
- 文档同步：PROTOCOL_DECISIONS.md（通道语义）、control_mode.py docstring、
  example_FASTFarm_zmq.py（其 5 个模式同步改为 3 个：纯偏航/纯诱导/双通道）

### 4.3 验证
- 单元：ControlModeSpec 新枚举；Cmd 兼容属性；power<->a 换算单向测试
- 冒烟：6T/60s，--mode 1（仅偏航）、--mode 2（仅诱导降载）、--mode 3（双通道，
  观测降载与偏航同时生效）

================================================================
执行顺序与验证（总）
================================================================

```text
Step 1  第 1 部分删除（一个 commit）    -> pytest(unit) + _compile.py OK + git grep 无旧引用
Step 2  第 2 部分脚本统一（一个 commit）-> py_compile + 6T/60s smoke（--controller greedy）
Step 3  第 3 部分结构调整（一个 commit）-> import 冒烟 + 6T/60s smoke
Step 4  第 4 部分模式收敛（一个 commit）-> 单元测试 + 6T/60s x3 mode smoke
Step 5  文档收尾（README/INTEGRATION_GUIDE/PROTOCOL_DECISIONS）
```

风险提示：
- 1.1 的 Fortran 删除前先确认 _compile.py 不再引用 DISCON_bridge/ZeroMQInterface
  （计划中已查：FCR 宏路径不编译它们；compile.bat 为旧入口）
- 第 1 部分待确认项（induction_vs_yaw_study）不在自动执行清单内，等你确认
- 所有删除/移动均可用 git revert 整步回滚；分支已有 phase11-rt-delivery tag 兜底

================================================================
附录 A — induction_vs_yaw_study ZMQ 化（方案 A：仅换驱动层）
================================================================

目标：保留研究目录结构与 LUT/换算/分析逻辑，只把 FAST.Farm 执行驱动
从文件桥（ContinuousFastFarmInterface + mode0/1）换成 ZMQ 链路；
默认保持旧口径（偏航静态锁定 + .outb 统计），保证与新结果可比。

### A.1 新增：replay/fastfarm_zmq_replay.py（约 200 行）
- 复用 closedloop.zmq_pipeline（build_farm_case/launch_farm）启动仿真：
  make_fastfarm_config 的布局/风况字段 -> FastFarmConfig（--wind_time_series 支持 bts）
- 命令下发（每控制步，静态 LUT 每步相同 / ZOH 保持）：
  * derating：LUT 的 power_target_mw -> ratio(=P/P_greedy) -> a = power_to_induction()
    （U 取 case.wind_speed_ms；沿用 closedloop.induction），下发 induction_ref + seq
  * yaw：两个模式：
    --yaw_mode static（默认，保口径）：首个状态帧后按 当前heading -> 目标absolute 换算
      一次性下发 yaw_delta 并每步重发（Heading latch 保持绝对目标不变，效果=静态失准，
      与旧“锁 DOF”的稳态一致）；
    --yaw_mode control：动态通道（每步重发目标，允许 ROSCO 偏航动态）
  * baseline：不下发任何命令（a=greedy 1/3 默认）
- 推进/采集：wait 到 low_step>=目标步（bridge.wait_step）；仿真结束后仍解析
  .outb 取功率（保持研究统计口径），不做闭环状态帧直接统计（除 yaw_mode 需要的 heading）
- 返回结构复用 SimulationOutput（time/power_mw/wind_speed/yaw_deg/pitch/torque/rotor/
  blade_loads + 附 yaw_cmd/ratio/power_target）→ 与旧 replay_fastfarm 输出兼容

### A.2 编排与入口修改
- run/run_replay.py：--sim fastfarm 时调用 replay_fastfarm_zmq；新增 --yaw_mode；
  --controls 不变（baseline/yaw/derating）
- steady/replay_steady.py、steady/run_replay_steady.py：同样切换（稳态 WindType=1 也可，
  但 300s+ 建议 bts；研究用例通常 <300s 不受 260s 影响）
- tests/test_conversion.py 等纯逻辑测试不动；新增驱动单测（mock engine）：
  验证 derating 下发 a、yaw_mode static 下发 delta 转换

### A.3 行为差异（文档标注于 README）
- 旧浏览器 mode1 功率跟踪（前馈+PI）-> 新 a 通道（诱导因子直接设定）：稳态功率等价，
  动态跟踪特性不同（研究用静态 LUT，无影响）
- 偏航：旧=锁 DOF 静止；新 static 模式=ZMQ 下发一次并 latch 保持（稳态等价）；
  control 模式=动态
- 断点续跑/失败日志（run_log.csv）/aborted 部分结果逻辑保留

### A.4 验证（实施后）
- 3 机 + inflow_08ms_TI10_s8D.bts（或研究原有的 90m_08mps.bts）跑 60s：
  baseline / derating(ratio 0.7) / yaw(失准 20°) 三组，抽查功率曲线合理性与
  与旧记录（docs/PROBE_RESULTS.md / results/）趋势一致
- git grep 确认研究目录不再引用 ContinuousFastFarmInterface

### A.5 执行顺序
- Step A1: replay/fastfarm_zmq_replay.py + 单测
- Step A2: run_replay.py / steady 切换 + --yaw_mode
- Step A3: 60s 三组回放验证 + README 更新
- 独立 commit，可单独 revert（已执行：A1-A3 落地并跑通）


================================================================
执行记录（完成度）
================================================================

| 步骤 | commit | 300s 回归 |
|------|--------|-----------|
| F 偏航动作指令直通 | 51244cf | ✓ 100/100 (310.2s wall) |
| A 删除旧文件桥 | c3bbaf6, acbfe54 | ✓ 100/100 (310.2s wall) |
| B 示例合一+factory | bed0215 | ✓ 100/100 (294.6s wall) |
| E 研究目录 ZMQ 化 | 3f63a79 | ✓ 100/100 (288.1s wall) + 研究回放 derating ok |
| C 架构归类 | c085050 | ✓ 100/100 (287.1s wall) |
| D 5-mode→3 模式 | 4339f1e, f278462, bc742ba | ✓ 100/100 (286.6s wall, mode 3) |

待办（非阻塞）：README/INTEGRATION_GUIDE 文档收尾（Step5）、合并器单测、6T smoke 固化为可选 pytest。
