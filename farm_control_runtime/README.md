# farm_control_runtime — 场控中间层可移植核心（实时交付源）

`farm_control_runtime` 是风电场闭环控制唯一中间层库：接收 Farm Controller 的动作命令，
维护命令通道（yaw / induction 独立 seq），生成 ROSCO 外部设定点，并聚合多速率状态
（ROSCO ~10 ms / OpenFAST 额外量 ~10 ms / AWAE ~1 s）成 ~1 s 的 `FarmStateFrame` 供
Farm Controller 订阅。

## 架构位置

```text
Farm Controller
      |
 Transport (ZMQ / Modbus)
      |
 farm_control_runtime   ← 本目录
      |
 Modified ROSCO
      |
 OpenFAST / FAST.Farm
```

runtime 内部组件：`CommandStore` / `StateStore` / `StateAggregator` / `YawActionManager` /
`InductionSupervisor` / `SignalStatistics` / `Watchdog` / `ROSCO API` / `Transport API`。

## 目录

```text
include/   公共 ABI（Phase 1 冻结）：fcr_protocol/state_types/runtime/transport/
           provider_api/rosco_api/fastfarm_rt_provider
src/       核心实现（runtime/command_store/state_store/angle_convention/watchdog）
transport/ ZeroMQ（离线）/ Modbus（实时）实现
rosco/     FarmControlInterface.f90 / FarmControlCBindings.f90 / ROSCO_INTEGRATION.md
offline/   Offline FAST.Farm Provider 与离线 harness（不进入 RT 交付包）
config/    运行时配置样例（yaml）
tests/     单元测试（C）与集成测试（Python，Phase 8）
tools/     构建脚本与 ABI 检查
```

## 构建与测试

Windows：

```powershell
powershell -ExecutionPolicy Bypass -File tools/build_windows.ps1
```

Linux（或任意 cmake 环境）：

```bash
cmake -S . -B build && cmake --build build
./build/fcr_unit_tests
```

ABI 检查：

```bash
python tools/check_abi.py
```

## 红线（详见 docs/farm_control/PROTOCOL_DECISIONS.md）

1. ROSCO 10 ms 控制路径内禁止网络/文件 I/O，只读共享 setpoint；
2. `yaw_delta` ≠ `Y_MErrSet`；新 seq 只生成一次绝对目标并锁存；
3. yaw / induction 通道独立 seq，互不覆盖；
4. 角度单位转换一律经 `angle_convention.c`；RT 侧不得再次反号；
5. AWAE 1 s 数据 ZOH，不伪造 10 ms 时间戳；
6. Offline Provider 不得读 `.out/.outb` 给闭环提供状态。

## 依赖

- C11 编译器（gcc / MSVC / clang），无第三方库（Phase 1 核心）；
- ZeroMQ Transport：libzmq + pyzmq（Phase 7 起）；
- Modbus Transport：自实现 TCP 寄存器编解码（Phase 9 起）。

## 状态

| Phase | 内容 | 状态 |
|---|---|---|
| 0 | baseline 冻结 + tag | 完成 |
| 1 | 公共 ABI + runtime 骨架 | 完成（本版） |
| 2 | ROSCO API 无扰接入 | - |
| 3 | 偏航增量端到端 | - |
| 4 | Offline Provider 快速量 | - |
| 5 | AWAE 流场 + 聚合统计 | - |
| 6 | InductionSupervisor | - |
| 7 | ZeroMQ + Python Client | - |
| 8 | 离线端到端验证 | - |
| 9 | Modbus Transport | - |
| 10 | RT Provider ICD | - |
| 11 | 实时交付包 | - |
