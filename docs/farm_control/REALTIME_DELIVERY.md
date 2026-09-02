# REALTIME_DELIVERY — 实时交付包说明（Phase 11）

## 1. 交付物

`dist/wfcrl-rt-delivery-<ver>.zip`（由 `farm_control_runtime/tools/package_realtime_delivery.py` 生成）：

```text
wfcrl-rt/
├── MANIFEST.txt
├── runtime/core/
│   ├── include/    (fcr_*.h 全部 ABI，含 provider/transport/rosco 契约)
│   ├── src/        (runtime 核心：command/state/aggregator/yaw/induction/watchdog/rosco_api)
│   ├── rosco/      (Fortran 桥：FarmControlInterface/CBindings + ROSCO_INTEGRATION.md)
│   ├── offline/    (离线 Provider 与 harness)
│   ├── transport/  (zmq + modbus + ZMQ_PROTOCOL.md + modbus_map.yaml)
│   ├── config/     (yaml 配置模板)
│   ├── tests/      (unit 39/39 + integration 离线闭环)
│   ├── CMakeLists.txt / build_windows.ps1 / build_linux.sh / check_abi.py
├── python/         (farm_protocol / farm_zmq_client / engine 接入)
├── fastfarm_src/   (DISCON.F90 / Controllers.f90 宏集成层 + _compile.py)
├── bin/            (DISCON_WT1.dll = FCR_FARM_CONTROL 集成构建；DISCON_ROSCO_TEMPLATE.IN)
└── docs/           (BASELINE / PROTOCOL_DECISIONS / OFFLINE_VALIDATION / ICD / 集成说明)
```

## 2. 构建与验证（离线，验收基线）

1. 编译 runtime 单元测试：`build_windows.ps1` → `tests/unit/test_runner.exe`，39/39 通过；
2. 集成 DLL：`wfcrl/simulators/fastfarm/src/_compile.py --mode farmcontrol` → `bin/DISCON_WT1.dll`；
3. 离线闭环：`tests/integration`（pytest，11/14 通过；详见 OFFLINE_VALIDATION.md），
   覆盖偏航方向/latch/幂等/TTL、诱导 seq/fallback、双通道独立、Provider 校核、通信故障；
4. ZeroMQ 冒烟：`wfcrl/engine/fastfarm_zmq.py` 提供 FarmController 连接模板；
5. Modbus 冒烟：`transport/modbus`（寄存器映射冻结于 modbus_map.yaml，Python 侧已验写/读/poll）。

## 3. 与 RT（实时）工程的接口点（交付给 RI/实时团队的契约）

| 接口 | 位置 | 说明 |
|---|---|---|
| Provider push API | include/fcr_provider_api.h | RT 调度器按 ICD 每 10 ms push；tower_base 由 ElastoDyn 提供 |
| RT Provider ICD | docs/FASTFARM_RT_PROVIDER_ICD.md | 逐标量来源/单位/valid 语义 + 验收测试向量 |
| ROSCO API | include/fcr_rosco_api.h、rosco/*.f90 | 10 ms setpoint/read 契约（多机 n_turbines） |
| 传输 | transport/zmq、transport/modbus | ZeroMQ（默认）/ Modbus TCP（备选） |
| 时钟 | fcr_publish_rt_clock | RT 侧统一挂钟推进，禁止各机累计 |

## 4. 红线（交付约束，与 PROTOCOL_DECISIONS.md 一致）

1. ROSCO 10 ms 路径无 socket/文件 I/O（传输在独立线程/1 s 路径）；
2. 闭环状态只来自 Provider push，禁止解析 .out/.outb；
3. AWAE 1 s 数据不得冒充 10 ms 数据（聚合器按 valid/source_time 判别）；
4. 角度/单位/方向换算只在 angle_convention.c；
5. yaw 与 induction 通道独立（各自 seq/TTL/生效语义）。

## 5. 版本

- delivery 版本：1.0.0-phase11（对应 git tag 建议 `phase11-rt-delivery`）；
- ABI 冻结：fcr_state_types.h（test_abi.c 校验）；协议冻结：fcr_protocol.h。
