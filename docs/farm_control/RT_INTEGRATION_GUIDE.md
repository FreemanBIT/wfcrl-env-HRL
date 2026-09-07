# RT_INTEGRATION_GUIDE — FAST.Farm RT（实时）集成指南

本文档面向 FAST.Farm RT 负责人：如何把本仓库交付的场控架构集成到实时平台上。
配套：docs/farm_control/FASTFARM_RT_PROVIDER_ICD.md（接口 ICD）、
PROTOCOL_DECISIONS.md（红线与约定）、REALTIME_DELIVERY.md（交付包说明）。

## 1. 架构承接关系（离线 → RT）

```text
                     离线已实现（本仓库，可复现）            RT 目标形态
runtime 载体      DISCON_WT1.dll 内嵌（探针）      farm_control_runtime 独立链接进
                                                  FAST.Farm 调度器侧（FAST_Farm.F90）
状态采集          Offline Provider（avrSWAP 探针）  RT Provider（调度器内直接取量）
单机控制          ROSCO（DLL 内 FCR 钩子）          ROSCO（不变，每 10 ms 读 runtime setpoint）
传输              ZeroMQ / Modbus                 同（transport/ 下同一套代码）
```

## 2. RT 负责人集成步骤（6 步）

### 2.1 引入 runtime 模块

把 `farm_control_runtime/`（src + include + transport）加入 FAST.Farm 工程：

```cmake
# FAST.Farm CMakeLists.txt 追加
add_subdirectory(farm_control_runtime)   # 或直接编译 src/*.c + transport/*
target_link_libraries(fastfarm PUBLIC farm_control_runtime ws2_32)  # Windows
```

### 2.2 在调度器侧挂接 RT Provider（替代 Offline 探针）

```fortran
! FAST_FarmSubstep / 每机 10 ms 末：
!   1) 采集本机标量（元素来源见 ICD 表 2.1）
!   2) 调用 Provider push API（与离线版同名、同语义）：
use fcr_provider_api   ! bind(C) 接口（farm_control_runtime/includes）
! RT 实现建议直接用 C 链接在调度器内（Fortran 包装可选）；
!   离线实现 fcr_offline_provider_publish_fast(tid, t, avrSWAP) 的 RT 等价：
call fcr_provider_publish_fast_fc(tid, t, rt_state_struct)   ! 填 FcrTurbineExtraState
! 每 DT_Low 步：
call fcr_provider_publish_flow_fc(tid, t, rt_flow_struct)    ! 轮盘/湍流数据
! tower_base/Ct/Cp 由 ElastoDyn/AD 工程量补齐（离线 DLL 无此量，valid=0）
```

### 2.3 挂接 ROSCO（单机控制器不变）

ROSCO 每 10 ms 主循环已包含 FCR 钩子（FCR_Step/FCR_PublishState），
RT 下保持与离线相同调用即可；唯一区别是 runtime 实例在调度器进程，
ROSCO 通过导出的 C 函数读写（符号导出方式与离线 DLL 内嵌时一致）。

### 2.4 配置传输

- ZeroMQ（默认）：新建/复用 `transport/zmq/zmq_transport.cpp`，
  环境变量 `FCR_ZMQ_CMD_PORT/FCR_ZMQ_STATE_PORT/FCR_LIBZMQ_PATH`；
- Modbus（备选）：`transport/modbus/modbus_transport.cpp`，寄存器映射见 modbus_map.yaml；
- RCP 侧 Python 客户端 `wfcrl/transport/farm_zmq_client.py` 可直接复用。

### 2.5 多机进程模型

- 离线每台机独立进程（FAST.Farm 现有结构）；RT 若为单进程多机，
  runtime 的 n_turbines=64 上限内自动循环；Provider 按 turbine_id 索引即可（无需改动）；
- 命令帧按槽位广播（所有机收到全场帧，各自取自己槽）——该语义与进程数无关。

### 2.6 验收（ICD §4 测试向量）

1. 空载 60 s：三机 fast_step/source_time 单调；各机时钟差 <0.5 s；
2. RCP 发 yaw seq → 2 s 内生效，目标差 ≤0.5°；
3. RCP 发 induction a=0.2 → 2 s 内生效，稳态降载 ∈ [10%, 17%]；
4. 拔网 → 命令 TTL 回退、仿真不中断；
5. tower_base 已知弯矩注入 → Provider 数值一致。

## 3. 责任人清单（checklist）

- [ ] farm_control_runtime 编译进 FAST.Farm 工程（含 ws2_32/ZMQ 链接）
- [ ] RT Provider 完成（publish_fast/publish_flow/rt_clock，tower_base 从 ElastoDyn 取）
- [ ] ROSCO FCR 钩子挂接验证（FCR_Step/read_setpoint/publish_state）
- [ ] ZeroMQ 双端口联通（客户端连入、状态帧流出）
- [ ] ICD §4 五项验收通过
- [ ] 机舱方位/载荷单位一致性复核（angle_convention/单位表）

## 4. 需要 RT 侧注意的已知边界

- 状态帧字段集为冻结 ABI（360 B/机）；如需新增载荷部位字段 → 走 ICD 修订
  （版本号 + test_abi.c 同步，见问题 2 扩展方案）；
- avrSWAP 探针记录（109/110/77/78/27）为离线路径；RT 直接取工程量，无此限制；
- ZeroMQ 传输位库：RT 环境建议使用 release 构建的 libzmq（离线工具链为 debug 断言版）。
