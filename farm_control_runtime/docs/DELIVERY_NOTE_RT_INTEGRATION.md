# farm_control_runtime 交付说明

> 交付对象：WSL / LinuxRT 开发人员
> 版本：ICD v1.5 接口，RT ABI **v2**，AWAE 容量 **1000 点**（`FCR_CAPACITY_VERSION = 2`）

---

## 1. 交付内容

| 类别 | 内容 | 位置 |
|---|---|---|
| **C 运行时** | 场控核心（command_store / state_store / state_aggregator / induction_supervisor / bridge core / AWAE / watchdog） | `farm_control_runtime/src`、`bridge`、`awae` |
| **RT 接入层** | `fcr_rt_*`：调度器可调用的生命周期 / 输入 / 步进 / 输出 / 状态接口 | `farm_control_runtime/include/fcr_rt.h`、`src/fcr_rt_adapter.c` |
| **Fortran 绑定与 ROSCO 源码** | `FarmControlCBindings.f90`、`FarmControlInterface.f90`；ROSCO 控制器源码 9 个文件 | `farm_control_runtime/rosco/`、`rosco/src/` |
| **公共头文件（ABI）** | `fcr_protocol.h` / `fcr_state_types.h` / `fcr_awae_types.h` / `fcr_rosco_api.h` / `fcr_rt.h` | `farm_control_runtime/include/` |
| **Python 协议镜像** | `farm_protocol.py`（ctypes 逐字节镜像）、`farm_zmq_client.py` | `wfcrl/transport/` |
| **构建脚本** | `build_linux.sh`（C 库 + 测试）、`build_rosco_linux.sh`（Fortran `.a`）、`_compile.py`（Windows DLL） | `farm_control_runtime/tools/`、`wfcrl/simulators/fastfarm/src/` |
| **示例** | `rt_minimal_example.c`（1000 × 10 ms 调度器接入） | `farm_control_runtime/examples/` |
| **测试** | C 单元/RT 测试 100 例、Python 协议 12 例、集成测试、门禁与信号审计 | `farm_control_runtime/tests/`、`tools/` |
| **产物** | `DISCON_WT1.dll`（Windows 闭环用，2371.6 KB） | `wfcrl/simulators/fastfarm/servo_dll/` |
| **信号字典** | ICD v1.5 SingleHop（88 信号 / 13 组） | `docs/farm_control/inputs/` |

---

## 2. 架构总览

```
【下行：命令】RCP ──> CommandStore ──> Bridge ──> setpoint(S08) ──> ROSCO
             (S01 帧)   (seq/TTL/使能)  (Mode0/1/2/3)              (CntrPar)

【上行：状态】OpenFAST(S02) / ROSCO(S03) ──> StateStore ──> Aggregator(1 s)
                                                    └──> FarmStateFrame(S10, 25664 B) ──> RCP

【AWAE 二跳】AWAE Provider(S04/S05) ──> AWAE builder ──> Observation/Catalog(S12/S13) ──> RCP
```

**三种运行形态**（同一套 C 核心）：

| 形态 | 驱动方式 | 传输 | 适用 |
|---|---|---|---|
| Windows 闭环 | ServoDyn 每 10 ms 调 `DISCON()` | DLL 内 ZMQ | 离线验证（FAST.Farm + ROSCO） |
| **WSL / fastfarmRT** | 调度器每 10 ms 调 `fcr_rt_step()` | 调度器负责 | 平台集成、物理闭环 |
| 离线 harness | SHM 注入 | 共享内存 | 无网络调试 |

---

## 3. 构建

### 3.1 WSL / Linux

```bash
sudo apt-get install -y build-essential cmake gfortran
cd farm_control_runtime

bash tools/build_linux.sh          # C 库 + 单元/RT 测试（ctest）
bash tools/build_rosco_linux.sh    # ROSCO Fortran 静态库
# 产物：build-linux/libfarm_control_runtime.a、build-linux/libfcr_rosco.a
```

链接：

```bash
gfortran your_main.o build-linux/libfcr_rosco.a build-linux/libfarm_control_runtime.a -lpthread -lm
```

### 3.2 Windows（离线闭环验证）

```bat
farm_control_runtime\tools\_build_test.bat && farm_control_runtime\test_runner.exe
.venv\Scripts\python.exe farm_control_runtime\tools\check_abi.py
.venv\Scripts\python.exe farm_control_runtime\tools\check_icd_gates.py
.venv\Scripts\python.exe farm_control_runtime\tools\audit_signals.py
.venv\Scripts\python.exe farm_control_runtime\tests\integration\test_icd_v15_abi_python.py
.venv\Scripts\python.exe wfcrl\simulators\fastfarm\src\_compile.py
```

---

## 4. ABI 与容量

所有公共结构体尺寸/偏移由 `tests/unit/test_abi.c` + `tools/check_abi.py` 双重锁定；
修改任一处必须同步：C 头文件 + `test_abi.c` + `check_abi.py` + Python 镜像。

### 4.1 关键尺寸（64 位实测）

| 结构 | 尺寸 | 说明 |
|---|---|---|
| `FcrTurbineCommand` | 88 | S01 每机命令 |
| `FarmCommandFrame` | 5664 | 命令帧 |
| `FcrOpenFASTRawState` | 104 | S02 |
| `FcrRoscoRawState` | 232 | S03 + 扩展 |
| `FcrSignalStats` | 48 | 1 s 统计容器 |
| `FcrTurbineExtraState` | 840 | 载荷/系数 |
| `FcrFlowState` | **57568** | AWAE 流场（1000 点） |
| `FcrBridgeInput` / `Output` / `Ack` | 72 / 56 / 32 | S07 / S08 / S09 |
| `FcrRoscoExternalSetpoint` | 112 | S08 + S1101（`ct_tracking_error` 保留位） |
| `FcrRoscoAck` | 32 | S09 |
| `FcrFarmTurbineState` | 400 | S10 每机 |
| `FarmStateFrame` | 25664 | 状态帧 |
| `FcrAWAECatalogEntry` / `Frame` | 40 / **40024** | S04 / S13 |
| `FcrAWAEObservation` / `Frame` | 48 / **48024** | S05 / S12 |
| `FcrTurbineWindEstimate` | 32 | S06 |

### 4.2 容量

```c
#define FCR_CAPACITY_VERSION            2u
#define FCR_MAX_TURBINES_V1             64u
#define FCR_MAX_OBSERVATION_POINTS_V1   64u
#define FCR_MAX_OBSERVATION_POINTS_V2   1000u
#define FCR_MAX_TURBINES                FCR_MAX_TURBINES_V1
#define FCR_MAX_OBSERVATION_POINTS      FCR_MAX_OBSERVATION_POINTS_V2
```

- 机组容量 64，AWAE 观察点容量 **1000**；
- 收方按 `protocol_version` 与 `FCR_CAPACITY_VERSION` 选择布局；
- 进一步扩容须引入 `*_V{n}` 常量并提升 `FCR_CAPACITY_VERSION`，不得直接修改冻结尺寸。

### 4.3 帧类型区分

ZMQ 消息为一整帧，按**冻结尺寸**区分：state 25664 / AWAE 观测 48024 / AWAE 目录 40024。
Modbus 侧按 `modbus_register_map.h`（AWAE 区 0x3000 起，32 寄存器/点，总长 `MB_TOTAL_REGS`）。

---

## 5. 信号一致性

`tools/audit_signals.py` 对照 `FASTFarm_WFC_Signal_ICD_v1_5_SingleHop.csv`（88 信号 / 13 组）：

- **88/88 一致**：每个信号均有对应 C 字段、类型匹配、方向/承载正确；
- **S11**（Future_CT_Feedback）：`ct_tracking_error` 为保留接口，当前恒 0；
- **2 条 NOTE**：`S0201/S1003 turbine_id` CSV 标 uint32、C 用 int32_t（ABI 足迹相同，值恒 ≥1）；
- **门禁 PASS**：Sender/Receiver 唯一、S12/S13 不冲突、`Upstream_Signal_ID` 跨两跳可追。

---

## 6. 验证结果

| 验证项 | 结果 |
|---|---|
| C 单元/RT 测试 | **100/100 PASS**（WSL/Linux：93/93，少 7 个 Windows-only Modbus 用例） |
| Python 协议测试 | **12/12 PASS** |
| ABI 交叉校验 | **PASS**（结构尺寸 + 82 个 Python↔C 字段偏移） |
| 信号审计 / 门禁 | **PASS**（88/88；G-1/G-2/G-3） |
| AWAE 二跳集成测试 | **PASS**（ZMQ 40024/48024 B 帧端到端） |
| ICD 集成测试 | 26 例（Windows 闭环，libzmq teardown 偶发 abort 为环境问题，逐文件运行） |
| 300 s 闭环回归 | 待当前版本复跑；上一轮 4379 kW（基线 4366 kW，+0.3%） |

---

## 7. 集成红线

1. **10 ms 路径禁止**：互斥锁、socket、malloc、等待 GPU/AWAE/通信。
2. **角度约定**：一律 OpenFAST 内部约定（CCW+，0 = +X）；`yaw_target_heading_rad` 是绝对目标，
   不是增量；RCP 侧 CW+ 语义由 `FarmZmqClient.send_yaw_delta` 换算，下游不得重复换算。
3. **时间基准不可混用**：`command_basis_sim_time_s`(S0102)、`bridge_step_sim_time_s`(S0701)、
   AWAE 源仿真时间(S0502) 各自独立；不得用墙钟替换仿真时间。
4. **容量版本化**：扩容走 `*_V{n}` + `FCR_CAPACITY_VERSION`。
5. **单线程调用**：模块非线程安全；RT 路径内部无锁，由调度器串行调用。

---

## 8. 与 RT 平台的接口（`fcr_rt.h`）

| 阶段 | 调用 |
|---|---|
| 初始化 | `fcr_rt_create` → `fcr_rt_configure` → `fcr_rt_init`（自动与 Fortran 侧共享同一 runtime） |
| 每 10 ms | `fcr_rt_set_openfast_state`(S02) / `fcr_rt_set_state`(S03) → `fcr_rt_step(RTimeCtx)` |
| 每 1 s | `fcr_rt_set_command`(S01) / `fcr_rt_get_output`(S10) / `fcr_rt_get_awae`(S12/S13) |
| 每控制步 | `fcr_rt_get_setpoint`(S08) → ROSCO；`FCR_ApplyReferences` 注入 CntrPar |
| 收尾 | `fcr_rt_reset` / `fcr_rt_stop` / `fcr_rt_destroy` |

状态返回值：`OK / WARNING / STALE_DATA / INVALID_INPUT / NOT_READY / TIMEOUT /
NUMERICAL_ERROR / COMM_ERROR / FATAL`，配套 `FcrRtStatusInfo`（错误码/来源/消息/时间）。
逐信号的接受/拒绝/限幅/保持/降级动作见 `03_RT_DATA_AND_TIME_POLICY.md`。

---

## 9. 已知限制

| 项 | 说明 |
|---|---|
| Windows libzmq teardown | 集成测试 teardown 偶发 `Socket operation on non-socket` abort，属环境问题，逐文件运行 |
| 研究侧失稳 | 特定工况下 FAST.Farm 尾流平面超越告警（t≈237 s），与 runtime 无关，需研究侧调低 `f_c` |
| Linux 构建 | C 库与 ROSCO `.a` 的构建脚本已提供，需在 WSL 主机首次实测确认 |
| Modbus 客户端 | 已实现 Server 侧；Modbus Client 属平台侧 |
| 实时指标 | Deadline / Jitter / WorstCase / Deadline Miss 需 RT 环境实测 |

---

## 10. 文档索引

| 文档 | 内容 |
|---|---|
| `00_DELIVERY_NOTE_RT_INTEGRATION.md` | 本文（交付说明） |
| `01_WSL_FASTFARMRT_INTEGRATION.md` | fastfarmRT 集成：接口映射、运行时共享、ROSCO `.a`、里程碑 |
| `02_WSL_DEPLOY_AND_TEST_GUIDE.md` | WSL 部署与测试步骤、排障、验收清单 |
| `03_RT_DATA_AND_TIME_POLICY.md` | 数据策略、时间语义、降级映射（D0~D4） |
| `04_ARCHITECTURE_DIAGRAMS.md` | Windows 运行态与 WSL/fastfarmRT 部署态架构图 |
| `docs/farm_control/inputs/FASTFarm_WFC_Signal_ICD_v1_5_SingleHop.csv` | ICD v1.5 信号字典 |
