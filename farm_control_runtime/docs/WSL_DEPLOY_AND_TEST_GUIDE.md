# farm_control_runtime（场控交互模块）— WSL 部署与测试指南

> **读者**：下一阶段开发人员（WSL / Linux 环境）
> **目标**：把交付的场控模块部署到 WSL，跑通构建与测试，并按其接口接入调度器
> **版本**：RT ABI **v2**，AWAE 容量 **1000 点**
> **配套文档**：`00_DELIVERY_NOTE_RT_INTEGRATION.md`（交付说明）、`01_WSL_FASTFARMRT_INTEGRATION.md`（fastfarmRT 接入）、`03_RT_DATA_AND_TIME_POLICY.md`（数据/时间/降级策略）

---

## 1. 交付物与范围

### 1.1 目录结构（解压后）

```
farm_control_runtime_delivery_20261022/
├── 00_DELIVERY_NOTE_RT_INTEGRATION.md      交付说明（内容/架构/ABI/红线）
├── 01_WSL_FASTFARMRT_INTEGRATION.md        fastfarmRT 接入（映射/共享/ROSCO .a）
├── 02_WSL_DEPLOY_AND_TEST_GUIDE.md         ← 本文档
├── 03_RT_DATA_AND_TIME_POLICY.md           数据策略/时间语义/降级映射
├── 04_ARCHITECTURE_DIAGRAMS.md             架构图（Windows / WSL-fastfarmRT）
├── figures/                                架构图 PNG/SVG
├── docs/farm_control/inputs/
│   └── FASTFarm_WFC_Signal_ICD_v1_5_SingleHop.csv   ICD v1.5 信号字典（88 信号）
├── farm_control_runtime/                   ★ 交付主体（WSL 使用）
│   ├── include/                            对外头文件（fcr_rt.h / fcr_protocol.h / ...）
│   ├── src/                                核心实现（含 fcr_rt_adapter.c）
│   ├── bridge/                             Bridge 核心（诱导映射/控制分配/Ct）
│   ├── awae/                               AWAE 帧组装（S04/S05/S12/S13）
│   ├── offline/                            离线 Provider + Linux SHM stub
│   ├── transport/                          ZMQ / Modbus（Windows-only，见 §1.2）
│   ├── tests/unit/                         单元 + RT 测试（93 例可在 WSL 跑）
│   ├── tools/                              build_linux.sh / build_rosco_linux.sh / 三个校验脚本
│   ├── examples/rt_minimal_example.c       最小调度器接入示例（可直接编译）
│   ├── rosco/                              Fortran 绑定 + ROSCO 源码（rosco/src/）
│   ├── config/                             诱导映射等配置
│   └── CMakeLists.txt                      跨平台构建入口
└── wfcrl/
    ├── transport/farm_protocol.py          Python 侧 ABI 镜像（ctypes）
    ├── transport/farm_zmq_client.py        Python 侧 ZMQ 客户端（Windows 闭环用）
    └── simulators/fastfarm/servo_dll/DISCON_WT1.dll   Windows 预编译 DLL（WSL 不使用）
```

### 1.2 WSL 可用 / 不可用

| 内容 | WSL | 说明 |
|---|---|---|
| 场控核心（`src/`、`bridge/`、`awae/`、`offline/provider/`） | ✅ | 纯 C11 |
| RT 接入层 `src/fcr_rt_adapter.c` + `include/fcr_rt.h` | ✅ | POSIX `CLOCK_MONOTONIC` |
| 单元/RT 测试（`tests/unit/`） | ✅ **93 例** | Modbus 7 例为 Windows-only |
| Python ABI 镜像 `farm_protocol.py` | ✅ | 纯 ctypes |
| 工具 `check_abi.py` / `check_icd_gates.py` / `audit_signals.py` | ✅ | 需 `gcc`（check_abi） |
| `transport/modbus/` | ❌ | Winsock，CMake 自动排除 |
| `offline/harness/shm_command_source.c` | ❌ | Windows SHM；Linux 用 `shm_command_source_stub.c`（no-op） |
| `transport/zmq/zmq_transport.cpp` | ❌ | `LoadLibrary`；WSL 不需要（传输归调度器） |
| `DISCON_WT1.dll` | ❌ | Windows PE（WSL 不使用该 DLL） |
| **FAST.Farm 闭环** | ✅ **可做** | WSL 侧 **fastfarmRT** 提供 OF/WD/AWAE（CPU/GPU）；接入见 `WSL_FASTFARMRT_INTEGRATION.md` |

---

## 2. 环境准备（WSL）

```bash
# 以 Ubuntu/Debian 为例
sudo apt-get update
sudo apt-get install -y build-essential cmake python3

# 版本检查（最低要求：gcc >= 7，cmake >= 3.16，python3 >= 3.8）
gcc --version | head -1
cmake --version | head -1
python3 --version
```

> 可选：若要跑 Python 的 ABI 校验脚本，需 `numpy`（`pip3 install numpy`）；`check_icd_gates.py` / `audit_signals.py` 仅用标准库。

---

## 3. 部署

```bash
# 1) 解压交付包到工作目录
unzip farm_control_runtime_delivery_20261022.zip -d ~/fcr
cd ~/fcr/farm_control_runtime_delivery_20261022

# 2) 确认关键文件齐全（应全部输出 True）
for f in \
  farm_control_runtime/include/fcr_rt.h \
  farm_control_runtime/src/fcr_rt_adapter.c \
  farm_control_runtime/offline/harness/shm_command_source_stub.c \
  farm_control_runtime/tools/build_linux.sh \
  farm_control_runtime/examples/rt_minimal_example.c \
  farm_control_runtime/CMakeLists.txt \
  farm_control_runtime/tests/unit/test_rt_adapter.c ; do
    [ -f "$f" ] && echo "True  $f" || echo "MISS  $f"
done

# 3) 给脚本执行权限（可选，用 bash 调用则不需要）
chmod +x farm_control_runtime/tools/build_linux.sh
```

---

## 4. 构建

### 4.1 推荐：CMake

```bash
cd farm_control_runtime
bash tools/build_linux.sh            # 等价于下面的三步
# 或手动：
cmake -S . -B build-linux -DCMAKE_BUILD_TYPE=Release
cmake --build build-linux --parallel "$(nproc)"
```

产物：

| 产物 | 路径 | 用途 |
|---|---|---|
| 静态库 | `build-linux/libfarm_control_runtime.a` | 集成到调度器/平台 |
| 测试可执行 | `build-linux/fcr_unit_tests` | 跑单元 + RT 测试 |

### 4.2 无 cmake 时的 gcc 兜底

```bash
bash tools/build_linux.sh --gcc
# 产物：tests/unit/test_runner_linux
```

### 4.3 集成到你的工程（两种方式）

**方式 A（推荐）：链接静态库**

```cmake
add_subdirectory(path/to/farm_control_runtime)
target_link_libraries(your_target PRIVATE farm_control_runtime)
```

或直接用 gcc：

```bash
gcc -std=gnu11 -I path/to/farm_control_runtime/include \
    your_scheduler.c path/to/build-linux/libfarm_control_runtime.a \
    -lpthread -lm -o your_scheduler
```

**方式 B：源码直接编入**（清单见 §6.2 的示例编译命令）

> 注意：非 Windows 下必须使用 `offline/harness/shm_command_source_stub.c`
> 而不是 `shm_command_source.c`；`rosco_api.c` 需要 `-lpthread`。

---

## 5. 测试

### 5.1 自动化测试（必跑）

```bash
cd farm_control_runtime
bash tools/build_linux.sh
# 预期最后一行：100% tests passed, 0 tests failed out of 1
# 内部用例数：93/93（Windows 为 100/100，差 7 个 Windows-only Modbus 用例）
```

期望输出片段：

```
== ctest ==
    Start 1: unit
1/1 Test #1: unit .............................   Passed
100% tests passed, 0 tests failed out of 1
```

也可直接运行测试程序看逐条结果：

```bash
./build-linux/fcr_unit_tests | tail -5
# == farm_control_runtime unit tests done: 93/93 passed ==
```

### 5.2 静态一致性检查（建议跑）

```bash
cd farm_control_runtime
python3 tools/check_abi.py          # 期望：ABI CHECK: PASS（82 字段偏移）
python3 tools/check_icd_gates.py    # 期望：G-1/G-2/G-3 静态检查通过（88 信号 / 13 组）
python3 tools/audit_signals.py      # 期望：88/88 一致（2 条 turbine_id 符号性 NOTE）
```

> `check_abi.py` 会调用 `gcc` 编译 C 探针；若失败请确认 `gcc` 在 PATH。

### 5.3 RT 接入冒烟测试（强烈建议）

```bash
cd farm_control_runtime
gcc -std=gnu11 -O2 -Iinclude examples/rt_minimal_example.c \
    build-linux/libfarm_control_runtime.a -lpthread -lm -o /tmp/rt_minimal_example
/tmp/rt_minimal_example
```

期望输出（实测基准）：

```
fcr_rt ABI version = 2
FCR_RT_ABI_VERSION = 2
[output] t=1.00 s seq=1 n_turbines=2
...
[output] t=10.00 s seq=10 n_turbines=2
T1 setpoint: induction_enable=1 seq=10 r_pitch=0.8640 r_speed=0.9524
perf: steps=1000 last=... us max=... us avg=... us over_budget=0
summary: 1000 steps, 10 SCADA frames, 0 warnings
after reset: induction_enable=0 seq=0 (期望 0/0)
done
```

**判读要点**：1000 次 step → 10 次输出（每 100×10 ms = 1 s）；`r_pitch=0.8640` 是 a=0.2 的理论值；reset 后通道归零。

---

## 6. 接入调度器（接口速查）

### 6.1 调用序列（《要求》§2.2/§3/§5）

```
create → configure → init
  ↓
每 10 ms：  set_state(每机)  →  step(RTimeCtx)  →  返回值/StatusInfo 判 D0~D4
每 1 s ：  set_command(RCP 命令)   /   get_output(状态帧 → RCP)
  ↓
reset（可选，run→reset→run）→ stop → destroy
```

### 6.2 关键约定（已与平台方确认）

| 项 | 约定 |
|---|---|
| 调用周期 | **10 ms**（调度器周期） |
| 场控交换周期 | **DT_SCADA = 1 s**（RCP ↔ 调度器） |
| 时间来源 | 调度器唯一时间源；`FcrRtTimeCtx` 只读传入，模块不自推时间 |
| 传输归属 | **调度器负责**（模块不持有 socket） |
| 输出生效 | 下一个 DT_Low |
| 降级判定 | 模块自判并上报（D3 退出场控由模块决定） |
| 状态回传 | 同步返回 `FcrRtStatus` + `FcrRtStatusInfo` |

### 6.3 接口清单（`include/fcr_rt.h`）

| 函数 | 用途 |
|---|---|
| `fcr_rt_get_version()` | 握手：应与 `FCR_RT_ABI_VERSION`(1) 一致 |
| `fcr_rt_create / configure / init` | 生命周期前三步 |
| `fcr_rt_set_command` | 送入 RCP 命令帧（S01） |
| `fcr_rt_set_state / set_extra / set_flow / set_wind_estimate / set_awae_catalog` | 送入平台状态（S02/S03/S06/AWAE） |
| `fcr_rt_step(h, &RTimeCtx, &StatusInfo)` | 每 10 ms 调用一次 |
| `fcr_rt_get_output` | 取 S10 状态帧（1 s 边界后有新帧才返回 1） |
| `fcr_rt_get_setpoint` | 取 S08 setpoint（交单机控制/ROSCO） |
| `fcr_rt_get_awae` | 取 S12/S13 AWAE 二跳帧 |
| `fcr_rt_get_perf` | 执行时统计（last/max/avg/超预算次数） |
| `fcr_rt_reset / stop / destroy` | 复位/停止/销毁 |

### 6.4 状态与降级（速查）

| 返回值 | 含义 | 建议动作 |
|---|---|---|
| `FCR_RT_OK` | 正常 | 继续 |
| `FCR_RT_WARNING` | 可继续（如超执行时预算） | 记录 |
| `FCR_RT_STALE_DATA` | 数据超龄（保持最后有效值，D1） | 记录并观察 |
| `FCR_RT_TIMEOUT` | 命令 TTL 到期、已退出外部控制（D3，边沿触发一次） | 记录 |
| `FCR_RT_INVALID_INPUT` | 非法输入被拒绝（NaN/Inf/时间回退/版本不符） | 记录并检查上游 |
| `FCR_RT_NUMERICAL_ERROR` | 状态量 NaN/Inf（D4 建议） | 停止算例 |
| `FCR_RT_FATAL` | 内部不变量破坏（D4） | 停止算例并回传日志 |
| `FCR_RT_NOT_READY` | 未 init 或已 stop | 检查生命周期 |

> 逐信号动作表（接受/拒绝/限幅/保持/降级/停止）见 `03_RT_DATA_AND_TIME_POLICY.md` §1；
> 时间语义（TTL/最大保持）见 §2；D0~D4 映射见 §3。

---

## 7. 在 WSL 可以完成的验收（对应平台准入）

| 准入 | WSL 可做 | 做法 |
|---|---|---|
| **M1 Module Verified** | ✅ | §5.1 单测 93/93 + §5.3 RT 示例 |
| **M2 Interface Verified** | ✅ | §5.2 ABI/门禁/审计 + §6 接口约定 |
| **M3 Scheduler Verified** | ✅（需你方调度器或 Mock） | 按 §6 时序接入；参考 `tests/unit/test_rt_adapter.c::test_mock_scheduler_1000_cycles`；**有 fastfarmRT 时可做物理闭环**（见 `WSL_FASTFARMRT_INTEGRATION.md`） |
| **M4 RT Ready** | ❌（采购后） | RT 框架导入/一致性/Jitter/Deadline 实测 |

---

## 8. 常见问题

| 现象 | 原因 | 处理 |
|---|---|---|
| 链接错误 `undefined reference to fcr_offline_shm_init` | 编了 `shm_command_source.c`（Windows） | 改用 `shm_command_source_stub.c`，或直接用 CMake |
| 链接错误 `pthread_mutex_*` | 未链接 pthread | 加 `-lpthread`（CMake 已处理） |
| `nanosleep` 隐式声明错误 | 缺 `<time.h>` 或用了 `-std=c11`（严格 ANSI） | 用 `-std=gnu11`（CMake 默认 `gnu11`） |
| Modbus 相关编译失败 | `modbus_transport.h` 依赖 Winsock | Linux 不编 `transport/modbus/`；测试用 `-DMB_STUB` |
| `undefined reference to fcr_zmq_transport_hook` | 期望弱符号 | 不应发生（`rosco_api.c` 已做 NULL 守卫）；若报错说明符号被强引用，请用 CMake 构建 |
| `check_abi.py` 失败在编译探针 | `gcc` 不在 PATH | `sudo apt-get install build-essential` |
| Python 侧尺寸断言失败 | 用了非 64 位 Python | 使用 64 位 Python3（`python3 -c "import struct;print(struct.calcsize('P')*8)"` 应为 64） |
| 想跑 FAST.Farm 闭环 | 以为 WSL 没有 FAST.Farm | **用 fastfarmRT**（OF/WD/AWAE，CPU/GPU）；接入见 `WSL_FASTFARMRT_INTEGRATION.md`；`DISCON_WT1.dll` 本身仍不可用（Windows PE） |

---

## 9. 交付验收清单（请在 WSL 逐项打勾）

- [ ] §3 部署完整性检查全部 `True`
- [ ] `bash tools/build_linux.sh` 构建成功
- [ ] `ctest` 通过，测试程序报告 **93/93 passed**
- [ ] `python3 tools/check_abi.py` → `ABI CHECK: PASS`
- [ ] `python3 tools/check_icd_gates.py` → PASS（88 信号 / 13 组）
- [ ] `python3 tools/audit_signals.py` → PASS（88/88）
- [ ] `examples/rt_minimal_example.c` 编译并运行，输出 10 次 `[output]`、`r_pitch=0.8640`、reset 后 `0/0`
- [ ] 按 §6 接入（或 Mock）调度器，10 ms/1 s 周期与 D0~D4 行为符合预期
- [ ] 记录并回传：gcc/cmake 版本、93 例结果、任何告警

---

## 10. 范围边界

**不在 WSL 交付范围**（属采购后 M4 或平台侧）：

- 实时框架导入与 RT 指标实测（Deadline/Jitter/WorstCase/Miss）；
- Linux 版 ZMQ/Modbus 传输实现（若平台调度器不接管传输）；
- Modbus Client 侧实现（当前提供 Server 侧）。

> **FAST.Farm 物理闭环不在此列**：WSL 侧由 **fastfarmRT** 提供，
> 接入方式见 `01_WSL_FASTFARMRT_INTEGRATION.md`。
> `DISCON_WT1.dll` 本身仍为 Windows 产物，WSL 不使用它。

---

*对应交付包：`farm_control_runtime_delivery_20261022.zip`*
