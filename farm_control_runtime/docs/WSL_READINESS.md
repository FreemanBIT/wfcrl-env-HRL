# WSL 交付就绪性检查（farm_control_runtime 场控交互模块）

> 检查对象：`dev/farm-control-runtime` 当前版本（含 RT 接入层 `fcr_rt_adapter`）
> 检查目的：能否交付给开发人员在 **WSL** 环境下做模块级/调度器级测试
> 检查日期：2026-10-22
> 检查方式：源码级可移植性审计（本机 **未安装 WSL**，Linux 构建未实测——见 §5）

---

## 0. 结论

**修复后可交付 WSL 测试（模块级 + 调度器级），但不包含 FAST.Farm 闭环。**

| 能力 | WSL 可用 | 说明 |
|---|---|---|
| 场控核心（命令/状态/桥接/AWAE/RT 适配层） | ✅ 可构建 | 纯 C11，无平台依赖 |
| 单元/RT 测试（91 例） | ✅ 可运行 | Modbus 7 例为 Windows-only |
| `fcr_rt_*` 调度器接入 + 自写 Mock Scheduler | ✅ | 见 `include/fcr_rt.h` |
| Python 协议镜像（`farm_protocol.py`） | ✅ | 纯 ctypes，Linux x86_64 布局一致 |
| ZMQ / Modbus 传输 | ❌ | Windows-only（RT 平台由调度器负责传输） |
| `DISCON_WT1.dll` / FAST.Farm 闭环 | ❌ | Windows PE + ROSCO Fortran DLL（属 M4） |

**交付前必须由开发人员在 WSL 主机执行一次 `bash tools/build_linux.sh`，确认 91/91 通过**（本机无 WSL，无法代跑）。

---

## 1. 审计发现的阻塞项与处理

| # | 阻塞项 | 证据 | 处理 |
|---|---|---|---|
| B1 | `transport/modbus/modbus_transport.c` 依赖 Winsock | `#include <winsock2.h>`、`_beginthreadex` | CMake 在非 WIN32 下**排除**；对应 `test_modbus.c` 排除并定义 `MB_STUB` |
| B2 | `offline/harness/shm_command_source.c` 依赖 Windows SHM/线程 | `#include <windows.h>`、`CreateThread` | 非 WIN32 下替换为 `shm_command_source_stub.c`（no-op；RT 由调度器传输） |
| B3 | `transport/zmq/zmq_transport.cpp` 依赖 `LoadLibraryA` | `<windows.h>`、`LoadLibraryA/GetProcAddress` | 原本即 `if(WIN32)`；另在 `rosco_api.c` 增加弱符号 NULL 守卫，避免未链接时空指针调用 |
| B4 | CMake 单测目标**漏列** `test_rt_adapter.c`（而 `test_main.c` 已引用其注册函数） | CMakeLists 单测列表 | 已补入；Linux 下 91 例 |
| B5 | 非 Windows 未链接 pthread | `rosco_api.c` 的 `#else #include <pthread.h>` | CMake 在非 WIN32 下 `find_package(Threads)` 并链接 |
| B6 | 无 Linux 构建入口 | 只有 `_build_test.bat` / `.ps1` / `_compile.py`（硬编码 TDM-GCC 与 DLL） | 新增/更新 `tools/build_linux.sh`（cmake 优先，gcc 兜底） |
| B7 | `bridge/rt_snapshot.h` 对齐宏 | `__declspec(align(64))` | 审计确认已有 `#if _MSC_VER / #else __attribute__((aligned(64)))` 分支，**无需改** |

> 另：`src/fcr_rt_adapter.c` 与 `src/rosco_api.c` 的平台分支（QPC ↔ CLOCK_MONOTONIC、CRITICAL_SECTION ↔ pthread）审计确认已具备。

---

## 2. WSL 快速开始

```bash
# 1) 依赖（Debian/Ubuntu）
sudo apt-get update && sudo apt-get install -y build-essential cmake

# 2) 构建 + 测试（在仓库根目录）
cd farm_control_runtime
bash tools/build_linux.sh            # cmake 构建 build-linux/ 并跑 ctest
# 或：bash tools/build_linux.sh --gcc   # 无 cmake 时的纯 gcc 兜底

# 3) 预期
#    ctest 通过；单元/RT 测试 91/91（Windows 为 98/98，差 7 个 Modbus 用例）
```

---

## 3. 开发人员在 WSL 侧的接入方式

1. **链接静态库**：`build-linux/libfarm_control_runtime.a` + `include/fcr_rt.h`；
2. **生命周期**：`fcr_rt_create → fcr_rt_configure → fcr_rt_init`；
3. **每 10 ms**：`fcr_rt_step(h, &RTimeCtx, &status)`；
4. **每 1 s**：`fcr_rt_set_command(...)` / `fcr_rt_get_output(...)`；
5. **传输**：由调度器负责（模块不持有 socket）；
6. **状态**：按 `FcrRtStatus` + `FcrRtStatusInfo` 执行 D0~D4（映射见 `RT_DATA_AND_TIME_POLICY.md`）。

参考实现：`tests/unit/test_rt_adapter.c::test_mock_scheduler_1000_cycles`（1000×10 ms 调度器模拟）。

---

## 4. WSL 侧可复现的验证清单

| 项 | 命令（在 `farm_control_runtime/`） | 预期 |
|---|---|---|
| 构建 | `bash tools/build_linux.sh` | 无错误 |
| 单测/RT | `ctest --test-dir build-linux --output-on-failure` | **91/91** |
| ABI（Python↔C） | `python3 tools/check_abi.py`（需 gcc） | PASS（82 字段偏移） |
| 信号门禁 | `python3 tools/check_icd_gates.py` | PASS（88 信号） |
| 信号审计 | `python3 tools/audit_signals.py` | PASS（88/88） |
| RT 接口版本 | 链接后调用 `fcr_rt_get_version()` | == `FCR_RT_ABI_VERSION` (1) |

---

## 5. 未实测项（必须由 WSL 主机确认）

| 项 | 原因 | 建议 |
|---|---|---|
| Linux 编译/链接 | 本机未安装 WSL，无法执行 | 开发人员首次 `bash tools/build_linux.sh` 验证 |
| 91 例测试结果 | 同上 | 以 ctest 输出为准；若有失败请回传日志 |
| Python 镜像在 Linux 的尺寸 | ctypes 布局理论上一致（LP64/固定宽度类型） | `python3 tools/check_abi.py` 会直接校验 |
| 调度器实机联调 | 平台调度器不在本仓库 | M3 联调（Mock/实际 Scheduler） |

---

## 6. 与 M4（采购后）的边界

以下**不属于**本次 WSL 交付范围，属采购后 M4：

- Linux FAST.Farm 二进制 + ROSCO `DISCON` 的 `.so` 构建（`_compile.py` 为 Windows 专用）；
- Linux 版 ZMQ/Modbus 传输实现（若平台调度器不接管传输）；
- RT vs WSL 结果一致性、Deadline/Jitter/WorstCase/Deadline Miss 实测。
