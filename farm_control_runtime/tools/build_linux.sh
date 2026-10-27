#!/usr/bin/env bash
# build_linux.sh — WSL/Linux 构建与单元/RT 测试
#
# 依赖（Debian/Ubuntu）:
#   sudo apt-get install -y build-essential cmake
#
# 用法:
#   ./tools/build_linux.sh            # cmake 构建 + ctest
#   ./tools/build_linux.sh --clean    # 先清理 build-linux/
#   ./tools/build_linux.sh --gcc      # 无 cmake 时的纯 gcc 兜底
#
# 说明（Linux 与 Windows 的差异）:
#   - Windows-only 模块由 CMake 自动排除（本脚本的 gcc 兜底同样排除）:
#       transport/modbus/modbus_transport.c   （Winsock）
#       offline/harness/shm_command_source.c  （Windows 共享内存 + CreateThread）
#       transport/zmq/zmq_transport.cpp       （LoadLibrary）
#     以 offline/harness/shm_command_source_stub.c（no-op）替代；
#     RT 平台由调度器负责传输（见 include/fcr_rt.h），故 no-op 是正确语义。
#   - src/fcr_rt_adapter.c 在 Linux 使用 clock_gettime(CLOCK_MONOTONIC)，
#     src/rosco_api.c 使用 pthread_mutex。
#   - Modbus 单测为 Windows-only（差 7 个用例）；Linux 下以 -DMB_STUB 跳过注册。
#   - DISCON_WT1.dll 为 Windows 产物，本脚本不构建；Linux FAST.Farm 集成属 M4。
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "${HERE}/.." && pwd)"
cd "${ROOT}"

MODE="cmake"
for arg in "$@"; do
    case "${arg}" in
        --clean) rm -rf build-linux ;;
        --gcc)   MODE="gcc" ;;
        -h|--help) sed -n '2,20p' "${BASH_SOURCE[0]}"; exit 0 ;;
    esac
done

# ---- 可移植源文件（Linux 全量）----
SRCS="src/angle_convention.c src/command_store.c src/state_store.c src/watchdog.c \
src/runtime.c src/rosco_api.c src/signal_statistics.c src/state_aggregator.c \
src/induction_supervisor.c src/yaw_action_manager.c src/bridge_runtime_adapter.c \
src/fcr_rt_adapter.c \
bridge/fcr_bridge_core.c bridge/yaw_adapter.c bridge/rt_snapshot.c \
awae/observation_manager.c awae/wind_mapper.c awae/awae_frame_builder.c \
offline/provider/offline_fastfarm_provider.c \
offline/harness/shm_command_source_stub.c"

# ---- 单测（排除 Windows-only 的 test_modbus.c）----
TESTS="tests/unit/test_framework.c tests/unit/test_main.c \
tests/unit/test_angle_convention.c tests/unit/test_command_store.c \
tests/unit/test_abi.c tests/unit/test_watchdog.c tests/unit/test_rosco_api.c \
tests/unit/test_yaw_action_manager.c tests/unit/test_offline_provider.c \
tests/unit/test_signal_statistics.c tests/unit/test_state_aggregator.c \
tests/unit/test_induction_supervisor.c tests/unit/test_bridge_core.c \
tests/unit/test_bridge_adapter.c tests/unit/test_awae.c \
tests/unit/test_rt_snapshot.c tests/unit/test_rt_adapter.c"

if [[ "${MODE}" == "cmake" ]] && command -v cmake >/dev/null 2>&1; then
    echo "== cmake configure/build =="
    cmake -S . -B build-linux -DCMAKE_BUILD_TYPE=Release
    cmake --build build-linux --parallel "$(nproc 2>/dev/null || echo 4)"
    echo "== ctest =="
    ctest --test-dir build-linux --output-on-failure
    echo "== done (cmake) =="
    exit 0
fi

echo "== gcc fallback =="
gcc -std=gnu11 -Wall -Wextra -O2 -DMB_STUB \
    -Iinclude -Isrc -Ibridge -Iawae -Ioffline/harness -Ioffline/provider \
    -Itransport/modbus -Itests/unit \
    ${SRCS} ${TESTS} \
    -o tests/unit/test_runner_linux -lpthread -lm
./tests/unit/test_runner_linux
echo "== done (gcc) =="
