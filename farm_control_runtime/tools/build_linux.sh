#!/usr/bin/env bash
# build_linux.sh — Linux 构建与单元测试（Phase 1）
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

if command -v cmake >/dev/null 2>&1; then
    cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
    cmake --build build
    ./build/fcr_unit_tests
else
    gcc -std=c11 -Wall -Wextra -O2 -Iinclude -Isrc -Itests/unit \
        -o tests/unit/test_runner \
        src/angle_convention.c src/command_store.c src/state_store.c \
        src/watchdog.c src/runtime.c \
        tests/unit/test_framework.c tests/unit/test_main.c \
        tests/unit/test_angle_convention.c tests/unit/test_command_store.c \
        tests/unit/test_abi.c tests/unit/test_watchdog.c -lm
    ./tests/unit/test_runner
fi
