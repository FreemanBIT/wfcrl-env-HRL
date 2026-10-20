# build_windows.ps1 — Windows 构建与单元测试（Phase 1）
# 用法: powershell -ExecutionPolicy Bypass -File tools/build_windows.ps1
# 需要: gcc（MinGW/TDM-GCC）或 MSVC 与 cmake；若 cmake 不可用，回退到 gcc 直接编译。

$ErrorActionPreference = "Stop"
$root = Split-Path -Parent $PSScriptRoot
$unitDir = Join-Path (Join-Path $root "tests") "unit"
$runner = Join-Path $unitDir "test_runner.exe"

Write-Host "== farm_control_runtime build (Windows) =="

# 优先 cmake
$cmake = Get-Command cmake -ErrorAction SilentlyContinue
if ($cmake) {
    Write-Host "[cmake] building..."
    Push-Location $root
    cmake -S . -B build -G "MinGW Makefiles" -DCMAKE_BUILD_TYPE=Release
    cmake --build build --config Release
    Pop-Location
    Write-Host "[cmake] running unit tests..."
    & (Join-Path (Join-Path $root "build") "fcr_unit_tests.exe")
    exit $LASTEXITCODE
}

# 回退：gcc 直接编译
$gcc = Get-Command gcc -ErrorAction SilentlyContinue
if (-not $gcc) { Write-Error "gcc not found"; exit 1 }
Write-Host "[gcc] building..."
Push-Location $root
gcc -std=c11 -Wall -Wextra -O2 -Iinclude -Isrc -Ibridge -Iawae -Ioffline/harness -Ioffline/provider -Itransport/modbus -Itests/unit -o $runner `
    src/angle_convention.c src/command_store.c src/state_store.c src/watchdog.c src/runtime.c `
    src/rosco_api.c src/signal_statistics.c src/state_aggregator.c src/induction_supervisor.c src/yaw_action_manager.c `
    bridge/fcr_bridge_core.c bridge/yaw_adapter.c bridge/rt_snapshot.c src/bridge_runtime_adapter.c `
    awae/observation_manager.c awae/wind_mapper.c awae/awae_frame_builder.c `
    offline/harness/shm_command_source.c offline/provider/offline_fastfarm_provider.c `
    transport/modbus/modbus_transport.c `
    tests/unit/test_framework.c tests/unit/test_main.c tests/unit/test_angle_convention.c `
    tests/unit/test_command_store.c tests/unit/test_abi.c tests/unit/test_watchdog.c `
    tests/unit/test_rosco_api.c tests/unit/test_yaw_action_manager.c tests/unit/test_offline_provider.c `
    tests/unit/test_signal_statistics.c tests/unit/test_state_aggregator.c tests/unit/test_induction_supervisor.c `
    tests/unit/test_modbus.c tests/unit/test_bridge_core.c tests/unit/test_bridge_adapter.c tests/unit/test_awae.c tests/unit/test_rt_snapshot.c -lws2_32
if ($LASTEXITCODE -ne 0) { Pop-Location; exit 1 }
Write-Host "[gcc] running unit tests..."
& $runner
$rc = $LASTEXITCODE
Pop-Location
exit $rc