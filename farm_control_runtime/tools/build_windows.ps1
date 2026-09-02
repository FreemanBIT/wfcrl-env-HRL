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
gcc -std=c11 -Wall -Wextra -O2 -Iinclude -Isrc -Itests/unit -o $runner `
    src/angle_convention.c src/command_store.c src/state_store.c src/watchdog.c src/runtime.c `
    tests/unit/test_framework.c tests/unit/test_main.c tests/unit/test_angle_convention.c `
    tests/unit/test_command_store.c tests/unit/test_abi.c tests/unit/test_watchdog.c
if ($LASTEXITCODE -ne 0) { Pop-Location; exit 1 }
Write-Host "[gcc] running unit tests..."
& $runner
$rc = $LASTEXITCODE
Pop-Location
exit $rc