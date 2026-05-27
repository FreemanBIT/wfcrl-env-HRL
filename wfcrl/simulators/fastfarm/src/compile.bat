@echo off
REM =============================================================================
REM 编译 DISCON_bridge.f90 → DISCON_WT1.dll (简单 NREL 5MW 文件 I/O 桥)
REM 输出: ..\servo_dll\DISCON_WT1.dll
REM =============================================================================
cd /d "%~dp0"
echo Compiling DISCON_bridge.f90 ...
gfortran -shared -static -o ..\servo_dll\DISCON_WT1.dll DISCON_bridge.f90
if %ERRORLEVEL% == 0 (
    echo [OK] DISCON_WT1.dll compiled successfully
) else (
    echo [FAIL] Compilation failed. Check gfortran installation.
)
pause
