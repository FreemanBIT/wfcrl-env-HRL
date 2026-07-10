@echo off
REM ====================================================================
REM run_all_turbsim.bat — 批量运行 turbsim_inp/ 下所有 .inp 生成 .bts
REM
REM 用法：
REM   1) 先生成 .inp：
REM        python -m induction_vs_yaw_study.turbsim_inp.generate_turbsim_inputs --out FarmInputs
REM   2) 修改下面的 TURBSIM_EXE 指向你的 TurbSim 可执行文件
REM   3) 双击本 bat，或在命令行运行
REM
REM 说明：36 个盒（3 风速 x 3 TI x 4 间距），大间距盒较宽(最大 1460m/164 点)，
REM       生成较慢，请预留时间。生成的 .bts 与 .inp 同目录同名。
REM ====================================================================

set TURBSIM_EXE=D:\HR_Project\wfcrl-env-HRL\FarmInputs\TurbSim_x64.exe
set INP_DIR=D:\HR_Project\wfcrl-env-HRL\FarmInputs

if not exist "%TURBSIM_EXE%" (
    echo [ERROR] 未找到 TurbSim 可执行文件：%TURBSIM_EXE%
    pause
    exit /b 1
)

cd /d "%INP_DIR%"

for %%f in (inflow_*_s*D.inp) do (
    echo ==== Running TurbSim: %%f ====
    "%TURBSIM_EXE%" %%f
    if errorlevel 1 (
        echo [WARN] TurbSim 返回错误：%%f
    )
)

echo.
echo 全部完成。请确认 %INP_DIR% 下已生成对应 .bts 文件。
pause
