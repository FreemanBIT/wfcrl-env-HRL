@echo off
REM =============================================================================
REM 编译 ROSCO v2.9.0 + WFCRL Bridge → DISCON_WT1.dll
REM 适配 OpenFAST 5.0.0
REM 输出: ..\servo_dll\DISCON_WT1.dll
REM =============================================================================
cd /d "%~dp0"

set "GF=D:\TDM-GCC-64\bin\gfortran.exe"
set "FLAGS=-ffree-line-length-0 -static-libgcc -static-libgfortran -static -fdefault-real-8 -fdefault-double-8 -cpp -DIMPLICIT_DLLEXPORT -O2"

echo Compiling ROSCO + WFCRL Bridge...
echo Sources: %CD%
"%GF%" -c %FLAGS% SysGnuWin.f90
"%GF%" -c %FLAGS% ROSCO_Types.f90
"%GF%" -c %FLAGS% Constants.f90
"%GF%" -c %FLAGS% Functions.f90
"%GF%" -c %FLAGS% Filters.f90
"%GF%" -c %FLAGS% ControllerBlocks.f90
"%GF%" -c %FLAGS% ROSCO_Helpers.f90
"%GF%" -c %FLAGS% ROSCO_IO.f90
"%GF%" -c %FLAGS% ReadSetParameters.f90
"%GF%" -c %FLAGS% Controllers.f90
"%GF%" -c %FLAGS% ExtControl.f90
"%GF%" -c %FLAGS% ZeroMQInterface.f90
"%GF%" -c %FLAGS% DISCON.F90
"%GF%" -shared -static -o "..\servo_dll\DISCON_WT1.dll" *.o %FLAGS%

if %ERRORLEVEL% == 0 (
    echo [OK] DISCON_WT1.dll compiled successfully
    dir "..\servo_dll\DISCON_WT1.dll"
) else (
    echo [FAIL] Compilation failed at stage with code %ERRORLEVEL%
)

del *.o *.mod 2>nul
pause
