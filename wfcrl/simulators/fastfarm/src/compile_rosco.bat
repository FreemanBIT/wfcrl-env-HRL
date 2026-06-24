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
"%GF%" -shared -static %FLAGS% -o "..\servo_dll\DISCON_WT1.dll" ^
  Constants.f90 ROSCO_Types.f90 SysGnuWin.f90 ^
  Filters.f90 Functions.f90 ControllerBlocks.f90 ^
  ROSCO_Helpers.f90 ReadSetParameters.f90 ROSCO_IO.f90 ^
  Controllers.f90 ExtControl.f90 ZeroMQInterface.f90 ^
  DISCON.F90

if %ERRORLEVEL% == 0 (
    echo [OK] DISCON_WT1.dll compiled successfully
    dir "..\servo_dll\DISCON_WT1.dll"
) else (
    echo [FAIL] Compilation failed at stage with code %ERRORLEVEL%
)

del *.o *.mod 2>nul
pause
