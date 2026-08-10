@echo off
setlocal EnableExtensions

if /I "%~1"=="/?" goto :usage
if /I "%~1"=="-h" goto :usage

set "PROJECT_ROOT=%~dp0.."
for %%I in ("%PROJECT_ROOT%") do set "PROJECT_ROOT=%%~fI"
cd /d "%PROJECT_ROOT%"
if errorlevel 1 (
    echo ERROR: Could not change to project directory: %PROJECT_ROOT%
    exit /b 1
)
set "CONFIG=LineFollowMissionDebug"
set "PORT=%~1"
if "%PORT%"=="" set "PORT=COM13"

set "BUNDLE_ROOT=C:\Users\chenha\AppData\Local\stm32cube\bundles"
set "CMAKE=%BUNDLE_ROOT%\cmake\4.0.1+st.3\bin\cmake.exe"
set "ARM_BIN=%BUNDLE_ROOT%\gnu-tools-for-stm32\13.3.1+st.9\bin"
set "ARM_GCC=%ARM_BIN%\arm-none-eabi-gcc.exe"
set "PROGRAMMER=%BUNDLE_ROOT%\programmer\2.23.0\bin\STM32_Programmer_CLI.exe"
set "HEX=%PROJECT_ROOT%\build\%CONFIG%\lidar_car.hex"

if not exist "%CMAKE%" (
    echo ERROR: CMake not found: %CMAKE%
    exit /b 1
)
if not exist "%ARM_GCC%" (
    echo ERROR: ARM compiler not found: %ARM_GCC%
    exit /b 1
)
if not exist "%PROGRAMMER%" (
    echo ERROR: STM32CubeProgrammer CLI not found: %PROGRAMMER%
    exit /b 1
)

set "PATH=%ARM_BIN%;%PATH%"

echo [1/3] Configuring %CONFIG%...
"%CMAKE%" --preset %CONFIG%
if errorlevel 1 goto :build_failed

echo [2/3] Building firmware...
"%CMAKE%" --build --preset %CONFIG%
if errorlevel 1 goto :build_failed
if not exist "%HEX%" (
    echo ERROR: HEX image not found: %HEX%
    exit /b 1
)

echo [3/3] Programming %HEX% through %PORT%...
echo Set BOOT0=1, keep BOOT1=0, then reset or power-cycle the target.
pause
"%PROGRAMMER%" -c port=%PORT% br=115200 P=EVEN db=8 sb=1 fc=OFF -w "%HEX%" -v
if errorlevel 1 goto :program_failed

echo.
echo UART programming and verification completed.
echo Set BOOT0=0, then reset or power-cycle the target to start the application.
exit /b 0

:build_failed
echo ERROR: Firmware build failed. No programming was attempted.
exit /b 1

:program_failed
echo ERROR: UART programming failed. Close programs using %PORT%, set BOOT0=1, then reset the target.
exit /b 1

:usage
echo Usage: %~nx0 [COM_PORT]
echo Example: %~nx0 COM13
echo The target must be in the STM32 ROM UART bootloader: BOOT0=1, BOOT1=0, then reset.
exit /b 0
