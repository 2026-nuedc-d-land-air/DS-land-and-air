param(
    [ValidateSet('Debug', 'Release', 'BluetoothMotorDebug', 'BluetoothMotor115200Debug', 'BluetoothMotorDriveDebug')]
    [string]$Configuration = 'Debug'
)

$ErrorActionPreference = 'Stop'
$projectRoot = Split-Path -Parent $PSScriptRoot
$bundleRoot = 'C:\Users\chenha\AppData\Local\stm32cube\bundles'
$cmake = Join-Path $bundleRoot 'cmake\4.0.1+st.3\bin\cmake.exe'
$toolchainBin = Join-Path $bundleRoot 'gnu-tools-for-stm32\13.3.1+st.9\bin'
$ninjaBin = Join-Path $bundleRoot 'ninja\1.13.1+st.1\bin'
$hex = Join-Path $projectRoot "build\$Configuration\lidar_car.hex"
$bin = Join-Path $projectRoot "build\$Configuration\lidar_car.bin"
$elf = Join-Path $projectRoot "build\$Configuration\lidar_car.elf"
$gdb = Join-Path $bundleRoot 'gnu-gdb-for-stm32\13.3.1+st.10\bin\arm-none-eabi-gdb.exe'

$gdbServerCandidates = @(
    'D:\AZBao\DSai\jlink\JLink_V926\JLinkGDBServerCL.exe',
    'F:\keil5\keil\MDK\ARM\Segger\JLinkGDBServerCL.exe'
)
$gdbServer = $gdbServerCandidates | Where-Object { Test-Path -LiteralPath $_ } | Select-Object -First 1

if (-not $gdbServer)
{
    throw 'SEGGER JLinkGDBServerCL.exe not found.'
}
if (-not (Test-Path -LiteralPath $gdb)) { throw "GDB not found: $gdb" }

$env:PATH = "$toolchainBin;$ninjaBin;$env:PATH"

& $cmake --preset $Configuration
if ($LASTEXITCODE -ne 0) { throw 'CMake configure failed.' }

& $cmake --build --preset $Configuration
if ($LASTEXITCODE -ne 0) { throw 'CMake build failed.' }

if (-not (Test-Path -LiteralPath $hex)) { throw "HEX not found: $hex" }
if (-not (Test-Path -LiteralPath $bin)) { throw "BIN not found: $bin" }
if (-not (Test-Path -LiteralPath $elf)) { throw "ELF not found: $elf" }

$serverLog = Join-Path $env:TEMP 'lidar_car_jlink_gdbserver.log'
$serverErrorLog = Join-Path $env:TEMP 'lidar_car_jlink_gdbserver.err.log'
$serverProcess = $null

try
{
    Get-Process -ErrorAction SilentlyContinue |
        Where-Object { $_.ProcessName -match '^JLink' } |
        Stop-Process -Force -ErrorAction SilentlyContinue
    Start-Sleep -Milliseconds 500

    Remove-Item -LiteralPath $serverLog,$serverErrorLog -Force -ErrorAction SilentlyContinue
    $serverProcess = Start-Process -FilePath $gdbServer `
        -ArgumentList @('-s','-if','SWD','-device','STM32F103ZE','-speed','1000','-endian','little','-novd','-port','2331') `
        -PassThru -WindowStyle Hidden `
        -RedirectStandardOutput $serverLog -RedirectStandardError $serverErrorLog
    Start-Sleep -Seconds 2

    if ($serverProcess.HasExited)
    {
        throw "J-Link GDB Server exited early.`n$(Get-Content -Raw -LiteralPath $serverLog -ErrorAction SilentlyContinue)`n$(Get-Content -Raw -LiteralPath $serverErrorLog -ErrorAction SilentlyContinue)"
    }

    $gdbOutput = (& $gdb $elf --batch `
        -ex 'target remote localhost:2331' `
        -ex 'monitor reset' `
        -ex 'load' `
        -ex 'compare-sections' `
        -ex 'monitor reset' `
        -ex 'monitor go' `
        -ex 'detach' 2>&1 | Out-String)
    Write-Output $gdbOutput

    if (($LASTEXITCODE -ne 0) -or ($gdbOutput -match 'MIS-MATCHED|Remote communication error|Connection timed out'))
    {
        throw 'J-Link GDB flash or section comparison failed.'
    }
}
finally
{
    if (($null -ne $serverProcess) -and (-not $serverProcess.HasExited))
    {
        Stop-Process -Id $serverProcess.Id -Force -ErrorAction SilentlyContinue
    }
    if (Test-Path -LiteralPath $serverLog) { Get-Content -LiteralPath $serverLog }
    if (Test-Path -LiteralPath $serverErrorLog) { Get-Content -LiteralPath $serverErrorLog }
}
