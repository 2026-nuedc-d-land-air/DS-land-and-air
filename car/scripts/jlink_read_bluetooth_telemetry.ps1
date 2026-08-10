param(
    [string]$Configuration = 'BluetoothMotorDebug'
)

$ErrorActionPreference = 'Stop'
$projectRoot = Split-Path -Parent $PSScriptRoot
$bundleRoot = 'C:\Users\chenha\AppData\Local\stm32cube\bundles'
$elf = Join-Path $projectRoot "build\$Configuration\lidar_car.elf"
$gdb = Join-Path $bundleRoot `
    'gnu-gdb-for-stm32\13.3.1+st.10\bin\arm-none-eabi-gdb.exe'
$gdbServer = 'D:\AZBao\DSai\jlink\JLink_V926\JLinkGDBServerCL.exe'
$serverLog = Join-Path $env:TEMP 'lidar_car_bt_telemetry_jlink.log'
$serverErrorLog = Join-Path $env:TEMP 'lidar_car_bt_telemetry_jlink.err.log'
$serverProcess = $null

if (-not (Test-Path -LiteralPath $elf)) { throw "ELF not found: $elf" }
if (-not (Test-Path -LiteralPath $gdb)) { throw "GDB not found: $gdb" }
if (-not (Test-Path -LiteralPath $gdbServer)) {
    throw "J-Link GDB server not found: $gdbServer"
}

try
{
    Get-Process -ErrorAction SilentlyContinue |
        Where-Object { $_.ProcessName -match '^JLink' } |
        Stop-Process -Force -ErrorAction SilentlyContinue
    Start-Sleep -Milliseconds 500

    Remove-Item -LiteralPath $serverLog,$serverErrorLog `
        -Force -ErrorAction SilentlyContinue
    $serverProcess = Start-Process -FilePath $gdbServer `
        -ArgumentList @('-s','-if','SWD','-device','STM32F103ZE',
                        '-speed','1000','-endian','little','-novd','-port','2331') `
        -PassThru -WindowStyle Hidden `
        -RedirectStandardOutput $serverLog `
        -RedirectStandardError $serverErrorLog
    Start-Sleep -Seconds 2

    if ($serverProcess.HasExited)
    {
        throw 'J-Link GDB server exited before telemetry capture.'
    }

    & $gdb $elf --batch `
        -ex 'target remote localhost:2331' `
        -ex 'printf "BT_COUNT="' `
        -ex 'p/u g_bluetoothRxCount' `
        -ex 'printf "BT_LAST_MS="' `
        -ex 'p/u g_bluetoothRxLastMs' `
        -ex 'printf "BT_LAST_BYTE="' `
        -ex 'p/x g_bluetoothRxLastByte' `
        -ex 'printf "BT_HISTORY_COUNT="' `
        -ex 'p/u g_bluetoothRxHistoryCount' `
        -ex 'printf "BT_HISTORY_WRITE_INDEX="' `
        -ex 'p/u g_bluetoothRxHistoryWriteIndex' `
        -ex 'printf "BT_HISTORY="' `
        -ex 'x/16ub g_bluetoothRxHistory' `
        -ex 'monitor go' `
        -ex 'detach'

    if ($LASTEXITCODE -ne 0)
    {
        throw 'Bluetooth telemetry read failed.'
    }
}
finally
{
    if (($null -ne $serverProcess) -and (-not $serverProcess.HasExited))
    {
        Stop-Process -Id $serverProcess.Id -Force -ErrorAction SilentlyContinue
    }
}
