param(
    [string]$PortName = 'COM13',
    [ValidateSet('F', 'B', 'L', 'R', '+', '-', 'S', '0', 'P', 'H', '?')]
    [string]$Command = 'P',
    [int]$CaptureSeconds = 3,
    [switch]$Flash
)

$ErrorActionPreference = 'Stop'

if ($CaptureSeconds -lt 2)
{
    throw 'CaptureSeconds must be at least 2.'
}

$projectRoot = Split-Path -Parent $PSScriptRoot
$timestamp = Get-Date -Format 'yyyyMMdd-HHmmss'
$outputPath = Join-Path $projectRoot "logs\car_pwm_step_$timestamp.log"

if ($Flash)
{
    & (Join-Path $PSScriptRoot 'jlink_flash_pwm_gyro_sweep.ps1')
    if ($LASTEXITCODE -ne 0) { throw 'PWM/gyro sweep flash failed.' }
    Start-Sleep -Seconds 2
}

$port = New-Object System.IO.Ports.SerialPort $PortName,115200,'None',8,'One'
$port.ReadTimeout = 100

try
{
    $port.Open()
    Start-Sleep -Milliseconds 250
    $port.DiscardInBuffer()
    $port.Write($Command)

    $deadline = [DateTime]::UtcNow.AddSeconds($CaptureSeconds)
    $capture = New-Object System.Text.StringBuilder
    while ([DateTime]::UtcNow -lt $deadline)
    {
        $chunk = $port.ReadExisting()
        if ($chunk.Length -ne 0)
        {
            [void]$capture.Append($chunk)
        }
        Start-Sleep -Milliseconds 30
    }

    $capture.ToString() | Set-Content -LiteralPath $outputPath -Encoding ascii
    Write-Output "PWM_SWEEP_LOG=$outputPath"
    Write-Output $capture.ToString()
}
finally
{
    if ($port.IsOpen) { $port.Close() }
    $port.Dispose()
}
