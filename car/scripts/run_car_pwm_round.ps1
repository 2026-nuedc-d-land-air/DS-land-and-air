param(
    [string]$PortName = 'COM13',
    [ValidateRange(1, 2000)]
    [int]$PulseDurationMs = 285,
    [ValidateRange(0, 2000)]
    [int]$InterPulseGapMs = 150,
    [ValidateRange(1, 10)]
    [int]$RepeatCount = 3,
    [ValidateRange(100, 5000)]
    [int]$PostCaptureMs = 700,
    [switch]$Flash
)

$ErrorActionPreference = 'Stop'

$projectRoot = Split-Path -Parent $PSScriptRoot
$timestamp = Get-Date -Format 'yyyyMMdd-HHmmss'
$outputPath = Join-Path $projectRoot "logs\car_pwm_round_$timestamp.log"
$sequence = @()
foreach ($motion in @('F', 'B', 'L', 'R'))
{
    for ($repeat = 0; $repeat -lt $RepeatCount; ++$repeat)
    {
        $sequence += $motion
    }
}

if ($Flash)
{
    & (Join-Path $PSScriptRoot 'jlink_flash_pwm_gyro_sweep.ps1')
    if ($LASTEXITCODE -ne 0) { throw 'PWM/gyro round flash failed.' }
    Start-Sleep -Seconds 2
}

$port = New-Object System.IO.Ports.SerialPort $PortName,115200,'None',8,'One'
$port.ReadTimeout = 100
$capture = New-Object System.Text.StringBuilder

function Receive-Until([DateTime]$Deadline)
{
    while ([DateTime]::UtcNow -lt $Deadline)
    {
        $chunk = $port.ReadExisting()
        if ($chunk.Length -ne 0)
        {
            [void]$capture.Append($chunk)
        }
        Start-Sleep -Milliseconds 15
    }
}

try
{
    $port.Open()
    Start-Sleep -Milliseconds 250
    $port.DiscardInBuffer()

    for ($index = 0; $index -lt $sequence.Count; ++$index)
    {
        $command = $sequence[$index]
        [void]$capture.Append("HOST_TX,index=$index,command=$command`r`n")
        $port.Write($command)
        Receive-Until ([DateTime]::UtcNow.AddMilliseconds(
            $PulseDurationMs + $InterPulseGapMs))
    }

    Receive-Until ([DateTime]::UtcNow.AddMilliseconds($PostCaptureMs))
    $capture.ToString() | Set-Content -LiteralPath $outputPath -Encoding ascii
    Write-Output "PWM_ROUND_LOG=$outputPath"
    Write-Output $capture.ToString()
}
finally
{
    if ($port.IsOpen) { $port.Close() }
    $port.Dispose()
}
