param(
    [string]$PortName = 'COM13',
    [int]$CaptureSeconds = 22,
    [switch]$Flash,
    [ValidatePattern('^[TIGOUDKSQR]$')]
    [string]$Command = 'T'
)

$ErrorActionPreference = 'Stop'

if (($Command -eq 'T') -and ($CaptureSeconds -lt 18))
{
    throw 'CaptureSeconds must be at least 18 to include the complete six-pose sequence.'
}
if (($Command -ne 'T') -and ($CaptureSeconds -lt 3))
{
    throw 'CaptureSeconds must be at least 3 for a single servo command.'
}

$projectRoot = Split-Path -Parent $PSScriptRoot
$timestamp = Get-Date -Format 'yyyyMMdd-HHmmss'
$outputPath = Join-Path $projectRoot "logs\servo_uart_sequence_$timestamp.log"

if ($Flash)
{
    & (Join-Path $PSScriptRoot 'jlink_flash_servo_uart_test.ps1')
    if ($LASTEXITCODE -ne 0) { throw 'Servo UART test flash failed.' }
    Start-Sleep -Seconds 2
}

$port = New-Object System.IO.Ports.SerialPort $PortName,115200,'None',8,'One'
$port.ReadTimeout = 100
$port.NewLine = "`n"

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
    Write-Output "SERVO_TEST_LOG=$outputPath"
    Write-Output $capture.ToString()
}
finally
{
    if ($port.IsOpen) { $port.Close() }
    $port.Dispose()
}
