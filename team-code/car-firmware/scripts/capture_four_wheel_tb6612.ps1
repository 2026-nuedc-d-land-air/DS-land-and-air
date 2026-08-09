[CmdletBinding()]
param(
    [Parameter(Mandatory = $true)]
    [string]$PortName,
    [ValidateSet('', 'H', 'G', 'F', 'B', 'S')]
    [string]$Command = '',
    [ValidateRange(1, 30)]
    [int]$Seconds = 10,
    [string]$OutputPath = ''
)

$ErrorActionPreference = 'Stop'

if ([string]::IsNullOrWhiteSpace($OutputPath))
{
    $stamp = Get-Date -Format 'yyyyMMdd-HHmmss'
    $projectRoot = Split-Path -Parent $PSScriptRoot
    $OutputPath = Join-Path $projectRoot "logs\four_wheel_tb6612_$stamp.log"
}

$serial = [System.IO.Ports.SerialPort]::new(
    $PortName, 115200,
    [System.IO.Ports.Parity]::None, 8,
    [System.IO.Ports.StopBits]::One
)
$serial.Handshake = [System.IO.Ports.Handshake]::None
$serial.NewLine = "`n"
$serial.ReadTimeout = 250

try
{
    $serial.Open()
    Start-Sleep -Milliseconds 250
    if (-not [string]::IsNullOrWhiteSpace($Command))
    {
        $serial.Write($Command)
        Write-Output "Sent four-wheel command: $Command"
    }

    Write-Output "Capturing $PortName at 115200 8N1 to $OutputPath"
    $deadline = [DateTime]::UtcNow.AddSeconds($Seconds)
    while ([DateTime]::UtcNow -lt $deadline)
    {
        try
        {
            $line = $serial.ReadLine().TrimEnd("`r")
            $record = '{0},{1}' -f (Get-Date -Format 'o'), $line
            Add-Content -LiteralPath $OutputPath -Value $record -Encoding utf8
            Write-Output $record
        }
        catch [System.TimeoutException]
        {
        }
    }
}
finally
{
    if ($serial.IsOpen)
    {
        $serial.Close()
    }
    $serial.Dispose()
}

Write-Output "FOUR_WHEEL_LOG=$OutputPath"
