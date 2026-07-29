[CmdletBinding()]
param(
    [Parameter(Mandatory = $true)]
    [string]$PortName,
    [ValidateSet('', 'H', 'E', 'A', 'B', 'C', 'D', 'G', 'P', '1', '2', '3', '4', '5', '6', '7', '8', 'S')]
    [string]$Command = '',
    [int]$Seconds = 20,
    [string]$OutputPath = ''
)

$ErrorActionPreference = 'Stop'

if ([string]::IsNullOrWhiteSpace($OutputPath))
{
    $stamp = Get-Date -Format 'yyyyMMdd-HHmmss'
    $projectRoot = Split-Path -Parent $PSScriptRoot
    $OutputPath = Join-Path $projectRoot "logs\expansion_actuator_$stamp.log"
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
    # Opening J-Link CDC can reset the target.  Waiting a fixed time is not
    # reliable: some resets finish after 1.2 s.  Consume the startup banner
    # and send the one-byte command only after the firmware prints its command
    # list, which is emitted after all peripheral initialization is complete.
    $startupDeadline = [DateTime]::UtcNow.AddSeconds(4)
    $startupReady = $false
    while ([DateTime]::UtcNow -lt $startupDeadline)
    {
        try
        {
            $line = $serial.ReadLine().TrimEnd("`r")
            $record = '{0},{1}' -f (Get-Date -Format 'o'), $line
            Add-Content -LiteralPath $OutputPath -Value $record -Encoding utf8
            Write-Output $record
            if ($line -like 'ACT,commands,*')
            {
                $startupReady = $true
                break
            }
        }
        catch [System.TimeoutException]
        {
        }
    }
    if (-not $startupReady)
    {
        # The firmware may already have been running before this port opened.
        # Give the CDC driver one final quiet interval, then issue the command.
        Start-Sleep -Milliseconds 250
    }
    if (-not [string]::IsNullOrWhiteSpace($Command))
    {
        $serial.Write($Command)
        Write-Output "Sent expansion actuator command: $Command"
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

Write-Output "EXPANSION_ACTUATOR_LOG=$OutputPath"
