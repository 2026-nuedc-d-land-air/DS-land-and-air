[CmdletBinding()]
param(
    [Parameter(Mandatory = $true)]
    [string]$PortName,
    [ValidateRange(5, 90)]
    [int]$Seconds = 40,
    [string]$OutputPath = ''
)

$ErrorActionPreference = 'Stop'

if ([string]::IsNullOrWhiteSpace($OutputPath))
{
    $stamp = Get-Date -Format 'yyyyMMdd-HHmmss'
    $projectRoot = Split-Path -Parent $PSScriptRoot
    $OutputPath = Join-Path $projectRoot "logs\line_follow_frozen_$stamp.log"
}

New-Item -ItemType File -Path $OutputPath -Force | Out-Null
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
    $serial.DiscardInBuffer()
    $serial.Write('F')
    Write-Output "Sent frozen-log dump command to $PortName"

    $deadline = [DateTime]::UtcNow.AddSeconds($Seconds)
    $sawBegin = $false
    $sawEnd = $false
    while ([DateTime]::UtcNow -lt $deadline)
    {
        try
        {
            $line = $serial.ReadLine().TrimEnd("`r")
            $record = '{0},{1}' -f (Get-Date -Format 'o'), $line
            Add-Content -LiteralPath $OutputPath -Value $record -Encoding utf8
            Write-Output $record
            if ($line -like 'LF,event=dump_begin,*') { $sawBegin = $true }
            if ($line -eq 'LF,event=dump_end')
            {
                $sawEnd = $true
                break
            }
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

if ((-not $sawBegin) -or (-not $sawEnd))
{
    throw "Frozen log did not terminate. Check that LineFollowMissionDebug is flashed and stopped. Raw capture: $OutputPath"
}

Write-Output "LINE_FOLLOW_FROZEN_LOG=$OutputPath"
