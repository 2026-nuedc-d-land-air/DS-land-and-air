[CmdletBinding()]
param(
    [Parameter(Mandatory = $true)]
    [string]$PortName,
    [switch]$Calibrate,
    [switch]$PerPositionReference,
    [ValidateRange(1, 8)]
    [int[]]$Positions = @(1, 2, 3, 4, 5, 6, 7, 8),
    [ValidateRange(1, 10)]
    [int]$SampleSeconds = 3,
    [ValidateRange(2, 10)]
    [int]$CalibrationSeconds = 3,
    [string]$OutputPath = ''
)

$ErrorActionPreference = 'Stop'

if ($Calibrate -and $PerPositionReference)
{
    throw 'Use either -Calibrate for a whole-array reference or -PerPositionReference for map-local white/black pairs.'
}

$Positions = @($Positions | Select-Object -Unique)
if ($Positions.Count -eq 0)
{
    throw 'Provide at least one physical position with -Positions 1..8.'
}

if ([string]::IsNullOrWhiteSpace($OutputPath))
{
    $stamp = Get-Date -Format 'yyyyMMdd-HHmmss'
    $projectRoot = Split-Path -Parent $PSScriptRoot
    $OutputPath = Join-Path $projectRoot "logs\gray_mapping_$stamp.log"
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

function Write-GrayLine([string]$Line)
{
    $record = '{0},{1}' -f (Get-Date -Format 'o'), $Line.TrimEnd("`r")
    Add-Content -LiteralPath $OutputPath -Value $record -Encoding utf8
    Write-Output $record
}

function Write-PhaseMarker([int]$Position, [string]$Phase)
{
    $record = '{0},HOST,position,x{1},phase,{2}' -f (Get-Date -Format 'o'), $Position, $Phase
    Add-Content -LiteralPath $OutputPath -Value $record -Encoding utf8
    Write-Output $record
}

function Capture-Window([int]$Seconds)
{
    $deadline = [DateTime]::UtcNow.AddSeconds($Seconds)
    while ([DateTime]::UtcNow -lt $deadline)
    {
        try
        {
            Write-GrayLine $serial.ReadLine()
        }
        catch [System.TimeoutException]
        {
        }
    }
}

function Send-GrayCommandAndWait(
    [string]$Command,
    [string]$ExpectedEvent,
    [string]$Description
)
{
    for ($attempt = 1; $attempt -le 3; $attempt++)
    {
        $serial.DiscardInBuffer()
        $serial.Write($Command)
        Write-Output "Sent $Description (attempt $attempt/3); waiting for firmware confirmation."

        $deadline = [DateTime]::UtcNow.AddSeconds(5)
        while ([DateTime]::UtcNow -lt $deadline)
        {
            try
            {
                $line = $serial.ReadLine().TrimEnd("`r")
                Write-GrayLine $line
                if ($line -match $ExpectedEvent)
                {
                    return
                }
            }
            catch [System.TimeoutException]
            {
            }
        }
    }

    throw "Firmware did not confirm $Description after three attempts. Stop and check the Link serial connection."
}

try
{
    $serial.Open()
    Start-Sleep -Milliseconds 250

    if ($Calibrate)
    {
        Read-Host 'Place every active gray head steadily on white, then press Enter' | Out-Null
        Send-GrayCommandAndWait 'W' 'GRAY,event,cal_white,' 'white calibration'
        Write-Output "Capturing white reference for $CalibrationSeconds seconds."
        Capture-Window $CalibrationSeconds

        Read-Host 'Place every active gray head steadily on black line, then press Enter' | Out-Null
        Send-GrayCommandAndWait 'K' 'GRAY,event,cal_black,' 'black calibration'
        Write-Output "Capturing black reference for $CalibrationSeconds seconds."
        Capture-Window $CalibrationSeconds
    }

    foreach ($position in $Positions)
    {
        # A periodic sample carrying the requested mark is also a valid acknowledgement.
        $markPattern = ',mark,' + $position + '(,|$)'
        if ($PerPositionReference)
        {
            Read-Host "Place x$position on adjacent map white (same height and lighting), then press Enter" | Out-Null
            Write-PhaseMarker $position 'white'
            Send-GrayCommandAndWait ([string]$position) $markPattern "x$position white marker"
            Write-Output "Sampling x$position map white for $SampleSeconds seconds."
            Capture-Window $SampleSeconds

            Read-Host "Move only x$position onto the real map black line, then press Enter" | Out-Null
            Write-PhaseMarker $position 'black'
            Send-GrayCommandAndWait ([string]$position) $markPattern "x$position black marker"
            Write-Output "Sampling x$position map black for $SampleSeconds seconds."
            Capture-Window $SampleSeconds
        }
        else
        {
            Read-Host "Place x$position steadily on the black line, then press Enter" | Out-Null
            Send-GrayCommandAndWait ([string]$position) $markPattern "x$position marker"
            Write-Output "Marked x$position; sampling for $SampleSeconds seconds."
            Capture-Window $SampleSeconds
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

Write-Output "GRAY_LOG=$OutputPath"
