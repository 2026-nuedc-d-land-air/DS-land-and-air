[CmdletBinding()]
param(
    [string]$WirelessPort = 'COM12',
    [ValidateSet(9600, 115200)]
    [int]$BaudRate = 115200,
    [ValidateRange(4, 30)]
    [int]$DurationSeconds = 6,
    [string]$OutputPath = ''
)

$ErrorActionPreference = 'Stop'

if ([string]::IsNullOrWhiteSpace($OutputPath))
{
    $stamp = Get-Date -Format 'yyyyMMdd-HHmmss'
    $projectRoot = Split-Path -Parent $PSScriptRoot
    $OutputPath = Join-Path $projectRoot "logs\line_follow_radio_heartbeat_$stamp.log"
}

function Write-TestLog([string]$Message)
{
    $record = '{0},{1}' -f (Get-Date -Format 'o'), $Message
    Add-Content -LiteralPath $OutputPath -Value $record -Encoding utf8
    Write-Output $record
}

function Get-Crc16CcittFalse([byte[]]$Data, [int]$Offset, [int]$Length)
{
    [uint16]$crc = 0xFFFF
    for ($index = 0; $index -lt $Length; ++$index)
    {
        $crc = [uint16]($crc -bxor ([uint16]$Data[$Offset + $index] -shl 8))
        for ($bit = 0; $bit -lt 8; ++$bit)
        {
            if (($crc -band 0x8000) -ne 0)
            {
                $crc = [uint16](($crc -shl 1) -bxor 0x1021)
            }
            else
            {
                $crc = [uint16]($crc -shl 1)
            }
        }
    }
    return $crc
}

function Get-UInt32BigEndian([byte[]]$Data, [int]$Offset)
{
    return [uint32]((([uint32]$Data[$Offset] -shl 24) -bor
                     ([uint32]$Data[$Offset + 1] -shl 16) -bor
                     ([uint32]$Data[$Offset + 2] -shl 8) -bor
                     ([uint32]$Data[$Offset + 3])))
}

$port = [System.IO.Ports.SerialPort]::new(
    $WirelessPort, $BaudRate,
    [System.IO.Ports.Parity]::None, 8,
    [System.IO.Ports.StopBits]::One
)
$port.Handshake = [System.IO.Ports.Handshake]::None
$port.ReadTimeout = 100
$bytes = [System.Collections.Generic.List[byte]]::new()
$frames = [System.Collections.Generic.List[object]]::new()
$validFrames = [System.Collections.Generic.List[object]]::new()
$exitCode = 0

try
{
    $port.Open()
    Start-Sleep -Milliseconds 200
    $port.DiscardInBuffer()
    Write-TestLog "START,wireless=$WirelessPort,baud=$BaudRate,expected=V22_HEARTBEAT_8B_500ms"

    $deadline = [DateTime]::UtcNow.AddSeconds($DurationSeconds)
    while ([DateTime]::UtcNow -lt $deadline)
    {
        try
        {
            $value = $port.ReadByte()
            if ($value -lt 0) { continue }
            $bytes.Add([byte]$value)

            while ($bytes.Count -ge 11)
            {
                if (($bytes[0] -ne 0xAA) -or ($bytes[1] -ne 0x55))
                {
                    $bytes.RemoveAt(0)
                    continue
                }
                if ($bytes.Count -lt 9) { break }
                $payloadLength = [int]$bytes[8]
                $frameLength = 11 + $payloadLength
                if ($payloadLength -gt 64)
                {
                    # Corrupt length: discard one byte so the stream parser
                    # can re-synchronise on a following AA 55 header.
                    $bytes.RemoveAt(0)
                    continue
                }
                if ($bytes.Count -lt $frameLength)
                {
                    break
                }

                [byte[]]$frame = $bytes.GetRange(0, $frameLength).ToArray()
                $receivedCrc = [uint16](($frame[$frameLength - 2] -shl 8) -bor
                                         $frame[$frameLength - 1])
                $calculatedCrc = Get-Crc16CcittFalse $frame 2 (7 + $payloadLength)
                if (($receivedCrc -eq $calculatedCrc) -and ($frame[2] -eq 0x02))
                {
                    $validFrames.Add([pscustomobject]@{
                        Type = [int]$frame[3]
                        Source = [int]$frame[4]
                        Destination = [int]$frame[5]
                        Sequence = [int]$frame[6]
                        PayloadLength = $payloadLength
                    })
                }
                if (($receivedCrc -eq $calculatedCrc) -and
                    ($frame[2] -eq 0x02) -and ($frame[3] -eq 0x03) -and
                    ($frame[4] -eq 0x30) -and ($frame[5] -eq 0x10) -and
                    ($payloadLength -eq 8) -and ($frame[11] -eq 0U) -and
                    ($frame[12] -eq 0U))
                {
                    $frames.Add([pscustomobject]@{
                        At = [DateTime]::UtcNow
                        Sequence = [int]$frame[6]
                        DeviceStatus = [int]$frame[9]
                        ErrorCode = [int]$frame[10]
                        UptimeMs = Get-UInt32BigEndian $frame 13
                    })
                }
                $bytes.RemoveRange(0, $frameLength)
            }
        }
        catch [System.TimeoutException]
        {
        }
    }

    $intervals = @()
    for ($index = 1; $index -lt $frames.Count; ++$index)
    {
        $intervals += ($frames[$index].At - $frames[$index - 1].At).TotalMilliseconds
    }
    $sequencesOk = $true
    for ($index = 1; $index -lt $frames.Count; ++$index)
    {
        if ($frames[$index].Sequence -ne (($frames[$index - 1].Sequence + 1) % 256))
        {
            $sequencesOk = $false
        }
    }
    $intervalsOk = (($intervals | Where-Object { ($_ -ge 400.0) -and ($_ -le 700.0) }).Count -ge 6)
    $uptimeStepsOk = $false
    if ($frames.Count -ge 2)
    {
        $uptimeStepsOk = ((1..($frames.Count - 1) | Where-Object {
            (($frames[$_].UptimeMs - $frames[$_ - 1].UptimeMs) -ge 400) -and
            (($frames[$_].UptimeMs - $frames[$_ - 1].UptimeMs) -le 700)
        }).Count -ge 6)
    }
    $carTaskRequestCount = @($validFrames | Where-Object {
        ($_.Type -eq 0x81) -and ($_.Source -eq 0x30)
    }).Count
    $pass = ($frames.Count -ge 8) -and $sequencesOk -and $intervalsOk -and $uptimeStepsOk -and
            ($carTaskRequestCount -eq 0)
    $meanInterval = if ($intervals.Count -ne 0) {
        ($intervals | Measure-Object -Average).Average
    } else {
        0.0
    }
    $sequenceText = ($frames | ForEach-Object { $_.Sequence }) -join ':'
    $statusText = ($frames | ForEach-Object { $_.DeviceStatus }) -join ':'
    $uptimeText = ($frames | ForEach-Object { $_.UptimeMs }) -join ':'
    Write-TestLog ("HEARTBEATS,valid={0},sequences={1},device_status={2},uptime_ms={3},mean_interval_ms={4:N1},intervals_ok={5},uptime_steps_ok={6},sequence_ok={7},car_task_requests={8}" -f
                   $frames.Count, $sequenceText, $statusText, $uptimeText,
                   $meanInterval, [int]$intervalsOk, [int]$uptimeStepsOk,
                   [int]$sequencesOk, $carTaskRequestCount)
    if (-not $pass)
    {
        throw 'Heartbeat broadcast test failed: need at least 8 valid consecutive V2.2 frames and 6 intervals in 400..700 ms.'
    }
    Write-TestLog 'RESULT,PASS,v22_heartbeat_broadcast_500ms=1,task_request_sent=0,motors_commanded=0'
}
catch
{
    $exitCode = 1
    Write-TestLog ("RESULT,FAIL,error={0}" -f $_.Exception.Message.Replace("`r", ' ').Replace("`n", ' '))
    Write-Error $_
}
finally
{
    if ($port.IsOpen) { $port.Close() }
    $port.Dispose()
    Write-Output "LINE_FOLLOW_RADIO_HEARTBEAT_LOG=$OutputPath"
}

if ($exitCode -ne 0)
{
    exit $exitCode
}
