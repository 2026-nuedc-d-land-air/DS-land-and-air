param(
    [Parameter(Mandatory = $true)]
    [string]$WirelessPort,
    [string]$DiagPort = 'COM13',
    [ValidateRange(1, 30)]
    [int]$TimeoutSeconds = 8,
    [Parameter(Mandatory = $true)]
    [ValidateRange(-4000, 4000)]
    [int]$DeltaXCm,
    [Parameter(Mandatory = $true)]
    [ValidateRange(-4000, 4000)]
    [int]$DeltaYCm,
    [Parameter(Mandatory = $true)]
    [ValidateRange(1, 65535)]
    [int]$CalibrationId,
    [ValidateRange(0, 255)]
    [int]$Sequence = 22,
    [switch]$AllowRecalibration,
    [int]$PreviousDeltaXCm = [int]::MaxValue,
    [int]$PreviousDeltaYCm = [int]::MaxValue,
    [switch]$Send
)

$ErrorActionPreference = 'Stop'
$unspecifiedDelta = [int]::MaxValue
$previousDeltaSpecified = (($PreviousDeltaXCm -ne $unspecifiedDelta) -or
                           ($PreviousDeltaYCm -ne $unspecifiedDelta))

if (($PreviousDeltaXCm -eq $unspecifiedDelta) -xor
    ($PreviousDeltaYCm -eq $unspecifiedDelta))
{
    throw 'Specify both -PreviousDeltaXCm and -PreviousDeltaYCm, or neither.'
}
if ($AllowRecalibration -and -not $PSBoundParameters.ContainsKey('Sequence'))
{
    throw 'Repeated calibration requires a new -Sequence value so it is not deduplicated with the previous 0x83 request.'
}

function Get-Crc16CcittFalse([byte[]]$Bytes, [int]$Offset, [int]$Length)
{
    [uint32]$crc = 0xFFFF
    for ($index = 0; $index -lt $Length; ++$index)
    {
        $crc = ($crc -bxor (([uint32]$Bytes[$Offset + $index]) -shl 8)) -band 0xFFFF
        for ($bit = 0; $bit -lt 8; ++$bit)
        {
            if (($crc -band 0x8000) -ne 0)
            {
                $crc = ((($crc -shl 1) -bxor 0x1021) -band 0xFFFF)
            }
            else
            {
                $crc = (($crc -shl 1) -band 0xFFFF)
            }
        }
    }
    return [uint16]$crc
}

function Add-I32Be([System.Collections.Generic.List[byte]]$Buffer, [int]$Value)
{
    [byte[]]$bytes = [BitConverter]::GetBytes([int32]$Value)
    [Array]::Reverse($bytes)
    $Buffer.AddRange($bytes)
}

function Add-U16Be([System.Collections.Generic.List[byte]]$Buffer, [int]$Value)
{
    $Buffer.Add([byte](($Value -shr 8) -band 0xFF))
    $Buffer.Add([byte]($Value -band 0xFF))
}

function New-CalibrationSetFrame
{
    $bytes = [System.Collections.Generic.List[byte]]::new()
    $bytes.AddRange([byte[]](0xAA, 0x55, 0x02, 0x83, 0x40, 0x30,
                                  [byte]$Sequence, 0x01, 0x0C))
    Add-I32Be $bytes $DeltaXCm
    Add-I32Be $bytes $DeltaYCm
    Add-U16Be $bytes $CalibrationId
    $bytes.AddRange([byte[]](0x01, 0x00))
    [byte[]]$withoutCrc = $bytes.ToArray()
    [uint16]$crc = Get-Crc16CcittFalse $withoutCrc 2 ($withoutCrc.Length - 2)
    Add-U16Be $bytes $crc
    return $bytes.ToArray()
}

function New-SerialPort([string]$PortName)
{
    $port = [System.IO.Ports.SerialPort]::new(
        $PortName, 115200, [System.IO.Ports.Parity]::None, 8,
        [System.IO.Ports.StopBits]::One)
    $port.Handshake = [System.IO.Ports.Handshake]::None
    $port.ReadTimeout = 50
    $port.WriteTimeout = 1000
    return $port
}

function Convert-ToHex([byte[]]$Bytes)
{
    return (($Bytes | ForEach-Object { $_.ToString('X2') }) -join ' ')
}

function Try-ConsumeV22Frame([System.Collections.Generic.List[byte]]$Buffer)
{
    while (($Buffer.Count -ge 2) -and
           (($Buffer[0] -ne 0xAA) -or ($Buffer[1] -ne 0x55)))
    {
        $Buffer.RemoveAt(0)
    }
    if ($Buffer.Count -lt 9)
    {
        return $null
    }

    [int]$payloadLength = $Buffer[8]
    if ($payloadLength -gt 64)
    {
        $Buffer.RemoveAt(0)
        return $null
    }
    [int]$frameLength = 11 + $payloadLength
    if ($Buffer.Count -lt $frameLength)
    {
        return $null
    }

    [byte[]]$frame = $Buffer.GetRange(0, $frameLength).ToArray()
    $Buffer.RemoveRange(0, $frameLength)
    if ($frame[2] -ne 0x02)
    {
        return $null
    }
    [uint16]$receivedCrc = (([uint16]$frame[$frameLength - 2] -shl 8) -bor
                             [uint16]$frame[$frameLength - 1])
    [uint16]$calculatedCrc = Get-Crc16CcittFalse $frame 2 (7 + $payloadLength)
    if ($receivedCrc -ne $calculatedCrc)
    {
        return $null
    }

    [byte[]]$payload = if ($payloadLength -eq 0)
    {
        @()
    }
    else
    {
        $frame[9..(8 + $payloadLength)]
    }
    return [PSCustomObject]@{
        Bytes = $frame
        Type = $frame[3]
        Source = $frame[4]
        Destination = $frame[5]
        Sequence = $frame[6]
        Flags = $frame[7]
        Payload = $payload
    }
}

function Get-U16Be([byte[]]$Bytes, [int]$Offset)
{
    return (([int]$Bytes[$Offset] -shl 8) -bor [int]$Bytes[$Offset + 1])
}

function Get-U32Be([byte[]]$Bytes, [int]$Offset)
{
    return (([uint32]$Bytes[$Offset] -shl 24) -bor
            ([uint32]$Bytes[$Offset + 1] -shl 16) -bor
            ([uint32]$Bytes[$Offset + 2] -shl 8) -bor
            [uint32]$Bytes[$Offset + 3])
}

function Get-I32Be([byte[]]$Bytes, [int]$Offset)
{
    [byte[]]$slice = $Bytes[$Offset..($Offset + 3)]
    [Array]::Reverse($slice)
    return [BitConverter]::ToInt32($slice, 0)
}

$projectRoot = Split-Path -Parent $PSScriptRoot
$logDirectory = Join-Path $projectRoot 'logs'
if (-not (Test-Path -LiteralPath $logDirectory))
{
    New-Item -ItemType Directory -Path $logDirectory -Force | Out-Null
}
if ($Send -and -not [string]::IsNullOrWhiteSpace($DiagPort) -and
    [string]::Equals($WirelessPort, $DiagPort,
                     [System.StringComparison]::OrdinalIgnoreCase))
{
    throw 'WirelessPort and DiagPort must be different COM ports. Use -DiagPort "" to run without MCU diagnostics.'
}
$stamp = Get-Date -Format 'yyyyMMdd-HHmmss'
$logPath = Join-Path $logDirectory "line_follow_calibration_link_$stamp.log"
$frame = New-CalibrationSetFrame
$wireless = $null
$diag = $null

function Write-TestLog([string]$Message)
{
    $line = '{0},{1}' -f (Get-Date -Format 'o'), $Message
    Add-Content -LiteralPath $logPath -Value $line -Encoding utf8
    Write-Output $line
}

try
{
    Write-TestLog ("CALIBRATION_SET,dx_cm={0},dy_cm={1},calibration_id={2},seq={3},frame={4}" -f
                   $DeltaXCm, $DeltaYCm, $CalibrationId, $Sequence,
                   (Convert-ToHex $frame))
    if (-not $Send)
    {
        Write-TestLog 'DRY_RUN,set -Send only while the car is stopped and the ground-station maintenance window is open'
        return
    }

    $wireless = New-SerialPort $WirelessPort
    $wireless.Open()
    if (-not [string]::IsNullOrWhiteSpace($DiagPort))
    {
        $diag = New-SerialPort $DiagPort
        $diag.Open()
    }
    Start-Sleep -Milliseconds 150
    $wireless.DiscardInBuffer()
    if ($null -ne $diag) { $diag.DiscardInBuffer() }

    $received = [System.Collections.Generic.List[byte]]::new()
    $diagText = ''
    $ackAccepted = $false
    $calibratedPose = $false
    $calibratedPoseCount = 0
    $lastCalibratedSequence = $null
    $lastCalibratedSourceTimeMs = $null
    $preCalibrationPose = $null
    $calibrationSentUtc = $null
    $verifyCoordinateTransform = $true
    $expectedObservedDeltaX = $DeltaXCm
    $expectedObservedDeltaY = $DeltaYCm
    $deadline = [DateTime]::UtcNow.AddSeconds($TimeoutSeconds)
    $maintenanceWindowSeen = $false

    Write-TestLog 'WAIT_WINDOW,require_car_pose_seq_mod_5_eq_2_then_send_after_35ms'
    while ([DateTime]::UtcNow -lt $deadline)
    {
        try
        {
            $value = $wireless.ReadByte()
            if ($value -ge 0)
            {
                $received.Add([byte]$value)
            }
        }
        catch [System.TimeoutException]
        {
        }

        while ($null -ne ($parsed = Try-ConsumeV22Frame $received))
        {
            if (($parsed.Type -eq 0x80) -and ($parsed.Source -eq 0x30) -and
                ($parsed.Destination -eq 0x10) -and
                ($parsed.Payload.Length -eq 22) -and
                (($parsed.Sequence % 5) -eq 2))
            {
                $preFlags = $parsed.Payload[1]
                $preCalibrationId = Get-U16Be $parsed.Payload 2
                $preWasCalibrated = (($preFlags -band 0x02) -ne 0)
                if ($preWasCalibrated)
                {
                    if (-not $AllowRecalibration)
                    {
                        throw 'The maintenance window CAR_POSE is already calibrated. Use -AllowRecalibration with a new -Sequence, or use PG12 to clear calibration first.'
                    }
                    if ($preCalibrationId -eq 0)
                    {
                        throw 'The current CALIBRATED CAR_POSE has CalibrationId=0 and is invalid. Use PG12 before sending 0x83.'
                    }
                    if ($previousDeltaSpecified)
                    {
                        $expectedObservedDeltaX = $DeltaXCm - $PreviousDeltaXCm
                        $expectedObservedDeltaY = $DeltaYCm - $PreviousDeltaYCm
                    }
                    else
                    {
                        $verifyCoordinateTransform = $false
                    }
                }
                elseif ($preCalibrationId -ne 0)
                {
                    throw 'The uncalibrated CAR_POSE has a nonzero CalibrationId. Use PG12 before sending 0x83.'
                }
                if (($preFlags -band 0x09) -ne 0x09)
                {
                    throw 'The maintenance window CAR_POSE lacks valid position/yaw data. Do not send 0x83.'
                }
                $preCalibrationPose = [PSCustomObject]@{
                    X = Get-I32Be $parsed.Payload 4
                    Y = Get-I32Be $parsed.Payload 8
                    Sequence = $parsed.Sequence
                    SourceTimeMs = Get-U32Be $parsed.Payload 18
                    WasCalibrated = $preWasCalibrated
                    CalibrationId = $preCalibrationId
                }
                $maintenanceWindowSeen = $true
                Write-TestLog ("WINDOW_CAR_POSE,seq={0},x_cm={1},y_cm={2},source_time_ms={3},calibrated={4},calibration_id={5}" -f
                               $preCalibrationPose.Sequence,
                               $preCalibrationPose.X,
                               $preCalibrationPose.Y,
                               $preCalibrationPose.SourceTimeMs,
                               [int]$preCalibrationPose.WasCalibrated,
                               $preCalibrationPose.CalibrationId)
                if ($preCalibrationPose.WasCalibrated)
                {
                    Write-TestLog ("RECALIBRATION_MODE,previous_calibration_id={0},coordinate_delta_check={1},expected_observed_dx_cm={2},expected_observed_dy_cm={3}" -f
                                   $preCalibrationPose.CalibrationId,
                                   [int]$verifyCoordinateTransform,
                                   $expectedObservedDeltaX,
                                   $expectedObservedDeltaY)
                }
                Start-Sleep -Milliseconds 35
                $calibrationSentUtc = [DateTime]::UtcNow
                Write-TestLog 'TX_0x83,wait_for_mcu_local_ack'
                $wireless.Write($frame, 0, $frame.Length)
                break
            }
        }
        if ($maintenanceWindowSeen) { break }
    }
    if (-not $maintenanceWindowSeen)
    {
        throw 'No CAR_POSE with Seq mod 5 = 2. Do not send CALIBRATION_SET outside the V2.3 maintenance window.'
    }

    $deadline = [DateTime]::UtcNow.AddSeconds($TimeoutSeconds)

    while ([DateTime]::UtcNow -lt $deadline)
    {
        try
        {
            $value = $wireless.ReadByte()
            if ($value -ge 0)
            {
                $received.Add([byte]$value)
            }
        }
        catch [System.TimeoutException]
        {
        }

        while ($null -ne ($parsed = Try-ConsumeV22Frame $received))
        {
            Write-TestLog ("RX,type=0x{0:X2},src=0x{1:X2},dst=0x{2:X2},seq={3},flags=0x{4:X2},payload={5}" -f
                           $parsed.Type, $parsed.Source, $parsed.Destination,
                           $parsed.Sequence, $parsed.Flags,
                           (Convert-ToHex $parsed.Payload))
            if (($parsed.Type -eq 0x11) -and ($parsed.Source -eq 0x30) -and
                ($parsed.Destination -eq 0x40) -and ($parsed.Payload.Length -eq 4) -and
                ($parsed.Payload[0] -eq 0x83) -and ($parsed.Payload[1] -eq $Sequence))
            {
                if (($parsed.Payload[2] -eq 0x00) -or ($parsed.Payload[2] -eq 0x01))
                {
                    $ackAccepted = $true
                    $ackLatencyMs = if ($null -ne $calibrationSentUtc) {
                        [math]::Round(([DateTime]::UtcNow - $calibrationSentUtc).TotalMilliseconds, 1)
                    } else {
                        -1
                    }
                    Write-TestLog ("ACK_0x83,accepted=1,owner=MCU_LOCAL,latency_ms={0}" -f $ackLatencyMs)
                }
                else
                {
                    Write-TestLog ("ACK_0x83,accepted=0,result={0},detail={1}" -f
                                   $parsed.Payload[2], $parsed.Payload[3])
                }
            }
            if (($parsed.Type -eq 0x80) -and ($parsed.Source -eq 0x30) -and
                ($parsed.Destination -eq 0x10) -and ($parsed.Payload.Length -eq 22))
            {
                $poseFlags = $parsed.Payload[1]
                $poseCalibrationId = Get-U16Be $parsed.Payload 2
                if ((($poseFlags -band 0x02) -ne 0) -and
                    ($poseCalibrationId -eq $CalibrationId))
                {
                    $poseX = Get-I32Be $parsed.Payload 4
                    $poseY = Get-I32Be $parsed.Payload 8
                    $poseSourceTimeMs = Get-U32Be $parsed.Payload 18
                    $observedDeltaX = $poseX - $preCalibrationPose.X
                    $observedDeltaY = $poseY - $preCalibrationPose.Y
                    $transformMatches = ((-not $verifyCoordinateTransform) -or
                                         (($observedDeltaX -eq $expectedObservedDeltaX) -and
                                          ($observedDeltaY -eq $expectedObservedDeltaY)))
                    if (-not $transformMatches)
                    {
                        $calibratedPoseCount = 0
                        Write-TestLog ("CALIBRATED_CAR_POSE_BAD_TRANSFORM,calibration_id={0},x_cm={1},y_cm={2},observed_dx_cm={3},observed_dy_cm={4},expected_dx_cm={5},expected_dy_cm={6}" -f
                                       $poseCalibrationId, $poseX, $poseY,
                                       $observedDeltaX, $observedDeltaY,
                                       $expectedObservedDeltaX,
                                       $expectedObservedDeltaY)
                    }
                    elseif (($null -ne $lastCalibratedSequence) -and
                            (($parsed.Sequence -eq $lastCalibratedSequence) -or
                             ($poseSourceTimeMs -le $lastCalibratedSourceTimeMs)))
                    {
                        $calibratedPoseCount = 0
                        Write-TestLog ("CALIBRATED_CAR_POSE_REORDERED,seq={0},source_time_ms={1}" -f
                                       $parsed.Sequence, $poseSourceTimeMs)
                    }
                    else
                    {
                        ++$calibratedPoseCount
                        $lastCalibratedSequence = $parsed.Sequence
                        $lastCalibratedSourceTimeMs = $poseSourceTimeMs
                        $calibratedPose = ($calibratedPoseCount -ge 3)
                        if ($preCalibrationPose.WasCalibrated -and
                            (-not $verifyCoordinateTransform))
                        {
                            Write-TestLog ("RECALIBRATED_CAR_POSE,calibration_id={0},seq={1},source_time_ms={2},x_cm={3},y_cm={4},observed_dx_cm={5},observed_dy_cm={6},coordinate_delta_check=OBSERVED_ONLY,consecutive={7}" -f
                                           $poseCalibrationId, $parsed.Sequence,
                                           $poseSourceTimeMs, $poseX, $poseY,
                                           $observedDeltaX, $observedDeltaY,
                                           $calibratedPoseCount)
                        }
                        else
                        {
                            Write-TestLog ("CALIBRATED_CAR_POSE,calibration_id={0},seq={1},source_time_ms={2},x_cm={3},y_cm={4},observed_dx_cm={5},observed_dy_cm={6},consecutive={7}" -f
                                           $poseCalibrationId, $parsed.Sequence,
                                           $poseSourceTimeMs, $poseX, $poseY,
                                           $observedDeltaX, $observedDeltaY,
                                           $calibratedPoseCount)
                        }
                    }
                }
                elseif ($calibratedPoseCount -ne 0)
                {
                    $calibratedPoseCount = 0
                    $lastCalibratedSequence = $null
                    $lastCalibratedSourceTimeMs = $null
                    Write-TestLog 'CALIBRATED_CAR_POSE_RESET,nonmatching_pose_received'
                }
            }
        }

        if ($null -ne $diag)
        {
            $diagText += $diag.ReadExisting()
        }
        if ($ackAccepted -and $calibratedPose)
        {
            break
        }
    }

    if ($null -ne $diag)
    {
        Write-TestLog ("DIAG,{0}" -f (($diagText -replace "`r?`n", ' | ').Trim()))
    }
    if (-not $ackAccepted)
    {
        throw 'No accepted 0x83 ACK. Check the MCU event log for calibration_applied_mcu or calibration_ack_radio_tx.'
    }
    if (-not $calibratedPose)
    {
        throw '0x83 was accepted but three ordered CALIBRATED CAR_POSE frames with the requested CalibrationId did not arrive. Check MCU-local DeltaX/DeltaY, raw Pi pose freshness, and request sequence.'
    }

    $resultMode = if ($preCalibrationPose.WasCalibrated) {
        'RECALIBRATION'
    } else {
        'INITIAL_CALIBRATION'
    }
    $coordinateCheck = if ($verifyCoordinateTransform) {
        'VERIFIED'
    } else {
        'OBSERVED_ONLY'
    }
    Write-TestLog ("RESULT,PASS,mode={0},ground_to_mcu_local_ack=1,calibrated_car_pose_3_frames=1,coordinate_transform={1}" -f
                   $resultMode, $coordinateCheck)
}
finally
{
    if (($null -ne $wireless) -and $wireless.IsOpen) { $wireless.Close() }
    if (($null -ne $diag) -and $diag.IsOpen) { $diag.Close() }
    if ($null -ne $wireless) { $wireless.Dispose() }
    if ($null -ne $diag) { $diag.Dispose() }
    Write-Output "Log: $logPath"
}
