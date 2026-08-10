[CmdletBinding()]
param(
    [Parameter(Mandatory = $true)]
    [string]$Path,
    [switch]$GyroRequired
)

$ErrorActionPreference = 'Stop'

if (-not (Test-Path -LiteralPath $Path))
{
    throw "Log not found: $Path"
}

$records = @()
$events = @{}
$timingEvents = @()
$terminalReason = 'UNKNOWN'

Get-Content -LiteralPath $Path | ForEach-Object {
    $line = $_
    $start = $line.IndexOf('LF,')
    if ($start -lt 0) { return }
    $line = $line.Substring($start)
    $parts = $line -split ','
    if (($parts.Length -lt 2) -or ($parts[0] -ne 'LF')) { return }

    if (($parts[1] -like 'event=*') -and ($parts.Length -ge 2))
    {
        $name = $parts[1].Substring(6)
        $eventField = @{}
        for ($index = 2; $index -lt $parts.Length; ++$index)
        {
            $separator = $parts[$index].IndexOf('=')
            if ($separator -gt 0)
            {
                $eventField[$parts[$index].Substring(0, $separator)] =
                    $parts[$index].Substring($separator + 1)
            }
        }
        if ($events.ContainsKey($name)) { $events[$name] += 1 }
        else { $events[$name] = 1 }
        if ($eventField.ContainsKey('elapsed_ms'))
        {
            $timingEvents += [pscustomobject]@{
                Name = $name
                Reason = if ($eventField.ContainsKey('reason')) {
                    $eventField['reason']
                } else {
                    'UNKNOWN'
                }
                TimeMs = if ($eventField.ContainsKey('t_ms')) {
                    [uint32]$eventField['t_ms']
                } else {
                    0
                }
                ElapsedMs = [uint32]$eventField['elapsed_ms']
                TimeOrigin = if ($eventField.ContainsKey('time_origin')) {
                    $eventField['time_origin']
                } else {
                    'UNKNOWN'
                }
            }
        }
        if ($name -eq 'dump_begin')
        {
            foreach ($part in $parts)
            {
                if ($part -like 'reason=*')
                {
                    $terminalReason = $part.Substring(7)
                    break
                }
            }
        }
        return
    }
    if (($parts[1] -ne 'rec') -or (($parts.Length % 2) -ne 0)) { return }

    $field = @{}
    for ($index = 2; ($index + 1) -lt $parts.Length; $index += 2)
    {
        $field[$parts[$index]] = $parts[$index + 1]
    }
    $required = @('t_ms','state','class','err_x100','yaw_ref','yaw','yaw_rate',
                  'heading_err','total_diff','target_l','target_r','meas_l',
                  'meas_r','cmd_l','cmd_r','gyro_fresh')
    if (($required | Where-Object { -not $field.ContainsKey($_) }).Count -ne 0)
    {
        return
    }
    $records += [pscustomobject]@{
        TimeMs = [int]$field['t_ms']; State = [int]$field['state']
        LineClass = [int]$field['class']; ErrorX100 = [int]$field['err_x100']
        RawMask = if ($field.ContainsKey('raw')) {
            [int]$field['raw']
        } else {
            -1
        }
        StableMask = if ($field.ContainsKey('stable')) {
            [int]$field['stable']
        } else {
            -1
        }
        LostConfirmCount = if ($field.ContainsKey('lost_confirm')) {
            [int]$field['lost_confirm']
        } else {
            -1
        }
        YawRef = [int]$field['yaw_ref']; Yaw = [int]$field['yaw']
        YawRate = [int]$field['yaw_rate']
        HeadingError = [int]$field['heading_err']; TotalDiff = [int]$field['total_diff']
        TargetL = [int]$field['target_l']; TargetR = [int]$field['target_r']
        Task2FastEntryBaseCps = if ($field.ContainsKey('task2_fast_base')) {
            [int]$field['task2_fast_base']
        } else {
            -1
        }
        Task2FastEntryRampActive = if ($field.ContainsKey('task2_fast_ramp_active')) {
            [int]$field['task2_fast_ramp_active']
        } else {
            -1
        }
        Task1DToAStabilityCapActive = if ($field.ContainsKey('task1_d_to_a_cap')) {
            [int]$field['task1_d_to_a_cap']
        } else {
            -1
        }
        MeasL = [int]$field['meas_l']; MeasR = [int]$field['meas_r']
        CmdL = [int]$field['cmd_l']; CmdR = [int]$field['cmd_r']
        GyroFresh = [int]$field['gyro_fresh']
        CenterCapture = if ($field.ContainsKey('center_capture')) {
            [int]$field['center_capture']
        } else {
            0
        }
        LostHoldDifferential = if ($field.ContainsKey('lost_hold_diff')) {
            [int]$field['lost_hold_diff']
        } else {
            0
        }
        LostYawRecovery = if ($field.ContainsKey('lost_yaw_recovery')) {
            [int]$field['lost_yaw_recovery']
        } else {
            0
        }
        LostSearchSource = if ($field.ContainsKey('lost_source')) {
            [int]$field['lost_source']
        } else {
            -1
        }
        LostReacquireCount = if ($field.ContainsKey('lost_reacquire_count')) {
            [int]$field['lost_reacquire_count']
        } else {
            0
        }
        HasLapYawTravel = $field.ContainsKey('lap_yaw_travel')
        LapYawTravel = if ($field.ContainsKey('lap_yaw_travel')) {
            [int]$field['lap_yaw_travel']
        } else {
            0
        }
        HasAReturnHeading = $field.ContainsKey('a_heading_err')
        AReturnHeadingError = if ($field.ContainsKey('a_heading_err')) {
            [int]$field['a_heading_err']
        } else {
            0
        }
        AReturnHeadingNearStart = if ($field.ContainsKey('a_heading_near_start')) {
            [int]$field['a_heading_near_start']
        } else {
            0
        }
        RunDistanceMm = if ($field.ContainsKey('run_distance_mm')) {
            [int]$field['run_distance_mm']
        } else {
            0
        }
        AMarkConfirmCount = if ($field.ContainsKey('a_mark_count')) {
            [int]$field['a_mark_count']
        } else {
            0
        }
    }
}

if ($records.Count -eq 0)
{
    throw 'No LF,rec frozen records found. Use capture_line_follow_frozen_log.ps1 after the car has stopped.'
}

function Mean([object[]]$items, [scriptblock]$selector)
{
    if ($items.Count -eq 0) { return 0.0 }
    return (($items | ForEach-Object { & $selector $_ } | Measure-Object -Average).Average)
}

$tracking = @($records | Where-Object { $_.State -eq 3 })
$absSpeedError = @($tracking | ForEach-Object {
    [math]::Abs($_.TargetL - $_.MeasL) + [math]::Abs($_.TargetR - $_.MeasR)
})
$maxAbsGrayError = ($records | ForEach-Object { [math]::Abs($_.ErrorX100) } |
                    Measure-Object -Maximum).Maximum
$maxAbsHeadingError = ($records | ForEach-Object { [math]::Abs($_.HeadingError) } |
                       Measure-Object -Maximum).Maximum
$gyroStale = @($records | Where-Object { $_.GyroFresh -eq 0 }).Count
$centerCaptureRecords = @($records | Where-Object { $_.CenterCapture -ne 0 }).Count
$edgeRecords = @($records | Where-Object { [math]::Abs($_.ErrorX100) -ge 400 }).Count
$lostHoldRecords = @($records | Where-Object { $_.State -eq 4 })
$lostDirectEdgeRecords = @($lostHoldRecords | Where-Object {
    $_.LostSearchSource -eq 1
})
$rawWhiteFilteredRecords = @($records | Where-Object {
    ($_.RawMask -eq 0) -and ($_.StableMask -gt 0)
})
$aMarkRecords = @($records | Where-Object { $_.LineClass -eq 0 })
$rawFullMarkRecords = @($records | Where-Object { $_.RawMask -eq 255 })
$hasLapYawTravel = @($records | Where-Object { $_.HasLapYawTravel }).Count -ne 0
$hasAReturnHeading = @($records | Where-Object { $_.HasAReturnHeading }).Count -ne 0
$aReturnHeadingNearStartRecords = @($records | Where-Object {
    $_.AReturnHeadingNearStart -ne 0
})
$reverseTargetRecords = @($records | Where-Object {
    ($_.TargetL -lt 0) -or ($_.TargetR -lt 0)
}).Count
$recordPeriodsMs = @()
$trackWheelTargetStepsCps = @()
$trackDifferentialStepsCps = @()
$task2FastBaseUpStepsCpsPer100Ms = @()
$recordIndex = 1
for ($recordIndex = 1; $recordIndex -lt $records.Count; ++$recordIndex)
{
    $previous = $records[$recordIndex - 1]
    $current = $records[$recordIndex]
    $recordPeriodMs = $current.TimeMs - $previous.TimeMs
    $recordPeriodsMs += $recordPeriodMs
    if (($recordPeriodMs -gt 0) -and
        ($previous.Task2FastEntryBaseCps -ge 0) -and
        ($current.Task2FastEntryBaseCps -gt $previous.Task2FastEntryBaseCps))
    {
        $task2FastBaseUpStepsCpsPer100Ms +=
            (($current.Task2FastEntryBaseCps - $previous.Task2FastEntryBaseCps) *
             100.0 / $recordPeriodMs)
    }
    if (($previous.State -eq 3) -and ($current.State -eq 3))
    {
        $trackWheelTargetStepsCps += [math]::Max(
            [math]::Abs($current.TargetL - $previous.TargetL),
            [math]::Abs($current.TargetR - $previous.TargetR))
        $trackDifferentialStepsCps += [math]::Abs(
            ($current.TargetR - $current.TargetL) -
            ($previous.TargetR - $previous.TargetL))
    }
}
$maxCommand = ($records | ForEach-Object { [math]::Max($_.CmdL, $_.CmdR) } |
               Measure-Object -Maximum).Maximum
$maxAbsLostHoldDifferential = ($lostHoldRecords | ForEach-Object {
    [math]::Abs($_.LostHoldDifferential)
} | Measure-Object -Maximum).Maximum
$maxAbsLostYawRecovery = ($lostHoldRecords | ForEach-Object {
    [math]::Abs($_.LostYawRecovery)
} | Measure-Object -Maximum).Maximum
$maxLostReacquireCount = ($lostHoldRecords | ForEach-Object {
    $_.LostReacquireCount
} | Measure-Object -Maximum).Maximum
$maxLostConfirmCount = ($records | Where-Object {
    $_.LostConfirmCount -ge 0
} | ForEach-Object { $_.LostConfirmCount } | Measure-Object -Maximum).Maximum
$maxAMarkConfirmCount = ($aMarkRecords | ForEach-Object {
    $_.AMarkConfirmCount
} | Measure-Object -Maximum).Maximum
$lostHoldWindowMs = 0
if ($lostHoldRecords.Count -ge 2)
{
    $lostHoldWindowMs = $lostHoldRecords[-1].TimeMs - $lostHoldRecords[0].TimeMs
}
if ($null -eq $maxAbsLostHoldDifferential)
{
    $maxAbsLostHoldDifferential = 0
}
if ($null -eq $maxAbsLostYawRecovery)
{
    $maxAbsLostYawRecovery = 0
}
if ($null -eq $maxLostReacquireCount)
{
    $maxLostReacquireCount = 0
}
if ($null -eq $maxLostConfirmCount)
{
    $maxLostConfirmCount = 'NA'
}
if ($null -eq $maxAMarkConfirmCount)
{
    $maxAMarkConfirmCount = 0
}
$meanRecordPeriodMs = if ($recordPeriodsMs.Count -ne 0) {
    ($recordPeriodsMs | Measure-Object -Average).Average
} else {
    0
}
$maxTrackWheelTargetStepCps = if ($trackWheelTargetStepsCps.Count -ne 0) {
    ($trackWheelTargetStepsCps | Measure-Object -Maximum).Maximum
} else {
    0
}
$maxTrackDifferentialStepCps = if ($trackDifferentialStepsCps.Count -ne 0) {
    ($trackDifferentialStepsCps | Measure-Object -Maximum).Maximum
} else {
    0
}
$task2FastEntryRampRecords = @($records | Where-Object {
    $_.Task2FastEntryRampActive -eq 1
}).Count
$task1DToAStabilityCapRecords = @($records | Where-Object {
    $_.Task1DToAStabilityCapActive -eq 1
}).Count
$maxTask2FastBaseUpStepCpsPer100Ms = if ($task2FastBaseUpStepsCpsPer100Ms.Count -ne 0) {
    ($task2FastBaseUpStepsCpsPer100Ms | Measure-Object -Maximum).Maximum
} else {
    'NA'
}

function Format-TimingEvents([object[]]$items)
{
    if ($items.Count -eq 0)
    {
        return 'NA'
    }
    return (($items | ForEach-Object {
        '{0}@{1}ms(origin={2},reason={3})' -f $_.Name, $_.ElapsedMs,
        $_.TimeOrigin, $_.Reason
    }) -join ';')
}

$motionTiming = @($timingEvents | Where-Object { $_.Name -eq 'motion_start' })
$taskTwoBTiming = @($timingEvents | Where-Object {
    ($_.Name -eq 'task2_b_within_15s') -or
    ($_.Name -eq 'task2_b_late') -or
    ($_.Name -eq 'task2_b_deadline_missed')
})
$lapTiming = @($timingEvents | Where-Object {
    ($_.Name -eq 'lap_complete') -or ($_.Name -eq 'lap_timeout')
})
$taskTwoBScore = if (($taskTwoBTiming | Where-Object {
    $_.Name -eq 'task2_b_within_15s'
}).Count -ne 0) {
    'PASS'
} elseif (($taskTwoBTiming | Where-Object {
    ($_.Name -eq 'task2_b_late') -or ($_.Name -eq 'task2_b_deadline_missed')
}).Count -ne 0) {
    'FAIL'
} else {
    'NA'
}
$lapScore = if (($lapTiming | Where-Object { $_.Name -eq 'lap_timeout' }).Count -ne 0) {
    'FAIL'
} elseif (($lapTiming | Where-Object {
    ($_.Name -eq 'lap_complete') -and ($_.ElapsedMs -le 90000)
}).Count -ne 0) {
    'PASS'
} elseif (($lapTiming | Where-Object { $_.Name -eq 'lap_complete' }).Count -ne 0) {
    'FAIL'
} else {
    'NA'
}

Write-Output "LINE_FOLLOW_LOG=$Path"
Write-Output ('RECORDS={0}, FIRST_MS={1}, LAST_MS={2}, WINDOW_MS={3}' -f
              $records.Count, $records[0].TimeMs, $records[-1].TimeMs,
              ($records[-1].TimeMs - $records[0].TimeMs))
Write-Output ('RUN_DISTANCE_MM_FIRST={0}, RUN_DISTANCE_MM_LAST={1}' -f
              $records[0].RunDistanceMm, $records[-1].RunDistanceMm)
Write-Output ('MEAN_RECORD_PERIOD_MS={0:N1}, MAX_TRACK_WHEEL_TARGET_STEP_CPS={1}, MAX_TRACK_DIFFERENTIAL_STEP_CPS={2}' -f
              $meanRecordPeriodMs, $maxTrackWheelTargetStepCps,
              $maxTrackDifferentialStepCps)
Write-Output ('TASK2_FAST_ENTRY_RAMP_RECORDS={0}, MAX_TASK2_FAST_BASE_UP_STEP_CPS_PER_100MS={1}' -f
              $task2FastEntryRampRecords,
              $(if ($maxTask2FastBaseUpStepCpsPer100Ms -is [double]) {
                    '{0:N1}' -f $maxTask2FastBaseUpStepCpsPer100Ms
                } else {
                    $maxTask2FastBaseUpStepCpsPer100Ms
                }))
Write-Output ('TASK1_D_TO_A_STABILITY_CAP_RECORDS={0}' -f
              $task1DToAStabilityCapRecords)
Write-Output ('TERMINAL_REASON={0}' -f $terminalReason)
Write-Output ('SCORING_MOTION_START={0}' -f (Format-TimingEvents $motionTiming))
Write-Output ('SCORING_TASK2_B_15S={0}, EVENTS={1}' -f
              $taskTwoBScore, (Format-TimingEvents $taskTwoBTiming))
Write-Output ('SCORING_LAP_90S={0}, EVENTS={1}' -f
              $lapScore, (Format-TimingEvents $lapTiming))
Write-Output ('TRACK_RECORDS={0}, GYRO_STALE_RECORDS={1}, MAX_ABS_GRAY_ERR_X100={2}, MAX_ABS_HEADING_ERR_TENTHS={3}, MAX_COMMAND_PCT={4}' -f
              $tracking.Count, $gyroStale, $maxAbsGrayError, $maxAbsHeadingError, $maxCommand)
Write-Output ('CENTER_CAPTURE_RECORDS={0}, EDGE_RECORDS_ABS_ERR_GE_400={1}, EDGE_RECORD_RATIO={2:P1}' -f
              $centerCaptureRecords, $edgeRecords, ($edgeRecords / [double]$records.Count))
Write-Output ('LOST_HOLD_RECORDS={0}, LOST_HOLD_WINDOW_MS={1}, MAX_ABS_LOST_HOLD_DIFF_CPS={2}, MAX_ABS_LOST_YAW_RECOVERY_CPS={3}, MAX_LOST_REACQUIRE_COUNT={4}, REVERSE_TARGET_RECORDS={5}' -f
              $lostHoldRecords.Count,
              $lostHoldWindowMs,
              $maxAbsLostHoldDifferential,
              $maxAbsLostYawRecovery,
              $maxLostReacquireCount,
              $reverseTargetRecords)
Write-Output ('RAW_WHITE_FILTERED_RECORDS={0}, MAX_LOST_CONFIRM_COUNT={1}' -f
              $rawWhiteFilteredRecords.Count,
              $maxLostConfirmCount)
Write-Output ('LOST_SEARCH_DIRECT_EDGE_RECORDS={0}' -f
              $lostDirectEdgeRecords.Count)
if ($hasLapYawTravel)
{
    $aMarkLapYawText = if ($aMarkRecords.Count -ne 0) {
        ($aMarkRecords | ForEach-Object { $_.LapYawTravel }) -join ';'
    } else {
        'NONE'
    }
    $rawFullLapYawText = if ($rawFullMarkRecords.Count -ne 0) {
        ($rawFullMarkRecords | ForEach-Object { $_.LapYawTravel }) -join ';'
    } else {
        'NONE'
    }
    Write-Output ('A_MARK_RECORDS={0}, RAW_FULL_A_RECORDS={1}, MAX_A_MARK_CONFIRM_COUNT={2}, A_MARK_LAP_YAW_TRAVEL={3}, RAW_FULL_A_LAP_YAW_TRAVEL={4}' -f
                  $aMarkRecords.Count, $rawFullMarkRecords.Count,
                  $maxAMarkConfirmCount, $aMarkLapYawText, $rawFullLapYawText)
}
else
{
    Write-Output ('A_MARK_RECORDS={0}, RAW_FULL_A_RECORDS={1}, MAX_A_MARK_CONFIRM_COUNT=NA, A_MARK_LAP_YAW_TRAVEL=NA, RAW_FULL_A_LAP_YAW_TRAVEL=NA' -f
                  $aMarkRecords.Count, $rawFullMarkRecords.Count)
}
if ($hasAReturnHeading)
{
    $minAbsAReturnHeadingError = ($records | ForEach-Object {
        [math]::Abs($_.AReturnHeadingError)
    } | Measure-Object -Minimum).Minimum
    Write-Output ('A_RETURN_HEADING_NEAR_START_RECORDS={0}, MIN_ABS_A_RETURN_HEADING_ERR_TENTHS={1}' -f
                  $aReturnHeadingNearStartRecords.Count,
                  $minAbsAReturnHeadingError)
}
else
{
    Write-Output 'A_RETURN_HEADING_NEAR_START_RECORDS=NA, MIN_ABS_A_RETURN_HEADING_ERR_TENTHS=NA'
}
Write-Output ('MEAN_TRACK_TARGET_L_CPS={0:N1}, MEAN_TRACK_MEAS_L_CPS={1:N1}, MEAN_TRACK_TARGET_R_CPS={2:N1}, MEAN_TRACK_MEAS_R_CPS={3:N1}' -f
              (Mean $tracking { param($r) $r.TargetL }), (Mean $tracking { param($r) $r.MeasL }),
              (Mean $tracking { param($r) $r.TargetR }), (Mean $tracking { param($r) $r.MeasR }))
Write-Output ('GYRO_REQUIRED={0}' -f [int]$GyroRequired.IsPresent)
if ($absSpeedError.Count -ne 0)
{
    Write-Output ('MEAN_TRACK_WHEEL_ERROR_SUM_CPS={0:N1}' -f (($absSpeedError | Measure-Object -Average).Average))
}
Write-Output 'EVENT_COUNTS:'
$events.GetEnumerator() | Sort-Object Name | ForEach-Object {
    Write-Output ('{0}={1}' -f $_.Key, $_.Value)
}

if ($GyroRequired.IsPresent -and ($gyroStale -ne 0))
{
    Write-Output 'VERDICT=FAIL_GYRO_STALE'
}
elseif ($tracking.Count -lt 10)
{
    Write-Output 'VERDICT=INCOMPLETE_TRACK_WINDOW'
}
elseif (($absSpeedError | Measure-Object -Average).Average -gt 900.0)
{
    Write-Output 'VERDICT=REVIEW_SPEED_LOOP'
}
else
{
    Write-Output 'VERDICT=REVIEW_LINE_GEOMETRY_AND_STEER_SIGN'
}
