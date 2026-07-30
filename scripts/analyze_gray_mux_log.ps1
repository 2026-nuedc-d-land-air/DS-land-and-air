[CmdletBinding()]
param(
    [Parameter(Mandatory = $true)]
    [string]$Path
)

$ErrorActionPreference = 'Stop'

if (-not (Test-Path -LiteralPath $Path))
{
    throw "Log not found: $Path"
}

$samples = @()
$eventCounts = @{}
$currentPosition = 0
$currentPhase = ''

Get-Content -LiteralPath $Path | ForEach-Object {
    if ($_ -match ',HOST,position,x([1-8]),phase,(white|black)')
    {
        $currentPosition = [int]$Matches[1]
        $currentPhase = $Matches[2]
        return
    }

    $parts = $_ -split ','
    if (($parts.Length -lt 4) -or ($parts[1] -ne 'GRAY') -or
        ($parts[2] -ne 'event'))
    {
        return
    }

    $fields = @{ event = $parts[3] }
    for ($index = 4; ($index + 1) -lt $parts.Length; $index += 2)
    {
        $fields[$parts[$index]] = $parts[$index + 1]
    }

    $event = $fields['event']
    if ($eventCounts.ContainsKey($event)) { $eventCounts[$event] += 1 }
    else { $eventCounts[$event] = 1 }

    if ($fields.ContainsKey('raw_mask') -and $fields.ContainsKey('active_mask') -and
        $fields.ContainsKey('stable_mask'))
    {
        $samples += [pscustomobject]@{
            Event = $event
            TimeMs = [int]$fields['t_ms']
            RawMask = [int]$fields['raw_mask']
            ActiveMask = [int]$fields['active_mask']
            StableMask = [int]$fields['stable_mask']
            State = $fields['state']
            Position = $currentPosition
            Phase = $currentPhase
        }
    }
}

if ($samples.Count -eq 0)
{
    throw 'No mux gray sample records found. Flash the mux8 GrayTrackingDebug image first.'
}

function Join-Masks([object[]]$values)
{
    return (($values | Sort-Object -Unique) -join ',')
}

Write-Output "GRAY_MUX_LOG=$Path"
Write-Output ('SAMPLES={0}, FIRST_MS={1}, LAST_MS={2}' -f $samples.Count,
              $samples[0].TimeMs, $samples[$samples.Count - 1].TimeMs)
Write-Output ('RAW_MASKS={0}' -f (Join-Masks @($samples | ForEach-Object { $_.RawMask })))
Write-Output ('ACTIVE_MASKS={0}' -f (Join-Masks @($samples | ForEach-Object { $_.ActiveMask })))
Write-Output ('STABLE_MASKS={0}' -f (Join-Masks @($samples | ForEach-Object { $_.StableMask })))
Write-Output 'EVENT_COUNTS:'
$eventCounts.GetEnumerator() | Sort-Object Name | ForEach-Object {
    Write-Output ('{0}={1}' -f $_.Key, $_.Value)
}

$mapSamples = @($samples | Where-Object {
    ($_.Event -eq 'sample') -and ($_.Position -ge 1) -and
    (($_.Phase -eq 'white') -or ($_.Phase -eq 'black'))
})

if ($mapSamples.Count -ne 0)
{
    Write-Output 'MUX_MAP_LOCAL_WHITE_BLACK:'
    Write-Output 'POS,WHITE_N,BLACK_N,SELECTED_BIT,WHITE_ACTIVE_BIT,BLACK_ACTIVE_BIT,RESULT'

    for ($position = 1; $position -le 8; ++$position)
    {
        $white = @($mapSamples | Where-Object {
            ($_.Position -eq $position) -and ($_.Phase -eq 'white')
        })
        $black = @($mapSamples | Where-Object {
            ($_.Position -eq $position) -and ($_.Phase -eq 'black')
        })
        $bit = 1 -shl ($position - 1)

        if (($white.Count -eq 0) -or ($black.Count -eq 0))
        {
            Write-Output ('{0},{1},{2},0x{3:X2},N/A,N/A,INCOMPLETE' -f
                          $position, $white.Count, $black.Count, $bit)
            continue
        }

        $whiteBits = @($white | ForEach-Object {
            if (($_.ActiveMask -band $bit) -ne 0) { 1 } else { 0 }
        })
        $blackBits = @($black | ForEach-Object {
            if (($_.ActiveMask -band $bit) -ne 0) { 1 } else { 0 }
        })
        $whiteValues = Join-Masks $whiteBits
        $blackValues = Join-Masks $blackBits
        $result = if (($whiteValues -eq '0') -and ($blackValues -eq '1'))
                  { 'CHANNEL_SWITCH' }
                  else { 'NO_CONFIRMED_SWITCH' }

        Write-Output ('{0},{1},{2},0x{3:X2},{4},{5},{6}' -f
                      $position, $white.Count, $black.Count, $bit,
                      $whiteValues, $blackValues, $result)
    }
}
