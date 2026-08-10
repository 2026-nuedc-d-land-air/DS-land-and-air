[CmdletBinding()]
param(
    [Parameter(Mandatory = $true)]
    [string]$Path
)

$ErrorActionPreference = 'Stop'

# Archive-only parser for logs produced before the YB-MVX05 mux correction.
# Those logs treated AD0/AD1/AD2 as ADC inputs and must not be used to map
# X1-X8 or tune the current line-following controller. Use
# analyze_gray_mux_log.ps1 for every current GrayTrackingDebug capture.
Write-Warning 'LEGACY ADC log analyzer: use analyze_gray_mux_log.ps1 for current mux8 captures.'

if (-not (Test-Path -LiteralPath $Path))
{
    throw "Log not found: $Path"
}

$samples = @()
$eventCounts = @{}
$marks = @()
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
    if (($parts.Length -lt 4) -or ($parts[1] -ne 'GRAY') -or ($parts[2] -ne 'event'))
    {
        return
    }

    $fields = @{ event = $parts[3] }
    for ($index = 4; ($index + 1) -lt $parts.Length; $index += 2)
    {
        $fields[$parts[$index]] = $parts[$index + 1]
    }

    if (-not $fields.ContainsKey('event'))
    {
        return
    }

    $event = $fields['event']
    if ($eventCounts.ContainsKey($event))
    {
        $eventCounts[$event] += 1
    }
    else
    {
        $eventCounts[$event] = 1
    }

    if (($event -eq 'mark') -and $fields.ContainsKey('mark') -and
        ($fields['mark'] -ne '0'))
    {
        $marks += ('L{0}@{1}ms mask={2}' -f $fields['mark'],
                   $fields['t_ms'], $fields['stable_mask'])
    }

    if ($fields.ContainsKey('adc0') -and $fields.ContainsKey('adc1') -and
        $fields.ContainsKey('adc2') -and $fields.ContainsKey('io0'))
    {
        $samples += [pscustomobject]@{
            Event = $event
            TimeMs = [int]$fields['t_ms']
            Adc0 = [int]$fields['adc0']
            Adc1 = [int]$fields['adc1']
            Adc2 = [int]$fields['adc2']
            Io0 = [int]$fields['io0']
            RawMask = [int]$fields['raw_mask']
            StableMask = [int]$fields['stable_mask']
            State = $fields['state']
            Position = $currentPosition
            Phase = $currentPhase
        }
    }
}

if ($samples.Count -eq 0)
{
    throw 'No GRAY sample records found.'
}

function Write-Range([string]$name, [object[]]$values)
{
    $minimum = ($values | Measure-Object -Minimum).Minimum
    $maximum = ($values | Measure-Object -Maximum).Maximum
    Write-Output ('{0}: min={1}, max={2}, span={3}' -f $name, $minimum, $maximum, ($maximum - $minimum))
}

Write-Output "GRAY_LOG=$Path"
Write-Output ('SAMPLES={0}, FIRST_MS={1}, LAST_MS={2}' -f $samples.Count,
              $samples[0].TimeMs, $samples[$samples.Count - 1].TimeMs)
Write-Range 'ADC0' @($samples | ForEach-Object { $_.Adc0 })
Write-Range 'ADC1' @($samples | ForEach-Object { $_.Adc1 })
Write-Range 'ADC2' @($samples | ForEach-Object { $_.Adc2 })
Write-Output ('OUT_VALUES={0}' -f (($samples | Select-Object -ExpandProperty Io0 -Unique | Sort-Object) -join ','))
Write-Output ('STABLE_MASKS={0}' -f (($samples | Select-Object -ExpandProperty StableMask -Unique | Sort-Object) -join ','))

Write-Output 'EVENT_COUNTS:'
$eventCounts.GetEnumerator() | Sort-Object Name | ForEach-Object {
    Write-Output ('{0}={1}' -f $_.Key, $_.Value)
}

if ($marks.Count -ne 0)
{
    Write-Output 'MARKS:'
    $marks | ForEach-Object { Write-Output $_ }
}

$mapSamples = @($samples | Where-Object {
    ($_.Event -eq 'sample') -and ($_.Position -ge 1) -and
    (($_.Phase -eq 'white') -or ($_.Phase -eq 'black'))
})

if ($mapSamples.Count -ne 0)
{
    Write-Output 'MAP_LOCAL_WHITE_BLACK:'
    Write-Output 'POS,WHITE_N,BLACK_N,ADC0_BLACK_MINUS_WHITE,ADC1_BLACK_MINUS_WHITE,ADC2_BLACK_MINUS_WHITE,OUT_WHITE,OUT_BLACK,OUT_RESULT'

    for ($position = 1; $position -le 8; ++$position)
    {
        $white = @($mapSamples | Where-Object {
            ($_.Position -eq $position) -and ($_.Phase -eq 'white')
        })
        $black = @($mapSamples | Where-Object {
            ($_.Position -eq $position) -and ($_.Phase -eq 'black')
        })

        if (($white.Count -eq 0) -or ($black.Count -eq 0))
        {
            Write-Output ('{0},{1},{2},N/A,N/A,N/A,N/A,N/A,INCOMPLETE' -f
                          $position, $white.Count, $black.Count)
            continue
        }

        $adc0Delta = [math]::Round((($black | Measure-Object -Property Adc0 -Average).Average -
                                    ($white | Measure-Object -Property Adc0 -Average).Average), 1)
        $adc1Delta = [math]::Round((($black | Measure-Object -Property Adc1 -Average).Average -
                                    ($white | Measure-Object -Property Adc1 -Average).Average), 1)
        $adc2Delta = [math]::Round((($black | Measure-Object -Property Adc2 -Average).Average -
                                    ($white | Measure-Object -Property Adc2 -Average).Average), 1)
        $whiteOut = (($white | Select-Object -ExpandProperty Io0 -Unique | Sort-Object) -join '/')
        $blackOut = (($black | Select-Object -ExpandProperty Io0 -Unique | Sort-Object) -join '/')
        $outResult = if (($whiteOut -ne $blackOut) -and
                         ($whiteOut -notmatch '/') -and ($blackOut -notmatch '/'))
                     { 'SWITCH' }
                     else { 'NO_CONFIRMED_SWITCH' }

        Write-Output ('{0},{1},{2},{3},{4},{5},{6},{7},{8}' -f
                      $position, $white.Count, $black.Count,
                      $adc0Delta, $adc1Delta, $adc2Delta,
                      $whiteOut, $blackOut, $outResult)
    }
}
