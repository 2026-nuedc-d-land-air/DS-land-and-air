[CmdletBinding()]
param(
    [Parameter(Mandatory = $true)]
    [string]$PortName,
    [Parameter(Mandatory = $true)]
    [ValidateRange(1, 8)]
    [int]$Position,
    [Parameter(Mandatory = $true)]
    [ValidateSet('white', 'black')]
    [string]$Phase,
    [ValidateRange(1, 10)]
    [int]$SampleSeconds = 3,
    [Parameter(Mandatory = $true)]
    [string]$OutputPath
)

$ErrorActionPreference = 'Stop'

$serial = [System.IO.Ports.SerialPort]::new(
    $PortName, 115200,
    [System.IO.Ports.Parity]::None, 8,
    [System.IO.Ports.StopBits]::One
)
$serial.Handshake = [System.IO.Ports.Handshake]::None
$serial.NewLine = "`n"
$serial.ReadTimeout = 250

function Write-Record([string]$Line)
{
    $record = '{0},{1}' -f (Get-Date -Format 'o'), $Line.TrimEnd("`r")
    Add-Content -LiteralPath $OutputPath -Value $record -Encoding utf8
    Write-Output $record
}

try
{
    if (-not (Test-Path -LiteralPath $OutputPath))
    {
        New-Item -ItemType File -Path $OutputPath -Force | Out-Null
    }

    $serial.Open()
    Start-Sleep -Milliseconds 150
    $serial.DiscardInBuffer()

    Write-Record ('HOST,position,x{0},phase,{1}' -f $Position, $Phase)
    $serial.Write([string]$Position)

    $confirmed = $false
    $confirmDeadline = [DateTime]::UtcNow.AddSeconds(5)
    while ([DateTime]::UtcNow -lt $confirmDeadline)
    {
        try
        {
            $line = $serial.ReadLine().TrimEnd("`r")
            Write-Record $line
            if (($line -match '^GRAY,event,mark,') -and
                ($line -match (',mark,' + $Position + '(,|$)')))
            {
                $confirmed = $true
                break
            }
        }
        catch [System.TimeoutException]
        {
        }
    }

    if (-not $confirmed)
    {
        throw "No x$Position mark confirmation from $PortName. Check PA9/PA10 and target power."
    }

    $deadline = [DateTime]::UtcNow.AddSeconds($SampleSeconds)
    while ([DateTime]::UtcNow -lt $deadline)
    {
        try
        {
            Write-Record $serial.ReadLine()
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

Write-Output "GRAY_LOG=$OutputPath"
