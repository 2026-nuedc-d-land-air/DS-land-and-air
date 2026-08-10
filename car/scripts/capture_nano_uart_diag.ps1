param(
    [string]$PortName = '',
    [int]$Seconds = 60,
    [string]$OutputPath = '',
    [string]$Command = ''
)

$ErrorActionPreference = 'Stop'

if ([string]::IsNullOrWhiteSpace($PortName))
{
    $ports = [System.IO.Ports.SerialPort]::GetPortNames() | Sort-Object
    Write-Output ('Available COM ports: ' + ($ports -join ', '))
    throw 'Pass the diagnostic USB-TTL port, for example -PortName COM16.'
}

if ([string]::IsNullOrWhiteSpace($OutputPath))
{
    $stamp = Get-Date -Format 'yyyyMMdd-HHmmss'
    $projectRoot = Split-Path -Parent $PSScriptRoot
    $OutputPath = Join-Path $projectRoot "logs\nano_uart_diag_$stamp.log"
}

$serial = [System.IO.Ports.SerialPort]::new(
    $PortName,
    115200,
    [System.IO.Ports.Parity]::None,
    8,
    [System.IO.Ports.StopBits]::One
)
$serial.Handshake = [System.IO.Ports.Handshake]::None
$serial.NewLine = "`n"
$serial.ReadTimeout = 250

try
{
    $serial.Open()
    if (-not [string]::IsNullOrWhiteSpace($Command))
    {
        $serial.Write($Command)
        Write-Output "Sent diagnostic command: $Command"
    }
    $deadline = [DateTime]::UtcNow.AddSeconds($Seconds)
    Write-Output "Capturing $PortName at 115200 8N1 to $OutputPath"

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

Write-Output "Capture complete: $OutputPath"
