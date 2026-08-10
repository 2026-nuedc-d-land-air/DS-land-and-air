[CmdletBinding()]
param(
    [Parameter(Mandatory = $true)]
    [string]$PortName
)

$ErrorActionPreference = 'Stop'
$serial = [System.IO.Ports.SerialPort]::new(
    $PortName, 115200,
    [System.IO.Ports.Parity]::None, 8,
    [System.IO.Ports.StopBits]::One
)
$serial.Handshake = [System.IO.Ports.Handshake]::None
$serial.NewLine = "`n"
$serial.ReadTimeout = 500

try
{
    $serial.Open()
    Start-Sleep -Milliseconds 100
    $serial.DiscardInBuffer()
    $serial.Write('P')
    $deadline = [DateTime]::UtcNow.AddSeconds(3)
    while ([DateTime]::UtcNow -lt $deadline)
    {
        try
        {
            $line = $serial.ReadLine().TrimEnd("`r")
            if ($line -like 'LF,event=status,*')
            {
                Write-Output $line
                return
            }
        }
        catch [System.TimeoutException]
        {
        }
    }
    throw 'No line-follow status response. Confirm LineFollowMissionDebug is flashed and COM port is correct.'
}
finally
{
    if ($serial.IsOpen) { $serial.Close() }
    $serial.Dispose()
}
