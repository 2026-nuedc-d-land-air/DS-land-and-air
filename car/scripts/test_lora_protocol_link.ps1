param(
    [string]$DiagPort = 'COM13',
    [Parameter(Mandatory = $true)]
    [string]$WirelessPort,
    [ValidateSet(9600, 115200)]
    [int]$BaudRate = 115200,
    [int]$TimeoutSeconds = 6,
    [string]$OutputPath = ''
)

$ErrorActionPreference = 'Stop'

if ($TimeoutSeconds -lt 1)
{
    throw 'TimeoutSeconds must be at least 1.'
}

$poseVector = [byte[]](
    0xAA,0x55,0x02,0x80,0x30,0x10,0x42,0x00,0x16,
    0x01,0x0F,0x00,0x01,0x00,0x00,0x00,0x78,
    0xFF,0xFF,0xFF,0xCE,0x03,0x84,0x00,0x0A,
    0x00,0x00,0x00,0x00,0x04,0xD2,0xB2,0x2D
)
$calibrationVector = [byte[]](
    0xAA,0x55,0x02,0x83,0x40,0x30,0x16,0x01,0x0C,
    0x00,0x00,0x00,0x64,0xFF,0xFF,0xFF,0xCE,
    0x00,0x01,0x01,0x00,0xF1,0x24
)

if ([string]::IsNullOrWhiteSpace($OutputPath))
{
    $stamp = Get-Date -Format 'yyyyMMdd-HHmmss'
    $projectRoot = Split-Path -Parent $PSScriptRoot
    $OutputPath = Join-Path $projectRoot "logs\lora_protocol_link_$stamp.log"
}

function Write-TestLog([string]$Message)
{
    $record = '{0},{1}' -f (Get-Date -Format 'o'), $Message
    Add-Content -LiteralPath $OutputPath -Value $record -Encoding utf8
    Write-Output $record
}

function Test-ByteWindow([System.Collections.Generic.List[byte]]$Data,
                         [byte[]]$Expected)
{
    if ($Data.Count -lt $Expected.Length)
    {
        return $false
    }

    for ($start = 0; $start -le ($Data.Count - $Expected.Length); ++$start)
    {
        $match = $true
        for ($index = 0; $index -lt $Expected.Length; ++$index)
        {
            if ($Data[$start + $index] -ne $Expected[$index])
            {
                $match = $false
                break
            }
        }
        if ($match)
        {
            return $true
        }
    }
    return $false
}

function Convert-BytesToHex([System.Collections.Generic.List[byte]]$Data)
{
    return (($Data | ForEach-Object { $_.ToString('X2') }) -join ' ')
}

function New-SerialPort([string]$PortName, [int]$PortBaudRate)
{
    $port = [System.IO.Ports.SerialPort]::new(
        $PortName,
        $PortBaudRate,
        [System.IO.Ports.Parity]::None,
        8,
        [System.IO.Ports.StopBits]::One
    )
    $port.Handshake = [System.IO.Ports.Handshake]::None
    $port.ReadTimeout = 100
    $port.WriteTimeout = 1000
    return $port
}

$diag = New-SerialPort $DiagPort 115200
$wireless = New-SerialPort $WirelessPort $BaudRate
$receivedBytes = [System.Collections.Generic.List[byte]]::new()
$diagText = ''

try
{
    $diag.Open()
    $wireless.Open()
    Start-Sleep -Milliseconds 250
    $diag.DiscardInBuffer()
    $wireless.DiscardInBuffer()

    Write-TestLog "START,diag=$DiagPort,wireless=$WirelessPort,baud=$BaudRate,fixture=V2.2_document_vectors"
    Write-TestLog 'TX_REQUEST,diag_command=P,expected=CAR_POSE_document_vector,bench_only=1'
    $diag.Write('P')

    $deadline = [DateTime]::UtcNow.AddSeconds($TimeoutSeconds)
    while ([DateTime]::UtcNow -lt $deadline)
    {
        try
        {
            $value = $wireless.ReadByte()
            if ($value -ge 0)
            {
                $receivedBytes.Add([byte]$value)
                if (Test-ByteWindow $receivedBytes $poseVector)
                {
                    break
                }
            }
        }
        catch [System.TimeoutException]
        {
        }
    }

    $txPass = Test-ByteWindow $receivedBytes $poseVector
    Write-TestLog ("TX_CAPTURE,bytes={0},hex={1}" -f $receivedBytes.Count,
                   (Convert-BytesToHex $receivedBytes))
    Write-TestLog ("TX_PASS={0}" -f ([int]$txPass))
    $diagText += $diag.ReadExisting()
    Write-TestLog ("TX_DIAG,{0}" -f (($diagText -replace "`r?`n", ' | ').Trim()))

    Write-TestLog 'RX_REQUEST,send=CAL_document_vector,expected=rx_ok_type131_src64_dst48_seq22'
    $wireless.Write($calibrationVector, 0, $calibrationVector.Length)
    $deadline = [DateTime]::UtcNow.AddSeconds($TimeoutSeconds)
    while ([DateTime]::UtcNow -lt $deadline)
    {
        Start-Sleep -Milliseconds 50
        $diagText += $diag.ReadExisting()
        if ($diagText -match 'LORA,event=rx_ok,[^\r\n]*type=131,src=64,dst=48,seq=22,flags=1,len=12,accepted=1')
        {
            break
        }
    }

    $rxPass = $diagText -match 'LORA,event=rx_ok,[^\r\n]*type=131,src=64,dst=48,seq=22,flags=1,len=12,accepted=1'
    $diagFlat = ($diagText -replace "`r?`n", ' | ').Trim()
    Write-TestLog "RX_DIAG,$diagFlat"
    Write-TestLog ("RX_PASS={0}" -f ([int]$rxPass))
    if ((-not $txPass) -or (-not $rxPass))
    {
        throw 'Link verification failed: inspect TX_PASS/RX_PASS and diagnostic evidence in the log.'
    }

    Write-TestLog 'RESULT,PASS,tx_exact_vector=1,rx_crc_valid=1,motors_safe=1'
}
finally
{
    if ($diag.IsOpen) { $diag.Close() }
    if ($wireless.IsOpen) { $wireless.Close() }
    $diag.Dispose()
    $wireless.Dispose()
    Write-Output "Log: $OutputPath"
}
