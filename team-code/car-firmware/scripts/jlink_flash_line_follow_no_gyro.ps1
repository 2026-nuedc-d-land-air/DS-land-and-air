[CmdletBinding()]
param(
    [ValidateRange(50, 4000)]
    [int]$SwdSpeedKhz = 1000
)

$ErrorActionPreference = 'Stop'
$scriptRoot = Split-Path -Parent $MyInvocation.MyCommand.Path
& (Join-Path $scriptRoot 'jlink_flash.ps1') -Configuration LineFollowNoGyroDebug -SwdSpeedKhz $SwdSpeedKhz
exit $LASTEXITCODE
