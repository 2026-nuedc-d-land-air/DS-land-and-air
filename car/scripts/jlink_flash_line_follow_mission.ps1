$ErrorActionPreference = 'Stop'
$scriptRoot = Split-Path -Parent $MyInvocation.MyCommand.Path
& (Join-Path $scriptRoot 'jlink_flash.ps1') -Configuration LineFollowMissionDebug
exit $LASTEXITCODE
