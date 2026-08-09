$ErrorActionPreference = 'Stop'

$scriptPath = Join-Path $PSScriptRoot 'jlink_flash.ps1'
& $scriptPath -Configuration LineFollowJY901Debug -SwdSpeedKhz 50
exit $LASTEXITCODE
