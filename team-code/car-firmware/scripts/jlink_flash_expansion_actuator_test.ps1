$ErrorActionPreference = 'Stop'
& (Join-Path $PSScriptRoot 'jlink_flash.ps1') `
    -Configuration ExpansionActuatorTestDebug `
    -Speed 100
