$ErrorActionPreference = 'Stop'

& (Join-Path $PSScriptRoot 'jlink_flash.ps1') `
    -Configuration FrontTb6612DirectTestDebug
