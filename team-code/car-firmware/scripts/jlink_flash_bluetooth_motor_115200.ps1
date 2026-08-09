$ErrorActionPreference = 'Stop'
& (Join-Path $PSScriptRoot 'jlink_flash.ps1') `
    -Configuration BluetoothMotor115200Debug
