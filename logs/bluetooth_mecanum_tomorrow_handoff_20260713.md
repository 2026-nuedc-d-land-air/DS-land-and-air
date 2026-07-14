# Bluetooth Mecanum Test Handoff

## Prepared artifacts

- Monitor-only preset: `BluetoothMotorDebug`
- Main source: `Src/main_bluetooth_mecanum_test.c`
- Motor output gate: `BLUETOOTH_MOTOR_OUTPUT_ENABLE_VALUE=0`
- Bluetooth UART: USART3 full remap, PD8 TX, PD9 RX, 9600 baud
- Alternate monitor preset: `BluetoothMotor115200Debug`
- Recent-byte telemetry: count, last byte, last timestamp, 16-byte ring buffer

## Tomorrow: phase 1, identify phone button bytes

Flash the isolated monitor build:

```powershell
.\scripts\jlink_flash_bluetooth_motor.ps1
```

Press phone buttons in a known order, for example:

```text
forward, backward, left, right, stop
```

Current phone button bytes:

```text
up=0x47 ('G'), left=0x48 ('H'), OK=0x49 ('I'),
right=0x4A ('J'), down=0x4B ('K')
```

Read the STM32 memory telemetry without resetting it:

```powershell
.\scripts\jlink_read_bluetooth_telemetry.ps1
```

The important output is:

```text
BT_COUNT
BT_LAST_BYTE
BT_HISTORY_COUNT
BT_HISTORY
```

## Tomorrow: phase 2, enable low-speed motion

Only after the phone byte map is confirmed:

1. Update the command mapping in `app_bluetooth_motor_test.c` if required.
2. Change the Bluetooth test preset output gate from `0` to `1`.
3. Build and flash again.
4. Lift the chassis and test one command at 35% PWM.
5. Read telemetry and record LF/RF/LR/RR actual direction.
6. Correct only the mismatched wheel sign or command vector.

## Known-good physical forward polarity

- LF: PE2=0, PE3=1
- RF: PE4=1, PE5=0
- LR: PC6=0, PG1=1
- RR: PE9=1, PE7=0

Do not use the default `Debug` preset for this test: its current main belongs to
the independent Nano UART5 task.
