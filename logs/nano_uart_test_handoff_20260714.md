# Jetson Nano UART5 Test Handoff

## Scope and safety

- Test layer: UART transport, text protocol, and diagnostic logging only.
- STM32 target: STM32F103ZET6.
- Nano UART: UART5, `PC12=MCU TX`, `PD2=MCU RX`, `115200 8N1`.
- PC diagnostic UART: USART1 remap, `PB6=MCU TX`, `PB7=MCU RX`, `115200 8N1`.
- `MotorSafe_InitOff()` runs before both UARTs and `MotorSafe_ForceOff()` runs continuously.
- UART4 sends the existing `#000PDST!` through `#005PDST!` stop commands once at boot.
- UART commands do not contain any motor or servo command.
- Use 3.3 V TTL only. Connect grounds. Do not connect either board's 5 V supply pin.
- Keep motor-driver and servo power disconnected during the first transport-only test.

## Preparation status checked on 2026-07-14

- STM32 Debug and Release builds both complete without compiler warnings.
- Debug HEX SHA256: `96E71B7EB08AAB32987E2D2CA5ABD666DFBDE8E1CD7487553FEB678384DEEBB4`.
- Release HEX SHA256: `01FA56EBAF74719131938B58C33B60E9419A4561E1F51BB36493A85BA53C27B8`.
- Nano is Python `3.6.9`; pyserial `3.5` is installed; user `nano` is in `dialout`.
- No USB-TTL endpoint was enumerated during inspection.
- `/dev/ttyTHS1` is owned by active/enabled `nvgetty`.
- The local Nano script passes Python 3.6 grammar validation.
- Final SSH rechecks timed out, so the script is prepared locally but not yet
  confirmed or copied at `/home/nano/yolov5/nano_uart_link_test.py`.

## Recommended physical path: USB-TTL plugged into Nano

This path does not require changing Nano services.

```text
STM32 PC12 / UART5 TX -> USB-TTL RX
STM32 PD2  / UART5 RX <- USB-TTL TX
STM32 GND              <-> USB-TTL GND
USB-TTL USB            -> Jetson Nano USB port
```

After insertion, use the stable `/dev/serial/by-id/...` name. Do not permanently
store `/dev/ttyUSB0`, because the number may change after replugging.

## Alternate path: Nano J41 direct UART

```text
Nano J41 pin 8  TX -> STM32 PD2 / UART5 RX
Nano J41 pin 10 RX <- STM32 PC12 / UART5 TX
Nano J41 pin 6 GND <-> STM32 GND
```

Current inspection found `/dev/ttyTHS1` occupied by `nvgetty`. Do not use this
path until `nvgetty` has been deliberately stopped/disabled for the test.
`/dev/ttyS0` is a kernel console and must not be used.

## PC diagnostic USB-TTL wiring

```text
STM32 PB6 / USART1 TX -> PC USB-TTL RX
STM32 PB7 / USART1 RX <- PC USB-TTL TX
STM32 GND              <-> PC USB-TTL GND
```

In adapter-centric wording, the adapter TX wire goes to PB7 and the adapter RX
wire goes to PB6.

## Build and flash

Prepared outputs:

```text
build/Debug/lidar_car.elf
build/Debug/lidar_car.hex
build/Debug/lidar_car.bin
build/Release/lidar_car.elf
build/Release/lidar_car.hex
build/Release/lidar_car.bin
```

Do not flash `build/BluetoothMotorDebug/lidar_car.hex`; it belongs to the older
Bluetooth motor test. The dedicated wrapper below always selects Debug.

Build, flash, verify sections, reset, and run with J-Link:

```powershell
.\scripts\jlink_flash_nano_uart.ps1
```

The expected PC diagnostic boot line is:

```text
BOOT,nano_uart_test,nano_baud=115200,diag_baud=115200,nano_pins=PC12_PD2,bluetooth_pins=PD8_PD9,diag_pins=PB6_PB7,motors_safe=1,servo_stop_sent=1
```

The same safe firmware also monitors JDY-3X on USART3 full remap `PD8/PD9` at
`9600 8N1`. It logs `BT_RX`, decoded `BT_KEY`, and periodic `BT_STAT` lines;
`BLUETOOTH_MOTOR_OUTPUT_ENABLE=0` is enforced by the build definition.

The Nano UART receives this initial line after every reset:

```text
STM32_READY,nano_uart_test,115200,motors_safe=1
```

It also receives an independent one-second marker, even if Nano TX is broken:

```text
HEARTBEAT,<uptime_ms>,rx_bytes=<count>,motors_safe=1
```

## Nano preparation and run

The prepared Python 3.6-compatible script is:

```text
yolov5/nano_uart_link_test.py
```

Copy it to `/home/nano/yolov5/nano_uart_link_test.py`, then list endpoints:

```bash
python3 /home/nano/yolov5/nano_uart_link_test.py --list
```

Run a 60-second PING/ECHO/VISION protocol check with the stable device name:

```bash
cd /home/nano/yolov5
python3 nano_uart_link_test.py \
  --port /dev/serial/by-id/REPLACE_WITH_ACTUAL_DEVICE \
  --duration 60 \
  --vision-test \
  --log nano_uart_link_20260714.csv
```

Only after the low-rate test passes, run the transport stress stage with no
motor/servo power:

```bash
python3 nano_uart_link_test.py \
  --port /dev/serial/by-id/REPLACE_WITH_ACTUAL_DEVICE \
  --duration 60 \
  --vision-rate 30 \
  --log nano_uart_vision30_20260714.csv
```

## PC diagnostic capture

List COM ports if the port is unknown:

```powershell
.\scripts\capture_nano_uart_diag.ps1
```

Capture a known port for 60 seconds:

```powershell
.\scripts\capture_nano_uart_diag.ps1 -PortName COM16 -Seconds 60
```

Freeze and export the 128-event RAM ring after a test:

```powershell
.\scripts\capture_nano_uart_diag.ps1 -PortName COM16 -Seconds 20 -Command E
```

## Protocol

```text
Nano -> STM32: PING,<seq>\n
STM32 -> Nano: PONG,<seq>,<uptime_ms>\r\n

Nano -> STM32: ECHO,<payload>\n
STM32 -> Nano: ECHO_ACK,<payload>\r\n

Nano -> STM32: VISION,<seq>,<class>,<confidence>,<x1>,<y1>,<x2>,<y2>\n
STM32 -> Nano: VISION_ACK,<seq>\r\n

Nano -> STM32: STATUS\n
STM32 -> Nano: STATUS,<uptime_ms>,rx_bytes=...,rx_lines=...,parse_err=...,motors_safe=1\r\n
```

## Diagnostic log fields

```text
BOOT,...
NANO_RX_LINE,<time_ms>,<line>
NANO_TX,<time_ms>,<line>
NANO_STAT,<time_ms>,rx_bytes=...,rx_lines=...,tx_lines=...,parse_err=...,overflow=...,motors_safe=1
FAULT,<time_ms>,<reason>
```

`NANO_STAT` also separates parity, framing, noise, and overrun errors and reports
`last_rx_age_ms`. The most recent 128 protocol/error events are retained in RAM.
From the PC diagnostic adapter send `F` to freeze, `E`/`D` to freeze and export,
or `C` to clear and resume the event ring.

Per-byte logging exists behind `NANO_UART_TRACE_BYTES=1`, but remains disabled by
default because blocking byte logs can disturb high-rate UART reception.

## Pass gates

1. `STM32_READY` is received after reset.
2. At least 60 PINGs produce matching PONG sequence values.
3. PONG success rate is 100% for the isolated one-minute test.
4. ECHO payloads match exactly.
5. Synthetic VISION lines receive matching `VISION_ACK` values.
6. STM32 diagnostic `parse_err=0` and `overflow=0`.
7. `uart_pe=0`, `uart_fe=0`, `uart_ne=0`, and `uart_ore=0`.
8. `motors_safe=1` remains present and no wheel or servo moves.

## Staged order imported from the radar-car wireless history

1. W0: logic power only; verify BOOT/STM32_READY/HEARTBEAT and exact 115200 8N1.
2. W1: low-rate PING/ECHO; verify byte/line counts and zero UART hardware errors.
3. W2: test LF, CRLF, empty ECHO payload, comma payload, malformed command, and
   overlength recovery; export the event ring afterward.
4. W3: motor/servo power still off; run synthetic VISION at 30 Hz for 60 seconds.
5. W4: connect the real YOLO output, still ACK/log only and still no motor command.

Do not enter a later stage when the earlier stage has parse, overrun, endpoint,
or wiring errors. This preserves the historical “raw link before business logic”
gate and prevents a serial fault from being misdiagnosed as a vision/control bug.

## Stop conditions

- Any wheel or servo moves: remove motor power immediately and inspect the flashed image/tag.
- Garbled bytes: verify `115200 8N1`, crossed TX/RX, shared ground, and 3.3 V levels.
- No device under `/dev/serial/by-id`: USB-TTL is not enumerated; do not guess a tty name.
- Direct J41 path cannot open `/dev/ttyTHS1`: confirm `nvgetty` ownership before changing code.
