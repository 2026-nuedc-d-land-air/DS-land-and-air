# Bluetooth Mecanum Direction Test

## UART

- Bluetooth: USART3 full remap, PD8 TX, PD9 RX, default 9600 baud.
- Diagnostic log: USART1 default, PA9 TX, PA10 RX, 115200 baud, routed to
  J-Link OB CDC COM port.

## Current command map

| Phone signal | Requested motion | LF | RF | LR | RR |
| --- | --- | ---: | ---: | ---: | ---: |
| `F` | Forward | +18 | +18 | +18 | +18 |
| `B` | Backward | -18 | -18 | -18 | -18 |
| `L` | Turn left | +8 | +18 | +8 | +18 |
| `R` | Turn right | +18 | +8 | +18 | +8 |
| `Q` | Rotate left | -18 | +18 | -18 | +18 |
| `E` | Rotate right | +18 | -18 | +18 | -18 |
| `S` or `0` | Stop | 0 | 0 | 0 | 0 |

The phone's confirmed aliases are `G=F`, `H=L`, `J=R`, `K=B`, and `I=S`.

After every power-up, press `I`/OK once to arm movement. Until then all motion
commands are logged but ignored. The motor build sends no READY or ACK strings
back to the Bluetooth module, preventing UART echo from becoming commands.

Unknown signals are logged and force a stop. Motion also stops after 800 ms
without another recognized motion signal. The current build has
`BLUETOOTH_MOTOR_OUTPUT_ENABLE=0`, so all motor commands are preview-only and
the four motors remain stopped.

## Verified physical positive direction

- LF forward: PE2=0, PE3=1, PWM PA2/TIM2_CH3.
- RF forward: PE4=1, PE5=0, PWM PA3/TIM2_CH4.
- LR forward after assembled-vehicle verification: PC6=1, PG1=0, PWM PA6.
- RR forward after assembled-vehicle verification: PE9=0, PE7=1, PWM PA7.

## Test procedure

1. Lift the chassis so all four wheels can rotate freely.
2. Open the J-Link CDC COM port at 115200 8N1.
3. Reset the STM32 and confirm `BT_MOTOR_READY`.
4. Confirm the boot line contains `output_enabled=0`.
5. Press one phone button once.
6. Save the resulting `BT_RX` and `MOTOR_PREVIEW` lines. The phone should also
   receive `ACK,<signal>` for recognized ASCII commands or `UNKNOWN,<signal>`.
7. Send the captured button-to-byte mapping for the next motor-enabled build.

Example expected log for phone signal `F`:

```text
BT_RX,1234,B,0x46,F
MOTOR_PREVIEW,1234,F,35,35,35,35
```
