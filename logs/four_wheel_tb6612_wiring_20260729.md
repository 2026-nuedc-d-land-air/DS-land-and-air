# Four-Wheel TB6612 Test Wiring

This is the active pin assignment for the `FourWheelTb6612Debug` test image.
The front TB6612 wiring remains unchanged. The rear TB6612 uses the new pins
below. All grounds, including the motor supply ground, TB6612 ground, and MCU
ground, must be common.

## Front TB6612

| Signal | STM32 pin | Function |
|---|---|---|
| PWMA | PA2 | TIM2_CH3 PWM |
| PWMB | PA3 | TIM2_CH4 PWM |
| AIN1 / AIN2 | PE2 / PE3 | Left-front direction GPIO |
| BIN1 / BIN2 | PE4 / PE5 | Right-front direction GPIO |
| STBY | PE6 | Front driver enable |
| Left encoder A / B | PA0 / PA1 | TIM5_CH1 / CH2 |
| Right encoder A / B | PA6 / PA7 | TIM3_CH1 / CH2 |

## Rear TB6612

| Signal | STM32 pin | Function |
|---|---|---|
| PWMA | PE13 | TIM1_CH3 PWM, full remap |
| PWMB | PE14 | TIM1_CH4 PWM, full remap |
| AIN1 / AIN2 | PF1 / PF2 | Left-rear direction GPIO |
| BIN1 / BIN2 | PF3 / PF4 | Right-rear direction GPIO |
| STBY | PB9 | Rear driver enable |
| Left encoder A / B | PB6 / PB7 | TIM4_CH1 / CH2 |
| Right encoder A / B | PC6 / PC7 | TIM8_CH1 / CH2 |

Connect TB6612 `A01/A02` to the left-rear motor `M+/M-`, and `B01/B02` to the
right-rear motor `M+/M-`. If a wheel moves opposite to the required vehicle
direction, swap that motor's `M+` and `M-` leads or invert only that motor in
software after the test has established its physical direction.

The competition gray module uses `PC0/PC1/PC2` as `AD0/AD1/AD2` address outputs
and `PG0` as selected digital `OUT`; `PG1` is unused. It has no known pin conflict
with this rear bench mapping. The four-wheel test image still does not initialize
gray decoding or line following.

## Direction Calibration

The 2026-07-30 PB9 test established the signs that make `G` command physical
vehicle forward. These signs apply only to `FourWheelTb6612Debug`:

| Wheel | Forward sign |
| --- | --- |
| Front-left | -1 |
| Front-right | -1 |
| Rear-left | +1 |
| Rear-right | -1 |

## Test Image

Build and flash `FourWheelTb6612Debug`:

```powershell
.\scripts\jlink_flash_four_wheel_tb6612.ps1
```

On reset, the firmware waits one second, drives all four motors at 65% PWM for
four seconds, then coasts and disables both TB6612 modules. USART1 logs use
`PA9/PA10` at 115200 baud.

| Serial command | Action |
|---|---|
| `G` | Run all four wheels forward for four seconds |
| `B` | Run all four wheels reverse for four seconds |
| `S` | Stop and disable both drivers immediately |
| `H` | Print command help |

The current test polls the rear encoder pins for transition diagnostics. It
does not yet enable TIM4/TIM8 hardware encoder mode; that belongs in the
closed-loop four-wheel control implementation after motor direction is fixed.
