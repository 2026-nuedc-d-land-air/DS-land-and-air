# Servo and Bluetooth Test Plan

## Frozen baseline

- Four-wheel direction checkpoint: `55366cd`.
- This firmware must not initialize or drive the TT motor PWM paths.
- All TT motor enable, standby, and direction outputs are forced low in the main loop.

## UART allocation

| Link | Peripheral | TX | RX | Baud |
| --- | --- | --- | --- | --- |
| Diagnostic log | USART1 remap | PB6 | PB7 | 115200 |
| ZL bus servo | UART4 | PC10 | PC11 | 115200 |
| Bluetooth | USART3 full remap | PD8 | PD9 | 9600 default |
| Jetson Nano | UART5 | PC12 | PD2 | 115200 |

Bluetooth baud can be changed at build time with `BLUETOOTH_BAUDRATE`.

## Servo pose table

| ID | Joint | Initial | Increasing value | Decreasing value | Calibration note |
| --- | --- | ---: | --- | --- | --- |
| S0 | Right yaw | 1700 | Outward | Inward | Observed offset about +200; command values remain authoritative |
| S1 | Right pitch | 1800 | Up | Down | Observed offset about +321 |
| S2 | Right end effector | 986 | Outward | Inward | Verify after assembly |
| S3 | Left yaw | 1600 | Outward | Inward | Verify after assembly |
| S4 | Left pitch | 1178 | Up | Down | Verify after assembly |
| S5 | Left end effector | 1400 | Outward | Inward | Verify after assembly |

| Pose | S0 | S1 | S2 | S3 | S4 | S5 |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| INITIAL | 1700 | 1800 | 986 | 1600 | 1178 | 1400 |
| GRAB | 1550 | 1450 | 1020 | 1450 | 846 | 1485 |
| OPEN | 1650 | 1450 | 1020 | 1550 | 846 | 1485 |
| LIFT | 1520 | 1720 | 1020 | 1400 | 1100 | 1485 |
| LOWER | 1520 | 1700 | 1020 | 1400 | 1100 | 1485 |
| STACK | 1560 | 1700 | 1020 | 1450 | 1100 | 1485 |

Each move command currently uses 1500 ms. No pose is sent automatically at boot.

## Command table

Commands may be entered through Bluetooth or the diagnostic UART RX pin.

| Command | Action |
| --- | --- |
| `I` or `0` | Initial pose |
| `G` or `1` | Grab pose |
| `O` or `2` | Open pose |
| `U` or `3` | Lift pose |
| `D` or `4` | Lower pose |
| `K` or `5` | Stack pose |
| `S` | Send stop command to IDs 0-5 |
| `R` | Release torque for IDs 0-5 |
| `Q` | Query angle for IDs 0-5 |
| `H` or `?` | Print help |

## First hardware test order

1. Disconnect or lift the chassis so wheel motion cannot translate the robot.
2. Connect common ground between STM32, servo controller, Bluetooth, and their supplies.
3. Power the servos from the specified external servo supply, not from the MCU 3.3 V rail.
4. Open the PB6/PB7 diagnostic link at 115200 8N1.
5. Reset the STM32 and confirm `motors_safe=1` and `automatic_motion=0`.
6. Send `H` through Bluetooth and verify both `CMD_RX` in the diagnostic log and `ACK,H` on Bluetooth.
7. Send `Q` and verify that responses are recorded as `SERVO_RX`.
8. With the mechanism unloaded, send `I` and verify S0-S5 one at a time against the table.
9. Test `G`, `O`, `U`, `D`, and `K` individually. Do not run an automatic sequence until every row is marked pass.
10. Save the terminal capture and complete the CSV template.

## Required observations

- Confirm each physical servo ID matches S0-S5.
- Record whether increasing position moves in the documented direction.
- Record any binding, excessive current, controller reset, missing response, or unexpected wheel output.
- If any axis direction or limit is wrong, stop and update only that pose/axis after preserving this build.
