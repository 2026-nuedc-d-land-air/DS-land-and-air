# Wheel direction calibration handoff

## Confirmed known-good

- Replacement L298N driver allows all four motors to move.
- Firmware test order is LF forward/reverse, RF forward/reverse, LR forward/reverse, RR forward/reverse.
- Current test timing per wheel: software forward for 2 seconds, stop for 1 second, software reverse for 2 seconds, then stop for 3 seconds before switching wheels.
- Latest Debug build passed; it has not been flashed by Codex because the user will flash it manually.

## Direction evidence

| Wheel | Software forward | Software reverse | Status |
| --- | --- | --- | --- |
| LF | Actual backward observed, but stage ownership is not yet isolated | Not yet distinguished | Partial |
| RF | Actual backward observed, but stage ownership is not yet isolated | Not yet distinguished | Partial |
| LR | Actual forward observed, but stage ownership is not yet isolated | Not yet distinguished | Partial |
| RR | Actual backward | Actual forward | Confirmed; invert direction |

## Next minimum action

Flash `build/Debug/lidar_car.hex`, then report each wheel as `software forward -> actual direction` and `software reverse -> actual direction` in LF, RF, LR, RR order. Do not change final direction constants until all four rows are complete.
