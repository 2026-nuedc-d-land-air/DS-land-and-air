# Motor Wiring Known-Good

## Rear L298N

- ENA / left rear PWM: PA6, TIM3_CH1
- IN1: PC6
- IN2: PG1
- ENB / right rear PWM: PA7, TIM3_CH2
- IN3: PE9
- IN4: PE7
- `PC6=1, PG1=0`: left rear backward
- `PC6=0, PG1=1`: left rear forward
- `PE9=1, PE7=0`: right rear forward
- `PE9=0, PE7=1`: right rear backward
- PE8 is not used because it failed the ODR/IDR and motion tests

## Front TB6612

- PWMA / left front PWM: PA2, TIM2_CH3
- AIN1: PE2
- AIN2: PE3
- PWMB / right front PWM: PA3, TIM2_CH4
- BIN1: PE4
- BIN2: PE5
- STBY: PE6
- `PE2=1, PE3=0`: left front backward
- `PE2=0, PE3=1`: left front forward
- `PE4=1, PE5=0`: right front forward
- `PE4=0, PE5=1`: right front backward

## Four-Wheel Direction Checkpoint (2026-07-13)

- Test order: left front, right front, left rear, right rear
- Each wheel: first polarity 2 s, stop 1 s, opposite polarity 2 s
- Wheel-to-wheel pause: 2 s
- Cycle pause: 3 s, then repeat
- Front enable: PA2/PWMA, PA3/PWMB, PE6/STBY
- Rear enable: PA6/ENA, PA7/ENB
- Verified physical sequence: LF backward/forward, RF forward/backward,
  LR backward/forward, RR forward/backward
