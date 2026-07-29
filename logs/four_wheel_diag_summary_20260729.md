# Four-wheel TB6612 G auto-run diagnosis 2026-07-29

## Pin map in firmware (matches user)
- PWMA=PC8 / TIM3_CH3 (full remap)
- PWMB=PC9 / TIM3_CH4
- AIN1=PA5, AIN2=PA4
- BIN1=PD8, BIN2=PD9
- STBY=PB8
- Encoder left PF1/PF2, right PF3/PF4 (input only)

## Flash
- Program & Verify O.K. (J-Link Commander)
- Image: FourWheelTb6612Debug, auto-start G at 1500 ms
- Diag UART restored to USART1 PA9/PA10 (COM13 works)

## Log evidence (`logs/four_wheel_tb6612_fullseq_20260729-102630.log`)
Sequence: A+ -> stop -> A- -> stop -> B+ -> stop -> B- -> FINISHED

| Stage | rear_stby | pwma | pwmb | ain_bits | bin_bits | left_tr | right_tr |
| --- | --- | --- | --- | --- | --- | --- | --- |
| AUX_A_RAW_POS | 1 | 1260 (35%) | 0 | 2 (AIN1) | 0 | 0 | 0 |
| AUX_A_RAW_NEG | 1 | 1260 | 0 | 1 (AIN2) | 0 | 0 | 0 |
| AUX_B_RAW_POS | 1 | 0 | 1260 | 0 | 2 (BIN1) | 0 | 0 |
| AUX_B_RAW_NEG | 1 | 0 | 1260 | 0 | 1 (BIN2) | 0 | 0 |
| stop/FINISHED | 0 | 0 | 0 | 0 | 0 | 0 | 0 |

J-Link probe during A drive: TIM3 CCR3=0x4EC (1260), CCR4=0, AFIO_MAPR TIM3 full remap set, PC8/PC9 AF_PP.

## Conclusion
MCU software path is driving both rear channels correctly. Encoder transitions stay 0 and user reports no motion => problem is **outside MCU pin drive** (VM/GND, STBY wire, PWMA/PWMB wire, motor AO/BO wires, mechanical lock), not the G state machine.
