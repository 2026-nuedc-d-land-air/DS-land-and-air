#ifndef __APP_MOTOR_H_
#define __APP_MOTOR_H_

#include "AllHeader.h"
#include "bsp_motor.h"

// Half of the sum of the distances between the chassis and motors of the trolley
#define Car_APB                     (188.0f)

void Set_Motor(int MOTOR_TYPE);
void Motion_Car_Control(int16_t V_x, int16_t V_y, int16_t V_z);

#endif
