#ifndef __MY_GET_DATA_H
#define __MY_GET_DATA_H

#include "ANO_LX.h"
#include "stm32f4xx.h"
#include "my_contrl.h"
#include "LX_FC_EXT_Sensor.h"
#include "Drv_AnoOf.h"

extern double rol, pit, yaw, yaw_part, yaw_last, yaw_round;
extern int16_t vel_x, vel_y, vel_z;                            // 速度，单位cm/s
extern int32_t hight;                                          // 高度,单位cm
extern int32_t dis_x, dis_y, dis_x_slam, dis_y_slam, yaw_slam; // 累积的距离,单位cm        // 累积的距离,单位cm
void get_data(void);

#endif
