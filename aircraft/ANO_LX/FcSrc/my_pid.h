#ifndef __MY_PID_H
#define __MY_PID_H

#include "stdint.h"

extern int32_t g_xy_limit_optical;
extern int32_t g_xy_limit_radar;
extern int32_t g_xy_limit_mv;

int Limit_max_int(int input, int max);
void keep_hight(int32_t hight_target, int32_t hight);
void keep_dis(double KP, double KD, int32_t dis_x_target, int32_t dis_y_target, int32_t dis_x, int32_t dis_y, int32_t max);
void keep_yaw(int32_t yaw_target_cmd, double yaw_now);

void pid_set_optical_xy_limit(int32_t limit);
void pid_set_radar_xy_limit(int32_t limit);
void pid_set_mv_xy_limit(int32_t limit);

int32_t pid_get_optical_xy_limit(void);
int32_t pid_get_radar_xy_limit(void);
int32_t pid_get_mv_xy_limit(void);

void PID(void);

#endif
