#ifndef _MY_CONTRL_H_
#define _MY_CONTRL_H_

#include <stdint.h>

#define YAW_MODE_HOLD_ABS 0 // 保持绝对航向(最短角差)
#define YAW_MODE_TURN_REL 1 // 相对当前转动(连续角目标)

extern int my_give_vel_x, my_give_vel_y, my_give_vel_z, my_give_vel_yaw, my_give_vel_rol, my_give_vel_pit;

extern int my_mode;
extern int32_t height_target, dis_x_target, dis_y_target, yaw_target;
extern int32_t hight_now, dis_x_now, dis_y_now, yaw_now, yaw_base; // 当前数据
extern double yaw_zero, yaw_fix;                                   // yaw_fix为以yaw_zero为零点的连续角
extern int32_t keep_hight_flag, keep_dis_flag, keep_yaw_flag, keep_cam_flag, keep_radar_flag;
extern unsigned int mission_step;
extern int my_task_time_dly_cnt_ms, time_cnt_ms;
extern int32_t dis_x_zero, dis_y_zero;
extern int dis_target_num;
extern uint8_t yaw_mode;

extern int dis_x_cam_target, dis_y_cam_target;
extern uint8_t cam_target_control_flag, cam_target_code_identity_flag;

#define cam_target_control_flag_on 1
#define cam_target_control_flag_off 0

void mode_select(int mode);
void my_delay_ms(int time);
void all_data_init(void);
float normalize_angle(float angle);
void emergency_stop(void);
void set_dis_zero(void);
void set_dis_field_global(void);
int32_t get_corrected_dis_x(int32_t dis_x);
int32_t get_corrected_dis_y(int32_t dis_y);
void dis_target_select(int dis_target_num);

// yaw控制接口
void yaw_set_hold_target(int32_t target_deg);
void yaw_hold_current(void);
void yaw_set_turn_delta(int32_t delta_deg);
void left_turn(int32_t delta_deg);
void right_turn(int32_t delta_deg);
float yaw_get_error(void);
uint8_t yaw_arrived(float threshold_deg);

#endif
