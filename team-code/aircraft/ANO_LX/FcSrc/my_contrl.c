#include "my_contrl.h"
#include "my_uart.h"
#include "my_send_test.h"
#include "LX_FC_Fun.h"
#include "my_get_data.h"

int my_mode;
int my_give_vel_x,
    my_give_vel_y,
    my_give_vel_z,
    my_give_vel_yaw,
    my_give_vel_rol,
    my_give_vel_pit;
extern int my_task_flag;
unsigned int mission_step;
int my_task_time_dly_cnt_ms, time_cnt_ms;

int32_t height_target, dis_x_target, dis_y_target, yaw_target;
int32_t hight_now, dis_x_now, dis_y_now, yaw_now, yaw_base;
double yaw_zero, yaw_fix;
int32_t keep_hight_flag, keep_dis_flag, keep_yaw_flag, keep_cam_flag, keep_radar_flag;
int32_t dis_x_zero = 0, dis_y_zero = 0;
int dis_target_num;
uint8_t yaw_mode = YAW_MODE_HOLD_ABS;

int dis_x_cam_target, dis_y_cam_target;
uint8_t cam_target_control_flag, cam_target_code_identity_flag;

void mode_select(int mode)
{
    keep_hight_flag = 0;
    keep_dis_flag = 0;
    keep_cam_flag = 0;
    keep_yaw_flag = 0;
    keep_radar_flag = 0;

    switch (mode)
    {
    case 1:
    case 2:
        keep_yaw_flag = 1;
        keep_hight_flag = 1;
        keep_dis_flag = 1;
        break;

    case 3:
        keep_yaw_flag = 1;
        keep_hight_flag = 1;
        keep_cam_flag = 1;
        break;

    case 4:
        keep_yaw_flag = 1;
        keep_hight_flag = 1;
        keep_radar_flag = 1;
        break;

    default:
        break;
    }
}

void all_data_init(void)
{
    my_task_time_dly_cnt_ms = 0;
    time_cnt_ms = 0;
}

void my_delay_ms(int time)
{
    if (my_task_time_dly_cnt_ms < time)
    {
        my_task_time_dly_cnt_ms += 20;
    }
    else
    {
        my_task_time_dly_cnt_ms = 0;
        my_send_esp_1_test(ESP_DIAG_START_WARNING);
        mission_step++;
    }
}

float normalize_angle(float angle)
{
    while (angle > 180.0f)
    {
        angle -= 360.0f;
    }
    while (angle <= -180.0f)
    {
        angle += 360.0f;
    }
    return angle;
}

void yaw_set_hold_target(int32_t target_deg)
{
    yaw_target = (int32_t)normalize_angle((float)target_deg);
    yaw_mode = YAW_MODE_HOLD_ABS;
}

void yaw_hold_current(void)
{
    yaw_target = (int32_t)normalize_angle((float)yaw_fix);
    yaw_mode = YAW_MODE_HOLD_ABS;
}

void yaw_set_turn_delta(int32_t delta_deg)
{
    yaw_target = (int32_t)(yaw_fix + (double)delta_deg);
    yaw_mode = YAW_MODE_TURN_REL;
}

void left_turn(int32_t delta_deg)
{
    if (delta_deg < 0)
    {
        delta_deg = -delta_deg;
    }
    yaw_set_turn_delta(-delta_deg);
}

void right_turn(int32_t delta_deg)
{
    if (delta_deg < 0)
    {
        delta_deg = -delta_deg;
    }
    yaw_set_turn_delta(delta_deg);
}

float yaw_get_error(void)
{
    if (yaw_mode == YAW_MODE_TURN_REL)
    {
        return (float)yaw_target - (float)yaw_fix;
    }
    return normalize_angle((float)yaw_target - (float)yaw_fix);
}

uint8_t yaw_arrived(float threshold_deg)
{
    float err = yaw_get_error();
    return (err < threshold_deg && err > -threshold_deg) ? 1u : 0u;
}

void emergency_stop(void)
{
    my_give_vel_rol = 0;
    my_give_vel_pit = 0;
}

void set_dis_zero(void)
{
    dis_target_num = 0;
    dis_x_zero = dis_x_slam;
    dis_y_zero = dis_y_slam;
}

void set_dis_field_global(void)
{
    /* MID360 and CAR_POSE are already in the calibrated FIELD_GLOBAL frame. */
    dis_target_num = 0;
    dis_x_zero = 0;
    dis_y_zero = 0;
}

int32_t get_corrected_dis_x(int32_t dis_x)
{
    return dis_x - dis_x_zero;
}

int32_t get_corrected_dis_y(int32_t dis_y)
{
    return dis_y - dis_y_zero;
}

void dis_target_select(int dis_target_num_local)
{
    switch (dis_target_num_local)
    {
    case 0:
        dis_x_target = 0;
        dis_y_target = 0;
        break;
    case 1:
        dis_x_target = 430;
        dis_y_target = 10;
        break;
    case 2:
        dis_x_target = 430;
        dis_y_target = 370;
        break;
    case 3:
        dis_x_target = 340;
        dis_y_target = 370;
        break;
    case 4:
        dis_x_target = 340;
        dis_y_target = 80;
        break;
    case 5:
        dis_x_target = 270;
        dis_y_target = 80;
        break;
    case 6:
        dis_x_target = 270;
        dis_y_target = 370;
        break;
    case 7:
        dis_x_target = 200;
        dis_y_target = 370;
        break;
    case 8:
        dis_x_target = 200;
        dis_y_target = 80;
        break;
    case 9:
        dis_x_target = 130;
        dis_y_target = 80;
        break;
    case 10:
        dis_x_target = 130;
        dis_y_target = 370;
        break;
    case 11:
        dis_x_target = 0;
        dis_y_target = 370;
        break;
    case 12:
        dis_x_target = 0;
        dis_y_target = 0;
        break;
    default:
        break;
    }
}
