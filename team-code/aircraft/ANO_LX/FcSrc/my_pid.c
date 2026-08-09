#include "my_pid.h"
#include "my_contrl.h"
#include "my_get_data.h"
#include "Ano_Math.h"


double KP_xy = 1, KI_xy = 0, KD_xy = 0, intrgrate_max = 30;

double KP_radar = 0.45, KI_radar = 0, KD_radar = 0.05;

double KP_MV = 0.6, KI_MV = 0, KD_MV = 0;
// 视觉联调单独给前后/左右两个轴留调参入口。
double KP_MV_X = 0.3;
double KP_MV_Y = 0.3;

/* 视觉位置环 D 项：先从小值开始 */
double KD_MV_X = 0.08;
double KD_MV_Y = 0.08;


double KP_yaw = 2.0;
int32_t yaw_deadband = 1;
int32_t yaw_output_max = 30;

// 位置环速度上限，可由任务层动态修改
int32_t g_xy_limit_optical = 30;
int32_t g_xy_limit_radar = 25;
int32_t g_xy_limit_mv = 15;

#define CAM_DEADBAND_CM 0.5
#define CAM_D_LIMIT_CMPS 8

/* 如果实际测试发现方向反了，只改下面两个符号即可。
 * 协议预期：x正=机头前方，y正=机头左侧。
 */
#define CAM_X_SIGN 1
#define CAM_Y_SIGN 1
#define CAM_X_LIMIT_CMPS 20
#define CAM_Y_LIMIT_CMPS 18

static int32_t abs_i32_pid(int32_t x)
{
    return (x >= 0) ? x : -x;
}

int Limit_max_int(int input, int max)
{
    if (input > max)
    {
        input = max;
    }
    else if (input < -max)
    {
        input = -max;
    }
    return input;
}

static int32_t clamp_limit(int32_t limit)
{
    if (limit < 5)
    {
        return 5;
    }
    if (limit > 50)
    {
        return 50;
    }
    return limit;
}

void pid_set_optical_xy_limit(int32_t limit)
{
    g_xy_limit_optical = clamp_limit(limit);
}

void pid_set_radar_xy_limit(int32_t limit)
{
    g_xy_limit_radar = clamp_limit(limit);
}

void pid_set_mv_xy_limit(int32_t limit)
{
    g_xy_limit_mv = clamp_limit(limit);
}

int32_t pid_get_optical_xy_limit(void)
{
    return g_xy_limit_optical;
}

int32_t pid_get_radar_xy_limit(void)
{
    return g_xy_limit_radar;
}

int32_t pid_get_mv_xy_limit(void)
{
    return g_xy_limit_mv;
}

/* ==========================
 * 内部计算函数：只计算，不直接写 my_give_vel_*
 * ========================== */

static int32_t calc_hight_vel(int32_t target_hight, int32_t now_hight)
{
    int32_t cal_vel_hight;

    cal_vel_hight = (target_hight - now_hight) * 3;
    return Limit_max_int(cal_vel_hight, 20);
}

static void calc_dis_vel(double KP,
                         double KD,
                         int32_t target_x,
                         int32_t target_y,
                         int32_t now_x,
                         int32_t now_y,
                         int32_t max,
                         int32_t *out_vx,
                         int32_t *out_vy)
{
    int32_t dx, dy;
    int32_t error;
    int32_t kp_out, kd_out, total_out;
    int32_t cal_vel_x, cal_vel_y;
    static int32_t error_last = 0;

    dx = target_x - now_x;
    dy = target_y - now_y;
    error = (int32_t)my_sqrt((double)dx * dx + (double)dy * dy);

    if (error <= 0)
    {
        *out_vx = 0;
        *out_vy = 0;
        error_last = 0;
        return;
    }

    kp_out = (int32_t)(KP * error);
    kd_out = (int32_t)((error - error_last) * KD);
    error_last = error;

    total_out = kp_out + kd_out;
    total_out = Limit_max_int(total_out, max);

    cal_vel_x = (int32_t)(total_out * (double)dx / (double)error);
    cal_vel_y = (int32_t)(total_out * (double)dy / (double)error);

    *out_vx = Limit_max_int(cal_vel_x, max);
    *out_vy = Limit_max_int(cal_vel_y, max);
}


static void calc_cam_vel(int32_t cam_x,
                         int32_t cam_y,
                         int32_t max_common,
                         int32_t *out_vx,
                         int32_t *out_vy)
{
    int32_t max_x = (CAM_X_LIMIT_CMPS < max_common) ? CAM_X_LIMIT_CMPS : max_common;
    int32_t max_y = (CAM_Y_LIMIT_CMPS < max_common) ? CAM_Y_LIMIT_CMPS : max_common;

    static int32_t last_cam_x = 0;
    static int32_t last_cam_y = 0;
    static uint8_t cam_d_inited = 0;

    int32_t err_x;
    int32_t err_y;
    int32_t d_x;
    int32_t d_y;

    int32_t p_out_x;
    int32_t p_out_y;
    int32_t d_out_x;
    int32_t d_out_y;

    if (abs_i32_pid(cam_x) <= CAM_DEADBAND_CM)
    {
        cam_x = 0;
    }
    if (abs_i32_pid(cam_y) <= CAM_DEADBAND_CM)
    {
        cam_y = 0;
    }

    err_x = CAM_X_SIGN * cam_x;
    err_y = CAM_Y_SIGN * cam_y;

    if (cam_d_inited == 0)
    {
        last_cam_x = err_x;
        last_cam_y = err_y;
        cam_d_inited = 1;
    }

    d_x = err_x - last_cam_x;
    d_y = err_y - last_cam_y;

    last_cam_x = err_x;
    last_cam_y = err_y;

    p_out_x = (int32_t)(KP_MV_X * (double)err_x);
    p_out_y = (int32_t)(KP_MV_Y * (double)err_y);

    d_out_x = (int32_t)(KD_MV_X * (double)d_x);
    d_out_y = (int32_t)(KD_MV_Y * (double)d_y);

    d_out_x = Limit_max_int(d_out_x, CAM_D_LIMIT_CMPS);
    d_out_y = Limit_max_int(d_out_y, CAM_D_LIMIT_CMPS);

    *out_vx = Limit_max_int(p_out_x + d_out_x, max_x);
    *out_vy = Limit_max_int(p_out_y + d_out_y, max_y);

    /*
     * 当二维码已经基本居中时，重置 D 项初始状态。
     * 避免下一次重新偏移时产生过大的微分突跳。
     */
    if (cam_x == 0 && cam_y == 0)
    {
        cam_d_inited = 0;
    }
}

static int32_t calc_yaw_vel(int32_t yaw_target_cmd, double yaw_now)
{
    float err_deg;
    int32_t cal_vel_yaw;

    if (yaw_mode == YAW_MODE_TURN_REL)
    {
        err_deg = (float)yaw_target_cmd - (float)yaw_now;
    }
    else
    {
        err_deg = normalize_angle((float)yaw_target_cmd - (float)yaw_now);
    }

    if (err_deg < (float)yaw_deadband && err_deg > -(float)yaw_deadband)
    {
        err_deg = 0.0f;
    }

    cal_vel_yaw = (int32_t)(KP_yaw * err_deg);
    return Limit_max_int(cal_vel_yaw, yaw_output_max);
}

/* ==========================
 * 保留原来的接口：如果别的文件调用这些函数，仍然可以正常工作。
 * 但 PID() 内部不会再用这些函数直接写全局变量。
 * ========================== */

void keep_hight(int32_t hight_target, int32_t hight)
{
    my_give_vel_z = calc_hight_vel(hight_target, hight);
}

void keep_dis(double KP, double KD, int32_t dis_x_target, int32_t dis_y_target, int32_t dis_x, int32_t dis_y, int32_t max)
{
    int32_t vx = 0;
    int32_t vy = 0;

    calc_dis_vel(KP, KD, dis_x_target, dis_y_target, dis_x, dis_y, max, &vx, &vy);
    my_give_vel_x = vx;
    my_give_vel_y = vy;
}

static void keep_cam_dis(int32_t cam_x, int32_t cam_y, int32_t max_common)
{
    int32_t vx = 0;
    int32_t vy = 0;

    calc_cam_vel(cam_x, cam_y, max_common, &vx, &vy);
    my_give_vel_x = vx;
    my_give_vel_y = vy;
}

void keep_yaw(int32_t yaw_target_cmd, double yaw_now)
{
    my_give_vel_yaw = calc_yaw_vel(yaw_target_cmd, yaw_now);
}

/*
 * 关键修改点：
 * 原来 PID() 一开始直接把 my_give_vel_x/y/z/yaw 清零。
 * 由于 ANO_LX_Task() 在 TIM7 中断中运行，它可能在 PID() 执行中途读取这些全局变量，
 * 从而把某一帧 0 速度发送给飞控。
 *
 * 现在改为：
 * 1. 使用局部变量 next_vel_x/y/z/yaw，默认值为 0；
 * 2. 根据当前控制 flag 计算本周期输出；
 * 3. 全部算完后，再一次性提交到 my_give_vel_x/y/z/yaw。
 *
 * 这样仍然保留“没有任务时输出为 0”的安全逻辑，
 * 但不会让中断读到“刚清零、还没重新计算”的中间状态。
 */
void PID(void)
{
    int32_t next_vel_x = 0;
    int32_t next_vel_y = 0;
    int32_t next_vel_z = 0;
    int32_t next_vel_yaw = 0;

    if (keep_hight_flag)
    {
        next_vel_z = calc_hight_vel(height_target, hight);
    }

    if (keep_dis_flag)
    {
        calc_dis_vel(KP_xy,
                     KD_xy,
                     dis_x_target,
                     dis_y_target,
                     dis_x,
                     dis_y,
                     g_xy_limit_optical,
                     &next_vel_x,
                     &next_vel_y);
    }

    if (keep_radar_flag)
    {
        int32_t corrected_x;
        int32_t corrected_y;

        corrected_x = get_corrected_dis_x(dis_x_slam);
        corrected_y = get_corrected_dis_y(dis_y_slam);

        calc_dis_vel(KP_radar,
                     KD_radar,
                     dis_x_target,
                     dis_y_target,
                     corrected_x,
                     corrected_y,
                     g_xy_limit_radar,
                     &next_vel_x,
                     &next_vel_y);
    }

    if (keep_cam_flag)
    {
        /* 视觉跟随改为分轴控制，避免某一轴误差过大时影响另一轴响应，
         * 同时方便单独调大前后方向增益。
         */
        calc_cam_vel(dis_x_cam_target,
                     dis_y_cam_target,
                     g_xy_limit_mv,
                     &next_vel_x,
                     &next_vel_y);
    }

    if (keep_yaw_flag)
    {
        next_vel_yaw = calc_yaw_vel(yaw_target, yaw_fix);
    }

    /*
     * 最后统一提交。
     * TIM7 中断即使在 PID() 前半段触发，也只能读到上一周期的完整速度；
     * 不会再读到 PID() 开头清零后的中间态。
     */
    my_give_vel_x = (int)next_vel_x;
    my_give_vel_y = (int)next_vel_y;
    my_give_vel_z = (int)next_vel_z;
    my_give_vel_yaw = (int)next_vel_yaw;
}
