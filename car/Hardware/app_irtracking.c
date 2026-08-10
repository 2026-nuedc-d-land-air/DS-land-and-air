#include "app_irtracking.h"

/*====================================================================================*/
/*                              【调参记录表】                                         */
/*====================================================================================*/
/*
 * 调参顺序：BIAS → KP → KD → 速度 → (KI一般不用)
 * 
 * 【历史记录】每次调参后记录在这里，方便回溯：
 * ------------------------------------------------------------------
 * 日期       | BIAS | KP_直线 | KD_直线 | KP_转弯 | KD_转弯 | 速度  | 效果
 * ------------------------------------------------------------------
 * 2026-01-31 |  12  |   35    |   90    |   70    |   40    | 350/250 | 初始值
 * 
 */

/*====================================================================================*/
/*                              循迹PID参数                                           */
/*====================================================================================*/

/* ===== 第一步：机械补偿（先调这个！）===== */
/* 直线偏左加大，偏右减小，每次调整5-10 */
#define MOTOR_BIAS_OFFSET       25

/* ===== 第二步：直线PID ===== */
#define IRTrack_KP_STRAIGHT     40.0f    /* 太小跟不上，太大会抖 */
#define IRTrack_KI_STRAIGHT     0.0f     /* 一般为0 */
#define IRTrack_KD_STRAIGHT     0.0f    /* 消除抖动 */

/* ===== 第三步：转弯PID ===== */
#define IRTrack_KP_TURN         80.0f    /* 太小转不过来 */
#define IRTrack_KI_TURN         0.0f     /* 一般为0 */
#define IRTrack_KD_TURN         40.0f    /* 防止转过头 */

/* ===== 第四步：速度 ===== */
#define IRR_SPEED_STRAIGHT      400      /* 直线速度 */
#define IRR_SPEED_TURN          250      /* 转弯速度 */

/* ===== 其他参数 ===== */
#define IRTrack_OUTPUT_MAX      800.0f
#define IRTrack_OUTPUT_MIN     -800.0f
#define ERROR_TURN_THRESHOLD    5

int pid_output_IRR = 0;
u8 trun_flag = 0;

/* PID状态 */
static float s_lastError = 0.0f;
static float s_integral = 0.0f;
static int8_t s_lastValidError = 0;  /* 记录上次有效误差，用于丢线恢复 */

/* 低通滤波器 - 平滑误差变化 */
#define ERROR_FILTER_ALPHA  0.7f
static float s_filteredError = 0.0f;

float PID_IR_Calc(int8_t raw_error, int is_turn_mode)
{
    float kp, ki, kd;
    float error;
    float derivative;
    float output;
    
    /* 低通滤波平滑误差 */
    s_filteredError = ERROR_FILTER_ALPHA * (float)raw_error + (1.0f - ERROR_FILTER_ALPHA) * s_filteredError;
    error = s_filteredError;
    
    /* 根据模式选择PID参数 */
    if (is_turn_mode) {
        kp = IRTrack_KP_TURN;
        ki = IRTrack_KI_TURN;
        kd = IRTrack_KD_TURN;
    } else {
        kp = IRTrack_KP_STRAIGHT;
        ki = IRTrack_KI_STRAIGHT;
        kd = IRTrack_KD_STRAIGHT;
    }
    
    /* 积分项（一般不用） */
    s_integral += error;
    if (s_integral > 500.0f) s_integral = 500.0f;
    if (s_integral < -500.0f) s_integral = -500.0f;
    
    /* 微分项 - 关键！抑制震荡 */
    derivative = error - s_lastError;
    s_lastError = error;
    
    /* PID计算 */
    output = kp * error + ki * s_integral + kd * derivative;
    
    /* 输出限幅 */
    if (output > IRTrack_OUTPUT_MAX) output = IRTrack_OUTPUT_MAX;
    if (output < IRTrack_OUTPUT_MIN) output = IRTrack_OUTPUT_MIN;
    
    return output;
}

//x1-x8 从左到右   x1-x8 count from left to right
void LineWalking(void)
{
    int is_turn_mode = 0;
    int speed = IRR_SPEED_STRAIGHT;
    u8 x1,x2,x3,x4,x5,x6,x7,x8;
    uint8_t sensor = Tracking_Read();
    
    /* 传感器状态：0=检测到黑线，1=白底 */
    x1 = (sensor >> 7) & 0x01;
    x2 = (sensor >> 6) & 0x01;
    x3 = (sensor >> 5) & 0x01;
    x4 = (sensor >> 4) & 0x01;
    x5 = (sensor >> 3) & 0x01;
    x6 = (sensor >> 2) & 0x01;
    x7 = (sensor >> 1) & 0x01;
    x8 = (sensor >> 0) & 0x01;
    
    /*====================================================================================*/
    /*                    加权平均法计算误差 - 业界标准方法                                 */
    /*====================================================================================*/
    /*
     * 原理：给每个传感器分配位置权重，计算加权平均得到连续误差值
     * 权重分配（从左到右）：-35, -25, -15, -5, +5, +15, +25, +35
     * 检测到黑线的传感器参与计算（状态=0时参与）
     * 误差 = Σ(权重 × 是否检测到) / 检测到的传感器数量
     */
    {
        /* 权重定义 - 可以根据传感器物理间距调整 */
        const int weights[8] = {-35, -25, -15, -5, 5, 15, 25, 35};
        
        /* 传感器状态取反：1=检测到黑线，0=白底（方便计算） */
        int s[8];
        s[0] = (x1 == 0) ? 1 : 0;
        s[1] = (x2 == 0) ? 1 : 0;
        s[2] = (x3 == 0) ? 1 : 0;
        s[3] = (x4 == 0) ? 1 : 0;
        s[4] = (x5 == 0) ? 1 : 0;
        s[5] = (x6 == 0) ? 1 : 0;
        s[6] = (x7 == 0) ? 1 : 0;
        s[7] = (x8 == 0) ? 1 : 0;
        
        /* 计算加权和与检测数量 */
        int weighted_sum = 0;
        int count = 0;
        int i;
        float error;
        
        for (i = 0; i < 8; i++) {
            weighted_sum += s[i] * weights[i];
            count += s[i];
        }
        
        /*==================== 误差计算 ====================*/
        if (count > 0) {
            /* 正常情况：计算加权平均 */
            error = (float)weighted_sum / (float)count;
            s_lastValidError = (int8_t)error;  /* 保存有效误差 */
        } else {
            /* 全白/丢线：使用上次误差方向，加大转向找线 */
            if (s_lastValidError > 0) {
                error = 40.0f;   /* 上次偏右，继续右转找线 */
            } else if (s_lastValidError < 0) {
                error = -40.0f;  /* 上次偏左，继续左转找线 */
            } else {
                error = 0.0f;    /* 不知道方向，保持直行 */
            }
            is_turn_mode = 1;
        }
        
        /*==================== 判断转弯模式 ====================*/
        /* 误差绝对值大于阈值 = 转弯模式 */
        if (error > ERROR_TURN_THRESHOLD || error < -ERROR_TURN_THRESHOLD) {
            is_turn_mode = 1;
        }
        
        /* 边缘传感器触发 = 强制转弯模式 */
        if (s[0] || s[7]) {
            is_turn_mode = 1;
        }
        
        /*==================== 速度控制 ====================*/
        speed = is_turn_mode ? IRR_SPEED_TURN : IRR_SPEED_STRAIGHT;
        
        /*==================== PID计算 ====================*/
        pid_output_IRR = (int)(PID_IR_Calc((int8_t)error, is_turn_mode));
        
        /*==================== 电机控制 ====================*/
        /* 加入基础偏差补偿：解决左右电机物理特性不一致导致的直线偏移 */
        Motion_Car_Control(speed, 0, pid_output_IRR + MOTOR_BIAS_OFFSET);
    }
}
#define PWM_OFFSET_L        0.5f        /* 左轮偏移 */
#define PWM_OFFSET_R        6.5f       /* 右轮偏移 - 偏左就加，偏右就减 */
