、#include "app_motor.h"
#include "bsp_encoder.h"
#include "bsp_systick.h"

/*====================================================================================*/
/*                              速度环参数（循迹优化版）                               */
/*====================================================================================*/

#define MOTOR_SPEED_LOOP_ENABLE 1

/* 电机速度参数 */
#define MOTOR_SPEED_MAX_MMS     2100.0f     /* 最大速度 (mm/s) */
#define MOTOR_DEADZONE_L_PCT    4           /* 左轮死区 (%) */
#define MOTOR_DEADZONE_R_PCT    5           /* 右轮死区 (%) */

/* 速度环 PID 参数 - 简化版，提高响应 */
#define MOTOR_PID_KP            0.020f      /* 比例系数 - 稍微增大 */
#define MOTOR_PID_KI            0.005f      /* 积分系数 - 减小 */
#define MOTOR_PID_KD            0.002f      /* 微分系数 - 加入抑制震荡 */
#define MOTOR_PID_INTEGRAL_LIMIT 800.0f     /* 积分限幅 */
#define MOTOR_PID_OUTPUT_LIMIT  30.0f       /* 输出限幅 (%) - 放宽 */
#define MOTOR_PID_MAX_DT_MS     200U        /* 最大时间间隔 */

/* PWM 变化率限制 - 简化为单一值，提高响应 */
#define MOTOR_PWM_SLEW_RATE     8.0f        /* 统一变化率 - 放宽限制 */

/* 速度滤波系数 */
#define MOTOR_SPEED_FILTER_ALPHA 0.5f       /* 减弱滤波，提高响应 */

typedef struct {
    float kp;
    float ki;
    float kd;
    float integral;
    float prevError;
} MotorPid_t;

static MotorPid_t s_pidLeft;
static MotorPid_t s_pidRight;
static uint32_t s_lastUpdateMs = 0;

/* 速度滤波缓存 */
static float s_filteredSpeedL = 0.0f;
static float s_filteredSpeedR = 0.0f;

/* 上一次 PWM 输出（用于变化率限制） */
static float s_lastPwmL = 0.0f;
static float s_lastPwmR = 0.0f;

static int16_t Motor_Clamp(int16_t value, int16_t min, int16_t max)
{
    if (value > max) return max;
    if (value < min) return min;
    return value;
}

static float Motor_ClampFloat(float value, float min, float max)
{
    if (value > max) return max;
    if (value < min) return min;
    return value;
}

static void Motor_PidInit(MotorPid_t *pid)
{
    pid->kp = MOTOR_PID_KP;
    pid->ki = MOTOR_PID_KI;
    pid->kd = MOTOR_PID_KD;
    pid->integral = 0.0f;
    pid->prevError = 0.0f;
}

static void Motor_PidReset(MotorPid_t *pid)
{
    pid->integral = 0.0f;
    pid->prevError = 0.0f;
}

static float Motor_PidUpdate(MotorPid_t *pid, float error, float dt)
{
    float derivative;
    float output;

    if (dt < 0.0005f) dt = 0.0005f;

    pid->integral += error * dt;
    pid->integral = Motor_ClampFloat(pid->integral, -MOTOR_PID_INTEGRAL_LIMIT, MOTOR_PID_INTEGRAL_LIMIT);

    derivative = (error - pid->prevError) / dt;
    output = pid->kp * error + pid->ki * pid->integral + pid->kd * derivative;
    pid->prevError = error;

    return Motor_ClampFloat(output, -MOTOR_PID_OUTPUT_LIMIT, MOTOR_PID_OUTPUT_LIMIT);
}

static int16_t Motor_CalcSpeedOutput(float targetMms, float actualMms, MotorPid_t *pid, float dt, int isLeft)
{
    float basePwm;
    float error;
    float correction;
    float pwm;
    int deadzone;
    float absTarget;
    float *pFiltered;
    float *pLastPwm;
    float filteredSpeed;
    float lastPwm;
    float deltaPwm;

    if (MOTOR_SPEED_MAX_MMS <= 0.1f)
    {
        return 0;
    }

    /* 获取上一次 PWM */
    pLastPwm = isLeft ? &s_lastPwmL : &s_lastPwmR;
    lastPwm = *pLastPwm;

    /* 目标为0时直接返回0 */
    if (targetMms > -0.1f && targetMms < 0.1f)
    {
        Motor_PidReset(pid);
        /* 重置滤波器和上次PWM */
        if (isLeft) { s_filteredSpeedL = 0.0f; s_lastPwmL = 0.0f; }
        else { s_filteredSpeedR = 0.0f; s_lastPwmR = 0.0f; }
        return 0;
    }

    /* 速度滤波：减少编码器噪声 */
    pFiltered = isLeft ? &s_filteredSpeedL : &s_filteredSpeedR;
    *pFiltered = MOTOR_SPEED_FILTER_ALPHA * actualMms + (1.0f - MOTOR_SPEED_FILTER_ALPHA) * (*pFiltered);
    filteredSpeed = *pFiltered;

    /* 计算基础 PWM (前馈) */
    basePwm = targetMms / MOTOR_SPEED_MAX_MMS * 100.0f;
    basePwm = Motor_ClampFloat(basePwm, -100.0f, 100.0f);

    /* PID 修正 - 使用滤波后的速度 */
    error = targetMms - filteredSpeed;
    correction = Motor_PidUpdate(pid, error, dt);
    pwm = basePwm + correction;
    pwm = Motor_ClampFloat(pwm, -100.0f, 100.0f);

    /* PWM 变化率限制 - 简化版，统一限制 */
    deltaPwm = pwm - lastPwm;
    
    /* 简单的变化率限制，提高响应速度 */
    if (deltaPwm > MOTOR_PWM_SLEW_RATE) deltaPwm = MOTOR_PWM_SLEW_RATE;
    else if (deltaPwm < -MOTOR_PWM_SLEW_RATE) deltaPwm = -MOTOR_PWM_SLEW_RATE;
    
    pwm = lastPwm + deltaPwm;
    
    *pLastPwm = pwm;  /* 保存本次 PWM */

    /* 死区补偿 */
    deadzone = isLeft ? MOTOR_DEADZONE_L_PCT : MOTOR_DEADZONE_R_PCT;
    
    /* 低速时增加死区余量，帮助快速脱离死区 */
    absTarget = (targetMms > 0) ? targetMms : -targetMms;
    if (absTarget < 500.0f)
    {
        deadzone += 3;  /* 低速时多补偿一点 */
    }
    
    /* 最小 PWM 保护 */
    if (pwm > 0.5f)
    {
        float minPwm = (float)deadzone + 2.0f;
        if (pwm < minPwm) pwm = minPwm;
        return (int16_t)(pwm + 0.5f);
    }
    else if (pwm < -0.5f)
    {
        float minPwm = (float)deadzone + 2.0f;
        if (pwm > -minPwm) pwm = -minPwm;
        return (int16_t)(pwm - 0.5f);
    }
    return 0;
}

void Set_Motor(int MOTOR_TYPE)
{
    (void)MOTOR_TYPE;
    Motor_Init();
    Motor_Enable(1);
    Encoder_Init();
    Encoder_Reset();
    Motor_PidInit(&s_pidLeft);
    Motor_PidInit(&s_pidRight);
    s_lastUpdateMs = 0;
}

void Motion_Car_Control(int16_t V_x, int16_t V_y, int16_t V_z)
{
    float robot_APB = Car_APB;
    float speed_spin = (float)V_z / 1000.0f * robot_APB;
    float speed_left_cmd = (float)V_x + speed_spin;
    float speed_right_cmd = (float)V_x - speed_spin;

    (void)V_y;

    if (V_x == 0 && V_y == 0 && V_z == 0)
    {
        Motor_SetSpeedBoth(0, 0);
        Motor_PidReset(&s_pidLeft);
        Motor_PidReset(&s_pidRight);
        s_lastUpdateMs = SysTick_GetMs();
        return;
    }

    speed_left_cmd = Motor_ClampFloat(speed_left_cmd, -1000.0f, 1000.0f);
    speed_right_cmd = Motor_ClampFloat(speed_right_cmd, -1000.0f, 1000.0f);

#if MOTOR_SPEED_LOOP_ENABLE
    {
        float targetLeftMms = speed_left_cmd / 1000.0f * MOTOR_SPEED_MAX_MMS;
        float targetRightMms = speed_right_cmd / 1000.0f * MOTOR_SPEED_MAX_MMS;
        uint32_t nowMs = SysTick_GetMs();
        uint32_t deltaMs;
        float dt;
        int16_t pwmLeft;
        int16_t pwmRight;

        if (s_lastUpdateMs == 0)
        {
            s_lastUpdateMs = nowMs;
        }

        deltaMs = nowMs - s_lastUpdateMs;
        if (deltaMs == 0)
        {
            deltaMs = 1;
        }
        if (deltaMs > MOTOR_PID_MAX_DT_MS)
        {
            deltaMs = MOTOR_PID_MAX_DT_MS;
        }
        s_lastUpdateMs = nowMs;

        Encoder_Update(deltaMs);

        dt = (float)deltaMs / 1000.0f;
        pwmLeft = Motor_CalcSpeedOutput(targetLeftMms, Encoder_GetSpeedMMS(ENCODER_LEFT), &s_pidLeft, dt, 1);
        pwmRight = Motor_CalcSpeedOutput(targetRightMms, Encoder_GetSpeedMMS(ENCODER_RIGHT), &s_pidRight, dt, 0);

        Motor_SetSpeedBoth(pwmLeft, pwmRight);
    }
#else
    {
        int16_t speed_left = Motor_Clamp((int16_t)speed_left_cmd, -1000, 1000);
        int16_t speed_right = Motor_Clamp((int16_t)speed_right_cmd, -1000, 1000);

        speed_left /= 10;
        speed_right /= 10;

        Motor_SetSpeedBoth(speed_left, speed_right);
    }
#endif
}

