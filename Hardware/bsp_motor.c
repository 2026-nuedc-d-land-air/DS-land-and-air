/**
  * @file    bsp_motor.c
  * @brief   电机驱动模块实现 - TB6612FNG驱动 + MG310电机
  * @note    使用TIM2产生PWM信号
  *          PWMA (左电机) - PA2 (TIM2_CH3)
  *          PWMB (右电机) - PA3 (TIM2_CH4)
  */

#include "bsp_motor.h"

/*====================================================================================*/
/*                                  私有变量                                           */
/*====================================================================================*/

static uint8_t s_motorEnabled = 0;

/*====================================================================================*/
/*                                  内部函数                                           */
/*====================================================================================*/

/**
  * @brief  设置左电机方向
  */
static void Motor_SetDirLeft(Motor_Dir_t dir)
{
    switch (dir)
    {
        case MOTOR_DIR_FORWARD:
            /* Verified physical forward: PE2=0, PE3=1. */
            GPIO_ResetBits(MOTOR_DIR_GPIO_PORT, MOTOR_AIN1_PIN);
            GPIO_SetBits(MOTOR_DIR_GPIO_PORT, MOTOR_AIN2_PIN);
            break;
        case MOTOR_DIR_BACKWARD:
            GPIO_SetBits(MOTOR_DIR_GPIO_PORT, MOTOR_AIN1_PIN);
            GPIO_ResetBits(MOTOR_DIR_GPIO_PORT, MOTOR_AIN2_PIN);
            break;
        case MOTOR_DIR_BRAKE:
            GPIO_SetBits(MOTOR_DIR_GPIO_PORT, MOTOR_AIN1_PIN);
            GPIO_SetBits(MOTOR_DIR_GPIO_PORT, MOTOR_AIN2_PIN);
            break;
    }
}

/**
  * @brief  设置右电机方向
  */
static void Motor_SetDirRight(Motor_Dir_t dir)
{
    switch (dir)
    {
        case MOTOR_DIR_FORWARD:
            GPIO_SetBits(MOTOR_DIR_GPIO_PORT, MOTOR_BIN1_PIN);
            GPIO_ResetBits(MOTOR_DIR_GPIO_PORT, MOTOR_BIN2_PIN);
            break;
        case MOTOR_DIR_BACKWARD:
            GPIO_ResetBits(MOTOR_DIR_GPIO_PORT, MOTOR_BIN1_PIN);
            GPIO_SetBits(MOTOR_DIR_GPIO_PORT, MOTOR_BIN2_PIN);
            break;
        case MOTOR_DIR_BRAKE:
            GPIO_SetBits(MOTOR_DIR_GPIO_PORT, MOTOR_BIN1_PIN);
            GPIO_SetBits(MOTOR_DIR_GPIO_PORT, MOTOR_BIN2_PIN);
            break;
    }
}

/**
  * @brief  设置左电机PWM值
  */
static void Motor_SetPWMLeft(uint16_t pwm)
{
    if (pwm > MOTOR_PWM_MAX) pwm = MOTOR_PWM_MAX;
    TIM_SetCompare3(MOTOR_TIM, pwm);
}

/**
  * @brief  设置右电机PWM值
  */
static void Motor_SetPWMRight(uint16_t pwm)
{
    if (pwm > MOTOR_PWM_MAX) pwm = MOTOR_PWM_MAX;
    TIM_SetCompare4(MOTOR_TIM, pwm);
}

/*====================================================================================*/
/*                                  接口函数                                           */
/*====================================================================================*/

/**
  * @brief  电机驱动初始化
  */
void Motor_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_OCInitTypeDef TIM_OCInitStructure;
    
    /* 使能时钟 */
    RCC_APB1PeriphClockCmd(MOTOR_TIM_CLK, ENABLE);
    RCC_APB2PeriphClockCmd(MOTOR_PWM_GPIO_CLK | MOTOR_DIR_GPIO_CLK |
                           RCC_APB2Periph_AFIO, ENABLE);
    
    /* 配置PWM引脚 - PA2, PA3 复用推挽输出 */
    GPIO_InitStructure.GPIO_Pin = MOTOR_PWMA_PIN | MOTOR_PWMB_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(MOTOR_PWM_GPIO_PORT, &GPIO_InitStructure);
    
    /* 配置方向控制引脚 - PE2-PE6 推挽输出 */
    GPIO_InitStructure.GPIO_Pin = MOTOR_AIN1_PIN | MOTOR_AIN2_PIN | 
                                   MOTOR_BIN1_PIN | MOTOR_BIN2_PIN | MOTOR_STBY_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(MOTOR_DIR_GPIO_PORT, &GPIO_InitStructure);
    
    /* 初始状态：待机 */
    GPIO_ResetBits(MOTOR_DIR_GPIO_PORT, MOTOR_STBY_PIN);
    GPIO_ResetBits(MOTOR_DIR_GPIO_PORT, MOTOR_AIN1_PIN | MOTOR_AIN2_PIN);
    GPIO_ResetBits(MOTOR_DIR_GPIO_PORT, MOTOR_BIN1_PIN | MOTOR_BIN2_PIN);
    
    /* TIM2时基配置 - 20kHz PWM */
    /* TIM2时钟 = 72MHz (APB1=36MHz, x2=72MHz) */
    TIM_TimeBaseStructure.TIM_Period = MOTOR_PWM_PERIOD - 1;
    TIM_TimeBaseStructure.TIM_Prescaler = 0;    /* 不分频 */
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(MOTOR_TIM, &TIM_TimeBaseStructure);
    
    /* TIM2_CH3 PWM配置 (左电机) */
    TIM_OCStructInit(&TIM_OCInitStructure);
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = 0;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OC3Init(MOTOR_TIM, &TIM_OCInitStructure);
    TIM_OC3PreloadConfig(MOTOR_TIM, TIM_OCPreload_Enable);
    
    /* TIM2_CH4 PWM配置 (右电机) */
    TIM_OC4Init(MOTOR_TIM, &TIM_OCInitStructure);
    TIM_OC4PreloadConfig(MOTOR_TIM, TIM_OCPreload_Enable);
    
    /* 使能ARR预装载 */
    TIM_ARRPreloadConfig(MOTOR_TIM, ENABLE);
    
    /* 使能定时器 */
    TIM_Cmd(MOTOR_TIM, ENABLE);
    
    s_motorEnabled = 0;
}

/**
  * @brief  设置电机速度
  * @param  motor: MOTOR_LEFT 或 MOTOR_RIGHT
  * @param  speed: 速度值 (-100 ~ +100)
  */
void Motor_SetSpeed(Motor_Index_t motor, int16_t speed)
{
    uint16_t pwm;
    Motor_Dir_t dir;
    
    /* 限幅 */
    if (speed > 100) speed = 100;
    if (speed < -100) speed = -100;
    
    /* 确定方向和PWM值 */
    if (speed >= 0)
    {
        dir = MOTOR_DIR_FORWARD;
        pwm = (uint16_t)(speed * MOTOR_PWM_MAX / 100);
    }
    else
    {
        dir = MOTOR_DIR_BACKWARD;
        pwm = (uint16_t)((-speed) * MOTOR_PWM_MAX / 100);
    }
    
    /* 设置电机 */
    if (motor == MOTOR_LEFT)
    {
        Motor_SetDirLeft(dir);
        Motor_SetPWMLeft(pwm);
    }
    else
    {
        Motor_SetDirRight(dir);
        Motor_SetPWMRight(pwm);
    }
}

/**
  * @brief  设置双电机速度
  */
void Motor_SetSpeedBoth(int16_t speedL, int16_t speedR)
{
    Motor_SetSpeed(MOTOR_LEFT, speedL);
    Motor_SetSpeed(MOTOR_RIGHT, speedR);
}

/**
  * @brief  电机刹车
  */
void Motor_Brake(Motor_Index_t motor)
{
    if (motor == MOTOR_LEFT)
    {
        Motor_SetDirLeft(MOTOR_DIR_BRAKE);
        Motor_SetPWMLeft(MOTOR_PWM_MAX);
    }
    else
    {
        Motor_SetDirRight(MOTOR_DIR_BRAKE);
        Motor_SetPWMRight(MOTOR_PWM_MAX);
    }
}

/**
  * @brief  双电机刹车
  */
void Motor_BrakeBoth(void)
{
    Motor_Brake(MOTOR_LEFT);
    Motor_Brake(MOTOR_RIGHT);
}

/**
  * @brief  电机滑行
  */
void Motor_Coast(Motor_Index_t motor)
{
    if (motor == MOTOR_LEFT)
    {
        GPIO_ResetBits(MOTOR_DIR_GPIO_PORT, MOTOR_AIN1_PIN | MOTOR_AIN2_PIN);
        Motor_SetPWMLeft(0);
    }
    else
    {
        GPIO_ResetBits(MOTOR_DIR_GPIO_PORT, MOTOR_BIN1_PIN | MOTOR_BIN2_PIN);
        Motor_SetPWMRight(0);
    }
}

/**
  * @brief  使能/禁用电机驱动
  */
void Motor_Enable(uint8_t enable)
{
    if (enable)
    {
        GPIO_SetBits(MOTOR_DIR_GPIO_PORT, MOTOR_STBY_PIN);
        s_motorEnabled = 1;
    }
    else
    {
        GPIO_ResetBits(MOTOR_DIR_GPIO_PORT, MOTOR_STBY_PIN);
        s_motorEnabled = 0;
    }
}

/**
  * @brief  获取电机是否使能
  */
uint8_t Motor_IsEnabled(void)
{
    return s_motorEnabled;
}
