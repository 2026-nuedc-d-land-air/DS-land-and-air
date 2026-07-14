#include "bsp_motor_safe.h"

/*
 * Known-good wiring checkpoint 55366cd:
 * Front enable/PWM: PA2, PA3; TB6612 STBY: PE6.
 * Front direction: PE2, PE3, PE4, PE5.
 * Rear enable/PWM: PA6, PA7.
 * Rear direction: PC6, PG1, PE9, PE7.
 */

void MotorSafe_ForceOff(void)
{
    GPIO_ResetBits(GPIOA, GPIO_Pin_2 | GPIO_Pin_3 |
                          GPIO_Pin_6 | GPIO_Pin_7);
    GPIO_ResetBits(GPIOE, GPIO_Pin_2 | GPIO_Pin_3 |
                          GPIO_Pin_4 | GPIO_Pin_5 |
                          GPIO_Pin_6 | GPIO_Pin_7 |
                          GPIO_Pin_9);
    GPIO_ResetBits(GPIOC, GPIO_Pin_6);
    GPIO_ResetBits(GPIOG, GPIO_Pin_1);
}

void MotorSafe_InitOff(void)
{
    GPIO_InitTypeDef gpio;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA |
                           RCC_APB2Periph_GPIOC |
                           RCC_APB2Periph_GPIOE |
                           RCC_APB2Periph_GPIOG, ENABLE);

    MotorSafe_ForceOff();

    GPIO_StructInit(&gpio);
    gpio.GPIO_Mode = GPIO_Mode_Out_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;

    gpio.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3 |
                    GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_Init(GPIOA, &gpio);

    gpio.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3 |
                    GPIO_Pin_4 | GPIO_Pin_5 |
                    GPIO_Pin_6 | GPIO_Pin_7 |
                    GPIO_Pin_9;
    GPIO_Init(GPIOE, &gpio);

    gpio.GPIO_Pin = GPIO_Pin_6;
    GPIO_Init(GPIOC, &gpio);

    gpio.GPIO_Pin = GPIO_Pin_1;
    GPIO_Init(GPIOG, &gpio);

    MotorSafe_ForceOff();
}
