#include "bsp_motor_safe.h"

/*
 * Front PWM: PA2, PA3; front direction/STBY: PE2-PE6.
 * Rear PWM: PE13, PE14; rear direction: PF1-PF4; rear STBY: PB9.
 */

void MotorSafe_ForceOff(void)
{
    GPIO_ResetBits(GPIOA, GPIO_Pin_2 | GPIO_Pin_3);
    GPIO_ResetBits(GPIOB, GPIO_Pin_9);
    GPIO_ResetBits(GPIOE, GPIO_Pin_2 | GPIO_Pin_3 |
                          GPIO_Pin_4 | GPIO_Pin_5 |
                          GPIO_Pin_6 | GPIO_Pin_13 | GPIO_Pin_14);
    GPIO_ResetBits(GPIOF, GPIO_Pin_1 | GPIO_Pin_2 |
                          GPIO_Pin_3 | GPIO_Pin_4);
}

void MotorSafe_InitOff(void)
{
    GPIO_InitTypeDef gpio;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA |
                           RCC_APB2Periph_GPIOB |
                           RCC_APB2Periph_GPIOE |
                           RCC_APB2Periph_GPIOF, ENABLE);

    MotorSafe_ForceOff();

    GPIO_StructInit(&gpio);
    gpio.GPIO_Mode = GPIO_Mode_Out_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;

    gpio.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3;
    GPIO_Init(GPIOA, &gpio);

    gpio.GPIO_Pin = GPIO_Pin_9;
    GPIO_Init(GPIOB, &gpio);

    gpio.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3 |
                    GPIO_Pin_4 | GPIO_Pin_5 |
                    GPIO_Pin_6 | GPIO_Pin_13 | GPIO_Pin_14;
    GPIO_Init(GPIOE, &gpio);

    gpio.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_2 |
                    GPIO_Pin_3 | GPIO_Pin_4;
    GPIO_Init(GPIOF, &gpio);

    MotorSafe_ForceOff();
}
