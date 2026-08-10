#include "bsp_gray_tracking.h"

#define GRAY_AD0_PIN          GPIO_Pin_0  /* PC0:  74HC4051 address bit 0 */
#define GRAY_AD1_PIN          GPIO_Pin_1  /* PC1:  74HC4051 address bit 1 */
#define GRAY_AD2_PIN          GPIO_Pin_2  /* PC2:  74HC4051 address bit 2 */
#define GRAY_OUT_PIN          GPIO_Pin_0  /* PG0:  selected channel output */
#define GRAY_ADDRESS_MASK     (GRAY_AD0_PIN | GRAY_AD1_PIN | GRAY_AD2_PIN)
#define GRAY_SETTLE_NOPS      24U

static void GrayTracking_SelectChannel(uint8_t channel)
{
    uint8_t index;

    GPIO_WriteBit(GPIOC, GRAY_AD0_PIN,
                  ((channel & 0x01U) != 0U) ? Bit_SET : Bit_RESET);
    GPIO_WriteBit(GPIOC, GRAY_AD1_PIN,
                  ((channel & 0x02U) != 0U) ? Bit_SET : Bit_RESET);
    GPIO_WriteBit(GPIOC, GRAY_AD2_PIN,
                  ((channel & 0x04U) != 0U) ? Bit_SET : Bit_RESET);

    /* 74HC4051 switching is sub-microsecond. Do not use SysTick Delay_us(),
     * because the application owns SysTick for its millisecond timebase. */
    for (index = 0U; index < GRAY_SETTLE_NOPS; ++index)
    {
        __NOP();
    }
}

void GrayTracking_Init(void)
{
    GPIO_InitTypeDef gpio;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC |
                           RCC_APB2Periph_GPIOG, ENABLE);

    GPIO_StructInit(&gpio);
    gpio.GPIO_Pin = GRAY_ADDRESS_MASK;
    gpio.GPIO_Mode = GPIO_Mode_Out_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &gpio);
    GPIO_ResetBits(GPIOC, GRAY_ADDRESS_MASK); /* channel 1 selected at reset */

    gpio.GPIO_Pin = GRAY_OUT_PIN;
    gpio.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOG, &gpio);
}

void GrayTracking_Read(GrayTrackingSample_t *sample)
{
    uint8_t channel;
    uint8_t mask = 0U;

    if (sample == 0)
    {
        return;
    }

    for (channel = 0U; channel < 8U; ++channel)
    {
        GrayTracking_SelectChannel(channel);
        if (GPIO_ReadInputDataBit(GPIOG, GRAY_OUT_PIN) != Bit_RESET)
        {
            mask |= (uint8_t)(1U << channel);
        }
    }

    sample->rawMask = mask;
}
