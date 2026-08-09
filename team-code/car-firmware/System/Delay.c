#include "stm32f10x.h"

static uint32_t s_ticksPerUs = 0U;

static void Delay_UpdateClock(void)
{
	SystemCoreClockUpdate();
	s_ticksPerUs = SystemCoreClock / 1000000U;
	if (s_ticksPerUs == 0U)
	{
		s_ticksPerUs = 1U;
	}
}

/**
  * @brief  微秒级延时
  * @param  xus 延时时长，范围：0~233015
  * @retval 无
  */
void Delay_us(uint32_t xus)
{
	uint32_t maxUs;
	uint32_t chunkUs;
	uint32_t ticks;

	if (s_ticksPerUs == 0U)
	{
		Delay_UpdateClock();
	}

	maxUs = 0x01000000U / s_ticksPerUs;
	if (maxUs == 0U)
	{
		maxUs = 1U;
	}

	while (xus != 0U)
	{
		chunkUs = (xus > maxUs) ? maxUs : xus;
		ticks = chunkUs * s_ticksPerUs;

		SysTick->LOAD = ticks - 1U;
		SysTick->VAL = 0U;
		SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk;
		while ((SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) == 0U)
		{
		}
		SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk;
		xus -= chunkUs;
	}
}

/**
  * @brief  毫秒级延时
  * @param  xms 延时时长，范围：0~4294967295
  * @retval 无
  */
void Delay_ms(uint32_t xms)
{
	while(xms--)
	{
		Delay_us(1000);
	}
}
 
/**
  * @brief  秒级延时
  * @param  xs 延时时长，范围：0~4294967295
  * @retval 无
  */
void Delay_s(uint32_t xs)
{
	while(xs--)
	{
		Delay_ms(1000);
	}
} 

/**
  * @brief  兼容性函数：初始化延时函数
  */
void delay_init(void)
{
	Delay_UpdateClock();
}

/**
  * @brief  兼容性函数：微秒级延时
  */
void delay_us(uint32_t us)
{
	Delay_us(us);
}

/**
  * @brief  兼容性函数：毫秒级延时
  */
void delay_ms(uint32_t ms)
{
	Delay_ms(ms);
}
