#include "bsp.h"
void BSP_init(void)
{
	SystemInit();
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); 
	
	
	delay_init();
//	delay_ms(1000); //�ȴ������ȶ�    Wait for infrared to stabilize
	
	
	USART1_init(115200);
	USART2_init(115200);//ʹ�ô���2 ���պ���    Use serial port 2 to receive infrared
	
	Motor_Init();
	Motor_Enable(1);

	//�ŵ�������Ч����Ȼ�����޷�����ʹ��    It will take effect at the end, otherwise it will not work properly.
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);//����jlink ֻ��SWD���Կڣ�PA15��PB3��4����ͨIO  Disable jlink and use only SWD debug port, PA15, PB3, 4 as normal IO
	
	
}
