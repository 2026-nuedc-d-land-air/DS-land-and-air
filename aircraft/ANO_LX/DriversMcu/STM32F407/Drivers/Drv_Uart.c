/******************** (C) COPYRIGHT 2017 ANO Tech ********************************
 * ����    �������ƴ�
 * ����    ��www.anotc.com
 * �Ա�    ��anotc.taobao.com
 * ����QȺ ��190169595
 * ����    ����������
 **********************************************************************************/
#include "Drv_Uart.h"
#include "Ano_DT_LX.h"
#include "Drv_UbloxGPS.h"
#include "Drv_AnoOf.h"
#include "my_uart.h"

void NoUse(u8 data) {}
// ���ڽ��շ��Ϳ��ٶ��壬ֱ���޸Ĵ˴��ĺ������ƺ꣬�޸ĳ��Լ��Ĵ��ڽ����ͷ��ͺ������Ƽ��ɣ�ע�⺯��������ʽ��ͳһ
#define U1GetOneByte MY_uart_esp_receive     // �ݶ�esp���ڣ���˫����
#define U2GetOneByte MY_uart_radar_receive   // �ݶ�Ϊ�״���������
#define U3GetOneByte MY_uart_maixcam_receive // �ݶ�mv��������
#define U4GetOneByte AnoOF_GetOneByte
#define U5GetOneByte ANO_DT_LX_Data_Receive_Prepare

//====uart1
void DrvUart1Init(u32 br_num)
{
    USART_InitTypeDef USART_InitStructure;
    USART_ClockInitTypeDef USART_ClockInitStruct;
    NVIC_InitTypeDef NVIC_InitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;

    USART_StructInit(&USART_InitStructure);

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE); // ����USART1ʱ��
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

    // �����ж����ȼ�
    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = NVIC_UART1_P;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = NVIC_UART1_S;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_USART1);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_USART1);

    // ����PA9��ΪUSART1��Tx
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    // ����PA10��ΪUSART1��Rx
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    //
    USART_DeInit(USART1);
    // ����USART1
    // �жϱ�������
    USART_InitStructure.USART_BaudRate = br_num;                                    // �����ʿ���ͨ������վ����
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;                     // 8λ����
    USART_InitStructure.USART_StopBits = USART_StopBits_1;                          // ��֡��β����1��ֹͣλ
    USART_InitStructure.USART_Parity = USART_Parity_No;                             // ������żУ��
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; // Ӳ��������ʧ��
    USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;                 // ���͡�����ʹ��
    // ����USART1ʱ��
    USART_ClockInitStruct.USART_Clock = USART_Clock_Disable;     // ʱ�ӵ͵�ƽ�
    USART_ClockInitStruct.USART_CPOL = USART_CPOL_Low;           // SLCK������ʱ������ļ���->�͵�ƽ
    USART_ClockInitStruct.USART_CPHA = USART_CPHA_2Edge;         // ʱ�ӵڶ������ؽ������ݲ���
    USART_ClockInitStruct.USART_LastBit = USART_LastBit_Disable; // ���һλ���ݵ�ʱ�����岻��SCLK���

    USART_Init(USART1, &USART_InitStructure);
    USART_ClockInit(USART1, &USART_ClockInitStruct);

    // ʹ��USART1�����ж�
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    // ʹ��USART1
    USART_Cmd(USART1, ENABLE);
}

u8 Tx1Buffer[256];
u8 Tx1Counter = 0;
u8 count1 = 0;
void DrvUart1SendBuf(unsigned char *DataToSend, u8 data_num)
{
    u8 i;
    for (i = 0; i < data_num; i++)
    {
        Tx1Buffer[count1++] = *(DataToSend + i);
    }

    if (!(USART1->CR1 & USART_CR1_TXEIE))
    {
        USART_ITConfig(USART1, USART_IT_TXE, ENABLE); // �򿪷����ж�
    }
}
u8 U1RxDataTmp[100];
u8 U1RxInCnt = 0;
u8 U1RxoutCnt = 0;
void drvU1GetByte(u8 data)
{
    U1RxDataTmp[U1RxInCnt++] = data;
    if (U1RxInCnt >= 100)
        U1RxInCnt = 0;
}
void drvU1DataCheck(void)
{
    while (U1RxInCnt != U1RxoutCnt)
    {
        U1GetOneByte(U1RxDataTmp[U1RxoutCnt++]);
        if (U1RxoutCnt >= 100)
            U1RxoutCnt = 0;
    }
}
void Usart1_IRQ(void)
{
    u8 com_data;

    if (USART1->SR & USART_SR_ORE) // ORE�ж�
    {
        com_data = USART1->DR;
    }
    // �����ж�
    if (USART_GetITStatus(USART1, USART_IT_RXNE))
    {
        USART_ClearITPendingBit(USART1, USART_IT_RXNE); // ����жϱ�־
        com_data = USART1->DR;
        drvU1GetByte(com_data);
    }
    // ���ͣ�������λ���ж�
    if (USART_GetITStatus(USART1, USART_IT_TXE))
    {
        USART1->DR = Tx1Buffer[Tx1Counter++]; // дDR����жϱ�־
        if (Tx1Counter == count1)
        {
            USART1->CR1 &= ~USART_CR1_TXEIE; // �ر�TXE�������жϣ��ж�
        }
    }
}

//====uart2
void DrvUart2Init(u32 br_num)
{
    USART_InitTypeDef USART_InitStructure;
    USART_ClockInitTypeDef USART_ClockInitStruct;
    NVIC_InitTypeDef NVIC_InitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;

    USART_StructInit(&USART_InitStructure);

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE); // ����USART2ʱ��
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

    // �����ж����ȼ�
    NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = NVIC_UART2_P;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = NVIC_UART2_S;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    GPIO_PinAFConfig(GPIOD, GPIO_PinSource5, GPIO_AF_USART2);
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource6, GPIO_AF_USART2);

    // ����PD5��ΪUSART2��Tx
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOD, &GPIO_InitStructure);
    // ����PD6��ΪUSART2��Rx
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOD, &GPIO_InitStructure);

    // ����USART2
    // �жϱ�������
    USART_InitStructure.USART_BaudRate = br_num;                                    // �����ʿ���ͨ������վ����
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;                     // 8λ����
    USART_InitStructure.USART_StopBits = USART_StopBits_1;                          // ��֡��β����1��ֹͣλ
    USART_InitStructure.USART_Parity = USART_Parity_No;                             // ������żУ��
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; // Ӳ��������ʧ��
    USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;                 // ���͡�����ʹ��
    // ����USART2ʱ��
    USART_ClockInitStruct.USART_Clock = USART_Clock_Disable;     // ʱ�ӵ͵�ƽ�
    USART_ClockInitStruct.USART_CPOL = USART_CPOL_Low;           // SLCK������ʱ������ļ���->�͵�ƽ
    USART_ClockInitStruct.USART_CPHA = USART_CPHA_2Edge;         // ʱ�ӵڶ������ؽ������ݲ���
    USART_ClockInitStruct.USART_LastBit = USART_LastBit_Disable; // ���һλ���ݵ�ʱ�����岻��SCLK���

    USART_Init(USART2, &USART_InitStructure);
    USART_ClockInit(USART2, &USART_ClockInitStruct);

    // ʹ��USART2�����ж�
    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
    // ʹ��USART2
    USART_Cmd(USART2, ENABLE);
}

u8 TxBuffer[256];
u8 TxCounter = 0;
u8 count = 0;
void DrvUart2SendBuf(unsigned char *DataToSend, u8 data_num)
{
    u8 i;
    for (i = 0; i < data_num; i++)
    {
        TxBuffer[count++] = *(DataToSend + i);
    }

    if (!(USART2->CR1 & USART_CR1_TXEIE))
    {
        USART_ITConfig(USART2, USART_IT_TXE, ENABLE); // �򿪷����ж�
    }
}
u8 U2RxDataTmp[100];
u8 U2RxInCnt = 0;
u8 U2RxoutCnt = 0;
void drvU2GetByte(u8 data)
{
    U2RxDataTmp[U2RxInCnt++] = data;
    if (U2RxInCnt >= 100)
        U2RxInCnt = 0;
}
void drvU2DataCheck(void)
{
    while (U2RxInCnt != U2RxoutCnt)
    {
        U2GetOneByte(U2RxDataTmp[U2RxoutCnt++]);
        if (U2RxoutCnt >= 100)
            U2RxoutCnt = 0;
    }
}
void Usart2_IRQ(void)
{
    u8 com_data;

    if (USART2->SR & USART_SR_ORE) // ORE�ж�
    {
        com_data = USART2->DR;
    }

    // �����ж�
    if (USART_GetITStatus(USART2, USART_IT_RXNE))
    {
        USART_ClearITPendingBit(USART2, USART_IT_RXNE); // ����жϱ�־
        com_data = USART2->DR;
        drvU2GetByte(com_data);
    }
    // ���ͣ�������λ���ж�
    if (USART_GetITStatus(USART2, USART_IT_TXE))
    {
        USART2->DR = TxBuffer[TxCounter++]; // дDR����жϱ�־
        if (TxCounter == count)
        {
            USART2->CR1 &= ~USART_CR1_TXEIE; // �ر�TXE�������жϣ��ж�
        }
    }
}

//====uart3
/////////////////////////////////////////////////////////////////////////////////////////////////////////////
void DrvUart3Init(u32 br_num)
{
    USART_InitTypeDef USART_InitStructure;
    USART_ClockInitTypeDef USART_ClockInitStruct;
    NVIC_InitTypeDef NVIC_InitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;

    USART_StructInit(&USART_InitStructure);

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE); // ����USART2ʱ��
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

    // �����ж����ȼ�
    NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_USART3);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource11, GPIO_AF_USART3);

    // ����PD5��ΪUSART2��Tx
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    // ����PD6��ΪUSART2��Rx
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    // ����USART2
    // �жϱ�������
    USART_InitStructure.USART_BaudRate = br_num;                                    // �����ʿ���ͨ������վ����
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;                     // 8λ����
    USART_InitStructure.USART_StopBits = USART_StopBits_1;                          // ��֡��β����1��ֹͣλ
    USART_InitStructure.USART_Parity = USART_Parity_No;                             // ������żУ��
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; // Ӳ��������ʧ��
    USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;                 // ���͡�����ʹ��
    // ����USART2ʱ��
    USART_ClockInitStruct.USART_Clock = USART_Clock_Disable;     // ʱ�ӵ͵�ƽ�
    USART_ClockInitStruct.USART_CPOL = USART_CPOL_Low;           // SLCK������ʱ������ļ���->�͵�ƽ
    USART_ClockInitStruct.USART_CPHA = USART_CPHA_2Edge;         // ʱ�ӵڶ������ؽ������ݲ���
    USART_ClockInitStruct.USART_LastBit = USART_LastBit_Disable; // ���һλ���ݵ�ʱ�����岻��SCLK���

    USART_Init(USART3, &USART_InitStructure);
    USART_ClockInit(USART3, &USART_ClockInitStruct);

    // ʹ��USART2�����ж�
    USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
    // ʹ��USART2
    USART_Cmd(USART3, ENABLE);
}

u8 Tx3Buffer[256];
u8 Tx3Counter = 0;
u8 count3 = 0;
void DrvUart3SendBuf(unsigned char *DataToSend, u8 data_num)
{
    u8 i;
    for (i = 0; i < data_num; i++)
    {
        Tx3Buffer[count3++] = *(DataToSend + i);
    }
    if (!(USART3->CR1 & USART_CR1_TXEIE))
    {
        USART_ITConfig(USART3, USART_IT_TXE, ENABLE); // �򿪷����ж�
    }
}
u8 U3RxDataTmp[100];
u8 U3RxInCnt = 0;
u8 U3RxoutCnt = 0;
void drvU3GetByte(u8 data)
{
    U3RxDataTmp[U3RxInCnt++] = data;
    if (U3RxInCnt >= 100)
        U3RxInCnt = 0;
}
void drvU3DataCheck(void)
{
    while (U3RxInCnt != U3RxoutCnt)
    {
        U3GetOneByte(U3RxDataTmp[U3RxoutCnt++]);
        if (U3RxoutCnt >= 100)
            U3RxoutCnt = 0;
    }
}
void Usart3_IRQ(void)
{
    u8 com_data;

    if (USART3->SR & USART_SR_ORE) // ORE�ж�
        com_data = USART3->DR;

    // �����ж�
    if (USART_GetITStatus(USART3, USART_IT_RXNE))
    {
        USART_ClearITPendingBit(USART3, USART_IT_RXNE); // ����жϱ�־
        com_data = USART3->DR;
        drvU3GetByte(com_data);
    }
    // ���ͣ�������λ���ж�
    if (USART_GetITStatus(USART3, USART_IT_TXE))
    {
        USART3->DR = Tx3Buffer[Tx3Counter++]; // дDR����жϱ�־
        if (Tx3Counter == count3)
        {
            USART3->CR1 &= ~USART_CR1_TXEIE; // �ر�TXE�������жϣ��ж�
        }
    }
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////

//====uart4
void DrvUart4Init(u32 br_num)
{
    USART_InitTypeDef USART_InitStructure;
    // USART_ClockInitTypeDef USART_ClockInitStruct;
    NVIC_InitTypeDef NVIC_InitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;

    USART_StructInit(&USART_InitStructure);

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE); // ����USART2ʱ��
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

    // �����ж����ȼ�
    NVIC_InitStructure.NVIC_IRQChannel = UART4_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = NVIC_UART4_P;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = NVIC_UART4_S;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    GPIO_PinAFConfig(GPIOA, GPIO_PinSource0, GPIO_AF_UART4);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource1, GPIO_AF_UART4);

    // ����PC12��ΪUART5��Tx
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    // ����PD2��ΪUART5��Rx
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    // ����UART5
    // �жϱ�������
    USART_InitStructure.USART_BaudRate = br_num;                                    // �����ʿ���ͨ������վ����
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;                     // 8λ����
    USART_InitStructure.USART_StopBits = USART_StopBits_1;                          // ��֡��β����1��ֹͣλ
    USART_InitStructure.USART_Parity = USART_Parity_No;                             // ������żУ��
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; // Ӳ��������ʧ��
    USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;                 // ���͡�����ʹ��
    USART_Init(UART4, &USART_InitStructure);

    // ʹ��UART5�����ж�
    USART_ITConfig(UART4, USART_IT_RXNE, ENABLE);
    // ʹ��USART5
    USART_Cmd(UART4, ENABLE);
}
u8 Tx4Buffer[256];
u8 Tx4Counter = 0;
u8 count4 = 0;
void DrvUart4SendBuf(unsigned char *DataToSend, u8 data_num)
{
    u8 i;
    for (i = 0; i < data_num; i++)
    {
        Tx4Buffer[count4++] = *(DataToSend + i);
    }

    if (!(UART4->CR1 & USART_CR1_TXEIE))
    {
        USART_ITConfig(UART4, USART_IT_TXE, ENABLE); // �򿪷����ж�
    }
}
u8 U4RxDataTmp[100];
u8 U4RxInCnt = 0;
u8 U4RxoutCnt = 0;
void drvU4GetByte(u8 data)
{
    U4RxDataTmp[U4RxInCnt++] = data;
    if (U4RxInCnt >= 100)
        U4RxInCnt = 0;
}
void drvU4DataCheck(void)
{
    while (U4RxInCnt != U4RxoutCnt)
    {
        U4GetOneByte(U4RxDataTmp[U4RxoutCnt++]);
        if (U4RxoutCnt >= 100)
            U4RxoutCnt = 0;
    }
}
void Uart4_IRQ(void)
{
    u8 com_data;

    if (UART4->SR & USART_SR_ORE) // ORE�ж�
    {
        com_data = UART4->DR;
    }
    // �����ж�
    if (USART_GetITStatus(UART4, USART_IT_RXNE))
    {
        USART_ClearITPendingBit(UART4, USART_IT_RXNE); // ����жϱ�־
        com_data = UART4->DR;
        drvU4GetByte(com_data);
    }

    // ���ͣ�������λ���ж�
    if (USART_GetITStatus(UART4, USART_IT_TXE))
    {
        UART4->DR = Tx4Buffer[Tx4Counter++]; // дDR����жϱ�־
        if (Tx4Counter == count4)
        {
            UART4->CR1 &= ~USART_CR1_TXEIE; // �ر�TXE�������жϣ��ж�
        }
    }
}

//====uart5
void DrvUart5Init(u32 br_num)
{
    USART_InitTypeDef USART_InitStructure;
    // USART_ClockInitTypeDef USART_ClockInitStruct;
    NVIC_InitTypeDef NVIC_InitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;

    USART_StructInit(&USART_InitStructure);

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART5, ENABLE); // ����USART2ʱ��
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

    // �����ж����ȼ�
    NVIC_InitStructure.NVIC_IRQChannel = UART5_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = NVIC_UART5_P;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = NVIC_UART5_S;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    GPIO_PinAFConfig(GPIOC, GPIO_PinSource12, GPIO_AF_UART5);
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource2, GPIO_AF_UART5);

    // ����PC12��ΪUART5��Tx
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
    // ����PD2��ΪUART5��Rx
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOD, &GPIO_InitStructure);

    // ����UART5
    // �жϱ�������
    USART_InitStructure.USART_BaudRate = br_num;                                    // �����ʿ���ͨ������վ����
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;                     // 8λ����
    USART_InitStructure.USART_StopBits = USART_StopBits_1;                          // ��֡��β����1��ֹͣλ
    USART_InitStructure.USART_Parity = USART_Parity_No;                             // ������żУ��
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; // Ӳ��������ʧ��
    USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;                 // ���͡�����ʹ��
    USART_Init(UART5, &USART_InitStructure);

    // ʹ��UART5�����ж�
    USART_ITConfig(UART5, USART_IT_RXNE, ENABLE);
    // ʹ��USART5
    USART_Cmd(UART5, ENABLE);
}
u8 Tx5Buffer[256];
u8 Tx5Counter = 0;
u8 count5 = 0;
void DrvUart5SendBuf(unsigned char *DataToSend, u8 data_num)
{
    u8 i;
    for (i = 0; i < data_num; i++)
    {
        Tx5Buffer[count5++] = *(DataToSend + i);
    }

    if (!(UART5->CR1 & USART_CR1_TXEIE))
    {
        USART_ITConfig(UART5, USART_IT_TXE, ENABLE); // �򿪷����ж�
    }
}
u8 U5RxDataTmp[100];
u8 U5RxInCnt = 0;
u8 U5RxoutCnt = 0;
void drvU5GetByte(u8 data)
{
    U5RxDataTmp[U5RxInCnt++] = data;
    if (U5RxInCnt >= 100)
        U5RxInCnt = 0;
}
void drvU5DataCheck(void)
{
    while (U5RxInCnt != U5RxoutCnt)
    {
        U5GetOneByte(U5RxDataTmp[U5RxoutCnt++]);
        if (U5RxoutCnt >= 100)
            U5RxoutCnt = 0;
    }
}
void Uart5_IRQ(void)
{
    u8 com_data;

    if (UART5->SR & USART_SR_ORE) // ORE�ж�
    {
        com_data = UART5->DR;
    }
    // �����ж�
    if (USART_GetITStatus(UART5, USART_IT_RXNE))
    {
        USART_ClearITPendingBit(UART5, USART_IT_RXNE); // ����жϱ�־
        com_data = UART5->DR;
        drvU5GetByte(com_data);
    }

    // ���ͣ�������λ���ж�
    if (USART_GetITStatus(UART5, USART_IT_TXE))
    {
        UART5->DR = Tx5Buffer[Tx5Counter++]; // дDR����жϱ�־
        if (Tx5Counter == count5)
        {
            UART5->CR1 &= ~USART_CR1_TXEIE; // �ر�TXE�������жϣ��ж�
        }
    }
}

void DrvUartDataCheck(void)
{
    drvU1DataCheck();
    drvU2DataCheck();
    drvU3DataCheck();
    drvU4DataCheck();
    drvU5DataCheck();
}
/******************* (C) COPYRIGHT 2014 ANO TECH *****END OF FILE************/
