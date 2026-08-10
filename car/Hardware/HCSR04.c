#include "HCSR04.h"

/*
 * HC-SR04 超声波测距驱动
 * Trig: PF0   Echo: PA8
 *
 * 使用 DWT CYCCNT (72 MHz) 做微秒级计时，精度 ~0.2cm
 * 提供阻塞 (GetValue) 和非阻塞 (StartMeasure/Poll) 两套接口
 */

/* ---- DWT 周期计数器 (直接寄存器，兼容旧版 CMSIS) ---- */
#define CORE_MHZ            72U
#define US_TO_CYC(us)       ((us) * CORE_MHZ)

#define DWT_CTRL_REG    (*(volatile uint32_t *)0xE0001000)
#define DWT_CYCCNT_REG  (*(volatile uint32_t *)0xE0001004)
#define DEMCR_REG       (*(volatile uint32_t *)0xE000EDFC)
#define DEMCR_TRCENA    (1UL << 24)
#define DWT_CYCCNTENA   (1UL << 0)

#define TRIG_CYC            US_TO_CYC(10)       /* 触发脉冲 10 μs   */
#define ECHO_WAIT_CYC       US_TO_CYC(5000)     /* 等 Echo 上升 5 ms */
#define ECHO_MEAS_CYC       US_TO_CYC(3500)     /* 最大回波 3.5 ms ≈ 60 cm */
#define BLOCK_TIMEOUT_CYC   US_TO_CYC(30000)    /* 阻塞模式 30 ms   */

static void DWT_Enable(void)
{
    DEMCR_REG |= DEMCR_TRCENA;
    DWT_CTRL_REG |= DWT_CYCCNTENA;
}

#define DWT_CYC()  (DWT_CYCCNT_REG)

/* ---- 非阻塞状态机 ---- */
typedef enum { US_IDLE, US_WAIT_HIGH, US_WAIT_LOW } US_State_t;

static volatile US_State_t s_state = US_IDLE;
static uint32_t s_echoStart  = 0;
static uint32_t s_timeout    = 0;
static float    s_distance   = -1.0f;
static uint8_t  s_newReady   = 0;

/* ---- 距离换算: Echo 周期数 → cm ---- */
static float CyclesToCm(uint32_t cyc)
{
    /* cm = us * 0.017  ;  us = cyc / 72 */
    float us = (float)cyc / (float)CORE_MHZ;
    return us * 0.017f;
}

/* ================================================================ */
/*                          公共接口                                 */
/* ================================================================ */

void HCSR04_Init(void)
{
    GPIO_InitTypeDef gpio;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA |
                           RCC_APB2Periph_GPIOF, ENABLE);

    /* Trig – PF0 推挽输出 */
    gpio.GPIO_Pin   = GPIO_Pin_0;
    gpio.GPIO_Mode  = GPIO_Mode_Out_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOF, &gpio);
    GPIO_ResetBits(GPIOF, GPIO_Pin_0);

    /* Echo – PA8 下拉输入 */
    gpio.GPIO_Pin  = GPIO_Pin_8;
    gpio.GPIO_Mode = GPIO_Mode_IPD;
    GPIO_Init(GPIOA, &gpio);

    DWT_Enable();
    s_state = US_IDLE;
}

/* ---------- 阻塞式（兼容旧接口）---------- */

float HCSR04_GetValue(void)
{
    uint32_t t, timeout, echoStart;

    /* 触发 10 μs */
    GPIO_SetBits(GPIOF, GPIO_Pin_0);
    t = DWT_CYC();
    while ((DWT_CYC() - t) < TRIG_CYC) {}
    GPIO_ResetBits(GPIOF, GPIO_Pin_0);

    /* 等 Echo 上升 */
    timeout = DWT_CYC() + BLOCK_TIMEOUT_CYC;
    while (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_8) == Bit_RESET) {
        if ((int32_t)(DWT_CYC() - timeout) >= 0) return -1.0f;
    }
    echoStart = DWT_CYC();

    /* 等 Echo 下降 */
    timeout = DWT_CYC() + BLOCK_TIMEOUT_CYC;
    while (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_8) == Bit_SET) {
        if ((int32_t)(DWT_CYC() - timeout) >= 0) return -1.0f;
    }

    {
        float cm = CyclesToCm(DWT_CYC() - echoStart);
        return (cm >= 2.0f && cm <= 400.0f) ? cm : -1.0f;
    }
}

/* ---------- 非阻塞式 ---------- */

void HCSR04_StartMeasure(void)
{
    uint32_t t;
    if (s_state != US_IDLE) return;

    /* 10 μs 触发（阻塞，可忽略） */
    GPIO_SetBits(GPIOF, GPIO_Pin_0);
    t = DWT_CYC();
    while ((DWT_CYC() - t) < TRIG_CYC) {}
    GPIO_ResetBits(GPIOF, GPIO_Pin_0);

    s_timeout  = DWT_CYC() + ECHO_WAIT_CYC;
    s_state    = US_WAIT_HIGH;
    s_newReady = 0;
}

void HCSR04_Poll(void)
{
    switch (s_state)
    {
    default:
    case US_IDLE:
        return;

    case US_WAIT_HIGH:
        if (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_8) == Bit_SET)
        {
            s_echoStart = DWT_CYC();
            s_timeout   = s_echoStart + ECHO_MEAS_CYC;
            s_state     = US_WAIT_LOW;
        }
        else if ((int32_t)(DWT_CYC() - s_timeout) >= 0)
        {
            s_distance = -1.0f;
            s_newReady = 1;
            s_state    = US_IDLE;
        }
        break;

    case US_WAIT_LOW:
        if (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_8) == Bit_RESET)
        {
            float cm   = CyclesToCm(DWT_CYC() - s_echoStart);
            s_distance = (cm >= 2.0f && cm <= 60.0f) ? cm : -1.0f;
            s_newReady = 1;
            s_state    = US_IDLE;
        }
        else if ((int32_t)(DWT_CYC() - s_timeout) >= 0)
        {
            s_distance = -1.0f;
            s_newReady = 1;
            s_state    = US_IDLE;
        }
        break;
    }
}

float HCSR04_GetDistance(void)
{
    return s_distance;
}

uint8_t HCSR04_IsNewReady(void)
{
    uint8_t r = s_newReady;
    s_newReady = 0;
    return r;
}
