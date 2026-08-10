/**
  * @file    app_ui.c
  * @brief   UI显示控制模块
  * @note    管理OLED多页面显示，支持页面切换
  * 
  * 页面说明：
  * - 综合页(PAGE_OVERVIEW): 三轴角度 + 二进制循迹 + 运行时间 + 速度
  * - 循迹页(PAGE_TRACKING): 二进制 + 偏移量 + 传感器状态标注
  * - 陀螺仪页(PAGE_GYROSCOPE): 三轴角度 + 最大速度 + 距离预留
  */

#include "app_ui.h"
#include "OLED.h"
#include "JY301P.h"
#include "app_stats.h"
#include "bsp_systick.h"
#include "bsp_key.h"
#include "bsp_led_pwm.h"
#include <string.h>

static void UI_DisplayMotorTest(void);

/*====================================================================================*/
/*                                  私有变量                                           */
/*====================================================================================*/

static UI_Page_t s_currentPage = UI_PAGE_MOTOR_TEST;
static uint8_t s_showStats = 0;         // 是否显示统计页 (长按触发)
static uint8_t s_pageChanged = 1;       // 页面切换标志，用于清屏
static uint8_t s_finishMode = 0;        // 终点锁定显示
static uint8_t s_obstacleMode = 0;      // 避障模式显示标志
static uint8_t s_obstacleHint = 0;      // 避障提示覆盖层
static uint8_t s_obstacleHintMs = 0;   // 避障提示计时
static float s_ultrasonicCm = 0.0f;     // 超声波距离(cm)

static UI_MotorTest_t s_motorTest;

/*====================================================================================*/
/*                                  全局变量                                           */
/*====================================================================================*/

TrackingAnalysis_t g_trackAnalysis;

/*====================================================================================*/
/*                                  初始化函数                                         */
/*====================================================================================*/

/**
  * @brief  UI模块初始化
  */
void UI_Init(void)
{
    s_currentPage = UI_PAGE_MOTOR_TEST;
    s_showStats = 0;
    s_pageChanged = 1;  // 初始化时需要清屏
    s_finishMode = 0;
    s_obstacleMode = 0;
    s_obstacleHint = 0;
    s_obstacleHintMs = 0;
    s_ultrasonicCm = 0.0f;

    memset(&s_motorTest, 0, sizeof(s_motorTest));

    /* 清零循迹分析数据 */
    memset(&g_trackAnalysis, 0, sizeof(g_trackAnalysis));
}

/*====================================================================================*/
/*                                  页面切换                                           */
/*====================================================================================*/

/**
  * @brief  切换到下一页 (短按)
  */
void UI_NextPage(void)
{
    if (s_finishMode)
    {
        return;
    }

    /* 如果在统计页，退出统计页 */
    if (s_showStats)
    {
        s_showStats = 0;
        s_pageChanged = 1;
        return;
    }
    
    /* 切换到下一页 */
    if (s_obstacleMode)
    {
        /* 避障模式下：在 MotorTest/Overview/Gyro/Tracking 之间循环 */
        if (s_currentPage == UI_PAGE_MOTOR_TEST)
        {
            s_currentPage = UI_PAGE_OVERVIEW;
        }
        else if (s_currentPage == UI_PAGE_OVERVIEW)
        {
            s_currentPage = UI_PAGE_GYROSCOPE;
        }
        else if (s_currentPage == UI_PAGE_GYROSCOPE)
        {
            s_currentPage = UI_PAGE_TRACKING;
        }
        else
        {
            s_currentPage = UI_PAGE_MOTOR_TEST;
        }
    }
    else
    {
        s_currentPage = (UI_Page_t)((s_currentPage + 1) % UI_PAGE_COUNT);
    }
    s_pageChanged = 1;
}

/**
  * @brief  获取当前页面
  */
UI_Page_t UI_GetCurrentPage(void)
{
    return s_currentPage;
}

/**
  * @brief  设置当前页面
  */
void UI_SetPage(UI_Page_t page)
{
    if (page < UI_PAGE_COUNT)
    {
        s_currentPage = page;
        s_pageChanged = 1;
    }
}

void UI_SetObstacleMode(uint8_t enable)
{
    s_obstacleMode = enable ? 1 : 0;
}

uint8_t UI_GetObstacleMode(void)
{
    return s_obstacleMode;
}

void UI_ShowObstacleHint(uint8_t enable)
{
    s_obstacleHint = enable ? 1 : 0;
    s_obstacleHintMs = 0;
    s_pageChanged = 1;
}

void UI_Tick(uint32_t deltaMs)
{
    if (s_obstacleHint)
    {
        if (s_obstacleHintMs < 5000U)
        {
            s_obstacleHintMs += (uint16_t)deltaMs;
        }
        if (s_obstacleHintMs >= 1500U)
        {
            s_obstacleHint = 0;
            s_pageChanged = 1;
        }
    }
}

void UI_RequestRefresh(void)
{
    s_pageChanged = 1;
}

/*====================================================================================*/
/*                                  循迹数据分析                                       */
/*====================================================================================*/

/**
  * @brief  更新循迹分析数据
  * @param  trackData: 8位循迹原始数据
  * @note   传感器排列假设: [S7][S6][S5][S4][S3][S2][S1][S0]
  *         S7=最左侧, S0=最右侧
  *         1=检测到黑线(遮挡), 0=未检测到
  */
void UI_UpdateTrackingAnalysis(uint8_t trackData)
{
    uint8_t i;
    int16_t weightedSum = 0;
    uint8_t activeCount = 0;
    uint8_t firstActive = 0xFF;
    uint8_t lastActive = 0xFF;
    
    g_trackAnalysis.rawData = trackData;
    
    /* 解析每个传感器状态 */
    for (i = 0; i < 8; i++)
    {
        /* 位7对应传感器0(最左), 位0对应传感器7(最右) */
        g_trackAnalysis.sensorStatus[i] = (trackData >> (7 - i)) & 0x01;
        
        if (g_trackAnalysis.sensorStatus[i])
        {
            activeCount++;
            if (firstActive == 0xFF) firstActive = i;
            lastActive = i;
            
            /* 加权计算偏移量: 位置0-7对应权重-3到+3 */
            weightedSum += (int16_t)(i * 2) - 7;
        }
    }
    
    /* 计算偏移量 */
    if (activeCount > 0)
    {
        g_trackAnalysis.offset = (int8_t)(weightedSum / (int16_t)activeCount);
    }
    else
    {
        g_trackAnalysis.offset = 0;
    }
    
    /* 计算线宽 */
    if (firstActive != 0xFF && lastActive != 0xFF)
    {
        g_trackAnalysis.lineWidth = lastActive - firstActive + 1;
    }
    else
    {
        g_trackAnalysis.lineWidth = 0;
    }
    
    /* 判断边缘位置 */
    if (activeCount == 0)
    {
        g_trackAnalysis.edgePosition = 0;   // 无检测
    }
    else if (activeCount == 8)
    {
        g_trackAnalysis.edgePosition = 4;   // 全遮挡
    }
    else if (firstActive <= 2 && lastActive <= 3)
    {
        g_trackAnalysis.edgePosition = 1;   // 左侧
    }
    else if (firstActive >= 4 && lastActive >= 5)
    {
        g_trackAnalysis.edgePosition = 3;   // 右侧
    }
    else
    {
        g_trackAnalysis.edgePosition = 2;   // 中间
    }
}

/*====================================================================================*/
/*                                  显示函数                                           */
/*====================================================================================*/

/**
  * @brief  显示当前页面
  */
void UI_Display(uint8_t trackData, uint8_t trackOK)
{
    /* 页面切换时清屏 */
    if (s_pageChanged)
    {
        OLED_Clear();
        s_pageChanged = 0;
    }
    
    /* 更新循迹分析 */
    UI_UpdateTrackingAnalysis(trackData);
    
    if (s_finishMode)
    {
        OLED_ShowString(1, 1, "SUCCESS ENDLINE");
        OLED_ShowString(2, 1, "STOPPED");
        OLED_ShowString(3, 1, "PLEASE RESET");
        OLED_ShowString(4, 1, " ");
        return;
    }

    /* 如果显示统计页 */
    if (s_showStats)
    {
        UI_DisplayStats();
        return;
    }

    if (s_obstacleHint)
    {
        OLED_ShowString(1, 1, "OBSTACLE MODE");
        OLED_ShowString(2, 1, s_obstacleMode ? "MODE: ON " : "MODE: OFF");
        OLED_ShowString(3, 1, "LONG C5 TOGGLE");
        OLED_ShowString(4, 1, "OBST + GYRO");
        return;
    }
    
    /* 根据当前页面显示 */
    switch (s_currentPage)
    {
        case UI_PAGE_MOTOR_TEST:
            UI_DisplayMotorTest();
            break;

        case UI_PAGE_OVERVIEW:
            UI_DisplayOverview(trackData, trackOK);
            break;

        case UI_PAGE_TRACKING:
            UI_DisplayTracking(trackData, trackOK);
            break;

        case UI_PAGE_GYROSCOPE:
            UI_DisplayGyroscope();
            break;

        default:
            UI_DisplayMotorTest();
            break;
    }
}

/**
  * @brief  显示综合信息页
  * @note   第1行: 二进制循迹 + 状态
  *         第2行: Roll Pitch
  *         第3行: Yaw + 运行时间
  *         第4行: Spd + k: + l:
  */
void UI_DisplayOverview(uint8_t trackData, uint8_t trackOK)
{
    uint8_t i;
    char timeBuf[8];
    uint8_t lpLevel;
    uint32_t pressTime;
    
    /* 综合页面LED控制: 两个LED都亮，使用保存的亮度 */
    /* 终点效果期间不控制LED */
    if (!LED_IsFinishEffectPlaying())
    {
        LED_SetBrightness(LED_GREEN, LED_GetSavedBrightness(LED_GREEN));
        LED_SetBrightness(LED_RED, LED_GetSavedBrightness(LED_RED));
        LED_Switch(LED_GREEN, 1);   /* PB9 绿色LED开启 */
        LED_Switch(LED_RED, 1);     /* PE0 红色LED开启 */
    }
    
    /* 第1行: 二进制循迹 + 状态 */
    for (i = 0; i < 8; i++)
    {
        OLED_ShowChar(1, 1 + i, g_trackAnalysis.sensorStatus[i] ? '1' : '0');
    }
    OLED_ShowString(1, 10, " ");
    if (trackOK == 0)
    {
        OLED_ShowString(1, 11, "ok   ");
    }
    else
    {
        OLED_ShowString(1, 11, "ERR  ");
    }
    
    /* 第2行: Roll Pitch */
    OLED_ShowString(2, 1, "R:");
    OLED_ShowSignedNum(2, 3, (int32_t)g_jy301p_data.angle[0], 4);
    OLED_ShowString(2, 9, "P:");
    OLED_ShowSignedNum(2, 11, (int32_t)g_jy301p_data.angle[1], 4);
    OLED_ShowString(2, 16, " ");
    
    /* 第3行: Yaw + 运行时间 */
    OLED_ShowString(3, 1, "Y:");
    OLED_ShowSignedNum(3, 3, (int32_t)g_jy301p_data.angle[2], 4);
    SysTick_GetTimeString(timeBuf);
    OLED_ShowString(3, 9, "T:");
    OLED_ShowString(3, 11, timeBuf);
    OLED_ShowString(3, 16, " ");
    
    /* 第4行: Spd + k: + l: */
    pressTime = Key_GetPressTime();
    lpLevel = Key_GetLongPressLevel();
    
    OLED_ShowString(4, 1, "S:");
    OLED_ShowNum(4, 3, (uint32_t)(g_stats.currentSpeed * 10), 2);
    
    OLED_ShowString(4, 6, "k:");
    if (pressTime > 0)
    {
        uint8_t kVal = (uint8_t)((pressTime / 100) % 10);
        OLED_ShowNum(4, 8, kVal, 1);
    }
    else
    {
        OLED_ShowString(4, 8, "0");
    }
    
    OLED_ShowString(4, 10, "l:");
    if (lpLevel > 0)
    {
        OLED_ShowNum(4, 12, lpLevel, 1);
    }
    else
    {
        OLED_ShowString(4, 12, "0");
    }
    
    OLED_ShowString(4, 13, "    ");
}

/**
  * @brief  显示循迹详情页
  * @note   第1行: 二进制信息
  *         第2行: 偏移量 + 状态
  *         第3行: ZD:1遮挡 ND:0未遮挡 (提示)
  *         第4行: Spd + k: + l:
  */
void UI_DisplayTracking(uint8_t trackData, uint8_t trackOK)
{
    uint8_t i;
    uint32_t pressTime;
    uint8_t lpLevel;
    
    /* 循迹页面LED控制: B9亮, E0灭，使用保存的亮度 */
    /* 终点效果期间不控制LED */
    if (!LED_IsFinishEffectPlaying())
    {
        LED_SetBrightness(LED_GREEN, LED_GetSavedBrightness(LED_GREEN));
        LED_Switch(LED_GREEN, 1);   /* PB9 绿色LED开启 */
        LED_Switch(LED_RED, 0);     /* PE0 红色LED关闭 */
    }
    
    /* 第1行: 二进制信息 */
    for (i = 0; i < 8; i++)
    {
        OLED_ShowChar(1, 1 + i, g_trackAnalysis.sensorStatus[i] ? '1' : '0');
    }
    OLED_ShowString(1, 10, " H:");
    OLED_ShowHexNum(1, 13, trackData, 2);
    OLED_ShowString(1, 15, " ");
    
    /* 第2行: 偏移量 + 状态 */
    OLED_ShowString(2, 1, "Off:");
    OLED_ShowSignedNum(2, 5, g_trackAnalysis.offset, 3);
    OLED_ShowString(2, 9, " ");
    if (trackOK == 0)
    {
        OLED_ShowString(2, 10, "ok ");
    }
    else
    {
        OLED_ShowString(2, 10, "ERR   ");
    }
    
    /* 第3行: ZD:1遮挡 ND:0未遮挡 (固定提示) */
    OLED_ShowString(3, 1, "ZD:1 Y ND:0 N");
    
    /* 第4行: Spd + k: + l: (与综合页面一致) */
    pressTime = Key_GetPressTime();
    lpLevel = Key_GetLongPressLevel();
    
    OLED_ShowString(4, 1, "S:");
    OLED_ShowNum(4, 3, (uint32_t)(g_stats.currentSpeed * 10), 2);
    
    OLED_ShowString(4, 6, "k:");
    if (pressTime > 0)
    {
        uint8_t kVal = (uint8_t)((pressTime / 100) % 10);
        OLED_ShowNum(4, 8, kVal, 1);
    }
    else
    {
        OLED_ShowString(4, 8, "0");
    }
    
    OLED_ShowString(4, 10, "l:");
    if (lpLevel > 0)
    {
        OLED_ShowNum(4, 12, lpLevel, 1);
    }
    else
    {
        OLED_ShowString(4, 12, "0");
    }
    
    OLED_ShowString(4, 13, "    ");
}

/**
  * @brief  显示陀螺仪/路程页面
  * @note   第1行: Roll Pitch
  *         第2行: Yaw + 速度
  *         第3行: 距离(预留编码器)
  *         第4行: Spd + k: + l:
  */
void UI_DisplayGyroscope(void)
{
    uint32_t pressTime;
    uint8_t lpLevel;
    
    /* 陀螺仪页面LED控制: E0亮, B9灭，使用保存的亮度 */
    /* 终点效果期间不控制LED */
    if (!LED_IsFinishEffectPlaying())
    {
        LED_SetBrightness(LED_RED, LED_GetSavedBrightness(LED_RED));
        LED_Switch(LED_GREEN, 0);   /* PB9 绿色LED关闭 */
        LED_Switch(LED_RED, 1);     /* PE0 红色LED开启 */
    }
    
    /* 第1行: Roll Pitch */
    OLED_ShowString(1, 1, "R:");
    OLED_ShowSignedNum(1, 3, (int32_t)g_jy301p_data.angle[0], 4);
    OLED_ShowString(1, 9, "P:");
    OLED_ShowSignedNum(1, 11, (int32_t)g_jy301p_data.angle[1], 4);
    OLED_ShowString(1, 16, " ");
    
    /* 第2行: Yaw + 速度 */
    OLED_ShowString(2, 1, "Y:");
    OLED_ShowSignedNum(2, 3, (int32_t)g_jy301p_data.angle[2], 4);
    OLED_ShowString(2, 9, "V:");
    OLED_ShowNum(2, 11, (uint32_t)(g_stats.currentSpeed * 100), 4);
    OLED_ShowString(2, 16, " ");
    
    /* 第3行: 超声波距离 */
    OLED_ShowString(3, 1, "US:");
    OLED_ShowNum(3, 4, (uint32_t)(s_ultrasonicCm + 0.5f), 4);
    OLED_ShowString(3, 8, "cm    ");
    
    /* 第4行: Spd + k: + l: (与其他页面一致) */
    pressTime = Key_GetPressTime();
    lpLevel = Key_GetLongPressLevel();
    
    OLED_ShowString(4, 1, "S:");
    OLED_ShowNum(4, 3, (uint32_t)(g_stats.currentSpeed * 10), 2);
    
    OLED_ShowString(4, 6, "k:");
    if (pressTime > 0)
    {
        uint8_t kVal = (uint8_t)((pressTime / 100) % 10);
        OLED_ShowNum(4, 8, kVal, 1);
    }
    else
    {
        OLED_ShowString(4, 8, "0");
    }
    
    OLED_ShowString(4, 10, "l:");
    if (lpLevel > 0)
    {
        OLED_ShowNum(4, 12, lpLevel, 1);
    }
    else
    {
        OLED_ShowString(4, 12, "0");
    }
    
    OLED_ShowString(4, 13, "    ");
}

/**
  * @brief  显示统计页 (长按触发)
  * @note   第1行: 标题
  *         第2行: 运行时间
  *         第3行: 距离 + 平均速度
  *         第4行: 最大速度 + 最大角速度
  */
void UI_DisplayStats(void)
{
    char timeBuf[8];
    static uint8_t toggleState = 0;
    
    /* 统计页面LED控制: 两个LED交替闪烁效果，使用保存的亮度 */
    /* 终点效果期间不控制LED */
    if (!LED_IsFinishEffectPlaying())
    {
        LED_SetBrightness(LED_GREEN, LED_GetSavedBrightness(LED_GREEN));
        LED_SetBrightness(LED_RED, LED_GetSavedBrightness(LED_RED));
        toggleState = !toggleState;
        LED_Switch(LED_GREEN, toggleState);
        LED_Switch(LED_RED, !toggleState);
    }
    
    /* 第1行: 标题 */
    OLED_ShowString(1, 1, "== STATISTICS ==");
    
    /* 第2行: 运行时间 */
    Stats_GetTimeString(timeBuf);
    OLED_ShowString(2, 1, "RunTime: ");
    OLED_ShowString(2, 10, timeBuf);
    OLED_ShowString(2, 15, "  ");
    
    /* 第3行: 距离 + 平均速度 */
    OLED_ShowString(3, 1, "Dist:");
    OLED_ShowSignedNum(3, 6, (int32_t)(g_stats.totalDistance * 100), 4);
    OLED_ShowString(3, 10, "cm");
    
    /* 第4行: 最大速度 + 最大角速度 */
    OLED_ShowString(4, 1, "MaxV:");
    OLED_ShowSignedNum(4, 6, (int32_t)(g_stats.maxSpeed * 100), 3);
    OLED_ShowString(4, 10, "MaxG:");
    OLED_ShowSignedNum(4, 15, (int32_t)g_stats.maxAngularSpeed, 2);
}

/**
  * @brief  触发显示统计页
  */
void UI_ShowStats(void)
{
    s_showStats = 1;
    s_pageChanged = 1;
}

/**
  * @brief  退出统计页
  */
void UI_HideStats(void)
{
    s_showStats = 0;
    s_pageChanged = 1;
}

/**
  * @brief  检查是否在统计页
  */
uint8_t UI_IsShowingStats(void)
{
    return s_showStats;
}

void UI_SetUltrasonicDistance(float distanceCm)
{
    if (distanceCm < 0.0f) distanceCm = 0.0f;
    s_ultrasonicCm = distanceCm;
}

void UI_SetFinishMode(uint8_t enable)
{
    s_finishMode = enable ? 1 : 0;
    s_pageChanged = 1;
    if (s_finishMode)
    {
        s_showStats = 0;
    }
}

uint8_t UI_IsFinishMode(void)
{
    return s_finishMode;
}

void UI_SetMotorTest(uint16_t pwmMinLeft, uint16_t pwmMinRight, float speedDiffMms)
{
    s_motorTest.valid = 1;
    s_motorTest.pwmMinLeft = pwmMinLeft;
    s_motorTest.pwmMinRight = pwmMinRight;
    s_motorTest.speedDiffMms = speedDiffMms;
}

void UI_ClearMotorTest(void)
{
    memset(&s_motorTest, 0, sizeof(s_motorTest));
    s_pageChanged = 1;
}

static void UI_DisplayMotorTest(void)
{
    uint16_t lPct;
    uint16_t rPct;
    int32_t diff;

    /* 基于当前电机PWM周期 (TIM5: 3600=100%) */
    lPct = (uint16_t)((uint32_t)s_motorTest.pwmMinLeft * 100U / 3600U);
    rPct = (uint16_t)((uint32_t)s_motorTest.pwmMinRight * 100U / 3600U);

    diff = (int32_t)(s_motorTest.speedDiffMms);

    OLED_ShowString(1, 1, "== MOTOR TEST ==");

    OLED_ShowString(2, 1, "Lmin ");
    OLED_ShowNum(2, 6, lPct, 2);
    OLED_ShowString(2, 8, "%:");
    OLED_ShowNum(2, 11, s_motorTest.pwmMinLeft, 4);

    OLED_ShowString(3, 1, "Rmin ");
    OLED_ShowNum(3, 6, rPct, 2);
    OLED_ShowString(3, 8, "%:");
    OLED_ShowNum(3, 11, s_motorTest.pwmMinRight, 4);

    OLED_ShowString(4, 1, "dV:");
    OLED_ShowSignedNum(4, 4, diff, 4);
    OLED_ShowString(4, 8, "mm/s   ");
}
