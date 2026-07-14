/**
  * @file    app_stats.c
  * @brief   运行统计模块
  * @note    统计运行时间、速度、距离等信息
  *          速度和距离预留给编码器接入后补全
  */

#include "app_stats.h"
#include "JY301P.h"
#include <math.h>

/*====================================================================================*/
/*                                  全局变量                                           */
/*====================================================================================*/

volatile Stats_Data_t g_stats;

/*====================================================================================*/
/*                                  初始化函数                                         */
/*====================================================================================*/

/**
  * @brief  统计模块初始化
  */
void Stats_Init(void)
{
    /* 清零所有统计数据 */
    g_stats.runTime_ms = 0;
    g_stats.runTime_sec = 0;
    g_stats.runTime_min = 0;
    
    g_stats.currentSpeed = 0.0f;
    g_stats.maxSpeed = 0.0f;
    g_stats.avgSpeed = 0.0f;
    
    g_stats.totalDistance = 0.0f;
    g_stats.maxAngularSpeed = 0.0f;
}

/*====================================================================================*/
/*                                  更新函数                                           */
/*====================================================================================*/

/**
  * @brief  统计数据更新
  * @param  deltaMs: 距离上次更新的时间间隔(ms)
  */
void Stats_Update(uint32_t deltaMs)
{
    /* 更新运行时间 */
    g_stats.runTime_ms += deltaMs;
    
    /* 转换为秒和分钟 - 使用临时变量避免优化问题 */
    uint32_t totalMs = g_stats.runTime_ms;
    uint32_t totalSec = totalMs / 1000UL;
    
    g_stats.runTime_sec = (uint16_t)totalSec;
    g_stats.runTime_min = (uint16_t)(totalSec / 60UL);
    
    /* 更新距离 (如果有速度数据) */
    if (g_stats.currentSpeed > 0.0f)
    {
        /* 距离 = 速度 × 时间 */
        g_stats.totalDistance += g_stats.currentSpeed * ((float)deltaMs / 1000.0f);
    }
    
    /* 更新平均速度 */
    if (g_stats.runTime_sec > 0)
    {
        g_stats.avgSpeed = g_stats.totalDistance / (float)g_stats.runTime_sec;
    }
}

/**
  * @brief  更新速度数据 (预留给编码器)
  * @param  speed: 当前速度 (m/s)
  */
void Stats_UpdateSpeed(float speed)
{
    g_stats.currentSpeed = speed;
    
    /* 更新最大速度 */
    if (speed > g_stats.maxSpeed)
    {
        g_stats.maxSpeed = speed;
    }
}

/**
  * @brief  更新角速度数据
  * @param  angularSpeed: 当前角速度 (°/s)
  */
void Stats_UpdateAngularSpeed(float angularSpeed)
{
    float absSpeed = (angularSpeed >= 0) ? angularSpeed : -angularSpeed;
    
    /* 更新最大角速度 */
    if (absSpeed > g_stats.maxAngularSpeed)
    {
        g_stats.maxAngularSpeed = absSpeed;
    }
}

/**
  * @brief  重置统计数据
  */
void Stats_Reset(void)
{
    Stats_Init();
}

/**
  * @brief  获取格式化的运行时间字符串
  * @param  buf: 输出缓冲区 (至少8字节: "MM:SS\0")
  */
void Stats_GetTimeString(char *buf)
{
    uint16_t min = g_stats.runTime_min;
    uint16_t sec = g_stats.runTime_sec % 60;
    
    /* 格式化为 "MM:SS" */
    buf[0] = '0' + (min / 10) % 10;
    buf[1] = '0' + min % 10;
    buf[2] = ':';
    buf[3] = '0' + sec / 10;
    buf[4] = '0' + sec % 10;
    buf[5] = '\0';
}
