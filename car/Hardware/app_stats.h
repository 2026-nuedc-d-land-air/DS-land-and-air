#ifndef __APP_STATS_H
#define __APP_STATS_H

#include "stm32f10x.h"

/*====================================================================================*/
/*                                  统计数据结构                                       */
/*====================================================================================*/

typedef struct {
    /* 运行时间 */
    uint32_t runTime_ms;        // 运行时间 (毫秒)
    uint16_t runTime_sec;       // 运行时间 (秒)
    uint16_t runTime_min;       // 运行时间 (分钟)
    
    /* 速度相关 (预留给编码器) */
    float currentSpeed;         // 当前速度 (m/s)
    float maxSpeed;             // 最大速度 (m/s)
    float avgSpeed;             // 平均速度 (m/s)
    
    /* 距离相关 (预留给编码器) */
    float totalDistance;        // 总行驶距离 (m)
    
    /* 陀螺仪辅助数据 */
    float maxAngularSpeed;      // 最大角速度 (°/s)
    
} Stats_Data_t;

/* 使用volatile防止编译器优化 */
extern volatile Stats_Data_t g_stats;

/*====================================================================================*/
/*                                  函数声明                                           */
/*====================================================================================*/

/**
  * @brief  统计模块初始化
  * @note   清零所有统计数据，记录启动时间
  */
void Stats_Init(void);

/**
  * @brief  统计数据更新
  * @param  deltaMs: 距离上次更新的时间间隔(ms)
  * @note   需要在主循环中周期性调用
  */
void Stats_Update(uint32_t deltaMs);

/**
  * @brief  更新速度数据 (预留给编码器)
  * @param  speed: 当前速度 (m/s)
  */
void Stats_UpdateSpeed(float speed);

/**
  * @brief  更新角速度数据
  * @param  angularSpeed: 当前角速度 (°/s)
  */
void Stats_UpdateAngularSpeed(float angularSpeed);

/**
  * @brief  重置统计数据
  */
void Stats_Reset(void);

/**
  * @brief  获取格式化的运行时间字符串
  * @param  buf: 输出缓冲区 (至少8字节: "MM:SS\0")
  */
void Stats_GetTimeString(char *buf);

#endif /* __APP_STATS_H */
