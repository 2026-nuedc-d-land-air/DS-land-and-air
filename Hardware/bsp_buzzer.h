/**
  * @file    bsp_buzzer.h
  * @brief   无源蜂鸣器控制模块 - 增强版
  * @note    PB8 - 低电平触发，使用PWM控制
  *          
  * 功能说明：
  * 1. 激活时短暂响一声
  * 2. 1分钟时响两声 (间隔较小)
  * 3. 终点判定：循迹L2-L7遮挡 + 陀螺仪数据稳定 + 时间>1分半 → 响三声升调
  * 4. 支持多种提示音效果
  */

#ifndef __BSP_BUZZER_H
#define __BSP_BUZZER_H

#include "stm32f10x.h"

/*====================================================================================*/
/*                                  音阶频率定义 (C调)                                  */
/*====================================================================================*/

/* 低音 */
#define NOTE_L1     262     /* 低音Do */
#define NOTE_L2     294     /* 低音Re */
#define NOTE_L3     330     /* 低音Mi */
#define NOTE_L4     349     /* 低音Fa */
#define NOTE_L5     392     /* 低音Sol */
#define NOTE_L6     440     /* 低音La */
#define NOTE_L7     494     /* 低音Si */

/* 中音 */
#define NOTE_M1     523     /* 中音Do */
#define NOTE_M2     587     /* 中音Re */
#define NOTE_M3     659     /* 中音Mi */
#define NOTE_M4     698     /* 中音Fa */
#define NOTE_M5     784     /* 中音Sol */
#define NOTE_M6     880     /* 中音La */
#define NOTE_M7     988     /* 中音Si */

/* 高音 */
#define NOTE_H1     1047    /* 高音Do */
#define NOTE_H2     1175    /* 高音Re */
#define NOTE_H3     1319    /* 高音Mi */
#define NOTE_H4     1397    /* 高音Fa */
#define NOTE_H5     1568    /* 高音Sol */
#define NOTE_H6     1760    /* 高音La */
#define NOTE_H7     1976    /* 高音Si */

/* 休止符 */
#define NOTE_REST   0       /* 休止 */

/*====================================================================================*/
/*                                  蜂鸣器状态定义                                     */
/*====================================================================================*/

typedef enum {
    BUZZER_IDLE = 0,        /* 空闲 */
    BUZZER_PLAYING          /* 正在播放 */
} BuzzerState_t;

/* 提示音类型 */
typedef enum {
    BEEP_ACTIVATE = 0,      /* 激活提示 - 短响1声 */
    BEEP_ONE_MINUTE,        /* 1分钟提醒 - 短响2声，间隔小 */
    BEEP_FINISH,            /* 终点到达 - 3声升调 */
    BEEP_SUCCESS,           /* 成功提示 - 上升音阶 */
    BEEP_ERROR,             /* 错误提示 - 下降音阶 */
    BEEP_WARNING,           /* 警告提示 - 急促双响 */
    BEEP_STARTUP,           /* 开机提示 - 欢迎音 */
    BEEP_KEY_PRESS,         /* 按键音 - 极短响 */
    BEEP_KEY_LONG,          /* 长按确认 - 两声确认 */
    BEEP_OBSTACLE,          /* 避障提示 - 低频双响 */
    BEEP_MAX
} BeepType_t;

/* 音符结构体 */
typedef struct {
    uint16_t frequency;     /* 频率 (Hz), 0表示休止 */
    uint16_t duration;      /* 持续时间 (ms) */
} BuzzerNote_t;

/* 提示音序列结构体 */
typedef struct {
    const BuzzerNote_t *notes;  /* 音符数组 */
    uint8_t noteCount;          /* 音符数量 */
    uint8_t volume;             /* 音量 (0-100) */
} BeepSequence_t;

/*====================================================================================*/
/*                                  终点判定参数                                       */
/*====================================================================================*/

/* 终点判定条件 */
#define FINISH_TRACK_MASK       0x7E    /* L2-L7遮挡: 0b01111110 */
#define FINISH_STABLE_COUNT     5       /* 陀螺仪稳定检测次数 */
#define FINISH_MIN_TIME_MS      90000   /* 最小时间 1分30秒 = 90000ms */
#define FINISH_ANGLE_THRESHOLD  2       /* 角度变化阈值 (度) */

/* 1分钟提醒 */
#define ONE_MINUTE_MS           60000   /* 1分钟 = 60000ms */

/*====================================================================================*/
/*                                  蜂鸣器参数                                         */
/*====================================================================================*/

#define BUZZER_DEFAULT_FREQ     3000    /* 默认蜂鸣器频率 3kHz */
#define BUZZER_DEFAULT_VOLUME   15      /* 默认音量 */

/*====================================================================================*/
/*                                  函数声明                                           */
/*====================================================================================*/

/**
  * @brief  蜂鸣器初始化
  */
void Buzzer_Init(void);

/**
  * @brief  蜂鸣器状态机更新 (需要在主循环中调用)
  * @param  deltaMs: 距离上次调用的时间间隔(ms)
  */
void Buzzer_Update(uint32_t deltaMs);

/**
  * @brief  播放指定类型的提示音
  * @param  type: 提示音类型
  */
void Buzzer_PlayBeep(BeepType_t type);

/**
  * @brief  触发蜂鸣器响一声 (激活提示)
  */
void Buzzer_BeepOnce(void);

/**
  * @brief  触发蜂鸣器响两声 (1分钟提醒)
  */
void Buzzer_BeepTwice(void);

/**
  * @brief  触发蜂鸣器响三声升调 (终点)
  */
void Buzzer_BeepTriple(void);

/**
  * @brief  触发避障低频双响
  */
void Buzzer_BeepObstacle(void);

/**
  * @brief  停止当前播放
  */
void Buzzer_Stop(void);

/**
  * @brief  检查蜂鸣器是否正在播放
  * @retval 1=正在播放, 0=空闲
  */
uint8_t Buzzer_IsPlaying(void);

/**
  * @brief  设置音量
  * @param  volume: 音量 (0-100)
  */
void Buzzer_SetVolume(uint8_t volume);

/**
  * @brief  检查终点条件
  * @param  trackData: 循迹数据
  * @param  angleRoll: Roll角度
  * @param  anglePitch: Pitch角度
  * @param  angleYaw: Yaw角度
  * @param  runTimeMs: 运行时间(ms)
  * @retval 1=到达终点, 0=未到达
  */
uint8_t Buzzer_CheckFinish(uint8_t trackData, int16_t angleRoll, 
                           int16_t anglePitch, int16_t angleYaw, 
                           uint32_t runTimeMs);

/**
  * @brief  检查1分钟提醒
  * @param  runTimeMs: 运行时间(ms)
  * @retval 1=需要提醒, 0=不需要
  */
uint8_t Buzzer_CheckOneMinute(uint32_t runTimeMs);

/**
  * @brief  获取计时是否已开始
  * @retval 1=已开始, 0=未开始
  */
uint8_t Buzzer_IsTimingStarted(void);

/**
  * @brief  开始计时 (检测到有效循迹数据时调用)
  */
void Buzzer_StartTiming(void);

/**
  * @brief  重置蜂鸣器状态
  */
void Buzzer_Reset(void);

#endif /* __BSP_BUZZER_H */
