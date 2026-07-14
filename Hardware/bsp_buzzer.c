/**
  * @file    bsp_buzzer.c
  * @brief   无源蜂鸣器控制模块实现 - 增强版
  * @note    PB8 - 低电平触发，使用PWM控制
  *          支持多种提示音效果，包括升调、降调等
  */

#include "bsp_buzzer.h"
#include "bsp_pwm.h"
#include "bsp_systick.h"
#include <stdlib.h>

/*====================================================================================*/
/*                                  提示音序列定义                                     */
/*====================================================================================*/

/* 激活提示 - 短响1声 */
static const BuzzerNote_t s_notesActivate[] = {
    {NOTE_M5, 60},      /* Sol - 60ms */
};

/* 1分钟提醒 - 短响2声，间隔小 (50ms) */
static const BuzzerNote_t s_notesOneMinute[] = {
    {NOTE_M3, 25},      /* Mi - 80ms */
    {NOTE_REST, 50},    /* 休止 - 50ms (间隔小) */
    {NOTE_M3, 25},      /* Mi - 80ms */
};

/* 终点到达 - 3声升调，有间隔 */
static const BuzzerNote_t s_notesFinish[] = {
    {NOTE_M1, 25},     /* Do - 150ms */
    {NOTE_REST, 50},    /* 休止 - 80ms */
    {NOTE_M3, 30},     /* Mi - 150ms (升调) */
    {NOTE_REST, 50},    /* 休止 - 80ms */
    {NOTE_M5, 35},     /* Sol - 200ms (再升调，稍长) */
};

/* 成功提示 - 上升音阶 Do-Mi-Sol-高Do */
static const BuzzerNote_t s_notesSuccess[] = {
    {NOTE_M1, 80},      /* Do */
    {NOTE_REST, 30},
    {NOTE_M3, 80},      /* Mi */
    {NOTE_REST, 30},
    {NOTE_M5, 80},      /* Sol */
    {NOTE_REST, 30},
    {NOTE_H1, 120},     /* 高Do */
};

/* 错误提示 - 下降音阶 */
static const BuzzerNote_t s_notesError[] = {
    {NOTE_M5, 100},     /* Sol */
    {NOTE_REST, 50},
    {NOTE_M3, 100},     /* Mi */
    {NOTE_REST, 50},
    {NOTE_M1, 150},     /* Do */
};

/* 警告提示 - 急促双响 */
static const BuzzerNote_t s_notesWarning[] = {
    {NOTE_M6, 50},      /* La - 急促 */
    {NOTE_REST, 30},
    {NOTE_M6, 50},      /* La */
    {NOTE_REST, 100},
    {NOTE_M6, 50},      /* La */
    {NOTE_REST, 30},
    {NOTE_M6, 50},      /* La */
};

/* 开机提示 - 欢迎音 Do-Sol-高Do */
static const BuzzerNote_t s_notesStartup[] = {
    {NOTE_M1, 100},     /* Do */
    {NOTE_REST, 50},
    {NOTE_M5, 100},     /* Sol */
    {NOTE_REST, 50},
    {NOTE_H1, 150},     /* 高Do */
};

/* 按键音 - 极短响 */
static const BuzzerNote_t s_notesKeyPress[] = {
    {NOTE_H1, 30},      /* 高Do - 30ms极短 */
};

/* 长按确认 - 两声确认 */
static const BuzzerNote_t s_notesKeyLong[] = {
    {NOTE_M5, 60},      /* Sol */
    {NOTE_REST, 40},
    {NOTE_H1, 80},      /* 高Do */
};

/* 避障提示 - 低频双响 */
static const BuzzerNote_t s_notesObstacle[] = {
    {NOTE_L2, 120},
    {NOTE_REST, 80},
    {NOTE_L2, 120},
};

/* 提示音序列表 */
static const BeepSequence_t s_beepSequences[BEEP_MAX] = {
    /* BEEP_ACTIVATE */     {s_notesActivate,   sizeof(s_notesActivate)/sizeof(BuzzerNote_t),   20},
    /* BEEP_ONE_MINUTE */   {s_notesOneMinute,  sizeof(s_notesOneMinute)/sizeof(BuzzerNote_t),  25},
    /* BEEP_FINISH */       {s_notesFinish,     sizeof(s_notesFinish)/sizeof(BuzzerNote_t),     30},
    /* BEEP_SUCCESS */      {s_notesSuccess,    sizeof(s_notesSuccess)/sizeof(BuzzerNote_t),    20},
    /* BEEP_ERROR */        {s_notesError,      sizeof(s_notesError)/sizeof(BuzzerNote_t),      25},
    /* BEEP_WARNING */      {s_notesWarning,    sizeof(s_notesWarning)/sizeof(BuzzerNote_t),    30},
    /* BEEP_STARTUP */      {s_notesStartup,    sizeof(s_notesStartup)/sizeof(BuzzerNote_t),    20},
    /* BEEP_KEY_PRESS */    {s_notesKeyPress,   sizeof(s_notesKeyPress)/sizeof(BuzzerNote_t),   15},
    /* BEEP_KEY_LONG */     {s_notesKeyLong,    sizeof(s_notesKeyLong)/sizeof(BuzzerNote_t),    20},
    /* BEEP_OBSTACLE */     {s_notesObstacle,   sizeof(s_notesObstacle)/sizeof(BuzzerNote_t),   18},
};

/*====================================================================================*/
/*                                  私有变量                                           */
/*====================================================================================*/

static BuzzerState_t s_buzzerState = BUZZER_IDLE;
static const BeepSequence_t *s_currentSequence = NULL;  /* 当前播放的序列 */
static uint8_t s_noteIndex = 0;         /* 当前音符索引 */
static uint32_t s_noteTimer = 0;        /* 音符计时器 */
static uint8_t s_globalVolume = BUZZER_DEFAULT_VOLUME;  /* 全局音量 */

/* 计时控制 */
static uint8_t s_timingStarted = 0;     /* 计时是否已开始 */
static uint8_t s_oneMinuteAlerted = 0;  /* 1分钟是否已提醒 */
static uint8_t s_finishAlerted = 0;     /* 终点是否已提醒 */

/* 陀螺仪稳定检测 */
static int16_t s_lastAngle[3] = {0, 0, 0};  /* 上次角度值 */
static uint8_t s_stableCount = 0;           /* 稳定计数 */

/* 循迹稳定检测 */
static uint8_t s_lastTrackData = 0;         /* 上次循迹数据 */
static uint8_t s_trackStableCount = 0;      /* 循迹稳定计数 */

/*====================================================================================*/
/*                                  私有函数                                           */
/*====================================================================================*/

/**
  * @brief  设置蜂鸣器频率和音量
  * @param  freq: 频率 (Hz), 0表示静音
  * @param  volume: 音量 (0-100)
  */
static void Buzzer_SetTone(uint16_t freq, uint8_t volume)
{
    if (freq == 0 || volume == 0)
    {
        PWM_SetDuty(0);
    }
    else
    {
        PWM_SetFreq(freq);
        PWM_SetDuty(volume);
    }
}

/**
  * @brief  播放当前音符
  */
static void Buzzer_PlayCurrentNote(void)
{
    const BuzzerNote_t *note;
    uint8_t volume;
    
    if (s_currentSequence == NULL || s_noteIndex >= s_currentSequence->noteCount)
    {
        return;
    }
    
    note = &s_currentSequence->notes[s_noteIndex];
    
    /* 计算实际音量 (序列音量 * 全局音量 / 100) */
    volume = (uint8_t)((uint16_t)s_currentSequence->volume * s_globalVolume / 100);
    
    Buzzer_SetTone(note->frequency, volume);
}

/*====================================================================================*/
/*                                  初始化函数                                         */
/*====================================================================================*/

/**
  * @brief  蜂鸣器初始化
  */
void Buzzer_Init(void)
{
    /* 初始化PWM */
    PWM_Init(BUZZER_DEFAULT_FREQ);
    PWM_SetDuty(0);  /* 初始关闭 */
    
    /* 初始化状态 */
    s_buzzerState = BUZZER_IDLE;
    s_currentSequence = NULL;
    s_noteIndex = 0;
    s_noteTimer = 0;
    s_globalVolume = BUZZER_DEFAULT_VOLUME;
    
    s_timingStarted = 0;
    s_oneMinuteAlerted = 0;
    s_finishAlerted = 0;
    
    s_stableCount = 0;
    s_trackStableCount = 0;
}

/*====================================================================================*/
/*                                  蜂鸣器控制                                         */
/*====================================================================================*/

/**
  * @brief  播放指定类型的提示音
  * @param  type: 提示音类型
  */
void Buzzer_PlayBeep(BeepType_t type)
{
    if (type >= BEEP_MAX)
    {
        return;
    }
    
    /* 设置当前序列 */
    s_currentSequence = &s_beepSequences[type];
    s_noteIndex = 0;
    s_noteTimer = 0;
    s_buzzerState = BUZZER_PLAYING;
    
    /* 播放第一个音符 */
    Buzzer_PlayCurrentNote();
}

/**
  * @brief  触发蜂鸣器响一声 (激活提示)
  */
void Buzzer_BeepOnce(void)
{
    Buzzer_PlayBeep(BEEP_ACTIVATE);
}

/**
  * @brief  触发蜂鸣器响两声 (1分钟提醒)
  */
void Buzzer_BeepTwice(void)
{
    Buzzer_PlayBeep(BEEP_ONE_MINUTE);
}

/**
  * @brief  触发蜂鸣器响三声升调 (终点)
  */
void Buzzer_BeepTriple(void)
{
    Buzzer_PlayBeep(BEEP_FINISH);
}

void Buzzer_BeepObstacle(void)
{
    Buzzer_PlayBeep(BEEP_OBSTACLE);
}

/**
  * @brief  停止当前播放
  */
void Buzzer_Stop(void)
{
    Buzzer_SetTone(0, 0);
    s_buzzerState = BUZZER_IDLE;
    s_currentSequence = NULL;
    s_noteIndex = 0;
    s_noteTimer = 0;
}

/**
  * @brief  检查蜂鸣器是否正在播放
  * @retval 1=正在播放, 0=空闲
  */
uint8_t Buzzer_IsPlaying(void)
{
    return (s_buzzerState == BUZZER_PLAYING) ? 1 : 0;
}

/**
  * @brief  设置音量
  * @param  volume: 音量 (0-100)
  */
void Buzzer_SetVolume(uint8_t volume)
{
    if (volume > 100) volume = 100;
    s_globalVolume = volume;
}

/**
  * @brief  蜂鸣器状态机更新 (需要在主循环中调用)
  * @param  deltaMs: 距离上次调用的时间间隔(ms)
  */
void Buzzer_Update(uint32_t deltaMs)
{
    const BuzzerNote_t *note;
    
    if (s_buzzerState != BUZZER_PLAYING || s_currentSequence == NULL)
    {
        return;
    }
    
    s_noteTimer += deltaMs;
    
    /* 获取当前音符 */
    note = &s_currentSequence->notes[s_noteIndex];
    
    /* 检查当前音符是否播放完成 */
    if (s_noteTimer >= note->duration)
    {
        s_noteIndex++;
        s_noteTimer = 0;
        
        if (s_noteIndex >= s_currentSequence->noteCount)
        {
            /* 序列播放完成 */
            Buzzer_Stop();
        }
        else
        {
            /* 播放下一个音符 */
            Buzzer_PlayCurrentNote();
        }
    }
}

/*====================================================================================*/
/*                                  计时控制                                           */
/*====================================================================================*/

/**
  * @brief  获取计时是否已开始
  * @retval 1=已开始, 0=未开始
  */
uint8_t Buzzer_IsTimingStarted(void)
{
    return s_timingStarted;
}

/**
  * @brief  开始计时 (检测到有效循迹数据时调用)
  */
void Buzzer_StartTiming(void)
{
    if (!s_timingStarted)
    {
        s_timingStarted = 1;
        /* 激活时响一声 */
        Buzzer_BeepOnce();
    }
}

/**
  * @brief  重置蜂鸣器状态
  */
void Buzzer_Reset(void)
{
    Buzzer_Stop();
    
    s_timingStarted = 0;
    s_oneMinuteAlerted = 0;
    s_finishAlerted = 0;
    
    s_stableCount = 0;
    s_trackStableCount = 0;
    s_lastTrackData = 0;
    s_lastAngle[0] = 0;
    s_lastAngle[1] = 0;
    s_lastAngle[2] = 0;
}

/*====================================================================================*/
/*                                  条件检测                                           */
/*====================================================================================*/

/**
  * @brief  检查1分钟提醒
  * @param  runTimeMs: 运行时间(ms)
  * @retval 1=需要提醒, 0=不需要
  */
uint8_t Buzzer_CheckOneMinute(uint32_t runTimeMs)
{
    if (!s_oneMinuteAlerted && runTimeMs >= ONE_MINUTE_MS)
    {
        s_oneMinuteAlerted = 1;
        return 1;
    }
    return 0;
}

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
                           uint32_t runTimeMs)
{
    uint8_t trackCondition = 0;
    uint8_t gyroCondition = 0;
    uint8_t timeCondition = 0;
    
    /* 已经提醒过终点，不再检测 */
    if (s_finishAlerted)
    {
        return 0;
    }
    
    /* 条件1: 时间 > 1分30秒 */
    timeCondition = (runTimeMs >= FINISH_MIN_TIME_MS) ? 1 : 0;
    if (!timeCondition)
    {
        return 0;
    }
    
    /* 条件2: 循迹L2-L7被遮挡 (至少6个传感器) */
    /* trackData的bit1-bit6对应L2-L7 */
    if ((trackData & FINISH_TRACK_MASK) == FINISH_TRACK_MASK)
    {
        /* 检查循迹数据是否稳定 */
        if (trackData == s_lastTrackData)
        {
            s_trackStableCount++;
        }
        else
        {
            s_trackStableCount = 0;
        }
        s_lastTrackData = trackData;
        
        trackCondition = (s_trackStableCount >= FINISH_STABLE_COUNT) ? 1 : 0;
    }
    else
    {
        s_trackStableCount = 0;
        s_lastTrackData = trackData;
        return 0;
    }
    
    /* 条件3: 陀螺仪数据稳定 (5个周期内无变化) */
    if (abs(angleRoll - s_lastAngle[0]) <= FINISH_ANGLE_THRESHOLD &&
        abs(anglePitch - s_lastAngle[1]) <= FINISH_ANGLE_THRESHOLD &&
        abs(angleYaw - s_lastAngle[2]) <= FINISH_ANGLE_THRESHOLD)
    {
        s_stableCount++;
    }
    else
    {
        s_stableCount = 0;
    }
    
    s_lastAngle[0] = angleRoll;
    s_lastAngle[1] = anglePitch;
    s_lastAngle[2] = angleYaw;
    
    gyroCondition = (s_stableCount >= FINISH_STABLE_COUNT) ? 1 : 0;
    
    /* 所有条件满足 */
    if (trackCondition && gyroCondition && timeCondition)
    {
        s_finishAlerted = 1;
        return 1;
    }
    
    return 0;
}
