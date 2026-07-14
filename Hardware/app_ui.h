#ifndef __APP_UI_H
#define __APP_UI_H

#include "stm32f10x.h"

/*====================================================================================*/
/*                                  页面定义                                           */
/*====================================================================================*/

typedef enum {
    UI_PAGE_MOTOR_TEST = 0, // 电机标定页(临时，用于测量)
    UI_PAGE_OVERVIEW,       // 综合信息页
    UI_PAGE_TRACKING,       // 循迹详情页
    UI_PAGE_GYROSCOPE,      // 陀螺仪详情页
    UI_PAGE_COUNT           // 页面总数
} UI_Page_t;

/*====================================================================================*/
/*                                  循迹分析结构                                       */
/*====================================================================================*/

typedef struct {
    uint8_t rawData;            // 原始8位数据
    int8_t offset;              // 偏移量 (-4 ~ +4, 0=中心)
    uint8_t lineWidth;          // 检测到的线宽 (连续1的个数)
    uint8_t edgePosition;       // 边缘位置: 0=无, 1=左, 2=中, 3=右, 4=全
    uint8_t sensorStatus[8];    // 每个传感器状态 (0/1)
} TrackingAnalysis_t;

extern TrackingAnalysis_t g_trackAnalysis;

/*====================================================================================*/
/*                                  函数声明                                           */
/*====================================================================================*/

/**
  * @brief  UI模块初始化
  */
void UI_Init(void);

/**
  * @brief  切换到下一页 (短按)
  */
void UI_NextPage(void);

/**
  * @brief  获取当前页面
  * @retval 当前页面枚举值
  */
UI_Page_t UI_GetCurrentPage(void);

/**
  * @brief  设置当前页面
  * @param  page: 目标页面
  */
void UI_SetPage(UI_Page_t page);

/**
  * @brief  更新循迹分析数据
  * @param  trackData: 8位循迹原始数据
  */
void UI_UpdateTrackingAnalysis(uint8_t trackData);

/**
  * @brief  显示当前页面
  * @param  trackData: 循迹数据
  * @param  trackOK: 循迹读取是否成功
  */
void UI_Display(uint8_t trackData, uint8_t trackOK);

/**
  * @brief  显示综合信息页
  */
void UI_DisplayOverview(uint8_t trackData, uint8_t trackOK);

/**
  * @brief  显示循迹详情页
  */
void UI_DisplayTracking(uint8_t trackData, uint8_t trackOK);

/**
  * @brief  显示陀螺仪详情页
  */
void UI_DisplayGyroscope(void);

void UI_SetObstacleMode(uint8_t enable);
uint8_t UI_GetObstacleMode(void);
void UI_ShowObstacleHint(uint8_t enable);
void UI_Tick(uint32_t deltaMs);
void UI_RequestRefresh(void);

void UI_SetUltrasonicDistance(float distanceCm);

/**
  * @brief  显示统计页 (长按触发)
  */
void UI_DisplayStats(void);

/**
  * @brief  触发显示统计页
  */
void UI_ShowStats(void);

/**
  * @brief  退出统计页
  */
void UI_HideStats(void);

/**
  * @brief  检查是否在统计页
  */
uint8_t UI_IsShowingStats(void);

void UI_SetFinishMode(uint8_t enable);
uint8_t UI_IsFinishMode(void);

typedef struct {
    uint8_t valid;
    uint16_t pwmMinLeft;
    uint16_t pwmMinRight;
    float speedDiffMms; /* left - right, mm/s */
} UI_MotorTest_t;

void UI_SetMotorTest(uint16_t pwmMinLeft, uint16_t pwmMinRight, float speedDiffMms);
void UI_ClearMotorTest(void);

#endif /* __APP_UI_H */
