#ifndef MAIN_H
#define MAIN_H

#include <Arduino.h>
#include <stdio.h>
#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

// ---------- 硬件引脚 ----------
#define PIN_VOICE_AND_LED  13
#define PIN_LASER          12
#define ON                 1
#define OFF                0

// ---------- 全局变量 ----------
extern volatile uint8_t RECEIVE_FLAG;
extern volatile uint8_t RECEIVE_FLAG_Lora;

// ---------- FreeRTOS 资源 ----------
typedef struct {
    uint8_t num;
    uint16_t mseconds;
} BeepRequest_t;

extern QueueHandle_t beepQueue;
extern SemaphoreHandle_t lxTxMutex;
extern SemaphoreHandle_t loraTxMutex;

// ---------- 数据结构 ----------
typedef struct {
    double rol, pit, yaw_part, yaw_last, yaw_round; // 姿态角（度）
    int16_t vel_x, vel_y;                            // 速度 cm/s
    int32_t hight;                                   // 高度 cm
    int16_t dis_x, dis_y, dis_x___, dis_y___, yaw;   // 累计距离 cm, yaw 原始值
} ANO_info_st;

typedef struct {
    uint8_t slam_Valid;
    int16_t real_P_X;
    int16_t real_P_Y;
    int16_t Target_P_X;
    int16_t Target_P_Y;
    uint16_t real_Angle;
    int16_t real_Distance;
    uint16_t obstacle_Angle;
    int32_t obstacle_Distance;
} Slam_data;

// ---------- 函数声明 ----------
void Beep(uint8_t num, uint16_t mseconds);
void Beep_Async(uint8_t num, uint16_t mseconds);
void laser_flash(uint8_t num, uint16_t mseconds);

// 数据解析
void data_anl_LX(uint8_t *data, uint8_t data_len);

#endif
