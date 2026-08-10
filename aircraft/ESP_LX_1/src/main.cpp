#include <Arduino.h>
#include "main.h"
#include "uart_lora.h"
#include "uart_lx.h"
#include "esp_task_wdt.h"

// ---------- 全局变量定义（只有这一处） ----------
volatile uint8_t RECEIVE_FLAG = 0;
volatile uint8_t RECEIVE_FLAG_Lora = 0;

// ---------- FreeRTOS 资源 ----------
QueueHandle_t beepQueue = nullptr;
SemaphoreHandle_t lxTxMutex = nullptr;
SemaphoreHandle_t loraTxMutex = nullptr;

static TaskHandle_t lxRxTaskHandle = nullptr;
static TaskHandle_t loraRxTaskHandle = nullptr;
static TaskHandle_t beepTaskHandle = nullptr;

// ---------- FreeRTOS 任务声明 ----------
static void task_receive_lx(void *pvParameters);
static void task_receive_lora(void *pvParameters);
static void task_beep(void *pvParameters);

void setup()
{
    pinMode(PIN_VOICE_AND_LED, OUTPUT);
    pinMode(PIN_LASER, OUTPUT);
    digitalWrite(PIN_VOICE_AND_LED, OFF);
    digitalWrite(PIN_LASER, OFF);

    Serial.begin(115200);

    /*
     * 注意：
     * 如果你当前工程使用 LX.setPins(LX_TX, LX_RX) 能正常通信，就先保持不变。
     * 部分 ESP32 Arduino Core 版本中 setPins 的参数顺序是 RX, TX。
     * 若后续串口完全无数据，请优先核对这里的 TX/RX 参数顺序。
     */
    LX.setPins(LX_TX, LX_RX);
    LX.begin(115200);
    Serial.printf("Uart-LX Init\n");

    Lora.setPins(Lora_TX, Lora_RX);
    Lora.begin(115200);
    Serial.printf("Uart-Lora Init\n");

    // 串口发送互斥锁：避免多个任务同时向 LX 串口写数据
    lxTxMutex = xSemaphoreCreateMutex();

    // ACK、任务状态和遥测共用 LoRa 发送通道，必须保证整帧原子发送。
    loraTxMutex = xSemaphoreCreateMutex();

    // 蜂鸣器队列：让接收任务只投递事件，不直接阻塞等待蜂鸣完成
    beepQueue = xQueueCreate(8, sizeof(BeepRequest_t));

    if (lxTxMutex == nullptr || loraTxMutex == nullptr || beepQueue == nullptr) {
        Serial.println("[RTOS] Mutex or queue create failed");
    }

    /*
     * 任务划分：
     * 1. LX_RX：专门接收飞控端 10ms 周期数据
     * 2. LORA_RX：专门接收无线串口 LoRa 数据
     * 3. BEEP：专门处理蜂鸣器，避免蜂鸣器延时阻塞接收任务
     *
     * ESP32-S3 是双核。这里把 LX 和 LoRa 放在不同核心，降低互相影响。
     * 如果你的工程后续加入 WiFi/蓝牙，可考虑把两个接收任务都放到 core 1。
     */
    xTaskCreatePinnedToCore(task_receive_lx,   "LX_RX",   4096, nullptr, 3, &lxRxTaskHandle,   1);
    xTaskCreatePinnedToCore(task_receive_lora, "LORA_RX", 4096, nullptr, 3, &loraRxTaskHandle, 0);
    xTaskCreatePinnedToCore(task_beep,         "BEEP",    2048, nullptr, 1, &beepTaskHandle,   1);

    Serial.println("[RTOS] Tasks started");
}

void loop()
{
    // loop 不再做串口接收，只作为 Arduino 主任务保活
    vTaskDelay(pdMS_TO_TICKS(20));

#if CONFIG_TASK_WDT
    esp_task_wdt_reset();
#endif
}

// ---------- LX 接收任务 ----------
static void task_receive_lx(void *pvParameters)
{
    (void)pvParameters;

    while (true) {
        data_receive_LX();
        lora_protocol_service();

        // 主动让出 CPU；1ms 足够高频，不会影响 10ms 周期帧
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

// ---------- LoRa 接收任务 ----------
static void task_receive_lora(void *pvParameters)
{
    (void)pvParameters;

    while (true) {
        data_receive_Lora();

        // 主动让出 CPU；避免 while available 结束后立刻空转
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

// ---------- 蜂鸣器任务 ----------
static void task_beep(void *pvParameters)
{
    (void)pvParameters;

    BeepRequest_t req;

    while (true) {
        if (xQueueReceive(beepQueue, &req, portMAX_DELAY) == pdTRUE) {
            Beep(req.num, req.mseconds);
        }
    }
}

// ---------- 异步蜂鸣：接收任务调用这个，不要直接调用 Beep ----------
void Beep_Async(uint8_t num, uint16_t mseconds)
{
    if (beepQueue == nullptr) {
        return;
    }

    BeepRequest_t req;
    req.num = num;
    req.mseconds = mseconds;

    // 队列满了就丢弃本次蜂鸣，不能反过来阻塞接收任务
    xQueueSend(beepQueue, &req, 0);
}

// ---------- 蜂鸣器：只允许 BEEP 任务里长时间调用 ----------
void Beep(uint8_t num, uint16_t mseconds)
{
    if (num == 0) {
        return;
    }

    TickType_t delayTicks = pdMS_TO_TICKS(mseconds);
    if (delayTicks == 0) {
        delayTicks = 1;
    }

    for (uint8_t i = 0; i < num; i++) {
        digitalWrite(PIN_VOICE_AND_LED, ON);
        vTaskDelay(delayTicks);
        digitalWrite(PIN_VOICE_AND_LED, OFF);

        if (i + 1 < num) {
            vTaskDelay(delayTicks);
        }
    }
}

// ---------- 激光闪烁 ----------
void laser_flash(uint8_t num, uint16_t mseconds)
{
    TickType_t delayTicks = pdMS_TO_TICKS(mseconds);
    if (delayTicks == 0) {
        delayTicks = 1;
    }

    if (num == 1) {
        digitalWrite(PIN_LASER, OFF);
        vTaskDelay(delayTicks);
        digitalWrite(PIN_LASER, ON);
    } else {
        for (int i = 0; i < num; i++) {
            digitalWrite(PIN_LASER, OFF);
            vTaskDelay(delayTicks);
            digitalWrite(PIN_LASER, ON);
            vTaskDelay(delayTicks);
        }
    }
}
