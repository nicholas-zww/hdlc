#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#define LOG_TAG "MainApp"
#include "logger.h"

#define QUEUE_SIZE 256

extern void loggerInit();
extern void uartInit(QueueHandle_t& uartQueue);
extern void serialDataHandler(void *pvParameters);

void task2(void *pvParameters)
{
    while (true)
    {
        LOGI("Task 2 running on core %d", xPortGetCoreID());
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

void app_entry()
{
    loggerInit();
    QueueHandle_t uartQueue = xQueueCreate(QUEUE_SIZE, sizeof(char));
    if (uartQueue == nullptr) {
        LOGE("ERROR: Failed to create UART queue!");
        while (1) { }
    }

    uartInit(uartQueue);
    xTaskCreate(serialDataHandler, "serialDataHandler", 4096, (void*)uartQueue, 5, NULL);
    xTaskCreate(task2, "Task2", 4096, NULL, 5, NULL);
}