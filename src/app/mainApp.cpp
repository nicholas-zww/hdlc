#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "frame_processor.h"

#define LOG_TAG "MainApp"
#include "logger.h"

#define QUEUE_SIZE 256

extern void loggerInit();
extern void usb_write(const char* data, size_t len);
extern void uartInit(QueueHandle_t& uartQueue);

/**
 * Callback function for receiving valid parsed frames
 */
static bool frame_received_callback(const uint8_t* payload, uint16_t length)
{
    printf("Receive Data: ");
    for (uint16_t i = 0; i < length && i < 32; i++) {
        printf("%02X ", payload[i]);
    }
    if (length > 32) {
        printf("... (%u more bytes)", length - 32);
    }
    printf("\n");

    uint8_t outData[32] = {0};
    uint16_t outlen = FrameProcessorCreateFrame(payload, length, outData, sizeof(outData));
    if (outlen > 0)
    {
        usb_write((const char*)outData, outlen);
    }
    return true;
}

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

    FrameProcessorInit(uartQueue, frame_received_callback, 256, 4096, 5);
    xTaskCreate(task2, "Task2", 4096, NULL, 5, NULL);
}