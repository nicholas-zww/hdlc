#include <stdio.h>
#include "osal/osal.h"
#include "frame_processing.h"
#include <filesystem>
#include "powerManagementTask.h"
#include "uiTask.h"

#define LOG_TAG "MainApp"
#include "logger.h"



#define QUEUE_SIZE 256

extern void initUserdata(void);
extern void loggerInit();
extern bool usb_write(const uint8_t* data, uint16_t len);
extern void uartInit(osalQueue_t& uartQueue);
extern void startOta(const std::string& otaFile);

/**
 * Callback function for receiving valid parsed frames
 */
static bool frame_received_callback(uint8_t command, const uint8_t *payload, uint16_t payloadLength)
{

    return true;
}

void listFiles(const std::string& path) {
    try {
        if (std::filesystem::exists(path) && std::filesystem::is_directory(path)) {
            for (const auto& entry : std::filesystem::directory_iterator(path)) {
                // entry.path().filename() gives just "file.txt"
                // entry.path() gives the full path "logs/file.txt"
                LOGI("File: %s", entry.path().filename().c_str());
            }
        }
    } catch (const std::filesystem::filesystem_error& e) {
        LOGE("Error: %s", e.what());
    }
}

void app_entry()
{
    loggerInit();

    static osalQueue_t uartQueue = {};

    if (osalQueueInit(&uartQueue, sizeof(char), QUEUE_SIZE) != OSAL_STATUS_OK) {
        LOGE("ERROR: Failed to create UART queue!");
        while (1) { }
    }

    uartInit(uartQueue);

    initUserdata();

    fipcFrameProcessingSetFrameReceivedCallback(frame_received_callback);
    fipcFrameProcessingInit(&uartQueue, usb_write);

    if (!startUiTask()) {
        LOGE("ERROR: Failed to start UI task");
    }

    if (!startPowerManagementTask()) {
        LOGE("ERROR: Failed to start power management task");
    }

    listFiles("/userdata");

}
