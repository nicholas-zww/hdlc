#include <stdio.h>
#include "osal/osal.h"
#include "frame_processing.h"
#include <filesystem>
#include <cstring>
#include "powerManagementTask.h"
#include "uiTask.h"
#include "wifiManagerTask.h"

#define LOG_TAG "MainApp"
#include "logger.h"



#define QUEUE_SIZE 256
constexpr uint8_t APP_COMMAND_SET_WIFI_CREDENTIALS = 0x30;
constexpr uint8_t WIFI_SSID_MAX_LEN = 32;
constexpr uint8_t WIFI_PASSWORD_MAX_LEN = 64;

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
    if (command == APP_COMMAND_SET_WIFI_CREDENTIALS) {
        if (payload == nullptr || payloadLength < 2U) {
            LOGW("SET_WIFI_CREDENTIALS invalid payload");
            return false;
        }

        const uint8_t ssidLen = payload[0];
        const uint8_t passwordLen = payload[1];
        const uint16_t expectedLen = static_cast<uint16_t>(2U + ssidLen + passwordLen);
        if (ssidLen > WIFI_SSID_MAX_LEN || passwordLen > WIFI_PASSWORD_MAX_LEN || payloadLength != expectedLen) {
            LOGW("SET_WIFI_CREDENTIALS invalid lengths ssid=%u pass=%u payload=%u",
                 static_cast<unsigned>(ssidLen),
                 static_cast<unsigned>(passwordLen),
                 static_cast<unsigned>(payloadLength));
            return false;
        }

        char ssid[WIFI_SSID_MAX_LEN + 1] = {};
        char password[WIFI_PASSWORD_MAX_LEN + 1] = {};

        if (ssidLen > 0U) {
            std::memcpy(ssid, &payload[2], ssidLen);
        }
        if (passwordLen > 0U) {
            std::memcpy(password, &payload[2U + ssidLen], passwordLen);
        }

        const bool applied = wifiSetCredentials(ssid, password);
        LOGI("SET_WIFI_CREDENTIALS result=%s", applied ? "ok" : "failed");
        return applied;
    }

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

    if (!startWifiManagerTask()) {
        LOGE("ERROR: Failed to start wifi manager task");
    }

    listFiles("/userdata");

}
