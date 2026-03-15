#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "frame_processor.h"
#include <filesystem>

#define LOG_TAG "MainApp"
#include "logger.h"

#include "fileTransfer.h"


#define QUEUE_SIZE 256

extern void initUserdata(void);
extern void loggerInit();
extern void usb_write(const char* data, size_t len);
extern void uartInit(QueueHandle_t& uartQueue);
extern void startOta(const std::string& otaFile);

static fileTransferReq fileTransfer;


static void sendResponse(uint8_t *data, uint16_t length)
{
    if ((length < 1) || (data == NULL))
    {
        LOGE("Invalid payload data");
        return;
    }

    uint16_t outSzie = length * 2 + 8;
    uint8_t *outData = (uint8_t*)malloc(outSzie);

    if (outData == NULL)
    {
        LOGE("malloc failed");
        return;
    }

    uint16_t outlen = FrameProcessorCreateFrame(data, length, outData, outSzie);
    if (outlen > 0)
    {
        usb_write((const char*)outData, outlen);
    }
    else
    {
        LOGE("failed to package msg");
    }
    free(outData);
}

/**
 * Callback function for receiving valid parsed frames
 */
static bool frame_received_callback(const uint8_t* payload, uint16_t length)
{
    if ((length < 1) || (payload == NULL))
    {
        LOGE("Invalid payload data");
        return false;
    }

    switch (payload[0])
    {
        case FILE_PACKAGE_ID_REQ:
            if (sizeof(fileDataRequest_t) == length) {
                fileDataRequest_t *req = (fileDataRequest_t *)payload;
                auto resp = (package_t*)malloc(sizeof(package_t));
                if (resp != nullptr) {
                    resp->seqId = req->reqId;
                    auto ret = fileTransfer.getTransferData((package_t*)resp);
                    if (ret == fileTransferReq::NO_ERR) {
                        resp->cmd = FILE_PACKAGE_DATA;
                        sendResponse((uint8_t*)(resp), sizeof(package_t));
                    }
                    else {
                        fileDataRequest_t endResp = {FILE_TRANSFER_DONE, static_cast<uint32_t>(ret)};
                        sendResponse((uint8_t*)(&endResp), sizeof(fileDataRequest_t));
                    }
                    free(resp);
                }
                else {
                    fileDataRequest_t endResp = {FILE_TRANSFER_DONE, static_cast<uint32_t>(FILE_TRANSFER_MEM_ERROR)};
                    sendResponse((uint8_t*)(&endResp), sizeof(fileDataRequest_t));
                }
            }
            else {
                LOGE("Invalid payload length %d", length);
            }
            break;

        case FILE_TRANSFER_GET:
            if (sizeof(fileGetRequest_t) == length) {
                fileGetRequest_t *req = (fileGetRequest_t *)payload;
                filePushRequest_t resp;
                auto ret = fileTransfer.raiseGetTransferReq(req, &resp);
                if (ret == fileTransferReq::NO_ERR) {
                    resp.cmd = FILE_TRANSFER_REQ;
                    sendResponse((uint8_t*)(&resp), sizeof(filePushRequest_t));
                }
                else {
                    fileDataRequest_t endResp = {FILE_TRANSFER_DONE, static_cast<uint32_t>(ret)};
                    sendResponse((uint8_t*)(&endResp), sizeof(fileDataRequest_t));
                }
            }
            else {
                LOGE("Invalid payload length %d", length);
            }
            break;

        case FILE_TRANSFER_REQ:
            if (sizeof(filePushRequest_t) == length) {
                filePushRequest_t *req = (filePushRequest_t *)payload;
                auto ret = fileTransfer.raisePushTransferReq(req);
                fileDataRequest_t resp;
                if (ret == fileTransferReq::NO_ERR) {
                    resp.cmd = FILE_PACKAGE_ID_REQ;
                    resp.reqId = 0;
                }
                else {
                    resp.cmd = FILE_TRANSFER_DONE;
                    resp.reqId = static_cast<uint32_t>(ret);
                }
                sendResponse((uint8_t*)(&resp), sizeof(fileDataRequest_t));
            }
            else {
                LOGE("Invalid payload length %d", length);
            }
            break;

        case FILE_PACKAGE_DATA:
            if (sizeof(package_t) == length) {
                package_t *pack = (package_t*)payload;

                fileDataRequest_t resp;
                auto ret = fileTransfer.pushTransferData(pack);
                if (ret == fileTransferReq::NO_ERR) {
                    resp.cmd = FILE_PACKAGE_ID_REQ;
                    resp.reqId = pack->seqId+1;
                }
                else if (ret == fileTransferReq::TRANSFER_COMPLETE) {
                    resp.cmd = FILE_TRANSFER_DONE;
                    resp.reqId = 0;
                }
                else {
                    resp.cmd = FILE_TRANSFER_DONE;
                    resp.reqId = static_cast<uint32_t>(ret);
                }
                sendResponse((uint8_t*)(&resp), sizeof(fileDataRequest_t));
            }
            else {
                LOGE("Invalid payload length %d", length);
            }
            break;

        case OTA_UPGRADE_REQ:
            if (!std::filesystem::exists("/userdata/firmware.bin")) {
                uint8_t resp[] = {static_cast<uint8_t>(OTA_UPGRADE_RESP), static_cast<uint8_t>(OTA_UPGRADE_FILE_MISSING)};
                sendResponse((uint8_t*)(&resp), sizeof(resp));
            }
            else {
                startOta("/userdata/firmware.bin");
            }
        default:
            break;
    }

    return true;
}

void task2(void *pvParameters)
{
    while (true)
    {
        // LOGI("Task 2 running on core %d", xPortGetCoreID());
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
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

    QueueHandle_t uartQueue = xQueueCreate(QUEUE_SIZE, sizeof(char));
    if (uartQueue == nullptr) {
        LOGE("ERROR: Failed to create UART queue!");
        while (1) { }
    }

    uartInit(uartQueue);

    initUserdata();

    FrameProcessorInit(uartQueue, frame_received_callback, 256, 4096, 5);
    xTaskCreate(task2, "Task2", 4096, NULL, 5, NULL);

    listFiles("/userdata");

}