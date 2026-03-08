#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "hdlc_parser.h"
#include <string>
#include <iomanip>
#include <sstream>

#define LOG_TAG "HDLC_Handler"
#include "logger.h"

extern void uart_write(const char* data, size_t len);

static void hdlc_frame_callback(const hdlc_frame_t* frame, bool crc_valid) {
    LOGI("[TASK] HDLC Frame - ");
    LOGI("ADDR: 0x{:02X}", frame->address);
    LOGI(" CTRL: 0x{:02X}", frame->control);
    LOGI(" LEN: {}", frame->data_length);

    if (crc_valid) {
        // Process the frame based on control byte
        // Control byte 0x03 = UI (Unnumbered Information)
        if (frame->control == 0x03) {
            LOGI("Received Data %s", [&frame]() -> std::string {
                    std::stringstream ss;
                    for (size_t i = 0; i < frame->data_length; ++i) {
                        ss << std::hex << frame->data[i];
                        if (i != frame->data_length - 1) ss << " ";
                    }
                    return ss.str();
                }().c_str()
            );

            // Echo back with different control byte (acknowledgment)
            uint8_t response_data[HDLC_MAX_DATA_LEN];
            uint16_t response_len = hdlc_packFrame(frame->address, 0x07, frame->data, frame->data_length, response_data, sizeof(response_data));
            if (response_len > 0) {
                uart_write((const char*)response_data, response_len);
            }
        }
    }
    else {
        LOGE("CRC mismatch - frame corrupted");
    }
}

void serialDataHandler(void *pvParameters)
{
    QueueHandle_t uartQueue = static_cast<QueueHandle_t>(pvParameters);
    hdlc_parser_init();
    hdlc_parser_set_callback(hdlc_frame_callback);

    uint8_t receivedChar;

    for (;;) {
        if (xQueueReceive( uartQueue, &receivedChar, portMAX_DELAY ) == pdPASS) {
            // Feed character to HDLC parser
            LOGI("Uart Data %02x", receivedChar);
            hdlc_parser_process_char((uint8_t)receivedChar);
        }
    }
}