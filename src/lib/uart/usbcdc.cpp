#include <cstdio>

extern "C" {
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/usb_serial_jtag.h"
}

void usb_task(void* arg)
{
    QueueHandle_t uartQueue = static_cast<QueueHandle_t>(arg);
    uint8_t ch;

    while (true)
    {
        int len = usb_serial_jtag_read_bytes(&ch, 1, portMAX_DELAY);
        if (len > 0)
        {
            xQueueSend(uartQueue, &ch, portMAX_DELAY);
            // usb_serial_jtag_write_bytes((const char*)&ch, 1, portMAX_DELAY);
        }
    }
}

// Initialize USB‑Serial‑JTAG driver and start task
void usb_init(QueueHandle_t& uartQueue)
{
    // USB Serial JTAG driver config
    usb_serial_jtag_driver_config_t config = {};
    config.tx_buffer_size = 256;  // size in bytes
    config.rx_buffer_size = 256;  // size in bytes

    // Install driver
    usb_serial_jtag_driver_install(&config);

    // Create the echo task
    xTaskCreate(
        usb_task,
        "usb_task",
        4096,
        (void*)uartQueue,
        10,
        nullptr
    );
}

void usb_write(const char* data, size_t len)
{
    for (size_t i = 0; i < len; i++)
    {
        printf("%02x ", data[i]);
    }
    printf("\n");

    usb_serial_jtag_write_bytes(data, len, portMAX_DELAY);

    if (usb_serial_jtag_wait_tx_done(portMAX_DELAY) != 0)
        printf("\nusb_write failed\n");
}