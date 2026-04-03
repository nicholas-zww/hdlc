#include <cstdio>

#include "osal/osal_queue.h"
#include "osal/osal_thread.h"
#include "osal/osal_mutex.h"

extern "C" {
#include "driver/usb_serial_jtag.h"
}

static osalThread_t usb_thread;
static osalMutex_t usb_write_mutex = {};
static bool usb_write_mutex_initialized = false;

static void init_usb_write_mutex()
{
    if (!usb_write_mutex_initialized) {
        if (osalMutexInit(&usb_write_mutex) == OSAL_STATUS_OK) {
            usb_write_mutex_initialized = true;
        }
    }
}

void usb_task(void* arg)
{
    osalQueue_t* uartQueue = static_cast<osalQueue_t*>(arg);
    uint8_t ch;

    while (true)
    {
        int len = usb_serial_jtag_read_bytes(&ch, 1, OSAL_WAIT_FOREVER);
        if (len > 0)
        {
            osalQueueSend(uartQueue, &ch, OSAL_WAIT_FOREVER);
        }
    }
}

// Initialize USB‑Serial‑JTAG driver and start task
void usb_init(osalQueue_t& uartQueue)
{
    init_usb_write_mutex();

    // USB Serial JTAG driver config
    usb_serial_jtag_driver_config_t config = {};
    config.tx_buffer_size = 256;  // size in bytes
    config.rx_buffer_size = 256;  // size in bytes

    // Install driver
    usb_serial_jtag_driver_install(&config);

    // Create the echo task
    const osalThreadAttr_t usbTaskAttr = {
        .stack_size_bytes = 4096,
        .priority = 10,
        .detached = true,
        .name = "usb_task",
    };
    osalThreadCreate(&usb_thread, usb_task, &uartQueue, &usbTaskAttr);
}

bool usb_write(const uint8_t* data, uint16_t len)
{
    // for (size_t i = 0; i < len; i++)
    // {
    //     printf("%02x ", data[i]);
    // }
    // printf("\n");

    if (usb_write_mutex_initialized
        && osalMutexLock(&usb_write_mutex, OSAL_WAIT_FOREVER) == OSAL_STATUS_OK) {
        usb_serial_jtag_write_bytes(data, len, OSAL_WAIT_FOREVER);

        if (usb_serial_jtag_wait_tx_done(OSAL_WAIT_FOREVER) != 0) {
            printf("\nusb_write failed\n");
        }

        (void)osalMutexUnlock(&usb_write_mutex);
    } else {
        usb_serial_jtag_write_bytes(data, len, OSAL_WAIT_FOREVER);

        if (usb_serial_jtag_wait_tx_done(OSAL_WAIT_FOREVER) != 0) {
            printf("\nusb_write failed\n");
        }
    }

    return true;
}
