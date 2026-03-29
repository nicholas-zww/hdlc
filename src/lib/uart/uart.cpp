#include <stdio.h>
#include <stdarg.h>
#include "osal/osal_queue.h"
#include "osal/osal_thread.h"

extern "C" {
#include "driver/uart.h"
#include "driver/gpio.h"
}

#define TAG "UART"

#define USING_LOG_UART

#define UART_PORT UART_NUM_1
#define BUF_SIZE 1024
#define UART_TX_PIN 17
#define UART_RX_PIN 18

static const uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT
};

#ifdef USING_LOG_UART
extern "C" {
#include "esp_log.h"
}

extern void usb_init(osalQueue_t& uartQueue);

static int uart_log_vprintf(const char *fmt, va_list args)
{
    char buffer[256];
    int len = vsnprintf(buffer, sizeof(buffer), fmt, args);

    if (len > 0) {
        uart_write_bytes(UART_PORT, buffer, len);
    }

    return len;
}
#else
static QueueHandle_t uart_queue;
static osalThread_t uart_task_thread;

void uart_event_task(void *pvParameters)
{
    uart_event_t event;
    uint8_t byte;

    while (true)
    {
        if (xQueueReceive(uart_queue, (void *)&event, portMAX_DELAY))
        {
            if (event.type == UART_DATA)
            {
                // Read byte-by-byte
                while (uart_read_bytes(UART_PORT, &byte, 1, 0) == 1)
                {
                    printf("Received byte: 0x%02X (%c)\n", byte, byte);
                }
            }

            if (event.type == UART_FIFO_OVF)
            {
                printf("UART FIFO Overflow\n");
                uart_flush_input(UART_PORT);
                xQueueReset(uart_queue);
            }

            if (event.type == UART_BUFFER_FULL)
            {
                printf("UART Buffer Full\n");
                uart_flush_input(UART_PORT);
                xQueueReset(uart_queue);
            }
        }
    }
}
#endif

#ifdef USING_LOG_UART
void loggerInit()
{
    uart_param_config(UART_PORT, &uart_config);

    // Set UART pins
    uart_set_pin(UART_PORT, UART_TX_PIN, UART_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(UART_PORT, 1024, 0, 0, NULL, 0);

    // Redirect ESP_LOG output
    esp_log_set_vprintf(uart_log_vprintf);
}

void uartInit(osalQueue_t& uartQueue)
{
    usb_init(uartQueue);
}

void uart_write(const char* data, size_t len)
{
    uart_write_bytes(UART_PORT, data, len);
}

#else
void loggerInit()
{

}

void uart_write(const char* data, size_t len)
{
    uart_write_bytes(UART_PORT, data, len);
}

void uartInit(osalQueue_t& uartQueue)
{
    (void)uartQueue;
    uart_param_config(UART_PORT, &uart_config);

    // Set UART pins
    uart_set_pin(UART_PORT, UART_TX_PIN, UART_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    // Install UART driver
    uart_driver_install(UART_PORT, BUF_SIZE * 2, BUF_SIZE * 2, 10, &uart_queue, 0);


    // Trigger interrupt when at least 1 byte arrives
    uart_set_rx_full_threshold(UART_PORT, 1);

    // Create task
    const osalThreadAttr_t uartTaskAttr = {
        .stack_size_bytes = 4096,
        .priority = 12,
        .detached = true,
        .name = "uart_event_task",
    };
    osalThreadCreate(&uart_task_thread, uart_event_task, NULL, &uartTaskAttr);
}
#endif
