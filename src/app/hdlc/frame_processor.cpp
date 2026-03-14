#include "frame_processor.h"

#include <stdlib.h>
#include <string.h>

#define LOG_TAG "FrameProcessor"
#include "logger.h"

/* ============================================================================
   INTERNAL STRUCTURE (Hidden from users)
   ============================================================================ */
#define FRAME_PROCESSOR_MAX_PAYLOAD(x)       (x - 6)
typedef struct
{
    /* FreeRTOS handles */
    QueueHandle_t queue;
    TaskHandle_t taskHandle;

    /* Callback and control */
    FrameProcessor_SendCallback callback;
    volatile bool isRunning;

    /* Parser state machine */
    uint8_t state;
    bool escapePending;

    /* Frame buffer and parsing state */
    uint8_t* buffer;
    uint16_t bufferSize;
    uint16_t payloadLen;
    uint16_t bufferIdx;
    uint16_t receivedCRC;
    uint8_t crcHByte;
} FrameProcessor_t;


/* ============================================================================
   GLOBAL SINGLETON INSTANCE
   ============================================================================ */
static FrameProcessor_t g_processor = {.queue = NULL,
                                       .taskHandle = NULL,
                                       .callback = NULL,
                                       .isRunning = false,
                                       .state = FRAME_PROCESSOR_STATE_IDLE,
                                       .escapePending = false,
                                       .buffer = NULL,
                                       .bufferSize = 0,
                                       .payloadLen = 0,
                                       .bufferIdx = 0,
                                       .receivedCRC = 0,
                                       .crcHByte = 0};

/* ============================================================================
   INTERNAL STATIC HELPER FUNCTIONS
   ============================================================================ */
/**
 * @brief Calculate CRC-16-CCITT checksum.
 *
 * Uses polynomial 0x1021 with initial value 0xFFFF and no final XOR.
 * This CRC is used to verify the integrity of frame payload data.
 *
 * @param data Pointer to input data buffer.
 * @param length Number of bytes to process.
 *
 * @return Calculated CRC16 value.
 */
static uint16_t FrameProcessorCalculateCrc(const uint8_t* data, size_t length)
{
    uint16_t crc = 0xFFFF;

    while (length--)
    {
        crc ^= (uint16_t)(*data++) << 8;

        for (uint8_t i = 0; i < 8; i++)
        {
            crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : (crc << 1);
        }
    }

    return crc;
}

/**
 * @brief Reset the internal frame parser state machine.
 *
 * Clears parser state variables and prepares the processor
 * to receive a new frame starting from the IDLE state.
 *
 * This function does not clear the payload buffer contents,
 * only resets indices and parsing flags.
 */
static void FrameProcessorResetParser(void)
{
    g_processor.state = FRAME_PROCESSOR_STATE_IDLE;
    g_processor.escapePending = false;
    g_processor.payloadLen = 0;
    g_processor.bufferIdx = 0;
    g_processor.receivedCRC = 0;
    g_processor.crcHByte = 0;
    // memset(g_processor.buffer, 0, g_processor.bufferSize);
}

/**
 * @brief Convert an escaped byte back to its original value.
 *
 * When byte stuffing is used, special bytes are transmitted
 * with an escape character followed by an escape code.
 * This function restores the original byte value.
 *
 * @param byte Escaped byte value.
 *
 * @return Original byte value, or 0xFF if the escape code is invalid.
 */
static uint8_t FrameProcessorUnescape(uint8_t byte)
{
    switch (byte)
    {
        case FRAME_PROCESSOR_ESCAPE_ESCAPE:
            return FRAME_PROCESSOR_ESCAPE_CHAR;
        case FRAME_PROCESSOR_ESCAPE_STARTER:
            return FRAME_PROCESSOR_STARTER_FLAG;
        case FRAME_PROCESSOR_ESCAPE_END:
            return FRAME_PROCESSOR_END_FLAG;
        default:
            return 0xFF;
    }
}

/**
 * @brief Check whether a byte requires escaping.
 *
 * Certain control bytes (START, END, ESCAPE) must be escaped
 * when transmitted inside payload or CRC fields.
 *
 * @param byte Input byte to check.
 *
 * @return true if the byte must be escaped, otherwise false.
 */
static bool FrameProcessorByteCheckEscape(uint8_t byte)
{
    return byte == FRAME_PROCESSOR_STARTER_FLAG || byte == FRAME_PROCESSOR_END_FLAG ||
           byte == FRAME_PROCESSOR_ESCAPE_CHAR;
}

/**
 * @brief Get escape code for a control byte.
 *
 * Converts a control byte into its corresponding escape code
 * used during byte stuffing.
 *
 * @param byte Original control byte.
 *
 * @return Escape code corresponding to the input byte.
 */
static uint8_t FrameProcessorGetEscapeChar(uint8_t byte)
{
    if (byte == FRAME_PROCESSOR_ESCAPE_CHAR)
        return FRAME_PROCESSOR_ESCAPE_ESCAPE;
    if (byte == FRAME_PROCESSOR_STARTER_FLAG)
        return FRAME_PROCESSOR_ESCAPE_STARTER;
    if (byte == FRAME_PROCESSOR_END_FLAG)
        return FRAME_PROCESSOR_ESCAPE_END;
    return 0;
}

/**
 * @brief Verify payload integrity using CRC.
 *
 * Calculates CRC from the provided payload and compares
 * it with the received CRC value.
 *
 * @param payload Pointer to payload buffer.
 * @param length Payload length in bytes.
 * @param receivedCRC CRC value received in the frame.
 *
 * @return true if CRC matches, otherwise false.
 */
static bool FrameProcessorVerifyCrc(const uint8_t* payload, uint16_t length, uint16_t receivedCRC)
{
    uint16_t calculatedCRC = FrameProcessorCalculateCrc(payload, length);
    return calculatedCRC == receivedCRC;
}

/**
 * @brief Process a fully received frame.
 *
 * Verifies the frame CRC and invokes the registered
 * callback function if the frame is valid.
 *
 * Invalid frames are silently discarded.
 */
static void FrameProcessorProcessFrame(void)
{
    if (! FrameProcessorVerifyCrc(g_processor.buffer, g_processor.payloadLen, g_processor.receivedCRC))
    {
        return; /* Invalid CRC - silently skip */
    }

    if (g_processor.callback != NULL)
    {
        g_processor.callback(g_processor.buffer, g_processor.payloadLen);
    }
}

/**
 * @brief Parse incoming byte stream using a state machine.
 *
 * Processes each received byte and reconstructs frames
 * according to the protocol format:
 *
 * START -> LENGTH -> PAYLOAD -> CRC -> END
 *
 * Handles byte unescaping, payload buffering, and state transitions.
 *
 * @param byte Incoming byte from the data stream.
 *
 * @return true if parsing continues normally, false if an error occurred.
 */
static bool FrameProcessorParseFrame(uint8_t byte)
{
    /* Handle escaped bytes */
    if (g_processor.escapePending)
    {
        uint8_t unescapedByte = FrameProcessorUnescape(byte);
        if (unescapedByte == 0xFF)
        {
            FrameProcessorResetParser();
            LOGE("Incorrect unescapedByte : %02x-%02x", byte, unescapedByte);
            return false;
        }
        byte = unescapedByte;
        g_processor.escapePending = false;

        /* After unescaping, process based on current state */
        if (g_processor.state == FRAME_PROCESSOR_STATE_WAITING_PAYLOAD)
        {
            if (g_processor.bufferIdx >= (g_processor.bufferSize - 1))
            {
                FrameProcessorResetParser();
                LOGE("Buffer overflow: %d/%d", g_processor.bufferIdx, (g_processor.bufferSize - 1));
                return false;
            }
            g_processor.buffer[g_processor.bufferIdx++] = byte;
            if (g_processor.bufferIdx == g_processor.payloadLen)
            {
                g_processor.state = FRAME_PROCESSOR_STATE_WAITING_CRC_H;
            }
            return true;
        }
        else if (g_processor.state == FRAME_PROCESSOR_STATE_WAITING_CRC_H)
        {
            g_processor.crcHByte = byte;
            g_processor.state = FRAME_PROCESSOR_STATE_WAITING_CRC_L;
            return true;
        }
        else if (g_processor.state == FRAME_PROCESSOR_STATE_WAITING_CRC_L)
        {
            g_processor.receivedCRC = ((uint16_t)g_processor.crcHByte << 8) | byte;
            g_processor.state = FRAME_PROCESSOR_STATE_WAITING_END;
            return true;
        }
    }

    /* Check for escape character */
    if (byte == FRAME_PROCESSOR_ESCAPE_CHAR)
    {
        g_processor.escapePending = true;
        return true;
    }

    /* State machine */
    switch (g_processor.state)
    {
        case FRAME_PROCESSOR_STATE_IDLE:
            if (byte == FRAME_PROCESSOR_STARTER_FLAG)
            {
                FrameProcessorResetParser();
                g_processor.state = FRAME_PROCESSOR_STATE_WAITING_LENGTH_H;
            }
            break;

        case FRAME_PROCESSOR_STATE_WAITING_LENGTH_H:
            g_processor.payloadLen = (uint16_t)byte << 8;
            g_processor.state = FRAME_PROCESSOR_STATE_WAITING_LENGTH_L;
            break;

        case FRAME_PROCESSOR_STATE_WAITING_LENGTH_L:
            g_processor.payloadLen |= byte;
            if (g_processor.payloadLen == 0 || g_processor.payloadLen > FRAME_PROCESSOR_MAX_PAYLOAD(g_processor.bufferSize))
            {
                FrameProcessorResetParser();
                LOGE("Incorrect payload length %d", g_processor.payloadLen);
                return false;
            }
            g_processor.bufferIdx = 0;
            g_processor.state = FRAME_PROCESSOR_STATE_WAITING_PAYLOAD;
            break;

        case FRAME_PROCESSOR_STATE_WAITING_PAYLOAD:
            if (g_processor.bufferIdx >= (g_processor.bufferSize - 1))
            {
                FrameProcessorResetParser();
                LOGE("Buffer overflow: %d/%d", g_processor.bufferIdx, (g_processor.bufferSize - 1));
                return false;
            }
            g_processor.buffer[g_processor.bufferIdx++] = byte;
            if (g_processor.bufferIdx == g_processor.payloadLen)
            {
                g_processor.state = FRAME_PROCESSOR_STATE_WAITING_CRC_H;
            }
            break;

        case FRAME_PROCESSOR_STATE_WAITING_CRC_H:
            g_processor.crcHByte = byte;
            g_processor.state = FRAME_PROCESSOR_STATE_WAITING_CRC_L;
            break;

        case FRAME_PROCESSOR_STATE_WAITING_CRC_L:
            g_processor.receivedCRC = ((uint16_t)g_processor.crcHByte << 8) | byte;
            g_processor.state = FRAME_PROCESSOR_STATE_WAITING_END;
            break;

        case FRAME_PROCESSOR_STATE_WAITING_END:
            if (byte == FRAME_PROCESSOR_END_FLAG)
            {
                FrameProcessorProcessFrame();
            }
            FrameProcessorResetParser();
            break;
    }

    return true;
}

/**
 * @brief FreeRTOS task responsible for frame parsing.
 *
 * Continuously reads bytes from the input queue and feeds them
 * to the frame parser. The task runs as long as the processor
 * is marked as running.
 *
 * @param pvParameters Unused task parameter.
 */
static void FrameProcessorTaskEntry(void* pvParameters)
{
    (void)pvParameters; /* Unused - use global instance */
    uint8_t rxByte;
    TickType_t xTicksToWait = portMAX_DELAY;

    while (g_processor.isRunning)
    {
        if (xQueueReceive(g_processor.queue, &rxByte, xTicksToWait) == pdPASS)
        {
            FrameProcessorParseFrame(rxByte);
        }
    }

    vTaskDelete(NULL);
}

/**
 * @brief Copy data into output buffer with byte escaping.
 *
 * Performs byte stuffing for control characters before
 * placing them into the output buffer.
 *
 * @param inBuf Input data buffer.
 * @param inBufSize Number of bytes in the input buffer.
 * @param outBuf Output buffer where escaped data will be written.
 * @param outBufSize Size of the output buffer.
 *
 * @return Number of bytes written to the output buffer.
 *         Returns 0 if the output buffer is too small.
 */
static uint16_t FrameProcessorPushToBuffer(const uint8_t* inBuf, uint16_t inBufSize, uint8_t* outBuf, uint16_t outBufSize)
{
    uint16_t datalen = 0;
    for (uint16_t i = 0; i < inBufSize; i++)
    {
        uint8_t byte = inBuf[i];
        uint8_t escape = FrameProcessorByteCheckEscape(byte);

        uint8_t needed = escape ? 2 : 1;
        if (datalen + needed > outBufSize)
        {
            LOGE("Output frame buffer size(%d, %d) too small, skip", datalen + needed, outBufSize);
            return 0;
        }

        if (escape)
        {
            outBuf[datalen++] = FRAME_PROCESSOR_ESCAPE_CHAR;
            byte = FrameProcessorGetEscapeChar(byte);
        }

        outBuf[datalen++] = byte;
    }
    return datalen;
}

/* ============================================================================
   PUBLIC API
   ============================================================================ */
/**
 * @brief Initialize the frame processor module.
 *
 * Creates the processing task and prepares the internal parser
 * to receive frames from the provided queue.
 *
 * @param queue FreeRTOS queue used to receive incoming bytes.
 * @param callback Callback invoked when a valid frame is received.
 * @param frameBufferSize frame buffer size
 * @param stackSize Task stack size (bytes). Default used if 0.
 * @param priority FreeRTOS task priority. Default used if 0.
 *
 * @return true if initialization succeeds, otherwise false.
 */
bool FrameProcessorInit(QueueHandle_t queue, FrameProcessor_SendCallback callback, uint16_t frameBufferSize, uint16_t stackSize, UBaseType_t priority)
{
    if (queue == NULL || callback == NULL)
    {
        LOGE("FrameProcessorInit: failed with input arguments");
        return false;
    }

    if (g_processor.isRunning)
    {
        LOGE("FrameProcessor is already running");
        return false; /* Already running */
    }

    /* Use defaults if 0 provided */
    if (stackSize == 0)
    {
        stackSize = 2048;
    }
    if (priority == 0)
    {
        priority = 5;
    }

    /* Initialize processor state */
    g_processor.buffer = (uint8_t*)malloc(sizeof(uint8_t) * frameBufferSize);
    if (g_processor.buffer == NULL)
    {
        LOGE("Malloc buffer failed, exit");
        return false;
    }
    g_processor.bufferSize = frameBufferSize;
    g_processor.queue = queue;
    g_processor.callback = callback;
    FrameProcessorResetParser();
    g_processor.isRunning = true;

    /* Create FreeRTOS task */
    BaseType_t result = xTaskCreate(FrameProcessorTaskEntry,
                                    "FrameProcessor",
                                    stackSize / sizeof(StackType_t),
                                    NULL, /* No parameter needed */
                                    priority,
                                    &g_processor.taskHandle);

    if (result != pdPASS)
    {
        g_processor.isRunning = false;
        g_processor.taskHandle = NULL;
        LOGE("Failed to start the frame process task");
        return false;
    }

    return true;
}

/**
 * @brief Deinitialize the frame processor.
 *
 * Stops the processing task, releases allocated resources,
 * and resets internal state.
 */
void FrameProcessorDeinit(void)
{
    if (! g_processor.isRunning)
    {
        return;
    }

    g_processor.isRunning = false;

    /* Wait for task to exit */
    vTaskDelay(pdMS_TO_TICKS(100));

    /* Force delete if still running */
    if (g_processor.taskHandle != NULL)
    {
        vTaskDelete(g_processor.taskHandle);
        g_processor.taskHandle = NULL;
    }

    if (g_processor.buffer != NULL)
        free(g_processor.buffer);

    g_processor.bufferSize = 0;
}

/**
 * @brief Check whether the frame processor is running.
 *
 * @return true if the processor task is active, otherwise false.
 */
bool FrameProcessorIsRunning(void) {
    return g_processor.isRunning;
}

/**
 * @brief Get current parser state.
 *
 * Mainly intended for debugging or diagnostics.
 *
 * @return Current state of the frame parser state machine.
 */
uint8_t FrameProcessorGetCurrentState(void) {
    return g_processor.state;
}

/**
 * @brief Reset the internal frame parser.
 *
 * Clears the current parsing state and prepares the parser
 * to receive a new frame.
 */
void FrameProcessorReset(void) {
    FrameProcessorResetParser();
}

/**
 * @brief Build a frame with CRC and byte stuffing.
 *
 * Constructs a complete frame using the protocol format:
 *
 * START | LENGTH | PAYLOAD | CRC | END
 *
 * Special bytes inside the payload and CRC fields are
 * automatically escaped.
 *
 * @param payload Pointer to payload data.
 * @param payloadLen Length of the payload.
 * @param outBuf Output buffer where the frame will be written.
 * @param outBufSize Size of the output buffer.
 *
 * @return Total frame length written to outBuf.
 *         Returns 0 if an error occurs (invalid input or buffer too small).
 */
uint16_t FrameProcessorCreateFrame(const uint8_t* payload, uint16_t payloadLen, uint8_t* outBuf, uint16_t outBufSize)
{
    if ((payload == NULL) || (outBuf == NULL))
    {
        LOGE("Create frame failed, incorrect input arguments");
        return 0;
    }

    /* Worst case: all bytes escape, so 2x payload + 8 bytes overhead */
    if ((payloadLen == 0) || ((payloadLen + 6) > outBufSize))
    {
        LOGE("Create frame failed, incorrect payload length %d/%d", payloadLen, outBufSize);
        return 0;
    }

    uint16_t datalen = 0;

    /* Add starter flag */
    outBuf[datalen++] = FRAME_PROCESSOR_STARTER_FLAG;

    /* Add length (big-endian) */
    outBuf[datalen++] = (uint8_t)((payloadLen >> 8) & 0xFF);
    outBuf[datalen++] = (uint8_t)(payloadLen & 0xFF);

    uint16_t tmp = FrameProcessorPushToBuffer(payload, payloadLen, &(outBuf[datalen]), outBufSize - datalen);
    if (tmp == 0)
    {
        LOGE("Output frame buffer size(%d) too small, skip", outBufSize);
        return 0;
    }

    datalen += tmp;

    /* Calculate and add CRC */
    uint16_t crc = FrameProcessorCalculateCrc(payload, payloadLen);
    const uint8_t crcBuf[] = {(uint8_t)((crc >> 8) & 0xFF), (uint8_t)(crc & 0xFF)};
    tmp = FrameProcessorPushToBuffer(crcBuf, sizeof(crcBuf), &(outBuf[datalen]), outBufSize - datalen);
    if (tmp == 0)
    {
        LOGE("Output frame buffer size(%d) too small, skip", outBufSize);
        return 0;
    }

    datalen += tmp;

    if (datalen < outBufSize)
        outBuf[datalen++] = FRAME_PROCESSOR_END_FLAG;
    else
        datalen = 0;

    return datalen;
}