#ifndef FRAME_PROCESSOR_H
#define FRAME_PROCESSOR_H

#include <stdint.h>
#include <stdbool.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>

/**
 * @brief Pure C FreeRTOS-based frame protocol processor with byte stuffing support
 *
 * Global singleton pattern - simple API with no create/destroy boilerplate.
 *
 * Frame format: [0x7E] [Length H] [Length L] [Payload*] [CRC H*] [CRC L*] [0x7D]
 * Where * indicates bytes that undergo byte stuffing/unstuffing
 *
 * Byte stuffing: 0xAA is the escape character
 *  - 0xAA 0x00 => 0xAA (original escape char)
 *  - 0xAA 0x01 => 0x7E (starter flag)
 *  - 0xAA 0x02 => 0x7D (end flag)
 *
 * Usage:
 *   frame_processor_init(queue, callback, 2048, 5);
 *   // ... process frames ...
 *   frame_processor_deinit();
 */

/* ============================================================================
   PROTOCOL CONSTANTS
   ============================================================================ */
/* Protocol constants (must match frame_processor.h) */
#define FRAME_PROCESSOR_STARTER_FLAG      0x7E
#define FRAME_PROCESSOR_END_FLAG          0x7D
#define FRAME_PROCESSOR_ESCAPE_CHAR       0xAA
#define FRAME_PROCESSOR_ESCAPE_ESCAPE     0x00
#define FRAME_PROCESSOR_ESCAPE_STARTER    0x01
#define FRAME_PROCESSOR_ESCAPE_END        0x02


/* ============================================================================
   STATE MACHINE CONSTANTS
   ============================================================================ */

#define FRAME_PROCESSOR_STATE_IDLE             0  /* Waiting for starter flag */
#define FRAME_PROCESSOR_STATE_WAITING_LENGTH_H 1  /* Waiting for length high byte */
#define FRAME_PROCESSOR_STATE_WAITING_LENGTH_L 2  /* Waiting for length low byte */
#define FRAME_PROCESSOR_STATE_WAITING_PAYLOAD  3  /* Waiting for payload bytes */
#define FRAME_PROCESSOR_STATE_WAITING_CRC_H    4  /* Waiting for CRC high byte */
#define FRAME_PROCESSOR_STATE_WAITING_CRC_L    5  /* Waiting for CRC low byte */
#define FRAME_PROCESSOR_STATE_WAITING_END      6  /* Waiting for end flag */

/* ============================================================================
   CALLBACK TYPE
   ============================================================================ */

/**
 * @brief Callback function type for valid parsed frames
 * @param payload Pointer to the payload data
 * @param length Length of the payload in bytes
 * @return true if frame was processed successfully, false otherwise
 */
typedef bool (*FrameProcessor_SendCallback)(const uint8_t* payload, uint16_t length);

/* ============================================================================
   PUBLIC API - GLOBAL SINGLETON
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
bool FrameProcessorInit(QueueHandle_t queue, FrameProcessor_SendCallback callback, uint16_t frameBufferSize, uint16_t stackSize, UBaseType_t priority);

/**
 * @brief Deinitialize the frame processor.
 *
 * Stops the processing task, releases allocated resources,
 * and resets internal state.
 */
void FrameProcessorDeinit(void);

/**
 * @brief Check whether the frame processor is running.
 *
 * @return true if the processor task is active, otherwise false.
 */
bool FrameProcessorIsRunning(void);

/**
 * @brief Get current parser state.
 *
 * Mainly intended for debugging or diagnostics.
 *
 * @return Current state of the frame parser state machine.
 */
uint8_t FrameProcessorGetCurrentState(void);

/**
 * @brief Reset the internal frame parser.
 *
 * Clears the current parsing state and prepares the parser
 * to receive a new frame.
 */
void FrameProcessorReset(void);
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
uint16_t FrameProcessorCreateFrame(const uint8_t* payload, uint16_t payloadLen, uint8_t* out_frame, uint16_t out_len);

#endif /* FRAME_PROCESSOR_H */
