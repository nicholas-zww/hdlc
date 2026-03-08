/**
 * HDLC Parser Header
 *
 * High-Level Data Link Control protocol parser
 * Handles frame detection, bit unstuffing, and CRC verification
 */

#ifndef HDLC_PARSER_H
#define HDLC_PARSER_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

// ============================================================================
// HDLC Constants
// ============================================================================

#define HDLC_FLAG           0x7E        // Frame delimiter
#define HDLC_ESCAPE         0x7D        // Control escape
#define HDLC_ESCAPE_MASK    0x20        // XOR mask for escaped bytes
#define HDLC_MAX_DATA_LEN   254         // Maximum data payload
#define HDLC_FCS_SIZE       2           // Frame Check Sequence size

// ============================================================================
// Parser State Machine States
// ============================================================================

typedef enum {
    HDLC_STATE_IDLE,        // Waiting for flag
    HDLC_STATE_ADDR,        // Received flag, waiting for address
    HDLC_STATE_CTRL,        // Received address, waiting for control
    HDLC_STATE_DATA,        // Receiving data
    HDLC_STATE_FCS_LOW,     // Received data, waiting for FCS low byte
    HDLC_STATE_FCS_HIGH     // Received FCS low, waiting for FCS high byte
} hdlc_state_t;

// ============================================================================
// Frame Structure
// ============================================================================

typedef struct {
    uint8_t address;
    uint8_t control;
    uint8_t data[HDLC_MAX_DATA_LEN];
    size_t data_length;
    uint16_t fcs;
} hdlc_frame_t;

// ============================================================================
// Callback Function Type
// ============================================================================

/**
 * Callback for when a valid HDLC frame is received
 *
 * @param frame Pointer to the received frame
 * @param crc_valid true if CRC check passed
 */
typedef void (*hdlc_frame_callback_t)(const hdlc_frame_t* frame, bool crc_valid);

// ============================================================================
// Public API
// ============================================================================

/**
 * Initialize the HDLC parser
 * Call once during setup
 */
void hdlc_parser_init(void);

/**
 * Process a received character through the HDLC state machine
 * Call for each character received from UART
 *
 * @param data The received character
 */
void hdlc_parser_process_char(uint8_t data);

/**
 * Set the callback function for received frames
 *
 * @param callback Function to call when frame is received
 */
void hdlc_parser_set_callback(hdlc_frame_callback_t callback);

/**
 * Encode and send an HDLC frame
 * Handles bit stuffing and FCS calculation
 *
 * @param address Frame address
 * @param control Frame control byte
 * @param data Pointer to data payload
 * @param length Length of data payload
 * @param buf Buffer to store the encoded frame
 * @param buf_size Size of the buffer
 * @return Number of bytes written to the buffer
 */
uint16_t hdlc_packFrame(uint8_t address, uint8_t control, const uint8_t* data, size_t length, uint8_t* buf, size_t buf_size);
/**
 * Calculate CRC-16-CCITT (used in HDLC)
 *
 * @param data Pointer to data
 * @param length Length of data
 * @return 16-bit CRC value
 */
uint16_t hdlc_calc_crc(const uint8_t* data, size_t length);

#endif // HDLC_PARSER_H
