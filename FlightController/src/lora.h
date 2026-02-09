/**
 * @file lora.h
 * @brief LoRa wireless communication driver for REYAX RYLR998 modules
 *
 * Provides AT command interface, data transmission, and reception handling for
 * LoRa wireless communication. Supports both blocking and non-blocking send
 * operations with automatic response parsing.
 *
 * The driver communicates with REYAX RYLR998 LoRa modules via UART using AT
 * commands. It handles module initialization, configuration, data transmission,
 * and automatic response processing.
 *
 * Usage:
 * 1. Call lora_init() during system initialization
 * 2. Call lora_service() periodically in main loop to process responses
 * 3. Use lora_send_string() or lora_send_string_nb() to transmit data
 * 4. Check lora_data_available() and use lora_get_received_data() to receive
 */

#ifndef LORA_H
#define LORA_H

#include "uart.h"
#include <stdint.h>

// LoRa configuration - CHANGE THESE FOR YOUR SETUP
#define LORA_ADDRESS 2       // This module's address (Pi is 1)
#define LORA_NETWORK_ID 6    // Network ID (3-15, must match other modules)
#define LORA_BAND 915000000  // 915MHz for US, 868MHz for EU
#define LORA_MAX_PAYLOAD 240 // Maximum data length per packet

#define MIN_SEND_INTERVAL 300 // 150 minimum 250 more stable

// LoRa parameters: Spreading Factor, Bandwidth, Coding Rate, Preamble
// SF: 7-12 (7=fastest/shortest range, 12=slowest/longest range)
// BW: 7=125kHz, 8=250kHz, 9=500kHz
// CR: 1=4/5, 2=4/6, 3=4/7, 4=4/8
// Preamble: 4-7
#define LORA_SF 7        // Spreading Factor (9 = good balance)
#define LORA_BW 9        // Bandwidth (125kHz)
#define LORA_CR 1        // Coding Rate (4/5)
#define LORA_PREAMBLE 12 // Preamble length

// Response timeout in milliseconds
#define LORA_TIMEOUT_MS 1000
#define LORA_RX_LINE_SIZE 300

/**
 * @brief LoRa operation status codes
 */
typedef enum {
    LORA_OK = 0,        /**< Operation completed successfully, +OK received */
    LORA_ERROR = -1,    /**< Error occurred, +ERR received or invalid response */
    LORA_TIMEOUT = -2,  /**< Operation timed out waiting for response */
    LORA_BUSY = -3      /**< Module busy, previous command still processing */
} lora_status_t;

/**
 * @brief Structure for received LoRa messages
 *
 * Contains all information from a received LoRa packet including payload,
 * sender address, and RF link quality metrics (RSSI and SNR).
 */
typedef struct {
    uint8_t sender_address;              /**< Address of sending module */
    uint8_t length;                      /**< Length of received data */
    uint8_t data[LORA_MAX_PAYLOAD + 1]; /**< Received payload (+1 for null terminator) */
    int16_t rssi;                        /**< Received Signal Strength Indicator (dBm) */
    int8_t snr;                          /**< Signal-to-Noise Ratio (dB) */
} lora_message_t;

/**
 * @brief Initialize LoRa module with configured parameters
 *
 * Sends AT commands to configure module address, network ID, frequency band,
 * and LoRa parameters (SF, BW, CR, preamble). Must be called before any other
 * LoRa functions. Includes 500ms power-up delay.
 *
 * @return LORA_OK on success, LORA_ERROR on failure
 */
int lora_init(void);

/**
 * @brief Check if LoRa module is ready to send
 *
 * Returns 1 if module has completed previous operation (+OK received or idle),
 * 0 if module is still processing a command.
 *
 * @return 1 if ready, 0 if busy
 */
int lora_is_ready(void);

/**
 * @brief Get last error code from LoRa module
 *
 * Returns the error code from the most recent +ERR response. Error codes
 * are module-specific (see REYAX RYLR998 documentation).
 *
 * @return Error code from last +ERR response, -1 if no error occurred
 */
int lora_error_code(void);

/**
 * @brief Manually set module ready state (internal use)
 *
 * @param ready 1 to mark module as ready, 0 to mark as busy
 */
void lora_set_ready(uint8_t ready);

/**
 * @brief Process incoming UART data and parse LoRa responses
 *
 * Must be called periodically (e.g., in main loop) to handle asynchronous
 * responses from the LoRa module. Processes +OK, +ERR, and +RCV messages.
 * Non-blocking - processes all available data and returns immediately.
 */
void lora_service(void);

/**
 * @brief Send AT command and wait for response (blocking)
 *
 * Sends an AT command to the LoRa module and waits for a response. Blocks
 * until response received or timeout occurs. Used internally for initialization
 * and can be used for custom AT commands.
 *
 * @param cmd AT command string (without \r\n terminator)
 * @param response Buffer to store response (can be NULL if not needed)
 * @param max_len Maximum length of response buffer
 * @param timeout_ms Timeout in milliseconds
 * @return LORA_OK, LORA_ERROR, or LORA_TIMEOUT
 */
int lora_send_at_command(const char *cmd, char *response, uint16_t max_len,
                         uint32_t timeout_ms);

/**
 * @brief Send binary data to a destination address (blocking)
 *
 * Sends data and waits for +OK response. Blocks until transmission confirmed
 * or timeout occurs. Use for reliable transmission when timing is not critical.
 *
 * @param dest_address Destination module address (0-65535)
 * @param data Pointer to data buffer
 * @param length Length of data (max LORA_MAX_PAYLOAD bytes)
 * @return LORA_OK on success, LORA_ERROR or LORA_TIMEOUT on failure
 */
int lora_send_data(uint8_t dest_address, const uint8_t *data, uint8_t length);

/**
 * @brief Send null-terminated string to a destination address (blocking)
 *
 * Convenience wrapper around lora_send_data() for string transmission.
 * Blocks until +OK received or timeout.
 *
 * @param dest_address Destination module address
 * @param str Null-terminated string to send
 * @return LORA_OK on success, LORA_ERROR or LORA_TIMEOUT on failure
 */
int lora_send_string(uint8_t dest_address, const char *str);

/**
 * @brief Send binary data to a destination address (non-blocking)
 *
 * Sends data and returns immediately without waiting for +OK. Used in
 * time-critical code (e.g., control loops). Response is processed by
 * lora_service(). Module ready state updated when +OK received.
 *
 * @param dest_address Destination module address (0-65535)
 * @param data Pointer to data buffer
 * @param length Length of data (max LORA_MAX_PAYLOAD bytes)
 * @return LORA_OK if command sent, LORA_ERROR if length exceeds maximum
 */
int lora_send_data_nb(uint8_t dest_address, const uint8_t *data,
                      uint8_t length);

/**
 * @brief Send null-terminated string to a destination address (non-blocking)
 *
 * Convenience wrapper around lora_send_data_nb() for string transmission.
 * Returns immediately without waiting for confirmation.
 *
 * @param dest_address Destination module address
 * @param str Null-terminated string to send
 * @return LORA_OK if command sent, LORA_ERROR if string too long
 */
int lora_send_string_nb(uint8_t dest_address, const char *str);

/**
 * @brief Check if a LoRa message has been received
 *
 * @return 1 if unread message available, 0 otherwise
 */
int lora_data_available(void);

/**
 * @brief Get pointer to most recently received message
 *
 * Returns pointer to internal message buffer containing the last received
 * message. Data remains valid until next message received or flag cleared.
 *
 * @return Pointer to lora_message_t structure with received data
 */
lora_message_t *lora_get_received_data(void);

/**
 * @brief Clear the received data flag
 *
 * Marks the current message as read. Must be called after processing a
 * received message to allow detection of new messages.
 */
void lora_clear_received_flag(void);

#endif // LORA_H
