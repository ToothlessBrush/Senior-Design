/**
 * @file bluetooth.h
 * @brief UART Bluetooth module driver (HC-05, HC-06, or compatible)
 *
 * Transparent serial driver for classic Bluetooth modules on UART2 (PA2/PA3).
 * Uses the same physical port as the LoRa driver — select via the USE_BLUETOOTH
 * macro in comm.h.
 *
 * Messages use a binary framing protocol:
 *   | 0xA5 (sync) | TYPE (1) | LEN (1) | PAYLOAD (LEN bytes) | CRC8 (1) |
 *
 * CRC8-DVB-S2 covers TYPE + LEN + PAYLOAD. Frames with bad CRC are silently
 * discarded.
 *
 * Baud rate is set by BT_BAUD. Factory defaults:
 *   HC-05: 9600, HC-06: 9600
 * Common configured rates: 38400, 115200
 */

#ifndef BLUETOOTH_H
#define BLUETOOTH_H

#include "protocol.h"
#include "uart.h"
#include <stdint.h>

// Baud rate must match what the module is configured for.
// RNBT (RN-42) default: 115200 | HC-05/HC-06 default: 9600
#define BT_BAUD 115200

#define BT_MAX_PAYLOAD 240

#define BT_SYNC_BYTE 0xA5

// Binary protocol type bytes (config + calibration only; flight control is
// CRSF) Uplink (GUI → firmware) command type bytes
#define BT_CMD_CALIBRATE 0x01      /**< No payload (0 bytes) */
#define BT_CMD_SET_PID 0x02        /**< CommandSetPid payload (21 bytes) */
#define BT_CMD_SET_MOTOR_BIAS 0x03 /**< CommandMotorBias payload (16 bytes) */
#define BT_CMD_CONFIG 0x04         /**< CommandConfig payload (116 bytes) */
#define BT_CMD_SAVE                                                            \
    0x05 /**< No payload (0 bytes) — save current runtime config to flash */

// Downlink (firmware → GUI) type bytes
#define BT_TELEM 0x10 /**< TelemetryPacket payload (60 bytes) */

typedef enum {
    BT_OK = 0,
    BT_ERROR = -1,
} bt_status_t;

/**
 * @brief Received Bluetooth message
 *
 * Mirrors the fields of lora_message_t used by bt_parse_command():
 * data[] and length. data[0] holds the TYPE byte; data[1..length-1] holds the
 * raw payload. Not null-terminated.
 */
typedef struct {
    uint8_t length;
    uint8_t data[BT_MAX_PAYLOAD + 1];
} bt_message_t;

/**
 * @brief Initialize UART2 at BT_BAUD
 * @return BT_OK
 */
int bt_init(void);

/**
 * @brief Process incoming UART bytes using the binary framing state machine
 *
 * Call periodically in the main loop. Sets the received flag when a complete,
 * valid frame is assembled. Non-blocking — drains all currently available
 * bytes.
 */
void bt_service(void);

/** @return 1 if an unread message is available, 0 otherwise */
int bt_data_available(void);

/** @return Pointer to the most recently received message */
bt_message_t *bt_get_received_data(void);

/** Mark the current message as consumed */
void bt_clear_received_flag(void);

/**
 * @brief Send raw bytes followed by \r\n via DMA
 */
int bt_send_data(const uint8_t *data, uint8_t length);

/**
 * @brief Send a null-terminated string followed by \r\n
 *
 * Waits if a previous DMA TX is still in progress, then returns once
 * the DMA transfer is started.
 */
int bt_send_string(const char *str);

/**
 * @brief Send a null-terminated string followed by \r\n (non-blocking alias)
 *
 * BT modules have no AT-command handshake, so this is identical to
 * bt_send_string(). Exists to match the comm.h interface.
 */
int bt_send_string_nb(const char *str);

/**
 * @brief Send a framed binary packet: 0xA5 | type | len | payload | CRC8-DVB-S2
 *
 * Builds and transmits the complete frame without appending \r\n.
 */
int bt_send_frame(uint8_t type, const uint8_t *payload, uint8_t length);

/**
 * @brief Parse a received binary frame into a ParsedCommand
 *
 * @param data   data[0] is the TYPE byte; data[1..length-1] is the payload
 * @param length Total bytes (1 + payload length)
 * @return ParsedCommand with type and payload filled in, or CMD_NONE on error
 */
ParsedCommand bt_parse_command(const uint8_t *data, uint8_t length);

#endif // BLUETOOTH_H
