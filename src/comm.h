/**
 * @file comm.h
 * @brief Communication abstraction layer (Bluetooth on UART2)
 *
 * Unified API for Bluetooth communication on UART2 (PA2/PA3).
 *
 * API:
 *   comm_init()                - initialize the Bluetooth module
 *   comm_service()             - process incoming data (call periodically)
 *   comm_data_available()      - non-zero if a complete message is ready
 *   comm_get_received_data()   - returns comm_message_t* (has .data, .length)
 *   comm_clear_received_flag() - mark current message as consumed
 *   comm_send_string(s)        - send string (blocking)
 *   comm_send_string_nb(s)     - send string (non-blocking)
 */

#ifndef COMM_H
#define COMM_H

#include "bluetooth.h"
typedef bt_message_t comm_message_t;

#define MIN_SEND_INTERVAL 20 // 50hz
#define comm_init() bt_init()
#define comm_service() bt_service()
#define comm_data_available() bt_data_available()
#define comm_get_received_data() bt_get_received_data()
#define comm_clear_received_flag() bt_clear_received_flag()
#define comm_send_string(s) bt_send_string(s)
#define comm_send_string_nb(s) bt_send_string_nb(s)
#define comm_send_frame(t, d, l) bt_send_frame((t), (d), (l))
#define comm_parse_command(d, l) bt_parse_command((d), (l))

#endif // COMM_H
