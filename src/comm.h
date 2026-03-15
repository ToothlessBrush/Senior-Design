/**
 * @file comm.h
 * @brief Communication module selector for UART2 (Bluetooth or LoRa)
 *
 * Provides a unified API that delegates to either the Bluetooth driver or the
 * LoRa driver depending on the USE_LORA compile-time macro. Only one module
 * may be physically connected to UART2 at a time.
 *
 * Default: Bluetooth. To use LoRa instead, define USE_LORA before including
 * this header, pass -DUSE_LORA as a compiler flag, or uncomment the define
 * below.
 *
 * Unified API:
 *   comm_init()                - initialize the selected module
 *   comm_service()             - process incoming data (call periodically)
 *   comm_data_available()      - non-zero if a complete message is ready
 *   comm_get_received_data()   - returns comm_message_t* (has .data, .length)
 *   comm_clear_received_flag() - mark current message as consumed
 *   comm_send_string(s)        - send string (blocking)
 *   comm_send_string_nb(s)     - send string (non-blocking where possible)
 */

#ifndef COMM_H
#define COMM_H

// Uncomment to use LoRa on UART2 instead of Bluetooth.
// Alternatively, pass -DUSE_LORA via CMakeLists.txt:
//   target_compile_definitions(FlightController PRIVATE USE_LORA)
// #define USE_LORA

#if defined(USE_LORA)

#include "lora.h"
#include "protocol.h"
typedef lora_message_t comm_message_t;
#define comm_init()                lora_init()
#define comm_service()             lora_service()
#define comm_data_available()      lora_data_available()
#define comm_get_received_data()   lora_get_received_data()
#define comm_clear_received_flag() lora_clear_received_flag()
#define comm_send_string(s)        lora_send_string(1, (s))
#define comm_send_string_nb(s)     lora_send_string_nb(1, (s))
#define comm_parse_command(d, l)   parse_command((d), (l))

#else // Bluetooth (default)

#include "bluetooth.h"
typedef bt_message_t comm_message_t;

// BT has no air-time duty-cycle restriction — send telemetry as fast as needed
#define MIN_SEND_INTERVAL 0
#define comm_init()                bt_init()
#define comm_service()             bt_service()
#define comm_data_available()      bt_data_available()
#define comm_get_received_data()   bt_get_received_data()
#define comm_clear_received_flag() bt_clear_received_flag()
#define comm_send_string(s)        bt_send_string(s)
#define comm_send_string_nb(s)     bt_send_string_nb(s)
#define comm_parse_command(d, l)   bt_parse_command((d), (l))

#endif // USE_LORA

#endif // COMM_H
