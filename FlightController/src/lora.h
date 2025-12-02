#ifndef LORA_H
#define LORA_H

#include "uart.h"
#include <stdint.h>

// LoRa configuration - CHANGE THESE FOR YOUR SETUP
#define LORA_ADDRESS 2       // This module's address (Pi is 1)
#define LORA_NETWORK_ID 6    // Network ID (3-15, must match other modules)
#define LORA_BAND 915000000  // 915MHz for US, 868MHz for EU
#define LORA_MAX_PAYLOAD 240 // Maximum data length per packet

#define MIN_SEND_INTERVAL 150

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

// LoRa status
typedef enum {
  LORA_OK = 0,
  LORA_ERROR = -1,
  LORA_TIMEOUT = -2,
  LORA_BUSY = -3
} lora_status_t;

// LoRa received message structure
typedef struct {
  uint8_t sender_address;
  uint8_t length;
  uint8_t data[LORA_MAX_PAYLOAD + 1]; // +1 for null terminator
  int16_t rssi;
  int8_t snr;
} lora_message_t;

// Function prototypes
int lora_init(void);
int lora_is_ready(void);
int lora_error_code(void);
void lora_set_ready(uint8_t ready);
void lora_service(void);

// Blocking send - waits for +OK response
int lora_send_at_command(const char *cmd, char *response, uint16_t max_len,
                         uint32_t timeout_ms);
int lora_send_data(uint8_t dest_address, const uint8_t *data, uint8_t length);
int lora_send_string(uint8_t dest_address, const char *str);

// Non-blocking send - returns immediately
int lora_send_data_nb(uint8_t dest_address, const uint8_t *data,
                      uint8_t length);
int lora_send_string_nb(uint8_t dest_address, const char *str);

// Receive data functions
int lora_data_available(void);
lora_message_t *lora_get_received_data(void);
void lora_clear_received_flag(void);

#endif // LORA_H
