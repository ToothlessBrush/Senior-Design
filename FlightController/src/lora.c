#include "lora.h"
#include "cmsis_gcc.h"
#include "systick.h"
#include "uart.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// Module ready flag (set when +OK received)
static volatile uint8_t module_ready = 1;

// RX line buffer for parsing responses
static char rx_line[LORA_RX_LINE_SIZE];
static uint16_t rx_idx = 0;

// Check if module is ready to send
int lora_is_ready(void) { return module_ready; }

// Set module ready state (used internally)
void lora_set_ready(uint8_t ready) { module_ready = ready; }

// Send AT command and wait for response (blocking)
int lora_send_at_command(const char *cmd, char *response, uint16_t max_len,
                         uint32_t timeout_ms) {
  // GDB visible variables
  const char *command_sent = cmd; // Visible in GDB
  uint32_t elapsed = 0;
  uint16_t idx = 0;
  uint8_t found_ok = 0;
  uint8_t found_err = 0;
  int error_code = -1;            // NEW: Store error code for debugging
  char debug_response[256] = {0}; // Local copy for debugging

  // Clear any pending data
  uart_flush();

  // Send command
  uart_send_string(cmd);

  if (response) {
    response[0] = '\0';
  }

  // Wait for response
  while (elapsed < timeout_ms) {
    if (uart_data_available()) {
      uint8_t c = uart_receive_byte();

      if (response && idx < max_len - 1) {
        response[idx] = c;
        response[idx + 1] = '\0';
      }

      if (idx < sizeof(debug_response) - 1) {
        debug_response[idx] = c;
        debug_response[idx + 1] = '\0';
      }

      idx++;

      // Check for complete response
      if (idx >= 3) {
        // Check for +OK
        if (strstr(debug_response, "+OK") != NULL) {
          found_ok = 1; // SET BREAKPOINT HERE to see response
          module_ready = 1;
          return LORA_OK;
        }

        // Check for +ERR with error code
        char *err_pos = strstr(debug_response, "+ERR=4");
        if (err_pos != NULL) {
          found_err = 1; // SET BREAKPOINT HERE to see response

          // Try to parse error code: +ERR=X
          char *equals = strchr(err_pos, '=');
          if (equals != NULL) {
            error_code = atoi(equals + 1); // Parse the number after '='
          }

          module_ready = 1;
          return LORA_ERROR;
        }
      }
    }
    delay_ms(1);
    elapsed++;
  }

  // Timeout - SET BREAKPOINT HERE
  uint8_t timed_out = 1;
  return LORA_TIMEOUT;
}
// Initialize LoRa module
int lora_init(void) {
  char response[128];
  char cmd[64];

  delay_ms(500); // Wait for module to power up

  // Test connection
  if (lora_send_at_command("AT\r\n", response, sizeof(response),
                           LORA_TIMEOUT_MS) != LORA_OK) {
    return LORA_ERROR;
  }
  delay_ms(500);

  // Set address
  snprintf(cmd, sizeof(cmd), "AT+ADDRESS=%d\r\n", LORA_ADDRESS);
  if (lora_send_at_command(cmd, response, sizeof(response), LORA_TIMEOUT_MS) !=
      LORA_OK) {
    return LORA_ERROR;
  }
  delay_ms(500);

  // Set network ID
  snprintf(cmd, sizeof(cmd), "AT+NETWORKID=%d\r\n", LORA_NETWORK_ID);
  if (lora_send_at_command(cmd, response, sizeof(response), LORA_TIMEOUT_MS) !=
      LORA_OK) {
    return LORA_ERROR;
  }
  delay_ms(500);

  // Set frequency band
  snprintf(cmd, sizeof(cmd), "AT+BAND=%lu\r\n", (unsigned long)LORA_BAND);
  if (lora_send_at_command(cmd, response, sizeof(response), LORA_TIMEOUT_MS) !=
      LORA_OK) {
    return LORA_ERROR;
  }
  delay_ms(500);

  // Set parameters: SF, BW, CR, Preamble
  snprintf(cmd, sizeof(cmd), "AT+PARAMETER=%d,%d,%d,%d\r\n", LORA_SF, LORA_BW,
           LORA_CR, LORA_PREAMBLE);
  if (lora_send_at_command(cmd, response, sizeof(response), LORA_TIMEOUT_MS) !=
      LORA_OK) {
    return LORA_ERROR;
  }
  delay_ms(500);

  module_ready = 1;
  return LORA_OK;
}
// Blocking send - waits for +OK
int lora_send_data(uint8_t dest_address, const uint8_t *data, uint8_t length) {
  if (length > LORA_MAX_PAYLOAD) {
    return LORA_ERROR;
  }

  char cmd[LORA_MAX_PAYLOAD + 50];
  int cmd_len =
      snprintf(cmd, sizeof(cmd), "AT+SEND=%d,%d\r\n,", dest_address, length);

  // Append data
  for (uint8_t i = 0; i < length; i++) {
    cmd[cmd_len++] = data[i];
  }
  cmd[cmd_len] = '\0';

  char response[64];
  return lora_send_at_command(cmd, response, sizeof(response), LORA_TIMEOUT_MS);
}

// Blocking string send
int lora_send_string(uint8_t dest_address, const char *str) {
  uint8_t len = strlen(str);
  return lora_send_data(dest_address, (const uint8_t *)str, len);
}

// Non-blocking send - returns immediately
int lora_send_data_nb(uint8_t dest_address, const uint8_t *data,
                      uint8_t length) {
  if (length > LORA_MAX_PAYLOAD) {
    return LORA_ERROR;
  }

  // Build command
  char cmd[LORA_MAX_PAYLOAD + 50];
  int cmd_len =
      snprintf(cmd, sizeof(cmd), "AT+SEND=%d,%d,\r\n", dest_address, length);

  // Append data
  for (uint8_t i = 0; i < length; i++) {
    cmd[cmd_len++] = data[i];
  }

  // Send command without waiting
  uart_send_data((uint8_t *)cmd, cmd_len);
  uart_send_string("\r\n");

  module_ready = 0; // Mark as busy until we get +OK

  return LORA_OK;
}

// Non-blocking string send
int lora_send_string_nb(uint8_t dest_address, const char *str) {
  uint8_t len = strlen(str);
  return lora_send_data_nb(dest_address, (const uint8_t *)str, len);
}

// Parse received message line
static int parse_received_message(char *line, lora_message_t *msg) {
  // Format: +RCV=<addr>,<len>,<data>,<RSSI>,<SNR>
  if (strncmp(line, "+RCV=", 5) != 0) {
    return 0;
  }

  char *ptr = line + 5;

  // Parse sender address
  msg->sender_address = atoi(ptr);
  ptr = strchr(ptr, ',');
  if (!ptr)
    return 0;
  ptr++;

  // Parse length
  msg->length = atoi(ptr);
  if (msg->length > LORA_MAX_PAYLOAD)
    return 0;
  ptr = strchr(ptr, ',');
  if (!ptr)
    return 0;
  ptr++;

  // Extract data
  uint8_t data_idx = 0;
  char *rssi_start = ptr;

  // Find where data ends (look for comma followed by negative number = RSSI)
  // We need to count backwards from the end since data can contain commas
  char *end = line + strlen(line);
  char *rssi_ptr = end - 1;
  int comma_count = 0;

  // Find last two commas (for RSSI and SNR)
  while (rssi_ptr > ptr && comma_count < 2) {
    if (*rssi_ptr == ',')
      comma_count++;
    if (comma_count < 2)
      rssi_ptr--;
  }

  // Extract data up to RSSI field
  while (ptr < rssi_ptr && data_idx < msg->length) {
    msg->data[data_idx++] = *ptr++;
  }
  msg->data[data_idx] = '\0';

  // Parse RSSI
  if (*ptr == ',') {
    ptr++;
    msg->rssi = atoi(ptr);
    ptr = strchr(ptr, ',');
    if (ptr) {
      ptr++;
      msg->snr = atoi(ptr);
    } else {
      msg->snr = 0;
    }
  } else {
    msg->rssi = 0;
    msg->snr = 0;
  }

  return 1;
}

// Check for received messages (call this regularly)
int lora_check_received(lora_message_t *msg) {
  // Process incoming data line by line
  while (uart_data_available()) {
    uint8_t c = uart_receive_byte();

    // End of line
    if (c == '\n' || c == '\r') {
      if (rx_idx > 0) {
        rx_line[rx_idx] = '\0';

        // Check for +OK (module ready)
        if (strstr(rx_line, "+OK") != NULL) {
          module_ready = 1;
        }
        // Check for +ERR
        else if (strstr(rx_line, "+ERR") != NULL) {
          module_ready = 1;
        }
        // Check for received message
        else if (parse_received_message(rx_line, msg)) {
          rx_idx = 0;
          return 1; // Message received!
        }

        rx_idx = 0;
      }
    }
    // Add to line buffer
    else if (rx_idx < LORA_RX_LINE_SIZE - 1) {
      rx_line[rx_idx++] = c;
    } else {
      // Buffer overflow, reset
      rx_idx = 0;
    }
  }

  return 0; // No message
}

// Process RX data (convenience function - just calls check_received with dummy
// msg)
void lora_process_rx(void) {
  lora_message_t dummy;
  lora_check_received(&dummy);
}
