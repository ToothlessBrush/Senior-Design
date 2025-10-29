#include "lora.h"
#include "systick.h"
#include "uart.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// Module ready flag (set when +OK received)
static volatile uint8_t module_ready = 1;
static volatile int last_error_code = -1;

// Check if module is ready to send
int lora_is_ready(void) { return module_ready; }

int lora_error_code(void) { return last_error_code; }

// Set module ready state (used internally)
void lora_set_ready(uint8_t ready) { module_ready = ready; }

// Send AT command and wait for response (blocking)
int lora_send_at_command(const char *cmd, char *response, uint16_t max_len,
                         uint32_t timeout_ms) {
  uint32_t elapsed = 0;
  uint16_t idx = 0; // index of response
  char full_response[256] = {0};

  // Clear any pending data
  uart_flush();

  // Send command
  uart_send_string(cmd);
  uart_send_string("\r\n");

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

      if (idx < sizeof(full_response) - 1) {
        full_response[idx] = c;
        full_response[idx + 1] = '\0';
      }

      idx++;

      // Check for complete response
      if (idx >= 2 && full_response[idx - 2] == '\r' &&
          full_response[idx - 1] == '\n') {

        if (strstr(full_response, "+OK\r\n") != NULL) {
          // response OK
          return LORA_OK;

        } else if (strstr(full_response, "+ERR=") != NULL) {
          // response ERR
          char *err_pos =
              strstr(full_response, "+ERR="); // go to where response is +ERR=
          if (err_pos != NULL) {              // if it exists
            last_error_code =
                atoi(err_pos +
                     5); // get the char at the 5th position and convert to int
          }
          return LORA_ERROR;

        } else {
          // Unknown response
          return LORA_ERROR;
        }
      }
    }
    delay_ms(1);
    elapsed++;
  }

  return LORA_TIMEOUT;
}

// Initialize LoRa module
int lora_init(void) {
  char response[128];
  char cmd[64];

  delay_ms(500); // Wait for module to power up

  // Test connection
  if (lora_send_at_command("AT", response, sizeof(response), LORA_TIMEOUT_MS) !=
      LORA_OK) {
    return LORA_ERROR;
  }
  delay_ms(100);

  // Set address
  snprintf(cmd, sizeof(cmd), "AT+ADDRESS=%d", LORA_ADDRESS);
  if (lora_send_at_command(cmd, response, sizeof(response), LORA_TIMEOUT_MS) !=
      LORA_OK) {
    return LORA_ERROR;
  }
  delay_ms(100);

  // Set network ID
  snprintf(cmd, sizeof(cmd), "AT+NETWORKID=%d", LORA_NETWORK_ID);
  if (lora_send_at_command(cmd, response, sizeof(response), LORA_TIMEOUT_MS) !=
      LORA_OK) {
    return LORA_ERROR;
  }
  delay_ms(100);

  // Set frequency band
  snprintf(cmd, sizeof(cmd), "AT+BAND=%lu", (unsigned long)LORA_BAND);
  if (lora_send_at_command(cmd, response, sizeof(response), LORA_TIMEOUT_MS) !=
      LORA_OK) {
    return LORA_ERROR;
  }
  delay_ms(100);

  // Set parameters: SF, BW, CR, Preamble
  snprintf(cmd, sizeof(cmd), "AT+PARAMETER=%d,%d,%d,%d", LORA_SF, LORA_BW,
           LORA_CR, LORA_PREAMBLE);
  if (lora_send_at_command(cmd, response, sizeof(response), LORA_TIMEOUT_MS) !=
      LORA_OK) {
    return LORA_ERROR;
  }
  delay_ms(100);

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
      snprintf(cmd, sizeof(cmd), "AT+SEND=%d,%d,", dest_address, length);

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

  if (!module_ready) {
    return LORA_BUSY;
  }

  if (length > LORA_MAX_PAYLOAD) {
    return LORA_ERROR;
  }

  // Build command
  char cmd[LORA_MAX_PAYLOAD + 50];
  int cmd_len =
      snprintf(cmd, sizeof(cmd), "AT+SEND=%d,%d,", dest_address, length);

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

// checks to see if loras gotten a response call this in loop
void lora_service(void) {
  static char response_buffer[64];
  static uint16_t response_idx = 0;

  // Process any incoming bytes (non-blocking)
  while (uart_data_available()) {
    uint8_t c = uart_receive_byte();

    if (response_idx < sizeof(response_buffer) - 1) {
      response_buffer[response_idx] = c;
      response_buffer[response_idx + 1] = '\0';
    }
    response_idx++;

    // Check for complete response
    if (response_idx >= 2 && response_buffer[response_idx - 2] == '\r' &&
        response_buffer[response_idx - 1] == '\n') {

      if (strstr(response_buffer, "+OK") != NULL) {
        module_ready = 1; // Ready for next command
      }
      // Reset for next response
      response_idx = 0;
      response_buffer[0] = '\0';
      break; // Process one response per call
    }
  }
}
