#ifndef UART_H
#define UART_H

#include <stdint.h>

// RX buffer size (must be power of 2 for efficient masking)
#define UART_RX_BUFFER_SIZE 256

// Initialize UART2 for LoRa communication
void uart_init(void);

// Send a single byte
void uart_send_byte(uint8_t data);

// Send a null-terminated string
void uart_send_string(const char *str);

// Send raw data buffer
void uart_send_data(const uint8_t *data, uint16_t len);

// Receive a single byte (blocking)
uint8_t uart_receive_byte(void);

// Check if data is available to read
int uart_data_available(void);

// Get number of bytes available in RX buffer
uint16_t uart_bytes_available(void);

// Flush RX buffer
void uart_flush(void);

#endif // UART_H
