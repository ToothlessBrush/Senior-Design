/**
 * @file uart.h
 * @brief UART2 driver for LoRa module communication (USART2, PA2/PA3)
 *
 * Provides interrupt-driven RX with circular buffer and DMA-based TX for
 * efficient serial communication at 115200 baud. Used for AT command
 * interface with REYAX RYLR998 LoRa modules.
 *
 * Hardware Configuration:
 * - USART2 on APB1 (50 MHz clock)
 * - PA2: TX (AF7)
 * - PA3: RX (AF7) with pull-up
 * - RX: Interrupt-driven with 256-byte circular buffer
 * - TX: DMA1 Stream 6, Channel 4 for efficient transmission
 *
 * Features:
 * - 115200 baud rate
 * - Interrupt-driven RX with overflow protection
 * - DMA-based TX for non-blocking transmission
 * - Circular RX buffer (power-of-2 size for fast masking)
 */

#ifndef UART_H
#define UART_H

#include <stdint.h>

#define UART_RX_BUFFER_SIZE 256  /**< RX buffer size (must be power of 2) */

/**
 * @brief Initialize UART2 peripheral for LoRa communication
 *
 * Configures GPIO pins (PA2/PA3), USART2 peripheral, RX interrupt,
 * and TX DMA. Enables 115200 baud 8N1 communication. Must be called
 * before any other UART functions.
 */
void uart_init(void);

/**
 * @brief Send single byte (blocking, non-DMA)
 *
 * Waits for previous byte transmission to complete, then sends byte
 * via USART data register. Use for simple single-byte transmissions.
 *
 * @param data Byte to transmit
 */
void uart_send_byte(uint8_t data);

/**
 * @brief Send null-terminated string via DMA
 *
 * Copies string to internal TX buffer and initiates DMA transfer.
 * Waits if previous DMA transfer still in progress. Non-blocking
 * after DMA start. Strings longer than 256 bytes are truncated.
 *
 * @param str Null-terminated string to send
 */
void uart_send_string(const char *str);

/**
 * @brief Send raw data buffer via DMA
 *
 * Copies data to internal TX buffer and initiates DMA transfer.
 * Waits if previous DMA transfer still in progress. Non-blocking
 * after DMA start. Data exceeding 256 bytes is truncated.
 *
 * @param data Pointer to data buffer
 * @param len Number of bytes to send (max 256)
 */
void uart_send_data(const uint8_t *data, uint16_t len);

/**
 * @brief Receive single byte from circular buffer (blocking)
 *
 * Waits until data available in RX buffer, then returns oldest byte.
 * Advances read pointer in circular buffer. Blocks indefinitely if
 * no data received.
 *
 * @return Received byte
 */
uint8_t uart_receive_byte(void);

/**
 * @brief Check if RX data available
 *
 * @return 1 if at least one byte available in RX buffer, 0 otherwise
 */
int uart_data_available(void);

/**
 * @brief Get number of bytes available in RX buffer
 *
 * Calculates unread byte count in circular buffer, handling wraparound.
 *
 * @return Number of bytes available to read (0-255)
 */
uint16_t uart_bytes_available(void);

/**
 * @brief Flush RX buffer (discard all received data)
 *
 * Resets read pointer to match write pointer, effectively clearing
 * all buffered data. Does not affect data currently being received.
 */
void uart_flush(void);

/**
 * @brief Check if TX DMA transfer in progress
 *
 * @return 1 if DMA transmission active, 0 if idle
 */
int uart_tx_busy(void);

#endif // UART_H
