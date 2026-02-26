/**
 * @file uart.h
 * @brief Multi-instance UART driver for LoRa and optical flow communication
 *
 * Provides interrupt-driven RX with circular buffer and DMA-based TX for
 * efficient serial communication. Supports multiple UART instances.
 *
 * Hardware Configuration:
 * UART2 (LoRa Module):
 * - USART2 on APB1 (50 MHz clock)
 * - PA2: TX (AF7)
 * - PA3: RX (AF7) with pull-up
 * - TX: DMA1 Stream 6, Channel 4
 *
 * UART6 (Optical Flow Sensor):
 * - USART6 on APB2 (100 MHz clock)
 * - PA11: TX (AF8)
 * - PA12: RX (AF8) with pull-up
 * - TX: DMA2 Stream 6, Channel 5
 *
 * Features:
 * - 115200 baud rate for both instances
 * - Interrupt-driven RX with overflow protection
 * - DMA-based TX for non-blocking transmission
 * - Circular RX buffer (power-of-2 size for fast masking)
 */

#ifndef UART_H
#define UART_H

#include <stdint.h>

#define UART_RX_BUFFER_SIZE 256  /**< RX buffer size (must be power of 2) */

/**
 * @brief UART instance enumeration
 */
typedef enum {
    UART_INSTANCE_2 = 0,  /**< UART2 - LoRa module (PA2/PA3) */
    UART_INSTANCE_6 = 1,  /**< UART6 - Optical flow sensor (PA11/PA12) */
    UART_INSTANCE_COUNT   /**< Total number of UART instances */
} uart_instance_t;

/**
 * @brief Initialize specified UART peripheral
 *
 * Configures GPIO pins, USART peripheral, RX interrupt, and TX DMA.
 * Enables 115200 baud 8N1 communication. Must be called before any
 * other UART functions for the specified instance.
 *
 * @param instance UART instance to initialize (UART_INSTANCE_2 or UART_INSTANCE_6)
 */
void uart_init(uart_instance_t instance);

/**
 * @brief Send single byte (blocking, non-DMA)
 *
 * Waits for previous byte transmission to complete, then sends byte
 * via USART data register. Use for simple single-byte transmissions.
 *
 * @param instance UART instance to use
 * @param data Byte to transmit
 */
void uart_send_byte(uart_instance_t instance, uint8_t data);

/**
 * @brief Send null-terminated string via DMA
 *
 * Copies string to internal TX buffer and initiates DMA transfer.
 * Waits if previous DMA transfer still in progress. Non-blocking
 * after DMA start. Strings longer than 256 bytes are truncated.
 *
 * @param instance UART instance to use
 * @param str Null-terminated string to send
 */
void uart_send_string(uart_instance_t instance, const char *str);

/**
 * @brief Send raw data buffer via DMA
 *
 * Copies data to internal TX buffer and initiates DMA transfer.
 * Waits if previous DMA transfer still in progress. Non-blocking
 * after DMA start. Data exceeding 256 bytes is truncated.
 *
 * @param instance UART instance to use
 * @param data Pointer to data buffer
 * @param len Number of bytes to send (max 256)
 */
void uart_send_data(uart_instance_t instance, const uint8_t *data, uint16_t len);

/**
 * @brief Receive single byte from circular buffer (blocking)
 *
 * Waits until data available in RX buffer, then returns oldest byte.
 * Advances read pointer in circular buffer. Blocks indefinitely if
 * no data received.
 *
 * @param instance UART instance to use
 * @return Received byte
 */
uint8_t uart_receive_byte(uart_instance_t instance);

/**
 * @brief Check if RX data available
 *
 * @param instance UART instance to check
 * @return 1 if at least one byte available in RX buffer, 0 otherwise
 */
int uart_data_available(uart_instance_t instance);

/**
 * @brief Get number of bytes available in RX buffer
 *
 * Calculates unread byte count in circular buffer, handling wraparound.
 *
 * @param instance UART instance to check
 * @return Number of bytes available to read (0-255)
 */
uint16_t uart_bytes_available(uart_instance_t instance);

/**
 * @brief Flush RX buffer (discard all received data)
 *
 * Resets read pointer to match write pointer, effectively clearing
 * all buffered data. Does not affect data currently being received.
 *
 * @param instance UART instance to flush
 */
void uart_flush(uart_instance_t instance);

/**
 * @brief Check if TX DMA transfer in progress
 *
 * @param instance UART instance to check
 * @return 1 if DMA transmission active, 0 if idle
 */
int uart_tx_busy(uart_instance_t instance);

#endif // UART_H
