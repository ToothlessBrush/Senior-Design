/**
 * @file spi.h
 * @brief SPI1 driver for LSM6DSL IMU communication
 *
 * Provides SPI master interface for communicating with LSM6DSL accelerometer/
 * gyroscope sensor. Configured for SPI Mode 3 (CPOL=1, CPHA=1) at APB2/64
 * clock rate.
 *
 * Hardware Configuration:
 * - SPI1 on APB2 (100 MHz clock, divided by 64 = ~1.56 MHz)
 * - PA4: CS (GPIO output, active low)
 * - PA5: SCK (AF5)
 * - PA6: MISO (AF5) with pull-up
 * - PA7: MOSI (AF5)
 *
 * LSM6DSL Protocol:
 * - Bit 7 of address byte: 1=read, 0=write
 * - Multi-byte reads use auto-increment (consecutive registers)
 */

#ifndef SPI_H
#define SPI_H

#include <stdbool.h>
#include <stdint.h>

/**
 * @brief Initialize SPI1 peripheral for LSM6DSL communication
 *
 * Configures GPIO pins, SPI1 peripheral in master mode with CPOL=1, CPHA=1,
 * and baud rate of APB2/64. Sets CS pin (PA4) to output high (inactive).
 *
 * @return true if initialization successful, false if SPI enable failed
 */
bool SPI1_Init(void);

/**
 * @brief Read single byte from SPI device register
 *
 * Sends register address with read bit set (bit 7 = 1), then reads response.
 * CS is asserted for the duration of the transaction.
 *
 * @param reg_addr Register address (0-127, bit 7 will be set for read)
 * @return Byte read from register
 */
uint8_t SPI_ReadByte(uint8_t reg_addr);

/**
 * @brief Write single byte to SPI device register
 *
 * Sends register address with write bit cleared (bit 7 = 0), then writes data.
 * CS is asserted for the duration of the transaction.
 *
 * @param reg_addr Register address (0-127, bit 7 will be cleared for write)
 * @param data Byte to write to register
 */
void SPI_WriteByte(uint8_t reg_addr, uint8_t data);

/**
 * @brief Read multiple consecutive bytes from SPI device
 *
 * Reads a block of consecutive registers using LSM6DSL auto-increment feature.
 * Sends address with read bit, then reads specified number of bytes.
 * CS is asserted for entire transaction.
 *
 * @param reg_addr Starting register address (bit 7 will be set for read)
 * @param size Number of bytes to read
 * @param data Buffer to store read data (must be at least 'size' bytes)
 */
void SPI_ReadBlock(uint8_t reg_addr, uint8_t size, uint8_t *data);

/**
 * @brief Assert chip select (pull CS pin low)
 *
 * Activates SPI device by pulling CS (PA4) low. Includes small delay for
 * setup time requirements.
 */
void SPI_CS_Low(void);

/**
 * @brief Deassert chip select (pull CS pin high)
 *
 * Deactivates SPI device by pulling CS (PA4) high. Includes small delay for
 * hold time requirements.
 */
void SPI_CS_High(void);

#endif
