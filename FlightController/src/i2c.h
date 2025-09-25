#ifndef I2C_H
#define I2C_H

#include <stdint.h>

// I2C initialization
void I2C1_Init(void);

// Basic I2C operations
uint8_t I2C_ReadByte(uint8_t device_addr, uint8_t reg_addr);
void I2C_WriteByte(uint8_t device_addr, uint8_t reg_addr, uint8_t data);
void I2C_ReadBlock(uint8_t device_addr, uint8_t reg_addr, uint8_t size,
                   uint8_t *data);

// Error handling
typedef enum {
  I2C_OK = 0,
  I2C_ERROR_TIMEOUT,
  I2C_ERROR_NACK,
  I2C_ERROR_BUS_BUSY
} I2C_Status_t;

#endif
