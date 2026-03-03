#ifndef I2C_H
#define I2C_H

#include <stdint.h>

#define MAG_ADDRESS 0b0011100

// I2C initialization
void I2C1_Init(void);

// Basic I2C operations

void I2C1_WriteByte(uint8_t device_addr, uint8_t data);
void I2C1_WriteByte_NoStop(uint8_t device_addr, uint8_t data);
void I2C1_BulkWrite(uint8_t address, uint8_t *data, uint16_t length);
uint8_t I2C1_ReadByte(uint8_t address);
void I2C1_BulkRead(uint8_t address, uint8_t *data, uint16_t length);

// Error handling
typedef enum {
    I2C_OK = 0,
    I2C_ERROR_TIMEOUT,
    I2C_ERROR_NACK,
    I2C_ERROR_BUS_BUSY
} I2C_Status_t;

#endif
