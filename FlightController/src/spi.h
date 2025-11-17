#ifndef SPI_H
#define SPI_H

#include <stdbool.h>
#include <stdint.h>

// SPI initialization
bool SPI1_Init(void);

// SPI operations
uint8_t SPI_ReadByte(uint8_t reg_addr);
void SPI_WriteByte(uint8_t reg_addr, uint8_t data);
void SPI_ReadBlock(uint8_t reg_addr, uint8_t size, uint8_t *data);

// CS (Chip Select) control
void SPI_CS_Low(void);
void SPI_CS_High(void);

#endif
