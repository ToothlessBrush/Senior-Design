#include "spi.h"
#include "stm32f411xe.h"
#include <stdbool.h>

bool SPI1_Init(void) {
  // Enable clocks
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN; // GPIO A
  RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;  // SPI1
  __DSB(); // Data Synchronization Barrier - ensure clocks are stable

  // Configure pins A5(SCK), A6(MISO), A7(MOSI)
  GPIOA->MODER &= ~((3 << (5 * 2)) | (3 << (6 * 2)) | (3 << (7 * 2)));
  GPIOA->MODER |=
      (2 << (5 * 2)) | (2 << (6 * 2)) | (2 << (7 * 2)); // Alternate function

  // Set alternate function to AF5 for SPI1 - clear first, then set
  GPIOA->AFR[0] &= ~((0xF << (5 * 4)) | (0xF << (6 * 4)) | (0xF << (7 * 4)));
  GPIOA->AFR[0] |=
      (5 << (5 * 4)) | (5 << (6 * 4)) | (5 << (7 * 4)); // AF5 for SPI1

  // Set high speed
  GPIOA->OSPEEDR |=
      (3 << (5 * 2)) | (3 << (6 * 2)) | (3 << (7 * 2)); // High speed

  // Configure pull-up/pull-down (important!)
  GPIOA->PUPDR &= ~((3 << (5 * 2)) | (3 << (6 * 2)) | (3 << (7 * 2)));
  GPIOA->PUPDR |= (1 << (6 * 2)); // Pull-up on MISO (A6)
  // SCK and MOSI don't need pull resistors in master mode

  // Configure A4 as CS (GPIO output)
  GPIOA->MODER &= ~(3 << (4 * 2));
  GPIOA->MODER |= (1 << (4 * 2));  // Output
  GPIOA->PUPDR &= ~(3 << (4 * 2)); // No pull resistor on CS
  GPIOA->ODR |= (1 << 4);          // CS high (inactive)

  // Configure SPI1
  SPI1->CR1 = 0;             // Reset
  SPI1->CR1 |= SPI_CR1_MSTR; // Master mode

  // Try slower baud rate first (divide by 64 instead of 32)
  SPI1->CR1 |= SPI_CR1_BR_2 | SPI_CR1_BR_0; // Baud rate /64 (slower)

  SPI1->CR1 |= SPI_CR1_CPOL; // Clock polarity high when idle
  SPI1->CR1 |= SPI_CR1_CPHA; // Clock phase - data capture on 2nd edge
  SPI1->CR1 |= SPI_CR1_SSM;  // Software slave management
  SPI1->CR1 |= SPI_CR1_SSI;  // Internal slave select
  SPI1->CR1 |= SPI_CR1_SPE;  // Enable SPI

  // Verify SPI is actually enabled
  if (!(SPI1->CR1 & SPI_CR1_SPE)) {
    return false;
  }
  return true;
}

void SPI_CS_Low(void) {
  GPIOA->ODR &= ~(1 << 4); // Pull CS low
  // Add small delay for setup time
  for (volatile int i = 0; i < 10; i++)
    ;
}

void SPI_CS_High(void) {
  GPIOA->ODR |= (1 << 4); // Pull CS high
  // Add small delay for hold time
  for (volatile int i = 0; i < 10; i++)
    ;
}

static uint8_t SPI_Transmit(uint8_t data) {
  // Clear any pending flags
  (void)SPI1->DR; // Dummy read to clear RXNE if set
  (void)SPI1->SR; // Dummy read of status register

  while (!(SPI1->SR & SPI_SR_TXE))
    ; // Wait for TX empty
  SPI1->DR = data;
  while (!(SPI1->SR & SPI_SR_RXNE))
    ; // Wait for RX not empty
  return SPI1->DR;
}

uint8_t SPI_ReadByte(uint8_t reg_addr) {
  uint8_t result;
  SPI_CS_Low();

  // LSM6DSL: bit 7 = R/W (1=read), bits 6-0 = address
  SPI_Transmit(reg_addr | 0x80); // Set read bit (bit 7)
  result = SPI_Transmit(0x00);   // Dummy byte to get data

  SPI_CS_High();
  return result;
}

void SPI_WriteByte(uint8_t reg_addr, uint8_t data) {
  SPI_CS_Low();

  // LSM6DSL: bit 7 = R/W (0=write), bits 6-0 = address
  SPI_Transmit(reg_addr & 0x7F); // Clear write bit (bit 7)
  SPI_Transmit(data);

  SPI_CS_High();
}

void SPI_ReadBlock(uint8_t reg_addr, uint8_t size, uint8_t *data) {
  SPI_CS_Low();

  // First byte: address with read bit
  SPI_Transmit(reg_addr | 0x80); // Set read bit

  // Read consecutive registers
  for (uint8_t i = 0; i < size; i++) {
    data[i] = SPI_Transmit(0x00); // Each dummy byte gets next register
  }

  SPI_CS_High();
}
