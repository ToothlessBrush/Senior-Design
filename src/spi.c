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

// ========== SPI2 Motor Controller Functions ==========

bool SPI2_Init(void) {
    // Enable clocks
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN; // GPIO B
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN; // GPIO C
    RCC->APB1ENR |= RCC_APB1ENR_SPI2EN;  // SPI2
    __DSB(); // Data Synchronization Barrier - ensure clocks are stable

    // B0, B1, B4, B5 are driven by another MCU on the same lines — configure as
    // floating inputs so this MCU does not interfere with those signals.
    GPIOB->MODER &= ~((3 << (0 * 2)) | (3 << (1 * 2)) | (3 << (4 * 2)) |
                      (3 << (5 * 2))); // Input (00)
    GPIOB->PUPDR &=
        ~((3 << (0 * 2)) | (3 << (1 * 2)) | (3 << (4 * 2)) | (3 << (5 * 2)));
    GPIOB->PUPDR |= ((2 << (0 * 2)) | (2 << (1 * 2)) | (2 << (4 * 2)) |
                     (2 << (5 * 2))); // Pull-down (10)

    // Configure pins B13(SCK), B15(MOSI) as alternate function
    GPIOB->MODER &= ~((3 << (13 * 2)) | (3 << (15 * 2)));
    GPIOB->MODER |= (2 << (13 * 2)) | (2 << (15 * 2)); // Alternate function

    // Set alternate function to AF5 for SPI2
    GPIOB->AFR[1] &= ~((0xF << ((13 - 8) * 4)) | (0xF << ((15 - 8) * 4)));
    GPIOB->AFR[1] |=
        (5 << ((13 - 8) * 4)) | (5 << ((15 - 8) * 4)); // AF5 for SPI2

    // Set high speed
    GPIOB->OSPEEDR |= (3 << (13 * 2)) | (3 << (15 * 2)); // High speed

    // Configure B12 as CS (GPIO output)
    GPIOB->MODER &= ~(3 << (12 * 2));
    GPIOB->MODER |= (1 << (12 * 2));  // Output
    GPIOB->PUPDR &= ~(3 << (12 * 2)); // No pull resistor on CS
    GPIOB->ODR |= (1 << 12);          // CS high (inactive)

    // Configure SPI2
    SPI2->CR1 = 0;             // Reset
    SPI2->CR1 |= SPI_CR1_MSTR; // Master mode

    // Baud rate /64 (APB1 is typically 50MHz, so SPI2 will run at ~781kHz)
    SPI2->CR1 |= SPI_CR1_BR_2 | SPI_CR1_BR_0;

    // CPOL=0, CPHA=0 (SPI Mode 0) - adjust if motor controller needs different
    // mode SPI2->CR1 |= SPI_CR1_CPOL; // Uncomment for CPOL=1 SPI2->CR1 |=
    // SPI_CR1_CPHA; // Uncomment for CPHA=1

    SPI2->CR1 |= SPI_CR1_SSM; // Software slave management
    SPI2->CR1 |= SPI_CR1_SSI; // Internal slave select
    SPI2->CR1 |= SPI_CR1_SPE; // Enable SPI

    SPI2->CR1 |= (1 << 11);

    // Verify SPI is actually enabled
    if (!(SPI2->CR1 & SPI_CR1_SPE)) {
        return false;
    }

    // PC14: motor status (input, no pull)
    GPIOC->MODER &= ~(3 << (14 * 2)); // Input (00)
    GPIOC->PUPDR &= ~(3 << (14 * 2)); // No pull

    // PC15: motor enable (output), default low (disabled)
    GPIOC->MODER &= ~(3 << (15 * 2));
    GPIOC->MODER |= (1 << (15 * 2)); // Output
    GPIOC->ODR &= ~(1 << 15);        // Low = disabled

    return true;
}

void SPI2_CS_LOW(void) {
    GPIOB->ODR &= ~(1 << 12); // Pull CS low
    // Add small delay for setup time
    for (volatile int i = 0; i < 10; i++)
        ;
}

void SPI2_CS_HIGH(void) {
    GPIOB->ODR |= (1 << 12); // Pull CS high
    // Add small delay for hold time
    for (volatile int i = 0; i < 10; i++)
        ;
}

void SPI2_Transmit(uint8_t data) {
    // Clear any pending flags
    (void)SPI2->DR; // Dummy read to clear RXNE if set
    (void)SPI2->SR; // Dummy read of status register

    while (!(SPI2->SR & SPI_SR_TXE))
        ; // Wait for TX empty
    SPI2->DR = data;
    while (!(SPI2->SR & SPI_SR_RXNE))
        ;           // Wait for RX not empty (even though we don't use the data)
    (void)SPI2->DR; // Read to clear RXNE flag
}
void SPI2_Transmit16(uint16_t data) {
    // Clear any pending flags
    (void)SPI2->DR; // Dummy read to clear RXNE if set
    (void)SPI2->SR; // Dummy read of status register

    while (!(SPI2->SR & SPI_SR_TXE))
        ; // Wait for TX empty
    SPI2->DR = data;
    while (!(SPI2->SR & SPI_SR_RXNE))
        ;           // Wait for RX not empty (even though we don't use the data)
    (void)SPI2->DR; // Read to clear RXNE flag
}
