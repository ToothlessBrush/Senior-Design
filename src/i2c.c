#include "i2c.h"
#include "stm32f411xe.h"

#define I2C_TIMEOUT 5000000

void I2C1_Init(void) {
    // Enable GPIOB and I2C1 clocks
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;        // Enable GPIOB clock
    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;          // Enable I2C1 clock
    // __DSB();

    // Configure PB6 (SCL) and PB7 (SDA) for I2C
    GPIOB->MODER |= (2 << (6 * 2)) | (2 << (7 * 2)); // Set PB6 and PB7 to alternate function mode
    GPIOB->AFR[0] |= (4 << (6 * 4)) | (4 << (7 * 4)); // Set AF4 (I2C1) for PB6 and PB7
    GPIOB->OSPEEDR |= (3 << (6 * 2)) | (3 << (7 * 2)); // Set high speed for PB6 and PB7
    GPIOB->PUPDR &= ~((3 << (6 * 2)) | (3 << (7 * 2))); // No pull-up or pull-down

    // I2C1 configuration
    I2C1->CR1 = 0;                                 // Reset Control Register 1
    I2C1->CR2 = 50;                                // Set peripheral clock frequency to 50 MHz

    // Configure for 100 kHz
    // CCR = (APB1 Clock / (I2C Speed * 2)) - 1
    // This means: (50 MHz / (100 kHz * 2)) - 1 = 249
    I2C1->CCR = 249;                               // Set clock control register for 100 kHz
    I2C1->TRISE = 51;                              // Set rise time for 50 MHz (TRISE = 1 + (Tf/1.5))
    
    I2C1->CR1 |= I2C_CR1_PE;                       // Enable I2C1
}

void I2C1_WriteByte(uint8_t device_addr, uint8_t data) {
    int timeout = 0;

    // Start I2C
    I2C1->CR1 |= I2C_CR1_START | I2C_CR1_ACK;

    // Wait until start is generated
    while (!(I2C1->SR1 & I2C_SR1_SB) && timeout++ < I2C_TIMEOUT);

    if (timeout >= I2C_TIMEOUT)
        return; // Timeout error

    timeout = 0;

    // Send address
    I2C1->DR = device_addr << 1; // Write operation

    // Wait for address to be sent
    while (!(I2C1->SR1 & I2C_SR1_ADDR) && timeout++ < I2C_TIMEOUT);

    timeout = 0;

    // Clear ADDR flag
    (void)I2C1->SR2;

    // Send data
    I2C1->DR = data;

    // Wait for data transmission to complete
    while (!(I2C1->SR1 & I2C_SR1_BTF) && timeout++ < I2C_TIMEOUT);

    // Stop I2C
    I2C1->CR1 |= I2C_CR1_STOP;
}

void I2C1_WriteByte_NoStop(uint8_t device_addr, uint8_t data) {
    int timeout = 0;

    // Start I2C
    I2C1->CR1 |= I2C_CR1_START | I2C_CR1_ACK;

    // Wait until start is generated
    while (!(I2C1->SR1 & I2C_SR1_SB) && timeout++ < I2C_TIMEOUT);
    
    if (timeout >= I2C_TIMEOUT)
        return; // Timeout error
    timeout = 0;

    // Send address
    I2C1->DR = device_addr << 1; // Write operation

    // Wait for address to be sent
    while (!(I2C1->SR1 & I2C_SR1_ADDR) && timeout++ < I2C_TIMEOUT);

    timeout = 0;

    // Clear ADDR flag
    (void)I2C1->SR2;

    // Send data
    I2C1->DR = data;

    // Wait for data transmission to complete
    while (!(I2C1->SR1 & I2C_SR1_BTF) && timeout++ < I2C_TIMEOUT);
}

void I2C1_BulkWrite(uint8_t address, uint8_t *data, uint16_t length) {
    int timeout = 0;

    I2C1->CR1 |= I2C_CR1_START;

    while (!(I2C1->SR1 & I2C_SR1_SB) && timeout++ < I2C_TIMEOUT);

    if (timeout >= I2C_TIMEOUT)
        return; // Timeout error
    timeout = 0;
    

    I2C1->DR = address << 1; // Write operation

    // Wait for address to be sent
    while (!(I2C1->SR1 & I2C_SR1_ADDR) && timeout++ < I2C_TIMEOUT);
    (void)I2C1->SR2; // Clear ADDR flag

    for (uint16_t i = 0; i < length; i++) {
        I2C1->DR = data[i];
        while (!(I2C1->SR1 & I2C_SR1_BTF) && timeout++ < I2C_TIMEOUT);
        timeout = 0;
    }

    I2C1->CR1 |= I2C_CR1_STOP; // Stop I2C
}

uint8_t I2C1_ReadByte(uint8_t address) {
    int timeout = 0;

    I2C1->CR1 |= I2C_CR1_START;

    while (!(I2C1->SR1 & I2C_SR1_SB) && timeout++ < I2C_TIMEOUT);

    if (timeout >= I2C_TIMEOUT)
        return 0; // Timeout error
    timeout = 0;

    I2C1->DR = address << 1 | 0x01; // Read operation

    while (!(I2C1->SR1 & I2C_SR1_ADDR) && timeout++ < I2C_TIMEOUT);
    timeout = 0;

    (void)I2C1->SR2; // Clear ADDR

    while (!(I2C1->SR1 & I2C_SR1_BTF) && timeout++ < I2C_TIMEOUT);

    // Generate stop condition before reading
    I2C1->CR1 |= I2C_CR1_STOP;

    return I2C1->DR; // Read data
}

void I2C1_BulkRead(uint8_t address, uint8_t *data, uint16_t length) {
    int timeout = 0;

    I2C1->CR1 |= I2C_CR1_START;

    while (!(I2C1->SR1 & I2C_SR1_SB) && timeout++ < I2C_TIMEOUT);

    if (timeout >= I2C_TIMEOUT)
        return; // Timeout error
    timeout = 0;

    I2C1->DR = address << 1 | 0x01; // Read operation

    while (!(I2C1->SR1 & I2C_SR1_ADDR) && timeout++ < I2C_TIMEOUT);
    (void)I2C1->SR2; // Clear ADDR flag

    for (uint16_t i = 0; i < length; i++) {
        while (!(I2C1->SR1 & I2C_SR1_BTF) && timeout++ < I2C_TIMEOUT); // Wait for data to be received
        data[i] = I2C1->DR; // Read received data
        I2C1->CR1 |= I2C_CR1_ACK; // Acknowledge received byte
    }

    // Generate stop condition
    I2C1->CR1 |= I2C_CR1_STOP;
}
