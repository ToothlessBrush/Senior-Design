#include "i2c.h"
#include "stm32f411xe.h"

#define I2C_TIMEOUT 10000

void I2C1_Init(void) {
    // Enable clocks
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN; // GPIO B
    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;  // I2C1

    // Configure pins PB6 (SCL) and PB7 (SDA)
    GPIOB->MODER &= ~((3 << (6 * 2)) | (3 << (7 * 2)));
    GPIOB->MODER |= (2 << (6 * 2)) | (2 << (7 * 2));   // Alternate function
    GPIOB->AFR[0] |= (4 << (6 * 4)) | (4 << (7 * 4));  // AF4 for I2C1
    GPIOB->OTYPER |= (1 << 6) | (1 << 7);              // Open drain
    GPIOB->PUPDR |= (1 << (6 * 2)) | (1 << (7 * 2));   // Pull-up
    GPIOB->OSPEEDR |= (3 << (6 * 2)) | (3 << (7 * 2)); // High speed

    // Configure I2C1
    I2C1->CR1 &= ~I2C_CR1_PE; // Disable I2C
    I2C1->CR2 = 42;           // 42MHz APB1 clock
    I2C1->CCR = 210;          // 100kHz standard mode
    I2C1->TRISE = 43;         // Rise time
    I2C1->CR1 |= I2C_CR1_PE;  // Enable I2C
}

static I2C_Status_t I2C_WaitFlag(volatile uint32_t *reg, uint32_t flag,
                                 uint8_t state) {
    uint32_t timeout = I2C_TIMEOUT;
    while ((((*reg) & flag) ? 1 : 0) != state) {
        if (--timeout == 0)
            return I2C_ERROR_TIMEOUT;
    }
    return I2C_OK;
}

uint8_t I2C_ReadByte(uint8_t device_addr, uint8_t reg_addr) {
    uint8_t data;

    // Start condition
    I2C1->CR1 |= I2C_CR1_START;
    if (I2C_WaitFlag(&I2C1->SR1, I2C_SR1_SB, 1) != I2C_OK)
        return 0;

    // Send device address + write
    I2C1->DR = device_addr << 1;
    if (I2C_WaitFlag(&I2C1->SR1, I2C_SR1_ADDR, 1) != I2C_OK)
        return 0;
    (void)I2C1->SR2; // Clear ADDR

    // Send register address
    I2C1->DR = reg_addr;
    if (I2C_WaitFlag(&I2C1->SR1, I2C_SR1_TXE, 1) != I2C_OK)
        return 0;

    // Restart + device address + read
    I2C1->CR1 |= I2C_CR1_START;
    if (I2C_WaitFlag(&I2C1->SR1, I2C_SR1_SB, 1) != I2C_OK)
        return 0;
    I2C1->DR = (device_addr << 1) | 1;
    if (I2C_WaitFlag(&I2C1->SR1, I2C_SR1_ADDR, 1) != I2C_OK)
        return 0;
    (void)I2C1->SR2;

    // Prepare for single byte read
    I2C1->CR1 &= ~I2C_CR1_ACK; // NACK
    I2C1->CR1 |= I2C_CR1_STOP; // Stop

    // Read data
    if (I2C_WaitFlag(&I2C1->SR1, I2C_SR1_RXNE, 1) != I2C_OK)
        return 0;
    data = I2C1->DR;

    return data;
}

void I2C_WriteByte(uint8_t device_addr, uint8_t reg_addr, uint8_t data) {
    // Start condition
    I2C1->CR1 |= I2C_CR1_START;
    if (I2C_WaitFlag(&I2C1->SR1, I2C_SR1_SB, 1) != I2C_OK)
        return;

    // Send device address + write
    I2C1->DR = device_addr << 1;
    if (I2C_WaitFlag(&I2C1->SR1, I2C_SR1_ADDR, 1) != I2C_OK)
        return;
    (void)I2C1->SR2;

    // Send register address
    I2C1->DR = reg_addr;
    if (I2C_WaitFlag(&I2C1->SR1, I2C_SR1_TXE, 1) != I2C_OK)
        return;

    // Send data
    I2C1->DR = data;
    if (I2C_WaitFlag(&I2C1->SR1, I2C_SR1_TXE, 1) != I2C_OK)
        return;

    // Stop condition
    I2C1->CR1 |= I2C_CR1_STOP;
}

void I2C_ReadBlock(uint8_t device_addr, uint8_t reg_addr, uint8_t size,
                   uint8_t *data) {
    for (uint8_t i = 0; i < size; i++) {
        data[i] = I2C_ReadByte(device_addr, reg_addr + i);
    }
}
