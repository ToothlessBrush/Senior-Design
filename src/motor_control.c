#include "motor_control.h"
#include "spi.h"
#include "stm32f411xe.h"
#include <stdint.h>

void SetMotorThrottleSPI(MotorID motor, uint16_t throttle) {
    if (throttle > 2000) {
        throttle = 2000;
    }

    // [15:14] unused | [13:11] motor ID (3 bits) | [10:0] throttle (11 bits)
    uint16_t packet = ((uint16_t)(motor & 0x07) << 11) | (throttle & 0x07FF);

    SPI2_CS_LOW();
    SPI2_Transmit((uint8_t)(packet >> 8)); // High byte
    SPI2_Transmit((uint8_t)(packet));      // Low byte
    SPI2_CS_HIGH();
}

void motor_enable(void) { GPIOC->ODR |= (1 << 15); }

void motor_disable(void) { GPIOC->ODR &= ~(1 << 15); }

bool motor_status_ok(void) { return (GPIOC->IDR & (1 << 14)) != 0; }
