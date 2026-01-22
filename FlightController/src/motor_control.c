#include "motor_control.h"
#include "dshot.h"
#include <stdint.h>
#include <stm32f411xe.h>

// NOTE:
// This implementation has a prototype Bi-Directional DSHOT telemetry decoder
// To Enable telemetry reading, uncomment the EXTI interrupts in the DMA interrupt handlers

// Provide single definitions for the motor objects and pointers declared extern
// in the header.
static dshotMotor motor1_obj;
static dshotMotor motor2_obj;
static dshotMotor motor3_obj;
static dshotMotor motor4_obj;

dshotMotor *motor1 = &motor1_obj;
dshotMotor *motor2 = &motor2_obj;
dshotMotor *motor3 = &motor3_obj;
dshotMotor *motor4 = &motor4_obj;

int motor1_counter = 0;
int motor2_counter = 0;
int motor3_counter = 0;
int motor4_counter = 0;
int motor1_data = 0;
int motor2_data = 0;
int motor3_data = 0;
int motor4_data = 0;

const uint8_t gcr_table[] = {
    0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x9, 0xA,
    0xB, 0x0, 0xD, 0xE, 0xF, 0x0, 0x0, 0x2, 0x3, 0x0, 0x0,
    0x0, 0x0, 0x0, 0x0, 0x0, 0x1, 0x0, 0x4, 0xC, 0x0,
};

void Decode(int data, dshotMotor *motor) {
    int gcr = (data ^ (data >> 1));
    int telemetry = 0;

    for (int i = 0; i < 4; i++) {
        uint8_t nibble = (gcr >> (i * 5)) & 0x1F;
        telemetry = gcr_table[nibble] << (i * 4) | telemetry;
    }
    int crc = telemetry & 0xF;
    telemetry = telemetry >> 4 & 0xFFF;
    uint16_t calc_crc =
        ~(telemetry ^ (telemetry >> 4) ^ (telemetry >> 8)) & 0xF;
    if (crc != calc_crc) {
        return;
    }

    if (telemetry & (1 << 8)) {
        uint16_t data = telemetry & 0xFF;
        uint8_t exp = (telemetry >> 9) & 0x7;
        motor->eRPM = data << exp;
    } else {
        uint16_t data = telemetry & 0xFF;
        uint8_t type = (telemetry >> 8) & 0xF;
        switch (type) {
        case 0x02:
            motor->temperature = data;
            break;
        case 0x04:
            motor->voltage = data * 25;
            break;
        case 0x06:
            motor->current = data;
            break;
        }
    }
}

void InitMotors() {
    InitiMotor(motor1);
    InitiMotor(motor2);
    InitiMotor(motor3);
    InitiMotor(motor4);

    InitDshot(motor1, motor2, motor3, motor4);
}

void StartMotors() {
    __NVIC_EnableIRQ(DMA1_Stream4_IRQn);
    __NVIC_EnableIRQ(DMA1_Stream5_IRQn);
    __NVIC_EnableIRQ(DMA1_Stream7_IRQn);
    __NVIC_EnableIRQ(DMA1_Stream2_IRQn);
    DMA1_Stream4->CR |= 1;
    DMA1_Stream5->CR |= 1;
    DMA1_Stream7->CR |= 1;
    DMA1_Stream2->CR |= 1;
    TIM3->EGR |= (1 << 6); // Initialize an update
    TIM3->CR1 |= 1;        // Start the timer
}

void StopMotors() {
    DMA1_Stream4->CR &= ~1;
    DMA1_Stream5->CR &= ~1;
    DMA1_Stream7->CR &= ~1;
    DMA1_Stream2->CR &= ~1;
    __NVIC_DisableIRQ(DMA1_Stream4_IRQn);
    __NVIC_DisableIRQ(DMA1_Stream5_IRQn);
    __NVIC_DisableIRQ(DMA1_Stream7_IRQn);
    __NVIC_DisableIRQ(DMA1_Stream2_IRQn);
    while (DMA1_Stream4->CR & 1 && DMA1_Stream5->CR & 1 &&
           DMA1_Stream7->CR & 1 && DMA1_Stream2->CR & 1) {
    }
    TIM3->CR1 &= ~1; // Stop the timer
}

void SetMotorThrottle(dshotMotor *motor, uint16_t throttle) {

    // Cap throttle to valid range
    if(throttle > 1024)
        throttle = 1024;
    ConstructDshotFrame(motor, throttle);
}

// This function can be used to keep the motors running in the background
void IdleMotors() {
    DMA1_Stream4->CR |= (1<<8); // Enable circular mode
    NVIC_DisableIRQ(DMA1_Stream4_IRQn);
    DMA1_Stream5->CR |= (1<<8); // Enable circular mode 
    NVIC_DisableIRQ(DMA1_Stream5_IRQn);
    DMA1_Stream7->CR |= (1<<8); // Enable circular mode 
    NVIC_DisableIRQ(DMA1_Stream7_IRQn);
    DMA1_Stream2->CR |= (1<<8); // Enable circular mode 
    NVIC_DisableIRQ(DMA1_Stream2_IRQn);
}

// This function is used to restart the normal motor control after idling
void StopIdleMotors() {
    DMA1_Stream4->CR &= ~(1<<8); // Disable circular mode
    NVIC_EnableIRQ(DMA1_Stream4_IRQn);
    DMA1_Stream5->CR &= ~(1<<8); // Disable circular mode 
    NVIC_EnableIRQ(DMA1_Stream5_IRQn);
    DMA1_Stream7->CR &= ~(1<<8); // Disable circular mode 
    NVIC_EnableIRQ(DMA1_Stream7_IRQn);
    DMA1_Stream2->CR &= ~(1<<8); // Disable circular mode 
    NVIC_EnableIRQ(DMA1_Stream2_IRQn);
}

// Motor 1 dma interrupt handler
void DMA1_Stream4_IRQHandler() {
    if (DMA1->HISR & (1 << 5)) // If transfer complete interrupt
    {
        DMA1->HIFCR |= (1 << 5); // Clear the interrupt

        if (motor1->updateBuffer) {
            for (int i = 0; i < dmaTransferSize; i++) {
                motor1->dshotBuffer[i] = motor1->dshotBuffer2[i];
            }
            motor1->updateBuffer = false;
        }
        while (DMA1_Stream4->CR & 1) {
        }
        DMA1_Stream4->NDTR = dmaTransferSize;
        DMA1_Stream4->M0AR = (uint32_t)motor1->dshotBuffer;

        GPIOB->MODER |= (2 << 8);
        DMA1_Stream4->CR |= 1;
    } else if (DMA1->HISR & (1 << 4)) // If half transfer interrupt
    {
        DMA1->HIFCR |= (1 << 4); // Clear the interrupt

        GPIOB->MODER &= ~(3 << 8); // Set pin to input
        EXTI->PR |= (1 << 4);      // Clear the interrupt
        // __NVIC_EnableIRQ(EXTI4_IRQn);
    }
}

// Motor 2 dma interrupt handler
void DMA1_Stream5_IRQHandler() {
    if (DMA1->HISR & (1 << 11)) // If transfer complete interrupt
    {
        DMA1->HIFCR |= (1 << 11); // Clear the interrupt

        if (motor2->updateBuffer) {
            for (int i = 0; i < dmaTransferSize; i++) {
                motor2->dshotBuffer[i] = motor2->dshotBuffer2[i];
            }
            motor2->updateBuffer = false;
        }
        while (DMA1_Stream5->CR & 1) {
        }
        DMA1_Stream5->NDTR = dmaTransferSize;
        DMA1_Stream5->M0AR = (uint32_t)motor2->dshotBuffer;

        GPIOB->MODER |= (2 << 10);
        DMA1_Stream5->CR |= 1;
    } else if (DMA1->HISR & (1 << 10)) // If half transfer interrupt
    {
        DMA1->HIFCR |= (1 << 10); // Clear the interrupt

        GPIOB->MODER &= ~(3 << 10); // Set pin to input
        EXTI->PR |= (1 << 5);       // Clear the interrupt
        // __NVIC_EnableIRQ(EXTI9_5_IRQn);
    }
}

// Motor 3 dma interrupt handler
void DMA1_Stream7_IRQHandler() {
    if (DMA1->HISR & (1 << 27)) // If transfer complete interrupt
    {
        DMA1->HIFCR |= (1 << 27); // Clear the interrupt

        if (motor3->updateBuffer) {
            for (int i = 0; i < dmaTransferSize; i++) {
                motor3->dshotBuffer[i] = motor3->dshotBuffer2[i];
            }
            motor3->updateBuffer = false;
        }
        while (DMA1_Stream7->CR & 1) {
        }
        DMA1_Stream7->NDTR = dmaTransferSize;
        DMA1_Stream7->M0AR = (uint32_t)motor3->dshotBuffer;

        GPIOB->MODER |= (2 << 0);
        DMA1_Stream7->CR |= 1;
    } else if (DMA1->HISR & (1 << 26)) // If half transfer interrupt
    {
        DMA1->HIFCR |= (1 << 26); // Clear the interrupt

        GPIOB->MODER &= ~(3 << 0); // Set pin to input
        EXTI->PR |= (1 << 0);      // Clear the interrupt
        // __NVIC_EnableIRQ(EXTI0_IRQn);
    }
}

// Motor 4 dma interrupt handler
void DMA1_Stream2_IRQHandler() {
    // If transfer complete interrupt
    if (DMA1->LISR & (1 << 21)) {
        DMA1->LIFCR |= (1 << 21); // Clear the interrupt

        if (motor4->updateBuffer) {
            for (int i = 0; i < dmaTransferSize; i++) {
                motor4->dshotBuffer[i] = motor4->dshotBuffer2[i];
            }
            motor4->updateBuffer = false;
        }
        while (DMA1_Stream2->CR & 1) {
        }
        DMA1_Stream2->NDTR = dmaTransferSize;
        DMA1_Stream2->M0AR = (uint32_t)motor4->dshotBuffer;

        GPIOB->MODER |= (2 << 2);
        DMA1_Stream2->CR |= 1;
    } else if (DMA1->LISR & (1 << 20)) // If half transfer interrupt
    {
        DMA1->LIFCR |= (1 << 20); // Clear the interrupt

        GPIOB->MODER &= ~(3 << 2); // Set pin to input
        EXTI->PR |= (1 << 1);      // Clear the interrupt
        // __NVIC_EnableIRQ(EXTI1_IRQn);
    }
}

// GPIO interrupt handlers for telemetry reading
// Motor 1
void EXTI4_IRQHandler() {
    if (EXTI->PR & (1 << 4) && !(GPIOB->IDR & (1 << 4))) {
        EXTI->PR |= (1 << 4); // Clear the interrupt
        __NVIC_DisableIRQ(EXTI4_IRQn);
        __NVIC_EnableIRQ(TIM4_IRQn);
        motor1_data = 0;
        TIM4->CNT = 0;  // Restart the timer
        TIM4->CR1 |= 1; // Start the timer
    }
}

// Motor 2
void EXTI9_5_IRQHandler() {
    if (EXTI->PR & (1 << 5) && !(GPIOB->IDR & (1 << 5))) {
        EXTI->PR |= (1 << 5); // Clear the interrupt
        __NVIC_DisableIRQ(EXTI9_5_IRQn);
        __NVIC_EnableIRQ(TIM5_IRQn);
        motor2_data = 0;
        TIM5->CNT = 0;  // Restart the timer
        TIM5->CR1 |= 1; // Start the timer
    }
}

// Motor 3
void EXTI0_IRQHandler() {
    if (EXTI->PR & (1 << 0) && !(GPIOB->IDR & (1 << 0))) {
        EXTI->PR |= (1 << 0); // Clear the interrupt
        __NVIC_DisableIRQ(EXTI0_IRQn);
        __NVIC_EnableIRQ(TIM1_BRK_TIM9_IRQn);
        motor3_data = 0;
        TIM9->CNT = 0;  // Restart the timer
        TIM9->CR1 |= 1; // Start the timer
    }
}
// Motor 4
void EXTI1_IRQHandler() {
    if (EXTI->PR & (1 << 1) && !(GPIOB->IDR & (1 << 1))) {
        EXTI->PR |= (1 << 1); // Clear the interrupt
        __NVIC_DisableIRQ(EXTI1_IRQn);
        __NVIC_EnableIRQ(TIM1_UP_TIM10_IRQn);
        motor4_data = 0;
        TIM10->CNT = 0;  // Restart the timer
        TIM10->CR1 |= 1; // Start the timer
    }
}
// Motor 1 timer interrupt handler
void TIM4_IRQHandler() {
    if (TIM4->SR & (1 << 1)) {
        TIM4->SR &= ~(1 << 1); // Clear the interrupt
        motor1_counter++;
        motor1_data = (motor1_data << 1) | ((GPIOB->IDR & (1 << 4)) >> 4);
        if (motor1_counter > 19) {
            TIM4->CR1 &= ~1; // Stop the timer
            motor1_counter = 0;
            Decode(motor1_data, motor1);
            __NVIC_DisableIRQ(TIM4_IRQn);
        }
    }
}
// Motor 2 timer interrupt handler
void TIM5_IRQHandler() {
    if (TIM5->SR & (1 << 1)) {
        TIM5->SR &= ~(1 << 1); // Clear the interrupt
        motor2_counter++;
        motor2_data = (motor2_data << 1) | ((GPIOB->IDR & (1 << 5)) >> 5);
        if (motor2_counter > 19) {
            TIM5->CR1 &= ~1; // Stop the timer
            motor2_counter = 0;
            Decode(motor2_data, motor2);
            __NVIC_DisableIRQ(TIM5_IRQn);
        }
    }
}

// Motor 3 timer interrupt handler
void TIM1_BRK_TIM9_IRQHandler() {
    if (TIM9->SR & (1 << 1)) {
        TIM9->SR &= ~(1 << 1); // Clear the interrupt
        // GPIOB->ODR |= (1<<7);

        motor3_counter++;

        motor3_data = (motor3_data << 1) | ((GPIOB->IDR & (1 << 0)) >> 0);
        if (motor3_counter > 19) {
            TIM9->CR1 &= ~1; // Stop the timer
            motor3_counter = 0;
            Decode(motor3_data, motor3);
            __NVIC_DisableIRQ(TIM1_BRK_TIM9_IRQn);
        }
        // GPIOB->ODR &= ~(1<<7);
    }
}

// Motor 4 timer interrupt handler
void TIM1_UP_TIM10_IRQHandler() {
    if (TIM10->SR & (1 << 1)) {
        TIM10->SR &= ~(1 << 1); // Clear the interrupt

        motor4_counter++;
        motor4_data = (motor4_data << 1) | ((GPIOB->IDR & (1 << 1)) >> 1);

        if (motor4_counter > 19) {
            TIM10->CR1 &= ~1; // Stop the timer
            motor4_counter = 0;
            Decode(motor4_data, motor4);
            __NVIC_DisableIRQ(TIM1_UP_TIM10_IRQn);
        }
    }
}
