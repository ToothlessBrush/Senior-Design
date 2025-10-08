#include "motor_control.h"
#include <stm32f411xe.h>

// Provide single definitions for the motor objects and pointers declared extern in the header.
static dshotMotor motor1_obj;
static dshotMotor motor2_obj;
static dshotMotor motor3_obj;
static dshotMotor motor4_obj;

dshotMotor* motor1 = &motor1_obj;
dshotMotor* motor2 = &motor2_obj;
dshotMotor* motor3 = &motor3_obj;
dshotMotor* motor4 = &motor4_obj;

void InitMotors()
{
    InitiMotor(motor1);
    InitiMotor(motor2);
    InitiMotor(motor3);
    InitiMotor(motor4);

    InitDshot(motor1, motor2, motor3, motor4);
}

void StartMotors()
{
    __NVIC_EnableIRQ(DMA1_Stream4_IRQn);
    __NVIC_EnableIRQ(DMA1_Stream5_IRQn);
    __NVIC_EnableIRQ(DMA1_Stream7_IRQn);
    __NVIC_EnableIRQ(DMA1_Stream2_IRQn);
    DMA1_Stream4->CR |= 1;
    DMA1_Stream5->CR |= 1;
    DMA1_Stream7->CR |= 1;
    DMA1_Stream2->CR |= 1;
    TIM3->EGR |= (1<<6);
    TIM3->CR1 |= 1; // Start the timer
}

void StopMotors()
{
    TIM3->CR1 &= ~1; // Stop the timer
    DMA1_Stream4->CR &= ~1;
    DMA1_Stream5->CR &= ~1;
    DMA1_Stream7->CR &= ~1;
    DMA1_Stream2->CR &= ~1;
    __NVIC_DisableIRQ(DMA1_Stream4_IRQn);
    __NVIC_DisableIRQ(DMA1_Stream5_IRQn);
    __NVIC_DisableIRQ(DMA1_Stream7_IRQn);
    __NVIC_DisableIRQ(DMA1_Stream2_IRQn);
    while(DMA1_Stream4->CR & 1 && DMA1_Stream5->CR & 1 && DMA1_Stream7->CR & 1 && DMA1_Stream2->CR & 1) {}
}

void SetMotorThrottle(dshotMotor* motor, uint16_t throttle)
{
    ConstructDshotFrame(motor, throttle);
}

// Motor 1 dma interrupt handler
void DMA1_Stream4_IRQHandler()
{
    DMA1->HIFCR |= (1<<5); // Clear the interrupt

    if (motor1->updateBuffer)
    {
        for (int i = 0; i < dmaTransferSize; i++)
        {
            motor1->dshotBuffer[i] = motor1->dshotBuffer2[i];
        }
        motor1->updateBuffer = false;
    }
    while(DMA1_Stream4->CR & 1) {}
    DMA1_Stream4->NDTR = dmaTransferSize;
    DMA1_Stream4->M0AR = (uint32_t) motor1->dshotBuffer;
    DMA1_Stream4->CR |= 1;
}

// Motor 2 dma interrupt handler
void DMA1_Stream5_IRQHandler()
{
    DMA1->HIFCR |= (1<<11); // Clear the interrupt

    if (motor2->updateBuffer)
    {
        for (int i = 0; i < dmaTransferSize; i++)
        {
            motor2->dshotBuffer[i] = motor2->dshotBuffer2[i];
        }
        motor2->updateBuffer = false;
    }
    while(DMA1_Stream5->CR & 1) {}
    DMA1_Stream5->NDTR = dmaTransferSize;
    DMA1_Stream5->M0AR = (uint32_t) motor2->dshotBuffer;
    DMA1_Stream5->CR |= 1;
}

// Motor 3 dma interrupt handler
void DMA1_Stream7_IRQHandler()
{
    DMA1->HIFCR |= (1<<27); // Clear the interrupt

    if (motor3->updateBuffer)
    {
        for (int i = 0; i < dmaTransferSize; i++)
        {
            motor3->dshotBuffer[i] = motor3->dshotBuffer2[i];
        }
        motor3->updateBuffer = false;
    }
    while(DMA1_Stream7->CR & 1) {}
    DMA1_Stream7->NDTR = dmaTransferSize;
    DMA1_Stream7->M0AR = (uint32_t) motor3->dshotBuffer;
    DMA1_Stream7->CR |= 1;
}

// Motor 4 dma interrupt handler
void DMA1_Stream2_IRQHandler()
{
    DMA1->LIFCR |= (1<<21); // Clear the interrupt

    if (motor4->updateBuffer)
    {
        for (int i = 0; i < dmaTransferSize; i++)
        {
            motor4->dshotBuffer[i] = motor4->dshotBuffer2[i];
        }
        motor4->updateBuffer = false;
    }
    while(DMA1_Stream2->CR & 1) {}
    DMA1_Stream2->NDTR = dmaTransferSize;
    DMA1_Stream2->M0AR = (uint32_t) motor4->dshotBuffer;
    DMA1_Stream2->CR |= 1;
}