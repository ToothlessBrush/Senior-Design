#include "dshot.h"
#include <stdint.h>
#include <stm32f411xe.h>

void ConstructDshotFrame(dshotMotor* motor, uint16_t throttle)
{
    // Check for max throttle
    if (throttle > 2047) throttle = 2047; 
    
    motor->throttle = throttle;

    // Shift to make space for telemetry bit
    throttle = throttle << 1;

    // Cacluate crc
    uint16_t crc = (throttle ^ (throttle >> 4) ^ (throttle >> 8)) & 0xF;

    // Construct the 16 bit frame
    uint16_t frame = (throttle << 4) | crc;
    for (int i = 1; i < 17; i++)
    {
        if (frame & (1 << (15 - i)))
        {
            motor->dshotBuffer2[i] = dshotHigh;
        }
        else
        {
            motor->dshotBuffer2[i] = dshotLow;
        }
    }
    for (int i = 17; i < dmaTransferSize; i++) 
        motor->dshotBuffer2[i] = 0; // inter-frame gap
    motor->dshotBuffer2[0] = 0;
    
    motor->updateBuffer = true;
}

void InitDshot(dshotMotor* motor1, dshotMotor* motor2, dshotMotor* motor3, dshotMotor* motor4)
{
// ============================================================================================ //
// =============================== Initialize GPIO ============================================ //
// ============================================================================================ //
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN; // Enable clock on GPIO port B
    GPIOB->MODER &= ~0xF0F; // Clear function bits for B0,1,4,5

    //               B0      B1      B4        B5    
    GPIOB->MODER    |= 2 | (2<<2) | (2<<8) | (2<<10); // Set Pin 0-3 to alternate function mode'
    GPIOB->PUPDR    |= 2 | (2<<2) | (2<<8) | (2<<10);; // Enable pull-down resistor
    GPIOB->OSPEEDR  |= 3 | (3<<2) | (3<<8) | (3<<10);; // High output speed
    GPIOB->AFR[0] &= ~0xFF00FF; // Clear function
    GPIOB->AFR[0] |= (2 | (2<<4) | (2<<16) | (2<<20)); // Set alternate function to AF02, TIM3

// ============================================================================================ //
 // ===================== Initialize Timer ==================================================== //
// ============================================================================================ //
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN; // Enable clock for TIM3

    // Reset the timer
    RCC->APB1RSTR |= 2;
    RCC->APB1RSTR &= ~2;

    TIM3->CR1 = 0; // Reset the clock
    
    TIM3->CCMR1 &= ~(3 | (3<<8)); // Set as ouput on channel 1 and 2
    TIM3->CCMR1 |= (6<<4) | (6<<12); // Set to PWM mode 1 for CCR1, CCR2
    TIM3->CCMR1 |= (1<<3) | (1<<11); // Set Preload to true for CCR1, CCR2
    TIM3->CCMR2 &= ~(3 | (3<<8)); // Set as output on channel 3 and 4
    TIM3->CCMR2 |= (6<<4) | (6<<12); // Set to PWM mode 1 for CCR3, CCR4
    TIM3->CCMR2 |= (1<<3) | (1<<11); // Set Preload to true for CCR3, CCR4
    TIM3->ARR = dshotWidth; // Auto reload value set to 500

    TIM3->CCR1 = dshotLow; // Match register 1 initial value
    TIM3->CCR2 = dshotLow; // Match register 2 initial value
    TIM3->CCR3 = dshotLow; // Match register 3 initial value
    TIM3->CCR4 = dshotLow; // Match register 4 initial value

    TIM3->CCER &= ~(2 | (2<<4) | (2<<8) | (2<<12)); // Active high on output
    TIM3->CCER |=  (1 | (1<<4) | (1<<8) | (1<<12)); // Output to the pin B0, B1, B4, B5

    TIM3->CR1 |= (1<<7); // Enable auto-reload preload
    TIM3->CR2 &= ~(1<<3); // DMA request on CC match
    TIM3->DIER |= (0xF<<9); // Enable CC1, 2, 3, and 4 DMA request
    TIM3->EGR |= (0xF<<1); // Generate DMA request on cc1, 2, 3, and 4 match
    // TIM2->CR1 |= 1; // Enable the counter
    TIM3->PSC = 4; // Prescale to get 25 Mhz timer clock

// ============================================================================================ //
// ============== Initialize DMA ============================================================== //
// ============================================================================================ //
    RCC->AHB1ENR |= (1<<21); // Enable the clock for DMA1


// ==== Motor 1 configuration ==== //
// Tim3 channel 1 -> DMA 1 Stream 4 Channel 5

    DMA1_Stream4->CR = 0; // Reset the stream

    while(DMA1_Stream4->CR & 1) {}
    DMA1_Stream4->CR |= (5<<25); // Set to channel 5
    DMA1_Stream4->CR |= (2<<16); // High priority
    DMA1_Stream4->CR |= (1<<6); // Mem-to-periph direction
    DMA1_Stream4->CR &= ~(1<<9); // Peripheral address is fixed
    DMA1_Stream4->CR |= (1<<10); // Memory is incremented
    DMA1_Stream4->CR |= (1<<11); // Periph data size 16 bits
    DMA1_Stream4->CR |= (1<<13); // Mem size 16 bits
    DMA1_Stream4->CR |= (1<<4); // Transfer complete interrupt enabled
    // DMA1_Stream4->CR |= (1<<8); // Enable circular mode for testing

    DMA1_Stream4->NDTR = dmaTransferSize; // Buffer size to transfer
    DMA1_Stream4->PAR = (uint32_t) &TIM3->CCR1; // Set the peripheral memory address to 
    DMA1_Stream4->M0AR = (uint32_t) motor1->dshotBuffer; // Set the memory location to the buffer
    DMA1_Stream4->FCR = 0;

    // __NVIC_EnableIRQ(DMA1_Stream4_IRQn);


// ==== Motor 2 configuration ==== //
// Tim3 channel 2 -> DMA 1 Stream 5 Channel 5

    DMA1_Stream5->CR = 0; // Reset the stream

    while(DMA1_Stream5->CR & 1) {}
    DMA1_Stream5->CR |= (5<<25); // Set to channel 5
    DMA1_Stream5->CR |= (2<<16); // High priority
    DMA1_Stream5->CR |= (1<<6); // Mem-to-periph direction
    DMA1_Stream5->CR &= ~(1<<9); // Peripheral address is fixed
    DMA1_Stream5->CR |= (1<<10); // Memory is incremented
    DMA1_Stream5->CR |= (1<<11); // Periph data size 16 bits
    DMA1_Stream5->CR |= (1<<13); // Mem size 16 bits
    DMA1_Stream5->CR |= (1<<4); // Transfer complete interrupt enabled
    // DMA1_Stream5->CR |= (1<<8); // Enable circular mode for testing

    DMA1_Stream5->NDTR = dmaTransferSize; // Buffer size to transfer
    DMA1_Stream5->PAR = (uint32_t) &TIM3->CCR2; // Set the peripheral memory address to 
    DMA1_Stream5->M0AR = (uint32_t) motor2->dshotBuffer; // Set the memory location to the buffer
    DMA1_Stream5->FCR = 0;

    // __NVIC_EnableIRQ(DMA1_Stream5_IRQn);

// ==== Motor 3 configuration ==== //
// Tim3 channel 3 -> DMA 1 Stream 7 Channel 5

    DMA1_Stream7->CR = 0; // Reset the stream

    while(DMA1_Stream7->CR & 1) {}
    DMA1_Stream7->CR |= (5<<25); // Set to channel 5
    DMA1_Stream7->CR |= (2<<16); // High priority
    DMA1_Stream7->CR |= (1<<6); // Mem-to-periph direction
    DMA1_Stream7->CR &= ~(1<<9); // Peripheral address is fixed
    DMA1_Stream7->CR |= (1<<10); // Memory is incremented
    DMA1_Stream7->CR |= (1<<11); // Periph data size 16 bits
    DMA1_Stream7->CR |= (1<<13); // Mem size 16 bits
    DMA1_Stream7->CR |= (1<<4); // Transfer complete interrupt enabled
    // DMA1_Stream7->CR |= (1<<8); // Enable circular mode for testing

    DMA1_Stream7->NDTR = dmaTransferSize; // Buffer size to transfer
    DMA1_Stream7->PAR = (uint32_t) &TIM3->CCR3; // Set the peripheral memory address to 
    DMA1_Stream7->M0AR = (uint32_t) motor3->dshotBuffer; // Set the memory location to the buffer
    DMA1_Stream7->FCR = 0;

    // __NVIC_EnableIRQ(DMA1_Stream7_IRQn);


// ==== Motor 4 configuration ==== //
// Tim3 channel 4 -> DMA 1 Stream 2 Channel 5

    DMA1_Stream2->CR = 0; // Reset the stream

    while(DMA1_Stream2->CR & 1) {}
    DMA1_Stream2->CR |= (5<<25); // Set to channel 5 for TIM3 Ch3
    DMA1_Stream2->CR |= (2<<16); // High priority
    DMA1_Stream2->CR |= (1<<6); // Mem-to-periph direction
    DMA1_Stream2->CR &= ~(1<<9); // Peripheral address is fixed
    DMA1_Stream2->CR |= (1<<10); // Memory is incremented
    DMA1_Stream2->CR |= (1<<11); // Periph data size 16 bits
    DMA1_Stream2->CR |= (1<<13); // Mem size 16 bits
    DMA1_Stream2->CR |= (1<<4); // Transfer complete interrupt enabled
    // DMA1_Stream2->CR |= (1<<8); // Enable circular mode for testing

    DMA1_Stream2->NDTR = dmaTransferSize; // Buffer size to transfer
    DMA1_Stream2->PAR = (uint32_t) &TIM3->CCR4; // Set the peripheral memory address to 
    DMA1_Stream2->M0AR = (uint32_t) motor4->dshotBuffer; // Set the memory location to the buffer
    DMA1_Stream2->FCR = 0;

    // __NVIC_EnableIRQ(DMA1_Stream2_IRQn);

}

void InitiMotor(dshotMotor* motor)
{
    motor->updateBuffer = false;
    motor->dshotBuffer[0] = 0;
    for (int i = 1; i < 17; i++)
    {
        motor->dshotBuffer[i] = dshotLow;
        motor->dshotBuffer2[i] = dshotLow;
    }
    for(int i = 17; i < dmaTransferSize; i++)
    {
        motor->dshotBuffer[i] = 0;
        motor->dshotBuffer2[i] = 0;
    }
    motor->throttle = 48;
    ConstructDshotFrame(motor, motor->throttle);
}