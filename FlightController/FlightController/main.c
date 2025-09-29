#include "RTE_Components.h"
#include <stdint.h>
#include CMSIS_device_header
#include <stm32f411xe.h>

#define dshotWidth 102
#define dshotHigh 77
#define dshotLow 39

uint32_t dshotBuffer[16] = {
    dshotHigh, dshotLow,  dshotLow,  dshotLow,  dshotLow, dshotLow,
    dshotHigh, dshotLow,  dshotHigh, dshotHigh, dshotLow, dshotLow,
    dshotLow,  dshotHigh, dshotHigh, dshotLow,
};

void InitGPIO() {
  RCC->AHB1ENR |= 1;     // Enable clock on GPIO port A
  GPIOA->MODER &= ~3;    // Clear function bits
  GPIOA->MODER |= 2;     // Set Pin 0 to alternate function mode'
  GPIOA->PUPDR |= 2;     // Enable pull-down resistor
  GPIOA->OSPEEDR |= 3;   // High output speed
  GPIOA->AFR[0] &= ~0xF; // Clear function
  GPIOA->AFR[0] |= 1;    // Set alternate function to A1, TIM2
}

void InitTimer() {
  RCC->APB1ENR |= 1; // Enable clock for TIM2

  // Reset the timer
  RCC->APB1RSTR |= 1;
  RCC->APB1RSTR &= ~1;

  TIM2->CR1 = 0; // Reset the clock

  TIM2->CCMR1 &= ~3;       // Set as ouput on channel 1
  TIM2->CCMR1 |= (6 << 4); // Set to PWM mode 1
  TIM2->CCMR1 |= (1 << 3); // Set Preload to true for CCR1
  TIM2->ARR = dshotWidth;  // Auto reload value set to 500
  TIM2->CCR1 = dshotLow;   // Match register 1 initial value
  TIM2->CCER &= ~2;        // Active high on output
  TIM2->CCER |= 1;         // Output to the pin A0
  TIM2->CR1 |= (1 << 7);   // Enable auto-reload preload
  TIM2->CR2 &= ~(1 << 3);  // DMA request on CC match
  TIM2->DIER |= (1 << 9);  // Enable CC1 DMA request
  TIM2->EGR |= (1 << 1);   // Generate DMA request on cc1 match
                           // TIM2->CR1 |= 1; // Enable the counter
                           // TIM2->PSC = 64000;
}

void DMAInit() {
  RCC->AHB1ENR |= (1 << 21); // Enable the clock for DMA1

  DMA1_Stream5->CR = 0; // Reset the stream

  while (DMA1_Stream5->CR & 1) {
  }
  DMA1_Stream5->CR |= (3 << 25);  // Set to stream 3 for TIM2 Ch1
  DMA1_Stream5->CR |= (2 << 16);  // High priority
  DMA1_Stream5->CR |= (1 << 6);   // Mem-to-periph direction
  DMA1_Stream5->CR &= ~(1 << 9);  // Peripheral address is fixed
  DMA1_Stream5->CR |= (1 << 10);  // Memory is incremented
  DMA1_Stream5->CR |= (10 << 11); // Periph data size 32 bits
  DMA1_Stream5->CR |= (10 << 13); // Mem size 32 bits
  DMA1_Stream5->CR |= (1 << 4);   // Transfer complete interrupt enabled
  // DMA1_Stream5->CR |= (1<<8); // Enable circular mode for testing

  DMA1_Stream5->NDTR = 16; // 16 items to transfer
  DMA1_Stream5->PAR =
      (uint32_t)&TIM2->CCR1; // Set the peripheral memory address to
  DMA1_Stream5->M0AR =
      (uint32_t)dshotBuffer; // Set the memory location to the buffer
  DMA1_Stream5->FCR = 0;

  __NVIC_EnableIRQ(DMA1_Stream5_IRQn);
}

void DMA1_Stream5_IRQHandler() {
  DMA1->HIFCR |= (1 << 11); // Clear the interrupt
  for (volatile int i = 0; i < 5; i++)
    ; // forced delay to wait for final bit
  TIM2->CCR1 = dshotWidth + 1;
  // TIM2->CCMR1 &= ~(7<<4); // Set to frozen
  // TIM2->CCMR1 |= (4<<4); // Set to active low
  TIM2->CR1 &= ~1; // Disable the timer
  while (DMA1_Stream5->CR & 1) {
  }
  DMA1_Stream5->NDTR = 16;
  DMA1_Stream5->M0AR = (uint32_t)dshotBuffer;
  for (volatile int i = 0; i < 16; i++)
    ; // Forced delay between dshot frames
  DMA1_Stream5->CR |= 1;
  // TIM2->CCMR1 |= (6<<4); // Set to PWM mode 1
  TIM2->CR1 |= 1;
}

int main() {
  InitGPIO();
  InitTimer();
  DMAInit();
  int mr = TIM2->CCR1;
  int dmaInt = DMA1_Stream5->NDTR;
  int buf = DMA1_Stream5->M0AR;

  DMA1_Stream5->CR |= 1; // Enable DMA
  TIM2->CR1 |= 1;        //  Enable  the timer

  while (1) {
    mr = TIM2->CCR1;
    dmaInt = DMA1_Stream5->NDTR;
    buf = DMA1_Stream5->M0AR;
  }
}
