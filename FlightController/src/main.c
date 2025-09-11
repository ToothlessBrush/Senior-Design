// src/main.c
#include <stdint.h>

// Include only the device-specific header
#include "stm32f411xe.h"

// Simple delay function
void delay_ms(uint32_t ms) {
  // Rough delay, assuming system clock
  for (uint32_t i = 0; i < ms * 1000; i++) {
    __asm__("nop");
  }
}

// System clock configuration (simplified)
void SystemClock_Config(void) {
  // Enable HSI
  RCC->CR |= RCC_CR_HSION;
  while (!(RCC->CR & RCC_CR_HSIRDY))
    ;

  // Use HSI as system clock (16 MHz)
  RCC->CFGR &= ~RCC_CFGR_SW;
  RCC->CFGR |= RCC_CFGR_SW_HSI;
  while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_HSI)
    ;
}

// GPIO initialization for LED (assuming PC13 like many STM32 boards)
void GPIO_Init(void) {
  // Enable GPIOC clock
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;

  // Configure PC13 as output
  GPIOC->MODER &= ~(3U << (13 * 2)); // Clear mode bits
  GPIOC->MODER |= (1U << (13 * 2));  // Set as output

  // Set output speed to low
  GPIOC->OSPEEDR &= ~(3U << (13 * 2));

  // Set as push-pull output
  GPIOC->OTYPER &= ~(1U << 13);

  // No pull-up/pull-down
  GPIOC->PUPDR &= ~(3U << (13 * 2));
}

int main(void) {
  // Initialize system clock
  SystemClock_Config();

  // Initialize GPIO
  GPIO_Init();

  while (1) {
    // Toggle LED on PC13
    GPIOC->ODR ^= (1U << 13);
    delay_ms(500);
  }

  return 0;
}
