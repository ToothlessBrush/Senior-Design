// src/main.c
// Simple LED blink for STM32F411
#include "stm32f411xe.h"

void led_init(void) {
  // Enable GPIOC clock (onboard LED is usually on PC13)
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
  __DSB(); // Data Synchronization Barrier

  // Configure PC13 as general purpose output
  GPIOC->MODER &= ~(GPIO_MODER_MODER13);  // Clear bits
  GPIOC->MODER |= (GPIO_MODER_MODER13_0); // Set to 01 (output)

  // Configure as push-pull (default)
  GPIOC->OTYPER &= ~(GPIO_OTYPER_OT_13);

  // No pull-up/pull-down
  GPIOC->PUPDR &= ~(GPIO_PUPDR_PUPDR13);

  // Set output speed to low (optional)
  GPIOC->OSPEEDR &= ~(GPIO_OSPEEDER_OSPEEDR13);
}

void delay(volatile uint32_t count) {
  while (count--) {
    __NOP();
  }
}

int main(void) {
  led_init();

  while (1) {
    // Turn LED on (most onboard LEDs are active low, so we write 0)
    GPIOC->ODR &= ~(1U << 13); // Clear bit 13 (LED ON)
    delay(1000000);            // Delay

    // Turn LED off
    GPIOC->ODR |= (1U << 13); // Set bit 13 (LED OFF)
    delay(1000000);           // Delay
  }
}
