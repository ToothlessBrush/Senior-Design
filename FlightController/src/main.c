#include "LSM6DSL.h"
#include "imu.h"
#include "spi.h"
#include "stm32f411xe.h"
#include <stdint.h>

#define IMU_ODR_HZ 6660.0f
#define FIXED_DT (1.0f / IMU_ODR_HZ) // ~0.00015015 seconds

void initGyroInterrupt(void) {
  // Enable GPIOA and SYSCFG clocks
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
  RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;

  // Configure PA0 as input with pull-down (since we want to detect HIGH)
  GPIOA->MODER &= ~(3UL << (0 * 2)); // Clear mode bits for PA0 (input mode)
  GPIOA->PUPDR &= ~(3UL << (0 * 2)); // Clear pull-up/down bits
  GPIOA->PUPDR |= (2UL << (0 * 2));  // Set pull-down (changed to pull-down)

  // Connect PA0 to EXTI0
  SYSCFG->EXTICR[0] &= ~SYSCFG_EXTICR1_EXTI0;   // Clear EXTI0 config
  SYSCFG->EXTICR[0] |= SYSCFG_EXTICR1_EXTI0_PA; // Connect PA0 to EXTI0

  // Configure EXTI0 for rising edge (HIGH)
  EXTI->IMR |= EXTI_IMR_MR0;    // Enable interrupt on line 0
  EXTI->RTSR |= EXTI_RTSR_TR0;  // Enable rising edge trigger
  EXTI->FTSR &= ~EXTI_FTSR_TR0; // Disable falling edge trigger
}

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

void toggle_led(void) { GPIOC->ODR ^= GPIO_ODR_ODR_13; }

// Simple delay function
void delay_ms(uint32_t ms) {
  // 16ms for now
  for (uint32_t i = 0; i < ms * 16000; i++) {
    __NOP(); // No operation - just burn CPU cycles
  }
}

int main(void) {

  led_init();
  initGyroInterrupt();
  SPI1_Init();
  enableIMU();

  int gyroRaw[3];
  int accRaw[3];
  float gyro[3];
  float acc[3];

  Attitude attitude = {0}; // assume we start on level ground of course

  while (1) {
    if (EXTI->PR & EXTI_PR_PR0) {
      EXTI->PR |= EXTI_PR_PR0; // I guess writing 1 to the pending bit clears it

      readGyroRaw(gyroRaw);
      readAccRaw(accRaw);
      gyroToRadPS(gyroRaw, gyro);
      accToG(accRaw, acc);

      updateOrientation(&attitude, acc, gyro, FIXED_DT);
    }
  }

  return 0;
}
