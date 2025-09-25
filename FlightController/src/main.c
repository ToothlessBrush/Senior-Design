#include "LSM6DSL.h"
#include "imu.h"
#include "spi.h"
#include "stm32f411xe.h"
#include <stdint.h>

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

// Simple delay function
void delay_ms(uint32_t ms) {
  // 16ms for now
  for (uint32_t i = 0; i < ms * 16000; i++) {
    __NOP(); // No operation - just burn CPU cycles
  }
}

int main(void) {

  led_init();

  int gyro_raw[3];
  float dps[3];
  int acc_raw[3];
  float acc_g[3];

  SPI1_Init();
  if (verifyIMU() != 1) {
    while (1)
      ;
  }

  enableIMU();
  delay_ms(100);

  while (1) {
    readGyroRaw(gyro_raw);
    gyroToRadPS(gyro_raw, dps);
    readAccRaw(acc_raw);
    accToG(acc_raw, acc_g);
    delay_ms(100); // idk how fast it can go
  }

  return 0;
}
