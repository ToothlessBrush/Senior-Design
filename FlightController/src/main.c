#include "LSM6DSL.h"
#include "imu.h"
#include "pid.h"
#include "stm32f411xe.h"
#include "systick.h"
#include "uart.h"
#include <stdio.h>
#include <stdlib.h>

#define IMU_ODR_HZ 6660.0f
#define FIXED_DT (1.0f / IMU_ODR_HZ) // ~0.00015015 seconds

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

// config chip to use 25mhz HSL clock and PLL x4 for a total of 100mhz
void SystemClock_Config_100MHz_HSE(void) {
  // Enable HSE (your 25 MHz external crystal)
  RCC->CR |= RCC_CR_HSEON;
  while (!(RCC->CR & RCC_CR_HSERDY))
    ; // Wait for HSE to stabilize

  // Enable power control clock
  RCC->APB1ENR |= RCC_APB1ENR_PWREN;

  // Set voltage scaling to Scale 1 (needed for 100 MHz)
  PWR->CR |= PWR_CR_VOS;

  // Configure Flash latency: 3 wait states for 100 MHz @ 3.3V
  FLASH->ACR = FLASH_ACR_ICEN |       // Instruction cache enable
               FLASH_ACR_DCEN |       // Data cache enable
               FLASH_ACR_LATENCY_3WS; // 3 wait states

  // Configure PLL using HSE (25 MHz external crystal)
  // PLL input = HSE / PLLM = 25 MHz / 25 = 1 MHz
  // VCO = 1 MHz * PLLN = 1 MHz * 200 = 200 MHz
  // SYSCLK = VCO / PLLP = 200 MHz / 2 = 100 MHz
  // USB clock = VCO / PLLQ = 200 MHz / 4 = 50 MHz

  RCC->PLLCFGR = (25 << RCC_PLLCFGR_PLLM_Pos) |  // PLLM = 25
                 (200 << RCC_PLLCFGR_PLLN_Pos) | // PLLN = 200
                 (0 << RCC_PLLCFGR_PLLP_Pos) |   // PLLP = 2 (00 = /2)
                 RCC_PLLCFGR_PLLSRC_HSE |        // PLL source = HSE
                 (4 << RCC_PLLCFGR_PLLQ_Pos);    // PLLQ = 4

  // Enable PLL
  RCC->CR |= RCC_CR_PLLON;
  while (!(RCC->CR & RCC_CR_PLLRDY))
    ; // Wait until PLL is ready

  // Configure bus prescalers
  RCC->CFGR =
      (RCC->CFGR & ~RCC_CFGR_HPRE) | RCC_CFGR_HPRE_DIV1; // AHB = 100 MHz
  RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_PPRE1) |
              RCC_CFGR_PPRE1_DIV2; // APB1 = 50 MHz (max)
  RCC->CFGR =
      (RCC->CFGR & ~RCC_CFGR_PPRE2) | RCC_CFGR_PPRE2_DIV1; // APB2 = 100 MHz

  // Switch system clock to PLL
  RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_SW) | RCC_CFGR_SW_PLL;
  while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL)
    ; // Wait until PLL is used

  // Update SystemCoreClock variable
  SystemCoreClock = 100000000;
}

int main(void) {

  SystemClock_Config_100MHz_HSE();

  systick_init();

  led_init();
  enableIMU();
  uart_init();

  uart_send_string("Drone Started!\r\n");

  // Initialize IMU context
  IMUContext imu = {0};
  imu_init(&imu);

  // Initialize PID conttext
  PIDContext pid = {0};
  pid_init(&pid, 1.0, 0.0, 0.05);

  // Set target attitude
  pid.setpoints.roll = 0.0f;
  pid.setpoints.pitch = 0.0f;
  pid.setpoints.yaw = 0.0f;

  while (1) {
    if (imu_data_ready()) {

      // Update IMU measurements and attitude
      IMU_update_context(&imu, FIXED_DT);

      // Update PID controllers
      pid_update(&pid, &imu, FIXED_DT);

      static int sample_count = 0;
      sample_count++;
      if (sample_count >= 1000) {
        char buffer[100];

        // Convert floats to integers (whole part and decimal part)
        int roll_int = (int)(pid.output.roll * 100);
        int pitch_int = (int)(pid.output.pitch * 100);
        int yaw_int = (int)(pid.output.yaw * 100);

        snprintf(buffer, sizeof(buffer),
                 "Roll: %s%d.%02d, Pitch: %s%d.%02d, Yaw: %s%d.%02d\r\n",
                 (roll_int < 0 ? "-" : " "), abs(roll_int / 100),
                 abs(roll_int % 100), (pitch_int < 0 ? "-" : " "),
                 abs(pitch_int / 100), abs(pitch_int % 100),
                 (yaw_int < 0 ? "-" : " "), abs(yaw_int / 100),
                 abs(yaw_int % 100));
        uart_send_string(buffer);

        toggle_led();
        sample_count = 0;
      }
    }
  }

  return 0;
}
