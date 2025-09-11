// src/main.c
// Include only the device-specific header
#include "stm32f411xe.h"

void gpio_init_pa0_af(void) {
  // Enable GPIOA clock
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
  __DSB();

  // PA0 as AF (AF1 for TIM2_CH1)
  GPIOA->MODER &= ~(GPIO_MODER_MODER0);  // clear
  GPIOA->MODER |= (GPIO_MODER_MODER0_1); // 10: Alternate function
  GPIOA->AFR[0] &= ~(0xF << (0 * 4));
  GPIOA->AFR[0] |= (1 << (0 * 4)); // AF1 = TIM2
  // Optional: push-pull, no pull-up/down
  GPIOA->OTYPER &= ~(GPIO_OTYPER_OT_0);
  GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPDR0);

  // Set PA1 as general purpose output (01)
  GPIOA->MODER &= ~(GPIO_MODER_MODER1); // clear bits

  // Push-pull (default), no pull-up/down
  GPIOA->OTYPER &= ~(GPIO_OTYPER_OT_1);
  GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPDR1);

  GPIOA->MODER &= ~(GPIO_MODER_MODER2); // clear bits

  // Push-pull (default), no pull-up/down
  GPIOA->OTYPER &= ~(GPIO_OTYPER_OT_2);
  GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPDR2);
}

void tim2_pwm_init(void) {
  // Enable TIM2 clock (APB1)
  RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
  __DSB();

  // Reset TIM2 to known state (optional)
  RCC->APB1RSTR |= RCC_APB1RSTR_TIM2RST;
  RCC->APB1RSTR &= ~RCC_APB1RSTR_TIM2RST;

  // Set prescaler for 1 MHz timer clock (1 us tick)
  // PSC = (TIMclk / desired_tick) - 1 = (84,000,000 / 1,000,000) - 1 = 83
  TIM2->PSC = 15;

  // Auto-reload for 20 ms period: ARR = 20000 - 1
  TIM2->ARR = 20000 - 1;

  // PWM mode 1 on CH1: OC1M = 110, OC1PE = 1
  TIM2->CCMR1 &= ~TIM_CCMR1_CC1S; // CC1 configured as output
  TIM2->CCMR1 |= (6 << 4);        // 110 = PWM mode 1
  TIM2->CCMR1 |= TIM_CCMR1_OC1PE; // enable preload for CCR1

  // Enable capture/compare channel 1 output
  TIM2->CCER &= ~TIM_CCER_CC1P; // active high
  TIM2->CCER |= TIM_CCER_CC1E;

  // Enable auto-reload preload
  TIM2->CR1 |= TIM_CR1_ARPE;

  // Set initial duty to 1.5 ms (neutral): CCR1 = 1500
  TIM2->CCR1 = 1000;

  // Enable counter (upcounting)
  TIM2->CR1 |= TIM_CR1_CEN;
}

int main(void) {
  // If you need to set SystemCoreClock ensure it's 84 MHz before using PSC
  // formula above. SystemInit() from startup file typically sets clocks; ensure
  // it's called.

  gpio_init_pa0_af();
  tim2_pwm_init();

  TIM2->CCR1 = 2000; // 1.0 ms = full min

  while (1) {

    if (((GPIOA->IDR) & 0x6) == 0x0) {
      TIM2->CCR1 = 1900;
    } else if (((GPIOA->IDR) & 0x6) == 0x2) {
      TIM2->CCR1 = 1000;
    } else if (((GPIOA->IDR) & 0x6) == 0x6) {
      for (int i = 1000; i < 1900; i++) {
        TIM2->CCR1 = i;
        for (volatile int j = 0; j < 10000; j++)
          __NOP();
      }
    } else {
      TIM2->CCR1 = 1900;
    }

    // Example: sweep between 1ms and 2ms every 2 seconds
    // TIM2->CCR1 = 1500; // 1.0 ms = full min
    // GPIOA->ODR ^= (1u << 1);
    // for (volatile int i=0; i<64000000; ++i) __NOP();
    // TIM2->CCR1 = 2000; // 2.0 ms = full max
    // for (volatile int i=0; i<2000000; ++i) __NOP();
    // TIM2->CCR1 = 1500; // neutral
    // for (volatile int i=0; i<4000000; ++i) __NOP();
  }
}
