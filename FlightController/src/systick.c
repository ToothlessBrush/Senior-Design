#include "systick.h"
#include "stm32f4xx.h"

static volatile uint32_t systick_counter = 0;

void systick_init(void) {
  // Configure SysTick to trigger every 1ms
  // SysTick runs at system clock (100MHz)
  // To get 1ms: 100,000,000 / 1000 = 100,000
  SysTick->LOAD = 100000 - 1;
  SysTick->VAL = 0;
  SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | // Use processor clock
                  SysTick_CTRL_TICKINT_Msk |   // Enable interrupt
                  SysTick_CTRL_ENABLE_Msk;     // Enable SysTick
}

// This function is called every 1ms
void SysTick_Handler(void) { systick_counter++; }

uint32_t millis(void) { return systick_counter; }

void delay_ms(uint32_t ms) {
  uint32_t start = millis();
  while ((millis() - start) < ms)
    ;
}

void delay_us(uint32_t us) {
  // For microsecond delays, use busy wait
  // 100MHz = 100 cycles per microsecond
  for (uint32_t i = 0; i < us; i++) {
    for (volatile uint32_t j = 0; j < 25; j++) {
      __NOP();
    }
  }
}

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

  RCC->PLLCFGR = (25 << 0) |  // PLLM = 25
                 (200 << 6) | // PLLN = 200
                 (0 << 16) |   // PLLP = 2 (00 = /2)
                 RCC_PLLCFGR_PLLSRC_HSE |        // PLL source = HSE
                 (4 << 24);    // PLLQ = 4

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
