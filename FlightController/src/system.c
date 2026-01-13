#include "system.h"
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
