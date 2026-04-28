#include "system.h"
#include "stm32f411xe.h"
#include <string.h>

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

void led_on(void) { GPIOC->ODR &= ~GPIO_ODR_ODR_13; } // Active low

void led_off(void) { GPIOC->ODR |= GPIO_ODR_ODR_13; } // Active low

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

    RCC->PLLCFGR = (25 << 0) |              // PLLM = 25
                   (200 << 6) |             // PLLN = 200
                   (0 << 16) |              // PLLP = 2 (00 = /2)
                   RCC_PLLCFGR_PLLSRC_HSE | // PLL source = HSE
                   (4 << 24);               // PLLQ = 4

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

void flash_unlock(void) {
    if (!(FLASH->CR & FLASH_CR_LOCK)) {
        return;
    }
    while (FLASH->SR & FLASH_SR_BSY)
        ;
    FLASH->KEYR = 0x45670123;
    FLASH->KEYR = 0xCDEF89AB;
}

void flash_lock(void) { FLASH->CR |= FLASH_CR_LOCK; }

// Erase a sector (0-7 on F411, voltage range 3.3V = parallelism x32)
void flash_erase_sector(uint8_t sector) {
    while (FLASH->SR & FLASH_SR_BSY)
        ;

    FLASH->CR &= ~FLASH_CR_SNB;
    FLASH->CR |= FLASH_CR_SER | (sector << FLASH_CR_SNB_Pos);
    FLASH->CR |= FLASH_CR_STRT;

    while (FLASH->SR & FLASH_SR_BSY)
        ;

    FLASH->CR &= ~FLASH_CR_SER; // clear erase mode
}

// Write a buffer of 32-bit words to a flash address
// addr must be 4-byte aligned, must be pre-erased
bool flash_write(uint32_t addr, const uint32_t *data, uint32_t len_words) {
    while (FLASH->SR & FLASH_SR_BSY)
        ;

    FLASH->CR &= ~FLASH_CR_PSIZE;  // clear parallelism
    FLASH->CR |= FLASH_CR_PSIZE_1; // x32 (for 3.3V supply)
    FLASH->CR |= FLASH_CR_PG;      // enable programming

    for (uint32_t i = 0; i < len_words; i++) {
        *(volatile uint32_t *)(addr + i * 4) = data[i];
        while (FLASH->SR & FLASH_SR_BSY)
            ;

        if (FLASH->SR & (FLASH_SR_PGSERR | FLASH_SR_PGPERR | FLASH_SR_WRPERR)) {
            FLASH->SR |= FLASH_SR_PGSERR | FLASH_SR_PGPERR | FLASH_SR_WRPERR;
            return false; // bail on error
        }
    }

    FLASH->CR &= ~FLASH_CR_PG; // disable programming

    return true;
}

// Save config to a flash sector — full erase + rewrite cycle
bool flash_save(uint8_t sector, uint32_t sector_addr, const void *data,
                uint32_t size_bytes) {
    uint32_t len_words = (size_bytes + 3) / 4; // round up to word boundary

    flash_unlock();
    flash_erase_sector(sector);
    bool status = flash_write(sector_addr, (const uint32_t *)data, len_words);
    flash_lock();

    return status;
}

// Read back — flash is memory-mapped, just cast the address
void flash_read(uint32_t addr, void *out, uint32_t size_bytes) {
    memcpy(out, (const void *)addr, size_bytes);
}

uint32_t crc32_compute(const void *data, uint32_t size_bytes) {
    if (!(RCC->AHB1ENR & RCC_AHB1ENR_CRCEN)) {
        RCC->AHB1ENR |= RCC_AHB1ENR_CRCEN;
        __DSB();
    }
    CRC->CR = CRC_CR_RESET;
    const uint32_t *words = (const uint32_t *)data;
    uint32_t n = size_bytes / 4;
    for (uint32_t i = 0; i < n; i++) {
        CRC->DR = words[i];
    }
    return CRC->DR;
}
