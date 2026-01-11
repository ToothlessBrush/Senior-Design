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
