#ifndef SYSTICK_H
#define SYSTICK_H

#include <stdint.h>

void systick_init(void);
uint32_t millis(void);
void delay_ms(uint32_t ms);

void SystemClock_Config_100MHz_HSE(void);

#endif
