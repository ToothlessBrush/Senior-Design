#ifndef SYSTEM_H
#define SYSTEM_H

// Initialize onboard LED (PC13)
#include "systick.h"
void led_init(void);

// Toggle LED state
void toggle_led(void);

void SystemClock_Config_100MHz_HSE(void);

#endif // SYSTEM_H
