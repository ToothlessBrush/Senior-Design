/**
 * @file systick.h
 * @brief SysTick timer for millisecond timekeeping and delays
 *
 * Provides Arduino-style millis() function and blocking delay_ms() using
 * ARM Cortex-M4 SysTick timer. Timer runs at system clock (100 MHz) with
 * 1 ms interrupt period.
 *
 * Features:
 * - 1 ms resolution counter
 * - 32-bit millisecond counter (wraps after ~49.7 days)
 * - Blocking millisecond delay function
 */

#ifndef SYSTICK_H
#define SYSTICK_H

#include <stdint.h>

/**
 * @brief Initialize SysTick timer for 1 ms interrupts
 *
 * Configures SysTick to generate interrupts every 1 ms at 100 MHz system clock.
 * Enables SysTick interrupt and starts the timer. Must be called during system
 * initialization before using millis() or delay_ms().
 */
void systick_init(void);

/**
 * @brief Get millisecond counter value
 *
 * Returns the number of milliseconds since systick_init() was called.
 * Counter increments in SysTick_Handler ISR every 1 ms. Wraps to 0 after
 * approximately 49.7 days (2^32 milliseconds).
 *
 * @return Milliseconds since initialization
 */
uint32_t millis(void);

/**
 * @brief Blocking delay in milliseconds
 *
 * Busy-waits for specified number of milliseconds using millis() counter.
 * Blocks program execution. Safe across counter wraparound.
 *
 * @param ms Number of milliseconds to delay
 */
void delay_ms(uint32_t ms);

#endif
