/**
 * @file system.h
 * @brief System initialization and onboard LED control
 *
 * Provides system clock configuration and onboard LED control functions
 * for STM32F411 microcontroller. LED is active-low on PC13 (STM32 "black pill"
 * development board).
 *
 * Clock Configuration:
 * - 25 MHz HSE (external crystal)
 * - PLL configured for 100 MHz system clock
 * - AHB: 100 MHz
 * - APB1: 50 MHz (max for F411)
 * - APB2: 100 MHz
 */

#ifndef SYSTEM_H
#define SYSTEM_H

#include "systick.h"

/**
 * @brief Initialize onboard LED (PC13)
 *
 * Configures PC13 as push-pull output for LED control. LED is active-low
 * (0 = on, 1 = off) on most STM32F411 development boards.
 */
void led_init(void);

/**
 * @brief Toggle LED state
 *
 * Inverts current LED state using XOR operation on output data register.
 */
void toggle_led(void);

/**
 * @brief Turn LED on
 *
 * Sets LED to on state (pulls PC13 low, as LED is active-low).
 */
void led_on(void);

/**
 * @brief Turn LED off
 *
 * Sets LED to off state (pulls PC13 high, as LED is active-low).
 */
void led_off(void);

/**
 * @brief Configure system clock to 100 MHz using HSE and PLL
 *
 * Configures STM32F411 to run at 100 MHz using 25 MHz external crystal:
 * - Enables HSE and waits for stabilization
 * - Configures PLL: (HSE / 25) * 200 / 2 = (25 / 25) * 200 / 2 = 100 MHz
 * - Sets flash latency to 3 wait states (required for 100 MHz at 3.3V)
 * - Enables instruction and data caches
 * - Configures bus prescalers: AHB /1, APB1 /2, APB2 /1
 * - Switches system clock source to PLL
 *
 * Clock Tree:
 * - SYSCLK: 100 MHz
 * - AHB (HCLK): 100 MHz
 * - APB1 (PCLK1): 50 MHz (timers run at 100 MHz)
 * - APB2 (PCLK2): 100 MHz
 *
 * Must be called before peripheral initialization.
 */
void SystemClock_Config_100MHz_HSE(void);

#endif // SYSTEM_H
