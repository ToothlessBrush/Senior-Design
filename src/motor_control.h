#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <stdbool.h>
#include <stdint.h>

/**
 * @brief Motor selection enum for SPI2 motor controller
 *
 * Defines motor identifiers for use with SetMotorThrottleSPI.
 * Each value has multiple bits set to prevent false triggering if
 * power is lost and all lines float to 0V while CS is active.
 */
typedef enum { Motor1 = 0x1, Motor2 = 0x2, Motor3 = 0x3, Motor4 = 0x4 } MotorID;

/**
 * @brief Send motor throttle command via SPI2
 *
 * Transmits throttle value to motor controller over SPI2. The motor controller
 * then forwards the command to the appropriate ESC.
 *
 * @param motor Motor identifier (Motor1, Motor2, Motor3, or Motor4)
 * @param throttle Throttle value (0-2000)
 */
void SetMotorThrottleSPI(MotorID motor, uint16_t throttle);

/** Assert PC14 to enable the external motor controller chip */
void motor_enable(void);

/** Deassert PC14 to disable the external motor controller chip */
void motor_disable(void);

/** Read PC15 motor status line; returns true when the chip signals ready */
bool motor_status_ok(void);

#endif
