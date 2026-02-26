#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H
#include "dshot.h"

#define throttleMin 47
#define throttleMax 1023

// Declare motor pointers as extern to avoid multiple definitions across
// translation units
extern dshotMotor *motor1;
extern dshotMotor *motor2;
extern dshotMotor *motor3;
extern dshotMotor *motor4;

void InitMotors();

void StartMotors();

void StopMotors();

void SetMotorThrottle(dshotMotor *motor, uint16_t throttle);

// This function can be used to keep the motors running in the background
void IdleMotors();

// This function is used to restart the normal motor control after idling
void StopIdleMotors();

// Motor 1 dma interrupt handler
void DMA1_Stream4_IRQHandler();

// Motor 2 dma interrupt handler
void DMA1_Stream5_IRQHandler();

// Motor 3 dma interrupt handler
void DMA1_Stream7_IRQHandler();

// Motor 4 dma interrupt handler
void DMA1_Stream2_IRQHandler();

/**
 * @brief Motor selection enum for SPI2 motor controller
 *
 * Defines motor identifiers for use with SetMotorThrottle function.
 * Each value has multiple bits set to prevent false triggering if
 * power is lost and all lines float to 0V while CS is active.
 */
typedef enum {
    Motor1 = 0x11, // 0b00010001
    Motor2 = 0x22, // 0b00100010
    Motor3 = 0x44, // 0b01000100
    Motor4 = 0x88  // 0b10001000
} MotorID;

/**
 * @brief Send motor throttle command via SPI2
 *
 * Transmits throttle value to motor controller over SPI2. The motor controller
 * then forwards the command to the appropriate ESC for eRPM calibration.
 *
 * @param motor Motor identifier (Motor1, Motor2, Motor3, or Motor4)
 * @param throttle Throttle value (0-2000)
 */
void SetMotorThrottleSPI(MotorID motor, uint16_t throttle);

#endif
