#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H
#include "dshot.h"

// Declare motor pointers as extern to avoid multiple definitions across translation units
extern dshotMotor* motor1;
extern dshotMotor* motor2;
extern dshotMotor* motor3;
extern dshotMotor* motor4;

void InitMotors();

void StartMotors();

void StopMotors();

void SetMotorThrottle(dshotMotor* motor, uint16_t throttle);

// Motor 1 dma interrupt handler
void DMA1_Stream4_IRQHandler();

// Motor 2 dma interrupt handler
void DMA1_Stream5_IRQHandler();

// Motor 3 dma interrupt handler
void DMA1_Stream7_IRQHandler();

// Motor 4 dma interrupt handler
void DMA1_Stream2_IRQHandler();

#endif