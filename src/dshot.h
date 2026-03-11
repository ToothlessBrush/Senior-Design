#ifndef DSHOT_H
#define DSHOT_H

#define dshotWidth 132
#define dshotHigh 100
#define dshotLow 52
#define dmaTransferSize 48
#define true 1
#define false 0
#define pollTimerWidth 538

#include <stdint.h>

typedef struct {
    uint8_t updateBuffer;
    uint16_t dshotBuffer[dmaTransferSize];
    uint16_t dshotBuffer2[dmaTransferSize];
    uint16_t throttle;
    uint16_t temperature;
    uint16_t voltage;
    uint16_t current;
    uint16_t eRPM;
} dshotMotor;

void ConstructDshotFrame(dshotMotor* motor, uint16_t throttle);

void InitDshot(dshotMotor* motor1, dshotMotor* motor2, dshotMotor* motor3, dshotMotor* motor4);

void InitiMotor(dshotMotor* motor);

#endif