#ifndef IMU_H
#define IMU_H

#include <stdint.h>

// IMU functions (same interface as original)
uint8_t verifyIMU(void);
void enableIMU(void);
void readAccRaw(int a[]);
void readGyroRaw(int g[]);
void gyroToRadPS(int gyro_raw[], float gyro_dps[]);
void accToG(int acc_raw[], float acc_g[]);

// Helper functions
void writeAccReg(uint8_t reg, uint8_t value);
void writeGyrReg(uint8_t reg, uint8_t value);

// Version info
extern int BerryIMUversion;

#endif
