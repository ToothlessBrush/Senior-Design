#ifndef IMU_H
#define IMU_H

#include <stdbool.h>
#include <stdint.h>

/**
 * Orientation contains pitch, roll, and yaw
 */
typedef struct {
  float roll;
  float pitch;
  float yaw;
} Attitude;

/**
 * contains the context of the imu including its current gyro acceleration,
 * acceleration, and attitude.
 */
typedef struct {
  int gyro_raw[3];
  int acc_raw[3];
  float gyro[3];
  float acc[3];
  Attitude attitude;
} IMUContext;

/**
 * check if the IMU is pluged in a and ready via spi
 */
bool verifyIMU(void);

/**
 * enables the imu by setting up spi and configuring the DSL.
 */
void enableIMU(void);

/**
 * read the raw acceleration data
 *
 * @param a[] 3-axis acceleration data
 */
void readAccRaw(int a[]);
void readGyroRaw(int g[]);
void gyroToRadPS(int gyro_raw[], float gyro_dps[]);
void accToG(int acc_raw[], float acc_g[]);
void updateOrientation(Attitude *attitude, float acc_g[], float gyro_rad_s[],
                       float dt);

void imu_init(IMUContext *ctx);
void IMU_update_context(IMUContext *ctx, float dt);
bool imu_data_ready();

// Helper functions
void writeAccReg(uint8_t reg, uint8_t value);
void writeGyrReg(uint8_t reg, uint8_t value);

// Version info
extern int BerryIMUversion;

#endif
