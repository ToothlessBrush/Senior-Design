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
 * Calibration data for sensors
 */
typedef struct {
    float gyro_bias[3];      // Gyroscope bias (rad/s)
    float accel_bias[3];     // Accelerometer bias (g)
} IMU_Calibration;

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
    IMU_Calibration cal;     // Calibration data
} IMU;

/**
 * check if the IMU is pluged in a and ready via spi
 */
bool verifyIMU(void);

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

bool imu_init(IMU *imu);
void IMU_update(IMU *imu, float dt);
bool imu_data_ready();

// Calibration functions
void IMU_calibrate_gyro(IMU *imu, uint16_t samples);
void IMU_calibrate_accel(IMU *imu, uint16_t samples);

// Helper functions
void writeAccReg(uint8_t reg, uint8_t value);
void writeGyrReg(uint8_t reg, uint8_t value);

// Version info
extern int BerryIMUversion;

#endif
