#ifndef IMU_H
#define IMU_H

#include <stdbool.h>
#include <stdint.h>

/**
 * Biquad low-pass filter for noise reduction
 */
typedef struct {
    float b0, b1, b2;  // Numerator coefficients
    float a1, a2;      // Denominator coefficients (a0 normalized to 1)
    float x1, x2;      // Input history
    float y1, y2;      // Output history
} Biquad_t;

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
    float mag_offset[3];     // Magnetometer hard iron offset (gauss)
    float mag_scale[3];      // Magnetometer soft iron scale factors
} IMU_Calibration;

/**
 * contains the context of the imu including its current gyro acceleration,
 * acceleration, magnetometer, and attitude.
 */
typedef struct {
    int gyro_raw[3];
    int acc_raw[3];
    int mag_raw[3];
    float gyro[3];
    float acc[3];
    float mag[3];              // Magnetometer (gauss)
    Attitude attitude;
    IMU_Calibration cal;       // Calibration data
    Biquad_t gyro_filter[3];   // Low-pass filters for gyro (X, Y, Z)
    Biquad_t acc_filter[3];    // Low-pass filters for accel (X, Y, Z)
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
void readMagRaw(int m[]);
void gyroToRadPS(int gyro_raw[], float gyro_dps[]);
void accToG(int acc_raw[], float acc_g[]);
void magToGauss(int mag_raw[], float mag_gauss[]);
void updateOrientation(Attitude *attitude, float acc_g[], float gyro_rad_s[],
                       float dt);

bool imu_init(IMU *imu);
void IMU_update(IMU *imu, float dt);
bool imu_data_ready();

// Calibration functions
void IMU_calibrate_gyro(IMU *imu, uint16_t samples);
void IMU_calibrate_accel(IMU *imu, uint16_t samples);
void IMU_calibrate_mag(IMU *imu, uint16_t samples);

// Filter functions
void biquad_lpf_init(Biquad_t *filter, float cutoff_freq, float sample_freq);
float biquad_apply(Biquad_t *filter, float input);

// Helper functions
void writeAccReg(uint8_t reg, uint8_t value);
void writeGyrReg(uint8_t reg, uint8_t value);

// Version info
extern int BerryIMUversion;

#endif
