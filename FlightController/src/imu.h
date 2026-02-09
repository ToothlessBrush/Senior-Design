#ifndef IMU_H
#define IMU_H

#include <stdbool.h>
#include <stdint.h>

/**
 * Biquad low-pass filter for noise reduction
 */
typedef struct {
    float b0, b1, b2; // Numerator coefficients
    float a1, a2;     // Denominator coefficients (a0 normalized to 1)
    float x1, x2;     // Input history
    float y1, y2;     // Output history
} Biquad_t;

/**
 * Orientation contains pitch, roll, and yaw
 */
typedef struct {
    float roll;
    float pitch;
    float yaw;
} Attitude;

typedef union {
    struct {
        float x;
        float y;
        float z;
    };
    float v[3];
} Vec3;

typedef union {
    struct {
        int x;
        int y;
        int z;
    };
    int v[3];
} IVec3;

typedef union {
    struct {
        float x;
        float y;
    };
    float v[2];
} Vec2;

typedef struct {
    float ax;
    float ay;
    float az;
} Acceleration;

/**
 * Calibration data for sensors
 */
typedef struct {
    Vec3 gyro_bias;  // Gyroscope bias (rad/s)
    Vec3 accel_bias; // Accelerometer bias (g)
    Vec3 mag_offset; // Magnetometer hard iron offset (gauss)
    Vec3 mag_scale;  // Magnetometer soft iron scale factors
} IMU_Calibration;

/**
 * contains the context of the imu including its current gyro acceleration,
 * acceleration, magnetometer, and attitude.
 */
typedef struct {
    IVec3 gyro_raw;
    IVec3 acc_raw;
    IVec3 mag_raw;
    Vec3 gyro;
    Vec3 acc;
    Vec3 mag; // Magnetometer (gauss)
    Attitude attitude;
    IMU_Calibration cal;     // Calibration data
    Biquad_t gyro_filter[3]; // Low-pass filters for gyro (X, Y, Z)
    Biquad_t acc_filter[3];  // Low-pass filters for accel (X, Y, Z)

    Vec2 velocity;
    Vec2 accel_hp;
    Vec2 accel_prev;
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
void readAccRaw(IVec3 *a);
void readGyroRaw(IVec3 *g);
void readMagRaw(IVec3 *m);
void gyroToRadPS(const IVec3 *gyro_raw, Vec3 *gyro_dps);
void accToG(const IVec3 *acc_raw, Vec3 *acc_g);
void magToGauss(const IVec3 *mag_raw, Vec3 *mag_gauss);
void updateOrientation(Attitude *attitude, const Vec3 *acc_g,
                       const Vec3 *gyro_rad_s, float dt);

bool imu_init(IMU *imu);
void IMU_update(IMU *imu, float dt);
bool imu_data_ready();

// Calibration functions
void IMU_calibrate(IMU *imu, uint16_t samples);
void IMU_calibrate_mag(IMU *imu, uint16_t samples);

// Filter functions
void biquad_lpf_init(Biquad_t *filter, float cutoff_freq, float sample_freq);
float biquad_apply(Biquad_t *filter, float input);

// Helper functions
void writeAccReg(uint8_t reg, uint8_t value);
void writeGyrReg(uint8_t reg, uint8_t value);

// reset
void IMU_reset_velocity(IMU *imu);

// Version info
extern int BerryIMUversion;

#endif
