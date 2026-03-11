/**
 * @file imu.h
 * @brief IMU driver for LSM6DSL accelerometer/gyroscope sensor
 *
 * Provides high-speed (6660 Hz) IMU data acquisition with:
 * - 3-axis gyroscope (±500 dps range, converted to rad/s)
 * - 3-axis accelerometer (±8g range)
 * - Complementary filter for attitude estimation (roll, pitch, yaw)
 * - Biquad low-pass filters for noise reduction (motor vibration rejection)
 * - Bias calibration for gyro and accelerometer
 * - Velocity estimation from high-pass filtered acceleration
 * - Interrupt-driven data ready signal for precise timing
 *
 * Hardware: LSM6DSL via SPI1, data ready interrupt on PA0 (EXTI0)
 * Update Rate: 6660 Hz (150 μs period)
 *
 * Coordinate System (NED - North-East-Down):
 * - X: Forward (nose direction)
 * - Y: Right (right wing direction)
 * - Z: Down
 * - Roll: Rotation about X (positive = right wing down)
 * - Pitch: Rotation about Y (positive = nose up)
 * - Yaw: Rotation about Z (positive = clockwise from above)
 */

#ifndef IMU_H
#define IMU_H

#include <stdbool.h>
#include <stdint.h>

/**
 * @brief Biquad (2nd order IIR) low-pass filter
 *
 * Implements Butterworth low-pass filter for reducing motor-induced
 * high-frequency noise in gyro and accelerometer readings.
 */
typedef struct {
    float b0, b1, b2; /**< Numerator coefficients */
    float a1, a2;     /**< Denominator coefficients (a0 normalized to 1) */
    float x1, x2;     /**< Input history (previous 2 samples) */
    float y1, y2;     /**< Output history (previous 2 samples) */
} Biquad_t;

/**
 * @brief 3D attitude (Euler angles in radians)
 *
 * Represents quadcopter orientation using Tait-Bryan angles.
 * Estimated using complementary filter combining gyro integration
 * and accelerometer-derived angles.
 */
typedef struct {
    float roll;  /**< Roll angle (radians, + right wing down) */
    float pitch; /**< Pitch angle (radians, + nose up) */
    float yaw;   /**< Yaw angle (radians, + clockwise from above) */
} Attitude;

/**
 * @brief 3D floating-point vector
 *
 * Can be accessed via named fields (x, y, z) or array indexing (v[0-2]).
 * Used for gyroscope (rad/s), accelerometer (g), and other 3-axis data.
 */
typedef union {
    struct {
        float x; /**< X component */
        float y; /**< Y component */
        float z; /**< Z component */
    };
    float v[3]; /**< Array access */
} Vec3;

/**
 * @brief 3D integer vector for raw sensor readings
 *
 * Used for unscaled 16-bit sensor data from LSM6DSL.
 */
typedef union {
    struct {
        int x; /**< X component (raw ADC counts) */
        int y; /**< Y component (raw ADC counts) */
        int z; /**< Z component (raw ADC counts) */
    };
    int v[3]; /**< Array access */
} IVec3;

/**
 * @brief 2D floating-point vector
 *
 * Used for horizontal plane data (velocity, high-pass filtered acceleration).
 */
typedef union {
    struct {
        float x; /**< X component (forward) */
        float y; /**< Y component (right) */
    };
    float v[2]; /**< Array access */
} Vec2;

/**
 * @brief 3-axis acceleration (deprecated, use Vec3 instead)
 */
typedef struct {
    float ax; /**< X acceleration */
    float ay; /**< Y acceleration */
    float az; /**< Z acceleration */
} Acceleration;

/**
 * @brief IMU calibration data
 *
 * Stores bias values determined during calibration. Applied as offsets
 * to raw sensor readings to compensate for sensor imperfections.
 */
typedef struct {
    Vec3 gyro_bias;  /**< Gyroscope bias (rad/s) - zero-rate offset */
    Vec3 accel_bias; /**< Accelerometer bias (g) - when Z should read 1g */
    Vec3 mag_offset; /**< Magnetometer hard iron offset (unused) */
    Vec3 mag_scale;  /**< Magnetometer soft iron scale (unused) */
} IMU_Calibration;

/**
 * @brief IMU context structure with sensor data and state
 *
 * Contains raw and processed sensor readings, attitude estimate, calibration data,
 * filters, and velocity estimation state. Single instance used throughout application.
 */
typedef struct {
    // Raw sensor readings (16-bit ADC counts)
    IVec3 gyro_raw; /**< Raw gyroscope data (LSB units) */
    IVec3 acc_raw;  /**< Raw accelerometer data (LSB units) */
    IVec3 mag_raw;  /**< Raw magnetometer data (unused) */

    // Scaled and calibrated sensor readings
    Vec3 gyro; /**< Calibrated gyroscope (rad/s) */
    Vec3 acc;  /**< Calibrated accelerometer (g) */
    Vec3 mag;  /**< Magnetometer (gauss, unused) */

    // Attitude estimation
    Attitude attitude; /**< Current orientation (roll, pitch, yaw in radians) */

    // Calibration and filtering
    IMU_Calibration cal;     /**< Bias calibration data */
    Biquad_t gyro_filter[3]; /**< Low-pass filters for gyro (50 Hz cutoff, X/Y/Z) */
    Biquad_t acc_filter[3];  /**< Low-pass filters for accel (25 Hz cutoff, X/Y/Z) */

    // Velocity estimation (for position hold outer loop)
    Vec2 velocity;   /**< Estimated horizontal velocity (m/s, X=forward, Y=right) */
    Vec2 accel_hp;   /**< High-pass filtered linear acceleration (g, gravity removed) */
    Vec2 accel_prev; /**< Previous linear acceleration for high-pass filter */
} IMU;

/**
 * @brief Verify IMU presence and identity via SPI
 *
 * Reads WHO_AM_I register (0x0F) from LSM6DSL. Expected response: 0x6A.
 *
 * @return true if LSM6DSL detected, false otherwise
 */
bool verifyIMU(void);

/**
 * @brief Read raw 3-axis accelerometer data from LSM6DSL
 *
 * Reads 6 bytes from OUTX_L_XL register (auto-increment) and combines
 * into signed 16-bit values for each axis.
 *
 * @param a Pointer to IVec3 to store raw accelerometer data
 */
void readAccRaw(IVec3 *a);

/**
 * @brief Read raw 3-axis gyroscope data from LSM6DSL
 *
 * Reads 6 bytes from OUTX_L_G register (auto-increment) and combines
 * into signed 16-bit values for each axis.
 *
 * @param g Pointer to IVec3 to store raw gyroscope data
 */
void readGyroRaw(IVec3 *g);

/**
 * @brief Read raw 3-axis magnetometer data (unused)
 *
 * @param m Pointer to IVec3 to store raw magnetometer data
 */
void readMagRaw(IVec3 *m);

/**
 * @brief Convert raw gyroscope readings to rad/s
 *
 * Scales raw 16-bit values to angular velocity in radians per second.
 * LSM6DSL configured for ±500 dps range: 1000 dps / 65536 counts.
 *
 * @param gyro_raw Pointer to raw gyroscope data
 * @param gyro_rad_s Pointer to output vector (rad/s)
 */
void gyroToRadPS(const IVec3 *gyro_raw, Vec3 *gyro_rad_s);

/**
 * @brief Convert raw accelerometer readings to g (gravitational units)
 *
 * Scales raw 16-bit values to acceleration in g.
 * LSM6DSL configured for ±8g range: 16g / 65536 counts.
 *
 * @param acc_raw Pointer to raw accelerometer data
 * @param acc_g Pointer to output vector (g)
 */
void accToG(const IVec3 *acc_raw, Vec3 *acc_g);

/**
 * @brief Convert raw magnetometer readings to gauss (unused)
 *
 * @param mag_raw Pointer to raw magnetometer data
 * @param mag_gauss Pointer to output vector (gauss)
 */
void magToGauss(const IVec3 *mag_raw, Vec3 *mag_gauss);

/**
 * @brief Update attitude estimate using complementary filter
 *
 * Combines gyroscope integration (high-pass) with accelerometer-derived
 * angles (low-pass). Complementary filter coefficient: 0.95 (95% gyro, 5% accel).
 * Yaw is pure gyro integration (no magnetometer correction).
 *
 * @param attitude Pointer to Attitude structure to update
 * @param acc_g Pointer to accelerometer data (g)
 * @param gyro_rad_s Pointer to gyroscope data (rad/s)
 * @param dt Time delta since last update (seconds)
 */
void updateOrientation(Attitude *attitude, const Vec3 *acc_g,
                       const Vec3 *gyro_rad_s, float dt);

/**
 * @brief Initialize IMU sensor and data structures
 *
 * Configures LSM6DSL via SPI:
 * - ODR: 6660 Hz for gyro and accel
 * - Gyro range: ±500 dps with LPF1 enabled
 * - Accel range: ±8g with LPF2 enabled (composite filter for vibration rejection)
 * - Data ready interrupt on INT1 -> PA0
 * - Software biquad filters: 50 Hz gyro, 25 Hz accel
 *
 * @param imu Pointer to IMU structure to initialize
 * @return true if initialization successful, false if WHO_AM_I check fails
 */
bool imu_init(IMU *imu);

/**
 * @brief Update IMU with new sensor readings
 *
 * Reads raw sensor data, applies scaling and calibration, runs low-pass filters,
 * updates attitude estimate, and calculates velocity from high-pass filtered
 * acceleration. Should be called at fixed 6660 Hz rate when imu_data_ready() is true.
 *
 * Processing pipeline:
 * 1. Read raw gyro and accel
 * 2. Convert to physical units (rad/s, g)
 * 3. Apply bias calibration
 * 4. Apply biquad low-pass filters
 * 5. Update attitude (complementary filter)
 * 6. Estimate velocity (high-pass filter + integration)
 *
 * @param imu Pointer to IMU structure
 * @param dt Time delta since last update (seconds, ~0.00015)
 */
void IMU_update(IMU *imu, float dt);

/**
 * @brief Check if new IMU data is ready
 *
 * Reads EXTI0 pending register to check if LSM6DSL data ready interrupt
 * has fired. Clears interrupt flag if set. Use to determine when to call
 * IMU_update() for precise timing.
 *
 * @return true if new data available, false otherwise
 */
bool imu_data_ready();

/**
 * @brief Calibrate gyroscope and accelerometer biases
 *
 * Collects specified number of samples at 6660 Hz and averages to determine
 * sensor biases. IMU must be stationary on level surface. Results stored in
 * imu->cal for automatic application in IMU_update().
 *
 * Gyro calibration: Averages readings to find zero-rate offset.
 * Accel calibration: Averages readings, subtracts 1g from Z axis (expects Z up).
 *
 * @param imu Pointer to IMU structure
 * @param samples Number of samples to collect (e.g., 6660 for 1 second)
 */
void IMU_calibrate(IMU *imu, uint16_t samples);

/**
 * @brief Calibrate magnetometer (unused)
 *
 * @param imu Pointer to IMU structure
 * @param samples Number of samples to collect
 */
void IMU_calibrate_mag(IMU *imu, uint16_t samples);

/**
 * @brief Initialize biquad low-pass filter
 *
 * Calculates Butterworth LPF coefficients (Q=0.707) for specified cutoff
 * frequency and sample rate. Zeros filter history.
 *
 * @param filter Pointer to Biquad_t structure
 * @param cutoff_freq Cutoff frequency (Hz)
 * @param sample_freq Sample rate (Hz, 6660 for IMU)
 */
void biquad_lpf_init(Biquad_t *filter, float cutoff_freq, float sample_freq);

/**
 * @brief Apply biquad filter to input sample
 *
 * Implements Direct Form II Transposed structure. Updates internal history
 * and returns filtered output.
 *
 * @param filter Pointer to Biquad_t structure
 * @param input Input sample
 * @return Filtered output
 */
float biquad_apply(Biquad_t *filter, float input);

/**
 * @brief Write byte to accelerometer register (unused helper)
 *
 * @param reg Register address
 * @param value Byte value to write
 */
void writeAccReg(uint8_t reg, uint8_t value);

/**
 * @brief Write byte to gyroscope register (unused helper)
 *
 * @param reg Register address
 * @param value Byte value to write
 */
void writeGyrReg(uint8_t reg, uint8_t value);

/**
 * @brief Reset velocity estimation state
 *
 * Zeros velocity, high-pass filtered acceleration, and previous acceleration.
 * Call when transitioning to armed state to prevent stale velocity estimates.
 *
 * @param imu Pointer to IMU structure
 */
void IMU_reset_velocity(IMU *imu);

/**
 * @brief BerryIMU version identifier (always 3 for BerryIMUv4/LSM6DSL)
 */
extern int BerryIMUversion;

#endif
