#include "imu.h"
#include "LIS3MDL.h"
#include "LSM6DSL.h"
#include "spi.h"
#include "stm32f411xe.h"
#include "systick.h"
#include <math.h>
#include <stdbool.h>
#include <stdint.h>

int BerryIMUversion = 3;

#define GYRO_SENSITIVITY_500DPS 17.50f

#define LSM6DSL_PULSE_CFG_G 0x0B

void initGyroInterrupt(void) {
    // Enable GPIOA and SYSCFG clocks
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;

    // Configure PA0 as input with pull-down (since we want to detect HIGH)
    GPIOA->MODER &= ~(3UL << (0 * 2)); // Clear mode bits for PA0 (input mode)
    GPIOA->PUPDR &= ~(3UL << (0 * 2)); // Clear pull-up/down bits
    GPIOA->PUPDR |= (2UL << (0 * 2));  // Set pull-down (changed to pull-down)

    // Connect PA0 to EXTI0
    SYSCFG->EXTICR[0] &= ~SYSCFG_EXTICR1_EXTI0;   // Clear EXTI0 config
    SYSCFG->EXTICR[0] |= SYSCFG_EXTICR1_EXTI0_PA; // Connect PA0 to EXTI0

    // Configure EXTI0 for rising edge (HIGH)
    EXTI->IMR |= EXTI_IMR_MR0;    // Enable interrupt on line 0
    EXTI->RTSR |= EXTI_RTSR_TR0;  // Enable rising edge trigger
    EXTI->FTSR &= ~EXTI_FTSR_TR0; // Disable falling edge trigger
}

void readAccRaw(IVec3 *a) {
    uint8_t block[6];

    // BerryIMU-4: Use SPI for LSM6DSL accelerometer
    SPI_ReadBlock(LSM6DSL_OUTX_L_XL, sizeof(block), block);

    // Combine readings for each axis
    a->x = (int16_t)(block[0] | block[1] << 8);
    a->y = (int16_t)(block[2] | block[3] << 8);
    a->z = (int16_t)(block[4] | block[5] << 8);
}

void readGyroRaw(IVec3 *g) {
    uint8_t block[6];

    // BerryIMU-4: Use SPI for LSM6DSL gyroscope
    SPI_ReadBlock(LSM6DSL_OUTX_L_G, sizeof(block), block);

    // Combine readings for each axis
    g->x = (int16_t)(block[0] | block[1] << 8);
    g->y = (int16_t)(block[2] | block[3] << 8);
    g->z = (int16_t)(block[4] | block[5] << 8);
}

void gyroToRadPS(const IVec3 *gyro_raw, Vec3 *gyro_rad_s) {
    for (int i = 0; i < 3; i++) {
        // ±500°/s range over 16-bit signed (-32768 to +32767)
        // Total range: 1000°/s, Total values: 65536
        // Scale: 1000°/s / 65536 counts = degrees per second
        // Convert to radians: * π/180
        gyro_rad_s->v[i] =
            gyro_raw->v[i] * (1000.0f / 65536.0f) * (3.14159265359f / 180.0f);
    }
}

void accToG(const IVec3 *acc_raw, Vec3 *acc_g) {
    for (int i = 0; i < 3; i++) {
        // g range is 16g and raw range is 65536
        acc_g->v[i] = (acc_raw->v[i] * 16.0f) / 65536.0f;
    }
}

bool verifyIMU(void) {
    // Detect BerryIMUv4 (LSM6DSL via SPI + LIS3MDL via I2C)
    uint8_t LSM6DSL_WHO_response =
        SPI_ReadByte(LSM6DSL_WHO_AM_I); // Should be 0x6A

    if (LSM6DSL_WHO_response == 0x6A) {
        return true;
    } else {
        return false;
    }
}

void updateOrientation(Attitude *orient, const Vec3 *acc_g,
                       const Vec3 *gyro_rad_s, float dt) {
    // Roll: rotation around X axis (uses Y and Z accelerations)
    float roll_from_acc = atan2f(acc_g->y, acc_g->z);

    // Pitch: rotation around Y axis (uses X and Z accelerations)
    float pitch_from_acc =
        atan2f(-acc_g->x, sqrtf(acc_g->y * acc_g->y + acc_g->z * acc_g->z));

    float alpha = 0.95f;

    // Gyro integration with complementary filter
    orient->roll = alpha * (orient->roll + gyro_rad_s->x * dt) +
                   (1.0f - alpha) * roll_from_acc;

    orient->pitch = alpha * (orient->pitch + gyro_rad_s->y * dt) +
                    (1.0f - alpha) * pitch_from_acc;

    orient->yaw += gyro_rad_s->z * dt;
}

bool imu_init(IMU *imu) {
    *imu = (IMU){0};

    initGyroInterrupt();
    SPI1_Init();
    delay_ms(100); // SPI/power stabilization

    if (!verifyIMU()) {
        return false;
    }

    // Software reset
    SPI_WriteByte(LSM6DSL_CTRL3_C, 0x01);
    delay_ms(50); // Wait for reset to complete

    // Configure gyroscope: ODR 6.66kHz, 2000 dps
    SPI_WriteByte(LSM6DSL_CTRL2_G, 0b10100100);

    // Configure accelerometer: ODR 6.66kHz, ±8g
    SPI_WriteByte(LSM6DSL_CTRL1_XL, 0b10101111);

    // Block Data Update (BDU) enabled, IF_INC enabled
    SPI_WriteByte(LSM6DSL_CTRL3_C, 0b01000100);

    // Configure gyro low-pass filter bandwidth
    // Bit 0: LPF1_SEL_G = 1 (enable LPF1)
    // This enables the gyro low-pass filter for noise reduction
    SPI_WriteByte(LSM6DSL_CTRL6_C, 0b00000001);

    // Configure gyro for high-performance mode with LPF1 enabled
    // Bit 7: G_HM_MODE = 0 (high-performance mode)
    // Bit 6: HP_G_EN = 0 (high-pass filter disabled, we want low-pass)
    // Bits 5-0: Other settings = 0
    SPI_WriteByte(LSM6DSL_CTRL7_G, 0b00000000);

    // Low-pass filter for accelerometer (composite filter)
    // Bit 7: LPF2_XL_EN = 1 (enable LPF2)
    // Bit 6: HPCF_XL[1] = 1
    // Bit 3: HP_SLOPE_XL_EN = 1 (enable slope filter)
    // Bits 2-0: Input composite = 000
    // This creates a stronger low-pass filter to reduce motor vibrations
    SPI_WriteByte(LSM6DSL_CTRL8_XL, 0b11001000);

    // Gyro data ready interrupt on INT1
    SPI_WriteByte(LSM6DSL_INT1_CTRL, 0b00000010);

    // Pulse mode interrupt (auto-clear)
    // SPI_WriteByte(LSM6DSL_PULSE_CFG_G, 0x80);

    // Initialize software low-pass filters
    // Sample rate is 6.66 kHz (6660 Hz)
    float sample_rate = 6660.0f;

    // Gyro filters: 100 Hz cutoff (good balance between noise rejection and
    // responsiveness)
    for (int i = 0; i < 3; i++) {
        biquad_lpf_init(&imu->gyro_filter[i], 50.0f, sample_rate);
    }

    // Accel filters: 50 Hz cutoff (more aggressive filtering for noisier
    // accelerometer)
    for (int i = 0; i < 3; i++) {
        biquad_lpf_init(&imu->acc_filter[i], 25.0f, sample_rate);
    }

    return true;
}

void IMU_update(IMU *imu, float dt) {
    readGyroRaw(&imu->gyro_raw);
    readAccRaw(&imu->acc_raw);

    gyroToRadPS(&imu->gyro_raw, &imu->gyro);
    accToG(&imu->acc_raw, &imu->acc);

    // Apply calibration (if not calibrated, bias is 0)
    imu->gyro.x -= imu->cal.gyro_bias.x;
    imu->gyro.y -= imu->cal.gyro_bias.y;
    imu->gyro.z -= imu->cal.gyro_bias.z;

    imu->acc.x -= imu->cal.accel_bias.x;
    imu->acc.y -= imu->cal.accel_bias.y;
    imu->acc.z -= imu->cal.accel_bias.z;

    // Apply software low-pass filters to reduce motor noise
    imu->gyro.x = biquad_apply(&imu->gyro_filter[0], imu->gyro.x);
    imu->gyro.y = biquad_apply(&imu->gyro_filter[1], imu->gyro.y);
    imu->gyro.z = biquad_apply(&imu->gyro_filter[2], imu->gyro.z);

    imu->acc.x = biquad_apply(&imu->acc_filter[0], imu->acc.x);
    imu->acc.y = biquad_apply(&imu->acc_filter[1], imu->acc.y);
    imu->acc.z = biquad_apply(&imu->acc_filter[2], imu->acc.z);

    updateOrientation(&imu->attitude, &imu->acc, &imu->gyro, dt);

    // Velocity estimation (dont ask me how this works)

    float sin_pitch = sinf(imu->attitude.pitch);
    float cos_pitch = cosf(imu->attitude.pitch);
    float sin_roll = sinf(imu->attitude.roll);
    // float cos_roll = cosf(imu->attitude.roll);

    float grav_x = -sin_pitch;
    float grav_y = -sin_roll * cos_pitch;

    float lin_ax = imu->acc.x - grav_x;
    float lin_ay = imu->acc.y - grav_y;

    // high pass filter
    float tau = 0.5f;
    float alpha_hp = tau / (tau + dt);

    imu->accel_hp.x = alpha_hp * (imu->accel_hp.x + lin_ax - imu->accel_prev.x);
    imu->accel_hp.y = alpha_hp * (imu->accel_hp.y + lin_ay - imu->accel_prev.y);
    imu->accel_prev.x = lin_ax;
    imu->accel_prev.y = lin_ay;

    // integrate acceleration to get velocity in m/s
    imu->velocity.x += imu->accel_hp.x * 9.81f * dt;
    imu->velocity.y += imu->accel_hp.y * 9.81f * dt;
}

bool imu_data_ready() {
    if (EXTI->PR & EXTI_PR_PR0) {
        EXTI->PR |= EXTI_PR_PR0; // clear interrupt bit
        return true;
    }

    return false;
}

void IMU_calibrate_gyro(IMU *imu, uint16_t samples) {
    // Calibrate gyroscope by averaging samples while stationary
    // This finds the bias (offset) that should be subtracted from readings

    Vec3 gyro_sum;

    // Collect samples
    for (uint16_t i = 0; i < samples; i++) {
        // Wait for data ready
        while (!imu_data_ready()) {
            // Wait for next sample
        }

        // Read raw gyro data
        readGyroRaw(&imu->gyro_raw);

        // Convert to rad/s and accumulate
        Vec3 gyro_temp;
        gyroToRadPS(&imu->gyro_raw, &gyro_temp);

        gyro_sum.x += gyro_temp.x;
        gyro_sum.y += gyro_temp.y;
        gyro_sum.z += gyro_temp.z;
    }

    // Calculate average bias
    imu->cal.gyro_bias.x = gyro_sum.x / samples;
    imu->cal.gyro_bias.y = gyro_sum.y / samples;
    imu->cal.gyro_bias.z = gyro_sum.z / samples;
}

void IMU_calibrate_accel(IMU *imu, uint16_t samples) {
    // Calibrate accelerometer by averaging samples while stationary
    // Assumes device is level with Z-axis pointing up (should read [0, 0, 1g])

    Vec3 accel_sum;

    // Collect samples
    for (uint16_t i = 0; i < samples; i++) {
        // Wait for data ready
        while (!imu_data_ready()) {
            // Wait for next sample
        }

        // Read gyro data first to clear sensor's internal data ready flag
        readGyroRaw(&imu->gyro_raw);

        // Read raw accel data
        readAccRaw(&imu->acc_raw);

        // Convert to g and accumulate
        Vec3 acc_temp;
        accToG(&imu->acc_raw, &acc_temp);

        accel_sum.x += acc_temp.x;
        accel_sum.y += acc_temp.y;
        accel_sum.z += acc_temp.z;
    }

    // Calculate average
    Vec3 accel_avg;
    accel_avg.x = accel_sum.x / samples;
    accel_avg.y = accel_sum.y / samples;
    accel_avg.z = accel_sum.z / samples;

    // Bias is the difference from expected [0, 0, 1g]
    imu->cal.accel_bias.x = accel_avg.x - 0.0f;
    imu->cal.accel_bias.y = accel_avg.y - 0.0f;
    imu->cal.accel_bias.z = accel_avg.z - 1.0f;
}

void biquad_lpf_init(Biquad_t *filter, float cutoff_freq, float sample_freq) {
    // Butterworth low-pass filter (Q = 0.707 for flat passband)
    float omega = 2.0f * 3.14159265359f * cutoff_freq / sample_freq;
    float sn = sinf(omega);
    float cs = cosf(omega);
    float alpha = sn / (2.0f * 0.707f); // Q = 0.707 for Butterworth response

    // Biquad coefficients
    float b0 = (1.0f - cs) / 2.0f;
    float b1 = 1.0f - cs;
    float b2 = (1.0f - cs) / 2.0f;
    float a0 = 1.0f + alpha;
    float a1 = -2.0f * cs;
    float a2 = 1.0f - alpha;

    // Normalize by a0
    filter->b0 = b0 / a0;
    filter->b1 = b1 / a0;
    filter->b2 = b2 / a0;
    filter->a1 = a1 / a0;
    filter->a2 = a2 / a0;

    // Initialize history to zero
    filter->x1 = filter->x2 = filter->y1 = filter->y2 = 0.0f;
}

float biquad_apply(Biquad_t *filter, float input) {
    // Direct Form II Transposed implementation
    float output = filter->b0 * input + filter->b1 * filter->x1 +
                   filter->b2 * filter->x2 - filter->a1 * filter->y1 -
                   filter->a2 * filter->y2;

    // Update history
    filter->x2 = filter->x1;
    filter->x1 = input;
    filter->y2 = filter->y1;
    filter->y1 = output;

    return output;
}

void IMU_reset_velocity(IMU *imu) {
    imu->velocity.x = 0.0f;
    imu->velocity.y = 0.0f;
    imu->accel_hp.x = 0.0f;
    imu->accel_hp.y = 0.0f;
    imu->accel_prev.x = 0.0f;
    imu->accel_prev.y = 0.0f;
}
