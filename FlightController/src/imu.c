#include "imu.h"
#include "LIS3MDL.h"
#include "LSM6DSL.h"
#include "spi.h"
#include <math.h>
#include <stdint.h>

int BerryIMUversion = 4; // Always v4

#define GYRO_SENSITIVITY_500DPS 17.50f

typedef struct {
  float roll;
  float pitch;
  float yaw;
} orientation;

void readAccRaw(int a[]) {
  uint8_t block[6];

  // BerryIMU-4: Use SPI for LSM6DSL accelerometer
  SPI_ReadBlock(LSM6DSL_OUTX_L_XL, sizeof(block), block);

  // Combine readings for each axis
  a[0] = (int16_t)(block[0] | block[1] << 8);
  a[1] = (int16_t)(block[2] | block[3] << 8);
  a[2] = (int16_t)(block[4] | block[5] << 8);
}

void readGyroRaw(int g[]) {
  uint8_t block[6];

  // BerryIMU-4: Use SPI for LSM6DSL gyroscope
  SPI_ReadBlock(LSM6DSL_OUTX_L_G, sizeof(block), block);

  // Combine readings for each axis
  g[0] = (int16_t)(block[0] | block[1] << 8);
  g[1] = (int16_t)(block[2] | block[3] << 8);
  g[2] = (int16_t)(block[4] | block[5] << 8);
}

void gyroToRadPS(int gyro_raw[], float gyro_rad_s[]) {
  for (int i = 0; i < 3; i++) {
    // ±500°/s range over 16-bit signed (-32768 to +32767)
    // Total range: 1000°/s, Total values: 65536
    // Scale: 1000°/s / 65536 counts = degrees per second
    // Convert to radians: * π/180
    gyro_rad_s[i] =
        gyro_raw[i] * (1000.0f / 65536.0f) * (3.14159265359f / 180.0f);
  }
}

void accToG(int acc_raw[], float acc_g[]) {
  for (int i = 0; i < 3; i++) {
    // g range is 16g and raw range is 65536
    acc_g[i] = (acc_raw[i] * 16.0f) / 65536.0f;
  }
}

uint8_t verifyIMU(void) {
  // Detect BerryIMUv4 (LSM6DSL via SPI + LIS3MDL via I2C)
  uint8_t LSM6DSL_WHO_response =
      SPI_ReadByte(LSM6DSL_WHO_AM_I); // Should be 0x6A

  if (LSM6DSL_WHO_response == 0x6A) {
    return 1;
  } else {
    return 0;
  }
}

void updateOrientation(orientation *orient, float acc_g[], float gyro_rad_s[],
                       float dt) {
  float acc_x_sq = acc_g[0] * acc_g[0];
  float acc_y_sq = acc_g[1] * acc_g[1];
  float acc_z_sq = acc_g[2] * acc_g[2];

  float pitch_from_acc = atanf(-acc_g[0] / sqrtf(acc_y_sq + acc_z_sq));
  float roll_from_acc = atanf(acc_g[1] / sqrtf(acc_x_sq + acc_z_sq));

  float alpha = 0.95f;

  orient->pitch = alpha * (orient->pitch + gyro_rad_s[1] * dt) +
                  (1.0f - alpha) * pitch_from_acc;
  orient->roll = alpha * (orient->roll + gyro_rad_s[0] * dt) +
                 (1.0f - alpha) * roll_from_acc;
  orient->yaw += gyro_rad_s[2] * dt;
}

void enableIMU(void) {
  // Enable gyroscope (SPI) - ODR 6.66 kHz, 2000 dps
  SPI_WriteByte(LSM6DSL_CTRL2_G, 0b10100100);

  // Enable accelerometer (SPI) - ODR 6.66 kHz, +/- 8g
  SPI_WriteByte(LSM6DSL_CTRL1_XL, 0b10101111);
  SPI_WriteByte(LSM6DSL_CTRL8_XL, 0b11001000); // Low pass filter enabled
  SPI_WriteByte(LSM6DSL_CTRL3_C, 0b01000100);  // Block Data update enabled
}
