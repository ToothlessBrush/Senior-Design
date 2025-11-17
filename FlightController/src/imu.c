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

void updateOrientation(Attitude *orient, float acc_g[], float gyro_rad_s[],
                       float dt) {
  // Roll: rotation around X axis (uses Y and Z accelerations)
  float roll_from_acc = atan2f(acc_g[1], acc_g[2]);

  // Pitch: rotation around Y axis (uses X and Z accelerations)
  float pitch_from_acc =
      atan2f(-acc_g[0], sqrtf(acc_g[1] * acc_g[1] + acc_g[2] * acc_g[2]));

  float alpha = 0.95f;

  // Gyro integration with complementary filter
  orient->roll = alpha * (orient->roll + gyro_rad_s[0] * dt) +
                 (1.0f - alpha) * roll_from_acc;

  orient->pitch = alpha * (orient->pitch + gyro_rad_s[1] * dt) +
                  (1.0f - alpha) * pitch_from_acc;

  orient->yaw += gyro_rad_s[2] * dt;
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

  // Low-pass filter for accelerometer
  SPI_WriteByte(LSM6DSL_CTRL8_XL, 0b11001000);

  // Block Data Update (BDU) enabled, IF_INC enabled
  SPI_WriteByte(LSM6DSL_CTRL3_C, 0b01000100);

  // Gyro data ready interrupt on INT1
  SPI_WriteByte(LSM6DSL_INT1_CTRL, 0b00000010);

  // Pulse mode interrupt (auto-clear)
  // SPI_WriteByte(LSM6DSL_PULSE_CFG_G, 0x80);

  return true;
}

void IMU_update(IMU *imu, float dt) {
  readGyroRaw(imu->gyro_raw);
  readAccRaw(imu->acc_raw);

  gyroToRadPS(imu->gyro_raw, imu->gyro);
  accToG(imu->acc_raw, imu->acc);

  updateOrientation(&imu->attitude, imu->acc, imu->gyro, dt);
}

bool imu_data_ready() {
  if (EXTI->PR & EXTI_PR_PR0) {
    EXTI->PR |= EXTI_PR_PR0; // clear interrupt bit
    return true;
  }

  return false;
}
