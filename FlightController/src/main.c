#include "LSM6DSL.h"
#include "imu.h"
#include "motor_control.h"
#include "pid.h"
#include "stm32f411xe.h"
#include "systick.h"
#include "uart.h"
#include <math.h>
#include <stdint.h>

#define IMU_ODR_HZ 6660.0f
#define FIXED_DT (1.0f / IMU_ODR_HZ) // ~0.00015015 seconds

#define MAX_ANGLE 0.17f
#define KP 1.0f
#define KI 0.0f
#define KD 0.5f

typedef enum {
  STATE_INIT,           // initial state
  STATE_DISARMED,       // initialized but not running
  STATE_ARMING,         // transitioning to flying
  STATE_FLYING,         // currently flying
  STATE_EMERGENCY_STOP, // stop everthing
} State;

void led_init(void);
void toggle_led(void) { GPIOC->ODR ^= GPIO_ODR_ODR_13; }
void drive_motors(float base_throttle, PIDContext *pid);

int main(void) {
  State state = STATE_INIT;
  IMUContext imu = {0};
  PIDContext pid = {0};

  for (;;) {
    switch (state) {
    case STATE_INIT:
      uart_send_string("Initializing...\r\n");

      // microcontroller config
      SystemClock_Config_100MHz_HSE();
      systick_init();
      led_init();
      enableIMU();
      uart_init();

      // flight controller init
      imu_init(&imu);
      pid_init(&pid, KP, KI, KD);
      InitMotors();

      // Set initial target attitude
      pid.setpoints.roll = 0.0f;
      pid.setpoints.pitch = 0.0f;
      pid.setpoints.yaw = 0.0f;

      state = STATE_DISARMED;
      break;

    case STATE_DISARMED:
      // idle until start condition currently nothing

      state = STATE_ARMING;
      break;
    case STATE_ARMING:
      uart_send_string("Arming...\r\n");
      StartMotors();

      state = STATE_FLYING;
      break;

    case STATE_FLYING:
      if (!imu_data_ready()) {
        break;
      }

      // Update IMU measurements and attitude
      IMU_update_context(&imu, FIXED_DT);

      // stop if greater then max angle
      if (fabsf(imu.attitude.roll) > MAX_ANGLE ||
          fabsf(imu.attitude.pitch) > MAX_ANGLE) {
        StopMotors();
        state = STATE_EMERGENCY_STOP;
        break;
      }

      // Update PID controllers
      pid_update(&pid, &imu, FIXED_DT);

      // drive motors
      drive_motors(0.1, &pid);

      break;

    case STATE_EMERGENCY_STOP:
      // constant stop motors
      StopMotors();

      break;
    }
  }

  return 0;
}

// idk where to put this function
void led_init(void) {
  // Enable GPIOC clock (onboard LED is usually on PC13)
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
  __DSB(); // Data Synchronization Barrier

  // Configure PC13 as general purpose output
  GPIOC->MODER &= ~(GPIO_MODER_MODER13);  // Clear bits
  GPIOC->MODER |= (GPIO_MODER_MODER13_0); // Set to 01 (output)

  // Configure as push-pull (default)
  GPIOC->OTYPER &= ~(GPIO_OTYPER_OT_13);

  // No pull-up/pull-down
  GPIOC->PUPDR &= ~(GPIO_PUPDR_PUPDR13);

  // Set output speed to low (optional)
  GPIOC->OSPEEDR &= ~(GPIO_OSPEEDER_OSPEEDR13);
}

void drive_motors(float base_throttle, PIDContext *pid) {

  int motor1_speed = base_throttle + pid->output.roll + pid->output.pitch +
                     pid->output.yaw; // north east
  int motor2_speed = base_throttle - pid->output.roll + pid->output.pitch -
                     pid->output.yaw; // north west
  int motor3_speed = base_throttle + pid->output.roll - pid->output.pitch +
                     pid->output.yaw; // south east
  int motor4_speed = base_throttle - pid->output.roll - pid->output.pitch -
                     pid->output.yaw; // south west

  // Clamp to 0-1 range
  motor1_speed = fmaxf(0.0f, fminf(1.0f, motor1_speed));
  motor2_speed = fmaxf(0.0f, fminf(1.0f, motor2_speed));
  motor3_speed = fmaxf(0.0f, fminf(1.0f, motor3_speed));
  motor4_speed = fmaxf(0.0f, fminf(1.0f, motor4_speed));

  // 1-2000 range
  SetMotorThrottle(motor1, (int16_t)(motor1_speed * 1999.0 + 1.0));
  SetMotorThrottle(motor2, (int16_t)(motor2_speed * 1999.0 + 1.0));
  SetMotorThrottle(motor3, (int16_t)(motor3_speed * 1999.0 + 1.0));
  SetMotorThrottle(motor4, (int16_t)(motor4_speed * 1999.0 + 1.0));
}
