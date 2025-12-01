#include "LSM6DSL.h"
#include "imu.h"
#include "lora.h"
#include "motor_control.h"
#include "pid.h"
#include "stm32f411xe.h"
#include "systick.h"
#include "uart.h"
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define IMU_ODR_HZ 6660.0f
#define FIXED_DT (1.0f / IMU_ODR_HZ) // ~0.00015015 seconds

#define MAX_ANGLE 0.17f
#define KP 0.0f
#define KI 0.0f
#define KD 0.0f

typedef enum {
  STATE_INIT,           // initial state
  STATE_DISARMED,       // initialized but not running
  STATE_ARMING,         // transitioning to flying
  STATE_FLYING,         // currently flying
  STATE_EMERGENCY_STOP, // stop everthing
} State;

void led_init(void);
void toggle_led(void) { GPIOC->ODR ^= GPIO_ODR_ODR_13; }
void drive_motors(float base_throttle, PID *pid);
void send_telem(IMU *imu, PID *pid);

typedef struct {
  int sign;    // 1 for positive, -1 for negative
  int whole;   // whole part (always positive)
  int decimal; // decimal part (always positive)
} float_parts_t;

void split_float(float value, int decimal_places, float_parts_t *result) {
  // Determine sign
  result->sign = (value < 0.0f) ? -1 : 1;

  // Work with absolute value
  float abs_value = (value < 0.0f) ? -value : value;

  // Extract whole part
  result->whole = (int)abs_value;

  // Extract decimal part
  float fractional = abs_value - (float)result->whole;

  // Scale up by 10^decimal_places and round
  int multiplier = 1;
  for (int i = 0; i < decimal_places; i++) {
    multiplier *= 10;
  }
  result->decimal = (int)(fractional * multiplier + 0.5f);

  // Handle rounding edge case (e.g., 1.999 with 2 decimals -> 2.00)
  if (result->decimal >= multiplier) {
    result->whole++;
    result->decimal -= multiplier;
  }
}

int main(void) {
  State state = STATE_INIT;
  IMU imu = {0};
  PID pid = {0};

  PIDCreateInfo pid_info = (PIDCreateInfo){
      .roll_Kp = 0.0f,
      .roll_Ki = 0.0f,
      .roll_Kd = 0.0f,
      .roll_limit = 0.2f, // 20% throttle

      .pitch_Kp = 0.0f,
      .pitch_Ki = 0.0f,
      .pitch_Kd = 0.0f,
      .pitch_limit = 0.2f, // 20% throttle

      .yaw_Kp = 0.0f,
      .yaw_Ki = 0.0f,
      .yaw_Kd = 0.0f,
      .yaw_limit = 0.1f, // 10% throttle

  };

  while (1) {
    switch (state) {
    case STATE_INIT:

      // microcontroller config
      SystemClock_Config_100MHz_HSE();
      systick_init();
      uart_init();
      // Initialize LoRa module
      if (lora_init() == LORA_OK) {
        lora_send_string(1, "LOG:INIT_OK");
      } else {
        lora_send_string(1, "LOG:INIT_FAIL");
      }

      if (!imu_init(&imu)) {
        while (1)
          ;
      }

      // pid_init(&pid, pid_info);
      // InitMotors();

      // Set initial target attitude
      pid.setpoints.roll = 0.0f;
      pid.setpoints.pitch = 0.0f;
      pid.setpoints.yaw = 0.0f;
      // lora_send_string(1, "LOG:DISARMED");

      // turn on led to signal init success
      led_init();

      state = STATE_DISARMED;
      break;

    case STATE_DISARMED:
      lora_service();

      // idle until start condition
      if (lora_data_available()) {
        lora_message_t *message = lora_get_received_data();
        if (message->data && strncmp((char *)message->data, "start", 5) == 0) {
          state = STATE_ARMING;
        }
      }

      break;
    case STATE_ARMING:
      lora_send_string(1, "LOG:ARMING");
      StartMotors();
      toggle_led();

      delay_ms(1000);

      drive_motors(0.1, &pid);
      lora_send_string(1, "LOG:ARMED");
      state = STATE_FLYING;
      break;

    case STATE_FLYING:
      lora_service();

      if (!imu_data_ready()) {
        break;
      }

      // Update IMU measurements and attitude
      IMU_update(&imu, FIXED_DT);

      // stop if greater then max angle
      if (fabsf(imu.attitude.roll) > MAX_ANGLE ||
          fabsf(imu.attitude.pitch) > MAX_ANGLE) {
        StopMotors();
        state = STATE_EMERGENCY_STOP;
        break;
      }

      send_telem(&imu, &pid);

      // Update PID controllers
      pid_update(&pid, &imu, FIXED_DT);

      drive_motors(1.0, &pid);

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

void drive_motors(float base_throttle, PID *pid) {

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

void send_telem(IMU *imu, PID *pid) {
  static int samples = 0;
  static char telem_buffer[200];

  if (lora_is_ready() && samples > 1000) {
    float_parts_t roll, pitch, yaw;
    float_parts_t roll_p, roll_i, roll_d;
    float_parts_t pitch_p, pitch_i, pitch_d;
    float_parts_t yaw_p, yaw_i, yaw_d;
    float_parts_t alt, voltage;

    // Split all the floats (3 decimal places for angles and PID terms)
    split_float(imu->attitude.roll, 3, &roll);
    split_float(imu->attitude.pitch, 3, &pitch);
    split_float(imu->attitude.yaw, 3, &yaw);

    split_float(pid->roll_pid.p_term, 3, &roll_p);
    split_float(pid->roll_pid.i_term, 3, &roll_i);
    split_float(pid->roll_pid.d_term, 3, &roll_d);

    split_float(pid->pitch_pid.p_term, 3, &pitch_p);
    split_float(pid->pitch_pid.i_term, 3, &pitch_i);
    split_float(pid->pitch_pid.d_term, 3, &pitch_d);

    split_float(pid->yaw_pid.p_term, 3, &yaw_p);
    split_float(pid->yaw_pid.i_term, 3, &yaw_i);
    split_float(pid->yaw_pid.d_term, 3, &yaw_d);

    // Altitude with 1 decimal place, voltage with 2
    split_float(0.0f, 1, &alt);
    split_float(0.0f, 2, &voltage);

    sprintf(telem_buffer,
            "LOG:%s%d.%03d:%s%d.%03d:%s%d.%03d:%s%d.%03d:%s%d.%03d:%s%d.%03d:"
            "%s%d.%03d:%s%d.%03d:%s%d.%03d:%s%d.%03d:%s%d.%03d:%s%d.%03d:"
            "%s%d.%01d:%s%d.%02d",
            // Roll, pitch, yaw
            roll.sign < 0 ? "-" : "", roll.whole, roll.decimal,
            pitch.sign < 0 ? "-" : "", pitch.whole, pitch.decimal,
            yaw.sign < 0 ? "-" : "", yaw.whole, yaw.decimal,
            // Roll PID terms
            roll_p.sign < 0 ? "-" : "", roll_p.whole, roll_p.decimal,
            roll_i.sign < 0 ? "-" : "", roll_i.whole, roll_i.decimal,
            roll_d.sign < 0 ? "-" : "", roll_d.whole, roll_d.decimal,
            // Pitch PID terms
            pitch_p.sign < 0 ? "-" : "", pitch_p.whole, pitch_p.decimal,
            pitch_i.sign < 0 ? "-" : "", pitch_i.whole, pitch_i.decimal,
            pitch_d.sign < 0 ? "-" : "", pitch_d.whole, pitch_d.decimal,
            // Yaw PID terms
            yaw_p.sign < 0 ? "-" : "", yaw_p.whole, yaw_p.decimal,
            yaw_i.sign < 0 ? "-" : "", yaw_i.whole, yaw_i.decimal,
            yaw_d.sign < 0 ? "-" : "", yaw_d.whole, yaw_d.decimal,
            // Altitude (1 decimal) and voltage (2 decimals)
            alt.sign < 0 ? "-" : "", alt.whole, alt.decimal,
            voltage.sign < 0 ? "-" : "", voltage.whole, voltage.decimal);

    lora_send_string_nb(1, telem_buffer);
    samples = 0;
  }
  samples++;
}
