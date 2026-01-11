#include "LSM6DSL.h"
#include "imu.h"
#include "lora.h"
#include "motor_control.h"
#include "pid.h"
#include "system.h"
#include "systick.h"
#include "uart.h"
#include "utils.h"
#include <math.h>
#include <stdint.h>
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

void drive_motors(float base_throttle, PID *pid);

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

            pid_init(&pid, pid_info);
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
                if (strncmp((char *)message->data, "FC:START", 5) == 0) {
                    state = STATE_ARMING;
                }
            }

            break;
        case STATE_ARMING:
            lora_send_string(1, "LOG:ARMING");
            InitMotors();
            StartMotors();

            delay_ms(1000);

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

            // Update PID controllers
            pid_update(&pid, &imu, FIXED_DT);

            drive_motors(1.0, &pid);

            send_telem(&imu, &pid);

            break;

        case STATE_EMERGENCY_STOP:
            // constant stop motors
            StopMotors();

            break;
        }
    }

    return 0;
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
    SetMotorThrottle(motor1, (int16_t)(motor1_speed * 1000.0 + 1.0));
    SetMotorThrottle(motor2, (int16_t)(motor2_speed * 1000.0 + 1.0));
    SetMotorThrottle(motor3, (int16_t)(motor3_speed * 1000.0 + 1.0));
    SetMotorThrottle(motor4, (int16_t)(motor4_speed * 1000.0 + 1.0));
}
