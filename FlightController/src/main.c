#include "LSM6DSL.h"
#include "imu.h"
#include "lora.h"
#include "motor_control.h"
#include "pid.h"
#include "protocol.h"
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
    STATE_DISARMING,      // transition to disarmed
    STATE_FLYING,         // currently flying
    STATE_EMERGENCY_STOP, // stop everthing
    STATE_CALIBRATE,
} State;

void drive_motors(float base_throttle, PID *pid);

int main(void) {
    State state = STATE_INIT;
    IMU imu = {0};
    PID pid = {0};
    float base_throttle = 0.0f; // Base throttle (0.0 to 1.0)

    PIDCreateInfo pid_info = (PIDCreateInfo){
        .roll_Kp = 0.1f,
        .roll_Ki = 0.1f,
        .roll_Kd = 0.1f,
        .roll_Ki_limit = 10.25f,
        .roll_limit = 0.2f, // 20% throttle

        .pitch_Kp = 0.1f,
        .pitch_Ki = 0.0f,
        .pitch_Kd = 0.0f,
        .pitch_Ki_limit = 10.25f,
        .pitch_limit = 0.2f, // 20% throttle

        .yaw_Kp = 0.1f,
        .yaw_Ki = 0.0f,
        .yaw_Kd = 0.0f,
        .yaw_Ki_limit = 10.25f,
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

            // turn on led to signal init success
            led_init();

            state = STATE_DISARMED;
            break;

        case STATE_DISARMED:
            lora_service();

            // Process incoming commands
            if (lora_data_available()) {
                lora_message_t *message = lora_get_received_data();
                CommandType cmd = parse_command(message->data, message->length);

                switch (cmd) {
                case CMD_START:
                    lora_send_string(1, "LOG:CMD_START");
                    state = STATE_ARMING;
                    break;

                case CMD_RESET:
                    lora_send_string(1, "LOG:CMD_RESET");
                    state = STATE_INIT;
                    break;
                case CMD_CALIBRATE:
                    lora_send_string(1, "LOG:CMD_CALIBRATE");
                    state = STATE_CALIBRATE;
                    break;

                default:
                    // Unknown or invalid command in DISARMED state
                    break;
                }

                lora_clear_received_flag();
            }

            break;

        case STATE_CALIBRATE:
            toggle_led();
            IMU_calibrate_accel(&imu, 1);
            IMU_calibrate_gyro(&imu, 1);
            toggle_led();
            state = STATE_DISARMED;
            break;

        case STATE_ARMING:
            lora_service(); // Process any pending LoRa data
            lora_send_string(1, "LOG:ARMING");
            InitMotors();
            StartMotors();

            delay_ms(2500);

            lora_send_string(1, "LOG:ARMED");
            lora_service(); // Ensure message is processed
            state = STATE_FLYING;
            break;

        case STATE_DISARMING:
            lora_send_string(1, "LOG:DISARMING");
            StopMotors();
            state = STATE_DISARMED;
            break;

        case STATE_FLYING:
            lora_service();

            // Check for incoming commands while flying
            if (lora_data_available()) {
                lora_message_t *message = lora_get_received_data();
                CommandType cmd = parse_command(message->data, message->length);

                switch (cmd) {
                case CMD_STOP:
                    lora_send_string(1, "LOG:CMD_STOP");
                    StopMotors();
                    state = STATE_DISARMED;
                    break;

                case CMD_EMERGENCY_STOP:
                    lora_send_string(1, "LOG:CMD_EMERGENCY_STOP");
                    StopMotors();
                    state = STATE_EMERGENCY_STOP;
                    break;

                case CMD_SET_ATTITUDE:
                    // TODO: Parse attitude setpoint from message
                    // CommandSetAttitude *att_cmd = (CommandSetAttitude
                    // *)message->data; pid.setpoints.roll = att_cmd->roll;
                    // pid.setpoints.pitch = att_cmd->pitch;
                    // pid.setpoints.yaw = att_cmd->yaw;
                    break;

                case CMD_SET_THROTTLE: {
                    float throttle_value;
                    if (parse_throttle_value(message->data, message->length,
                                             &throttle_value)) {
                        base_throttle = throttle_value;
                        lora_send_string(1, "LOG:THROTTLE_SET");
                    } else {
                        lora_send_string(1, "LOG:THROTTLE_PARSE_FAIL");
                    }
                    break;
                }

                case CMD_UPDATE_PID:
                    // TODO: Parse PID gains from message
                    break;

                default:
                    break;
                }

                lora_clear_received_flag();
            }

            if (!imu_data_ready()) {
                break;
            }

            // Update IMU measurements and attitude
            IMU_update(&imu, FIXED_DT);

            // stop if greater then max angle
            // if (fabsf(imu.attitude.roll) > MAX_ANGLE ||
            //     fabsf(imu.attitude.pitch) > MAX_ANGLE) {
            //     StopMotors();
            //     state = STATE_EMERGENCY_STOP;
            //     break;
            // }

            // Update PID controllers
            pid_update(&pid, &imu, FIXED_DT);

            drive_motors(base_throttle, &pid);

            // Send string telemetry (required for LoRa AT command format)
            send_telem(&imu, &pid);

            break;

        case STATE_EMERGENCY_STOP:
            lora_service();

            // Check for reset command
            if (lora_data_available()) {
                lora_message_t *message = lora_get_received_data();
                CommandType cmd = parse_command(message->data, message->length);

                if (cmd == CMD_RESET) {
                    lora_send_string(1, "LOG:RESET_FROM_EMERGENCY");
                    state = STATE_DISARMED;
                }

                lora_clear_received_flag();
            }

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
