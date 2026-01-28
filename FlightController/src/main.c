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

#define IMU_ODR_HZ 6660.0f
#define FIXED_DT (1.0f / IMU_ODR_HZ) // ~0.00015015 seconds

#define MAX_ANGLE 0.17f
#define KP 0.0f
#define KI 0.0f
#define KD 0.0f

#define MIN_THROTTLE 0.05f

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

static State handle_disarmed_command(ParsedCommand cmd) {
    switch (cmd.type) {
    case CMD_START:
        lora_send_string(1, "LOG:CMD_START");
        return STATE_ARMING;

    case CMD_RESET:
        lora_send_string(1, "LOG:CMD_RESET");
        return STATE_INIT;

    case CMD_CALIBRATE:
        lora_send_string(1, "LOG:CMD_CALIBRATE");
        return STATE_CALIBRATE;

    default:
        return STATE_DISARMED;
    }
}

static State handle_flying_command(ParsedCommand cmd, lora_message_t *message,
                                   float *base_throttle, PID *pid) {
    switch (cmd.type) {
    case CMD_STOP:
        lora_send_string_nb(1, "LOG:CMD_STOP");
        StopMotors();
        return STATE_DISARMED;

    case CMD_EMERGENCY_STOP:
        lora_send_string_nb(1, "LOG:CMD_EMERGENCY_STOP");
        StopMotors();
        return STATE_EMERGENCY_STOP;

    case CMD_SET_POINT:
        lora_send_string_nb(1, "LOG:SET_POINT_SET");
        pid->setpoints.pitch = cmd.payload.setpoint.pitch;
        pid->setpoints.roll = cmd.payload.setpoint.roll;
        pid->setpoints.yaw = cmd.payload.setpoint.yaw;
        return STATE_FLYING;

    case CMD_SET_THROTTLE:
        lora_send_string_nb(1, "LOG:THROTTLE_SET");
        *base_throttle = cmd.payload.throttle.value;
        return STATE_FLYING;

    case CMD_HEART_BEAT:
        *base_throttle = cmd.payload.heartbeat.base_throttle;

        pid->setpoints.pitch = cmd.payload.heartbeat.pitch;
        pid->setpoints.roll = cmd.payload.heartbeat.roll;
        pid->setpoints.yaw = cmd.payload.heartbeat.yaw;
        return STATE_FLYING;

    case CMD_UPDATE_PID:
        // TODO: Parse PID gains from message
        return STATE_FLYING;

    default:
        return STATE_FLYING;
    }
}

static State handle_emergency_stop_command(ParsedCommand cmd) {
    if (cmd.type == CMD_RESET) {
        lora_send_string(1, "LOG:RESET_FROM_EMERGENCY");
        return STATE_DISARMED;
    }
    return STATE_EMERGENCY_STOP;
}

int main(void) {
    State state = STATE_INIT;
    IMU imu = {0};
    PID pid = {0};
    float base_throttle = 0.10f; // Base throttle (0.0 to 1.0)

    PIDCreateInfo pid_info = (PIDCreateInfo){
        .roll_Kp = 1.0f,
        .roll_Ki = 0.0f,
        .roll_Kd = 0.0f,
        .roll_Ki_limit = 10.25f,
        .roll_limit = 0.2f, // 20% throttle

        .pitch_Kp = 1.0f,
        .pitch_Ki = 0.0f,
        .pitch_Kd = 0.0f,
        .pitch_Ki_limit = 10.25f,
        .pitch_limit = 0.2f, // 20% throttle

        .yaw_Kp = 0.0f,
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

            if (lora_data_available()) {
                lora_message_t *message = lora_get_received_data();
                ParsedCommand cmd =
                    parse_command(message->data, message->length);
                state = handle_disarmed_command(cmd);
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
            if (!imu_data_ready()) {
                // IMU not ready - use idle time for non-critical tasks
                lora_service();

                if (lora_data_available()) {
                    lora_message_t *message = lora_get_received_data();
                    ParsedCommand cmd =
                        parse_command(message->data, message->length);
                    state = handle_flying_command(cmd, message, &base_throttle,
                                                  &pid);
                    lora_clear_received_flag();
                }

                // Send telemetry during idle time (never blocks control loop)
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

            send_telem(&imu, &pid);

            break;

        case STATE_EMERGENCY_STOP:
            lora_service();

            if (lora_data_available()) {
                lora_message_t *message = lora_get_received_data();
                ParsedCommand cmd =
                    parse_command(message->data, message->length);
                state = handle_emergency_stop_command(cmd);
                lora_clear_received_flag();
            }

            StopMotors();

            break;
        }
    }

    return 0;
}

void drive_motors(float base_throttle, PID *pid) {

    // these values are flipped since they are wired opposite of placement
    float motor4_speed = base_throttle + pid->output.roll + pid->output.pitch +
                         pid->output.yaw; // north east
    float motor3_speed = base_throttle - pid->output.roll + pid->output.pitch -
                         pid->output.yaw; // north west
    float motor2_speed = base_throttle + pid->output.roll - pid->output.pitch +
                         pid->output.yaw; // south east
    float motor1_speed = base_throttle - pid->output.roll - pid->output.pitch -
                         pid->output.yaw; // south west
    // Clamp to 0-1 range
    motor1_speed = fmaxf(0.0f, fminf(1.0f, motor1_speed));
    motor2_speed = fmaxf(0.0f, fminf(1.0f, motor2_speed));
    motor3_speed = fmaxf(0.0f, fminf(1.0f, motor3_speed));
    motor4_speed = fmaxf(0.0f, fminf(1.0f, motor4_speed));

    // Ensure each motor is above minimum throttle
    if (motor1_speed < MIN_THROTTLE)
        motor1_speed = MIN_THROTTLE;
    if (motor2_speed < MIN_THROTTLE)
        motor2_speed = MIN_THROTTLE;
    if (motor3_speed < MIN_THROTTLE)
        motor3_speed = MIN_THROTTLE;
    if (motor4_speed < MIN_THROTTLE)
        motor4_speed = MIN_THROTTLE;

    // 50-1024 range
    SetMotorThrottle(motor1, (int16_t)(motor1_speed * 1000.0 + 1.0));
    SetMotorThrottle(motor2, (int16_t)(motor2_speed * 1000.0 + 1.0));
    SetMotorThrottle(motor3, (int16_t)(motor3_speed * 1000.0 + 1.0));
    SetMotorThrottle(motor4, (int16_t)(motor4_speed * 1000.0 + 1.0));
}
