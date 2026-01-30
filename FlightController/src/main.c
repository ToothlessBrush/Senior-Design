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

#define MAX_ANGLE 1.5f
#define KP 0.0f
#define KI 0.0f
#define KD 0.0f

#define MIN_THROTTLE 0.05f
#define MAX_THROTTLE 1.0f
#define HEARTBEAT_TIMEOUT_MS 2000

typedef enum {
    STATE_INIT,           // initial state
    STATE_DISARMED,       // initialized but not running
    STATE_ARMING,         // transitioning to flying
    STATE_DISARMING,      // transition to disarmed
    STATE_FLYING,         // currently flying
    STATE_EMERGENCY_STOP, // stop everthing
    STATE_CALIBRATE,
    STATE_MANUAL,
} State;

typedef struct {
    int16_t motor1;
    int16_t motor2;
    int16_t motor3;
    int16_t motor4;
} MotorBias;

void drive_motors(float base_throttle, PID *pid, MotorBias *bias);

static State handle_manual_command(ParsedCommand cmd, MotorBias *bias,
                                   uint32_t *last_heartbeat_time) {
    switch (cmd.type) {
    case CMD_HEART_BEAT:
        *last_heartbeat_time = millis();
        return STATE_MANUAL;
    case CMD_SET_MOTOR_BIAS:
        bias->motor1 = cmd.payload.bias.motor1;
        bias->motor2 = cmd.payload.bias.motor2;
        bias->motor1 = cmd.payload.bias.motor3;
        bias->motor1 = cmd.payload.bias.motor4;
        return STATE_MANUAL;
    }
}

static State handle_disarmed_command(ParsedCommand cmd, PID *pid) {
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
    case CMD_SET_PID:
        lora_send_string(1, "LOG:CMD_SET_PID");
        PIDController *active_pid;
        switch (cmd.payload.pid.axis) {
        case 0: // pitch
            active_pid = &pid->pitch_pid;
            break;
        case 1: // roll
            active_pid = &pid->roll_pid;
            break;
        case 2: // yaw
            active_pid = &pid->yaw_pid;
            break;
        default:
            lora_send_string(1, "LOG:INVALID PID AXIS");
            return STATE_DISARMED;
        }
        active_pid->Kp = cmd.payload.pid.P;
        active_pid->Ki = cmd.payload.pid.I;
        active_pid->Kd = cmd.payload.pid.D;
        active_pid->integral_limit = cmd.payload.pid.I_limit;
        active_pid->output_limit = cmd.payload.pid.pid_limit;

        lora_send_string(1, "LOG:PID_SET_SUCCESS");
        return STATE_DISARMED;
    case CMD_START_MANUAL:

        lora_send_string(1, "LOG:CMD_START_MANUAL");
        return STATE_MANUAL;

    default:
        return STATE_DISARMED;
    }
}

static State handle_flying_command(ParsedCommand cmd, lora_message_t *message,
                                   float *base_throttle, PID *pid,
                                   uint32_t *last_heartbeat_time) {
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

        // Update heartbeat timestamp
        *last_heartbeat_time = millis();
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
    MotorBias bias = {0};
    float base_throttle = 0.10f; // Base throttle (0.0 to 1.0)
    uint32_t last_heartbeat_time = 0;

    PIDCreateInfo pid_info = (PIDCreateInfo){
        .roll_Kp = 0.0f,
        .roll_Ki = 0.0f,
        .roll_Kd = 0.0f,
        .roll_Ki_limit = 0.0f,
        .roll_limit = 0.0f, // 20% throttle

        .pitch_Kp = 0.0f,
        .pitch_Ki = 0.0f,
        .pitch_Kd = 0.0f,
        .pitch_Ki_limit = 0.0f,
        .pitch_limit = 0.0f, // 20% throttle

        .yaw_Kp = 0.0f,
        .yaw_Ki = 0.0f,
        .yaw_Kd = 0.0f,
        .yaw_Ki_limit = 0.0f,
        .yaw_limit = 0.0f, // 10% throttle

    };

    while (1) {
        switch (state) {
        case STATE_INIT:

            // microcontroller config
            SystemClock_Config_100MHz_HSE();
            systick_init();
            // wait for component power on
            delay_ms(1000);
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
                state = handle_disarmed_command(cmd, &pid);
                lora_clear_received_flag();
            }

            break;

        case STATE_CALIBRATE:
            toggle_led();
            IMU_calibrate_accel(&imu, 1000);
            IMU_calibrate_gyro(&imu, 1000);
            toggle_led();
            state = STATE_DISARMED;
            break;

        case STATE_ARMING:
            base_throttle = 0.0;
            pid_reset(&pid);
            lora_service(); // Process any pending LoRa data
            lora_send_string(1, "LOG:ARMING");
            InitMotors();
            StartMotors();

            delay_ms(2500);

            lora_send_string(1, "LOG:ARMED");
            lora_service();                 // Ensure message is processed
            last_heartbeat_time = millis(); // Initialize heartbeat timer
            state = STATE_FLYING;
            break;

        case STATE_DISARMING:
            lora_send_string(1, "LOG:DISARMING");
            StopMotors();
            state = STATE_DISARMED;
            break;

        case STATE_FLYING:
            // Check for heartbeat timeout
            if (millis() - last_heartbeat_time > HEARTBEAT_TIMEOUT_MS) {
                lora_send_string_nb(1, "LOG:HEARTBEAT_TIMEOUT");
                StopMotors();
                state = STATE_EMERGENCY_STOP;
                break;
            }

            if (!imu_data_ready()) {
                // IMU not ready - use idle time for non-critical tasks
                lora_service();

                if (lora_data_available()) {
                    lora_message_t *message = lora_get_received_data();
                    ParsedCommand cmd =
                        parse_command(message->data, message->length);
                    state = handle_flying_command(cmd, message, &base_throttle,
                                                  &pid, &last_heartbeat_time);
                    lora_clear_received_flag();
                }

                break;
            }

            // Update IMU measurements and attitude
            IMU_update(&imu, FIXED_DT);

            // stop if greater then max angle
            // if (fabsf(imu.attitude.roll) > MAX_ANGLE ||
            //     fabsf(imu.attitude.pitch) > MAX_ANGLE) {
            //     StopMotors();
            //     lora_send_string_nb(1, "LOG:MAX_ANGLE_REACHED");
            //     state = STATE_EMERGENCY_STOP;
            //     break;
            // }

            // Update PID controllers
            pid_update(&pid, &imu, FIXED_DT);

            drive_motors(base_throttle, &pid, &bias);

            send_telem(&imu, &pid);

            break;
        case STATE_MANUAL:

            // Check for heartbeat timeout
            if (millis() - last_heartbeat_time > HEARTBEAT_TIMEOUT_MS) {
                lora_send_string_nb(1, "LOG:HEARTBEAT_TIMEOUT");
                StopMotors();
                state = STATE_EMERGENCY_STOP;
                break;
            }

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

void drive_motors(float base_throttle, PID *pid, MotorBias *bias) {

    // these values are flipped since they are wired opposite of placement
    float motor4_speed = base_throttle + bias->motor4 + pid->output.pitch;
    float motor3_speed = base_throttle + bias->motor3 - pid->output.roll;
    float motor2_speed = base_throttle + bias->motor2 + pid->output.roll;
    float motor1_speed = base_throttle + bias->motor1 - pid->output.pitch;

    // Clamp to 0-1 range
    motor1_speed = fmaxf(MIN_THROTTLE, fminf(MAX_THROTTLE, motor1_speed));
    motor2_speed = fmaxf(MIN_THROTTLE, fminf(MAX_THROTTLE, motor2_speed));
    motor3_speed = fmaxf(MIN_THROTTLE, fminf(MAX_THROTTLE, motor3_speed));
    motor4_speed = fmaxf(MIN_THROTTLE, fminf(MAX_THROTTLE, motor4_speed));

    // 50-1024 range
    SetMotorThrottle(motor1, (int16_t)(motor1_speed * 1000.0 + 1.0));
    SetMotorThrottle(motor2, (int16_t)(motor2_speed * 1000.0 + 1.0));
    SetMotorThrottle(motor3, (int16_t)(motor3_speed * 1000.0 + 1.0));
    SetMotorThrottle(motor4, (int16_t)(motor4_speed * 1000.0 + 1.0));
}
