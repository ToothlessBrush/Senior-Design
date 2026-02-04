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
    STATE_ARMING_MANUAL,  // transitioning to manual mode
    STATE_DISARMING,      // transition to disarmed
    STATE_FLYING,         // currently flying
    STATE_EMERGENCY_STOP, // stop everthing
    STATE_CALIBRATE,
    STATE_MANUAL,
} State;

typedef struct {
    float motor1;
    float motor2;
    float motor3;
    float motor4;
} MotorBias;

void drive_motors(float base_throttle, PID *pid, MotorBias *bias);

static State handle_manual_command(ParsedCommand cmd, MotorBias *bias,
                                   uint32_t *last_heartbeat_time) {
    *last_heartbeat_time = millis();
    switch (cmd.type) {
    case CMD_HEART_BEAT:
        return STATE_MANUAL;

    case CMD_SET_MOTOR_BIAS:
        // Values are already normalized (0.0-1.0)
        bias->motor1 = cmd.payload.bias.motor1;
        bias->motor2 = cmd.payload.bias.motor2;
        bias->motor3 = cmd.payload.bias.motor3;
        bias->motor4 = cmd.payload.bias.motor4;
        lora_send_string_nb(1, "LOG:MOTOR_BIAS_SET");
        return STATE_MANUAL;

    case CMD_STOP:
        lora_send_string_nb(1, "LOG:CMD_STOP");
        StopMotors();
        return STATE_DISARMED;

    case CMD_EMERGENCY_STOP:
        lora_send_string_nb(1, "LOG:CMD_EMERGENCY_STOP");
        StopMotors();
        return STATE_EMERGENCY_STOP;

    default:
        return STATE_MANUAL;
    }
}

static State handle_disarmed_command(ParsedCommand cmd, PID *pid,
                                     MotorBias *bias) {
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

    case CMD_CONFIG:
        lora_send_string(1, "LOG:SYNC_CONFIG");

        bias->motor1 = cmd.payload.sync.motor1;
        bias->motor2 = cmd.payload.sync.motor2;
        bias->motor3 = cmd.payload.sync.motor3;
        bias->motor4 = cmd.payload.sync.motor4;

        pid->pitch_pid.Kp = cmd.payload.sync.pitch_Kp;
        pid->pitch_pid.Ki = cmd.payload.sync.pitch_Ki;
        pid->pitch_pid.Kd = cmd.payload.sync.pitch_Kd;
        pid->pitch_pid.integral_limit = cmd.payload.sync.pitch_i_limit;
        pid->pitch_pid.output_limit = cmd.payload.sync.pitch_pid_limit;

        pid->roll_pid.Kp = cmd.payload.sync.roll_Kp;
        pid->roll_pid.Ki = cmd.payload.sync.roll_Ki;
        pid->roll_pid.Kd = cmd.payload.sync.roll_Kd;
        pid->roll_pid.integral_limit = cmd.payload.sync.roll_i_limit;
        pid->roll_pid.output_limit = cmd.payload.sync.roll_pid_limit;

        pid->yaw_pid.Kp = cmd.payload.sync.yaw_Kp;
        pid->yaw_pid.Ki = cmd.payload.sync.yaw_Ki;
        pid->yaw_pid.Kd = cmd.payload.sync.yaw_Kd;
        pid->yaw_pid.integral_limit = cmd.payload.sync.yaw_i_limit;
        pid->yaw_pid.output_limit = cmd.payload.sync.yaw_pid_limit;
        return STATE_DISARMED;

    case CMD_SET_MOTOR_BIAS:
        // Values are already normalized (0.0-1.0)
        bias->motor1 = cmd.payload.bias.motor1;
        bias->motor2 = cmd.payload.bias.motor2;
        bias->motor3 = cmd.payload.bias.motor3;
        bias->motor4 = cmd.payload.bias.motor4;
        lora_send_string(1, "LOG:MOTOR_BIAS_SET");
        return STATE_DISARMED;

    case CMD_START_MANUAL:
        lora_send_string(1, "LOG:CMD_START_MANUAL");
        return STATE_ARMING_MANUAL;

    default:
        return STATE_DISARMED;
    }
}

static State handle_flying_command(ParsedCommand cmd, MotorBias *bias,
                                   float *base_throttle, PID *pid,
                                   uint32_t *last_heartbeat_time) {
    *last_heartbeat_time = millis();
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
    case CMD_SET_MOTOR_BIAS:
        // Values are already normalized (0.0-1.0)
        bias->motor1 = cmd.payload.bias.motor1;
        bias->motor2 = cmd.payload.bias.motor2;
        bias->motor3 = cmd.payload.bias.motor3;
        bias->motor4 = cmd.payload.bias.motor4;
        lora_send_string_nb(1, "LOG:MOTOR_BIAS_SET");
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

    case CMD_SET_PID:
        lora_send_string_nb(1, "LOG:CMD_SET_PID");
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
            lora_send_string_nb(1, "LOG:INVALID PID AXIS");
            return STATE_FLYING;
        }
        active_pid->Kp = cmd.payload.pid.P;
        active_pid->Ki = cmd.payload.pid.I;
        active_pid->Kd = cmd.payload.pid.D;
        active_pid->integral_limit = cmd.payload.pid.I_limit;
        active_pid->output_limit = cmd.payload.pid.pid_limit;

        lora_send_string_nb(1, "LOG:PID_SET_SUCCESS");
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
            delay_ms(500);
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

            lora_send_string(1, "CMD:GET_CONFIG");
            delay_ms(300);

            led_init();
            state = STATE_DISARMED;
            break;

        case STATE_DISARMED:
            lora_service();

            if (lora_data_available()) {
                lora_message_t *message = lora_get_received_data();
                ParsedCommand cmd =
                    parse_command(message->data, message->length);
                state = handle_disarmed_command(cmd, &pid, &bias);
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

        case STATE_ARMING_MANUAL:
            // Reset motor bias to zero
            bias.motor1 = 0;
            bias.motor2 = 0;
            bias.motor3 = 0;
            bias.motor4 = 0;

            lora_service(); // Process any pending LoRa data
            lora_send_string(1, "LOG:ARMING_MANUAL");
            InitMotors();
            StartMotors();

            delay_ms(2500);

            lora_send_string(1, "LOG:MANUAL_ARMED");
            lora_service();                 // Ensure message is processed
            last_heartbeat_time = millis(); // Initialize heartbeat timer
            state = STATE_MANUAL;
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
                    state = handle_flying_command(cmd, &bias, &base_throttle,
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

            lora_service();

            if (lora_data_available()) {
                lora_message_t *message = lora_get_received_data();
                ParsedCommand cmd =
                    parse_command(message->data, message->length);
                state = handle_manual_command(cmd, &bias, &last_heartbeat_time);
                lora_clear_received_flag();
            }

            // Drive motors with manual bias values (no PID correction)
            // Set PID output to zero for manual mode
            pid.output.pitch = 0.0f;
            pid.output.roll = 0.0f;
            pid.output.yaw = 0.0f;
            drive_motors(0.0f, &pid, &bias);

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

    // Motor mixing: pitch, roll, and yaw corrections
    // CW props (1,4) create CCW torque -> add yaw | CCW props (2,3) create CW
    // torque -> subtract yaw
    float motor4_speed =
        base_throttle + bias->motor4 + pid->output.pitch + pid->output.yaw;
    float motor3_speed =
        base_throttle + bias->motor3 - pid->output.roll - pid->output.yaw;
    float motor2_speed =
        base_throttle + bias->motor2 + pid->output.roll - pid->output.yaw;
    float motor1_speed =
        base_throttle + bias->motor1 - pid->output.pitch + pid->output.yaw;

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
