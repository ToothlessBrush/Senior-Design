#include "flight_controller.h"
#include "comm.h"
#include "crsf.h"
#include "imu.h"
#include "motor_control.h"
#include "optical_flow.h"
#include "pid.h"
#include "protocol.h"
#include "spi.h"
#include "system.h"
#include "systick.h"
#include <math.h>

// #define NO_IMU /* uncomment to test throttle/motors without IMU hardware */

#define IMU_ODR_HZ 6660.0f
#define FIXED_DT (1.0f / IMU_ODR_HZ)
#define IMU_DRDY_TIMEOUT_US 5000 // 5 ms ~ 33 missed DRDY periods

#define MIN_THROTTLE 0.05f
#define MAX_THROTTLE 1.0f
#define MAX_BASE_THROTTLE 1.0f // full stick = 40% throttle

// CRSF channel indices — AETR order
#define CRSF_CH_ROLL 0     // A - Aileron
#define CRSF_CH_PITCH 1    // E - Elevator
#define CRSF_CH_THROTTLE 2 // T - Throttle
#define CRSF_CH_YAW 3      // R - Rudder
#define CRSF_CH_ARM 4      // CH5 - Arm switch

#define CRSF_MID 1500.0f
#define CRSF_RANGE 500.0f // 1500 ± 500 spans 988–2012 µs
#define CRSF_THR_MIN 988.0f
#define CRSF_THR_SPAN 1024.0f // 2012 - 988

#define CRSF_ARM_THRESHOLD 1700.0f
#define CRSF_DISARM_THRESHOLD 1300.0f
#define CRSF_THROTTLE_LOW 1100.0f
#define CRSF_SIGNAL_TIMEOUT_MS 1000

// Maximum stick-commanded angles (radians)
#define MAX_ROLL_ANGLE 0.5f  // ~28 deg
#define MAX_PITCH_ANGLE 0.5f // ~28 deg
#define MAX_YAW_ANGLE 0.5f   // ~28 deg

// persistant data
#define FLASH_CONFIG_MAGIC 0xDEADBEF0 // bumped: CommandConfig grew 76→116 bytes

typedef struct __attribute__((packed)) {
    uint32_t magic;
    CommandConfig config;
    IMU_Calibration cal;
} FlashConfig;

uint8_t fc_state = 0;

static IMU imu = {0};
static PID pid = {0};
static float base_throttle = 0.0f;

typedef struct {
    float motor1; /**< Motor 1 bias (front-right, +x+y, CW) */
    float motor2; /**< Motor 2 bias (front-left,  -x+y, CCW) */
    float motor3; /**< Motor 3 bias (rear-right,  +x-y, CW) */
    float motor4; /**< Motor 4 bias (rear-left,   -x-y, CCW) */
} MotorBias;

static MotorBias bias = {0};

static uint8_t led_config_blink_ticks = 0;

// Frame: Y+ = front, X+ = right, Z+ = up (right-hand).
// Roll  = rotation about Y (front): right motors (+x) get +roll, left motors
// (-x) get -roll. Pitch = rotation about X (right): front motors (+y) get
// +pitch, rear motors (-y) get -pitch. Yaw   = rotation about Z: CW motors
// (M1,M3) react +yaw, CCW motors (M2,M4) react -yaw.
static void drive_motors(float throttle, PID *p, MotorBias *b) {
    // +x +y (front-right, CW)
    float m1 =
        throttle + b->motor1 + p->output.roll + p->output.pitch + p->output.yaw;
    // -x +y (front-left, CCW)
    float m2 =
        throttle + b->motor2 - p->output.roll + p->output.pitch - p->output.yaw;
    // +x -y (rear-right, CW)
    float m3 =
        throttle + b->motor3 + p->output.roll - p->output.pitch + p->output.yaw;
    // -x -y (rear-left, CCW)
    float m4 =
        throttle + b->motor4 - p->output.roll - p->output.pitch - p->output.yaw;

    m1 = fmaxf(MIN_THROTTLE, fminf(MAX_THROTTLE, m1));
    m2 = fmaxf(MIN_THROTTLE, fminf(MAX_THROTTLE, m2));
    m3 = fmaxf(MIN_THROTTLE, fminf(MAX_THROTTLE, m3));
    m4 = fmaxf(MIN_THROTTLE, fminf(MAX_THROTTLE, m4));

    SetMotorThrottleSPI(Motor1, (uint16_t)(m1 * 2000.0f + 1.0f));
    SetMotorThrottleSPI(Motor2, (uint16_t)(m2 * 2000.0f + 1.0f));
    SetMotorThrottleSPI(Motor3, (uint16_t)(m3 * 2000.0f + 1.0f));
    SetMotorThrottleSPI(Motor4, (uint16_t)(m4 * 2000.0f + 1.0f));
}

void fc_init(void) {
#ifndef NO_IMU
    if (imu_init(&imu))
        fc_state |= FC_IMU_OK;
#endif
    crsf_init();
    led_init();
    led_off();

    optical_flow_init();
    comm_init();
    SPI2_Init();

    FlashConfig saved;
    flash_read(CONFIG_SECTOR_ADDR, &saved, sizeof(FlashConfig));

    if (saved.magic == FLASH_CONFIG_MAGIC) {
        imu.cal = saved.cal;

        bias.motor1 = saved.config.motor1;
        bias.motor2 = saved.config.motor2;
        bias.motor3 = saved.config.motor3;
        bias.motor4 = saved.config.motor4;

        PIDCreateInfo pid_info = {
            .roll_Kp = saved.config.roll_Kp,
            .roll_Ki = saved.config.roll_Ki,
            .roll_Kd = saved.config.roll_Kd,
            .roll_Ki_limit = saved.config.roll_i_limit,
            .roll_limit = saved.config.roll_pid_limit,
            .pitch_Kp = saved.config.pitch_Kp,
            .pitch_Ki = saved.config.pitch_Ki,
            .pitch_Kd = saved.config.pitch_Kd,
            .pitch_Ki_limit = saved.config.pitch_i_limit,
            .pitch_limit = saved.config.pitch_pid_limit,
            .yaw_Kp = saved.config.yaw_Kp,
            .yaw_Ki = saved.config.yaw_Ki,
            .yaw_Kd = saved.config.yaw_Kd,
            .yaw_Ki_limit = saved.config.yaw_i_limit,
            .yaw_limit = saved.config.yaw_pid_limit,
            .velocity_x_Kp = saved.config.velocity_x_Kp,
            .velocity_x_Ki = saved.config.velocity_x_Ki,
            .velocity_x_Kd = saved.config.velocity_x_Kd,
            .velocity_x_Ki_limit = saved.config.velocity_x_i_limit,
            .velocity_x_limit = saved.config.velocity_x_pid_limit,
            .velocity_y_Kp = saved.config.velocity_y_Kp,
            .velocity_y_Ki = saved.config.velocity_y_Ki,
            .velocity_y_Kd = saved.config.velocity_y_Kd,
            .velocity_y_Ki_limit = saved.config.velocity_y_i_limit,
            .velocity_y_limit = saved.config.velocity_y_pid_limit,
        };

        pid_init(&pid, &pid_info);
        comm_send_string("LOG:INIT_CONFIG_OK");
    } else {
        PIDCreateInfo pid_info = {0};
        pid_init(&pid, &pid_info);
        comm_send_string("LOG:INIT_NO_CONFIG");
    }
}

void arm(void) {
    // reset stuff
    base_throttle = 0.0f;
    pid_reset(&pid);
    imu.attitude.yaw = 0.0;
    IMU_reset_velocity(&imu);

    // start motors — wait for controller to signal ready via PC15
    motor_enable();
    uint32_t deadline = millis() + 5000;
    while (!motor_status_ok() && millis() < deadline) {
    }
    if (!motor_status_ok()) {
        motor_disable();
        return;
    }

    fc_state |= FC_ARMED;
}

void disarm(void) {
    motor_disable();
    fc_state &= ~FC_ARMED;
}

void task_imu_pid(void) {
#ifndef NO_IMU
    static uint32_t last_drdy_us = 0;
    uint32_t now = micros();

    if (imu_data_ready()) {
        last_drdy_us = now;
        fc_state |= FC_IMU_OK;
        IMU_update(&imu, FIXED_DT);
    } else {
        if (now - last_drdy_us > IMU_DRDY_TIMEOUT_US)
            fc_state &= ~FC_IMU_OK;
        return;
    }
#else
    static uint32_t last_us = 0;
    uint32_t now = micros();
    if (now - last_us < 150)
        return;
    last_us = now;
#endif

    pid_update(&pid, &imu, FIXED_DT);

    if (!IS_ARMED(fc_state))
        return;

    drive_motors(base_throttle, &pid, &bias);
}

void task_led(void) {
    static uint8_t tick = 0;
    tick++;

    if (IS_ARMED(fc_state)) {
        // Fast blink: 5Hz (on/off every tick)
        (tick % 2) ? led_on() : led_off();
        return;
    }

    if (led_config_blink_ticks > 0) {
        // Config received: 2 quick blinks
        (led_config_blink_ticks % 2) ? led_on() : led_off();
        led_config_blink_ticks--;
        return;
    }

    if (!IS_IMU_OK(fc_state)) {
        // IMU fault: double-pulse every 2 seconds
        // tick % 8 at 2Hz = 4s cycle; pulses at positions 0,1 and 2,3
        uint8_t t = tick % 8;
        (t == 0 || t == 2) ? led_on() : led_off();
        return;
    }

    if (IS_SIGNAL_OK(fc_state)) {
        // Connected, disarmed: solid on
        led_on();
        return;
    }

    // No signal: slow blink, 1Hz (on for 100ms every second)
    (tick % 10 == 0) ? led_on() : led_off();
}

// Handle configuration commands from LoRa (or future BT module on UART2).
// Only config commands are accepted here — arm/disarm is CRSF-only.
void task_config_service(void) {
    comm_service();

    if (!comm_data_available())
        return;

    comm_message_t *msg = comm_get_received_data();
    ParsedCommand cmd = comm_parse_command(msg->data, msg->length);
    comm_clear_received_flag();

    switch (cmd.type) {
    case CMD_SET_PID: {
        PIDController *axis_pid;
        switch (cmd.payload.pid.axis) {
        case 0:
            axis_pid = &pid.pitch_pid;
            break;
        case 1:
            axis_pid = &pid.roll_pid;
            break;
        case 2:
            axis_pid = &pid.yaw_pid;
            break;
        case 3:
            axis_pid = &pid.velocity_x_pid;
            break;
        case 4:
            axis_pid = &pid.velocity_y_pid;
            break;
        default:
            comm_send_string_nb("LOG:BAD_AXIS");
            return;
        }
        axis_pid->Kp = cmd.payload.pid.P;
        axis_pid->Ki = cmd.payload.pid.I;
        axis_pid->Kd = cmd.payload.pid.D;
        axis_pid->integral_limit = cmd.payload.pid.I_limit;
        axis_pid->output_limit = cmd.payload.pid.pid_limit;

        led_config_blink_ticks = 4;
        comm_send_string_nb("LOG:SET_PID");
        break;
    }

    case CMD_SET_MOTOR_BIAS:
        bias.motor1 = cmd.payload.bias.motor1;
        bias.motor2 = cmd.payload.bias.motor2;
        bias.motor3 = cmd.payload.bias.motor3;
        bias.motor4 = cmd.payload.bias.motor4;
        led_config_blink_ticks = 4;
        comm_send_string_nb("LOG:SET_BIAS");
        break;

    case CMD_CONFIG:
        bias.motor1 = cmd.payload.sync.motor1;
        bias.motor2 = cmd.payload.sync.motor2;
        bias.motor3 = cmd.payload.sync.motor3;
        bias.motor4 = cmd.payload.sync.motor4;

        pid.pitch_pid.Kp = cmd.payload.sync.pitch_Kp;
        pid.pitch_pid.Ki = cmd.payload.sync.pitch_Ki;
        pid.pitch_pid.Kd = cmd.payload.sync.pitch_Kd;
        pid.pitch_pid.integral_limit = cmd.payload.sync.pitch_i_limit;
        pid.pitch_pid.output_limit = cmd.payload.sync.pitch_pid_limit;

        pid.roll_pid.Kp = cmd.payload.sync.roll_Kp;
        pid.roll_pid.Ki = cmd.payload.sync.roll_Ki;
        pid.roll_pid.Kd = cmd.payload.sync.roll_Kd;
        pid.roll_pid.integral_limit = cmd.payload.sync.roll_i_limit;
        pid.roll_pid.output_limit = cmd.payload.sync.roll_pid_limit;

        pid.yaw_pid.Kp = cmd.payload.sync.yaw_Kp;
        pid.yaw_pid.Ki = cmd.payload.sync.yaw_Ki;
        pid.yaw_pid.Kd = cmd.payload.sync.yaw_Kd;
        pid.yaw_pid.integral_limit = cmd.payload.sync.yaw_i_limit;
        pid.yaw_pid.output_limit = cmd.payload.sync.yaw_pid_limit;

        pid.velocity_x_pid.Kp = cmd.payload.sync.velocity_x_Kp;
        pid.velocity_x_pid.Ki = cmd.payload.sync.velocity_x_Ki;
        pid.velocity_x_pid.Kd = cmd.payload.sync.velocity_x_Kd;
        pid.velocity_x_pid.integral_limit = cmd.payload.sync.velocity_x_i_limit;
        pid.velocity_x_pid.output_limit = cmd.payload.sync.velocity_x_pid_limit;

        pid.velocity_y_pid.Kp = cmd.payload.sync.velocity_y_Kp;
        pid.velocity_y_pid.Ki = cmd.payload.sync.velocity_y_Ki;
        pid.velocity_y_pid.Kd = cmd.payload.sync.velocity_y_Kd;
        pid.velocity_y_pid.integral_limit = cmd.payload.sync.velocity_y_i_limit;
        pid.velocity_y_pid.output_limit = cmd.payload.sync.velocity_y_pid_limit;

        led_config_blink_ticks = 4;
        comm_send_string_nb("LOG:SET_CONFIG");
        break;

    case CMD_SAVE: {
        CommandConfig cfg = {
            .motor1 = bias.motor1,
            .motor2 = bias.motor2,
            .motor3 = bias.motor3,
            .motor4 = bias.motor4,
            .roll_Kp = pid.roll_pid.Kp,
            .roll_Ki = pid.roll_pid.Ki,
            .roll_Kd = pid.roll_pid.Kd,
            .roll_i_limit = pid.roll_pid.integral_limit,
            .roll_pid_limit = pid.roll_pid.output_limit,
            .pitch_Kp = pid.pitch_pid.Kp,
            .pitch_Ki = pid.pitch_pid.Ki,
            .pitch_Kd = pid.pitch_pid.Kd,
            .pitch_i_limit = pid.pitch_pid.integral_limit,
            .pitch_pid_limit = pid.pitch_pid.output_limit,
            .yaw_Kp = pid.yaw_pid.Kp,
            .yaw_Ki = pid.yaw_pid.Ki,
            .yaw_Kd = pid.yaw_pid.Kd,
            .yaw_i_limit = pid.yaw_pid.integral_limit,
            .yaw_pid_limit = pid.yaw_pid.output_limit,
            .velocity_x_Kp = pid.velocity_x_pid.Kp,
            .velocity_x_Ki = pid.velocity_x_pid.Ki,
            .velocity_x_Kd = pid.velocity_x_pid.Kd,
            .velocity_x_i_limit = pid.velocity_x_pid.integral_limit,
            .velocity_x_pid_limit = pid.velocity_x_pid.output_limit,
            .velocity_y_Kp = pid.velocity_y_pid.Kp,
            .velocity_y_Ki = pid.velocity_y_pid.Ki,
            .velocity_y_Kd = pid.velocity_y_pid.Kd,
            .velocity_y_i_limit = pid.velocity_y_pid.integral_limit,
            .velocity_y_pid_limit = pid.velocity_y_pid.output_limit,
        };
        FlashConfig to_save = {
            .magic = FLASH_CONFIG_MAGIC,
            .config = cfg,
            .cal = imu.cal,
        };
        bool save_ok = flash_save(CONFIG_SECTOR, CONFIG_SECTOR_ADDR, &to_save,
                                  sizeof(FlashConfig));
        led_config_blink_ticks = 4;
        comm_send_string_nb(save_ok ? "LOG:SAVE_OK" : "LOG:SAVE_FAILED");
        break;
    }

    case CMD_CALIBRATE:
        if (!IS_ARMED(fc_state)) {
            toggle_led();
            IMU_calibrate(&imu, 6660);
            toggle_led();

            FlashConfig to_save;
            flash_read(CONFIG_SECTOR_ADDR, &to_save, sizeof(FlashConfig));
            to_save.magic = FLASH_CONFIG_MAGIC;
            to_save.cal = imu.cal; // overwrite with fresh cal
            bool cal_ok = flash_save(CONFIG_SECTOR, CONFIG_SECTOR_ADDR,
                                     &to_save, sizeof(FlashConfig));
            comm_send_string_nb(cal_ok ? "LOG:CALIBRATE_SAVED"
                                       : "LOG:CALIBRATE_SAVE_FAILED");
        }
        break;

    default:
        comm_send_string_nb("LOG:PARSE_ERR");
        break;
    }
}

void task_optical_flow(void) {
    optical_flow_update();

    if (!IS_ARMED(fc_state))
        return;

    const optical_flow_data_t *of = optical_flow_get_data();
    if (!optical_flow_is_flow_valid(of))
        return;

    // flow_vel_x/y are actual velocity in cm/s (MTF-02P compensates internally)
    imu.velocity.x = of->flow_vel_x / 100.0f;
    imu.velocity.y = of->flow_vel_y / 100.0f;
}

void task_crsf_service(void) {
    crsf_process();

    if (crsf_signal_age_ms() > CRSF_SIGNAL_TIMEOUT_MS) {
        fc_state &= ~FC_SIGNAL_OK;
        if (IS_ARMED(fc_state)) {
            fc_state |= FC_FAILSAFE;
            disarm();
        }
        return;
    }

    fc_state |= FC_SIGNAL_OK;
    fc_state &= ~FC_FAILSAFE;

    float arm_ch = crsf_get_channel(CRSF_CH_ARM);
    float throttle_ch = crsf_get_channel(CRSF_CH_THROTTLE);

    if (!IS_ARMED(fc_state)) {
        if (arm_ch > CRSF_ARM_THRESHOLD && throttle_ch < CRSF_THROTTLE_LOW)
            arm();
    } else {
        if (arm_ch < CRSF_DISARM_THRESHOLD) {
            disarm();
            return;
        }

        // Map sticks to setpoints and throttle while armed.
        // Roll  = rotation about Y (front): aileron stick banks left/right.
        // Pitch = rotation about X (right): elevator stick tilts nose up/down.
        float roll_ch = crsf_get_channel(CRSF_CH_ROLL);
        float pitch_ch = crsf_get_channel(CRSF_CH_PITCH);
        float yaw_ch = crsf_get_channel(CRSF_CH_YAW);

        pid.setpoints.roll =
            ((roll_ch - CRSF_MID) / CRSF_RANGE) * MAX_ROLL_ANGLE;
        pid.setpoints.pitch =
            ((pitch_ch - CRSF_MID) / CRSF_RANGE) * MAX_PITCH_ANGLE;
        pid.setpoints.yaw = ((yaw_ch - CRSF_MID) / CRSF_RANGE) * MAX_YAW_ANGLE;

        base_throttle =
            ((throttle_ch - CRSF_THR_MIN) / CRSF_THR_SPAN) * MAX_BASE_THROTTLE;
        if (base_throttle < 0.0f)
            base_throttle = 0.0f;
        if (base_throttle > MAX_BASE_THROTTLE)
            base_throttle = MAX_BASE_THROTTLE;
    }
}

void task_telementry(void) {
    TelemetryPacket packet = {0};
    packet.timestamp_ms = millis();
    packet.roll = imu.attitude.roll;
    packet.pitch = imu.attitude.pitch;
    packet.yaw = imu.attitude.yaw;
    packet.roll_p_term = pid.roll_pid.p_term;
    packet.roll_i_term = pid.roll_pid.i_term;
    packet.roll_d_term = pid.roll_pid.d_term;
    packet.pitch_p_term = pid.pitch_pid.p_term;
    packet.pitch_i_term = pid.pitch_pid.i_term;
    packet.pitch_d_term = pid.pitch_pid.d_term;
    packet.yaw_p_term = pid.yaw_pid.p_term;
    packet.yaw_i_term = pid.yaw_pid.i_term;
    packet.yaw_d_term = pid.yaw_pid.d_term;
    packet.vel_x = imu.velocity.x;
    packet.vel_y = imu.velocity.y;

    comm_send_frame(BT_TELEM, (const uint8_t *)&packet,
                    sizeof(TelemetryPacket));
}
