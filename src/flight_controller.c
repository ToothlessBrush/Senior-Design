#include "flight_controller.h"
#include "comm.h"
#include "crsf.h"
#include "imu.h"
#include "motor_control.h"
#include "optical_flow.h"
#include "pid.h"
#include "protocol.h"
#include "system.h"
#include "systick.h"
#include "utils.h"
#include <math.h>

#define IMU_ODR_HZ 6660.0f
#define FIXED_DT (1.0f / IMU_ODR_HZ)

#define MIN_THROTTLE 0.05f
#define MAX_THROTTLE 1.0f

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

uint8_t fc_state = 0;

static IMU imu = {0};
static PID pid = {0};
static float base_throttle = 0.0f;

typedef struct {
  float motor1; /**< Motor 1 bias (rear left, CW) */
  float motor2; /**< Motor 2 bias (front right, CCW) */
  float motor3; /**< Motor 3 bias (rear right, CCW) */
  float motor4; /**< Motor 4 bias (front left, CW) */
} MotorBias;

static MotorBias bias = {0};

static void drive_motors(float throttle, PID *p, MotorBias *b) {
  float m4 = throttle + b->motor4 + p->output.pitch + p->output.yaw;
  float m3 = throttle + b->motor3 - p->output.roll - p->output.yaw;
  float m2 = throttle + b->motor2 + p->output.roll - p->output.yaw;
  float m1 = throttle + b->motor1 - p->output.pitch + p->output.yaw;

  m1 = fmaxf(MIN_THROTTLE, fminf(MAX_THROTTLE, m1));
  m2 = fmaxf(MIN_THROTTLE, fminf(MAX_THROTTLE, m2));
  m3 = fmaxf(MIN_THROTTLE, fminf(MAX_THROTTLE, m3));
  m4 = fmaxf(MIN_THROTTLE, fminf(MAX_THROTTLE, m4));

  SetMotorThrottle(motor1, (int16_t)(m1 * 1000.0f + 1.0f));
  SetMotorThrottle(motor2, (int16_t)(m2 * 1000.0f + 1.0f));
  SetMotorThrottle(motor3, (int16_t)(m3 * 1000.0f + 1.0f));
  SetMotorThrottle(motor4, (int16_t)(m4 * 1000.0f + 1.0f));
}

void fc_init(void) {
  PIDCreateInfo pid_info = {0}; // gains configured via tuning commands
  imu_init(&imu);
  pid_init(&pid, &pid_info);
  crsf_init();
  led_init();
  led_off();

  optical_flow_init();
  comm_init();
  comm_send_string("CMD:GET_CONFIG"); // request full config from ground station
}

void arm(void) {
  base_throttle = 0.0f;
  pid_reset(&pid);
  IMU_reset_velocity(&imu);

  InitMotors();
  StartMotors();
  delay_ms(2500); // ESC initialization

  fc_state |= FC_ARMED;
}

void disarm(void) {
  StopMotors();
  fc_state &= ~FC_ARMED;
}

void task_imu_pid(void) {
  if (!IS_ARMED(fc_state))
    return;

  if (!imu_data_ready())
    return;

  IMU_update(&imu, FIXED_DT);
  pid_update(&pid, &imu, FIXED_DT);
  drive_motors(base_throttle, &pid, &bias);
  send_telem(&imu, &pid); // self-rate-limited to MIN_SEND_INTERVAL
}

void task_led(void) {
  static uint8_t tick = 0;
  tick++;

  if (IS_ARMED(fc_state)) {
    // Fast blink: 5Hz (on/off every tick)
    (tick % 2) ? led_on() : led_off();
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
      comm_send_string_nb("ERR:BAD_AXIS");
      return;
    }
    axis_pid->Kp = cmd.payload.pid.P;
    axis_pid->Ki = cmd.payload.pid.I;
    axis_pid->Kd = cmd.payload.pid.D;
    axis_pid->integral_limit = cmd.payload.pid.I_limit;
    axis_pid->output_limit = cmd.payload.pid.pid_limit;
    comm_send_string_nb("ACK:PID");
    break;
  }

  case CMD_SET_MOTOR_BIAS:
    bias.motor1 = cmd.payload.bias.motor1;
    bias.motor2 = cmd.payload.bias.motor2;
    bias.motor3 = cmd.payload.bias.motor3;
    bias.motor4 = cmd.payload.bias.motor4;
    comm_send_string_nb("ACK:BIAS");
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

    comm_send_string_nb("ACK:CONFIG");
    break;

  case CMD_CALIBRATE:
    if (!IS_ARMED(fc_state)) {
      toggle_led();
      IMU_calibrate(&imu, 6660);
      toggle_led();
      comm_send_string_nb("ACK:CALIBRATE");
    }
    break;

  default:
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

    // Map sticks to setpoints and throttle while armed
    float roll_ch = crsf_get_channel(CRSF_CH_ROLL);
    float pitch_ch = crsf_get_channel(CRSF_CH_PITCH);
    float yaw_ch = crsf_get_channel(CRSF_CH_YAW);

    pid.setpoints.roll = ((roll_ch - CRSF_MID) / CRSF_RANGE) * MAX_ROLL_ANGLE;
    pid.setpoints.pitch =
        ((pitch_ch - CRSF_MID) / CRSF_RANGE) * MAX_PITCH_ANGLE;
    pid.setpoints.yaw = ((yaw_ch - CRSF_MID) / CRSF_RANGE) * MAX_YAW_ANGLE;

    base_throttle = (throttle_ch - CRSF_THR_MIN) / CRSF_THR_SPAN;
    if (base_throttle < 0.0f)
      base_throttle = 0.0f;
    if (base_throttle > 1.0f)
      base_throttle = 1.0f;
  }
}
