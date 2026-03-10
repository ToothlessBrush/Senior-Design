#include "flight_controller.h"
#include "imu.h"
#include "motor_control.h"
#include "pid.h"
#include "system.h"
#include "systick.h"
#include <math.h>

#define IMU_ODR_HZ 6660.0f
#define FIXED_DT (1.0f / IMU_ODR_HZ) // ~0.00015015 seconds

#define MAX_ANGLE 1.5f
#define KP 0.0f
#define KI 0.0f
#define KD 0.0f

#define MIN_THROTTLE 0.05f
#define MAX_THROTTLE 1.0f
#define HEARTBEAT_TIMEOUT_MS 2000
#define LED_FLASH_DURATION_MS 50

static IMU imu = {0};
static PID pid = {0};
static float base_throttle = 0.0f; // Base throttle (0.0 to 1.0)

typedef struct {
    float motor1; /**< Motor 1 bias (rear left, CW) */
    float motor2; /**< Motor 2 bias (front right, CCW) */
    float motor3; /**< Motor 3 bias (rear right, CCW) */
    float motor4; /**< Motor 4 bias (front left, CW) */
} MotorBias;

static MotorBias bias = {0};

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

void task_imu_pid() {
    if (!IS_ARMED(state)) {
        return;
    }

    if (!imu_data_ready()) {
        return;
    }

    IMU_update(&imu, FIXED_DT);

    pid_update(&pid, &imu, FIXED_DT);

    drive_motors(base_throttle, &pid, &bias);
}

void task_led() {
    if (IS_CONNECTED(state)) {
        led_on();
        return;
    }

    toggle_led();
}

void arm() {
    base_throttle = 0.0;

    pid_reset(&pid);
    IMU_reset_velocity(&imu);

    InitMotors();
    StartMotors();

    delay_ms(2500);

    state |= FC_ARMED;
}

void disarm() {
    StopMotors();

    state &= ~FC_ARMED;
}
