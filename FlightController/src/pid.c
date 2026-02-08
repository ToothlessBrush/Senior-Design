#include "pid.h"

const float PI = 3.141592;

void pid_init(PID *pid, PIDCreateInfo create_info) {
    PIDController roll_controller = (PIDController){
        .Kp = create_info.roll_Kp,
        .Ki = create_info.roll_Ki,
        .Kd = create_info.roll_Kd,
        .integral = 0,
        .previous_error = 0,
        .output_limit = create_info.roll_limit,
        .integral_limit = create_info.roll_Ki_limit,
    };

    PIDController pitch_controller = (PIDController){
        .Kp = create_info.pitch_Kp,
        .Ki = create_info.pitch_Ki,
        .Kd = create_info.pitch_Kd,
        .integral = 0,
        .previous_error = 0,
        .output_limit = create_info.pitch_limit,
        .integral_limit = create_info.pitch_Ki_limit,
    };

    PIDController yaw_controller = (PIDController){
        .Kp = create_info.yaw_Kp,
        .Ki = create_info.yaw_Ki,
        .Kd = create_info.yaw_Kd,
        .integral = 0,
        .previous_error = 0,
        .output_limit = create_info.yaw_limit,
        .integral_limit = create_info.yaw_Ki_limit,
    };

    PIDController velocity_x_controller = (PIDController){
        .Kp = create_info.velocity_x_Kp,
        .Ki = create_info.velocity_x_Ki,
        .Kd = create_info.velocity_x_Kd,
        .integral = 0,
        .previous_error = 0,
        .output_limit = create_info.velocity_x_limit,
        .integral_limit = create_info.velocity_x_Ki_limit,
    };

    PIDController velocity_y_controller = (PIDController){
        .Kp = create_info.velocity_y_Kp,
        .Ki = create_info.velocity_y_Ki,
        .Kd = create_info.velocity_y_Kd,
        .integral = 0,
        .previous_error = 0,
        .output_limit = create_info.velocity_y_limit,
        .integral_limit = create_info.velocity_y_Ki_limit,
    };

    *pid = (PID){
        .pitch_pid = pitch_controller,
        .roll_pid = roll_controller,
        .yaw_pid = yaw_controller,
        .velocity_x_pid = velocity_x_controller,
        .velocity_y_pid = velocity_y_controller,
        .velocity_correction_enabled =
            true, // disabled during non-zero setpoint
    };
}

static inline float constrainf(float val, float min, float max) {
    if (val < min)
        return min;
    if (val > max)
        return max;
    return val;
}

static float normalize_angle(float angle) {
    while (angle > PI) {
        angle = angle - 2 * PI;
    }
    while (angle < -PI) {
        angle = angle + 2 * PI;
    }
    return angle;
}

void pid_reset(PID *pid) {
    PIDController *pitch = &pid->pitch_pid;
    pitch->integral = 0.0;
    pitch->previous_error = 0.0;
    PIDController *roll = &pid->roll_pid;
    roll->integral = 0.0;
    roll->previous_error = 0.0;
    PIDController *yaw = &pid->yaw_pid;
    yaw->integral = 0.0;
    yaw->previous_error = 0.0;

    // Reset acceleration correction PIDs
    PIDController *accel_x = &pid->velocity_x_pid;
    accel_x->integral = 0.0;
    accel_x->previous_error = 0.0;
    PIDController *accel_y = &pid->velocity_y_pid;
    accel_y->integral = 0.0;
    accel_y->previous_error = 0.0;
}

// get output from error
float get_pid_output(PIDController *pid, float setpoint, float measurement,
                     float angular_velocity, float dt) {
    float error = normalize_angle(setpoint - measurement);

    // P term
    float P = pid->Kp * error;

    // I term (steady state correction)
    pid->integral = pid->integral + error * dt;
    pid->integral =
        constrainf(pid->integral, -pid->integral_limit, pid->integral_limit);
    float I = pid->Ki * pid->integral;

    // D term (brake force)
    float D = -pid->Kd * angular_velocity;

    // Store individual terms for telemetry
    pid->p_term = P;
    pid->i_term = I;
    pid->d_term = D;

    float output = P + I + D;

    output = constrainf(output, -pid->output_limit, pid->output_limit);

    pid->previous_error = error;

    return output;
}

// Acceleration correction to prevent horizontal drift
void pid_velocity_correction(PID *pid, IMU *imu, float dt) {
    if (!pid->velocity_correction_enabled) {
        // If disabled, just copy base setpoints to active setpoints
        pid->setpoints = pid->base_setpoints;
        return;
    }

    float pitch_correction =
        get_pid_output(&pid->velocity_x_pid, 0.0f, imu->velocity[0],
                       imu->accel_hp[0] * 9.81f, dt);

    float roll_correction =
        get_pid_output(&pid->velocity_y_pid, 0.0f, imu->velocity[1],
                       imu->accel_hp[1] * 9.81f, dt);

    pid->setpoints.pitch = pid->base_setpoints.pitch - pitch_correction;
    pid->setpoints.roll = pid->base_setpoints.roll - roll_correction;
}

// update all 3 pid axis
void pid_update(PID *pid, IMU *imu, float dt) {
    pid->measurement = imu->attitude;

    pid->output.pitch =
        get_pid_output(&pid->pitch_pid, pid->setpoints.pitch,
                       pid->measurement.pitch, imu->gyro[1], dt);
    pid->output.roll = get_pid_output(&pid->roll_pid, pid->setpoints.roll,
                                      pid->measurement.roll, imu->gyro[0], dt);
    pid->output.yaw = get_pid_output(&pid->yaw_pid, pid->setpoints.yaw,
                                     pid->measurement.yaw, imu->gyro[2], dt);
}
