#include "pid.h"

const float PI = 3.141592;

void pid_init(PIDContext *pid, float Kp, float Ki, float Kd) {
  PIDController controller = (PIDController){
      .Kp = Kp,
      .Ki = Ki,
      .Kd = Kd,
      .integral = 0,
      .previous_error = 0,
      .output_limit = -0.5,
      .integral_limit = 0.25,
  };

  *pid = (PIDContext){
      .pitch_pid = controller,
      .roll_pid = controller,
      .yaw_pid = controller,
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

  float output = P + I + D;

  output = constrainf(output, -pid->output_limit, pid->output_limit);

  pid->previous_error = error;

  return output;
}

// update all 3 pid axis
void pid_update(PIDContext *pid, IMUContext *imu, float dt) {
  pid->measurement = imu->attitude;

  pid->output.pitch = get_pid_output(&pid->pitch_pid, pid->setpoints.pitch,
                                     pid->measurement.pitch, imu->gyro[1], dt);
  pid->output.roll = get_pid_output(&pid->roll_pid, pid->setpoints.roll,
                                    pid->measurement.roll, imu->gyro[0], dt);
  pid->output.yaw = get_pid_output(&pid->yaw_pid, pid->setpoints.yaw,
                                   pid->measurement.yaw, imu->gyro[2], dt);
}
