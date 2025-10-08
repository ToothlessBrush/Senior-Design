#include "pid.h"

const float PI = 3.141592;

void pid_init(PIDContext *pid, float Kp, float Ki, float Kd) {
  PIDController controller = (PIDController){
      .Kp = Kp,
      .Ki = Ki,
      .Kd = Kd,
      .integral = 0,
      .previous_error = 0,
      .output_min = -100.0,
      .output_max = 100.0,
  };

  *pid = (PIDContext){
      .pitch_pid = controller,
      .roll_pid = controller,
      .yaw_pid = controller,
  };
}

float normalize_angle(float angle) {
  while (angle > PI) {
    angle = angle - 2 * PI;
  }
  while (angle < -PI) {
    angle = angle + 2 * PI;
  }
  return angle;
}

float get_pid_output(PIDController *pid, float setpoint, float measurement,
                     float dt) {
  float error = normalize_angle(setpoint - measurement);

  float P = pid->Kp * error;

  pid->integral = pid->integral + error * dt;
  float I = pid->Ki * pid->integral;

  float derivative = (error - pid->previous_error) / dt;
  float D = pid->Kd * derivative;

  float output = P + I + D;

  if (output > pid->output_max) {
    output = pid->output_max;
  } else if (output < pid->output_min) {
    output = pid->output_min;
  }

  pid->previous_error = error;

  return output;
}

void pid_update(PIDContext *pid, IMUContext *imu, float dt) {
  pid->measurement = imu->attitude;

  pid->output.pitch = get_pid_output(&pid->pitch_pid, pid->setpoints.pitch,
                                     pid->measurement.pitch, dt);
  pid->output.roll = get_pid_output(&pid->roll_pid, pid->setpoints.roll,
                                    pid->measurement.roll, dt);
  pid->output.yaw = get_pid_output(&pid->yaw_pid, pid->setpoints.yaw,
                                   pid->measurement.yaw, dt);
}
