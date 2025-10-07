#include "pid.h"

const float PI = 3.141592;

PIDController create_pid(float Kp, float Ki, float Kd, float min, float max) {
  PIDController pid;

  pid.Kp = Kp;
  pid.Ki = Ki;
  pid.Kd = Kd;

  pid.integral = 0;
  pid.previous_error = 0;
  pid.output_min = min;
  pid.output_max = max;

  return pid;
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

float update_pid(PIDController *pid, float setpoint, float measurement,
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
