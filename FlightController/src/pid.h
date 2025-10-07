typedef struct PIDController {
  float Kp;
  float Ki;
  float Kd;

  float integral;
  float previous_error;
  float output_min;
  float output_max;
} PIDController;

PIDController create_pid(float Kp, float Ki, float Kd, float min, float max);

float update_pid(PIDController *pid, float setpoint, float measurement,
                 float dt);
