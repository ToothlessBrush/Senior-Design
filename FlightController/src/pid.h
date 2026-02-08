/**
 * PID controller for quadcopter attitude control
 * Implements proportional-integral-derivative control for pitch, roll, and yaw
 */

#ifndef PID_H
#define PID_H

#include "imu.h"

/**
 * Single-axis PID controller
 */
typedef struct {
    // PID gains
    float Kp; // Proportional gain
    float Ki; // Integral gain
    float Kd; // Derivative gain

    // Internal state
    float integral;       // Accumulated integral term
    float previous_error; // Error from previous timestep (for derivative)
    float integral_limit;

    // Output limits
    float output_limit; // Minimum output value

    // Individual term values (for telemetry)
    float p_term;
    float i_term;
    float d_term;
} PIDController;

/**
 * Multi-axis PID context for attitude control
 * Contains separate PID controllers for pitch, roll, and yaw
 */
typedef struct {
    Attitude setpoints;   // Desired attitude (target angles)
    Attitude measurement; // Current attitude (from IMU)
    Attitude output;      // PID control outputs

    PIDController pitch_pid; // Pitch axis controller
    PIDController roll_pid;  // Roll axis controller
    PIDController yaw_pid;   // Yaw axis controller

    // velocity correction PIDs (for position hold)
    PIDController velocity_x_pid; // Forward/backward acceleration correction
    PIDController velocity_y_pid; // Left/right acceleration correction

    Attitude base_setpoints; // Base setpoints (before acceleration correction)
    bool velocity_correction_enabled; // Enable/disable acceleration correction

} PID;

typedef struct {
    float roll_Kp;
    float roll_Ki;
    float roll_Kd;
    float roll_Ki_limit;
    float roll_limit;

    float pitch_Kp;
    float pitch_Ki;
    float pitch_Kd;
    float pitch_Ki_limit;
    float pitch_limit;

    float yaw_Kp;
    float yaw_Ki;
    float yaw_Kd;
    float yaw_Ki_limit;
    float yaw_limit;

    // Acceleration correction PID parameters
    float velocity_x_Kp;
    float velocity_x_Ki;
    float velocity_x_Kd;
    float velocity_x_Ki_limit;
    float velocity_x_limit;

    float velocity_y_Kp;
    float velocity_y_Ki;
    float velocity_y_Kd;
    float velocity_y_Ki_limit;
    float velocity_y_limit;

} PIDCreateInfo;

/**
 * Initialize PID context with gain values
 * Sets up all three axis controllers with the same gains
 *
 * @param pid Pointer to PID context to initialize
 * @param create_info info for creating pid such as gains and limits
 */
void pid_init(PID *pid, const PIDCreateInfo *create_info);

/**
 * Update all PID controllers with current IMU measurements
 * Calculates control outputs for pitch, roll, and yaw
 *
 * @param pid Pointer to PID context
 * @param imu Pointer to IMU context (provides current attitude)
 * @param dt Time delta since last update (seconds)
 */
void pid_update(PID *pid, const IMU *imu, float dt);

/**
 * Update acceleration correction PIDs to adjust setpoints
 * Prevents horizontal drift by correcting for measured acceleration
 *
 * @param pid Pointer to PID context
 * @param imu Pointer to IMU context (provides acceleration data)
 * @param dt Time delta since last update (seconds)
 */
void pid_velocity_correction(PID *pid, const IMU *imu, float dt);

void pid_reset(PID *pid);

/**
 * Calculate PID output for a single axis
 * Implements the full PID algorithm with output clamping
 *
 * @param pid Pointer to single-axis PID controller
 * @param setpoint Desired value (target angle)
 * @param measurement Current value (measured angle)
 * @param angular_velocity rate of change of value
 * @param dt Time delta since last update (seconds)
 * @return Clamped PID output
 */
float get_pid_output(PIDController *pid, float setpoint, float measurement,
                     float angular_velocity, float dt);

#endif // PID_H
