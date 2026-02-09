/**
 * @file pid.h
 * @brief PID controller for quadcopter attitude and velocity control
 *
 * Implements cascaded PID control system:
 * - Inner loop: Attitude control (pitch, roll, yaw) using gyroscope feedback
 * - Outer loop: Velocity correction (position hold) adjusting attitude setpoints
 *
 * The PID uses direct gyroscope rate feedback for the derivative term instead of
 * differentiating error, providing cleaner derivative action and better noise immunity.
 *
 * Features:
 * - Independent PID controllers for each axis
 * - Integral windup protection with configurable limits
 * - Output limiting for safety
 * - Velocity-based position correction (prevents drift)
 * - Telemetry tracking of individual P, I, D terms
 * - Angle normalization for correct wraparound handling
 */

#ifndef PID_H
#define PID_H

#include "imu.h"

/**
 * @brief Single-axis PID controller with anti-windup
 *
 * Implements PID control with direct rate feedback for derivative term.
 * The D term uses measured angular velocity instead of error derivative,
 * eliminating derivative kick and reducing noise sensitivity.
 */
typedef struct {
    float Kp; /**< Proportional gain (error response) */
    float Ki; /**< Integral gain (steady-state error correction) */
    float Kd; /**< Derivative gain (damping, applied to rate measurement) */

    float integral;       /**< Accumulated integral term */
    float previous_error; /**< Error from previous timestep (unused with rate feedback) */
    float integral_limit; /**< Integral windup limit (prevents excessive I accumulation) */

    float output_limit; /**< Output saturation limit (±limit) */

    // Telemetry values
    float p_term; /**< Last P term value (for telemetry/debugging) */
    float i_term; /**< Last I term value (for telemetry/debugging) */
    float d_term; /**< Last D term value (for telemetry/debugging) */
} PIDController;

/**
 * @brief Multi-axis PID context for cascaded attitude and velocity control
 *
 * Contains five PID controllers:
 * - Three for attitude control (pitch, roll, yaw) - inner loop
 * - Two for velocity correction (X, Y axes) - outer loop
 *
 * The velocity correction PIDs adjust the attitude setpoints to counteract
 * horizontal drift, implementing a simple position hold feature.
 */
typedef struct {
    Attitude setpoints;   /**< Active attitude setpoints (after velocity correction) */
    Attitude measurement; /**< Current measured attitude from IMU */
    Attitude output;      /**< PID control outputs (throttle corrections for each axis) */

    PIDController pitch_pid; /**< Pitch axis controller (nose up/down) */
    PIDController roll_pid;  /**< Roll axis controller (left/right tilt) */
    PIDController yaw_pid;   /**< Yaw axis controller (rotation about vertical) */

    PIDController velocity_x_pid; /**< Forward/backward velocity correction (adjusts pitch) */
    PIDController velocity_y_pid; /**< Left/right velocity correction (adjusts roll) */

    Attitude base_setpoints;          /**< Base setpoints from command (before velocity correction) */
    bool velocity_correction_enabled; /**< Enable/disable velocity correction outer loop */
} PID;

/**
 * @brief PID initialization parameters for all five controllers
 *
 * Contains gains, integral limits, and output limits for all axes.
 * Used to configure the PID system during initialization.
 */
typedef struct {
    // Roll axis (right wing down = positive roll)
    float roll_Kp;       /**< Roll proportional gain */
    float roll_Ki;       /**< Roll integral gain */
    float roll_Kd;       /**< Roll derivative gain */
    float roll_Ki_limit; /**< Roll integral windup limit */
    float roll_limit;    /**< Roll output limit (max throttle correction) */

    // Pitch axis (nose up = positive pitch)
    float pitch_Kp;       /**< Pitch proportional gain */
    float pitch_Ki;       /**< Pitch integral gain */
    float pitch_Kd;       /**< Pitch derivative gain */
    float pitch_Ki_limit; /**< Pitch integral windup limit */
    float pitch_limit;    /**< Pitch output limit (max throttle correction) */

    // Yaw axis (clockwise from above = positive yaw)
    float yaw_Kp;       /**< Yaw proportional gain */
    float yaw_Ki;       /**< Yaw integral gain */
    float yaw_Kd;       /**< Yaw derivative gain */
    float yaw_Ki_limit; /**< Yaw integral windup limit */
    float yaw_limit;    /**< Yaw output limit (max throttle correction) */

    // Velocity X (forward/backward position correction)
    float velocity_x_Kp;       /**< Velocity X proportional gain */
    float velocity_x_Ki;       /**< Velocity X integral gain */
    float velocity_x_Kd;       /**< Velocity X derivative gain */
    float velocity_x_Ki_limit; /**< Velocity X integral windup limit */
    float velocity_x_limit;    /**< Velocity X output limit (max pitch setpoint adjustment) */

    // Velocity Y (left/right position correction)
    float velocity_y_Kp;       /**< Velocity Y proportional gain */
    float velocity_y_Ki;       /**< Velocity Y integral gain */
    float velocity_y_Kd;       /**< Velocity Y derivative gain */
    float velocity_y_Ki_limit; /**< Velocity Y integral windup limit */
    float velocity_y_limit;    /**< Velocity Y output limit (max roll setpoint adjustment) */
} PIDCreateInfo;

/**
 * @brief Initialize PID system with gains and limits
 *
 * Configures all five PID controllers (pitch, roll, yaw, velocity_x, velocity_y)
 * with specified gains and limits. Zeros out all internal state (integrals, errors).
 * Enables velocity correction by default.
 *
 * @param pid Pointer to PID structure to initialize
 * @param create_info Pointer to configuration structure with gains and limits
 */
void pid_init(PID *pid, const PIDCreateInfo *create_info);

/**
 * @brief Update attitude PID controllers with current IMU measurements
 *
 * Reads current attitude from IMU and calculates control outputs for pitch, roll,
 * and yaw axes. Uses gyroscope rates for derivative term. Should be called at
 * fixed rate (6660 Hz) after IMU_update().
 *
 * Output is throttle correction in range ±output_limit for each axis:
 * - Positive pitch output: increase rear motors, decrease front motors (nose up)
 * - Positive roll output: increase right motors, decrease left motors (roll right)
 * - Positive yaw output: increase CW motors, decrease CCW motors (yaw right)
 *
 * @param pid Pointer to PID context
 * @param imu Pointer to IMU context with current attitude and gyro data
 * @param dt Time delta since last update (seconds, typically ~0.00015)
 */
void pid_update(PID *pid, const IMU *imu, float dt);

/**
 * @brief Update velocity correction PIDs to adjust attitude setpoints
 *
 * Outer loop controller that adjusts pitch and roll setpoints based on measured
 * horizontal velocity to prevent drift. Uses high-pass filtered acceleration and
 * integrated velocity from IMU.
 *
 * - Velocity X correction adjusts pitch setpoint (forward/back position hold)
 * - Velocity Y correction adjusts roll setpoint (left/right position hold)
 *
 * If velocity_correction_enabled is false, simply copies base_setpoints to setpoints.
 * Should be called before pid_update() in the control loop.
 *
 * @param pid Pointer to PID context
 * @param imu Pointer to IMU context with velocity and acceleration data
 * @param dt Time delta since last update (seconds)
 */
void pid_velocity_correction(PID *pid, const IMU *imu, float dt);

/**
 * @brief Reset all PID controller states
 *
 * Zeros integral accumulators and previous errors for all five PID controllers.
 * Call when transitioning to armed state or after emergency stop to prevent
 * integral windup from carrying over.
 */
void pid_reset(PID *pid);

/**
 * @brief Calculate PID output for a single axis using rate feedback
 *
 * Implements PID control with direct rate feedback for derivative term:
 * - P term: proportional to angle error (setpoint - measurement)
 * - I term: integral of angle error with anti-windup clamping
 * - D term: proportional to negative angular velocity (damping)
 *
 * Angle error is normalized to ±π for correct wraparound handling.
 * Final output is clamped to ±output_limit. Individual terms are stored
 * in PIDController for telemetry.
 *
 * @param pid Pointer to single-axis PID controller
 * @param setpoint Desired angle (radians)
 * @param measurement Current measured angle (radians)
 * @param angular_velocity Current angular velocity (rad/s, from gyroscope)
 * @param dt Time delta since last update (seconds)
 * @return Clamped PID output (throttle correction)
 */
float get_pid_output(PIDController *pid, float setpoint, float measurement,
                     float angular_velocity, float dt);

#endif // PID_H
