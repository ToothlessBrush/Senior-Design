/**
 * @file protocol.h
 * @brief Flight controller command and telemetry protocol definitions
 *
 * Defines command and telemetry structures shared between the flight controller
 * and ground station. Communication uses the Bluetooth binary framing protocol
 * defined in bluetooth.h.
 */

#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <stdint.h>

/**
 * @brief Flight controller command types
 *
 * Enumeration of all supported commands that can be sent to the flight
 * controller via Bluetooth binary protocol.
 */
typedef enum {
    CMD_NONE,           /**< No command or parse failure */
    CMD_CALIBRATE,      /**< Calibrate IMU gyro/accel bias */
    CMD_SET_PID,        /**< Tune PID parameters for a single axis */
    CMD_CONFIG,         /**< Sync flight configuration (throttle curve + angle sensitivity) */
    CMD_SAVE,           /**< Save current runtime config to flash */
} CommandType;

/**
 * @brief Set PID parameters command payload
 *
 * Updates PID gains for a specific axis (pitch, roll, yaw, or velocity).
 * Allows in-flight tuning of control parameters.
 */
typedef struct __attribute__((packed)) {
    float P;         /**< Proportional gain */
    float I;         /**< Integral gain */
    float D;         /**< Derivative gain */
    float I_limit;   /**< Integral windup limit */
    float pid_limit; /**< Output limit */
    uint8_t axis;    /**< Axis: 0=pitch, 1=roll, 2=yaw, 3=vel_x, 4=vel_y, 5=vel_z */
} CommandSetPid;

/**
 * @brief Flight configuration: throttle curve and angle sensitivity.
 * PID tuning is handled separately.
 */
typedef struct __attribute__((packed)) {
    float throttle_hover;  /**< Hover throttle point (0.0-1.0, default 0.45) */
    float throttle_expo;   /**< Throttle expo blend (0=linear, 1=full cubic, default 0.6) */
    float max_roll_angle;  /**< Max stick roll angle (radians, default 0.5 ~28 deg) */
    float max_pitch_angle; /**< Max stick pitch angle (radians, default 0.5 ~28 deg) */
    float max_yaw_rate;    /**< Max stick yaw rate (rad/s, default 1.571 ~90 deg/s) */
} CommandConfig;

/**
 * @brief Telemetry packet structure for downlink to ground station
 *
 * Contains current attitude and PID term breakdown for monitoring and
 * debugging. Transmitted back to ground station for display and logging.
 */
typedef struct __attribute__((packed)) {
    uint32_t timestamp_ms; /**< System uptime in milliseconds */

    // Current attitude
    float roll;  /**< Current roll angle (radians) */
    float pitch; /**< Current pitch angle (radians) */
    float yaw;   /**< Current yaw angle (radians) */

    // Roll PID term breakdown
    float roll_p_term; /**< Roll proportional term */
    float roll_i_term; /**< Roll integral term */
    float roll_d_term; /**< Roll derivative term */

    // Pitch PID term breakdown
    float pitch_p_term; /**< Pitch proportional term */
    float pitch_i_term; /**< Pitch integral term */
    float pitch_d_term; /**< Pitch derivative term */

    // Yaw PID term breakdown
    float yaw_p_term; /**< Yaw proportional term */
    float yaw_i_term; /**< Yaw integral term */
    float yaw_d_term; /**< Yaw derivative term */

    // Gyroscope angular velocity (rad/s)
    float gyro_x; /**< Gyro rate X (rad/s) */
    float gyro_y; /**< Gyro rate Y (rad/s) */
    float gyro_z; /**< Gyro rate Z (rad/s) */

    // Velocity estimate (m/s)
    float vel_x; /**< Forward velocity (m/s, +forward) */
    float vel_y; /**< Lateral velocity (m/s, +right) */
    float vel_z; /**< Vertical velocity (m/s, +up) */

    // Height from optical flow distance sensor (m)
    float height; /**< Height above ground (m, from ToF sensor) */

    // Motor throttle outputs (0.0-1.0, after mixing and clamping)
    float motor1; /**< Motor 1 throttle */
    float motor2; /**< Motor 2 throttle */
    float motor3; /**< Motor 3 throttle */
    float motor4; /**< Motor 4 throttle */

    // Commanded setpoints from pilot sticks (for hover-point visualization)
    float input_throttle; /**< Post-curve base throttle (0.0-1.0) */
    float input_roll;     /**< Roll setpoint (radians) */
    float input_pitch;    /**< Pitch setpoint (radians) */
    float input_yaw;      /**< Yaw setpoint (radians, accumulated) */
} TelemetryPacket;

/**
 * @brief Parsed command structure with type and payload
 */
typedef struct {
    CommandType type; /**< Command type determining which payload is valid */
    union {
        CommandSetPid pid;       /**< Valid if type == CMD_SET_PID */
        CommandConfig config;    /**< Valid if type == CMD_CONFIG */
    } payload;
} ParsedCommand;

#endif // PROTOCOL_H
