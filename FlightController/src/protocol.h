/**
 * @file protocol.h
 * @brief Flight controller command and telemetry protocol definitions
 *
 * Defines the wire protocol for communication between ground control station
 * and flight controller. Supports both text-based command prefixes (FC:, ES)
 * and hex-encoded binary payloads (ST:, SP:, HB:, etc.) for efficient transmission
 * over LoRa links.
 *
 * Protocol Format:
 * - Text commands: "FC:START", "FC:STOP", "ES" (emergency stop)
 * - Binary commands: "PREFIX:HEXDATA" where HEXDATA is hex-encoded binary struct
 *   - ST: Set throttle (8 hex chars = 4 bytes float)
 *   - SP: Set point (24 hex = 12 bytes, 3 floats)
 *   - HB: Heartbeat (32 hex = 16 bytes, 4 floats)
 *   - TP: Tune PID (42 hex = 21 bytes, 5 floats + 1 uint8_t)
 *   - MB: Motor bias (32 hex = 16 bytes, 4 floats)
 *   - CF: Config sync (152 hex = 76 bytes, full configuration)
 */

#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <stdint.h>

/**
 * @brief Flight controller command types
 *
 * Enumeration of all supported commands that can be sent to the flight
 * controller. Commands are parsed from received LoRa messages.
 */
typedef enum {
    CMD_NONE,           /**< No command or parse failure */
    CMD_START,          /**< Start/arm for autonomous flight (text: "FC:START") */
    CMD_STOP,           /**< Stop/disarm motors (text: "FC:STOP") */
    CMD_EMERGENCY_STOP, /**< Emergency stop - immediate motor shutdown (text: "ES") */
    CMD_UPDATE_PID,     /**< Update PID gains (deprecated) */
    CMD_SET_THROTTLE,   /**< Set base throttle (binary: "ST:XXXXXXXX") */
    CMD_CALIBRATE,      /**< Calibrate IMU gyro/accel bias (text: "FC:CALIBRATE") */
    CMD_RESET,          /**< Reset from emergency stop to disarmed (text: "FC:RESET") */
    CMD_SET_POINT,      /**< Set attitude setpoint (binary: "SP:XXXXXXXXXXXXXXXXXXXXXXXX") */
    CMD_HEART_BEAT,     /**< Heartbeat with throttle + setpoint (binary: "HB:...") */
    CMD_SET_PID,        /**< Tune PID parameters (binary: "TP:...") */
    CMD_SET_MOTOR_BIAS, /**< Set per-motor trim values (binary: "MB:...") */
    CMD_START_MANUAL,   /**< Start manual control mode (text: "FC:MANUAL") */
    CMD_CONFIG,         /**< Sync full configuration (binary: "CF:...") */
} CommandType;

/**
 * @brief Set throttle command payload (ST: command)
 *
 * Sets the base throttle level for all motors. PID corrections are added
 * to this base value.
 */
typedef struct __attribute__((packed)) {
    float value; /**< Throttle level (0.0 to 1.0) */
} CommandSetThrottle;

/**
 * @brief Set attitude setpoint command payload (SP: command)
 *
 * Sets the target attitude for PID control loops. All angles in radians.
 */
typedef struct __attribute__((packed)) {
    float roll;  /**< Target roll angle (radians, + right wing down) */
    float pitch; /**< Target pitch angle (radians, + nose up) */
    float yaw;   /**< Target yaw angle (radians, + clockwise from above) */
} CommandSetPoint;

/**
 * @brief Heartbeat command payload (HB: command)
 *
 * Combined heartbeat and control update sent periodically by ground station.
 * Includes both throttle and attitude setpoints. Timeout triggers emergency stop.
 */
typedef struct __attribute__((packed)) {
    float base_throttle; /**< Base throttle (0.0-1.0) */
    float roll;          /**< Target roll (radians) */
    float pitch;         /**< Target pitch (radians) */
    float yaw;           /**< Target yaw (radians) */
} CommandHeartBeat;

/**
 * @brief Set PID parameters command payload (TP: command)
 *
 * Updates PID gains for a specific axis (pitch, roll, yaw, or velocity).
 * Allows in-flight tuning of control parameters.
 */
typedef struct __attribute__((packed)) {
    float P;          /**< Proportional gain */
    float I;          /**< Integral gain */
    float D;          /**< Derivative gain */
    float I_limit;    /**< Integral windup limit */
    float pid_limit;  /**< Output limit */
    uint8_t axis;     /**< Axis: 0=pitch, 1=roll, 2=yaw, 3=vel_x, 4=vel_y */
} CommandSetPid;

/**
 * @brief Set motor bias command payload (MB: command)
 *
 * Individual motor trim values to compensate for manufacturing variations
 * or imbalanced weight distribution. Values are added to final motor throttle.
 */
typedef struct __attribute__((packed)) {
    float motor1; /**< Motor 1 bias (rear left, CW) */
    float motor2; /**< Motor 2 bias (front right, CCW) */
    float motor3; /**< Motor 3 bias (rear right, CCW) */
    float motor4; /**< Motor 4 bias (front left, CW) */
} CommandMotorBias;

/**
 * @brief Configuration sync command payload (CF: command)
 *
 * Full configuration synchronization from ground station to flight controller.
 * Contains all motor biases and PID parameters for all five control axes.
 * Transmitted as 152 hex characters (76 bytes).
 */
typedef struct __attribute__((packed)) {
    // Motor trim values (0.0-1.0)
    float motor1;  /**< Motor 1 bias */
    float motor2;  /**< Motor 2 bias */
    float motor3;  /**< Motor 3 bias */
    float motor4;  /**< Motor 4 bias */

    // PID parameters for roll axis
    float roll_Kp;         /**< Roll proportional gain */
    float roll_Ki;         /**< Roll integral gain */
    float roll_Kd;         /**< Roll derivative gain */
    float roll_i_limit;    /**< Roll integral limit */
    float roll_pid_limit;  /**< Roll output limit */

    // PID parameters for pitch axis
    float pitch_Kp;        /**< Pitch proportional gain */
    float pitch_Ki;        /**< Pitch integral gain */
    float pitch_Kd;        /**< Pitch derivative gain */
    float pitch_i_limit;   /**< Pitch integral limit */
    float pitch_pid_limit; /**< Pitch output limit */

    // PID parameters for yaw axis
    float yaw_Kp;          /**< Yaw proportional gain */
    float yaw_Ki;          /**< Yaw integral gain */
    float yaw_Kd;          /**< Yaw derivative gain */
    float yaw_i_limit;     /**< Yaw integral limit */
    float yaw_pid_limit;   /**< Yaw output limit */

    // PID parameters for velocity X (forward/backward position correction)
    float velocity_x_Kp;        /**< Velocity X proportional gain */
    float velocity_x_Ki;        /**< Velocity X integral gain */
    float velocity_x_Kd;        /**< Velocity X derivative gain */
    float velocity_x_i_limit;   /**< Velocity X integral limit */
    float velocity_x_pid_limit; /**< Velocity X output limit (max pitch correction) */

    // PID parameters for velocity Y (left/right position correction)
    float velocity_y_Kp;        /**< Velocity Y proportional gain */
    float velocity_y_Ki;        /**< Velocity Y integral gain */
    float velocity_y_Kd;        /**< Velocity Y derivative gain */
    float velocity_y_i_limit;   /**< Velocity Y integral limit */
    float velocity_y_pid_limit; /**< Velocity Y output limit (max roll correction) */
} CommandConfig;

/**
 * @brief Telemetry packet structure for downlink to ground station
 *
 * Contains current attitude and PID term breakdown for monitoring and debugging.
 * Transmitted back to ground station for display and logging.
 */
typedef struct __attribute__((packed)) {
    uint32_t timestamp_ms; /**< System uptime in milliseconds */

    // Current attitude
    float roll;   /**< Current roll angle (radians) */
    float pitch;  /**< Current pitch angle (radians) */
    float yaw;    /**< Current yaw angle (radians) */

    // Roll PID term breakdown
    float roll_p_term;  /**< Roll proportional term */
    float roll_i_term;  /**< Roll integral term */
    float roll_d_term;  /**< Roll derivative term */

    // Pitch PID term breakdown
    float pitch_p_term; /**< Pitch proportional term */
    float pitch_i_term; /**< Pitch integral term */
    float pitch_d_term; /**< Pitch derivative term */

    // Yaw PID term breakdown
    float yaw_p_term;   /**< Yaw proportional term */
    float yaw_i_term;   /**< Yaw integral term */
    float yaw_d_term;   /**< Yaw derivative term */
} TelemetryPacket;

/**
 * @brief Parsed command structure with type and payload
 *
 * Result of parse_command(). Contains command type and a union of all
 * possible command payloads. Only the field corresponding to the command
 * type contains valid data.
 */
typedef struct {
    CommandType type; /**< Command type determining which payload is valid */
    union {
        CommandSetThrottle throttle; /**< Valid if type == CMD_SET_THROTTLE */
        CommandSetPoint setpoint;    /**< Valid if type == CMD_SET_POINT */
        CommandHeartBeat heartbeat;  /**< Valid if type == CMD_HEART_BEAT */
        CommandSetPid pid;           /**< Valid if type == CMD_SET_PID */
        CommandMotorBias bias;       /**< Valid if type == CMD_SET_MOTOR_BIAS */
        CommandConfig sync;          /**< Valid if type == CMD_CONFIG */
    } payload;
} ParsedCommand;

/**
 * @brief Parse received command data into structured format
 *
 * Identifies command type from prefix and decodes hex-encoded binary payloads.
 * Supports both text commands ("FC:START", "ES") and binary commands with
 * hex-encoded struct data ("ST:XXXXXXXX", "HB:...", etc.).
 *
 * Text Commands:
 * - "ES" -> CMD_EMERGENCY_STOP
 * - "FC:START" -> CMD_START
 * - "FC:STOP" -> CMD_STOP
 * - "FC:MANUAL" -> CMD_START_MANUAL
 * - "FC:CALIBRATE" -> CMD_CALIBRATE
 * - "FC:RESET" -> CMD_RESET
 *
 * Binary Commands (hex-encoded):
 * - "ST:XXXXXXXX" (8 hex) -> CMD_SET_THROTTLE
 * - "SP:XXXXXXXXXXXXXXXXXXXXXXXX" (24 hex) -> CMD_SET_POINT
 * - "HB:XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX" (32 hex) -> CMD_HEART_BEAT
 * - "TP:XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX" (42 hex) -> CMD_SET_PID
 * - "MB:XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX" (32 hex) -> CMD_SET_MOTOR_BIAS
 * - "CF:..." (152 hex) -> CMD_CONFIG
 *
 * @param data Pointer to received command bytes
 * @param length Length of received data
 * @return ParsedCommand with type set and payload decoded (if applicable)
 */
ParsedCommand parse_command(const uint8_t *data, uint8_t length);

#endif // PROTOCOL_H
