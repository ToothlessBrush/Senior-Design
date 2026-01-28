#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <stdint.h>

// Command protocol - flight controller commands
typedef enum {
    CMD_NONE,           // No command
    CMD_START,          // Start/arm the flight controller
    CMD_STOP,           // Stop/disarm the flight controller
    CMD_EMERGENCY_STOP, // Emergency stop (immediately stop motors)
    CMD_UPDATE_PID,     // Update PID gains
    CMD_SET_THROTTLE,   // Set base throttle
    CMD_CALIBRATE,      // Calibrate IMU
    CMD_RESET,          // Reset flight controller
    CMD_SET_POINT,      // set the set point of the pid (the goal for angle)
    CMD_HEART_BEAT,
} CommandType;

// Throttle command payload
typedef struct __attribute__((packed)) {
    float value; // 0.0 to 1.0
} CommandSetThrottle;

// Attitude setpoint command payload
typedef struct __attribute__((packed)) {
    float roll;  // radians
    float pitch; // radians
    float yaw;   // radians
} CommandSetPoint;

typedef struct __attribute__((packed)) {
    float base_throttle;
    float roll;
    float pitch;
    float yaw;
} CommandHeartBeat;

// Telemetry packet structure - packed for efficient transmission
typedef struct __attribute__((packed)) {
    uint32_t timestamp_ms; // System timestamp in milliseconds

    // Attitude (radians)
    float roll;
    float pitch;
    float yaw;

    // PID terms for roll
    float roll_p_term;
    float roll_i_term;
    float roll_d_term;

    // PID terms for pitch
    float pitch_p_term;
    float pitch_i_term;
    float pitch_d_term;

    // PID terms for yaw
    float yaw_p_term;
    float yaw_i_term;
    float yaw_d_term;
} TelemetryPacket;

typedef struct {
    CommandType type;
    union {
        CommandSetThrottle throttle;
        CommandSetPoint setpoint;
        CommandHeartBeat heartbeat;
    } payload;
} ParsedCommand;

// Command parsing function
// Returns the command type parsed from the received data
// For string-based commands (legacy), returns the appropriate enum
// For binary commands, directly returns the command byte
ParsedCommand parse_command(const uint8_t *data, uint8_t length);

#endif // PROTOCOL_H
