#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <stdint.h>

// Command protocol - flight controller commands
typedef enum {
    CMD_NONE = 0x00,           // No command
    CMD_START = 0x01,          // Start/arm the flight controller
    CMD_STOP = 0x02,           // Stop/disarm the flight controller
    CMD_EMERGENCY_STOP = 0x03, // Emergency stop (immediately stop motors)
    CMD_UPDATE_PID = 0x04,     // Update PID gains
    CMD_SET_THROTTLE = 0x05,   // Set base throttle
    CMD_SET_ATTITUDE = 0x06,   // Set target attitude (roll/pitch/yaw setpoints)
    CMD_CALIBRATE = 0x07,      // Calibrate IMU
    CMD_RESET = 0x08,          // Reset flight controller
} CommandType;

// Command structure with payload data
typedef struct __attribute__((packed)) {
    uint8_t command;  // CommandType enum value
    uint8_t reserved; // Reserved for future use / alignment
} Command;

// PID update command payload
typedef struct __attribute__((packed)) {
    uint8_t command; // CMD_UPDATE_PID
    uint8_t axis;    // 0=roll, 1=pitch, 2=yaw
    float kp;
    float ki;
    float kd;
} CommandUpdatePID;

// Throttle command payload
typedef struct __attribute__((packed)) {
    uint8_t command; // CMD_SET_THROTTLE
    uint8_t reserved;
    float throttle; // 0.0 to 1.0
} CommandSetThrottle;

// Attitude setpoint command payload
typedef struct __attribute__((packed)) {
    uint8_t command; // CMD_SET_ATTITUDE
    uint8_t reserved;
    float roll;  // radians
    float pitch; // radians
    float yaw;   // radians
} CommandSetAttitude;

// Telemetry packet structure - packed for efficient transmission
typedef struct __attribute__((packed)) {
    uint32_t timestamp_ms; // System timestamp in milliseconds

    // Attitude (radians)
    float roll;
    float pitch;
    float yaw;

    // Angular velocities (rad/s)
    float gyro_x;
    float gyro_y;
    float gyro_z;

    // PID outputs (normalized 0-1)
    float pid_roll_output;
    float pid_pitch_output;
    float pid_yaw_output;

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

    // Additional telemetry
    float altitude;     // meters (currently unused, set to 0)
    float battery_volt; // volts (currently unused, set to 0)

    uint8_t state; // Flight controller state
    uint8_t flags; // Status flags (bit 0: motors armed, bit 1: IMU ready, etc.)

} TelemetryPacket;

// Command parsing function
// Returns the command type parsed from the received data
// For string-based commands (legacy), returns the appropriate enum
// For binary commands, directly returns the command byte
CommandType parse_command(const uint8_t *data, uint8_t length);

// Parse throttle value from FC:SET_THROTTLE:<value> string command
// Returns 1 on success, 0 on failure
// throttle_out will be set to the parsed value (0.0 to 1.0)
int parse_throttle_value(const uint8_t *data, uint8_t length, float *throttle_out);

// Helper to check if received data is a binary command (vs string)
int is_binary_command(const uint8_t *data, uint8_t length);

#endif // PROTOCOL_H
