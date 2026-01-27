#include "protocol.h"
#include <string.h>

// Parse command from received data
// Supports both legacy string commands and new binary commands
CommandType parse_command(const uint8_t *data, uint8_t length) {
    if (length < 1) {
        return CMD_NONE;
    }

    // Legacy string-based command parsing
    // FC:START -> CMD_START
    if (strncmp((const char *)data, "FC:START", 8) == 0) {
        return CMD_START;
    }
    // FC:STOP -> CMD_STOP
    else if (strncmp((const char *)data, "FC:STOP", 7) == 0) {
        return CMD_STOP;
    }
    // FC:EMERGENCY -> CMD_EMERGENCY_STOP
    else if (strncmp((const char *)data, "FC:EMERGENCY", 12) == 0) {
        return CMD_EMERGENCY_STOP;
    }
    // FC:RESET -> CMD_RESET
    else if (strncmp((const char *)data, "FC:RESET", 8) == 0) {
        return CMD_RESET;
    }
    // FC:CALIBRATE -> CMD_CALIBRATE
    else if (strncmp((const char *)data, "FC:CALIBRATE", 12) == 0) {
        return CMD_CALIBRATE;
    }
    // FC:SET_THROTTLE:<value> -> CMD_SET_THROTTLE
    else if (strncmp((const char *)data, "FC:SET_THROTTLE:", 16) == 0) {
        return CMD_SET_THROTTLE;
    }

    return CMD_NONE;
}

// Simple custom atof for bare-metal (avoids newlib syscall dependencies)
static float simple_atof(const char *str) {
    float result = 0.0f;
    float fraction = 0.0f;
    int divisor = 1;
    int sign = 1;
    int in_fraction = 0;

    // Handle negative sign
    if (*str == '-') {
        sign = -1;
        str++;
    } else if (*str == '+') {
        str++;
    }

    // Parse digits
    while (*str) {
        if (*str >= '0' && *str <= '9') {
            if (in_fraction) {
                fraction = fraction * 10.0f + (*str - '0');
                divisor *= 10;
            } else {
                result = result * 10.0f + (*str - '0');
            }
        } else if (*str == '.' && !in_fraction) {
            in_fraction = 1;
        } else {
            break; // Stop on invalid character
        }
        str++;
    }

    result += fraction / (float)divisor;
    return result * sign;
}

// Parse throttle value from FC:SET_THROTTLE:<value> command
// Expected format: FC:SET_THROTTLE:<value> where value is 0.0 to 1.0
// Returns 1 on success, 0 on failure
int parse_throttle_value(const uint8_t *data, uint8_t length,
                         float *throttle_out) {
    if (length < 17) { // Minimum: "FC:SET_THROTTLE:0"
        return 0;
    }

    // Check for correct prefix
    if (strncmp((const char *)data, "FC:SET_THROTTLE:", 16) != 0) {
        return 0;
    }

    // Extract the value part (after the colon)
    const char *value_str = (const char *)data + 16;

    // Validate that we have at least one digit
    if (*value_str < '0' || *value_str > '9') {
        return 0;
    }

    // Convert string to float using custom parser
    float value = simple_atof(value_str);

    // Clamp value to 0.0 - 1.0 range
    if (value < 0.0f) {
        value = 0.0f;
    } else if (value > 1.0f) {
        value = 1.0f;
    }

    *throttle_out = value;
    return 1;
}

int parse_set_point_value(const uint8_t *data, uint8_t length,
                          float *throttle_out) {
    if (length < 17) { // Minimum: "FC:SET_THROTTLE:0"
        return 0;
    }

    // Check for correct prefix
    if (strncmp((const char *)data, "FC:SET_POINT:", 13) != 0) {
        return 0;
    }

    // Extract the value part (after the colon)
    const char *value_str = (const char *)data + 16;

    // Validate that we have at least one digit
    if (*value_str < '0' || *value_str > '9') {
        return 0;
    }

    // Convert string to float using custom parser
    float value = simple_atof(value_str);

    // Clamp value to 0.0 - 1.0 range
    if (value < 0.0f) {
        value = 0.0f;
    } else if (value > 1.0f) {
        value = 1.0f;
    }

    *throttle_out = value;
    return 1;
}
