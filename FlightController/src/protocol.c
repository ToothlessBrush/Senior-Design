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

    return CMD_NONE;
}
