#include "protocol.h"

/** checks if the data starts with a prefix
 */
static int starts_with(const uint8_t *data, uint8_t len, const char *prefix) {
    for (uint8_t i = 0; prefix[i]; i++) {
        if (i >= len || data[i] != prefix[i])
            return 0;
    }
    return 1;
}

// Convert hex char to nibble value
static uint8_t hex_nibble(uint8_t c) {
    if (c >= '0' && c <= '9')
        return c - '0';
    if (c >= 'A' && c <= 'F')
        return c - 'A' + 10;
    if (c >= 'a' && c <= 'f')
        return c - 'a' + 10;
    return 0;
}

// Convert hex string to bytes, returns number of bytes written
static uint8_t hex_to_bytes(const uint8_t *hex, uint8_t hex_len, uint8_t *out,
                            uint8_t max_out) {
    uint8_t count = 0;
    for (uint8_t i = 0; i + 1 < hex_len && count < max_out; i += 2) {
        out[count++] = (hex_nibble(hex[i]) << 4) | hex_nibble(hex[i + 1]);
    }
    return count;
}

// Parse command from received data
// Supports both legacy string commands and new binary commands
ParsedCommand parse_command(const uint8_t *data, uint8_t length) {
    ParsedCommand cmd = {.type = CMD_NONE};

    if (starts_with(data, length, "FC:START")) {
        cmd.type = CMD_START;
        return cmd;
    }
    if (starts_with(data, length, "FC:STOP")) {
        cmd.type = CMD_STOP;
        return cmd;
    }
    if (starts_with(data, length, "FC:EMERGENCY")) {
        cmd.type = CMD_EMERGENCY_STOP;
        return cmd;
    }
    if (starts_with(data, length, "FC:CALIBRATE")) {
        cmd.type = CMD_CALIBRATE;
        return cmd;
    }
    if (starts_with(data, length, "FC:RESET")) {
        cmd.type = CMD_RESET;
        return cmd;
    }

    // ST:<hex> - throttle
    if (starts_with(data, length, "ST:") && length >= 11) { // 3 + 8 hex
        cmd.type = CMD_SET_THROTTLE;
        hex_to_bytes(&data[3], 8, (uint8_t *)&cmd.payload.throttle, 4);
    }
    // SP:<hex> - setpoint
    else if (starts_with(data, length, "SP:") && length >= 27) { // 3 + 24 hex
        cmd.type = CMD_SET_POINT;
        hex_to_bytes(&data[3], 24, (uint8_t *)&cmd.payload.setpoint, 12);
    }

    else if (starts_with(data, length, "HB:") && length >= 35) { // 3 + 32 hex
        cmd.type = CMD_HEART_BEAT,
        hex_to_bytes(&data[3], 24, (uint8_t *)&cmd.payload.heartbeat, 16);
    }

    return cmd;
}
