#include "utils.h"
#include "lora.h"
#include "protocol.h"
#include "system.h"
#include "systick.h"
#include <stdio.h>

void split_float(float value, int decimal_places, float_parts_t *result) {
    // Determine sign
    result->sign = (value < 0.0f) ? -1 : 1;

    // Work with absolute value
    float abs_value = (value < 0.0f) ? -value : value;

    // Extract whole part
    result->whole = (int)abs_value;

    // Extract decimal part
    float fractional = abs_value - (float)result->whole;

    // Scale up by 10^decimal_places and round
    int multiplier = 1;
    for (int i = 0; i < decimal_places; i++) {
        multiplier *= 10;
    }
    result->decimal = (int)(fractional * multiplier + 0.5f);

    // Handle rounding edge case (e.g., 1.999 with 2 decimals -> 2.00)
    if (result->decimal >= multiplier) {
        result->whole++;
        result->decimal -= multiplier;
    }
}

// Fast byte-to-hex conversion (no sprintf needed)
static inline void byte_to_hex(uint8_t byte, char *out) {
    static const char hex[] = "0123456789ABCDEF";
    out[0] = hex[byte >> 4];
    out[1] = hex[byte & 0x0F];
}

void send_telem(IMU *imu, PID *pid) {
    static char hex_buffer[200]; // Buffer for hex-encoded data

    uint32_t now = millis();
    static uint32_t last_send = 0;

    if (now - last_send >= MIN_SEND_INTERVAL) {
        last_send = now;

        // Pack data into binary struct (reuse TelemetryPacket)
        TelemetryPacket packet = {0};
        packet.timestamp_ms = now;
        packet.roll = imu->attitude.roll;
        packet.pitch = imu->attitude.pitch;
        packet.yaw = imu->attitude.yaw;
        packet.roll_p_term = pid->roll_pid.p_term;
        packet.roll_i_term = pid->roll_pid.i_term;
        packet.roll_d_term = pid->roll_pid.d_term;
        packet.pitch_p_term = pid->pitch_pid.p_term;
        packet.pitch_i_term = pid->pitch_pid.i_term;
        packet.pitch_d_term = pid->pitch_pid.d_term;
        packet.yaw_p_term = pid->yaw_pid.p_term;
        packet.yaw_i_term = pid->yaw_pid.i_term;
        packet.yaw_d_term = pid->yaw_pid.d_term;

        // Convert to hex string (fast - just table lookups and bit shifts)
        const uint8_t *data = (const uint8_t *)&packet;
        char *hex_ptr = hex_buffer;

        // Add header
        *hex_ptr++ = 'T';
        *hex_ptr++ = ':';

        // Convert each byte to 2 hex characters
        for (size_t i = 0; i < sizeof(TelemetryPacket); i++) {
            byte_to_hex(data[i], hex_ptr);
            hex_ptr += 2;
        }

        *hex_ptr = '\0'; // Null terminate

        lora_send_string_nb(1, hex_buffer);
    }
}
