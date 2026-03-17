#include "utils.h"
#include "comm.h"
#include "protocol.h"
#include "systick.h"
#include <stdio.h>
#include <string.h>

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

void send_telem(const IMU *imu, const PID *pid) {
    TelemetryPacket packet = {0};
    packet.timestamp_ms = millis();
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
    packet.vel_x = imu->velocity.x;
    packet.vel_y = imu->velocity.y;

    comm_send_frame(BT_TELEM, (const uint8_t *)&packet, sizeof(TelemetryPacket));
}
