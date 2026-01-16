#include "utils.h"
#include "lora.h"
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

void send_telem(IMU *imu, PID *pid) {
    static int samples = 0;
    static char telem_buffer[200];

    uint32_t now = millis();
    static uint32_t last_send = 0;

    if (now - last_send >= MIN_SEND_INTERVAL) {
        last_send = now;
        float_parts_t roll, pitch, yaw;
        float_parts_t roll_p, roll_i, roll_d;
        float_parts_t pitch_p, pitch_i, pitch_d;
        float_parts_t yaw_p, yaw_i, yaw_d;
        float_parts_t alt, voltage;

        // Split all the floats (3 decimal places for angles and PID terms)
        split_float(imu->attitude.roll, 3, &roll);
        split_float(imu->attitude.pitch, 3, &pitch);
        split_float(imu->attitude.yaw, 3, &yaw);

        split_float(pid->roll_pid.p_term, 3, &roll_p);
        split_float(pid->roll_pid.i_term, 3, &roll_i);
        split_float(pid->roll_pid.d_term, 3, &roll_d);

        split_float(pid->pitch_pid.p_term, 3, &pitch_p);
        split_float(pid->pitch_pid.i_term, 3, &pitch_i);
        split_float(pid->pitch_pid.d_term, 3, &pitch_d);

        split_float(pid->yaw_pid.p_term, 3, &yaw_p);
        split_float(pid->yaw_pid.i_term, 3, &yaw_i);
        split_float(pid->yaw_pid.d_term, 3, &yaw_d);

        // Altitude with 1 decimal place, voltage with 2
        split_float(0.0f, 1, &alt);
        split_float(0.0f, 2, &voltage);

        sprintf(
            telem_buffer,
            "TELEM:%s%d.%03d:%s%d.%03d:%s%d.%03d:%s%d.%03d:%s%d.%03d:%s%d.%03d:"
            "%s%d.%03d:%s%d.%03d:%s%d.%03d:%s%d.%03d:%s%d.%03d:%s%d.%03d:"
            "%s%d.%01d:%s%d.%02d",
            // Roll, pitch, yaw
            roll.sign < 0 ? "-" : "", roll.whole, roll.decimal,
            pitch.sign < 0 ? "-" : "", pitch.whole, pitch.decimal,
            yaw.sign < 0 ? "-" : "", yaw.whole, yaw.decimal,
            // Roll PID terms
            roll_p.sign < 0 ? "-" : "", roll_p.whole, roll_p.decimal,
            roll_i.sign < 0 ? "-" : "", roll_i.whole, roll_i.decimal,
            roll_d.sign < 0 ? "-" : "", roll_d.whole, roll_d.decimal,
            // Pitch PID terms
            pitch_p.sign < 0 ? "-" : "", pitch_p.whole, pitch_p.decimal,
            pitch_i.sign < 0 ? "-" : "", pitch_i.whole, pitch_i.decimal,
            pitch_d.sign < 0 ? "-" : "", pitch_d.whole, pitch_d.decimal,
            // Yaw PID terms
            yaw_p.sign < 0 ? "-" : "", yaw_p.whole, yaw_p.decimal,
            yaw_i.sign < 0 ? "-" : "", yaw_i.whole, yaw_i.decimal,
            yaw_d.sign < 0 ? "-" : "", yaw_d.whole, yaw_d.decimal,
            // Altitude (1 decimal) and voltage (2 decimals)
            alt.sign < 0 ? "-" : "", alt.whole, alt.decimal,
            voltage.sign < 0 ? "-" : "", voltage.whole, voltage.decimal);

        lora_send_string_nb(1, telem_buffer);
        samples = 0;
    }
    samples++;
}

// Send binary telemetry (much more efficient than string format)
void send_telem_binary(IMU *imu, PID *pid, uint8_t state) {
    static uint32_t last_send = 0;
    uint32_t now = millis();

    if (now - last_send >= MIN_SEND_INTERVAL) {
        last_send = now;

        TelemetryPacket packet = {0};

        // Fill in timestamp
        packet.timestamp_ms = now;

        // Fill in attitude (roll, pitch, yaw)
        packet.roll = imu->attitude.roll;
        packet.pitch = imu->attitude.pitch;
        packet.yaw = imu->attitude.yaw;

        // Fill in gyro data (rad/s)
        packet.gyro_x = imu->gyro[0];
        packet.gyro_y = imu->gyro[1];
        packet.gyro_z = imu->gyro[2];

        // Fill in PID outputs
        packet.pid_roll_output = pid->output.roll;
        packet.pid_pitch_output = pid->output.pitch;
        packet.pid_yaw_output = pid->output.yaw;

        // Fill in PID terms for roll
        packet.roll_p_term = pid->roll_pid.p_term;
        packet.roll_i_term = pid->roll_pid.i_term;
        packet.roll_d_term = pid->roll_pid.d_term;

        // Fill in PID terms for pitch
        packet.pitch_p_term = pid->pitch_pid.p_term;
        packet.pitch_i_term = pid->pitch_pid.i_term;
        packet.pitch_d_term = pid->pitch_pid.d_term;

        // Fill in PID terms for yaw
        packet.yaw_p_term = pid->yaw_pid.p_term;
        packet.yaw_i_term = pid->yaw_pid.i_term;
        packet.yaw_d_term = pid->yaw_pid.d_term;

        // Additional telemetry (currently unused)
        packet.altitude = 0.0f;
        packet.battery_volt = 0.0f;

        // State and flags
        packet.state = state;
        packet.flags = 0x00; // TODO: Add status flags (motors armed, IMU ready,
                             // etc.)

        // Send as binary data
        lora_send_data_nb(1, (const uint8_t *)&packet, sizeof(TelemetryPacket));
    }
}
