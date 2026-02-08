#ifndef UTILS_H
#define UTILS_H

#include "imu.h"
#include "pid.h"
#include <stdint.h>

typedef struct {
    int sign;    // 1 for positive, -1 for negative
    int whole;   // whole part (always positive)
    int decimal; // decimal part (always positive)
} float_parts_t;

// Splits a float into sign, whole, and decimal parts for formatted output
void split_float(float value, int decimal_places, float_parts_t *result);

// Send telemetry over LoRa as hex-encoded binary (much faster than sprintf)
// Format: "H:<hex_data>" where hex_data is TelemetryPacket encoded as hex
// string ~10x faster than floating-point sprintf - critical for real-time
// control loop
void send_telem(const IMU *imu, const PID *pid);

#endif // UTILS_H
