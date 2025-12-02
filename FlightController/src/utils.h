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

// Send telemetry data over LoRa
void send_telem(IMU *imu, PID *pid);

#endif // UTILS_H
