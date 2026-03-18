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
