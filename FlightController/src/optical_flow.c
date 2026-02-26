#include "optical_flow.h"
#include "uart.h"
#include <string.h>

// Parser state
static parse_state_t parse_state = PARSE_STATE_STX;
static uint8_t payload_buffer[256];
static uint8_t payload_index = 0;
static uint8_t payload_length = 0;
static uint8_t checksum = 0;
static uint8_t sequence = 0;

// Latest sensor data
static optical_flow_data_t sensor_data = {0};

// Debug counters
static uint32_t total_bytes_received = 0;
static uint32_t frames_parsed = 0;
static uint32_t checksum_errors = 0;
static uint32_t header_errors = 0;
static uint8_t last_byte_received = 0;
static uint8_t last_dev_id = 0;
static uint8_t last_sys_id = 0;
static uint8_t last_msg_id = 0;

/**
 * @brief Extract uint32 from buffer (little-endian)
 */
static inline uint32_t extract_uint32(const uint8_t *buf, uint8_t offset) {
    return (uint32_t)buf[offset] | ((uint32_t)buf[offset + 1] << 8) |
           ((uint32_t)buf[offset + 2] << 16) |
           ((uint32_t)buf[offset + 3] << 24);
}

/**
 * @brief Extract int16 from buffer (little-endian)
 */
static inline int16_t extract_int16(const uint8_t *buf, uint8_t offset) {
    return (int16_t)(buf[offset] | (buf[offset + 1] << 8));
}

/**
 * @brief Extract uint16 from buffer (little-endian)
 */
static inline uint16_t extract_uint16(const uint8_t *buf, uint8_t offset) {
    return (uint16_t)(buf[offset] | (buf[offset + 1] << 8));
}

/**
 * @brief Parse payload and update sensor data
 */
static void parse_payload(void) {
    if (payload_length != MICROLINK_MTF01_PAYLOAD_LEN) {
        return; // Invalid payload length
    }

    // Extract all fields from payload (20 bytes total)
    uint8_t offset = 0;

    sensor_data.system_time = extract_uint32(payload_buffer, offset);
    offset += 4;

    sensor_data.distance = extract_uint32(payload_buffer, offset);
    offset += 4;

    sensor_data.distance_strength = payload_buffer[offset++];
    sensor_data.distance_precision = payload_buffer[offset++];
    sensor_data.distance_status = payload_buffer[offset++];
    sensor_data.reserved1 = payload_buffer[offset++];

    sensor_data.flow_vel_x = extract_int16(payload_buffer, offset);
    offset += 2;

    sensor_data.flow_vel_y = extract_int16(payload_buffer, offset);
    offset += 2;

    sensor_data.flow_quality = payload_buffer[offset++];
    sensor_data.flow_status = payload_buffer[offset++];

    sensor_data.reserved2 = extract_uint16(payload_buffer, offset);
    offset += 2;

    sensor_data.sequence = sequence;
    sensor_data.data_valid = 1;
}

/**
 * @brief Reset parser to initial state
 */
static void reset_parser(void) {
    parse_state = PARSE_STATE_STX;
    payload_index = 0;
    payload_length = 0;
    checksum = 0;
}

/**
 * @brief Process a single byte from UART
 */
static void process_byte(uint8_t byte) {
    total_bytes_received++;
    last_byte_received = byte;

    switch (parse_state) {
    case PARSE_STATE_STX:
        if (byte == MICROLINK_STX) {
            checksum = byte;
            parse_state = PARSE_STATE_DEV_ID;
        }
        break;

    case PARSE_STATE_DEV_ID:
        last_dev_id = byte;
        if (byte == MICROLINK_DEV_ID) {
            checksum += byte;
            parse_state = PARSE_STATE_SYS_ID;
        } else {
            header_errors++;
            reset_parser();
        }
        break;

    case PARSE_STATE_SYS_ID:
        last_sys_id = byte;
        if (byte == MICROLINK_SYS_ID) {
            checksum += byte;
            parse_state = PARSE_STATE_MSG_ID;
        } else {
            header_errors++;
            reset_parser();
        }
        break;

    case PARSE_STATE_MSG_ID:
        last_msg_id = byte;
        if (byte == MICROLINK_MSG_ID) {
            checksum += byte;
            parse_state = PARSE_STATE_SEQ;
        } else {
            header_errors++;
            reset_parser();
        }
        break;

    case PARSE_STATE_SEQ:
        sequence = byte;
        checksum += byte;
        parse_state = PARSE_STATE_PAYLOAD_LEN;
        break;

    case PARSE_STATE_PAYLOAD_LEN:
        payload_length = byte;
        checksum += byte;
        payload_index = 0;

        if (payload_length > 0) {
            parse_state = PARSE_STATE_PAYLOAD;
        } else {
            parse_state = PARSE_STATE_CHECKSUM;
        }
        break;

    case PARSE_STATE_PAYLOAD:
        payload_buffer[payload_index++] = byte;
        checksum += byte;

        if (payload_index >= payload_length) {
            parse_state = PARSE_STATE_CHECKSUM;
        }
        break;

    case PARSE_STATE_CHECKSUM:
        // Verify checksum
        if (byte == (checksum & 0xFF)) {
            // Valid frame - parse payload
            parse_payload();
            frames_parsed++;
        } else {
            checksum_errors++;
        }
        // Reset for next frame
        reset_parser();
        break;

    default:
        reset_parser();
        break;
    }
}

void optical_flow_init(void) {
    // Initialize UART6 for optical flow sensor
    uart_init(UART_INSTANCE_6);

    // Initialize sensor data
    memset(&sensor_data, 0, sizeof(sensor_data));

    // Reset parser state
    reset_parser();
}

void optical_flow_update(void) {
    // Process all available bytes from UART6
    while (uart_data_available(UART_INSTANCE_6)) {
        uint8_t byte = uart_receive_byte(UART_INSTANCE_6);
        process_byte(byte);
    }
}

const optical_flow_data_t *optical_flow_get_data(void) { return &sensor_data; }

float optical_flow_calc_velocity(int16_t flow_vel, uint32_t distance) {
    // Check if distance is valid
    if (distance == 0) {
        return 0.0f;
    }

    // Convert distance from mm to m, then calculate velocity
    // speed(cm/s) = flow_vel * distance(m)
    float distance_m = (float)distance / 1000.0f;
    return (float)flow_vel * distance_m;
}

void optical_flow_get_debug_stats(uint32_t *bytes, uint32_t *frames,
                                  uint32_t *chk_err, uint32_t *hdr_err,
                                  uint8_t *state, uint8_t *last_dev,
                                  uint8_t *last_sys, uint8_t *last_msg) {
    if (bytes)
        *bytes = total_bytes_received;
    if (frames)
        *frames = frames_parsed;
    if (chk_err)
        *chk_err = checksum_errors;
    if (hdr_err)
        *hdr_err = header_errors;
    if (state)
        *state = parse_state;
    if (last_dev)
        *last_dev = last_dev_id;
    if (last_sys)
        *last_sys = last_sys_id;
    if (last_msg)
        *last_msg = last_msg_id;
}
