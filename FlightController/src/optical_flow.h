/**
 * @file optical_flow.h
 * @brief MTF-01 optical flow sensor driver using Microlink protocol
 *
 * Parses Microlink frames from the MTF-01 optical flow sensor over UART6.
 * The sensor provides distance measurement and optical flow velocity data.
 *
 * Hardware Configuration:
 * - UART6 on PA11/PA12 at 115200 baud
 * - Microlink protocol frame format
 *
 * Frame Structure:
 * [STX][DEV_ID][SYS_ID][MSG_ID][SEQ][PAYLOAD_LEN][PAYLOAD][CHECKSUM]
 *
 * Features:
 * - Distance measurement (mm)
 * - Optical flow velocity (cm/s @ 1m)
 * - Quality and status indicators
 * - Checksum validation
 */

#ifndef OPTICAL_FLOW_H
#define OPTICAL_FLOW_H

#include <stdint.h>

// Microlink protocol constants
#define MICROLINK_STX 0xEF
#define MICROLINK_DEV_ID 0x0F
#define MICROLINK_SYS_ID 0x00
#define MICROLINK_MSG_ID 0x51
#define MICROLINK_MTF01_PAYLOAD_LEN 0x14 // 20 bytes

// Frame parsing states
typedef enum {
    PARSE_STATE_STX,
    PARSE_STATE_DEV_ID,
    PARSE_STATE_SYS_ID,
    PARSE_STATE_MSG_ID,
    PARSE_STATE_SEQ,
    PARSE_STATE_PAYLOAD_LEN,
    PARSE_STATE_PAYLOAD,
    PARSE_STATE_CHECKSUM
} parse_state_t;

#ifdef UNIT_TEST
void optical_flow_feed_byte(uint8_t byte);
#endif

/**
 * @brief MTF-01 sensor data structure (20-byte payload)
 */
typedef struct {
    uint32_t system_time;       /**< System time in milliseconds */
    uint32_t distance;          /**< Distance in mm (min=1, 0=invalid) */
    uint8_t distance_strength;  /**< Distance strength (0-255) */
    uint8_t distance_precision; /**< Distance precision (0-255, lower=better) */
    uint8_t distance_status;    /**< Distance status (1=valid, 0=invalid) */
    uint8_t reserved1;          /**< Reserved byte */
    int16_t flow_vel_x;         /**< Flow velocity X (cm/s @ 1m) */
    int16_t flow_vel_y;         /**< Flow velocity Y (cm/s @ 1m) */
    uint8_t flow_quality;       /**< Flow quality (0-255, higher=better) */
    uint8_t flow_status;        /**< Flow status (1=valid, 0=invalid) */
    uint16_t reserved2;         /**< Reserved 2 bytes */
    uint8_t sequence;           /**< Frame sequence number */
    uint8_t data_valid;         /**< Set to 1 when new valid data received */
} optical_flow_data_t;

/**
 * @brief Initialize optical flow sensor communication
 *
 * Initializes UART6 for communication with the MTF-01 sensor.
 * Must be called before using any other optical flow functions.
 */
void optical_flow_init(void);

/**
 * @brief Update optical flow sensor data (call in main loop)
 *
 * Processes incoming UART data and parses Microlink frames.
 * Call this regularly (e.g., in main loop) to update sensor data.
 * Non-blocking - processes available bytes only.
 */
void optical_flow_update(void);

/**
 * @brief Get latest optical flow sensor data
 *
 * Returns pointer to internal data structure containing the latest
 * valid sensor readings. Check data_valid flag before using.
 *
 * @return Pointer to optical_flow_data_t structure
 */
const optical_flow_data_t *optical_flow_get_data(void);

/**
 * @brief Calculate actual velocity from flow velocity and distance
 *
 * Converts flow velocity (cm/s @ 1m) to actual velocity (cm/s) using
 * the formula: speed(cm/s) = flow_vel * distance(m)
 *
 * @param flow_vel Flow velocity from sensor (cm/s @ 1m)
 * @param distance Distance from sensor (mm)
 * @return Actual velocity in cm/s (returns 0 if distance invalid)
 */
float optical_flow_calc_velocity(int16_t flow_vel, uint32_t distance);

/**
 * @brief Check if distance data is valid
 *
 * @param data Pointer to optical flow data
 * @return 1 if distance valid, 0 otherwise
 */
static inline uint8_t
optical_flow_is_distance_valid(const optical_flow_data_t *data) {
    return (data->distance_status == 1 && data->distance > 0);
}

/**
 * @brief Check if flow data is valid
 *
 * @param data Pointer to optical flow data
 * @return 1 if flow valid, 0 otherwise
 */
static inline uint8_t
optical_flow_is_flow_valid(const optical_flow_data_t *data) {
    return (data->flow_status == 1);
}

/**
 * @brief Get debug statistics from the parser
 *
 * @param bytes Total bytes received from UART
 * @param frames Number of valid frames parsed
 * @param chk_err Number of checksum errors
 * @param hdr_err Number of header validation errors
 * @param state Current parser state
 * @param last_dev Last DEV_ID byte received
 * @param last_sys Last SYS_ID byte received
 * @param last_msg Last MSG_ID byte received
 */
void optical_flow_get_debug_stats(uint32_t *bytes, uint32_t *frames,
                                  uint32_t *chk_err, uint32_t *hdr_err,
                                  uint8_t *state, uint8_t *last_dev,
                                  uint8_t *last_sys, uint8_t *last_msg);

#endif // OPTICAL_FLOW_H
