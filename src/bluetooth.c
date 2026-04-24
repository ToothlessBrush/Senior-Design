#include "bluetooth.h"
#include "protocol.h"
#include "uart.h"
#include <string.h>

static volatile uint8_t data_received_flag = 0;
static bt_message_t received_message;

static uint8_t crc8_dvb_s2(uint8_t crc, uint8_t a) {
    crc ^= a;
    for (int i = 0; i < 8; i++)
        crc = (crc & 0x80) ? (crc << 1) ^ 0xD5 : (crc << 1);
    return crc;
}

int bt_init(void) {
    uart_init(UART_INSTANCE_2, BT_BAUD);
    return BT_OK;
}

void bt_service(void) {
    typedef enum {
        STATE_SYNC,
        STATE_TYPE,
        STATE_LEN,
        STATE_PAYLOAD,
        STATE_CRC,
    } bt_state_t;

    static bt_state_t state = STATE_SYNC;
    static uint8_t pkt_type = 0;
    static uint8_t pkt_len = 0;
    static uint8_t payload_idx = 0;
    static uint8_t payload_buf[BT_MAX_PAYLOAD];

    while (uart_data_available(UART_INSTANCE_2)) {
        uint8_t c = uart_receive_byte(UART_INSTANCE_2);

        switch (state) {
        case STATE_SYNC:
            if (c == BT_SYNC_BYTE)
                state = STATE_TYPE;
            break;

        case STATE_TYPE:
            pkt_type = c;
            state = STATE_LEN;
            break;

        case STATE_LEN:
            if (c > BT_MAX_PAYLOAD) {
                state = STATE_SYNC; // oversized — discard
            } else {
                pkt_len = c;
                payload_idx = 0;
                state = (pkt_len == 0) ? STATE_CRC : STATE_PAYLOAD;
            }
            break;

        case STATE_PAYLOAD:
            payload_buf[payload_idx++] = c;
            if (payload_idx == pkt_len)
                state = STATE_CRC;
            break;

        case STATE_CRC: {
            uint8_t crc = 0;
            crc = crc8_dvb_s2(crc, pkt_type);
            crc = crc8_dvb_s2(crc, pkt_len);
            for (uint8_t i = 0; i < pkt_len; i++)
                crc = crc8_dvb_s2(crc, payload_buf[i]);

            if (crc == c) {
                received_message.data[0] = pkt_type;
                memcpy(&received_message.data[1], payload_buf, pkt_len);
                received_message.length = 1 + pkt_len;
                data_received_flag = 1;
            }
            state = STATE_SYNC;
            break;
        }
        }
    }
}

int bt_data_available(void) { return data_received_flag; }

bt_message_t *bt_get_received_data(void) { return &received_message; }

void bt_clear_received_flag(void) { data_received_flag = 0; }

int bt_send_frame(uint8_t type, const uint8_t *payload, uint8_t length) {
    uint8_t frame[BT_MAX_PAYLOAD + 4];
    uint8_t crc = 0;
    crc = crc8_dvb_s2(crc, type);
    crc = crc8_dvb_s2(crc, length);
    for (uint8_t i = 0; i < length; i++)
        crc = crc8_dvb_s2(crc, payload[i]);

    frame[0] = BT_SYNC_BYTE;
    frame[1] = type;
    frame[2] = length;
    memcpy(&frame[3], payload, length);
    frame[3 + length] = crc;

    uart_send_data(UART_INSTANCE_2, frame, 4 + length);
    return BT_OK;
}

int bt_send_data(const uint8_t *data, uint8_t length) {
    uart_send_data(UART_INSTANCE_2, data, (uint16_t)length);
    uart_send_string(UART_INSTANCE_2, "\r\n");
    return BT_OK;
}

int bt_send_string(const char *str) {
    uart_send_string(UART_INSTANCE_2, str);
    uart_send_string(UART_INSTANCE_2, "\r\n");
    return BT_OK;
}

int bt_send_string_nb(const char *str) {
    uart_send_string(UART_INSTANCE_2, str);
    uart_send_string(UART_INSTANCE_2, "\r\n");
    return BT_OK;
}

ParsedCommand bt_parse_command(const uint8_t *data, uint8_t length) {
    ParsedCommand cmd = {0};

    if (length < 1)
        return cmd;

    uint8_t type = data[0];
    const uint8_t *payload = &data[1];
    uint8_t payload_len = length - 1;

    switch (type) {
    case BT_CMD_CALIBRATE:
        cmd.type = CMD_CALIBRATE;
        break;

    case BT_CMD_SET_PID:
        if (payload_len == sizeof(CommandSetPid)) {
            cmd.type = CMD_SET_PID;
            memcpy(&cmd.payload.pid, payload, sizeof(CommandSetPid));
        }
        break;

    case BT_CMD_CONFIG:
        if (payload_len == sizeof(CommandConfig)) {
            cmd.type = CMD_CONFIG;
            memcpy(&cmd.payload.config, payload, sizeof(CommandConfig));
        }
        break;

    case BT_CMD_SAVE:
        cmd.type = CMD_SAVE;
        break;

    default:
        cmd.type = CMD_NONE;
        break;
    }

    return cmd;
}
