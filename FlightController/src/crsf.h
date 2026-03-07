#ifndef CRSF_H
#define CRSF_H

#include <stdbool.h>
#include <stdint.h>

// e.g. uart_read_byte(1) reads UART1

#define CRSF_MAX_CHANNELS 16

#define CRSF_ADDRESS_FLIGHT_CONTROLLER 0xC8
#define CRSF_ADDRESS_BROADCAST 0x00
#define CRSF_FRAME_SIZE_MAX 64
#define CRSF_FRAME_LENGTH_ADDRESS 1
#define CRSF_FRAME_LENGTH_FRAMELENGTH 1
#define CRSF_FRAME_LENGTH_TYPE 1
#define CRSF_FRAME_LENGTH_CRC 1
#define CRSF_FRAME_LENGTH_TYPE_CRC 2

#define CRSF_FRAMETYPE_RC_CHANNELS_PACKED 0x16
#define CRSF_FRAMETYPE_SUBSET_RC_CHANNELS_PACKED 0x17

#define CRSF_SUBSET_RC_STARTING_CHANNEL_BITS 5
#define CRSF_SUBSET_RC_STARTING_CHANNEL_MASK 0x1F
#define CRSF_SUBSET_RC_RES_CONFIGURATION_BITS 2
#define CRSF_SUBSET_RC_RES_CONFIGURATION_MASK 0x03
#define CRSF_SUBSET_RC_RESERVED_CONFIGURATION_BITS 1

#define CRSF_SUBSET_RC_RES_CONF_10B 0
#define CRSF_SUBSET_RC_RES_CONF_11B 1
#define CRSF_SUBSET_RC_RES_CONF_12B 2
#define CRSF_SUBSET_RC_RES_CONF_13B 3

#define CRSF_SUBSET_RC_RES_BITS_10B 10
#define CRSF_SUBSET_RC_RES_BITS_11B 11
#define CRSF_SUBSET_RC_RES_BITS_12B 12
#define CRSF_SUBSET_RC_RES_BITS_13B 13

#define CRSF_SUBSET_RC_RES_MASK_10B 0x03FF
#define CRSF_SUBSET_RC_RES_MASK_11B 0x07FF
#define CRSF_SUBSET_RC_RES_MASK_12B 0x0FFF
#define CRSF_SUBSET_RC_RES_MASK_13B 0x1FFF

#define CRSF_RC_CHANNEL_SCALE_LEGACY 0.62477120195241f
#define CRSF_SUBSET_RC_CHANNEL_SCALE_10B 1.26760563380281f
#define CRSF_SUBSET_RC_CHANNEL_SCALE_11B 0.63380281690140f
#define CRSF_SUBSET_RC_CHANNEL_SCALE_12B 0.31690140845070f
#define CRSF_SUBSET_RC_CHANNEL_SCALE_13B 0.15845070422535f

typedef struct {
    uint8_t deviceAddress;
    uint8_t frameLength;
    uint8_t type;
    uint8_t payload[CRSF_FRAME_SIZE_MAX - 4];
} crsfFrame_s;

typedef union {
    uint8_t bytes[CRSF_FRAME_SIZE_MAX];
    crsfFrame_s frame;
} crsfFrameDef_t;

// 0x16 packed channel payload — 11 bits × 16 channels = 22 bytes
typedef struct {
    unsigned int chan0 : 11;
    unsigned int chan1 : 11;
    unsigned int chan2 : 11;
    unsigned int chan3 : 11;
    unsigned int chan4 : 11;
    unsigned int chan5 : 11;
    unsigned int chan6 : 11;
    unsigned int chan7 : 11;
    unsigned int chan8 : 11;
    unsigned int chan9 : 11;
    unsigned int chan10 : 11;
    unsigned int chan11 : 11;
    unsigned int chan12 : 11;
    unsigned int chan13 : 11;
    unsigned int chan14 : 11;
    unsigned int chan15 : 11;
} __attribute__((__packed__)) crsfPayloadRcChannelsPacked_t;

void crsf_init(void);
void crsf_process(void); // Call from main loop — drains UART RX buffer
float crsf_get_channel(uint8_t ch); // Returns PWM µs value (988–2012)

#endif // CRSF_H
