
#include "crsf.h"
#ifndef UNIT_TEST
#include "systick.h"
#include "uart.h"
#endif
#include <stdint.h>
#include <string.h>

static crsfFrameDef_t crsfFrame;            // frame being assembled
static crsfFrameDef_t crsfChannelDataFrame; // last validated frame

static uint16_t crsfChannelData[CRSF_MAX_CHANNELS];
static float channelScale = CRSF_RC_CHANNEL_SCALE_LEGACY;
static uint8_t crsfPos = 0; // current byte position within frame being assembled
static uint32_t last_frame_ms = 0; // millis() at last valid RC frame

static uint8_t crc8_dvb_s2(uint8_t crc, uint8_t a) {
    crc ^= a;
    for (int i = 0; i < 8; i++)
        crc = (crc & 0x80) ? (crc << 1) ^ 0xD5 : (crc << 1);
    return crc;
}

static uint8_t crsfFrameCRC(void) {
    // covers type + payload; index 0 = address, 1 = frameLength, 2 = type ...
    uint8_t crc = 0;
    for (int i = 2; i < crsfFrame.frame.frameLength + 1; i++)
        crc = crc8_dvb_s2(crc, crsfFrame.bytes[i]);
    return crc;
}

static void crsf_decode_channels_from_frame(void) {
    if (crsfChannelDataFrame.frame.type == CRSF_FRAMETYPE_RC_CHANNELS_PACKED) {
        const crsfPayloadRcChannelsPacked_t *rc =
            (const crsfPayloadRcChannelsPacked_t *)
                crsfChannelDataFrame.frame.payload;
        channelScale = CRSF_RC_CHANNEL_SCALE_LEGACY;
        crsfChannelData[0] = rc->chan0;
        crsfChannelData[1] = rc->chan1;
        crsfChannelData[2] = rc->chan2;
        crsfChannelData[3] = rc->chan3;
        crsfChannelData[4] = rc->chan4;
        crsfChannelData[5] = rc->chan5;
        crsfChannelData[6] = rc->chan6;
        crsfChannelData[7] = rc->chan7;
        crsfChannelData[8] = rc->chan8;
        crsfChannelData[9] = rc->chan9;
        crsfChannelData[10] = rc->chan10;
        crsfChannelData[11] = rc->chan11;
        crsfChannelData[12] = rc->chan12;
        crsfChannelData[13] = rc->chan13;
        crsfChannelData[14] = rc->chan14;
        crsfChannelData[15] = rc->chan15;
    } else {
        // Subset RC frame (0x17) — variable resolution, variable channel count
        uint8_t readByteIndex = 0;
        const uint8_t *payload = crsfChannelDataFrame.frame.payload;

        uint8_t configByte = payload[readByteIndex++];
        uint8_t startChannel =
            configByte & CRSF_SUBSET_RC_STARTING_CHANNEL_MASK;
        configByte >>= CRSF_SUBSET_RC_STARTING_CHANNEL_BITS;

        uint8_t channelBits;
        uint16_t channelMask;
        uint8_t channelRes = configByte & CRSF_SUBSET_RC_RES_CONFIGURATION_MASK;
        configByte >>= CRSF_SUBSET_RC_RES_CONFIGURATION_BITS;

        switch (channelRes) {
        case CRSF_SUBSET_RC_RES_CONF_10B:
            channelBits = CRSF_SUBSET_RC_RES_BITS_10B;
            channelMask = CRSF_SUBSET_RC_RES_MASK_10B;
            channelScale = CRSF_SUBSET_RC_CHANNEL_SCALE_10B;
            break;
        default:
        case CRSF_SUBSET_RC_RES_CONF_11B:
            channelBits = CRSF_SUBSET_RC_RES_BITS_11B;
            channelMask = CRSF_SUBSET_RC_RES_MASK_11B;
            channelScale = CRSF_SUBSET_RC_CHANNEL_SCALE_11B;
            break;
        case CRSF_SUBSET_RC_RES_CONF_12B:
            channelBits = CRSF_SUBSET_RC_RES_BITS_12B;
            channelMask = CRSF_SUBSET_RC_RES_MASK_12B;
            channelScale = CRSF_SUBSET_RC_CHANNEL_SCALE_12B;
            break;
        case CRSF_SUBSET_RC_RES_CONF_13B:
            channelBits = CRSF_SUBSET_RC_RES_BITS_13B;
            channelMask = CRSF_SUBSET_RC_RES_MASK_13B;
            channelScale = CRSF_SUBSET_RC_CHANNEL_SCALE_13B;
            break;
        }

        configByte >>=
            CRSF_SUBSET_RC_RESERVED_CONFIGURATION_BITS; // reserved bit

        uint8_t numOfChannels = ((crsfChannelDataFrame.frame.frameLength -
                                  CRSF_FRAME_LENGTH_TYPE_CRC - 1) *
                                 8) /
                                channelBits;

        uint8_t bitsMerged = 0;
        uint32_t readValue = 0;
        for (uint8_t n = 0; n < numOfChannels; n++) {
            while (bitsMerged < channelBits) {
                uint8_t readByte = payload[readByteIndex++];
                readValue |= ((uint32_t)readByte) << bitsMerged;
                bitsMerged += 8;
            }
            if ((startChannel + n) < CRSF_MAX_CHANNELS)
                crsfChannelData[startChannel + n] = readValue & channelMask;
            readValue >>= channelBits;
            bitsMerged -= channelBits;
        }
    }
}

// ── public API
// ────────────────────────────────────────────────────────────────

void crsf_init(void) {
    uart_init(UART_INSTANCE_1, 420000);
    memset(crsfChannelData, 0, sizeof(crsfChannelData));
    crsfPos = 0;
}

// Feed a single byte into the CRSF frame assembler.
// Call this from crsf_process() or directly in unit tests.
void crsf_feed_byte(uint8_t byte) {
    // Determine expected full frame length once we have byte 1 (frameLength).
    // Full frame = address(1) + frameLength_field(1) + frameLength bytes.
    // frameLength itself includes type + payload + CRC but NOT address/len
    // fields.
    const int fullFrameLength =
        (crsfPos < 3)
            ? 5
            : (int)(crsfFrame.frame.frameLength + CRSF_FRAME_LENGTH_ADDRESS +
                    CRSF_FRAME_LENGTH_FRAMELENGTH);

    // Clamp to max to guard against garbage data
    const int clampedLength = (fullFrameLength > CRSF_FRAME_SIZE_MAX)
                                  ? CRSF_FRAME_SIZE_MAX
                                  : fullFrameLength;

    if (crsfPos == 0) {
        // Only start a frame on a known flight-controller address byte.
        // This filters most mid-stream garbage.
        if (byte != CRSF_ADDRESS_FLIGHT_CONTROLLER &&
            byte != CRSF_ADDRESS_BROADCAST)
            return;
    }

    if (crsfPos < clampedLength)
        crsfFrame.bytes[crsfPos++] = byte;

    if (crsfPos >= clampedLength) {
        // Full frame received — validate CRC
        const uint8_t crc = crsfFrameCRC();
        if (crc == crsfFrame.bytes[clampedLength - 1]) {
            switch (crsfFrame.frame.type) {

            case CRSF_FRAMETYPE_RC_CHANNELS_PACKED:
            case CRSF_FRAMETYPE_SUBSET_RC_CHANNELS_PACKED:
                if (crsfFrame.frame.deviceAddress ==
                    CRSF_ADDRESS_FLIGHT_CONTROLLER) {
                    memcpy(&crsfChannelDataFrame, &crsfFrame,
                           sizeof(crsfFrame));
                    crsf_decode_channels_from_frame();
#ifndef UNIT_TEST
                    last_frame_ms = millis();
#endif
                }
                break;

            default:
                break;
            }
        }
        crsfPos = 0; // reset regardless of CRC outcome
    }
}

// Call from main loop — drains whatever bytes are sitting in the UART RX
// buffer.
void crsf_process(void) {
#ifndef UNIT_TEST
    while (uart_data_available(UART_INSTANCE_1))
        crsf_feed_byte((uint8_t)uart_receive_byte(UART_INSTANCE_1));
#endif
}

uint32_t crsf_signal_age_ms(void) {
#ifndef UNIT_TEST
    if (last_frame_ms == 0)
        return UINT32_MAX;
    return millis() - last_frame_ms;
#else
    return 0;
#endif
}

float crsf_get_channel(uint8_t ch) {
    if (ch >= CRSF_MAX_CHANNELS)
        return 1500.0f; // safe midpoint

    if (channelScale == CRSF_RC_CHANNEL_SCALE_LEGACY) {
        // 0x16 frame: raw 172–1811 → PWM 988–2012 µs
        return (channelScale * (float)crsfChannelData[ch]) + 881.0f;
    } else {
        // 0x17 subset frame
        return (channelScale * (float)crsfChannelData[ch]) + 988.0f;
    }
}
