/**
 * Host-native unit tests for the CRSF frame parser.
 *
 * Build:  cd tests/host && make
 * Run:    ./test_crsf
 *
 * Compiled with -DUNIT_TEST so crsf.c omits the uart.h include and the
 * crsf_process() body, leaving crsf_feed_byte() fully testable without any
 * STM32 hardware or UART driver.
 */

#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "crsf.h"

// ── CRC8 DVB-S2 lookup table (polynomial 0xD5, from CRSF spec) ───────────────

static const uint8_t crc8tab[256] = {
    0x00, 0xD5, 0x7F, 0xAA, 0xFE, 0x2B, 0x81, 0x54, 0x29, 0xFC, 0x56, 0x83,
    0xD7, 0x02, 0xA8, 0x7D, 0x52, 0x87, 0x2D, 0xF8, 0xAC, 0x79, 0xD3, 0x06,
    0x7B, 0xAE, 0x04, 0xD1, 0x85, 0x50, 0xFA, 0x2F, 0xA4, 0x71, 0xDB, 0x0E,
    0x5A, 0x8F, 0x25, 0xF0, 0x8D, 0x58, 0xF2, 0x27, 0x73, 0xA6, 0x0C, 0xD9,
    0xF6, 0x23, 0x89, 0x5C, 0x08, 0xDD, 0x77, 0xA2, 0xDF, 0x0A, 0xA0, 0x75,
    0x21, 0xF4, 0x5E, 0x8B, 0x9D, 0x48, 0xE2, 0x37, 0x63, 0xB6, 0x1C, 0xC9,
    0xB4, 0x61, 0xCB, 0x1E, 0x4A, 0x9F, 0x35, 0xE0, 0xCF, 0x1A, 0xB0, 0x65,
    0x31, 0xE4, 0x4E, 0x9B, 0xE6, 0x33, 0x99, 0x4C, 0x18, 0xCD, 0x67, 0xB2,
    0x39, 0xEC, 0x46, 0x93, 0xC7, 0x12, 0xB8, 0x6D, 0x10, 0xC5, 0x6F, 0xBA,
    0xEE, 0x3B, 0x91, 0x44, 0x6B, 0xBE, 0x14, 0xC1, 0x95, 0x40, 0xEA, 0x3F,
    0x42, 0x97, 0x3D, 0xE8, 0xBC, 0x69, 0xC3, 0x16, 0xEF, 0x3A, 0x90, 0x45,
    0x11, 0xC4, 0x6E, 0xBB, 0xC6, 0x13, 0xB9, 0x6C, 0x38, 0xED, 0x47, 0x92,
    0xBD, 0x68, 0xC2, 0x17, 0x43, 0x96, 0x3C, 0xE9, 0x94, 0x41, 0xEB, 0x3E,
    0x6A, 0xBF, 0x15, 0xC0, 0x4B, 0x9E, 0x34, 0xE1, 0xB5, 0x60, 0xCA, 0x1F,
    0x62, 0xB7, 0x1D, 0xC8, 0x9C, 0x49, 0xE3, 0x36, 0x19, 0xCC, 0x66, 0xB3,
    0xE7, 0x32, 0x98, 0x4D, 0x30, 0xE5, 0x4F, 0x9A, 0xCE, 0x1B, 0xB1, 0x64,
    0x72, 0xA7, 0x0D, 0xD8, 0x8C, 0x59, 0xF3, 0x26, 0x5B, 0x8E, 0x24, 0xF1,
    0xA5, 0x70, 0xDA, 0x0F, 0x20, 0xF5, 0x5F, 0x8A, 0xDE, 0x0B, 0xA1, 0x74,
    0x09, 0xDC, 0x76, 0xA3, 0xF7, 0x22, 0x88, 0x5D, 0xD6, 0x03, 0xA9, 0x7C,
    0x28, 0xFD, 0x57, 0x82, 0xFF, 0x2A, 0x80, 0x55, 0x01, 0xD4, 0x7E, 0xAB,
    0x84, 0x51, 0xFB, 0x2E, 0x7A, 0xAF, 0x05, 0xD0, 0xAD, 0x78, 0xD2, 0x07,
    0x53, 0x86, 0x2C, 0xF9,
};

static uint8_t compute_crc(const uint8_t *data, int len) {
    uint8_t crc = 0;
    for (int i = 0; i < len; i++)
        crc = crc8tab[crc ^ data[i]];
    return crc;
}

// Pack 16 11-bit channel values into a 22-byte CRSF payload (LSB-first).
static void pack_rc_channels(uint8_t *payload, const uint16_t *ch) {
    memset(payload, 0, 22);
    for (int i = 0; i < 16; i++) {
        int bitPos = i * 11;
        uint16_t val = ch[i];
        for (int b = 0; b < 11; b++) {
            if (val & (1u << b))
                payload[bitPos / 8] |= (uint8_t)(1u << (bitPos % 8));
            bitPos++;
        }
    }
}

// Build a complete 26-byte CRSF RC_CHANNELS_PACKED frame with correct CRC.
//   frame[0]    = 0xC8  (address: flight controller)
//   frame[1]    = 0x18  (frameLength = 24: type + 22-byte payload + CRC)
//   frame[2]    = 0x16  (type: RC_CHANNELS_PACKED)
//   frame[3..24]= 22-byte packed channel payload
//   frame[25]   = CRC8 over frame[2..24]
static void build_rc_frame(uint8_t *frame, const uint16_t *channels) {
    uint8_t payload[22];
    pack_rc_channels(payload, channels);
    frame[0] = CRSF_ADDRESS_FLIGHT_CONTROLLER;
    frame[1] = 0x18; // frameLength = 24 (1 type + 22 payload + 1 CRC)
    frame[2] = CRSF_FRAMETYPE_RC_CHANNELS_PACKED;
    memcpy(frame + 3, payload, 22);
    frame[25] = compute_crc(frame + 2, 23); // CRC over type + payload
}

static void print_channels(void) {
    printf("Channels:\n");
    for (int i = 0; i < 16; i++)
        printf("  CH%02d = %.2f us\n", i, crsf_get_channel(i));
    printf("\n");
}

static void feed_frame(const uint8_t *frame, int len) {
    for (int i = 0; i < len; i++)
        crsf_feed_byte(frame[i]);
}

// Simple test runner helpers — no external framework needed.
static int g_failures = 0;

#define CHECK_NEAR(name, got, expected, tol)                                   \
    do {                                                                       \
        float _g = (got), _e = (expected), _d = _g - _e;                       \
        if (_d < 0.0f)                                                         \
            _d = -_d;                                                          \
        if (_d <= (tol)) {                                                     \
            printf("  PASS  %s  (%.3f)\n", (name), _g);                        \
        } else {                                                               \
            printf("  FAIL  %s  got=%.3f  expected=%.3f  tol=%.3f\n", (name),  \
                   _g, _e, (float)(tol));                                      \
            g_failures++;                                                      \
        }                                                                      \
    } while (0)

// Tests
static void test_partial_frame(void) {
    printf("test_partial_frame\n");
    crsf_init();

    uint16_t channels[16];
    for (int i = 0; i < 16; i++)
        channels[i] = 992;

    uint8_t frame[26];
    build_rc_frame(frame, channels);

    // Feed only part of the frame
    feed_frame(frame, 10);

    // Should not update yet
    float expected = CRSF_RC_CHANNEL_SCALE_LEGACY * 0.0f + 881.0f;
    CHECK_NEAR("no update on partial frame", crsf_get_channel(0), expected,
               1.0f);

    // Feed the rest
    feed_frame(frame + 10, 16);

    float exp_mid = CRSF_RC_CHANNEL_SCALE_LEGACY * 992.0f + 881.0f;
    CHECK_NEAR("update after full frame", crsf_get_channel(0), exp_mid, 1.0f);
    print_channels();
}

static void test_wrong_frame_type(void) {
    printf("test_wrong_frame_type\n");
    crsf_init();

    uint16_t channels[16];
    for (int i = 0; i < 16; i++)
        channels[i] = 1811;

    uint8_t frame[26];
    build_rc_frame(frame, channels);

    frame[2] = 0x14; // some other CRSF frame type
    frame[25] = compute_crc(frame + 2, 23);

    feed_frame(frame, 26);

    float expected = CRSF_RC_CHANNEL_SCALE_LEGACY * 0.0f + 881.0f;
    CHECK_NEAR("wrong type ignored", crsf_get_channel(0), expected, 1.0f);
    print_channels();
}

static void test_invalid_length(void) {
    printf("test_invalid_length\n");
    crsf_init();

    uint16_t channels[16];
    for (int i = 0; i < 16; i++)
        channels[i] = 1811;

    uint8_t frame[26];
    build_rc_frame(frame, channels);

    frame[1] = 0x05; // impossible length

    feed_frame(frame, 26);

    float expected = CRSF_RC_CHANNEL_SCALE_LEGACY * 0.0f + 881.0f;
    CHECK_NEAR("invalid length ignored", crsf_get_channel(0), expected, 1.0f);
    print_channels();
}

static void test_bit_boundaries(void) {
    printf("test_bit_boundaries\n");
    crsf_init();

    uint16_t channels[16] = {0,    1,    1023, 1024, 2047, 0,    1,    1023,
                             1024, 2047, 0,    1,    1023, 1024, 2047, 0};

    uint8_t frame[26];
    build_rc_frame(frame, channels);
    feed_frame(frame, 26);

    float v0 = CRSF_RC_CHANNEL_SCALE_LEGACY * 0.0f + 881.0f;
    float v1 = CRSF_RC_CHANNEL_SCALE_LEGACY * 1.0f + 881.0f;
    float v1023 = CRSF_RC_CHANNEL_SCALE_LEGACY * 1023.0f + 881.0f;
    float v1024 = CRSF_RC_CHANNEL_SCALE_LEGACY * 1024.0f + 881.0f;
    float v2047 = CRSF_RC_CHANNEL_SCALE_LEGACY * 2047.0f + 881.0f;

    CHECK_NEAR("ch0=0", crsf_get_channel(0), v0, 1.0f);
    CHECK_NEAR("ch1=1", crsf_get_channel(1), v1, 1.0f);
    CHECK_NEAR("ch2=1023", crsf_get_channel(2), v1023, 1.0f);
    CHECK_NEAR("ch3=1024", crsf_get_channel(3), v1024, 1.0f);
    CHECK_NEAR("ch4=2047", crsf_get_channel(4), v2047, 1.0f);
    print_channels();
}

static void test_streamed_bytes(void) {
    printf("test_streamed_bytes\n");
    crsf_init();

    uint16_t channels[16];
    for (int i = 0; i < 16; i++)
        channels[i] = 992;

    uint8_t frame[26];
    build_rc_frame(frame, channels);

    for (int i = 0; i < 26; i++)
        crsf_feed_byte(frame[i]);

    float expected = CRSF_RC_CHANNEL_SCALE_LEGACY * 992.0f + 881.0f;
    CHECK_NEAR("byte-stream frame", crsf_get_channel(0), expected, 1.0f);
    print_channels();
}

#include <stdlib.h>

static void test_noise_fuzz(void) {
    printf("test_noise_fuzz\n");
    crsf_init();

    uint16_t channels[16];
    for (int i = 0; i < 16; i++)
        channels[i] = 992;

    uint8_t frame[26];
    build_rc_frame(frame, channels);

    for (int i = 0; i < 10000; i++) {

        if (rand() % 200 == 0) {
            feed_frame(frame, 26);
        } else {
            crsf_feed_byte(rand() & 0xFF);
        }
    }

    float expected = CRSF_RC_CHANNEL_SCALE_LEGACY * 992.0f + 881.0f;
    CHECK_NEAR("fuzz final value", crsf_get_channel(0), expected, 1.0f);
    print_channels();
}

// All 16 channels at mid-stick (992).
// Expected PWM = CRSF_RC_CHANNEL_SCALE_LEGACY * 992 + 881 ≈ 1500.7 µs.
static void test_midstick(void) {
    printf("test_midstick\n");
    crsf_init();

    uint16_t channels[16];
    for (int i = 0; i < 16; i++)
        channels[i] = 992;

    uint8_t frame[26];
    build_rc_frame(frame, channels);
    feed_frame(frame, 26);

    float expected = CRSF_RC_CHANNEL_SCALE_LEGACY * 992.0f + 881.0f;
    CHECK_NEAR("ch0  mid", crsf_get_channel(0), expected, 1.0f);
    CHECK_NEAR("ch7  mid", crsf_get_channel(7), expected, 1.0f);
    CHECK_NEAR("ch15 mid", crsf_get_channel(15), expected, 1.0f);
    print_channels();
}

// Channel 0 at minimum (172 → ~988 µs), channel 1 at maximum (1811 → ~2012 µs).
static void test_min_max(void) {
    printf("test_min_max\n");
    crsf_init();

    uint16_t channels[16];
    for (int i = 0; i < 16; i++)
        channels[i] = 992;
    channels[0] = 172;
    channels[1] = 1811;

    uint8_t frame[26];
    build_rc_frame(frame, channels);
    feed_frame(frame, 26);

    float exp_min = CRSF_RC_CHANNEL_SCALE_LEGACY * 172.0f + 881.0f;  // ≈ 988.5
    float exp_max = CRSF_RC_CHANNEL_SCALE_LEGACY * 1811.0f + 881.0f; // ≈ 2012.5
    CHECK_NEAR("ch0 min", crsf_get_channel(0), exp_min, 1.0f);
    CHECK_NEAR("ch1 max", crsf_get_channel(1), exp_max, 1.0f);
    // Other channels should still be mid
    float exp_mid = CRSF_RC_CHANNEL_SCALE_LEGACY * 992.0f + 881.0f;
    CHECK_NEAR("ch2 mid", crsf_get_channel(2), exp_mid, 1.0f);
    print_channels();
}

// A frame with a corrupted CRC must NOT update channel data.
// After crsf_init() all channels are 0 → get_channel(0) = 881.0 µs.
static void test_bad_crc_ignored(void) {
    printf("test_bad_crc_ignored\n");
    crsf_init();

    uint16_t channels[16];
    for (int i = 0; i < 16; i++)
        channels[i] = 1811;

    uint8_t frame[26];
    build_rc_frame(frame, channels);
    frame[25] ^= 0xFF; // corrupt CRC
    feed_frame(frame, 26);

    // Channels must remain at the post-init value (0).
    float expected = CRSF_RC_CHANNEL_SCALE_LEGACY * 0.0f + 881.0f; // = 881.0
    CHECK_NEAR("ch0 unchanged after bad CRC", crsf_get_channel(0), expected,
               1.0f);
    print_channels();
}

// Garbage bytes before a valid frame must be discarded.
static void test_garbage_prefix(void) {
    printf("test_garbage_prefix\n");
    crsf_init();

    uint16_t channels[16];
    for (int i = 0; i < 16; i++)
        channels[i] = 992;

    uint8_t frame[26];
    build_rc_frame(frame, channels);

    // Feed garbage (none of these are valid CRSF address bytes)
    uint8_t garbage[] = {0x01, 0x02, 0x03, 0xDE, 0xAD, 0xBE};
    feed_frame(garbage, (int)sizeof(garbage));
    // Then the real frame
    feed_frame(frame, 26);

    float expected = CRSF_RC_CHANNEL_SCALE_LEGACY * 992.0f + 881.0f;
    CHECK_NEAR("ch0 after garbage prefix", crsf_get_channel(0), expected, 1.0f);
    print_channels();
}

// Two back-to-back valid frames — second frame's values must win.
static void test_two_frames(void) {
    printf("test_two_frames\n");
    crsf_init();

    uint16_t ch_first[16], ch_second[16];
    for (int i = 0; i < 16; i++) {
        ch_first[i] = 172;
        ch_second[i] = 1811;
    }

    uint8_t frame1[26], frame2[26];
    build_rc_frame(frame1, ch_first);
    build_rc_frame(frame2, ch_second);
    feed_frame(frame1, 26);
    feed_frame(frame2, 26);

    float expected = CRSF_RC_CHANNEL_SCALE_LEGACY * 1811.0f + 881.0f;
    CHECK_NEAR("ch0 second frame wins", crsf_get_channel(0), expected, 1.0f);
    print_channels();
}

// main

int main(void) {

    test_midstick();
    test_min_max();
    test_bad_crc_ignored();
    test_garbage_prefix();
    test_two_frames();

    test_partial_frame();
    test_wrong_frame_type();
    test_invalid_length();
    test_bit_boundaries();
    test_streamed_bytes();
    test_noise_fuzz();

    printf("\n%s  (%d failure%s)\n", g_failures ? "FAILED" : "PASSED",
           g_failures, g_failures == 1 ? "" : "s");

    return g_failures ? 1 : 0;
}
