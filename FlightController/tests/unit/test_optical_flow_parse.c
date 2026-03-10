#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "optical_flow.h"

/* ── Test harness ────────────────────────────────────────────────────────── */

static int g_failures = 0;

#define CHECK_INT(name, got, expected)                                         \
    do {                                                                       \
        int64_t _g = (int64_t)(got), _e = (int64_t)(expected);                 \
        if (_g == _e) {                                                        \
            printf("  PASS  %s  (%lld)\n", (name), (long long)_g);             \
        } else {                                                               \
            printf("  FAIL  %s  got=%lld  expected=%lld\n", (name),            \
                   (long long)_g, (long long)_e);                              \
            g_failures++;                                                      \
        }                                                                      \
    } while (0)

#define CHECK_NEAR(name, got, expected, tol)                                   \
    do {                                                                       \
        float _g = (got), _e = (expected), _d = _g - _e;                       \
        if (_d < 0.0f)                                                         \
            _d = -_d;                                                          \
        if (_d <= (tol)) {                                                     \
            printf("  PASS  %s  (%.4f)\n", (name), _g);                        \
        } else {                                                               \
            printf("  FAIL  %s  got=%.4f  expected=%.4f  tol=%.4f\n", (name),  \
                   _g, _e, (float)(tol));                                      \
            g_failures++;                                                      \
        }                                                                      \
    } while (0)

/* ── Frame construction ──────────────────────────────────────────────────── */

/*
 * Frame layout:
 *   [STX][DEV_ID][SYS_ID][MSG_ID][SEQ][LEN][...payload (LEN bytes)...][CHKSUM]
 *
 * Checksum = sum of all bytes from STX through last payload byte, & 0xFF.
 * This matches the parser: checksum = STX, then += each subsequent byte.
 */

#define FRAME_HEADER_LEN 6
#define FRAME_PAYLOAD_LEN MICROLINK_MTF01_PAYLOAD_LEN              /* 20 */
#define FRAME_TOTAL_LEN (FRAME_HEADER_LEN + FRAME_PAYLOAD_LEN + 1) /* 27 */

typedef struct {
    uint32_t system_time;
    uint32_t distance;
    uint8_t distance_strength;
    uint8_t distance_precision;
    uint8_t distance_status;
    uint8_t reserved1;
    int16_t flow_vel_x;
    int16_t flow_vel_y;
    uint8_t flow_quality;
    uint8_t flow_status;
    uint16_t reserved2;
} payload_fields_t;

static void encode_payload(uint8_t *buf, const payload_fields_t *f) {
    buf[0] = (f->system_time >> 0) & 0xFF;
    buf[1] = (f->system_time >> 8) & 0xFF;
    buf[2] = (f->system_time >> 16) & 0xFF;
    buf[3] = (f->system_time >> 24) & 0xFF;
    buf[4] = (f->distance >> 0) & 0xFF;
    buf[5] = (f->distance >> 8) & 0xFF;
    buf[6] = (f->distance >> 16) & 0xFF;
    buf[7] = (f->distance >> 24) & 0xFF;
    buf[8] = f->distance_strength;
    buf[9] = f->distance_precision;
    buf[10] = f->distance_status;
    buf[11] = f->reserved1;
    buf[12] = ((uint16_t)f->flow_vel_x >> 0) & 0xFF;
    buf[13] = ((uint16_t)f->flow_vel_x >> 8) & 0xFF;
    buf[14] = ((uint16_t)f->flow_vel_y >> 0) & 0xFF;
    buf[15] = ((uint16_t)f->flow_vel_y >> 8) & 0xFF;
    buf[16] = f->flow_quality;
    buf[17] = f->flow_status;
    buf[18] = (f->reserved2 >> 0) & 0xFF;
    buf[19] = (f->reserved2 >> 8) & 0xFF;
}

/* Build a variable-length frame with an arbitrary payload buffer and length. */
static void build_frame(uint8_t *frame, uint8_t seq, const uint8_t *payload,
                        uint8_t payload_len) {
    frame[0] = MICROLINK_STX;
    frame[1] = MICROLINK_DEV_ID;
    frame[2] = MICROLINK_SYS_ID;
    frame[3] = MICROLINK_MSG_ID;
    frame[4] = seq;
    frame[5] = payload_len;
    memcpy(frame + FRAME_HEADER_LEN, payload, payload_len);

    uint8_t chk = 0;
    for (int i = 0; i < FRAME_HEADER_LEN + payload_len; i++)
        chk += frame[i];
    frame[FRAME_HEADER_LEN + payload_len] = chk;
}

static void build_valid_frame(uint8_t *frame, uint8_t seq,
                              const payload_fields_t *fields) {
    uint8_t payload[FRAME_PAYLOAD_LEN];
    encode_payload(payload, fields);
    build_frame(frame, seq, payload, FRAME_PAYLOAD_LEN);
}

static void feed_frame(const uint8_t *frame, int len) {
    for (int i = 0; i < len; i++)
        optical_flow_feed_byte(frame[i]);
}

static void get_stats(uint32_t *bytes, uint32_t *frames, uint32_t *chk_err,
                      uint32_t *hdr_err) {
    optical_flow_get_debug_stats(bytes, frames, chk_err, hdr_err, NULL, NULL,
                                 NULL, NULL);
}

/* ── Nominal sensor readings used across tests ───────────────────────────── */

static const payload_fields_t k_nominal = {
    .system_time = 1000,
    .distance = 500,
    .distance_strength = 200,
    .distance_precision = 10,
    .distance_status = 1,
    .reserved1 = 0,
    .flow_vel_x = 100,
    .flow_vel_y = -50,
    .flow_quality = 180,
    .flow_status = 1,
    .reserved2 = 0,
};

/* ── Tests ───────────────────────────────────────────────────────────────── */

/* Happy path: verify every field decodes correctly. */
static void test_valid_frame_all_fields(void) {
    printf("test_valid_frame_all_fields\n");
    optical_flow_init();

    uint8_t frame[FRAME_TOTAL_LEN];
    build_valid_frame(frame, 0x07, &k_nominal);
    feed_frame(frame, FRAME_TOTAL_LEN);

    const optical_flow_data_t *d = optical_flow_get_data();
    CHECK_INT("data_valid", d->data_valid, 1);
    CHECK_INT("sequence", d->sequence, 0x07);
    CHECK_INT("system_time", d->system_time, 1000);
    CHECK_INT("distance", d->distance, 500);
    CHECK_INT("distance_strength", d->distance_strength, 200);
    CHECK_INT("distance_precision", d->distance_precision, 10);
    CHECK_INT("distance_status", d->distance_status, 1);
    CHECK_INT("flow_vel_x", d->flow_vel_x, 100);
    CHECK_INT("flow_vel_y", d->flow_vel_y, -50);
    CHECK_INT("flow_quality", d->flow_quality, 180);
    CHECK_INT("flow_status", d->flow_status, 1);
    printf("\n");
}

/* Signed int16 extremes must round-trip correctly. */
static void test_signed_flow_velocity_extremes(void) {
    printf("test_signed_flow_velocity_extremes\n");
    optical_flow_init();

    payload_fields_t f = k_nominal;
    f.flow_vel_x = -32768; /* INT16_MIN */
    f.flow_vel_y = 32767;  /* INT16_MAX */

    uint8_t frame[FRAME_TOTAL_LEN];
    build_valid_frame(frame, 1, &f);
    feed_frame(frame, FRAME_TOTAL_LEN);

    const optical_flow_data_t *d = optical_flow_get_data();
    CHECK_INT("flow_vel_x INT16_MIN", d->flow_vel_x, -32768);
    CHECK_INT("flow_vel_y INT16_MAX", d->flow_vel_y, 32767);
    printf("\n");
}

/* All fields at their maximum uint values. */
static void test_max_field_values(void) {
    printf("test_max_field_values\n");
    optical_flow_init();

    payload_fields_t f = {
        .system_time = 0xFFFFFFFF,
        .distance = 0xFFFFFFFF,
        .distance_strength = 0xFF,
        .distance_precision = 0xFF,
        .distance_status = 0xFF,
        .reserved1 = 0xFF,
        .flow_vel_x = 32767,
        .flow_vel_y = 32767,
        .flow_quality = 0xFF,
        .flow_status = 0xFF,
        .reserved2 = 0xFFFF,
    };

    uint8_t frame[FRAME_TOTAL_LEN];
    build_valid_frame(frame, 0xFF, &f);
    feed_frame(frame, FRAME_TOTAL_LEN);

    const optical_flow_data_t *d = optical_flow_get_data();
    CHECK_INT("data_valid", d->data_valid, 1);
    CHECK_INT("system_time", d->system_time, 0xFFFFFFFF);
    CHECK_INT("distance", d->distance, 0xFFFFFFFF);
    CHECK_INT("flow_vel_x", d->flow_vel_x, 32767);
    CHECK_INT("sequence", d->sequence, 0xFF);
    printf("\n");
}

/* Corrupted checksum must not update sensor data and must increment error
 * counter. */
static void test_bad_checksum_rejected(void) {
    printf("test_bad_checksum_rejected\n");
    optical_flow_init();

    uint8_t frame[FRAME_TOTAL_LEN];
    build_valid_frame(frame, 0, &k_nominal);
    frame[FRAME_TOTAL_LEN - 1] ^= 0xFF; /* flip all bits in checksum */
    feed_frame(frame, FRAME_TOTAL_LEN);

    const optical_flow_data_t *d = optical_flow_get_data();
    CHECK_INT("data_valid stays 0", d->data_valid, 0);

    uint32_t bytes, frames, chk_err, hdr_err;
    get_stats(&bytes, &frames, &chk_err, &hdr_err);
    CHECK_INT("checksum_errors = 1", chk_err, 1);
    CHECK_INT("frames_parsed = 0", frames, 0);
    printf("\n");
}

/* Off-by-one checksum (one bit flipped) must also be caught. */
static void test_checksum_off_by_one(void) {
    printf("test_checksum_off_by_one\n");
    optical_flow_init();

    uint8_t frame[FRAME_TOTAL_LEN];
    build_valid_frame(frame, 0, &k_nominal);
    frame[FRAME_TOTAL_LEN - 1] ^= 0x01;
    feed_frame(frame, FRAME_TOTAL_LEN);

    CHECK_INT("data_valid stays 0", optical_flow_get_data()->data_valid, 0);
    printf("\n");
}

/* Wrong DEV_ID must reset parser and increment header_errors. */
static void test_bad_dev_id(void) {
    printf("test_bad_dev_id\n");
    optical_flow_init();

    uint8_t frame[FRAME_TOTAL_LEN];
    build_valid_frame(frame, 0, &k_nominal);
    frame[1] = 0xAB; /* corrupt DEV_ID */
    feed_frame(frame, FRAME_TOTAL_LEN);

    CHECK_INT("data_valid stays 0", optical_flow_get_data()->data_valid, 0);

    uint32_t bytes, frames, chk_err, hdr_err;
    get_stats(&bytes, &frames, &chk_err, &hdr_err);
    CHECK_INT("header_errors >= 1", (hdr_err >= 1), 1);
    printf("\n");
}

/* Wrong SYS_ID must reset parser and increment header_errors. */
static void test_bad_sys_id(void) {
    printf("test_bad_sys_id\n");
    optical_flow_init();

    uint8_t frame[FRAME_TOTAL_LEN];
    build_valid_frame(frame, 0, &k_nominal);
    frame[2] = 0x11;
    feed_frame(frame, FRAME_TOTAL_LEN);

    CHECK_INT("data_valid stays 0", optical_flow_get_data()->data_valid, 0);

    uint32_t bytes, frames, chk_err, hdr_err;
    get_stats(&bytes, &frames, &chk_err, &hdr_err);
    CHECK_INT("header_errors >= 1", (hdr_err >= 1), 1);
    printf("\n");
}

/* Wrong MSG_ID must reset parser and increment header_errors. */
static void test_bad_msg_id(void) {
    printf("test_bad_msg_id\n");
    optical_flow_init();

    uint8_t frame[FRAME_TOTAL_LEN];
    build_valid_frame(frame, 0, &k_nominal);
    frame[3] = 0x99;
    feed_frame(frame, FRAME_TOTAL_LEN);

    CHECK_INT("data_valid stays 0", optical_flow_get_data()->data_valid, 0);

    uint32_t bytes, frames, chk_err, hdr_err;
    get_stats(&bytes, &frames, &chk_err, &hdr_err);
    CHECK_INT("header_errors >= 1", (hdr_err >= 1), 1);
    printf("\n");
}

/*
 * A frame with the wrong payload length field (but a valid checksum for that
 * shorter payload) must be accepted at the checksum level but rejected by
 * parse_payload() — data_valid stays 0, frames_parsed still increments.
 */
static void test_wrong_payload_length_field(void) {
    printf("test_wrong_payload_length_field\n");
    optical_flow_init();

    const uint8_t bad_len = 10; /* not MICROLINK_MTF01_PAYLOAD_LEN */
    const int bad_total = FRAME_HEADER_LEN + bad_len + 1;
    uint8_t bad_frame[FRAME_HEADER_LEN + 10 + 1]; /* max 10 payload bytes */

    bad_frame[0] = MICROLINK_STX;
    bad_frame[1] = MICROLINK_DEV_ID;
    bad_frame[2] = MICROLINK_SYS_ID;
    bad_frame[3] = MICROLINK_MSG_ID;
    bad_frame[4] = 0x00; /* seq */
    bad_frame[5] = bad_len;
    memset(bad_frame + FRAME_HEADER_LEN, 0x55, bad_len);

    uint8_t chk = 0;
    for (int i = 0; i < FRAME_HEADER_LEN + bad_len; i++)
        chk += bad_frame[i];
    bad_frame[FRAME_HEADER_LEN + bad_len] = chk;

    feed_frame(bad_frame, bad_total);

    CHECK_INT("data_valid stays 0 on wrong len",
              optical_flow_get_data()->data_valid, 0);

    /* Checksum was valid so frames_parsed still ticks up */
    uint32_t bytes, frames, chk_err, hdr_err;
    get_stats(&bytes, &frames, &chk_err, &hdr_err);
    CHECK_INT("frames_parsed = 1 (checksum ok, payload rejected)", frames, 1);
    printf("\n");
}

/* Garbage bytes before a valid frame must be silently discarded. */
static void test_garbage_before_frame(void) {
    printf("test_garbage_before_frame\n");
    optical_flow_init();

    uint8_t garbage[] = {0x00, 0x11, 0x22, 0xAB, 0xCD, 0x12, 0x34, 0x55};
    feed_frame(garbage, (int)sizeof(garbage));

    uint8_t frame[FRAME_TOTAL_LEN];
    build_valid_frame(frame, 3, &k_nominal);
    feed_frame(frame, FRAME_TOTAL_LEN);

    const optical_flow_data_t *d = optical_flow_get_data();
    CHECK_INT("data_valid after garbage+frame", d->data_valid, 1);
    CHECK_INT("sequence correct", d->sequence, 3);
    printf("\n");
}

/* Parser must not update until the frame is complete. */
static void test_partial_frame_no_update(void) {
    printf("test_partial_frame_no_update\n");
    optical_flow_init();

    uint8_t frame[FRAME_TOTAL_LEN];
    build_valid_frame(frame, 5, &k_nominal);

    /* Feed header only, stop before payload */
    feed_frame(frame, FRAME_HEADER_LEN);
    CHECK_INT("data_valid stays 0 mid-frame",
              optical_flow_get_data()->data_valid, 0);

    /* Complete the frame */
    feed_frame(frame + FRAME_HEADER_LEN, FRAME_TOTAL_LEN - FRAME_HEADER_LEN);
    const optical_flow_data_t *d = optical_flow_get_data();
    CHECK_INT("data_valid after completing frame", d->data_valid, 1);
    CHECK_INT("distance correct after partial", d->distance,
              k_nominal.distance);
    printf("\n");
}

/* Byte fed at a time (simulating one UART interrupt per byte). */
static void test_single_byte_streaming(void) {
    printf("test_single_byte_streaming\n");
    optical_flow_init();

    uint8_t frame[FRAME_TOTAL_LEN];
    build_valid_frame(frame, 11, &k_nominal);

    for (int i = 0; i < FRAME_TOTAL_LEN; i++)
        optical_flow_feed_byte(frame[i]);

    const optical_flow_data_t *d = optical_flow_get_data();
    CHECK_INT("data_valid", d->data_valid, 1);
    CHECK_INT("distance", d->distance, k_nominal.distance);
    CHECK_INT("flow_vel_x", d->flow_vel_x, k_nominal.flow_vel_x);
    printf("\n");
}

/* Second frame overwrites first — most-recent data wins. */
static void test_two_consecutive_frames(void) {
    printf("test_two_consecutive_frames\n");
    optical_flow_init();

    payload_fields_t f1 = k_nominal;
    f1.distance = 100;
    payload_fields_t f2 = k_nominal;
    f2.distance = 999;

    uint8_t frame1[FRAME_TOTAL_LEN], frame2[FRAME_TOTAL_LEN];
    build_valid_frame(frame1, 1, &f1);
    build_valid_frame(frame2, 2, &f2);
    feed_frame(frame1, FRAME_TOTAL_LEN);
    feed_frame(frame2, FRAME_TOTAL_LEN);

    const optical_flow_data_t *d = optical_flow_get_data();
    CHECK_INT("distance from frame2", d->distance, 999);
    CHECK_INT("sequence from frame2", d->sequence, 2);

    uint32_t bytes, frames, chk_err, hdr_err;
    get_stats(&bytes, &frames, &chk_err, &hdr_err);
    CHECK_INT("frames_parsed = 2", frames, 2);
    printf("\n");
}

/*
 * STX byte (0xEF) embedded inside the payload must NOT restart the parser.
 * The state machine only looks for STX in PARSE_STATE_STX.
 */
static void test_stx_byte_inside_payload(void) {
    printf("test_stx_byte_inside_payload\n");
    optical_flow_init();

    payload_fields_t f = k_nominal;
    f.system_time = 0xEFEFEFEF; /* STX byte appears 4 times in first field */
    f.distance = 0xEF0000EF;

    uint8_t frame[FRAME_TOTAL_LEN];
    build_valid_frame(frame, 9, &f);
    feed_frame(frame, FRAME_TOTAL_LEN);

    const optical_flow_data_t *d = optical_flow_get_data();
    CHECK_INT("data_valid with STX in payload", d->data_valid, 1);
    CHECK_INT("system_time with STX bytes", d->system_time, 0xEFEFEFEF);
    CHECK_INT("distance with STX bytes", d->distance, 0xEF0000EF);
    printf("\n");
}

/* Sequence number must track the SEQ byte from each frame. */
static void test_sequence_tracking(void) {
    printf("test_sequence_tracking\n");
    optical_flow_init();

    uint8_t frame[FRAME_TOTAL_LEN];
    for (uint8_t seq = 0; seq < 8; seq++) {
        build_valid_frame(frame, seq, &k_nominal);
        feed_frame(frame, FRAME_TOTAL_LEN);

        char name[32];
        snprintf(name, sizeof(name), "seq=%u", seq);
        CHECK_INT(name, optical_flow_get_data()->sequence, seq);
    }
    printf("\n");
}

/* optical_flow_calc_velocity: speed = flow_vel * (distance_mm / 1000) */
static void test_calc_velocity(void) {
    printf("test_calc_velocity\n");
    CHECK_NEAR("100 @ 1m", optical_flow_calc_velocity(100, 1000), 100.0f,
               0.01f);
    CHECK_NEAR("100 @ 0.5m", optical_flow_calc_velocity(100, 500), 50.0f,
               0.01f);
    CHECK_NEAR("100 @ 2m", optical_flow_calc_velocity(100, 2000), 200.0f,
               0.01f);
    CHECK_NEAR("-50 @ 1m", optical_flow_calc_velocity(-50, 1000), -50.0f,
               0.01f);
    CHECK_NEAR("0 flow @ 1m", optical_flow_calc_velocity(0, 1000), 0.0f,
               0.001f);
    CHECK_NEAR("100 @ 0mm", optical_flow_calc_velocity(100, 0), 0.0f, 0.001f);
    printf("\n");
}

/* optical_flow_is_distance_valid: requires status==1 AND distance>0. */
static void test_distance_valid_flag(void) {
    printf("test_distance_valid_flag\n");
    uint8_t frame[FRAME_TOTAL_LEN];
    payload_fields_t f = k_nominal;

    /* Case 1: status=1, distance>0 → valid */
    optical_flow_init();
    f.distance_status = 1;
    f.distance = 500;
    build_valid_frame(frame, 0, &f);
    feed_frame(frame, FRAME_TOTAL_LEN);
    CHECK_INT("valid: status=1 dist=500",
              optical_flow_is_distance_valid(optical_flow_get_data()), 1);

    /* Case 2: status=0 → invalid regardless of distance */
    optical_flow_init();
    f.distance_status = 0;
    f.distance = 500;
    build_valid_frame(frame, 0, &f);
    feed_frame(frame, FRAME_TOTAL_LEN);
    CHECK_INT("invalid: status=0",
              optical_flow_is_distance_valid(optical_flow_get_data()), 0);

    /* Case 3: status=1 but distance=0 → invalid */
    optical_flow_init();
    f.distance_status = 1;
    f.distance = 0;
    build_valid_frame(frame, 0, &f);
    feed_frame(frame, FRAME_TOTAL_LEN);
    CHECK_INT("invalid: status=1 but dist=0",
              optical_flow_is_distance_valid(optical_flow_get_data()), 0);
    printf("\n");
}

/* optical_flow_is_flow_valid: requires flow_status==1 only. */
static void test_flow_valid_flag(void) {
    printf("test_flow_valid_flag\n");
    uint8_t frame[FRAME_TOTAL_LEN];
    payload_fields_t f = k_nominal;

    optical_flow_init();
    f.flow_status = 1;
    build_valid_frame(frame, 0, &f);
    feed_frame(frame, FRAME_TOTAL_LEN);
    CHECK_INT("valid: flow_status=1",
              optical_flow_is_flow_valid(optical_flow_get_data()), 1);

    optical_flow_init();
    f.flow_status = 0;
    build_valid_frame(frame, 0, &f);
    feed_frame(frame, FRAME_TOTAL_LEN);
    CHECK_INT("invalid: flow_status=0",
              optical_flow_is_flow_valid(optical_flow_get_data()), 0);
    printf("\n");
}

/* Debug counters must accurately reflect what was processed. */
static void test_debug_counters(void) {
    printf("test_debug_counters\n");
    optical_flow_init();

    uint8_t frame[FRAME_TOTAL_LEN];
    build_valid_frame(frame, 0, &k_nominal);
    feed_frame(frame, FRAME_TOTAL_LEN);

    uint32_t bytes, frames, chk_err, hdr_err;
    get_stats(&bytes, &frames, &chk_err, &hdr_err);
    CHECK_INT("total_bytes = FRAME_TOTAL_LEN", bytes, FRAME_TOTAL_LEN);
    CHECK_INT("frames_parsed = 1", frames, 1);
    CHECK_INT("checksum_errors = 0", chk_err, 0);
    CHECK_INT("header_errors = 0", hdr_err, 0);
    printf("\n");
}

/*
 * After a bad checksum, the parser must recover and correctly parse the
 * next clean frame — ensures reset_parser() restores a clean state.
 */
static void test_recovery_after_bad_checksum(void) {
    printf("test_recovery_after_bad_checksum\n");
    optical_flow_init();

    uint8_t bad[FRAME_TOTAL_LEN], good[FRAME_TOTAL_LEN];
    build_valid_frame(bad, 0, &k_nominal);
    bad[FRAME_TOTAL_LEN - 1] ^= 0xFF;

    payload_fields_t f2 = k_nominal;
    f2.distance = 1234;
    build_valid_frame(good, 7, &f2);

    feed_frame(bad, FRAME_TOTAL_LEN);
    feed_frame(good, FRAME_TOTAL_LEN);

    const optical_flow_data_t *d = optical_flow_get_data();
    CHECK_INT("recovered: data_valid", d->data_valid, 1);
    CHECK_INT("recovered: distance", d->distance, 1234);
    CHECK_INT("recovered: sequence", d->sequence, 7);
    printf("\n");
}

/*
 * Flood of random bytes with occasional valid frames sprinkled in.
 * Parser must remain stable and correctly parse the final clean frame.
 */
static void test_noise_fuzz(void) {
    printf("test_noise_fuzz\n");
    optical_flow_init();

    uint8_t frame[FRAME_TOTAL_LEN];
    build_valid_frame(frame, 42, &k_nominal);

    srand(0xDEADBEEF);
    for (int i = 0; i < 50000; i++) {
        if (rand() % 300 == 0)
            feed_frame(frame, FRAME_TOTAL_LEN);
        else
            optical_flow_feed_byte((uint8_t)(rand() & 0xFF));
    }

    /* Final clean frame must be parsed correctly */
    payload_fields_t f_final = k_nominal;
    f_final.distance = 7777;
    build_valid_frame(frame, 99, &f_final);
    feed_frame(frame, FRAME_TOTAL_LEN);

    const optical_flow_data_t *d = optical_flow_get_data();
    CHECK_INT("fuzz: data_valid", d->data_valid, 1);
    CHECK_INT("fuzz: sequence", d->sequence, 99);
    CHECK_INT("fuzz: distance", d->distance, 7777);
    printf("\n");
}

/* ── main ────────────────────────────────────────────────────────────────── */

int main(void) {
    test_valid_frame_all_fields();
    test_signed_flow_velocity_extremes();
    test_max_field_values();

    test_bad_checksum_rejected();
    test_checksum_off_by_one();
    test_bad_dev_id();
    test_bad_sys_id();
    test_bad_msg_id();
    test_wrong_payload_length_field();

    test_garbage_before_frame();
    test_partial_frame_no_update();
    test_single_byte_streaming();
    test_two_consecutive_frames();
    test_stx_byte_inside_payload();
    test_sequence_tracking();

    test_calc_velocity();
    test_distance_valid_flag();
    test_flow_valid_flag();
    test_debug_counters();
    test_recovery_after_bad_checksum();
    test_noise_fuzz();

    printf("\n%s  (%d failure%s)\n", g_failures ? "FAILED" : "PASSED",
           g_failures, g_failures == 1 ? "" : "s");
    return g_failures ? 1 : 0;
}
