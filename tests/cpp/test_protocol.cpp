// test_protocol.cpp -- wire format, CRC and validation tests.
//
// Every test is named for the failure it prevents, so the reason it exists
// outlives anyone's memory of writing it.

#include <cstring>

#include "../../firmware/src/rover_protocol.h"
#include "test_framework.h"

using namespace rover;

// --- CRC-8 ------------------------------------------------------------------

static void crc8_known_answer_vector() {
    // CRC-8/SAE-J1850 of "123456789" is 0x4B. This test is the whole reason
    // the CRC was written before anything that uses it: a CRC with the right
    // shape and the wrong parameters produces plausible garbage, and the only
    // cheap way to know is a published check value.
    //
    // This repo has already shipped a CRC documented as one variant and
    // implemented as another. Anyone writing a second implementation from the
    // spec would have had every frame rejected.
    const char* v = "123456789";
    CHECK_EQ(crc8(reinterpret_cast<const uint8_t*>(v), 9), 0x4B);
}

static void crc8_is_not_a_different_variant() {
    // Guards against someone "fixing" the polynomial or the init value. These
    // are the published check values of the neighbours we are NOT using.
    const char* v = "123456789";
    uint8_t ours = crc8(reinterpret_cast<const uint8_t*>(v), 9);
    CHECK(ours != 0xDF);   // CRC-8/AUTOSAR, poly 0x2F
    CHECK(ours != 0xF4);   // CRC-8/SMBUS,   poly 0x07
    CHECK(ours != 0xA1);   // CRC-8/BLUETOOTH
}

static void crc8_of_nothing_is_the_init_xored() {
    CHECK_EQ(crc8(nullptr, 0), 0x00);     // 0xFF ^ 0xFF
}

static void crc8_detects_every_single_bit_flip() {
    uint8_t base[6] = {0x12, 0x34, 0x56, 0x78, 0x9A, 0xBC};
    const uint8_t want = crc8(base, 6);
    for (int byte = 0; byte < 6; ++byte) {
        for (int bit = 0; bit < 8; ++bit) {
            uint8_t mutated[6];
            for (int i = 0; i < 6; ++i) mutated[i] = base[i];
            mutated[byte] ^= static_cast<uint8_t>(1u << bit);
            CHECK(crc8(mutated, 6) != want);
        }
    }
}

static void frame_crc_is_seeded_with_the_can_id() {
    // THE reason frameCrc8 exists. Without the id in the seed, a correctly
    // CRC'd payload delivered on the WRONG id would validate -- a telemetry
    // frame transmitted with CONTROL's id would be accepted as a command.
    const uint8_t payload[6] = {1, 2, 3, 4, 5, 6};
    const uint8_t a = frameCrc8(CAN_ID_CONTROL, payload, 6);
    const uint8_t b = frameCrc8(CAN_ID_TELEM_DRIVE_L, payload, 6);
    const uint8_t c = frameCrc8(CAN_ID_TELEM_POWER, payload, 6);
    CHECK(a != b);
    CHECK(a != c);
    CHECK(b != c);
}

static void frame_crc_is_stable_for_the_same_inputs() {
    const uint8_t payload[6] = {9, 8, 7, 6, 5, 4};
    CHECK_EQ(frameCrc8(CAN_ID_CONTROL, payload, 6),
             frameCrc8(CAN_ID_CONTROL, payload, 6));
}

// --- sequence arithmetic ----------------------------------------------------

static void seq_delta_is_correct_across_the_wrap() {
    // Unsigned subtraction, so 255 -> 0 is a delta of 1, not a 255-frame loss.
    // Same "subtract, never compare" rule as the millis() handling.
    CHECK_EQ(seqDelta(0, 1), 1);
    CHECK_EQ(seqDelta(254, 255), 1);
    CHECK_EQ(seqDelta(255, 0), 1);     // the wrap is healthy
    CHECK_EQ(seqDelta(254, 1), 3);     // two frames lost across the wrap
    CHECK_EQ(seqDelta(1, 1), 0);       // duplicate
    CHECK_EQ(seqDelta(0, 255), 255);
}

// --- CONTROL round trip -----------------------------------------------------

static ControlMsg makeControl(int16_t d, int16_t s, uint8_t mode, bool stop,
                              bool abort, bool ret, uint8_t ind, uint8_t seq,
                              bool c2_lost = false) {
    ControlMsg m;
    m.drive_cmd = d; m.steer_cmd = s; m.mode = mode;
    m.stop = stop; m.autonomy_abort = abort; m.return_request = ret;
    m.c2_lost = c2_lost;
    m.indicator_request = ind; m.seq = seq;
    return m;
}

static void command_range_predicate_holds_at_the_boundaries() {
    // Off by one here is the whole bug: +/-1000 is full scale and legal.
    CHECK(isValidCommand(0));
    CHECK(isValidCommand(CMD_MIN));
    CHECK(isValidCommand(CMD_MAX));
    CHECK(!isValidCommand(CMD_MIN - 1));
    CHECK(!isValidCommand(CMD_MAX + 1));
    CHECK(!isValidCommand(-32768));
    CHECK(!isValidCommand(32767));
}

static void out_of_range_commands_are_rejected_at_decode() {
    // Rejected, not clamped: clamping would quietly turn a misunderstood
    // command into full throttle. Both fields are checked independently.
    const int16_t bad[] = {-32768, CMD_MIN - 1, CMD_MAX + 1, 32767};
    for (int16_t v : bad) {
        uint8_t buf[8] = {0};
        ControlMsg out;
        encodeControl(makeControl(v, 0, MODE_MANUAL, false, false, false,
                                  INDICATOR_OFF, 0), buf);
        CHECK_EQ(decodeControl(CAN_ID_CONTROL, buf, FRAME_DLC, out),
                 DECODE_BAD_RANGE);
        encodeControl(makeControl(0, v, MODE_MANUAL, false, false, false,
                                  INDICATOR_OFF, 0), buf);
        CHECK_EQ(decodeControl(CAN_ID_CONTROL, buf, FRAME_DLC, out),
                 DECODE_BAD_RANGE);
    }
}

static void the_encoder_still_writes_any_int16() {
    // Validation is the receiver's job, and the receiver is what must fail
    // closed. The encoder stays a faithful serialiser so a test can put a
    // hostile value on the bus.
    uint8_t buf[8] = {0};
    CHECK_EQ(encodeControl(makeControl(-32768, 32767, MODE_MANUAL, false,
                                       false, false, INDICATOR_OFF, 0), buf),
             FRAME_DLC);
    CHECK_EQ(buf[0], 0x00); CHECK_EQ(buf[1], 0x80);   // int16 extremes still
    CHECK_EQ(buf[2], 0xFF); CHECK_EQ(buf[3], 0x7F);   // encode little-endian
}

static void control_round_trips_every_field() {
    const ControlMsg cases[] = {
        makeControl(0, 0, MODE_DISABLED, true, false, false, INDICATOR_OFF, 0),
        makeControl(1000, -1000, MODE_AUTONOMOUS, false, false, false,
                    INDICATOR_AUTONOMOUS, 1),
        makeControl(-1000, 1000, MODE_MANUAL, true, true, true,
                    INDICATOR_ARRIVED, 255),
        // Was the int16 extremes case. Those no longer decode -- they are
        // outside CMD_MIN..CMD_MAX, see
        // out_of_range_commands_are_rejected_at_decode -- so this case keeps
        // the flag and indicator combination with in-range commands.
        makeControl(-999, 999, MODE_MANUAL, false, true, false,
                    INDICATOR_FAULT, 128),
        makeControl(250, -50, MODE_AUTONOMOUS, false, false, false,
                    INDICATOR_AUTONOMOUS, 64, /*c2_lost=*/true),
        makeControl(0, 0, MODE_MANUAL, true, true, true,
                    INDICATOR_ARRIVED, 65, /*c2_lost=*/true),  // every flag set
    };
    for (const ControlMsg& in : cases) {
        uint8_t buf[8] = {0};
        CHECK_EQ(encodeControl(in, buf), FRAME_DLC);
        ControlMsg out;
        CHECK_EQ(decodeControl(CAN_ID_CONTROL, buf, FRAME_DLC, out), DECODE_OK);
        CHECK_EQ(out.drive_cmd, in.drive_cmd);
        CHECK_EQ(out.steer_cmd, in.steer_cmd);
        CHECK_EQ(out.mode, in.mode);
        CHECK_EQ(out.stop ? 1 : 0, in.stop ? 1 : 0);
        CHECK_EQ(out.autonomy_abort ? 1 : 0, in.autonomy_abort ? 1 : 0);
        CHECK_EQ(out.return_request ? 1 : 0, in.return_request ? 1 : 0);
        CHECK_EQ(out.c2_lost ? 1 : 0, in.c2_lost ? 1 : 0);
        CHECK_EQ(out.indicator_request, in.indicator_request);
        CHECK_EQ(out.seq, in.seq);
    }
}

static void control_flags_are_independent() {
    // Packing four things into one byte invites a mask or shift mistake that
    // makes two flags move together. Set each alone and check the others stay
    // clear.
    for (int which = 0; which < 4; ++which) {
        ControlMsg in = makeControl(0, 0, MODE_MANUAL,
                                    which == 0, which == 1, which == 2,
                                    INDICATOR_OFF, 0, /*c2_lost=*/which == 3);
        uint8_t buf[8] = {0};
        encodeControl(in, buf);
        ControlMsg out;
        CHECK_EQ(decodeControl(CAN_ID_CONTROL, buf, FRAME_DLC, out), DECODE_OK);
        CHECK_EQ(out.stop ? 1 : 0, which == 0 ? 1 : 0);
        CHECK_EQ(out.autonomy_abort ? 1 : 0, which == 1 ? 1 : 0);
        CHECK_EQ(out.return_request ? 1 : 0, which == 2 ? 1 : 0);
        CHECK_EQ(out.c2_lost ? 1 : 0, which == 3 ? 1 : 0);
        CHECK_EQ(out.indicator_request, INDICATOR_OFF);
    }
}

static void indicator_request_survives_alongside_flags() {
    // The indicator lives in bits 3-5 of the same byte as the three booleans.
    for (uint8_t ind = 0; ind <= INDICATOR_FAULT; ++ind) {
        ControlMsg in = makeControl(0, 0, MODE_MANUAL, true, true, true, ind, 0);
        uint8_t buf[8] = {0};
        encodeControl(in, buf);
        ControlMsg out;
        CHECK_EQ(decodeControl(CAN_ID_CONTROL, buf, FRAME_DLC, out), DECODE_OK);
        CHECK_EQ(out.indicator_request, ind);
        CHECK(out.stop && out.autonomy_abort && out.return_request);
    }
}

// --- telemetry round trips --------------------------------------------------

static void telemetry_drive_l_round_trips() {
    TelemetryDriveL in; in.enc_left = -123456; in.steer_fb = -1000;
    uint8_t buf[8] = {0};
    CHECK_EQ(encodeTelemetryDriveL(in, 7, buf), FRAME_DLC);
    TelemetryDriveL out; uint8_t seq = 0;
    CHECK_EQ(decodeTelemetryDriveL(CAN_ID_TELEM_DRIVE_L, buf, FRAME_DLC, out, seq),
             DECODE_OK);
    CHECK_EQ(out.enc_left, in.enc_left);
    CHECK_EQ(out.steer_fb, in.steer_fb);
    CHECK_EQ(seq, 7);
}

static void telemetry_drive_r_round_trips() {
    TelemetryDriveR in; in.enc_right = INT32_MAX_V; in.cmd_age_ms = CMD_AGE_UNKNOWN;
    uint8_t buf[8] = {0};
    encodeTelemetryDriveR(in, 200, buf);
    TelemetryDriveR out; uint8_t seq = 0;
    CHECK_EQ(decodeTelemetryDriveR(CAN_ID_TELEM_DRIVE_R, buf, FRAME_DLC, out, seq),
             DECODE_OK);
    CHECK_EQ(out.enc_right, INT32_MAX_V);
    CHECK_EQ(out.cmd_age_ms, CMD_AGE_UNKNOWN);
    CHECK_EQ(seq, 200);
}

static void telemetry_power_round_trips_signed_and_wide_faults() {
    // Current AND voltage are signed: a braking motor genuinely produces
    // negative current, and unsigned would wrap it to a huge positive value --
    // the worst kind of bad telemetry, because it looks plausible.
    // fault_status is uint16 since v0.4; the uint8 version was out of bits.
    TelemetryPower in;
    in.current_ca = -10000;                      // -100.00 A
    in.voltage_cv = 2400;                        //  +24.00 V
    in.fault_status = static_cast<uint16_t>(FAULT_SEQ_GAP | FAULT_CRC_ERROR
                                            | FAULT_COMM_TIMEOUT);
    uint8_t buf[8] = {0};
    encodeTelemetryPower(in, 3, buf);
    TelemetryPower out; uint8_t seq = 0;
    CHECK_EQ(decodeTelemetryPower(CAN_ID_TELEM_POWER, buf, FRAME_DLC, out, seq),
             DECODE_OK);
    CHECK_EQ(out.current_ca, -10000);
    CHECK_EQ(out.voltage_cv, 2400);
    CHECK_EQ(out.fault_status, in.fault_status);
    CHECK(out.fault_status > 0xFF);              // genuinely needs 16 bits
}

static void telemetry_state_round_trips() {
    TelemetryState in;
    in.mode = MODE_AUTONOMOUS; in.jetson_link = LINK_DEGRADED;
    in.c2_link = LINK_LOST; in.controller_health = CTRL_HEALTH_FAULT;
    in.indicator_state = INDICATOR_ARRIVED; in.reserved = 0;
    uint8_t buf[8] = {0};
    encodeTelemetryState(in, 42, buf);
    TelemetryState out; uint8_t seq = 0;
    CHECK_EQ(decodeTelemetryState(CAN_ID_TELEM_STATE, buf, FRAME_DLC, out, seq),
             DECODE_OK);
    CHECK_EQ(out.mode, MODE_AUTONOMOUS);
    CHECK_EQ(out.jetson_link, LINK_DEGRADED);
    CHECK_EQ(out.c2_link, LINK_LOST);
    CHECK_EQ(out.controller_health, CTRL_HEALTH_FAULT);
    CHECK_EQ(out.indicator_state, INDICATOR_ARRIVED);
    CHECK_EQ(seq, 42);
}

// --- validation -------------------------------------------------------------

static void wrong_dlc_is_rejected() {
    uint8_t buf[8] = {0};
    encodeControl(makeControl(100, 0, MODE_MANUAL, false, false, false,
                              INDICATOR_OFF, 0), buf);
    ControlMsg out;
    for (uint8_t len = 0; len <= 8; ++len) {
        if (len == FRAME_DLC) continue;
        CHECK_EQ(decodeControl(CAN_ID_CONTROL, buf, len, out), DECODE_BAD_DLC);
    }
    CHECK_EQ(decodeControl(CAN_ID_CONTROL, buf, FRAME_DLC, out), DECODE_OK);
}

static void a_corrupted_byte_is_caught_by_the_app_layer_crc() {
    // CAN's own CRC would never see this: it is corruption of the bytes AFTER
    // the sender assembled them, which is precisely the software-path failure
    // domain the application-layer CRC exists to cover.
    uint8_t good[8] = {0};
    encodeControl(makeControl(500, -200, MODE_MANUAL, false, false, false,
                              INDICATOR_TELEOP, 11), good);
    ControlMsg out;
    for (int byte = 0; byte < 7; ++byte) {          // every byte the CRC covers
        for (int bit = 0; bit < 8; ++bit) {
            uint8_t bad[8];
            for (int i = 0; i < 8; ++i) bad[i] = good[i];
            bad[byte] ^= static_cast<uint8_t>(1u << bit);
            DecodeResult r = decodeControl(CAN_ID_CONTROL, bad, FRAME_DLC, out);
            CHECK(r != DECODE_OK);
        }
    }
}

static void a_frame_on_the_wrong_can_id_fails() {
    // Because the CRC is id-seeded. A CONTROL payload arriving on a telemetry
    // id -- or a telemetry frame mistakenly transmitted with CONTROL's id --
    // must not validate.
    uint8_t buf[8] = {0};
    encodeControl(makeControl(900, 0, MODE_MANUAL, false, false, false,
                              INDICATOR_OFF, 5), buf);
    ControlMsg out;
    CHECK_EQ(decodeControl(CAN_ID_CONTROL, buf, FRAME_DLC, out), DECODE_OK);
    CHECK_EQ(decodeControl(CAN_ID_TELEM_POWER, buf, FRAME_DLC, out), DECODE_BAD_CRC);
    CHECK_EQ(decodeControl(0x321, buf, FRAME_DLC, out), DECODE_BAD_CRC);
}

static void mode_predicates_cover_all_256_values() {
    // `mode` is a uint8: 256 values, 3 defined. An earlier version asked
    // "is mode == DISABLED?" and stopped only then, so every undefined value
    // read as drivable and permitted full throttle. Exhaustive here because
    // 256 cases is cheap and the cost of missing one is a moving rover.
    for (int m = 0; m <= 255; ++m) {
        const uint8_t mode = static_cast<uint8_t>(m);
        const bool known = (m == MODE_DISABLED || m == MODE_MANUAL || m == MODE_AUTONOMOUS);
        const bool drivable = (m == MODE_MANUAL || m == MODE_AUTONOMOUS);
        CHECK_EQ(isKnownMode(mode), known);
        CHECK_EQ(modePermitsMotion(mode), drivable);
    }
}

static void disabled_is_known_but_not_drivable() {
    // The distinction the earlier bug turned on: DISABLED is a LEGITIMATE
    // command (decode accepts it) that forbids motion. An undefined value is a
    // PROTOCOL ERROR (decode rejects it).
    CHECK(isKnownMode(MODE_DISABLED));
    CHECK(!modePermitsMotion(MODE_DISABLED));
}

static void undefined_mode_is_rejected_at_decode() {
    ControlMsg out;
    for (int m = 3; m <= 255; ++m) {
        uint8_t buf[8] = {0};
        encodeControl(makeControl(1000, 500, static_cast<uint8_t>(m), false,
                                  false, false, INDICATOR_OFF, 0), buf);
        CHECK_EQ(decodeControl(CAN_ID_CONTROL, buf, FRAME_DLC, out), DECODE_BAD_MODE);
    }
}

static void reserved_flag_bits_are_rejected() {
    // Reserved bits set means a newer sender is using a field we do not
    // understand, so we cannot safely interpret the rest of the flags byte.
    // Only bit 7 is reserved now: bit 6 carries c2_lost.
    ControlMsg out;
    uint8_t buf[8] = {0};
    encodeControl(makeControl(0, 0, MODE_MANUAL, false, false, false,
                              INDICATOR_OFF, 0), buf);
    buf[5] |= CTRL_FLAG_RESERVED_MASK;
    buf[FRAME_CRC_OFFSET] = frameCrc8(CAN_ID_CONTROL, buf, FRAME_SEQ_OFFSET + 1);
    CHECK_EQ(decodeControl(CAN_ID_CONTROL, buf, FRAME_DLC, out), DECODE_BAD_FLAGS);
    CHECK_EQ(CTRL_FLAG_RESERVED_MASK, 0x80);
}

static void the_c2_flag_bit_is_a_field_not_a_reserved_bit() {
    // The guard against a typo'd reserved mask swallowing a live field: bit 6
    // must decode as c2_lost, not reject the frame.
    ControlMsg out;
    uint8_t buf[8] = {0};
    encodeControl(makeControl(0, 0, MODE_MANUAL, false, false, false,
                              INDICATOR_OFF, 0, /*c2_lost=*/true), buf);
    CHECK_EQ(buf[5] & CTRL_FLAG_C2_LOST, CTRL_FLAG_C2_LOST);
    CHECK_EQ(buf[5] & CTRL_FLAG_RESERVED_MASK, 0);
    CHECK_EQ(decodeControl(CAN_ID_CONTROL, buf, FRAME_DLC, out), DECODE_OK);
    CHECK(out.c2_lost);
}

static void indicator_predicate_covers_all_256_values() {
    for (int i = 0; i <= 255; ++i) {
        CHECK_EQ(isKnownIndicator(static_cast<uint8_t>(i)), i <= INDICATOR_FAULT);
    }
}

static void undefined_indicator_request_is_rejected() {
    // Bits 3-5 hold 0-7 but only 0-4 are defined, so 5, 6 and 7 are reachable
    // on the wire and must be rejected rather than displayed as something.
    ControlMsg out;
    for (uint8_t ind = 5; ind <= 7; ++ind) {
        uint8_t buf[8] = {0};
        encodeControl(makeControl(0, 0, MODE_MANUAL, false, false, false, ind, 0), buf);
        CHECK_EQ(decodeControl(CAN_ID_CONTROL, buf, FRAME_DLC, out),
                 DECODE_BAD_INDICATOR);
    }
}

// --- geometry and priority --------------------------------------------------

static void every_frame_is_exactly_eight_bytes() {
    // 8 bytes is Classic CAN's limit, so this protocol runs on a Classic bus
    // and alongside Classic-only motor controllers. Exceeding it would force
    // CAN FD transceivers onto every node.
    CHECK_EQ(FRAME_DLC, 8);
    CHECK_EQ(FRAME_PAYLOAD + 2, FRAME_DLC);     // payload + seq + crc
    uint8_t buf[8] = {0};
    CHECK_EQ(encodeControl(makeControl(0,0,MODE_MANUAL,0,0,0,0,0), buf), 8);
    TelemetryDriveL dl = {0, 0};
    CHECK_EQ(encodeTelemetryDriveL(dl, 0, buf), 8);
    TelemetryDriveR dr = {0, 0};
    CHECK_EQ(encodeTelemetryDriveR(dr, 0, buf), 8);
    TelemetryPower pw = {0, 0, 0};
    CHECK_EQ(encodeTelemetryPower(pw, 0, buf), 8);
    TelemetryState st = {0, 0, 0, 0, 0, 0};
    CHECK_EQ(encodeTelemetryState(st, 0, buf), 8);
}

static void control_outranks_telemetry_on_the_bus() {
    // CAN arbitration is dominant-low: the numerically lowest id wins. A late
    // command can hurt the rover; late telemetry only annoys an operator.
    CHECK(CAN_ID_CONTROL < CAN_ID_TELEM_DRIVE_L);
    CHECK(CAN_ID_CONTROL < CAN_ID_TELEM_DRIVE_R);
    CHECK(CAN_ID_CONTROL < CAN_ID_TELEM_POWER);
    CHECK(CAN_ID_CONTROL < CAN_ID_TELEM_STATE);
    // Headroom below CONTROL reserved for a future dedicated e-stop frame.
    CHECK(CAN_ID_CONTROL >= 0x100);
}

static void telemetry_ids_are_distinct() {
    CHECK(CAN_ID_TELEM_DRIVE_L != CAN_ID_TELEM_DRIVE_R);
    CHECK(CAN_ID_TELEM_DRIVE_R != CAN_ID_TELEM_POWER);
    CHECK(CAN_ID_TELEM_POWER   != CAN_ID_TELEM_STATE);
}

// --- range handling ---------------------------------------------------------

static void encoder_wrap_avoids_undefined_behaviour() {
    // In Python an unwrapped counter raised inside struct.pack. In C++ signed
    // overflow is UNDEFINED BEHAVIOUR, which is worse: no exception, just a
    // compiler free to do anything. CI runs this under UBSan.
    CHECK_EQ(wrapI32(static_cast<int64_t>(INT32_MAX_V) + 1), INT32_MIN_V);
    CHECK_EQ(wrapI32(static_cast<int64_t>(INT32_MIN_V) - 1), INT32_MAX_V);
    CHECK_EQ(wrapI32(0), 0);
    CHECK_EQ(wrapI32(123456), 123456);
    CHECK_EQ(wrapI32(4294967296LL), 0);
}

static void cmd_age_sentinel_is_distinct_from_saturation() {
    // "never spoken to me" and "stopped speaking 65 s ago" are different
    // faults with different fixes, so they must be different values.
    CHECK(CMD_AGE_UNKNOWN != CMD_AGE_MAX);
    CHECK_EQ(clampCmdAgeMs(1000000000LL), CMD_AGE_MAX);
    CHECK(clampCmdAgeMs(1000000000LL) < CMD_AGE_UNKNOWN);
    CHECK_EQ(clampCmdAgeMs(-5), 0);
    CHECK_EQ(clampCmdAgeMs(250), 250);
}

static void fault_names_includes_the_new_wide_bits() {
    const char* names[16];
    size_t n = faultNames(static_cast<uint16_t>(FAULT_SEQ_GAP | FAULT_CRC_ERROR),
                          names, 16);
    CHECK_EQ(n, static_cast<size_t>(2));
    CHECK_STREQ(names[0], "SEQ_GAP");
    CHECK_STREQ(names[1], "CRC_ERROR");
    CHECK_EQ(faultNames(0, names, 16), static_cast<size_t>(0));
}

static void the_c2_fault_bit_is_named() {
    // 0x0080, which v0.4 had left reserved, is FAULT_C2_LINK_LOST. The value
    // is pinned so the C++ and Python fault words cannot drift apart.
    CHECK_EQ(static_cast<unsigned>(FAULT_C2_LINK_LOST), 0x0080u);
    const char* names[16];
    CHECK_EQ(faultNames(FAULT_C2_LINK_LOST, names, 16), static_cast<size_t>(1));
    CHECK(std::strcmp(names[0], "C2_LINK_LOST") == 0);
}

int main() {
    std::printf("test_protocol\n");
    RUN_TEST(crc8_known_answer_vector);
    RUN_TEST(crc8_is_not_a_different_variant);
    RUN_TEST(crc8_of_nothing_is_the_init_xored);
    RUN_TEST(crc8_detects_every_single_bit_flip);
    RUN_TEST(frame_crc_is_seeded_with_the_can_id);
    RUN_TEST(frame_crc_is_stable_for_the_same_inputs);
    RUN_TEST(seq_delta_is_correct_across_the_wrap);
    RUN_TEST(control_round_trips_every_field);
    RUN_TEST(command_range_predicate_holds_at_the_boundaries);
    RUN_TEST(out_of_range_commands_are_rejected_at_decode);
    RUN_TEST(the_encoder_still_writes_any_int16);
    RUN_TEST(control_flags_are_independent);
    RUN_TEST(indicator_request_survives_alongside_flags);
    RUN_TEST(telemetry_drive_l_round_trips);
    RUN_TEST(telemetry_drive_r_round_trips);
    RUN_TEST(telemetry_power_round_trips_signed_and_wide_faults);
    RUN_TEST(telemetry_state_round_trips);
    RUN_TEST(wrong_dlc_is_rejected);
    RUN_TEST(a_corrupted_byte_is_caught_by_the_app_layer_crc);
    RUN_TEST(a_frame_on_the_wrong_can_id_fails);
    RUN_TEST(mode_predicates_cover_all_256_values);
    RUN_TEST(disabled_is_known_but_not_drivable);
    RUN_TEST(undefined_mode_is_rejected_at_decode);
    RUN_TEST(reserved_flag_bits_are_rejected);
    RUN_TEST(the_c2_flag_bit_is_a_field_not_a_reserved_bit);
    RUN_TEST(indicator_predicate_covers_all_256_values);
    RUN_TEST(undefined_indicator_request_is_rejected);
    RUN_TEST(every_frame_is_exactly_eight_bytes);
    RUN_TEST(control_outranks_telemetry_on_the_bus);
    RUN_TEST(telemetry_ids_are_distinct);
    RUN_TEST(encoder_wrap_avoids_undefined_behaviour);
    RUN_TEST(cmd_age_sentinel_is_distinct_from_saturation);
    RUN_TEST(fault_names_includes_the_new_wide_bits);
    RUN_TEST(the_c2_fault_bit_is_named);
    return testing::summary("test_protocol");
}
