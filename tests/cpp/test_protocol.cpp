// test_protocol.cpp -- wire format tests for the rover CAN messages.
//
// Every test is named for the failure it prevents, so the reason it exists
// outlives anyone's memory of writing it.

#include "../../firmware/src/rover_protocol.h"
#include "test_framework.h"

using namespace rover;

// --- round trip -------------------------------------------------------------

static void control_round_trips() {
    const ControlMsg cases[] = {
        {     0,     0, MODE_DISABLED,   1},
        {  1000, -1000, MODE_AUTONOMOUS, 0},
        { -1000,  1000, MODE_MANUAL,     1},
        {-32768, 32767, MODE_MANUAL,     0},   // int16 extremes
    };
    for (const ControlMsg& in : cases) {
        uint8_t buf[8] = {0};
        uint8_t dlc = encodeControl(in, buf);
        CHECK_EQ(dlc, CONTROL_DLC);
        ControlMsg out;
        CHECK(decodeControl(buf, dlc, out));
        CHECK_EQ(out.drive_cmd, in.drive_cmd);
        CHECK_EQ(out.steer_cmd, in.steer_cmd);
        CHECK_EQ(out.mode, in.mode);
        CHECK_EQ(out.stop, in.stop);
    }
}

static void telemetry_motion_round_trips() {
    const TelemetryMotion in = {-123456, 123456};
    uint8_t buf[8] = {0};
    uint8_t dlc = encodeTelemetryMotion(in, buf);
    CHECK_EQ(dlc, TELEM_MOTION_DLC);
    TelemetryMotion out;
    CHECK(decodeTelemetryMotion(buf, dlc, out));
    CHECK_EQ(out.enc_left, in.enc_left);
    CHECK_EQ(out.enc_right, in.enc_right);
}

static void telemetry_status_round_trips() {
    const TelemetryStatus in = {-1000, -2500,
                                static_cast<uint8_t>(FAULT_OVER_CURRENT), 42};
    uint8_t buf[8] = {0};
    uint8_t dlc = encodeTelemetryStatus(in, buf);
    CHECK_EQ(dlc, TELEM_STATUS_DLC);
    TelemetryStatus out;
    CHECK(decodeTelemetryStatus(buf, dlc, out));
    CHECK_EQ(out.steer_fb, in.steer_fb);
    CHECK_EQ(out.current_ca, in.current_ca);
    CHECK_EQ(out.fault_status, in.fault_status);
    CHECK_EQ(out.cmd_age_ms, in.cmd_age_ms);
}

static void current_is_signed_for_regenerative_braking() {
    // Unsigned current would wrap a braking motor's negative reading into a
    // huge positive value -- the worst kind of bad telemetry, because it looks
    // entirely plausible on a dashboard.
    const TelemetryStatus in = {0, -10000, 0, 0};   // -100.00 A
    uint8_t buf[8] = {0};
    encodeTelemetryStatus(in, buf);
    TelemetryStatus out;
    CHECK(decodeTelemetryStatus(buf, TELEM_STATUS_DLC, out));
    CHECK_EQ(out.current_ca, -10000);
}

static void int32_extremes_survive_the_wire() {
    const TelemetryMotion in = {INT32_MIN_V, INT32_MAX_V};
    uint8_t buf[8] = {0};
    encodeTelemetryMotion(in, buf);
    TelemetryMotion out;
    CHECK(decodeTelemetryMotion(buf, TELEM_MOTION_DLC, out));
    CHECK_EQ(out.enc_left, INT32_MIN_V);
    CHECK_EQ(out.enc_right, INT32_MAX_V);
}

// --- DLC validation ---------------------------------------------------------

static void wrong_dlc_is_rejected() {
    // CAN guarantees the frame is intact; it cannot tell us the sender agrees
    // on what the bytes MEAN. A DLC mismatch is the one cheap signal that a
    // peer is running a different protocol version, so it must be checked.
    uint8_t buf[8] = {0};
    encodeControl({100, 0, MODE_MANUAL, 0}, buf);
    ControlMsg out;
    for (uint8_t bad_len = 0; bad_len <= 8; ++bad_len) {
        if (bad_len == CONTROL_DLC) continue;
        CHECK(!decodeControl(buf, bad_len, out));
    }
    CHECK(decodeControl(buf, CONTROL_DLC, out));   // the right one still works
}

static void every_message_fits_classic_can() {
    // Designing a >8-byte frame would lock every node on the bus into
    // CAN FD transceivers, including Classic-only motor controllers.
    CHECK(CONTROL_DLC <= 8);
    CHECK(TELEM_MOTION_DLC <= 8);
    CHECK(TELEM_STATUS_DLC <= 8);
}

static void control_outranks_telemetry_on_the_bus() {
    // CAN arbitration is dominant-low: the numerically lowest id wins. A late
    // command can hurt the rover; late telemetry only annoys an operator.
    CHECK(CAN_ID_CONTROL < CAN_ID_TELEM_MOTION);
    CHECK(CAN_ID_CONTROL < CAN_ID_TELEM_STATUS);
    // Headroom left below CONTROL for a future dedicated e-stop frame.
    CHECK(CAN_ID_CONTROL > 0x000);
}

// --- range handling ---------------------------------------------------------

static void encoder_wrap_avoids_undefined_behaviour() {
    // In Python an unwrapped counter raised inside struct.pack and killed the
    // loop with the motors live. In C++ signed overflow is UNDEFINED
    // BEHAVIOUR, which is worse: no exception, just a compiler free to do
    // anything. wrapI32 takes int64 and wraps explicitly.
    CHECK_EQ(wrapI32(static_cast<int64_t>(INT32_MAX_V) + 1), INT32_MIN_V);
    CHECK_EQ(wrapI32(static_cast<int64_t>(INT32_MIN_V) - 1), INT32_MAX_V);
    CHECK_EQ(wrapI32(0), 0);
    CHECK_EQ(wrapI32(123456), 123456);
    CHECK_EQ(wrapI32(INT32_MAX_V), INT32_MAX_V);
    CHECK_EQ(wrapI32(INT32_MIN_V), INT32_MIN_V);
    CHECK_EQ(wrapI32(4294967296LL), 0);            // exactly 2^32
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

static void fault_names_reports_combined_faults() {
    const char* names[8];
    size_t n = faultNames(FAULT_COMM_TIMEOUT | FAULT_OVER_CURRENT, names, 8);
    CHECK_EQ(n, static_cast<size_t>(2));
    CHECK_STREQ(names[0], "COMM_TIMEOUT");
    CHECK_STREQ(names[1], "OVER_CURRENT");
    CHECK_EQ(faultNames(0, names, 8), static_cast<size_t>(0));
}

int main() {
    std::printf("test_protocol\n");
    RUN_TEST(control_round_trips);
    RUN_TEST(telemetry_motion_round_trips);
    RUN_TEST(telemetry_status_round_trips);
    RUN_TEST(current_is_signed_for_regenerative_braking);
    RUN_TEST(int32_extremes_survive_the_wire);
    RUN_TEST(wrong_dlc_is_rejected);
    RUN_TEST(every_message_fits_classic_can);
    RUN_TEST(control_outranks_telemetry_on_the_bus);
    RUN_TEST(encoder_wrap_avoids_undefined_behaviour);
    RUN_TEST(cmd_age_sentinel_is_distinct_from_saturation);
    RUN_TEST(fault_names_reports_combined_faults);
    return testing::summary("test_protocol");
}
