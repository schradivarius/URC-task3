// golden_vectors.cpp -- emit encoded frames as hex, for cross-language pinning.
//
// The protocol codec is the one thing that must exist in both C++ (Teensy) and
// Python (Jetson), because neither can import the other. This program prints
// what the C++ encoder produces; tests/host/test_golden_vectors.py encodes the
// same inputs in Python and asserts the bytes are identical.
//
// That turns "we were careful to keep them in sync" into a CI failure the
// moment someone changes a field width, order or endianness on one side only.
//
// Output format, one vector per line:   NAME|arg,arg,...|hexbytes

#include <cstdio>
#include "../firmware/src/rover_protocol.h"

using namespace rover;

static void emit(const char* name, const char* args, const uint8_t* buf, uint8_t len) {
    std::printf("%s|%s|", name, args);
    for (uint8_t i = 0; i < len; ++i) std::printf("%02x", buf[i]);
    std::printf("\n");
}

int main() {
    uint8_t buf[8];
    char args[128];

    struct { int16_t d, s; uint8_t m, st; } controls[] = {
        {0, 0, MODE_DISABLED, 1},
        {500, -200, MODE_MANUAL, 0},
        {1000, -1000, MODE_AUTONOMOUS, 0},
        {-1000, 1000, MODE_MANUAL, 1},
        {-32768, 32767, MODE_MANUAL, 0},
        {-21846, 170, MODE_MANUAL, 0},     // 0xAAAA: the old UART false-sync case
    };
    for (auto& c : controls) {
        ControlMsg m = {c.d, c.s, c.m, c.st, INDICATOR_OFF};
        uint8_t n = encodeControl(m, buf);
        std::snprintf(args, sizeof(args), "%d,%d,%u,%u", c.d, c.s, c.m, c.st);
        emit("CONTROL", args, buf, n);
    }

    long long motions[][2] = {
        {0, 0}, {1, -1}, {123456, -123456},
        {INT32_MAX_V, INT32_MIN_V},
        {2147483648LL, -2147483649LL},     // deliberately out of range: wraps
    };
    for (auto& mo : motions) {
        TelemetryMotion m = {wrapI32(mo[0]), wrapI32(mo[1])};
        uint8_t n = encodeTelemetryMotion(m, buf);
        std::snprintf(args, sizeof(args), "%lld,%lld", mo[0], mo[1]);
        emit("TELEM_MOTION", args, buf, n);
    }

    // Mode predicates for all 256 values, so the Jetson and the Teensy cannot
    // disagree about which modes permit motion (issue #4). A host that thinks
    // mode 3 is drivable while the controller stops on it is its own bug.
    for (int m = 0; m <= 255; ++m) {
        std::printf("MODE|%d|%d%d\n", m,
                    isKnownMode((uint8_t)m) ? 1 : 0,
                    modePermitsMotion((uint8_t)m) ? 1 : 0);
    }

    // Command-range predicate at and around the boundaries, so the Jetson
    // cannot think a value is sendable while the controller rejects it.
    const int16_t range_cases[] = {-32768, -1001, -1000, -999, 0, 999, 1000, 1001, 32767};
    for (int16_t v : range_cases) {
        std::printf("RANGE|%d|%d\n", v, isValidCommand(v) ? 1 : 0);
    }

    struct { int16_t fb, ca; uint8_t f; uint16_t age; } statuses[] = {
        {0, 0, 0, 0},
        {-1000, -2500, FAULT_OVER_CURRENT, 42},
        {1000, 32767, (uint8_t)(FAULT_COMM_TIMEOUT | FAULT_FIRMWARE_FAULT), CMD_AGE_MAX},
        {-1, -10000, 0xFF, CMD_AGE_UNKNOWN},
    };
    for (auto& s : statuses) {
        TelemetryStatus m = {s.fb, s.ca, s.f, s.age, INDICATOR_OFF};
        uint8_t n = encodeTelemetryStatus(m, buf);
        std::snprintf(args, sizeof(args), "%d,%d,%u,%u", s.fb, s.ca, s.f, s.age);
        emit("TELEM_STATUS", args, buf, n);
    }
    return 0;
}
