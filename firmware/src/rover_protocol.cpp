// rover_protocol.cpp -- little-endian serialization for the rover CAN messages.
// See rover_protocol.h for the design rationale.

#include "rover_protocol.h"

namespace rover {
namespace {

// Explicit little-endian put/get. Byte-at-a-time rather than memcpy of a
// packed struct, so nothing here depends on compiler padding or host
// endianness. This is the C++ counterpart of Python's '<' struct prefix.

inline void putU16(uint8_t* b, uint16_t v) {
    b[0] = static_cast<uint8_t>(v & 0xFF);
    b[1] = static_cast<uint8_t>((v >> 8) & 0xFF);
}

inline void putU32(uint8_t* b, uint32_t v) {
    b[0] = static_cast<uint8_t>(v & 0xFF);
    b[1] = static_cast<uint8_t>((v >> 8) & 0xFF);
    b[2] = static_cast<uint8_t>((v >> 16) & 0xFF);
    b[3] = static_cast<uint8_t>((v >> 24) & 0xFF);
}

inline uint16_t getU16(const uint8_t* b) {
    return static_cast<uint16_t>(b[0]) | (static_cast<uint16_t>(b[1]) << 8);
}

inline uint32_t getU32(const uint8_t* b) {
    return static_cast<uint32_t>(b[0])
         | (static_cast<uint32_t>(b[1]) << 8)
         | (static_cast<uint32_t>(b[2]) << 16)
         | (static_cast<uint32_t>(b[3]) << 24);
}

// Signed values go over the wire as two's-complement bit patterns. Converting
// through the unsigned type is the defined way to do this: casting a
// too-large unsigned straight to a signed type is implementation-defined
// before C++20.
inline void putI16(uint8_t* b, int16_t v) { putU16(b, static_cast<uint16_t>(v)); }
inline void putI32(uint8_t* b, int32_t v) { putU32(b, static_cast<uint32_t>(v)); }

inline int16_t getI16(const uint8_t* b) {
    uint16_t u = getU16(b);
    return (u & 0x8000u) ? static_cast<int16_t>(static_cast<int32_t>(u) - 65536)
                         : static_cast<int16_t>(u);
}

inline int32_t getI32(const uint8_t* b) {
    uint32_t u = getU32(b);
    return (u & 0x80000000u)
        ? static_cast<int32_t>(static_cast<int64_t>(u) - 4294967296LL)
        : static_cast<int32_t>(u);
}

}  // namespace

bool isKnownMode(uint8_t mode) {
    return mode == MODE_DISABLED || mode == MODE_MANUAL || mode == MODE_AUTONOMOUS;
}

bool modePermitsMotion(uint8_t mode) {
    // Whitelist. Every value not named here -- including DISABLED and every
    // undefined value -- means stop. See the note in rover_protocol.h.
    return mode == MODE_MANUAL || mode == MODE_AUTONOMOUS;
}

int32_t wrapI32(int64_t value) {
    // Shift into unsigned range, wrap with a modulo that is well defined for
    // negatives, then shift back. Avoids signed overflow entirely, which in
    // C++ is undefined behaviour rather than a wrap.
    const int64_t span = 4294967296LL;                 // 2^32
    int64_t shifted = (value - INT32_MIN_V) % span;
    if (shifted < 0) shifted += span;
    return static_cast<int32_t>(shifted + INT32_MIN_V);
}

uint16_t clampCmdAgeMs(int64_t age_ms) {
    if (age_ms < 0) return 0;
    if (age_ms >= static_cast<int64_t>(CMD_AGE_MAX)) return CMD_AGE_MAX;
    return static_cast<uint16_t>(age_ms);
}

size_t faultNames(uint8_t bitmask, const char** out, size_t cap) {
    static const uint8_t bits[] = {
        FAULT_JETSON_HEARTBEAT_LOST, FAULT_OVER_CURRENT, FAULT_ESTOP_ACTIVE,
        FAULT_ENCODER_FAULT, FAULT_UNDERVOLTAGE, FAULT_FIRMWARE_FAULT,
        FAULT_PROTOCOL_ERROR, FAULT_C2_LINK_LOST,
    };
    static const char* names[] = {
        "JETSON_HEARTBEAT_LOST", "OVER_CURRENT", "ESTOP_ACTIVE",
        "ENCODER_FAULT", "UNDERVOLTAGE", "FIRMWARE_FAULT",
        "PROTOCOL_ERROR", "C2_LINK_LOST",
    };
    size_t n = 0;
    for (size_t i = 0; i < sizeof(bits) && n < cap; ++i) {
        if (bitmask & bits[i]) out[n++] = names[i];
    }
    return n;
}

// --- CONTROL ---------------------------------------------------------------

uint8_t encodeControl(const ControlMsg& msg, uint8_t* buf) {
    putI16(buf + 0, msg.drive_cmd);
    putI16(buf + 2, msg.steer_cmd);
    buf[4] = msg.mode;
    buf[5] = msg.stop ? 1 : 0;
    buf[6] = msg.c2_lost ? 1 : 0;
    return CONTROL_DLC;
}

bool decodeControl(const uint8_t* buf, uint8_t len, ControlMsg& out) {
    if (len != CONTROL_DLC) return false;
    // An undefined mode means the sender disagrees with us about the protocol,
    // exactly like a wrong DLC. Reject the whole frame rather than storing a
    // value we cannot reason about -- and, because a rejected frame does not
    // refresh the command watchdog, the rover stops via the existing path and
    // the operator gets a reported fault instead of a silent halt.
    //
    // This is intentionally NOT forward-compatible the way an unknown CAN id
    // is. A newer peer sending a mode we do not implement must stop this
    // rover, not be tolerated.
    if (!isKnownMode(buf[4])) return false;
    out.drive_cmd = getI16(buf + 0);
    out.steer_cmd = getI16(buf + 2);
    out.mode      = buf[4];
    out.stop      = buf[5] ? 1 : 0;
    out.c2_lost   = buf[6] ? 1 : 0;
    return true;
}

// --- TELEMETRY: motion -----------------------------------------------------

uint8_t encodeTelemetryMotion(const TelemetryMotion& msg, uint8_t* buf) {
    putI32(buf + 0, msg.enc_left);
    putI32(buf + 4, msg.enc_right);
    return TELEM_MOTION_DLC;
}

bool decodeTelemetryMotion(const uint8_t* buf, uint8_t len, TelemetryMotion& out) {
    if (len != TELEM_MOTION_DLC) return false;
    out.enc_left  = getI32(buf + 0);
    out.enc_right = getI32(buf + 4);
    return true;
}

// --- TELEMETRY: status -----------------------------------------------------

uint8_t encodeTelemetryStatus(const TelemetryStatus& msg, uint8_t* buf) {
    putI16(buf + 0, msg.steer_fb);
    putI16(buf + 2, msg.current_ca);
    buf[4] = msg.fault_status;
    putU16(buf + 5, msg.cmd_age_ms);
    return TELEM_STATUS_DLC;
}

bool decodeTelemetryStatus(const uint8_t* buf, uint8_t len, TelemetryStatus& out) {
    if (len != TELEM_STATUS_DLC) return false;
    out.steer_fb     = getI16(buf + 0);
    out.current_ca   = getI16(buf + 2);
    out.fault_status = buf[4];
    out.cmd_age_ms   = getU16(buf + 5);
    return true;
}

}  // namespace rover
