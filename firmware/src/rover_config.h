// rover_config.h -- Every protocol and timing constant, in one place.
//
// PURE C++, like the rest of firmware/src: compiles for the Teensy and
// natively for the tests. host/rover_config.py mirrors this file, and
// tests/host/test_config_sync.py fails if the two ever disagree.
//
// What belongs here: values someone might tune or that both ends must agree
// on (bit rate, timeouts, CAN ids, payload sizes, valid ranges, version).
// What does not: values that describe a message's CONTENTS (fault bits,
// indicator codes, command-age sentinels) -- those stay in rover_protocol.h.

#ifndef ROVER_CONFIG_H
#define ROVER_CONFIG_H

#include <stdint.h>

namespace rover {

// --- Protocol version ------------------------------------------------------
// Bump whenever a message's layout or meaning changes. Not carried on the
// wire yet; the DLC check is still the only runtime mismatch signal.
static const uint8_t PROTOCOL_VERSION = 1;

// --- Link ------------------------------------------------------------------
static const uint32_t CAN_BITRATE_HZ = 500000;   // 500 kbps; every node must match

// --- Timing ----------------------------------------------------------------
static const uint32_t DEFAULT_WATCHDOG_TIMEOUT_MS = 300;   // no valid CONTROL -> stop
static const uint32_t DEFAULT_TELEMETRY_PERIOD_MS = 50;    // 20 Hz
static const uint32_t HW_WATCHDOG_MS              = 1000;  // Teensy resets if the loop stalls

// --- CAN identifiers -------------------------------------------------------
// On CAN the identifier is ALSO the priority: arbitration is bitwise and
// dominant-low, so the NUMERICALLY LOWEST id wins the bus. Commands outrank
// telemetry because a late command can hurt the rover and late telemetry only
// annoys an operator. 0x000-0x0FF is left free for a future e-stop frame.
enum : uint32_t {
    CAN_ID_CONTROL       = 0x100,  // Jetson -> Teensy, highest priority in use
    CAN_ID_TELEM_MOTION  = 0x200,  // Teensy -> Jetson, encoders
    CAN_ID_TELEM_STATUS  = 0x201,  // Teensy -> Jetson, steering/current/faults
};

// --- Payload sizes (DLC) ---------------------------------------------------
// Every message fits in 8 bytes ON PURPOSE: that is Classic CAN's limit, so
// this protocol runs unchanged on a Classic bus (CAN1/CAN2) or a CAN FD bus
// (CAN3), alongside Classic-only motor controllers.
enum : uint8_t {
    CONTROL_DLC      = 8,
    TELEM_MOTION_DLC = 8,
    TELEM_STATUS_DLC = 8,
};

// --- Operating modes (ControlMsg::mode) ------------------------------------
enum : uint8_t {
    MODE_DISABLED   = 0,
    MODE_MANUAL     = 1,
    MODE_AUTONOMOUS = 2,
};
static const uint8_t MODE_MAX = MODE_AUTONOMOUS;   // highest defined mode

// --- Sensor thresholds -----------------------------------------------------
// Hundredths of an amp, matching TelemetryStatus::current_ca. A placeholder
// until the real current sensor is characterised, but it belongs here rather
// than in the .ino: it is a value someone will tune, not board wiring.
static const int16_t OVER_CURRENT_CA = 4000;   // 40.00 A

// --- Valid command ranges --------------------------------------------------
// drive_cmd and steer_cmd are tenths of a percent, so +/-1000 is full scale.
// Anything outside is rejected at decode (see isValidCommand()).
static const int16_t CMD_MIN = -1000;
static const int16_t CMD_MAX =  1000;

}  // namespace rover

#endif  // ROVER_CONFIG_H
