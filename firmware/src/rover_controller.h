// rover_controller.h -- The rover's command/safety state machine.
//
// PURE C++. No Arduino.h, no FlexCAN, no hardware, and the clock is injected.
// That is the whole point: this is the safety-critical code, so it must be
// testable natively with a fake clock rather than only observable by watching
// a board. tests/cpp/test_controller.cpp drives it with time it controls.
//
// This file is a near line-for-line port of the CircuitPython controller.py.
// It survived the move from UART to CAN untouched in substance, because none
// of it ever knew how bytes reached it -- which is exactly why it was a
// separate module in the first place.
//
// THE ONE REAL DIFFERENCE FROM THE PYTHON VERSION: millis() WRAPS.
//   CircuitPython's time.monotonic_ns() is a 64-bit nanosecond counter that
//   will not wrap in any mission this rover survives, so plain subtraction was
//   safe. Arduino/Teensy millis() is uint32_t and wraps every ~49.7 days.
//
//   Unsigned subtraction handles that correctly ON ITS OWN -- (now - then) is
//   computed modulo 2^32, so it stays right across the wrap, provided the
//   real interval is under 49.7 days. What is NOT safe is comparing
//   timestamps directly (now > deadline), which breaks the moment the counter
//   wraps past the deadline. So every time comparison in this file is written
//   as an elapsed-time subtraction, never as a comparison of two timestamps.
//   test_controller.cpp exercises this across the wrap boundary explicitly.

#ifndef ROVER_CONTROLLER_H
#define ROVER_CONTROLLER_H

#include <stdint.h>

#include "rover_protocol.h"

namespace rover {

// Injected clock. On the Teensy this is `millis`. In tests it is a counter
// the test advances by hand, so watchdog behaviour is checked deterministically
// instead of with sleeps.
typedef uint32_t (*MillisFn)();

static const uint32_t DEFAULT_WATCHDOG_TIMEOUT_MS = 300;
static const uint32_t DEFAULT_TELEMETRY_PERIOD_MS = 50;  // 20 Hz

class RoverController {
public:
    explicit RoverController(MillisFn now_ms,
                             uint32_t watchdog_timeout_ms = DEFAULT_WATCHDOG_TIMEOUT_MS,
                             uint32_t telemetry_period_ms = DEFAULT_TELEMETRY_PERIOD_MS);

    // Feed one received CAN frame. Returns true if it was a CONTROL frame that
    // was accepted (correct id AND correct DLC). Anything else is ignored and
    // -- critically -- does NOT refresh the watchdog.
    bool ingestFrame(uint32_t can_id, const uint8_t* buf, uint8_t len);

    // Milliseconds since the last accepted CONTROL frame, or CMD_AGE_UNKNOWN.
    uint16_t cmdAgeMs() const;

    // True if no valid CONTROL frame has arrived within the watchdog timeout.
    // True at boot, before anything has ever arrived.
    bool watchdogTripped() const;

    // THE fail-safe decision. A comm timeout, an explicit stop flag, and any
    // mode that does not positively permit motion all force a stop through
    // this one function, so there is exactly one place where "should the rover
    // be moving?" is answered.
    //
    // The mode test is a whitelist (modePermitsMotion), not "== DISABLED".
    // Asking only about DISABLED let every undefined mode value read as
    // drivable -- see issue #4 and the note in rover_protocol.h.
    bool effectiveStop() const;

    uint8_t indicatorState() const;

    // The drive/steer actually permitted right now: zeroed under stop, so a
    // caller cannot accidentally act on a stale command.
    void commandedOutputs(int16_t& drive, int16_t& steer) const;

    // True once per telemetry period. Self-pacing, and it will not burst out a
    // backlog after a stalled loop.
    bool telemetryDue();

    // Assemble the status frame, OR-ing in COMM_TIMEOUT when the watchdog has
    // tripped so the fault can never be reported inconsistently with cmdAgeMs.
    TelemetryStatus buildStatus(int16_t steer_fb, int16_t current_ca,
                                uint8_t sensor_faults = 0,
                                uint8_t extra_faults = 0) const;

    const ControlMsg& lastControl() const { return last_control_; }
    uint32_t controlFramesAccepted() const { return control_frames_accepted_; }
    uint32_t framesIgnored() const { return frames_ignored_; }

    // True when the most recent frame addressed to CAN_ID_CONTROL could not be
    // interpreted (wrong DLC, or a mode this firmware does not define). Clears
    // when a valid frame arrives, so it reports a live condition rather than
    // latching for the session. Surfaced as FAULT_PROTOCOL_ERROR.
    bool protocolError() const { return protocol_error_; }

private:
    uint32_t elapsedSinceControl() const;

    MillisFn now_ms_;
    uint32_t watchdog_timeout_ms_;
    uint32_t telemetry_period_ms_;

    ControlMsg last_control_;
    bool       have_control_;
    uint32_t   last_control_ms_;
    uint32_t   next_telemetry_ms_;
    uint32_t   control_frames_accepted_;
    uint32_t   frames_ignored_;
    bool       protocol_error_;
};

}  // namespace rover

#endif  // ROVER_CONTROLLER_H
