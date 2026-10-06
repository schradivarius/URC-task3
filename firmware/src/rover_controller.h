// rover_controller.h -- The rover's command/safety state machine.
//
// PURE C++. No Arduino.h, no FlexCAN, no hardware, and the clock is injected.
// That is the point: this is the safety-critical code, so it must be testable
// natively with a fake clock rather than only observable by watching a board.
//
// THE millis() WRAP, which the CircuitPython original did not have
//   CircuitPython's monotonic_ns() is a 64-bit nanosecond counter that will not
//   wrap in any mission this rover survives. Arduino millis() is uint32_t and
//   wraps every ~49.7 days. Unsigned subtraction handles that correctly on its
//   own -- (now - then) is computed modulo 2^32 -- but comparing timestamps
//   directly (now >= then + timeout) does NOT: it breaks the moment the counter
//   wraps past the deadline. So every time comparison here is an elapsed-time
//   SUBTRACTION, never a comparison of two timestamps. The same reasoning
//   applies to sequence numbers; see seqDelta().

#ifndef ROVER_CONTROLLER_H
#define ROVER_CONTROLLER_H

#include <stdint.h>

#include "rover_protocol.h"

namespace rover {

// Injected clock. On the Teensy this is `millis`. In tests it is a counter the
// test advances by hand, so watchdog behaviour is checked deterministically
// instead of with sleeps.
typedef uint32_t (*MillisFn)();

static const uint32_t DEFAULT_WATCHDOG_TIMEOUT_MS = 300;
static const uint32_t DEFAULT_TELEMETRY_PERIOD_MS = 50;   // 20 Hz

// How long a link-quality complaint (a sequence gap or a CRC error) keeps the
// Jetson link reported as DEGRADED after the last occurrence. Long enough that
// an operator actually sees it on a dashboard refreshing at 20 Hz.
static const uint32_t LINK_DEGRADED_HOLD_MS = 1000;

class RoverController {
public:
    explicit RoverController(MillisFn now_ms,
                             uint32_t watchdog_timeout_ms = DEFAULT_WATCHDOG_TIMEOUT_MS,
                             uint32_t telemetry_period_ms = DEFAULT_TELEMETRY_PERIOD_MS);

    // Feed one received CAN frame. Returns true only if it was a CONTROL frame
    // that fully validated (right id, right DLC, good CRC, known mode, clean
    // flags). Anything else is ignored and -- critically -- does NOT refresh
    // the command watchdog.
    bool ingestFrame(uint32_t can_id, const uint8_t* buf, uint8_t len);

    // --- safety ----------------------------------------------------------

    uint16_t cmdAgeMs() const;
    bool watchdogTripped() const;

    // THE fail-safe decision. A comm timeout, an explicit stop flag, any mode
    // that does not positively permit motion, and an autonomy abort while in
    // AUTONOMOUS all force a stop through this one function, so there is
    // exactly one place where "should the rover be moving?" is answered.
    //
    // The mode test is a whitelist (modePermitsMotion), not "== DISABLED":
    // asking only about DISABLED let every undefined mode read as drivable.
    bool effectiveStop() const;

    // The drive/steer actually permitted right now: zeroed under stop, so a
    // caller cannot accidentally act on a stale command.
    void commandedOutputs(int16_t& drive, int16_t& steer) const;

    // --- transmit scheduling ---------------------------------------------

    // True once per telemetry period. Self-pacing, and it will not burst out a
    // backlog after a stalled loop. Advances the telemetry sequence number, so
    // call it ONCE per cycle and stamp all four frames with telemetrySeq().
    bool telemetryDue();

    // The sequence number for the current cycle. All four telemetry frames of
    // one cycle share it, which is how the Jetson tells a coherent snapshot
    // from one torn across two cycles.
    uint8_t telemetrySeq() const { return telemetry_seq_; }

    // --- telemetry builders ----------------------------------------------

    TelemetryDriveL buildDriveL(int32_t enc_left, int16_t steer_fb) const;
    TelemetryDriveR buildDriveR(int32_t enc_right) const;
    TelemetryPower  buildPower(int16_t current_ca, int16_t voltage_cv,
                               uint16_t sensor_faults = 0,
                               uint16_t extra_faults = 0) const;
    TelemetryState  buildState(uint8_t controller_health = CTRL_HEALTH_NOT_REPORTED,
                               uint8_t c2_link = LINK_NOT_REPORTED) const;

    // Every fault the controller itself knows about, OR-ed together. Exposed
    // so the firmware can report the same word it acts on.
    uint16_t faultWord(uint16_t sensor_faults = 0, uint16_t extra_faults = 0) const;

    // --- link and indicator state ----------------------------------------

    // LINK_* for the Jetson link, derived from the command watchdog and from
    // recent sequence gaps or CRC errors. NOT merged with the C2 link: the
    // 2027 autonomy course deliberately includes areas with no C2
    // line-of-sight while onboard autonomy keeps working, so collapsing the
    // two would stop the rover exactly where it is supposed to keep going.
    uint8_t jetsonLinkState() const;

    // What the status indicator should actually show: the Jetson's request,
    // overridden to INDICATOR_FAULT whenever a fault is active, so the
    // indicator cannot cheerfully display "autonomous" on a faulted rover.
    uint8_t indicatorState() const;

    // --- accessors --------------------------------------------------------

    const ControlMsg& lastControl() const { return last_control_; }
    bool autonomyAbort() const { return last_control_.autonomy_abort; }
    bool returnRequest() const { return last_control_.return_request; }

    uint32_t controlFramesAccepted() const { return control_frames_accepted_; }
    uint32_t framesIgnored() const { return frames_ignored_; }
    uint32_t seqGapsSeen() const { return seq_gaps_seen_; }
    uint32_t crcErrorsSeen() const { return crc_errors_seen_; }
    uint32_t framesLostEstimate() const { return frames_lost_estimate_; }

    // The reason the most recent frame on CAN_ID_CONTROL was rejected, or
    // DECODE_OK if it was accepted.
    DecodeResult lastDecodeResult() const { return last_decode_; }

private:
    uint32_t elapsedSinceControl() const;
    bool recentlyDegraded() const;

    MillisFn now_ms_;
    uint32_t watchdog_timeout_ms_;
    uint32_t telemetry_period_ms_;

    ControlMsg last_control_;
    bool       have_control_;
    uint32_t   last_control_ms_;
    uint32_t   next_telemetry_ms_;
    uint8_t    telemetry_seq_;

    bool       have_seq_;
    uint8_t    last_seq_;

    DecodeResult last_decode_;
    bool       protocol_error_;
    bool       crc_error_;
    bool       seq_gap_;
    uint32_t   last_degraded_ms_;
    bool       ever_degraded_;

    uint32_t   control_frames_accepted_;
    uint32_t   frames_ignored_;
    uint32_t   seq_gaps_seen_;
    uint32_t   crc_errors_seen_;
    uint32_t   frames_lost_estimate_;
};

}  // namespace rover

#endif  // ROVER_CONTROLLER_H
