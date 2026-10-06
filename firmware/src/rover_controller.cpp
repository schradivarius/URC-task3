#include "rover_controller.h"

namespace rover {

RoverController::RoverController(MillisFn now_ms,
                                 uint32_t watchdog_timeout_ms,
                                 uint32_t telemetry_period_ms)
    : now_ms_(now_ms),
      watchdog_timeout_ms_(watchdog_timeout_ms),
      telemetry_period_ms_(telemetry_period_ms),
      have_control_(false),
      last_control_ms_(0),
      next_telemetry_ms_(now_ms ? now_ms() : 0),
      telemetry_seq_(0),
      have_seq_(false),
      last_seq_(0),
      last_decode_(DECODE_OK),
      protocol_error_(false),
      crc_error_(false),
      seq_gap_(false),
      last_degraded_ms_(0),
      ever_degraded_(false),
      control_frames_accepted_(0),
      frames_ignored_(0),
      seq_gaps_seen_(0),
      crc_errors_seen_(0),
      frames_lost_estimate_(0) {
    // Boot state is the safest available: stopped, disabled, no command ever
    // received -- so cmdAgeMs() reads CMD_AGE_UNKNOWN, the watchdog is tripped
    // from the first cycle, and the rover cannot move until the Jetson asks.
    last_control_.drive_cmd         = 0;
    last_control_.steer_cmd         = 0;
    last_control_.mode              = MODE_DISABLED;
    last_control_.stop              = true;
    last_control_.autonomy_abort    = false;
    last_control_.return_request    = false;
    last_control_.indicator_request = INDICATOR_OFF;
    last_control_.seq               = 0;
}

bool RoverController::ingestFrame(uint32_t can_id, const uint8_t* buf, uint8_t len) {
    // Other traffic on a shared bus is normal, not a fault. Counting it as one
    // would make the protocol-error flag permanently on and therefore useless.
    if (can_id != CAN_ID_CONTROL) { frames_ignored_++; return false; }

    ControlMsg msg;
    const DecodeResult r = decodeControl(can_id, buf, len, msg);
    last_decode_ = r;

    if (r != DECODE_OK) {
        // The peer disagrees with us about the protocol -- a bad CRC, a wrong
        // length, an undefined mode, reserved bits set. It must NOT refresh
        // the watchdog, or a mismatched node could keep the rover alive while
        // sending commands it never actually understood.
        frames_ignored_++;
        if (r == DECODE_BAD_CRC) {
            crc_error_ = true;
            crc_errors_seen_++;
            last_degraded_ms_ = now_ms_();
            ever_degraded_ = true;
        } else {
            protocol_error_ = true;
        }
        return false;
    }

    // Sequence check. seqDelta is an unsigned subtraction, so it is correct
    // across the 255->0 wrap without a special case -- the same "subtract,
    // never compare" rule as the millis() handling.
    //   delta == 1  healthy
    //   delta == 0  duplicate (or a 256-frame loss, which is ~13 s at 20 Hz
    //               and would have tripped the watchdog long before)
    //   delta >  1  delta-1 frames were lost or arrived out of order
    if (have_seq_) {
        const uint8_t delta = seqDelta(last_seq_, msg.seq);
        if (delta != 1) {
            seq_gap_ = true;
            seq_gaps_seen_++;
            if (delta > 1) frames_lost_estimate_ += static_cast<uint32_t>(delta - 1);
            last_degraded_ms_ = now_ms_();
            ever_degraded_ = true;
        } else {
            seq_gap_ = false;
        }
    }
    last_seq_ = msg.seq;
    have_seq_ = true;

    protocol_error_  = false;   // a good frame clears the condition
    crc_error_       = false;
    last_control_    = msg;
    have_control_    = true;
    last_control_ms_ = now_ms_();
    control_frames_accepted_++;
    return true;
}

// Unsigned subtraction, computed modulo 2^32, so this stays correct when
// millis() wraps. Never written as a comparison of two timestamps.
uint32_t RoverController::elapsedSinceControl() const {
    return now_ms_() - last_control_ms_;
}

uint16_t RoverController::cmdAgeMs() const {
    if (!have_control_) return CMD_AGE_UNKNOWN;
    return clampCmdAgeMs(static_cast<int64_t>(elapsedSinceControl()));
}

bool RoverController::watchdogTripped() const {
    if (!have_control_) return true;
    return elapsedSinceControl() >= watchdog_timeout_ms_;
}

bool RoverController::effectiveStop() const {
    return watchdogTripped()
        || last_control_.stop
        || !modePermitsMotion(last_control_.mode)
        // An autonomy abort means "stop what you are doing autonomously". In
        // AUTONOMOUS that is a stop; in MANUAL the operator is already in
        // control so it has no autonomous task to abort.
        //
        // NOTE FOR REVIEW: this behaviour is a judgement call, not something
        // the requirement spelled out. The alternative is to expose the flag
        // and let a higher layer act on it -- but a defined field with no
        // effect anywhere is the antipattern we already had to fix once with
        // FAULT_FIRMWARE_FAULT, so it acts here until someone decides better.
        || (last_control_.autonomy_abort && last_control_.mode == MODE_AUTONOMOUS);
}

void RoverController::commandedOutputs(int16_t& drive, int16_t& steer) const {
    if (effectiveStop()) { drive = 0; steer = 0; return; }
    drive = last_control_.drive_cmd;
    steer = last_control_.steer_cmd;
}

bool RoverController::telemetryDue() {
    const uint32_t now = now_ms_();
    // Signed difference of unsigned timestamps: correct across wraparound, and
    // negative while the deadline is still in the future.
    const int32_t since_due = static_cast<int32_t>(now - next_telemetry_ms_);
    if (since_due < 0) return false;

    next_telemetry_ms_ += telemetry_period_ms_;
    // Never let a stalled loop accumulate a backlog it then bursts out.
    if (static_cast<int32_t>(now - next_telemetry_ms_)
            > static_cast<int32_t>(telemetry_period_ms_)) {
        next_telemetry_ms_ = now + telemetry_period_ms_;
    }
    telemetry_seq_++;   // wraps; all four frames of this cycle share it
    return true;
}

bool RoverController::recentlyDegraded() const {
    if (!ever_degraded_) return false;
    return (now_ms_() - last_degraded_ms_) < LINK_DEGRADED_HOLD_MS;
}

uint8_t RoverController::jetsonLinkState() const {
    if (watchdogTripped()) return LINK_LOST;
    if (recentlyDegraded()) return LINK_DEGRADED;
    return LINK_OK;
}

uint16_t RoverController::faultWord(uint16_t sensor_faults, uint16_t extra_faults) const {
    uint16_t faults = static_cast<uint16_t>(sensor_faults | extra_faults);
    if (watchdogTripped()) faults |= FAULT_COMM_TIMEOUT;
    // Without these the rover could halt or degrade while telemetry read "no
    // faults, link healthy" -- a silent stop is its own hazard.
    if (protocol_error_)   faults |= FAULT_PROTOCOL_ERROR;
    if (crc_error_)        faults |= FAULT_CRC_ERROR;
    if (seq_gap_)          faults |= FAULT_SEQ_GAP;
    return faults;
}

uint8_t RoverController::indicatorState() const {
    // A faulted rover must not display "autonomous" or "arrived". The fault
    // state wins over whatever the Jetson asked for.
    if (faultWord() != 0) return INDICATOR_FAULT;
    return last_control_.indicator_request;
}

TelemetryDriveL RoverController::buildDriveL(int32_t enc_left, int16_t steer_fb) const {
    TelemetryDriveL m;
    m.enc_left = enc_left;
    m.steer_fb = steer_fb;
    return m;
}

TelemetryDriveR RoverController::buildDriveR(int32_t enc_right) const {
    TelemetryDriveR m;
    m.enc_right  = enc_right;
    m.cmd_age_ms = cmdAgeMs();
    return m;
}

TelemetryPower RoverController::buildPower(int16_t current_ca, int16_t voltage_cv,
                                           uint16_t sensor_faults,
                                           uint16_t extra_faults) const {
    TelemetryPower m;
    m.current_ca   = current_ca;
    m.voltage_cv   = voltage_cv;
    m.fault_status = faultWord(sensor_faults, extra_faults);
    return m;
}

TelemetryState RoverController::buildState(uint8_t controller_health,
                                           uint8_t c2_link) const {
    TelemetryState m;
    // An ECHO of the mode the controller believes it is in, so the operator
    // can confirm the controller agrees with what was commanded. Without it a
    // dropped or misread mode change is invisible from the outside.
    m.mode              = last_control_.mode;
    m.jetson_link       = jetsonLinkState();
    // Reported, never derived from the Jetson link: they are separate links
    // and must not be treated as equivalent. Populating this from forwarded
    // C2 state is a separate work item; until then it is honestly
    // LINK_NOT_REPORTED rather than a misleading LINK_OK.
    m.c2_link           = c2_link;
    m.controller_health = controller_health;
    m.indicator_state   = indicatorState();
    m.reserved          = 0;
    return m;
}

}  // namespace rover
