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
    // Assume the worst about C2 too, matching every other boot default. It is
    // only visible through c2LinkState(), which reports NOT_REPORTED until a
    // real frame arrives, so this cannot masquerade as forwarded state.
    last_control_.c2_lost           = true;
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

        // autonomy abort is almost the same instruction as stop on mcu but hardware and jetson implementations will differ greatly
        if (last_control_.autonomy_abort && last_control_.mode == MODE_AUTONOMOUS) {last_control_.mode = MODE_MANUAL;}
        if (last_control_.stop) {last_control_.mode = MODE_DISABLED;}
        
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
        || (last_control_.autonomy_abort && last_control_.mode == MODE_AUTONOMOUS)
        // C2 lost in MANUAL: the operator's commands can no longer reach the
        // Jetson, so nobody is driving. In AUTONOMOUS the Jetson IS driving
        // and is provably alive, or the watchdog above would already have
        // stopped us -- and the autonomy course deliberately runs out of
        // base-station line of sight. In DISABLED we are stopped anyway.
        || (last_control_.c2_lost && last_control_.mode == MODE_MANUAL);
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

uint8_t RoverController::c2LinkState() const {
    // Never derived from the Jetson link -- only ever reported. Two different
    // reasons to say "not reported", and neither may read as LINK_OK.
    //
    // This body is what a controller-side radio replaces; see the header and
    // PROTOCOL.md 3.5.2. A locally measured link cannot go stale, so the
    // second case below collapses into the first once that lands -- but only
    // for the local reading. A forwarded bit kept as a cross-check is still
    // stale-able and still needs this rule.
    if (!have_control_)    return LINK_NOT_REPORTED;  // nobody has told us
    if (watchdogTripped()) return LINK_NOT_REPORTED;  // what we were told is stale
    return last_control_.c2_lost ? LINK_LOST : LINK_OK;
}

uint16_t RoverController::faultWord(uint16_t sensor_faults, uint16_t extra_faults) const {
    uint16_t faults = static_cast<uint16_t>(sensor_faults | extra_faults);
    if (watchdogTripped()) faults |= FAULT_COMM_TIMEOUT;
    // Without these the rover could halt or degrade while telemetry read "no
    // faults, link healthy" -- a silent stop is its own hazard.
    if (protocol_error_)   faults |= FAULT_PROTOCOL_ERROR;
    if (crc_error_)        faults |= FAULT_CRC_ERROR;
    if (seq_gap_)          faults |= FAULT_SEQ_GAP;
    // Raised in EVERY mode, even though it only stops the rover in MANUAL.
    // During an autonomous run it is the one sign that the rover is out of
    // contact, which an operator needs whether or not it changes the driving.
    // Suppressed once the value is stale: see c2LinkState().
    if (c2LinkState() == LINK_LOST) faults |= FAULT_C2_LINK_LOST;
    return faults;
}

uint8_t RoverController::indicatorState() const {
    // URC requires the light to show what the rover is ACTUALLY doing -- red
    // for autonomous, blue for teleoperation, flashing green on arrival -- and
    // the controller knows its own mode. So the Jetson's request is honoured
    // only where it agrees with that mode; otherwise the controller overrides
    // it. Checked top to bottom, first match wins. PROTOCOL.md 3.5.

    // 1. Nothing is driving this rover, or nothing can reach it. Not "off":
    //    OFF is a legitimate idle state and this is a failure, so say so.
    if (watchdogTripped()) return INDICATOR_FAULT;
    if (last_control_.c2_lost && last_control_.mode == MODE_MANUAL) {
        return INDICATOR_FAULT;
    }

    // 2. Commanded not to move. A legitimate state, not a fault.
    if (last_control_.mode == MODE_DISABLED) return INDICATOR_OFF;

    // 3. A human-driven rover never shows red or green, whatever is requested.
    if (last_control_.mode == MODE_MANUAL) return INDICATOR_TELEOP;

    // 4. Autonomous. Only the ARRIVED request matters; anything else is red.
    //    `stop` deliberately does not change the light: a paused autonomous
    //    rover is still under autonomous operation.

    // indicator byte hijacked for returning a confirmation of return_request sent by jetson
    if (last_control_.return_request) return INDICATOR_ARRIVED;
  
    if (last_control_.indicator_request == INDICATOR_ARRIVED) return INDICATOR_ARRIVED;
    return INDICATOR_AUTONOMOUS;
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

TelemetryState RoverController::buildState(uint8_t controller_health) const {
    TelemetryState m;
    // An ECHO of the mode the controller believes it is in, so the operator
    // can confirm the controller agrees with what was commanded. Without it a
    // dropped or misread mode change is invisible from the outside.
    m.mode              = last_control_.mode;
    m.jetson_link       = jetsonLinkState();
    // Reported, never derived from the Jetson link: they are separate links
    // and must not be treated as equivalent.
    m.c2_link           = c2LinkState();
    m.controller_health = controller_health;
    m.indicator_state   = indicatorState();
    m.reserved          = 0;
    return m;
}

}  // namespace rover
