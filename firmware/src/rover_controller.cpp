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
      control_frames_accepted_(0),
      frames_ignored_(0),
      protocol_error_(false) {
    // Boot state is the safest available: stopped, in SAFE, and with no
    // command ever received -- so cmdAgeMs() reads CMD_AGE_UNKNOWN, the
    // watchdog is tripped from the very first cycle, and the rover cannot
    // move until the Jetson actually asks it to.
    last_control_.drive_cmd = 0;
    last_control_.steer_cmd = 0;
    last_control_.mode      = MODE_SAFE;
    last_control_.stop      = 1;
    last_control_.indicator_request = INDICATOR_OFF;
    last_control_.c2_lost   = 1;
}

bool RoverController::ingestFrame(uint32_t can_id, const uint8_t* buf, uint8_t len) {
    if (can_id != CAN_ID_CONTROL) { frames_ignored_++; return false; }

    ControlMsg msg;
    if (!decodeControl(buf, len, msg)) {
        // Right id, but the frame is uninterpretable -- a wrong DLC, or a mode
        // this firmware does not define. Either way the peer disagrees with us
        // about the protocol. It must NOT refresh the watchdog, or a
        // mismatched node could keep the rover alive while it acts on commands
        // it never actually understood.
        frames_ignored_++;
        protocol_error_ = true;
        return false;
    }

    protocol_error_  = false;        // a good frame clears the condition
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

uint8_t RoverController::activeMode() const {
    const bool jetson_lost   = watchdogTripped();
    const bool operator_lost = last_control_.mode == MODE_TELEOP && last_control_.c2_lost != 0;
    if (jetson_lost || operator_lost) return MODE_FAULT;
    return last_control_.mode;
}

bool RoverController::effectiveStop() const {
    return last_control_.stop != 0 || !modePermitsMotion(activeMode());
}

uint8_t RoverController::indicatorState() const {
    const bool arrived = last_control_.indicator_request == INDICATOR_GREEN_FLASH;
    switch (activeMode()) {
        case MODE_TELEOP:     return INDICATOR_BLUE;
        case MODE_AUTONOMOUS: return arrived ? INDICATOR_GREEN_FLASH : INDICATOR_RED;
        default:              return INDICATOR_OFF;    // MODE_SAFE or MODE_FAULT
    }
}

void RoverController::commandedOutputs(int16_t& drive, int16_t& steer) const {
    if (effectiveStop()) { drive = 0; steer = 0; return; }
    drive = last_control_.drive_cmd;
    steer = last_control_.steer_cmd;
}

bool RoverController::telemetryDue() {
    const uint32_t now = now_ms_();
    // Signed difference of unsigned timestamps: correct across wraparound,
    // and negative while the deadline is still in the future.
    const int32_t since_due = static_cast<int32_t>(now - next_telemetry_ms_);
    if (since_due < 0) return false;

    next_telemetry_ms_ += telemetry_period_ms_;
    // Never let a stalled loop accumulate a backlog it then bursts out.
    if (static_cast<int32_t>(now - next_telemetry_ms_)
            > static_cast<int32_t>(telemetry_period_ms_)) {
        next_telemetry_ms_ = now + telemetry_period_ms_;
    }
    return true;
}

TelemetryStatus RoverController::buildStatus(int16_t steer_fb, int16_t current_ca,
                                             uint8_t sensor_faults,
                                             uint8_t extra_faults) const {
    TelemetryStatus st;
    st.steer_fb     = steer_fb;
    st.current_ca   = current_ca;
    st.fault_status = static_cast<uint8_t>(sensor_faults | extra_faults);
    if (watchdogTripped()) st.fault_status |= FAULT_JETSON_HEARTBEAT_LOST;
    // Without this the rover would halt on a bad mode while telemetry read
    // "no faults, link healthy" -- a silent stop is its own hazard.
    if (protocol_error_)   st.fault_status |= FAULT_PROTOCOL_ERROR;
    if (last_control_.c2_lost != 0) st.fault_status |= FAULT_C2_LINK_LOST;
    st.cmd_age_ms   = cmdAgeMs();
    st.indicator_state = indicatorState();
    return st;
}

}  // namespace rover
