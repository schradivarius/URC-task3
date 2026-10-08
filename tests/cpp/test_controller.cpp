// test_controller.cpp -- safety state machine tests.
//
// Start a safety review here. Every test drives a FAKE CLOCK the test advances
// by hand, so watchdog timing is exact and deterministic rather than
// sleep-dependent -- a flaky safety test gets ignored, which is worse than no
// safety test.

#include "../../firmware/src/rover_controller.h"
#include "test_framework.h"

using namespace rover;

// --- fake clock -------------------------------------------------------------

static uint32_t g_now = 0;
static uint32_t fakeMillis() { return g_now; }
static void advance(uint32_t ms) { g_now += ms; }   // uint32 wraps naturally

static RoverController freshController(uint32_t start_ms = 0) {
    g_now = start_ms;
    return RoverController(fakeMillis, 300, 50);
}

static bool sendControl(RoverController& ctl, int16_t drive, int16_t steer,
                        uint8_t mode, uint8_t stop, uint8_t c2_lost,
                        uint8_t indicator_request = INDICATOR_OFF) {
    ControlMsg m = {drive, steer, mode, stop, indicator_request, c2_lost};
    uint8_t buf[8] = {0};
    uint8_t dlc = encodeControl(m, buf);
    return ctl.ingestFrame(CAN_ID_CONTROL, buf, dlc);
}

// --- boot and basic release -------------------------------------------------

static void boots_stopped_and_safe() {
    // Before any command arrives the rover must not be able to move.
    RoverController ctl = freshController();
    CHECK(ctl.effectiveStop());
    CHECK(ctl.watchdogTripped());
    CHECK_EQ(ctl.cmdAgeMs(), CMD_AGE_UNKNOWN);
    CHECK_EQ(ctl.lastControl().mode, MODE_SAFE);
    int16_t d, s; ctl.commandedOutputs(d, s);
    CHECK_EQ(d, 0); CHECK_EQ(s, 0);
}

static void valid_control_releases_the_stop() {
    RoverController ctl = freshController();
    CHECK(sendControl(ctl, 500, 100, MODE_TELEOP, 0, 0));
    CHECK(!ctl.effectiveStop());
    int16_t d, s; ctl.commandedOutputs(d, s);
    CHECK_EQ(d, 500); CHECK_EQ(s, 100);
}

// --- watchdog ---------------------------------------------------------------

static void watchdog_trips_exactly_at_the_boundary() {
    RoverController ctl = freshController();
    sendControl(ctl, 500, 0, MODE_TELEOP, 0, 0);
    advance(299);
    CHECK(!ctl.watchdogTripped());          // one millisecond early
    advance(1);
    CHECK(ctl.watchdogTripped());           // exactly on time
    CHECK(ctl.effectiveStop());
    int16_t d, s; ctl.commandedOutputs(d, s);
    CHECK_EQ(d, 0); CHECK_EQ(s, 0);
}

static void watchdog_clears_when_commands_resume() {
    RoverController ctl = freshController();
    sendControl(ctl, 500, 0, MODE_TELEOP, 0, 0);
    advance(500);
    CHECK(ctl.effectiveStop());
    sendControl(ctl, 500, 0, MODE_TELEOP, 0, 0);
    CHECK(!ctl.effectiveStop());            // recovers on its own
}

// --- THE NEW HAZARD: millis() wraps every ~49.7 days ------------------------

static void watchdog_is_correct_across_millis_wraparound() {
    // CircuitPython's monotonic_ns() never wrapped, so the Python version was
    // safe by construction. Arduino millis() is uint32_t and wraps. This test
    // starts 256 ms before the wrap and crosses it.
    //
    // It passes because elapsed time is computed as an unsigned SUBTRACTION
    // (now - then), which is correct modulo 2^32. It would FAIL if anyone
    // rewrote the check as a timestamp comparison (now >= then + timeout),
    // which overflows and reports "not yet" forever.
    RoverController ctl = freshController(0xFFFFFF00u);
    sendControl(ctl, 500, 0, MODE_TELEOP, 0, 0);

    // CRITICAL intermediate check, BEFORE the clock wraps. This is the window
    // where the naive version breaks and the correct one does not: the naive
    // deadline (last + 300) has itself wrapped to a tiny number, so the huge
    // pre-wrap `now` compares greater than it and reports a timeout that has
    // not happened. Without this assertion the whole test passes against the
    // buggy implementation -- verified by building one and running it.
    advance(80);
    CHECK(g_now > 0xFFFFFF00u);              // still pre-wrap
    CHECK(!ctl.watchdogTripped());           // 80ms elapsed, nowhere near 300
    CHECK_EQ(ctl.cmdAgeMs(), 80);

    advance(219);                            // 299 total; clock now past the wrap
    CHECK(g_now < 0xFFFFFF00u);              // confirm it really did wrap
    CHECK(!ctl.watchdogTripped());
    CHECK_EQ(ctl.cmdAgeMs(), 299);

    advance(1);
    CHECK(ctl.watchdogTripped());
    CHECK_EQ(ctl.cmdAgeMs(), 300);
}

static void telemetry_pacing_is_correct_across_wraparound() {
    RoverController ctl = freshController(0xFFFFFFF0u);
    CHECK(ctl.telemetryDue());               // first call is due immediately
    CHECK(!ctl.telemetryDue());
    advance(50);                             // crosses the wrap
    CHECK(g_now < 0xFFFFFFF0u);
    CHECK(ctl.telemetryDue());
}

// --- the other two stop triggers --------------------------------------------

static void explicit_stop_flag_wins_in_any_mode() {
    const uint8_t modes[] = {MODE_TELEOP, MODE_AUTONOMOUS};
    for (uint8_t mode : modes) {
        RoverController ctl = freshController();
        sendControl(ctl, 1000, 500, mode, 1, 0);
        CHECK(ctl.effectiveStop());
        int16_t d, s; ctl.commandedOutputs(d, s);
        CHECK_EQ(d, 0); CHECK_EQ(s, 0);
    }
}

static void stop_is_releasable_without_a_mode_change() {
    // stop is a separate field rather than a third mode value precisely so an
    // e-stop can be asserted AND released without a mode round trip.
    RoverController ctl = freshController();
    sendControl(ctl, 500, 0, MODE_TELEOP, 1, 0);
    CHECK(ctl.effectiveStop());
    sendControl(ctl, 500, 0, MODE_TELEOP, 0, 0);
    CHECK(!ctl.effectiveStop());
}

static void safe_mode_forces_stop_at_full_throttle() {
    RoverController ctl = freshController();
    sendControl(ctl, 1000, 1000, MODE_SAFE, 0, 0);
    CHECK(ctl.effectiveStop());
    int16_t d, s; ctl.commandedOutputs(d, s);
    CHECK_EQ(d, 0); CHECK_EQ(s, 0);
}

// --- what must NOT refresh the watchdog -------------------------------------

static void wrong_dlc_does_not_refresh_the_watchdog() {
    // A peer on a mismatched protocol version must not keep the rover alive
    // while sending commands it never actually understood.
    RoverController ctl = freshController();
    sendControl(ctl, 500, 0, MODE_TELEOP, 0, 0);
    advance(290);
    uint8_t buf[8] = {0};
    CHECK(!ctl.ingestFrame(CAN_ID_CONTROL, buf, 4));   // wrong DLC
    advance(10);
    CHECK(ctl.watchdogTripped());
    CHECK_EQ(ctl.framesIgnored(), 1u);
}

static void foreign_can_id_does_not_refresh_the_watchdog() {
    // A shared bus carries motor-controller and payload traffic too. None of
    // it may count as a command from the Jetson.
    RoverController ctl = freshController();
    sendControl(ctl, 500, 0, MODE_TELEOP, 0, 0);
    advance(290);
    uint8_t buf[8] = {0};
    encodeControl({999, 0, MODE_TELEOP, 0, INDICATOR_OFF, 0}, buf);
    CHECK(!ctl.ingestFrame(0x321, buf, CONTROL_DLC));  // someone else's frame
    advance(10);
    CHECK(ctl.watchdogTripped());
    CHECK_EQ(ctl.lastControl().drive_cmd, 500);        // not overwritten
}

// --- undefined mode (issue #4) ----------------------------------------------

static void undefined_mode_does_not_permit_motion() {
    // REGRESSION, issue #4. Before the fix, mode=255 with stop=0 and a fresh
    // command permitted drive=1000, steer=500. The rover moved in a mode no
    // firmware defines.
    for (int m = 3; m <= 255; m += 29) {       // sample the space; the
        RoverController ctl = freshController();   // exhaustive check is in
        ControlMsg msg = {1000, 500, (uint8_t)m, 0, INDICATOR_OFF, 0};   // test_protocol.cpp
        uint8_t buf[8] = {0};
        uint8_t dlc = encodeControl(msg, buf);
        CHECK(!ctl.ingestFrame(CAN_ID_CONTROL, buf, dlc));   // rejected
        CHECK(ctl.effectiveStop());
        int16_t d, s; ctl.commandedOutputs(d, s);
        CHECK_EQ(d, 0); CHECK_EQ(s, 0);
    }
}

static void undefined_mode_does_not_refresh_the_watchdog() {
    // A peer spamming an undefined mode must not keep the rover alive.
    RoverController ctl = freshController();
    sendControl(ctl, 500, 0, MODE_TELEOP, 0, 0);
    advance(290);
    for (int i = 0; i < 5; ++i) {
        ControlMsg msg = {1000, 0, 7, 0, INDICATOR_OFF, 0};
        uint8_t buf[8] = {0};
        ctl.ingestFrame(CAN_ID_CONTROL, buf, encodeControl(msg, buf));
    }
    advance(10);
    CHECK(ctl.watchdogTripped());
    CHECK(ctl.effectiveStop());
}

static void undefined_mode_is_reported_not_silent() {
    RoverController ctl = freshController();
    ControlMsg msg = {1000, 0, 42, 0, INDICATOR_OFF, 0};
    uint8_t buf[8] = {0};
    ctl.ingestFrame(CAN_ID_CONTROL, buf, encodeControl(msg, buf));
    CHECK(ctl.protocolError());
    TelemetryStatus st = ctl.buildStatus(0, 0);
    CHECK(st.fault_status & FAULT_PROTOCOL_ERROR);
}

static void a_good_frame_clears_the_protocol_error() {
    // Self-healing: fix the sender and the fault goes away, rather than
    // latching and misleading the operator for the rest of the session.
    RoverController ctl = freshController();
    ControlMsg bad = {1000, 0, 42, 0, INDICATOR_OFF, 0};
    uint8_t buf[8] = {0};
    ctl.ingestFrame(CAN_ID_CONTROL, buf, encodeControl(bad, buf));
    CHECK(ctl.protocolError());
    CHECK(sendControl(ctl, 500, 0, MODE_TELEOP, 0, 0));
    CHECK(!ctl.protocolError());
    CHECK(!(ctl.buildStatus(0, 0).fault_status & FAULT_PROTOCOL_ERROR));
    CHECK(!ctl.effectiveStop());
}

static void wrong_dlc_also_reports_a_protocol_error() {
    RoverController ctl = freshController();
    uint8_t buf[8] = {0};
    CHECK(!ctl.ingestFrame(CAN_ID_CONTROL, buf, 4));
    CHECK(ctl.protocolError());
}

static void a_foreign_id_is_not_a_protocol_error() {
    // Other traffic on a shared bus is normal, not a fault. Flagging it would
    // make PROTOCOL_ERROR permanently on and therefore useless.
    RoverController ctl = freshController();
    uint8_t buf[8] = {0};
    ctl.ingestFrame(0x321, buf, 8);
    CHECK(!ctl.protocolError());
}

// --- telemetry --------------------------------------------------------------

static void telemetry_reports_jetson_heartbeat_lost_and_age() {
    RoverController ctl = freshController();
    sendControl(ctl, 500, 0, MODE_TELEOP, 0, 0);
    advance(400);
    TelemetryStatus st = ctl.buildStatus(3, 0);
    CHECK(st.fault_status & FAULT_JETSON_HEARTBEAT_LOST);
    CHECK_EQ(st.cmd_age_ms, 400);
}

static void age_is_unknown_before_the_first_command() {
    RoverController ctl = freshController();
    TelemetryStatus st = ctl.buildStatus(0, 0);
    CHECK_EQ(st.cmd_age_ms, CMD_AGE_UNKNOWN);
    CHECK(st.fault_status & FAULT_JETSON_HEARTBEAT_LOST);
}

static void sensor_faults_survive_alongside_jetson_heartbeat_lost() {
    // A comm fault must not mask a genuine over-current fault.
    RoverController ctl = freshController();
    TelemetryStatus st = ctl.buildStatus(0, 0, FAULT_OVER_CURRENT);
    CHECK(st.fault_status & FAULT_OVER_CURRENT);
    CHECK(st.fault_status & FAULT_JETSON_HEARTBEAT_LOST);
}

static void firmware_fault_bit_is_carried_through() {
    RoverController ctl = freshController();
    sendControl(ctl, 100, 0, MODE_TELEOP, 0, 0);
    TelemetryStatus st = ctl.buildStatus(0, 0, 0, FAULT_FIRMWARE_FAULT);
    CHECK(st.fault_status & FAULT_FIRMWARE_FAULT);
    CHECK(!(st.fault_status & FAULT_JETSON_HEARTBEAT_LOST));   // link is healthy
}

static void telemetry_paces_at_the_configured_period() {
    RoverController ctl = freshController();
    CHECK(ctl.telemetryDue());
    CHECK(!ctl.telemetryDue());
    advance(50);
    CHECK(ctl.telemetryDue());
}

static void telemetry_does_not_burst_after_a_stall() {
    // After a long stall the scheduler must resync to now, not fire once per
    // missed period in a burst that floods the bus.
    RoverController ctl = freshController();
    ctl.telemetryDue();
    advance(5000);                     // a 5 second stall
    CHECK(ctl.telemetryDue());
    CHECK(!ctl.telemetryDue());        // exactly one, not a hundred
}

// --- active mode: what the rover is actually executing ----------------------

static void boot_reports_fault_until_the_jetson_speaks() {
    RoverController ctl = freshController();
    CHECK_EQ(ctl.activeMode(), MODE_FAULT);
}

static void active_mode_follows_the_commanded_mode() {
    const uint8_t modes[] = {MODE_SAFE, MODE_TELEOP, MODE_AUTONOMOUS};
    for (uint8_t mode : modes) {
        RoverController ctl = freshController();
        sendControl(ctl, 500, 0, mode, 0, 0);
        CHECK_EQ(ctl.activeMode(), mode);
    }
}

static void heartbeat_loss_is_a_fault_in_every_mode() {
    const uint8_t modes[] = {MODE_SAFE, MODE_TELEOP, MODE_AUTONOMOUS};
    for (uint8_t mode : modes) {
        RoverController ctl = freshController();
        sendControl(ctl, 500, 0, mode, 0, 0);
        advance(299);
        CHECK_EQ(ctl.activeMode(), mode);          // one ms early: still fine
        advance(1);
        CHECK_EQ(ctl.activeMode(), MODE_FAULT);
    }
}

static void fault_clears_when_commands_resume() {
    RoverController ctl = freshController();
    sendControl(ctl, 500, 0, MODE_TELEOP, 0, 0);
    advance(300);
    CHECK_EQ(ctl.activeMode(), MODE_FAULT);
    sendControl(ctl, 500, 0, MODE_TELEOP, 0, 0);
    CHECK_EQ(ctl.activeMode(), MODE_TELEOP);
}

static void c2_loss_faults_teleop_but_not_autonomy() {
    RoverController ctl = freshController();
    sendControl(ctl, 500, 0, MODE_TELEOP, 0, 1);
    CHECK_EQ(ctl.activeMode(), MODE_FAULT);
    CHECK(ctl.effectiveStop());

    sendControl(ctl, 500, 0, MODE_AUTONOMOUS, 0, 1);
    CHECK_EQ(ctl.activeMode(), MODE_AUTONOMOUS);
    CHECK(!ctl.effectiveStop());
}

static void a_pause_is_not_a_fault() {
    RoverController ctl = freshController();
    sendControl(ctl, 500, 0, MODE_AUTONOMOUS, 1, 0);
    CHECK(ctl.effectiveStop());
    CHECK_EQ(ctl.activeMode(), MODE_AUTONOMOUS);
}

static void fault_cannot_be_commanded() {
    RoverController ctl = freshController();
    sendControl(ctl, 500, 0, MODE_TELEOP, 0, 0);
    CHECK(!sendControl(ctl, 500, 0, MODE_FAULT, 0, 0));
    CHECK(ctl.protocolError());
    CHECK_EQ(ctl.activeMode(), MODE_TELEOP);       // last valid command, until the watchdog
}

// --- status indicator -------------------------------------------------------

static void autonomous_mode_shows_red() {
    RoverController ctl = freshController();
    sendControl(ctl, 500, 0, MODE_AUTONOMOUS, 0, 0, INDICATOR_RED);
    CHECK_EQ(ctl.indicatorState(), INDICATOR_RED);
}

static void autonomous_mode_shows_green_flash() {
    RoverController ctl = freshController();
    sendControl(ctl, 500, 0, MODE_AUTONOMOUS, 0, 0, INDICATOR_GREEN_FLASH);
    CHECK_EQ(ctl.indicatorState(), INDICATOR_GREEN_FLASH);
}

static void watchdog_trip_turns_indicator_off() {
    RoverController ctl = freshController();
    sendControl(ctl, 500, 0, MODE_AUTONOMOUS, 0, 0, INDICATOR_RED);
    CHECK_EQ(ctl.indicatorState(), INDICATOR_RED);
    advance(300);
    CHECK_EQ(ctl.indicatorState(), INDICATOR_OFF);
}

static void teleop_mode_always_shows_blue() {
    const uint8_t requests[] = {INDICATOR_BLUE, INDICATOR_RED, INDICATOR_GREEN_FLASH};
    for (uint8_t request : requests) {
        RoverController ctl = freshController();
        sendControl(ctl, 500, 0, MODE_TELEOP, 0, 0, request);
        CHECK_EQ(ctl.indicatorState(), INDICATOR_BLUE);
    }
}

static void safe_mode_shows_off() {
    RoverController ctl = freshController();
    sendControl(ctl, 500, 0, MODE_SAFE, 0, 0, INDICATOR_BLUE);
    CHECK_EQ(ctl.indicatorState(), INDICATOR_OFF);
}

static void paused_autonomy_stays_red() {
    RoverController ctl = freshController();
    sendControl(ctl, 500, 0, MODE_AUTONOMOUS, 1, 0, INDICATOR_RED);
    CHECK_EQ(ctl.indicatorState(), INDICATOR_RED);
}

static void teleop_without_c2_shows_off() {
    // The operator link is gone, so nobody is teleoperating: not blue.
    RoverController ctl = freshController();
    sendControl(ctl, 500, 0, MODE_TELEOP, 0, 1);
    CHECK_EQ(ctl.indicatorState(), INDICATOR_OFF);
}

static void telemetry_reports_indicator_state(){
    RoverController ctl = freshController();
    sendControl(ctl, 500, 0, MODE_AUTONOMOUS, 0, 0, INDICATOR_GREEN_FLASH);
    TelemetryStatus st = ctl.buildStatus(0,0);
    CHECK_EQ(st.indicator_state, INDICATOR_GREEN_FLASH);
}

int main() {
    std::printf("test_controller\n");
    RUN_TEST(boots_stopped_and_safe);
    RUN_TEST(valid_control_releases_the_stop);
    RUN_TEST(watchdog_trips_exactly_at_the_boundary);
    RUN_TEST(watchdog_clears_when_commands_resume);
    RUN_TEST(watchdog_is_correct_across_millis_wraparound);
    RUN_TEST(telemetry_pacing_is_correct_across_wraparound);
    RUN_TEST(explicit_stop_flag_wins_in_any_mode);
    RUN_TEST(stop_is_releasable_without_a_mode_change);
    RUN_TEST(safe_mode_forces_stop_at_full_throttle);
    RUN_TEST(wrong_dlc_does_not_refresh_the_watchdog);
    RUN_TEST(foreign_can_id_does_not_refresh_the_watchdog);
    RUN_TEST(undefined_mode_does_not_permit_motion);
    RUN_TEST(undefined_mode_does_not_refresh_the_watchdog);
    RUN_TEST(undefined_mode_is_reported_not_silent);
    RUN_TEST(a_good_frame_clears_the_protocol_error);
    RUN_TEST(wrong_dlc_also_reports_a_protocol_error);
    RUN_TEST(a_foreign_id_is_not_a_protocol_error);
    RUN_TEST(telemetry_reports_jetson_heartbeat_lost_and_age);
    RUN_TEST(boot_reports_fault_until_the_jetson_speaks);
    RUN_TEST(active_mode_follows_the_commanded_mode);
    RUN_TEST(heartbeat_loss_is_a_fault_in_every_mode);
    RUN_TEST(fault_clears_when_commands_resume);
    RUN_TEST(c2_loss_faults_teleop_but_not_autonomy);
    RUN_TEST(a_pause_is_not_a_fault);
    RUN_TEST(fault_cannot_be_commanded);
    RUN_TEST(autonomous_mode_shows_red);
    RUN_TEST(autonomous_mode_shows_green_flash);
    RUN_TEST(watchdog_trip_turns_indicator_off);
    RUN_TEST(teleop_mode_always_shows_blue);
    RUN_TEST(safe_mode_shows_off);
    RUN_TEST(paused_autonomy_stays_red);
    RUN_TEST(teleop_without_c2_shows_off);
    RUN_TEST(telemetry_reports_indicator_state);
    RUN_TEST(age_is_unknown_before_the_first_command);
    RUN_TEST(sensor_faults_survive_alongside_jetson_heartbeat_lost);
    RUN_TEST(firmware_fault_bit_is_carried_through);
    RUN_TEST(telemetry_paces_at_the_configured_period);
    RUN_TEST(telemetry_does_not_burst_after_a_stall);
    return testing::summary("test_controller");
}
