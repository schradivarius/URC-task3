// test_controller.cpp -- safety state machine tests.
//
// Start a safety review here. Every test drives a FAKE CLOCK the test advances
// by hand, so watchdog timing is exact and deterministic rather than
// sleep-dependent -- a flaky safety test gets ignored, which is worse than no
// safety test.

#include "../../firmware/src/rover_controller.h"
#include "test_framework.h"

using namespace rover;

static uint32_t g_now = 0;
static uint32_t fakeMillis() { return g_now; }
static void advance(uint32_t ms) { g_now += ms; }   // uint32 wraps naturally

static RoverController freshController(uint32_t start_ms = 0) {
    g_now = start_ms;
    return RoverController(fakeMillis, 300, 50);
}

// Send a well-formed CONTROL frame. seq increments so the healthy path never
// trips the gap detector by accident.
static uint8_t g_seq = 0;
static bool sendControl(RoverController& ctl, int16_t drive, int16_t steer,
                        uint8_t mode, bool stop, bool abort = false,
                        bool ret = false, uint8_t ind = INDICATOR_OFF,
                        bool c2_lost = false) {
    ControlMsg m;
    m.drive_cmd = drive; m.steer_cmd = steer; m.mode = mode;
    m.stop = stop; m.autonomy_abort = abort; m.return_request = ret;
    m.c2_lost = c2_lost;
    m.indicator_request = ind; m.seq = ++g_seq;
    uint8_t buf[8] = {0};
    return ctl.ingestFrame(CAN_ID_CONTROL, buf, encodeControl(m, buf));
}

static bool sendWithSeq(RoverController& ctl, uint8_t seq, int16_t drive = 400) {
    ControlMsg m;
    m.drive_cmd = drive; m.steer_cmd = 0; m.mode = MODE_MANUAL;
    m.stop = false; m.autonomy_abort = false; m.return_request = false;
    m.c2_lost = false;
    m.indicator_request = INDICATOR_OFF; m.seq = seq;
    uint8_t buf[8] = {0};
    return ctl.ingestFrame(CAN_ID_CONTROL, buf, encodeControl(m, buf));
}

// --- boot and basic release -------------------------------------------------

static void boots_stopped_and_disabled() {
    RoverController ctl = freshController();
    CHECK(ctl.effectiveStop());
    CHECK(ctl.watchdogTripped());
    CHECK_EQ(ctl.cmdAgeMs(), CMD_AGE_UNKNOWN);
    CHECK_EQ(ctl.lastControl().mode, MODE_DISABLED);
    CHECK_EQ(ctl.jetsonLinkState(), LINK_LOST);
    int16_t d, s; ctl.commandedOutputs(d, s);
    CHECK_EQ(d, 0); CHECK_EQ(s, 0);
}

static void valid_control_releases_the_stop() {
    RoverController ctl = freshController();
    CHECK(sendControl(ctl, 500, 100, MODE_MANUAL, false));
    CHECK(!ctl.effectiveStop());
    CHECK_EQ(ctl.jetsonLinkState(), LINK_OK);
    int16_t d, s; ctl.commandedOutputs(d, s);
    CHECK_EQ(d, 500); CHECK_EQ(s, 100);
}

// --- watchdog ---------------------------------------------------------------

static void watchdog_trips_exactly_at_the_boundary() {
    RoverController ctl = freshController();
    sendControl(ctl, 500, 0, MODE_MANUAL, false);
    advance(299);
    CHECK(!ctl.watchdogTripped());
    advance(1);
    CHECK(ctl.watchdogTripped());
    CHECK(ctl.effectiveStop());
    CHECK_EQ(ctl.jetsonLinkState(), LINK_LOST);
}

static void watchdog_is_correct_across_millis_wraparound() {
    // Arduino millis() is uint32 and wraps every ~49.7 days. This passes
    // because elapsed time is an unsigned SUBTRACTION, correct modulo 2^32. It
    // FAILS against a timestamp comparison (now >= then + timeout) -- verified
    // by building one deliberately.
    RoverController ctl = freshController(0xFFFFFF00u);
    sendControl(ctl, 500, 0, MODE_MANUAL, false);

    // CRITICAL pre-wrap check. A naive deadline (last + 300) has itself
    // wrapped to a tiny number here, so a huge pre-wrap `now` compares greater
    // and reports a timeout that has not happened. Without this assertion the
    // whole test passes against the buggy implementation.
    advance(80);
    CHECK(g_now > 0xFFFFFF00u);
    CHECK(!ctl.watchdogTripped());
    CHECK_EQ(ctl.cmdAgeMs(), 80);

    advance(219);                    // 299 total; clock now past the wrap
    CHECK(g_now < 0xFFFFFF00u);
    CHECK(!ctl.watchdogTripped());
    advance(1);
    CHECK(ctl.watchdogTripped());
}

// --- the stop triggers ------------------------------------------------------

static void explicit_stop_flag_wins_in_any_mode() {
    const uint8_t modes[] = {MODE_MANUAL, MODE_AUTONOMOUS};
    for (uint8_t mode : modes) {
        RoverController ctl = freshController();
        sendControl(ctl, 1000, 500, mode, true);
        CHECK(ctl.effectiveStop());
        int16_t d, s; ctl.commandedOutputs(d, s);
        CHECK_EQ(d, 0); CHECK_EQ(s, 0);
    }
}

static void stop_is_releasable_without_a_mode_change() {
    RoverController ctl = freshController();
    sendControl(ctl, 500, 0, MODE_MANUAL, true);
    CHECK(ctl.effectiveStop());
    sendControl(ctl, 500, 0, MODE_MANUAL, false);
    CHECK(!ctl.effectiveStop());
}

static void disabled_mode_forces_stop_at_full_throttle() {
    RoverController ctl = freshController();
    sendControl(ctl, 1000, 1000, MODE_DISABLED, false);
    CHECK(ctl.effectiveStop());
}

static void undefined_mode_never_permits_motion() {
    for (int m = 3; m <= 255; m += 29) {
        RoverController ctl = freshController();
        ControlMsg msg;
        msg.drive_cmd = 1000; msg.steer_cmd = 500;
        msg.mode = static_cast<uint8_t>(m);
        msg.stop = false; msg.autonomy_abort = false; msg.return_request = false;
        msg.indicator_request = INDICATOR_OFF; msg.seq = 1;
        uint8_t buf[8] = {0};
        CHECK(!ctl.ingestFrame(CAN_ID_CONTROL, buf, encodeControl(msg, buf)));
        CHECK(ctl.effectiveStop());
        CHECK_EQ(ctl.lastDecodeResult(), DECODE_BAD_MODE);
    }
}

// --- autonomy abort and return request --------------------------------------

static void autonomy_abort_stops_an_autonomous_rover() {
    RoverController ctl = freshController();
    sendControl(ctl, 800, 0, MODE_AUTONOMOUS, false, /*abort=*/true);
    CHECK(ctl.autonomyAbort());
    CHECK(ctl.effectiveStop());
    int16_t d, s; ctl.commandedOutputs(d, s);
    CHECK_EQ(d, 0); CHECK_EQ(s, 0);
}

static void autonomy_abort_does_not_stop_a_teleoperated_rover() {
    // In MANUAL the operator is already driving; there is no autonomous task
    // to abort, and stopping would surprise them mid-manoeuvre.
    RoverController ctl = freshController();
    sendControl(ctl, 800, 0, MODE_MANUAL, false, /*abort=*/true);
    CHECK(ctl.autonomyAbort());
    CHECK(!ctl.effectiveStop());
    int16_t d, s; ctl.commandedOutputs(d, s);
    CHECK_EQ(d, 800);
}

static void return_request_is_carried_but_does_not_stop() {
    // A behaviour request for a higher layer, not a safety input.
    RoverController ctl = freshController();
    sendControl(ctl, 500, 0, MODE_AUTONOMOUS, false, false, /*ret=*/true);
    CHECK(ctl.returnRequest());
    CHECK(!ctl.effectiveStop());
}

// --- what must NOT refresh the watchdog -------------------------------------

static void wrong_dlc_does_not_refresh_the_watchdog() {
    RoverController ctl = freshController();
    sendControl(ctl, 500, 0, MODE_MANUAL, false);
    advance(290);
    uint8_t buf[8] = {0};
    CHECK(!ctl.ingestFrame(CAN_ID_CONTROL, buf, 4));
    CHECK_EQ(ctl.lastDecodeResult(), DECODE_BAD_DLC);
    advance(10);
    CHECK(ctl.watchdogTripped());
}

static void a_bad_crc_does_not_refresh_the_watchdog() {
    // The app-layer CRC's whole purpose: a frame corrupted in the software
    // path is rejected, and a rejected frame must not keep the rover alive.
    RoverController ctl = freshController();
    sendControl(ctl, 500, 0, MODE_MANUAL, false);
    advance(290);
    for (int i = 0; i < 5; ++i) {
        ControlMsg m;
        m.drive_cmd = 1000; m.steer_cmd = 0; m.mode = MODE_MANUAL;
        m.stop = false; m.autonomy_abort = false; m.return_request = false;
        m.indicator_request = INDICATOR_OFF; m.seq = static_cast<uint8_t>(i);
        uint8_t buf[8] = {0};
        encodeControl(m, buf);
        buf[0] ^= 0xFF;                       // corrupt AFTER sealing
        CHECK(!ctl.ingestFrame(CAN_ID_CONTROL, buf, FRAME_DLC));
        CHECK_EQ(ctl.lastDecodeResult(), DECODE_BAD_CRC);
    }
    advance(10);
    CHECK(ctl.watchdogTripped());
    CHECK(ctl.faultWord() & FAULT_CRC_ERROR);
    CHECK_EQ(ctl.crcErrorsSeen(), 5u);
}

static void foreign_can_id_does_not_refresh_the_watchdog() {
    RoverController ctl = freshController();
    sendControl(ctl, 500, 0, MODE_MANUAL, false);
    advance(290);
    uint8_t buf[8] = {0};
    ControlMsg m;
    m.drive_cmd = 999; m.steer_cmd = 0; m.mode = MODE_MANUAL;
    m.stop = false; m.autonomy_abort = false; m.return_request = false;
    m.indicator_request = INDICATOR_OFF; m.seq = 9;
    encodeControl(m, buf);
    CHECK(!ctl.ingestFrame(0x321, buf, FRAME_DLC));
    advance(10);
    CHECK(ctl.watchdogTripped());
    CHECK_EQ(ctl.lastControl().drive_cmd, 500);     // not overwritten
}

static void out_of_range_command_never_moves_the_rover() {
    RoverController ctl = freshController();
    CHECK(!sendControl(ctl, CMD_MAX + 1, 0, MODE_MANUAL, false));
    int16_t drive = -1, steer = -1;
    ctl.commandedOutputs(drive, steer);
    CHECK_EQ(drive, 0);
    CHECK_EQ(steer, 0);
}

static void out_of_range_command_does_not_refresh_the_watchdog() {
    // The whole point of rejecting rather than clamping: a sender that
    // disagrees with us about the scale must not be able to keep the rover
    // alive on commands we never understood.
    RoverController ctl = freshController();
    CHECK(sendControl(ctl, 500, 0, MODE_MANUAL, false));
    advance(290);
    CHECK(!ctl.watchdogTripped());
    for (int i = 0; i < 5; ++i) sendControl(ctl, 30000, 0, MODE_MANUAL, false);
    advance(20);
    CHECK(ctl.watchdogTripped());
    CHECK_EQ(ctl.lastControl().drive_cmd, 500);     // not overwritten
}

static void out_of_range_command_is_reported_as_a_protocol_error() {
    // A stop must be explainable. A rover that halts while telemetry reads
    // "no faults" is its own hazard.
    RoverController ctl = freshController();
    sendControl(ctl, 0, CMD_MIN - 1, MODE_MANUAL, false);
    CHECK(ctl.faultWord() & FAULT_PROTOCOL_ERROR);
    CHECK(!(ctl.faultWord() & FAULT_CRC_ERROR));    // not a corruption fault
    CHECK(sendControl(ctl, 0, CMD_MIN, MODE_MANUAL, false));   // full scale is fine
    CHECK(!(ctl.faultWord() & FAULT_PROTOCOL_ERROR));          // and self-clears
}

static void a_foreign_id_is_not_a_fault() {
    // Other traffic on a shared bus is normal. Flagging it would make the
    // protocol-error bit permanently on and therefore useless.
    RoverController ctl = freshController();
    sendControl(ctl, 400, 0, MODE_MANUAL, false);
    uint8_t buf[8] = {0};
    ctl.ingestFrame(0x321, buf, 8);
    CHECK(!(ctl.faultWord() & FAULT_PROTOCOL_ERROR));
    CHECK(!(ctl.faultWord() & FAULT_CRC_ERROR));
}

// --- sequence numbers -------------------------------------------------------

static void contiguous_sequence_numbers_are_healthy() {
    RoverController ctl = freshController();
    for (uint8_t seq = 1; seq <= 20; ++seq) CHECK(sendWithSeq(ctl, seq));
    CHECK(!(ctl.faultWord() & FAULT_SEQ_GAP));
    CHECK_EQ(ctl.seqGapsSeen(), 0u);
    CHECK_EQ(ctl.framesLostEstimate(), 0u);
    CHECK_EQ(ctl.jetsonLinkState(), LINK_OK);
}

static void a_sequence_gap_is_detected_and_counted() {
    RoverController ctl = freshController();
    sendWithSeq(ctl, 10);
    sendWithSeq(ctl, 14);                // 11, 12, 13 missing
    CHECK(ctl.faultWord() & FAULT_SEQ_GAP);
    CHECK_EQ(ctl.seqGapsSeen(), 1u);
    CHECK_EQ(ctl.framesLostEstimate(), 3u);
    CHECK_EQ(ctl.jetsonLinkState(), LINK_DEGRADED);
}

static void the_sequence_wrap_is_not_a_gap() {
    // 255 -> 0 is a delta of 1. Treating it as a 255-frame loss would raise a
    // false fault once every 256 frames, i.e. every 13 seconds at 20 Hz.
    RoverController ctl = freshController();
    sendWithSeq(ctl, 254);
    sendWithSeq(ctl, 255);
    sendWithSeq(ctl, 0);
    sendWithSeq(ctl, 1);
    CHECK(!(ctl.faultWord() & FAULT_SEQ_GAP));
    CHECK_EQ(ctl.seqGapsSeen(), 0u);
}

static void a_duplicate_sequence_number_is_flagged() {
    RoverController ctl = freshController();
    sendWithSeq(ctl, 5);
    sendWithSeq(ctl, 5);                 // delta 0
    CHECK(ctl.faultWord() & FAULT_SEQ_GAP);
    CHECK_EQ(ctl.framesLostEstimate(), 0u);   // nothing lost, just repeated
}

static void a_sequence_gap_clears_when_the_stream_recovers() {
    RoverController ctl = freshController();
    sendWithSeq(ctl, 1);
    sendWithSeq(ctl, 9);
    CHECK(ctl.faultWord() & FAULT_SEQ_GAP);
    sendWithSeq(ctl, 10);                // contiguous again
    CHECK(!(ctl.faultWord() & FAULT_SEQ_GAP));
}

static void the_first_frame_is_never_a_gap() {
    // There is no previous sequence number to compare against at boot.
    RoverController ctl = freshController();
    CHECK(sendWithSeq(ctl, 200));
    CHECK(!(ctl.faultWord() & FAULT_SEQ_GAP));
    CHECK_EQ(ctl.seqGapsSeen(), 0u);
}

static void the_link_returns_to_ok_after_the_degraded_hold_expires() {
    RoverController ctl = freshController();
    sendWithSeq(ctl, 1);
    sendWithSeq(ctl, 5);
    CHECK_EQ(ctl.jetsonLinkState(), LINK_DEGRADED);
    // Keep the command watchdog fed while the degraded hold runs out.
    for (uint32_t t = 0; t < LINK_DEGRADED_HOLD_MS + 100; t += 50) {
        advance(50);
        sendWithSeq(ctl, static_cast<uint8_t>(6 + t / 50));
    }
    CHECK_EQ(ctl.jetsonLinkState(), LINK_OK);
}

// --- link separation: the point of the whole exercise -----------------------

static void the_c2_link_is_never_derived_from_the_jetson_link() {
    // The 2027 autonomy course deliberately includes areas with no C2
    // line-of-sight while onboard autonomy keeps working. Collapsing the two
    // links would stop the rover exactly where it is supposed to keep going.
    RoverController ctl = freshController();

    // Before any frame: honestly "nobody told us", NOT a misleading OK, and
    // not derived from the boot default of the c2_lost field either.
    CHECK_EQ(ctl.c2LinkState(), LINK_NOT_REPORTED);

    sendControl(ctl, 500, 0, MODE_AUTONOMOUS, false, false, false,
                INDICATOR_AUTONOMOUS, /*c2_lost=*/false);
    TelemetryState st = ctl.buildState();
    CHECK_EQ(st.jetson_link, LINK_OK);
    CHECK_EQ(st.c2_link, LINK_OK);

    // A reported C2 loss must not change the Jetson link, and must not stop a
    // healthy autonomous rover.
    sendControl(ctl, 500, 0, MODE_AUTONOMOUS, false, false, false,
                INDICATOR_AUTONOMOUS, /*c2_lost=*/true);
    st = ctl.buildState();
    CHECK_EQ(st.c2_link, LINK_LOST);
    CHECK_EQ(st.jetson_link, LINK_OK);
    CHECK(!ctl.effectiveStop());
}

static void c2_loss_stops_a_teleoperated_rover_but_not_an_autonomous_one() {
    // The asymmetry is the whole point. In MANUAL the operator's commands can
    // no longer arrive, so nobody is driving. In AUTONOMOUS the Jetson IS
    // driving, and is provably alive or the watchdog would have stopped us.
    RoverController ctl = freshController();

    sendControl(ctl, 800, 0, MODE_AUTONOMOUS, false, false, false,
                INDICATOR_AUTONOMOUS, /*c2_lost=*/true);
    CHECK(!ctl.effectiveStop());
    int16_t drive = -1, steer = -1;
    ctl.commandedOutputs(drive, steer);
    CHECK_EQ(drive, 800);

    sendControl(ctl, 800, 0, MODE_MANUAL, false, false, false,
                INDICATOR_TELEOP, /*c2_lost=*/true);
    CHECK(ctl.effectiveStop());
    ctl.commandedOutputs(drive, steer);
    CHECK_EQ(drive, 0);

    // And it releases again when the Jetson says the link is back, without a
    // mode round trip.
    sendControl(ctl, 800, 0, MODE_MANUAL, false, false, false,
                INDICATOR_TELEOP, /*c2_lost=*/false);
    CHECK(!ctl.effectiveStop());
}

static void the_c2_fault_is_raised_in_every_mode() {
    // It only stops the rover in MANUAL, but during an autonomous run it is
    // the single sign that the rover is out of contact, so it must be
    // reported whether or not it changes the driving.
    const uint8_t modes[] = {MODE_DISABLED, MODE_MANUAL, MODE_AUTONOMOUS};
    for (uint8_t mode : modes) {
        RoverController ctl = freshController();
        sendControl(ctl, 0, 0, mode, false, false, false, INDICATOR_OFF,
                    /*c2_lost=*/true);
        CHECK((ctl.faultWord() & FAULT_C2_LINK_LOST) != 0);
        sendControl(ctl, 0, 0, mode, false, false, false, INDICATOR_OFF,
                    /*c2_lost=*/false);
        CHECK((ctl.faultWord() & FAULT_C2_LINK_LOST) == 0);
    }
}

static void a_stale_c2_bit_is_reported_as_unknown_not_as_ok_or_lost() {
    // The controller only knows c2_lost from the LAST CONTROL frame. Once the
    // Jetson goes silent that value is stale: a clear bit ten seconds ago does
    // not mean C2 is fine now, and a set bit does not mean it is still down.
    // Reporting either would be worse than admitting we do not know.
    for (int lost = 0; lost <= 1; ++lost) {
        RoverController ctl = freshController();
        sendControl(ctl, 0, 0, MODE_AUTONOMOUS, false, false, false,
                    INDICATOR_AUTONOMOUS, /*c2_lost=*/lost != 0);
        CHECK_EQ(ctl.c2LinkState(), lost ? LINK_LOST : LINK_OK);

        advance(400);                    // watchdog trips; the bit goes stale
        CHECK_EQ(ctl.c2LinkState(), LINK_NOT_REPORTED);
        CHECK_EQ(ctl.buildState().c2_link, LINK_NOT_REPORTED);
        // And the fault bit goes with it, rather than latching a stale claim.
        CHECK((ctl.faultWord() & FAULT_C2_LINK_LOST) == 0);
        CHECK((ctl.faultWord() & FAULT_COMM_TIMEOUT) != 0);
    }
}

// --- telemetry --------------------------------------------------------------

static void state_echoes_the_mode_the_controller_believes_in() {
    // Without the echo, a dropped or misread mode change is invisible from
    // outside the rover.
    RoverController ctl = freshController();
    sendControl(ctl, 500, 0, MODE_AUTONOMOUS, false);
    CHECK_EQ(ctl.buildState().mode, MODE_AUTONOMOUS);
    sendControl(ctl, 0, 0, MODE_DISABLED, false);
    CHECK_EQ(ctl.buildState().mode, MODE_DISABLED);
}

static void the_indicator_follows_the_mode_the_controller_is_in() {
    // URC scores this light on what the rover is ACTUALLY doing, so the
    // controller's own mode wins over the Jetson's request.
    RoverController ctl = freshController();

    sendControl(ctl, 500, 0, MODE_AUTONOMOUS, false, false, false,
                INDICATOR_AUTONOMOUS);
    CHECK_EQ(ctl.indicatorState(), INDICATOR_AUTONOMOUS);
    CHECK_EQ(ctl.buildState().indicator_state, INDICATOR_AUTONOMOUS);

    // Only ARRIVED is honoured in AUTONOMOUS; it is the one thing the
    // controller cannot know for itself.
    sendControl(ctl, 500, 0, MODE_AUTONOMOUS, false, false, false,
                INDICATOR_ARRIVED);
    CHECK_EQ(ctl.indicatorState(), INDICATOR_ARRIVED);

    // A human-driven rover never shows red or green, whatever is asked for.
    sendControl(ctl, 500, 0, MODE_MANUAL, false, false, false,
                INDICATOR_AUTONOMOUS);
    CHECK_EQ(ctl.indicatorState(), INDICATOR_TELEOP);
    sendControl(ctl, 500, 0, MODE_MANUAL, false, false, false,
                INDICATOR_ARRIVED);
    CHECK_EQ(ctl.indicatorState(), INDICATOR_TELEOP);

    // DISABLED is a legitimate idle state, not a fault.
    sendControl(ctl, 0, 0, MODE_DISABLED, false, false, false,
                INDICATOR_AUTONOMOUS);
    CHECK_EQ(ctl.indicatorState(), INDICATOR_OFF);
}

static void the_indicator_shows_fault_when_nothing_is_driving() {
    // A rover nobody can reach must not cheerfully display "autonomous" or
    // "arrived" -- and OFF would read as a legitimate idle state, so FAULT.
    RoverController ctl = freshController();
    sendControl(ctl, 500, 0, MODE_AUTONOMOUS, false, false, false,
                INDICATOR_AUTONOMOUS);
    advance(400);                        // watchdog trips -> COMM_TIMEOUT
    CHECK_EQ(ctl.indicatorState(), INDICATOR_FAULT);
    CHECK_EQ(ctl.buildState().indicator_state, INDICATOR_FAULT);

    // C2 lost in MANUAL is the other case where nobody is driving.
    RoverController ctl2 = freshController();
    sendControl(ctl2, 500, 0, MODE_MANUAL, false, false, false,
                INDICATOR_TELEOP, /*c2_lost=*/true);
    CHECK_EQ(ctl2.indicatorState(), INDICATOR_FAULT);
}

static void a_non_stopping_fault_does_not_blank_the_indicator() {
    // URC requires red while autonomous. An undervoltage warning or a single
    // lost frame must not take the light away from what the rover is doing;
    // those belong in fault_status, which is where an operator reads them.
    RoverController ctl = freshController();
    sendControl(ctl, 500, 0, MODE_AUTONOMOUS, false, false, false,
                INDICATOR_AUTONOMOUS);
    TelemetryPower pw = ctl.buildPower(0, 2400, FAULT_UNDERVOLTAGE);
    CHECK((pw.fault_status & FAULT_UNDERVOLTAGE) != 0);
    CHECK_EQ(ctl.indicatorState(), INDICATOR_AUTONOMOUS);
}

static void power_reports_the_same_fault_word_the_controller_acts_on() {
    RoverController ctl = freshController();
    TelemetryPower pw = ctl.buildPower(150, 2400, FAULT_OVER_CURRENT,
                                       FAULT_FIRMWARE_FAULT);
    CHECK(pw.fault_status & FAULT_OVER_CURRENT);
    CHECK(pw.fault_status & FAULT_FIRMWARE_FAULT);
    CHECK(pw.fault_status & FAULT_COMM_TIMEOUT);    // no command yet
    CHECK_EQ(pw.fault_status, ctl.faultWord(FAULT_OVER_CURRENT, FAULT_FIRMWARE_FAULT));
    CHECK_EQ(pw.current_ca, 150);
    CHECK_EQ(pw.voltage_cv, 2400);
}

static void drive_r_carries_the_command_age() {
    RoverController ctl = freshController();
    sendControl(ctl, 500, 0, MODE_MANUAL, false);
    advance(120);
    CHECK_EQ(ctl.buildDriveR(4242).cmd_age_ms, 120);
    CHECK_EQ(ctl.buildDriveR(4242).enc_right, 4242);
}

static void age_is_unknown_before_the_first_command() {
    RoverController ctl = freshController();
    CHECK_EQ(ctl.buildDriveR(0).cmd_age_ms, CMD_AGE_UNKNOWN);
    CHECK(ctl.buildPower(0, 0).fault_status & FAULT_COMM_TIMEOUT);
}

// --- telemetry pacing and the shared sequence number ------------------------

static void telemetry_paces_at_the_configured_period() {
    RoverController ctl = freshController();
    CHECK(ctl.telemetryDue());
    CHECK(!ctl.telemetryDue());
    advance(50);
    CHECK(ctl.telemetryDue());
}

static void the_telemetry_sequence_advances_once_per_cycle() {
    // All four frames of a cycle must share one sequence number -- that is how
    // the Jetson tells a coherent snapshot from one torn across two cycles.
    RoverController ctl = freshController();
    CHECK(ctl.telemetryDue());
    const uint8_t first = ctl.telemetrySeq();
    CHECK_EQ(ctl.telemetrySeq(), first);      // reading it does not advance it
    CHECK_EQ(ctl.telemetrySeq(), first);
    advance(50);
    CHECK(ctl.telemetryDue());
    CHECK_EQ(seqDelta(first, ctl.telemetrySeq()), 1);
}

static void telemetry_does_not_burst_after_a_stall() {
    RoverController ctl = freshController();
    ctl.telemetryDue();
    advance(5000);                       // a 5 second stall
    CHECK(ctl.telemetryDue());
    CHECK(!ctl.telemetryDue());          // exactly one, not a hundred
}

static void telemetry_pacing_is_correct_across_wraparound() {
    RoverController ctl = freshController(0xFFFFFFF0u);
    CHECK(ctl.telemetryDue());
    CHECK(!ctl.telemetryDue());
    advance(50);
    CHECK(g_now < 0xFFFFFFF0u);
    CHECK(ctl.telemetryDue());
}

int main() {
    std::printf("test_controller\n");
    RUN_TEST(boots_stopped_and_disabled);
    RUN_TEST(valid_control_releases_the_stop);
    RUN_TEST(watchdog_trips_exactly_at_the_boundary);
    RUN_TEST(watchdog_is_correct_across_millis_wraparound);
    RUN_TEST(explicit_stop_flag_wins_in_any_mode);
    RUN_TEST(stop_is_releasable_without_a_mode_change);
    RUN_TEST(disabled_mode_forces_stop_at_full_throttle);
    RUN_TEST(undefined_mode_never_permits_motion);
    RUN_TEST(autonomy_abort_stops_an_autonomous_rover);
    RUN_TEST(autonomy_abort_does_not_stop_a_teleoperated_rover);
    RUN_TEST(return_request_is_carried_but_does_not_stop);
    RUN_TEST(wrong_dlc_does_not_refresh_the_watchdog);
    RUN_TEST(a_bad_crc_does_not_refresh_the_watchdog);
    RUN_TEST(foreign_can_id_does_not_refresh_the_watchdog);
    RUN_TEST(a_foreign_id_is_not_a_fault);
    RUN_TEST(out_of_range_command_never_moves_the_rover);
    RUN_TEST(out_of_range_command_does_not_refresh_the_watchdog);
    RUN_TEST(out_of_range_command_is_reported_as_a_protocol_error);
    RUN_TEST(contiguous_sequence_numbers_are_healthy);
    RUN_TEST(a_sequence_gap_is_detected_and_counted);
    RUN_TEST(the_sequence_wrap_is_not_a_gap);
    RUN_TEST(a_duplicate_sequence_number_is_flagged);
    RUN_TEST(a_sequence_gap_clears_when_the_stream_recovers);
    RUN_TEST(the_first_frame_is_never_a_gap);
    RUN_TEST(the_link_returns_to_ok_after_the_degraded_hold_expires);
    RUN_TEST(the_c2_link_is_never_derived_from_the_jetson_link);
    RUN_TEST(c2_loss_stops_a_teleoperated_rover_but_not_an_autonomous_one);
    RUN_TEST(the_c2_fault_is_raised_in_every_mode);
    RUN_TEST(a_stale_c2_bit_is_reported_as_unknown_not_as_ok_or_lost);
    RUN_TEST(state_echoes_the_mode_the_controller_believes_in);
    RUN_TEST(the_indicator_follows_the_mode_the_controller_is_in);
    RUN_TEST(the_indicator_shows_fault_when_nothing_is_driving);
    RUN_TEST(a_non_stopping_fault_does_not_blank_the_indicator);
    RUN_TEST(power_reports_the_same_fault_word_the_controller_acts_on);
    RUN_TEST(drive_r_carries_the_command_age);
    RUN_TEST(age_is_unknown_before_the_first_command);
    RUN_TEST(telemetry_paces_at_the_configured_period);
    RUN_TEST(the_telemetry_sequence_advances_once_per_cycle);
    RUN_TEST(telemetry_does_not_burst_after_a_stall);
    RUN_TEST(telemetry_pacing_is_correct_across_wraparound);
    return testing::summary("test_controller");
}
