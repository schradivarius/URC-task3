// rover_firmware.ino -- Rover embedded controller firmware.
// Target: Teensy 4.1 (NXP i.MX RT1062, Cortex-M7 @ 600 MHz), Arduino/Teensyduino.
//
// BUILD
//   Arduino IDE + Teensyduino, board "Teensy 4.1". Requires two libraries:
//     FlexCAN_T4   https://github.com/tonton81/FlexCAN_T4
//     WDT_T4       https://github.com/tonton81/WDT_T4  (header: Watchdog_t4.h)
//   Sources under src/ are compiled automatically by the Arduino build.
//
// WIRING -- READ BEFORE POWERING ANYTHING
//   * Teensy 4.1 is 3.3 V and NOT 5 V tolerant. A 5 V encoder or sensor wired
//     direct to a pin destroys the board. Level-shift everything.
//   * CAN needs a TRANSCEIVER per bus; the MCU cannot drive a differential
//     bus itself. CAN3 is the FD-capable bus on Teensy 4.1.
//   * The bus needs 120 ohm termination at BOTH physical ends. Exactly one
//     resistor, or three, is the most common CAN bring-up failure.
//
// WHY THIS FILE IS SHORT
//   All the protocol and safety logic lives in src/rover_protocol.* and
//   src/rover_controller.*, which are pure C++ with no Arduino dependency and
//   are unit-tested natively (`make -C tests/cpp test`, 38 tests). This file
//   is only the hardware wiring around them. If you are reviewing safety
//   behaviour, read rover_controller.h, not this.
//
// SAFETY STRUCTURE
//   1. Hardware watchdog (WDT_T4). If this loop stops feeding it, the Teensy
//      resets, which drives every pin to a safe state and de-energizes the
//      drives. This is the backstop for a hang or a crash.
//   2. Boot-fault reporting. The reset itself is invisible to the Jetson
//      except as a gap in telemetry, so the NEXT boot inspects the reset
//      status register and raises FAULT_FIRMWARE_FAULT for the whole session.
//   3. Command watchdog, in RoverController: no valid CONTROL frame within
//      300 ms forces a stop and raises FAULT_JETSON_HEARTBEAT_LOST.
//   4. Commands are zeroed by commandedOutputs() before they ever reach the
//      motor layer, so a future edit to setMotorOutputs() cannot accidentally
//      act on a stale command.
//
//   Note one honest difference from the CircuitPython version: there is no
//   try/except guard here, because Arduino builds run without exceptions. The
//   hardware watchdog is the whole story for an unexpected fault, which is
//   why arming it is not optional.

#include <FlexCAN_T4.h>
#include <Watchdog_t4.h>

#include "src/rover_controller.h"
#include "src/rover_protocol.h"

using namespace rover;

// ---------------------------------------------------------------------------
// Configuration
// ---------------------------------------------------------------------------

static const uint32_t CAN_BITRATE_HZ      = 500000;   // 500 kbps
static const uint32_t HW_WATCHDOG_MS      = 1000;     // loop must feed within
static const int      LED_PIN             = 13;       // Teensy onboard LED
// LED_PIN is a telemetry heartbeat blink ONLY. It is not the status indicator.
//
// TODO (hardware): the URC status light has no pin assignment and no driver
// yet. RoverController::indicatorState() already decides what it should show
// and TELEM_STATE reports it, so the whole chain exists except the output --
// indicator_state is currently a value the Jetson can read and nobody can see.
//
// This costs POINTS, not just compliance: rule 1.e.xvii scores a target as
// reached only on "autonomously stopping within 1m of the target location and
// indicating its arrival", 25 points per target. So it is its own hardware
// item, not part of the setMotorOutputs() stub.
//
// Rule 1.e.ii also constrains the hardware, not only the colours: the
// indicator must be ON THE BACK of the rover and VISIBLE IN BRIGHT DAYLIGHT
// (it suggests an LED array or high-power LED). Every event runs in full
// daylight, so a bench-sized indicator will not pass.
//
// Mapping (PROTOCOL.md 3.5.1):
//   INDICATOR_OFF        off
//   INDICATOR_TELEOP     blue
//   INDICATOR_AUTONOMOUS red
//   INDICATOR_ARRIVED    flashing green   (this firmware owns the flash rate)
//   INDICATOR_FAULT      yellow, solid    (placeholder -- see PROTOCOL.md 9)
static const int16_t  OVER_CURRENT_CA     = 4000;     // 40.00 A, placeholder
static const int16_t  UNDERVOLTAGE_CV     = 2000;     // 20.00 V, placeholder

// Flip to false once real encoders / current sensing / steering feedback are
// wired in. See readSensors() for where the real code goes.
static const bool SIMULATE_SENSORS = true;

// CAN3 is the FD-capable bus on Teensy 4.1. Classic frames are used here so
// the protocol also runs on CAN1/CAN2 and alongside Classic-only motor
// controllers; switching this to CAN1 or CAN2 needs no other change.
FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> Can;
WDT_T4<WDT1> wdt;

static RoverController* ctl = nullptr;
static uint16_t g_boot_faults = 0;

// ---------------------------------------------------------------------------
// Sensor / motor I/O -- placeholder implementations.
//
// These are the ONLY functions that should need touching once real hardware
// is available. Everything above is protocol and safety and should not change.
//
// Teensy 4.1 gives you 55 digital I/O and 18 analog inputs, so unlike the
// previous board you can wire encoders and current sense directly. Putting
// the motor controllers on CAN instead is still usually the better call --
// they report their own encoder counts and current over the bus.
// ---------------------------------------------------------------------------

static int32_t sim_enc_left  = 0;
static int32_t sim_enc_right = 0;
static int16_t sim_steer_fb  = 0;

struct SensorReading {
    int32_t  enc_left;
    int32_t  enc_right;
    int16_t  steer_fb;
    int16_t  current_ca;   // SIGNED centiamps  (1 cA = 10 mA)
    int16_t  voltage_cv;   // SIGNED centivolts (1 cV = 10 mV)
    uint16_t fault_bits;
    uint8_t  controller_health;  // CTRL_HEALTH_*
};

static SensorReading readSensors(int16_t drive, int16_t steer, bool stopped) {
    SensorReading r;
    if (!SIMULATE_SENSORS) {
        // --- real sensor reads go here ---
        //   enc_*       : i.MX RT quadrature encoder peripherals, or CAN
        //   steer_fb    : analogRead() scaled into -1000..1000
        //   current_ca  : analogRead() scaled to SIGNED centiamps (1 cA=10 mA)
        //   voltage_cv  : analogRead() on the pack divider, SIGNED centivolts
        //   fault_bits  : FAULT_OVER_CURRENT / FAULT_ENCODER_FAULT / etc.
        //   controller_health : CTRL_HEALTH_* from motor-driver telemetry,
        //                 which on a CAN bus arrives as its own messages
        r.enc_left = 0; r.enc_right = 0; r.steer_fb = 0;
        r.current_ca = 0; r.voltage_cv = 0; r.fault_bits = 0;
        r.controller_health = CTRL_HEALTH_NOT_REPORTED;
        return r;
    }

    const int16_t d = stopped ? 0 : drive;
    const int16_t target = stopped ? 0 : steer;

    // wrapI32 takes int64 and wraps explicitly. Letting an int32 counter
    // overflow would be UNDEFINED BEHAVIOUR in C++ -- not a wrap, not an
    // exception, but a compiler free to do anything at all.
    sim_enc_left  = wrapI32(static_cast<int64_t>(sim_enc_left)  + d / 10);
    sim_enc_right = wrapI32(static_cast<int64_t>(sim_enc_right) + d / 10);

    if (sim_steer_fb < target)      sim_steer_fb = min(target, (int16_t)(sim_steer_fb + 25));
    else if (sim_steer_fb > target) sim_steer_fb = max(target, (int16_t)(sim_steer_fb - 25));

    r.enc_left   = sim_enc_left;
    r.enc_right  = sim_enc_right;
    r.steer_fb   = sim_steer_fb;
    r.current_ca = stopped ? 0 : static_cast<int16_t>(abs(d) * 3 / 2);
    // Fake 24 V pack sagging a little under load.
    r.voltage_cv = static_cast<int16_t>(2400 - abs(d) / 40);
    r.fault_bits = 0;
    if (r.current_ca > OVER_CURRENT_CA) r.fault_bits |= FAULT_OVER_CURRENT;
    if (r.voltage_cv < UNDERVOLTAGE_CV) r.fault_bits |= FAULT_UNDERVOLTAGE;
    r.controller_health = CTRL_HEALTH_NOT_REPORTED;
    return r;
}

static void setMotorOutputs(int16_t drive, int16_t steer, bool stopped) {
    (void)drive; (void)steer; (void)stopped;
    // TODO once hardware exists: drive real motor/servo outputs here.
    // When `stopped` is true this MUST de-energize the drives regardless of
    // `drive` -- that is the entire point of the watchdog and stop paths.
    // The caller already zeroes the commands, but honour the flag anyway.
}

// ---------------------------------------------------------------------------
// Boot fault detection
// ---------------------------------------------------------------------------

static uint16_t detectBootFault() {
    // SRC_SRSR is the i.MX RT reset status register; a watchdog reset means
    // the previous run of this firmware stopped feeding the watchdog, i.e. it
    // hung or crashed. The Jetson needs to know: telemetry resuming after an
    // unexplained gap otherwise looks like a flaky bus rather than a
    // controller that restarted mid-drive.
    //
    // VERIFY ON HARDWARE before trusting this bit mask. It is written to fail
    // SAFE -- an unrecognised reset cause reports nothing rather than raising
    // a false alarm -- but a missed watchdog reset is a silent gap, so confirm
    // it by deliberately hanging the loop once on the bench.
    const uint32_t srsr = SRC_SRSR;
    SRC_SRSR = srsr;                        // write-1-to-clear for next boot
    const uint32_t WDOG_BITS = (1u << 4) | (1u << 7);   // WDOG and WDOG3
    return (srsr & WDOG_BITS) ? FAULT_FIRMWARE_FAULT : 0;
}

// ---------------------------------------------------------------------------
// Telemetry
// ---------------------------------------------------------------------------

// Four frames per cycle, because each must fit Classic CAN's 8 bytes once the
// per-frame sequence number and CRC are accounted for. All four carry the SAME
// sequence number, which is how the Jetson tells a coherent snapshot from one
// torn across two cycles. See rover_protocol.h for why not one CAN FD frame.
static void sendTelemetry(const SensorReading& s) {
    const uint8_t seq = ctl->telemetrySeq();
    CAN_message_t frame;

    TelemetryDriveL dl = ctl->buildDriveL(s.enc_left, s.steer_fb);
    frame.id  = CAN_ID_TELEM_DRIVE_L;
    frame.len = encodeTelemetryDriveL(dl, seq, frame.buf);
    Can.write(frame);

    TelemetryDriveR dr = ctl->buildDriveR(s.enc_right);
    frame.id  = CAN_ID_TELEM_DRIVE_R;
    frame.len = encodeTelemetryDriveR(dr, seq, frame.buf);
    Can.write(frame);

    TelemetryPower pw =
        ctl->buildPower(s.current_ca, s.voltage_cv, s.fault_bits, g_boot_faults);
    frame.id  = CAN_ID_TELEM_POWER;
    frame.len = encodeTelemetryPower(pw, seq, frame.buf);
    Can.write(frame);

    // c2_link comes from the c2_lost bit the Jetson forwards, via
    // c2LinkState(). It is deliberately NOT derived from the Jetson link --
    // the two are separate links and must never be treated as equivalent --
    // and it reads LINK_NOT_REPORTED while that bit is absent or stale, which
    // is an honest "we do not know" rather than a misleading LINK_OK.
    TelemetryState st = ctl->buildState(s.controller_health);
    frame.id  = CAN_ID_TELEM_STATE;
    frame.len = encodeTelemetryState(st, seq, frame.buf);
    Can.write(frame);
}

// ---------------------------------------------------------------------------
// Arduino entry points
// ---------------------------------------------------------------------------

void setup() {
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);

    Serial.begin(115200);                   // USB console, for diagnostics only
    g_boot_faults = detectBootFault();

    Can.begin();
    Can.setBaudRate(CAN_BITRATE_HZ);
    // Accept only the Jetson's CONTROL id. Filtering in hardware keeps
    // motor-controller and payload traffic on a shared bus from ever reaching
    // the CPU, which matters far more at 1 Mbps than at 20 Hz.
    Can.setMBFilter(REJECT_ALL);
    Can.setMBFilter(MB0, CAN_ID_CONTROL);
    Can.enableMBInterrupts();

    static RoverController controller(millis, DEFAULT_WATCHDOG_TIMEOUT_MS,
                                      DEFAULT_TELEMETRY_PERIOD_MS);
    ctl = &controller;

    WDT_timings_t wdt_config;
    // Both fields are doubles in SECONDS. `trigger` defaults to 5 s, which
    // would sit ABOVE our 1 s timeout, so set it explicitly: it is when the
    // optional warning callback fires and must be below the reset timeout.
    wdt_config.trigger = (HW_WATCHDOG_MS / 1000.0) / 2.0;
    wdt_config.timeout = HW_WATCHDOG_MS / 1000.0;
    wdt.begin(wdt_config);

    Serial.printf("rover firmware up: CAN3 @ %lu bps, hw watchdog %lu ms%s\n",
                  (unsigned long)CAN_BITRATE_HZ, (unsigned long)HW_WATCHDOG_MS,
                  g_boot_faults ? ", RECOVERED FROM WATCHDOG RESET" : "");
}

void loop() {
    wdt.feed();

    // Drain everything waiting. read() returns false when the queue is empty.
    CAN_message_t rx;
    while (Can.read(rx)) {
        ctl->ingestFrame(rx.id, rx.buf, rx.len);
    }

    int16_t drive, steer;
    ctl->commandedOutputs(drive, steer);        // already zeroed under stop
    const bool stopped = ctl->effectiveStop();

    SensorReading s = readSensors(drive, steer, stopped);
    setMotorOutputs(drive, steer, stopped);

    if (ctl->telemetryDue()) {
        sendTelemetry(s);
        digitalWriteFast(LED_PIN, !digitalReadFast(LED_PIN));
    }
}
