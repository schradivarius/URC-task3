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
//
// Protocol and timing constants (CAN_BITRATE_HZ, HW_WATCHDOG_MS, timeouts,
// ids, OVER_CURRENT_CA) live in src/rover_config.h. Only board-specific
// wiring stays here.
// ---------------------------------------------------------------------------

static const int LED_PIN = 13;   // Teensy onboard LED

// Flip to false once real encoders / current sensing / steering feedback are
// wired in. See readSensors() for where the real code goes.
static const bool SIMULATE_SENSORS = true;

// CAN3 is the FD-capable bus on Teensy 4.1. Classic frames are used here so
// the protocol also runs on CAN1/CAN2 and alongside Classic-only motor
// controllers; switching this to CAN1 or CAN2 needs no other change.
FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> Can;
WDT_T4<WDT1> wdt;

static RoverController* ctl = nullptr;
static uint8_t g_boot_faults = 0;

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
    int32_t enc_left;
    int32_t enc_right;
    int16_t steer_fb;
    int16_t current_ca;
    uint8_t fault_bits;
};

static SensorReading readSensors(int16_t drive, int16_t steer, bool stopped) {
    SensorReading r;
    if (!SIMULATE_SENSORS) {
        // --- real sensor reads go here ---
        //   enc_*       : i.MX RT quadrature encoder peripherals, or CAN
        //   steer_fb    : analogRead() scaled into -1000..1000
        //   current_ca  : analogRead() scaled to SIGNED centiamps (1 cA=10 mA)
        //   fault_bits  : FAULT_OVER_CURRENT / FAULT_ENCODER_FAULT / etc.
        r.enc_left = 0; r.enc_right = 0; r.steer_fb = 0;
        r.current_ca = 0; r.fault_bits = 0;
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
    r.fault_bits = (r.current_ca > OVER_CURRENT_CA) ? FAULT_OVER_CURRENT : 0;
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

static uint8_t detectBootFault() {
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

static void sendTelemetry(const SensorReading& s) {
    CAN_message_t frame;

    TelemetryMotion motion = {s.enc_left, s.enc_right};
    frame.id  = CAN_ID_TELEM_MOTION;
    frame.len = encodeTelemetryMotion(motion, frame.buf);
    Can.write(frame);

    TelemetryStatus status =
        ctl->buildStatus(s.steer_fb, s.current_ca, s.fault_bits, g_boot_faults);
    frame.id  = CAN_ID_TELEM_STATUS;
    frame.len = encodeTelemetryStatus(status, frame.buf);
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
