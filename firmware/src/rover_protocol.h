// rover_protocol.h -- Rover Jetson <-> Teensy CAN message definitions.
//
// PURE C++. No Arduino.h, no FlexCAN, no hardware. That is deliberate: this
// file and rover_controller.* compile unchanged for the Teensy AND natively
// on a laptop, so the safety-critical logic is unit-tested by `make test`
// with no board attached. Only rover_firmware.ino touches hardware.
//
// WHAT CHANGED FROM THE UART VERSION, AND WHY IT SHRANK
//   The old framing.py carried a start byte, a length byte, a CRC-16 and a
//   108-line resynchronizing stream parser. All of it existed because a UART
//   is a featureless byte stream with no message boundaries. CAN delivers
//   whole frames or nothing: the controller does framing, a 15-bit CRC and
//   automatic retransmission in silicon. So all of that is gone, and an
//   entire class of bug -- a corrupted length byte stalling the link past the
//   watchdog -- is now structurally impossible rather than merely tested for.
//
//   What survives is the part that was never about the wire: the message
//   fields, the fault bitmask, the command-age semantics and the fail-safe
//   rule. See rover_controller.h.
//
// BYTE ORDER
//   Every multi-byte field is little-endian, written byte by byte rather than
//   memcpy'd from a packed struct. Slightly more code, but it does not depend
//   on the compiler's struct layout or the CPU's endianness, so the Teensy
//   (ARM), the Jetson (ARM) and a developer laptop (x86) cannot disagree.
//   host/rover_protocol.py mirrors this exactly, and a golden-vector test
//   pins the two implementations together byte for byte.

#ifndef ROVER_PROTOCOL_H
#define ROVER_PROTOCOL_H

#include <stdint.h>
#include <stddef.h>

namespace rover {

// ---------------------------------------------------------------------------
// CAN identifiers.
//
// On CAN the identifier is ALSO the priority: arbitration is bitwise and
// dominant-low, so the NUMERICALLY LOWEST id wins the bus, and the loser
// backs off without its message being corrupted. So this table is a priority
// ordering, not just a set of names. Commands outrank telemetry because a
// late command can hurt the rover and late telemetry only annoys an operator.
//
// 0x000-0x0FF is deliberately left free above CONTROL for a future dedicated
// e-stop frame, which must outrank everything here.
// ---------------------------------------------------------------------------

enum : uint32_t {
    CAN_ID_CONTROL       = 0x100,  // Jetson -> Teensy, highest priority in use
    CAN_ID_TELEM_MOTION  = 0x200,  // Teensy -> Jetson, encoders
    CAN_ID_TELEM_STATUS  = 0x201,  // Teensy -> Jetson, steering/current/faults
};

// Operating modes (ControlMsg::mode)
enum : uint8_t {
    MODE_DISABLED   = 0,
    MODE_MANUAL     = 1,
    MODE_AUTONOMOUS = 2,
};

// Indicator Values (ControlMsg: LED indicator)          
enum : uint8_t {
    INDICATOR_OFF   = 0, // mode disabled or watchdog tripped
    INDICATOR_BLUE = 1, // mode_manual lights blue LED
    INDICATOR_RED = 2, // mode_autonomous lights red LED
    INDICATOR_GREEN_FLASH = 3 // target reached by autonomous mode
};              

// Fault bitmask (TelemetryStatus::fault_status). Powers of two so faults
// combine: COMM_TIMEOUT | OVER_CURRENT == 0x03 and both survive.
enum : uint8_t {
    FAULT_COMM_TIMEOUT   = 0x01,  // no valid CONTROL frame within the watchdog
    FAULT_OVER_CURRENT   = 0x02,
    FAULT_ESTOP_ACTIVE   = 0x04,
    FAULT_ENCODER_FAULT  = 0x08,
    FAULT_UNDERVOLTAGE   = 0x10,
    FAULT_FIRMWARE_FAULT = 0x20,  // this boot followed a watchdog reset
    FAULT_PROTOCOL_ERROR = 0x40,  // a frame on our id could not be interpreted
};

// Command age. Two distinct reserved values, because "you have never spoken
// to me" and "you stopped speaking to me 65 seconds ago" call for different
// operator responses: the first is a wiring or bus-config problem, the second
// is something that died mid-mission.
enum : uint16_t {
    CMD_AGE_UNKNOWN = 0xFFFF,  // no valid CONTROL frame has EVER arrived
    CMD_AGE_MAX     = 0xFFFE,  // saturation ceiling for a real measurement
};

// Payload sizes. Every message fits in 8 bytes ON PURPOSE: that is Classic
// CAN's limit, so this protocol runs unchanged on a Classic bus (CAN1/CAN2)
// or a CAN FD bus (CAN3). Designing a 15-byte telemetry frame would have
// locked the rover into FD-capable transceivers on every node, including the
// motor controllers, many of which are Classic-only.
enum : uint8_t {
    CONTROL_DLC      = 7,
    TELEM_MOTION_DLC = 8,
    TELEM_STATUS_DLC = 8,
};

static const int32_t INT32_MIN_V = -2147483647 - 1;
static const int32_t INT32_MAX_V = 2147483647;

// ---------------------------------------------------------------------------
// Messages
// ---------------------------------------------------------------------------

// ---------------------------------------------------------------------------
// Mode validation.
//
// `mode` is a uint8 on the wire, so it can carry any of 256 values while only
// three are defined. Two separate questions follow, and conflating them is the
// bug this pair of functions exists to prevent (issue #4):
//
//   isKnownMode()      -- is this a value this protocol version defines?
//                         DISABLED is KNOWN and VALID; it is a legitimate
//                         command that happens to mean "do not move".
//   modePermitsMotion() -- may the rover move in this mode?
//
// modePermitsMotion is a WHITELIST, deliberately. The original code asked
// "is mode == DISABLED?" and stopped only then, so any undefined value --
// from a protocol mismatch, a faulty sender, or corruption in the software
// path after CAN's CRC has already passed -- read as "not disabled" and
// permitted full throttle. A safety predicate must fail CLOSED: anything this
// firmware does not positively recognise as drivable means stop.
// ---------------------------------------------------------------------------

bool isKnownMode(uint8_t mode);
bool isKnownIndicator(uint8_t indicator_request);
bool modePermitsMotion(uint8_t mode);

struct ControlMsg {
    int16_t drive_cmd;  // -1000..1000, tenths of a percent of full effort
    int16_t steer_cmd;  // -1000..1000, tenths of a percent of full range
    uint8_t mode;       // MODE_*
    uint8_t stop;       // 1 forces an immediate stop regardless of mode
    uint8_t indicator_request; // INDICATOR_* 
};

// Telemetry is split across two frames so each fits Classic CAN's 8 bytes.
// They carry separate ids, so the Jetson can tell which arrived and a lost
// motion frame does not cost it the fault status.
struct TelemetryMotion {
    int32_t enc_left;   // cumulative ticks, WRAPS -- treat as relative
    int32_t enc_right;
};

struct TelemetryStatus {
    int16_t  steer_fb;     // measured steering, same scale as steer_cmd
    int16_t  current_ca;   // SIGNED centiamps (1 cA = 10 mA), +/-327.67 A
    uint8_t  fault_status; // FAULT_* bitmask
    uint16_t cmd_age_ms;   // ms since the last valid CONTROL frame
    uint8_t indicator_state;
};

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

// Wrap an unbounded accumulator into int32 range.
//
// Encoder counters grow without limit. In the Python version an unwrapped
// counter eventually raised inside struct.pack, killing the main loop with
// the motors still energized. C++ would not raise -- signed overflow is
// undefined behaviour, which is worse. Counts are relative anyway; the
// receiver handles rollover when differencing.
int32_t wrapI32(int64_t value);

// Clamp a measured age to the wire range, keeping CMD_AGE_UNKNOWN reserved.
uint16_t clampCmdAgeMs(int64_t age_ms);

// Which fault names are set, for logging. Writes up to `cap` pointers into
// `out` and returns how many were written.
size_t faultNames(uint8_t bitmask, const char** out, size_t cap);

// ---------------------------------------------------------------------------
// Encode / decode.
//
// encode* writes into an 8-byte CAN payload buffer and returns the DLC.
// decode* validates the DLC first and returns false if it does not match --
// the CAN analogue of the UART version's LEN validation. The hardware already
// guarantees the frame is intact; this guards against a peer running a
// different protocol version, which no CRC can catch.
// ---------------------------------------------------------------------------

uint8_t encodeControl(const ControlMsg& msg, uint8_t* buf);
bool    decodeControl(const uint8_t* buf, uint8_t len, ControlMsg& out);

uint8_t encodeTelemetryMotion(const TelemetryMotion& msg, uint8_t* buf);
bool    decodeTelemetryMotion(const uint8_t* buf, uint8_t len, TelemetryMotion& out);

uint8_t encodeTelemetryStatus(const TelemetryStatus& msg, uint8_t* buf);
bool    decodeTelemetryStatus(const uint8_t* buf, uint8_t len, TelemetryStatus& out);

}  // namespace rover

#endif  // ROVER_PROTOCOL_H
