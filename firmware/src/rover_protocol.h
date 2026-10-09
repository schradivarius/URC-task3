// rover_protocol.h -- Rover Jetson <-> Teensy CAN message definitions.
//
// PURE C++. No Arduino.h, no FlexCAN, no hardware. That is deliberate: this
// file and rover_controller.* compile unchanged for the Teensy AND natively on
// a laptop, so the safety-critical logic is unit-tested by `make test` with no
// board attached. Only rover_firmware.ino touches hardware.
//
// ---------------------------------------------------------------------------
// v0.4: structured command and telemetry packets
// ---------------------------------------------------------------------------
// Adds, to both directions: a sequence number, an application-layer CRC-8, and
// the command/status fields the 2027 requirements call for.
//
// WHY AN APPLICATION-LAYER CRC WHEN CAN ALREADY HAS ONE
//   CAN's 15-bit CRC is computed by the transmitting controller and checked by
//   the receiving controller. It protects the WIRE between those two chips.
//   Everything outside that span is unprotected: the sender's software
//   assembling the struct, DMA between memory and the CAN peripheral, the
//   receiver's driver copying the frame into our buffer. A bug or a memory
//   fault in any of those produces a frame CAN considers perfectly valid and
//   delivers with wrong contents.
//
//   The CRC here is computed by OUR code over OUR struct and checked by OUR
//   code on the far side, so it covers the whole path end to end. This is the
//   same reasoning behind AUTOSAR's E2E protection, which also layers a CRC
//   on top of CAN's. Paired with the sequence number -- which catches loss,
//   duplication and reordering, things a CRC cannot see -- the two together
//   are what make the link end-to-end protected rather than merely wire-
//   protected.
//
//   Do not "simplify" it away as redundant. It is not redundant; it covers a
//   different failure domain.
//
// WHY EVERY FRAME IS STILL 8 BYTES
//   8 bytes is Classic CAN's limit, so this protocol runs unchanged on CAN1 or
//   CAN2 (Classic) and on CAN3 (CAN FD), and alongside motor controllers that
//   are commonly Classic-only. Telemetry is therefore SPLIT ACROSS FOUR
//   FRAMES rather than sent as one CAN FD frame. See the note on snapshot
//   tearing at CAN_ID_TELEM_DRIVE_L.
//
// BYTE ORDER
//   Little-endian, written byte by byte rather than memcpy'd from a packed
//   struct, so nothing depends on compiler padding or CPU endianness. The
//   Teensy (ARM), the Jetson (ARM) and a dev laptop (x86) cannot disagree.
//   host/rover_protocol.py mirrors this, and tests/host/test_golden_vectors.py
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
// dominant-low, so the numerically lowest id wins the bus and the loser backs
// off without its message being corrupted. This table is a priority ordering.
//
// 0x000-0x0FF stays free above CONTROL for a future dedicated e-stop frame,
// which must outrank everything here.
// ---------------------------------------------------------------------------

enum : uint32_t {
    CAN_ID_CONTROL       = 0x100,  // Jetson -> Teensy. Highest priority in use.

    // Telemetry, Teensy -> Jetson. Four frames because each must fit Classic
    // CAN's 8 bytes once the per-frame sequence number and CRC are accounted
    // for, leaving 6 payload bytes each.
    //
    // SNAPSHOT TEARING: every frame of one control cycle carries the SAME
    // sequence number. The Jetson can therefore tell whether the four frames
    // it holds came from one cycle or straddle two, and discard or flag a torn
    // snapshot rather than silently mixing a fresh encoder reading with a
    // stale fault word. A single CAN FD frame would be atomic and avoid this
    // entirely; that is the main argument for moving telemetry to CAN3 in FD
    // mode later, and the field definitions below would not change.
    CAN_ID_TELEM_DRIVE_L = 0x200,  // left encoder + steering feedback
    CAN_ID_TELEM_DRIVE_R = 0x201,  // right encoder + command age
    CAN_ID_TELEM_POWER   = 0x202,  // current, voltage, fault word
    CAN_ID_TELEM_STATE   = 0x203,  // mode, link health, controller health
};

// Operating modes (ControlMsg::mode)
enum : uint8_t {
    MODE_DISABLED   = 0,
    MODE_MANUAL     = 1,
    MODE_AUTONOMOUS = 2,
};

// ---------------------------------------------------------------------------
// CONTROL flags byte (ControlMsg::flags, wire byte 5).
//
// Packed into one byte because CONTROL lands on exactly 8 bytes with the
// sequence number and CRC included, and there was no room for five separate
// byte-wide fields.
//
//   bit  0     stop            1 = stop now, regardless of mode
//   bit  1     autonomy_abort  1 = abandon the current autonomous task
//   bit  2     return_request  1 = begin the return-to-base behaviour
//   bits 3-5   indicator_request  requested status-indicator state (0-7)
//   bit  6     c2_lost         1 = the Jetson has lost the base-station link
//   bit  7     reserved, MUST be transmitted as 0
//
// c2_lost is the Jetson's REPORT on a link the controller cannot see. The
// controller detects a silent Jetson itself (the command watchdog), but it has
// no radio and therefore no view of the base station <-> Jetson link. So the
// Jetson watches that link (host/c2_link.py) and forwards the verdict in every
// CONTROL frame. The two conditions must stay separate: C2 loss with a healthy
// Jetson is EXPECTED on the autonomy course, where line of sight to the base
// station drops while onboard autonomy keeps running.
// ---------------------------------------------------------------------------

enum : uint8_t {
    CTRL_FLAG_STOP           = 0x01,
    CTRL_FLAG_AUTONOMY_ABORT = 0x02,
    CTRL_FLAG_RETURN_REQUEST = 0x04,
    CTRL_INDICATOR_MASK      = 0x38,  // bits 3-5
    CTRL_INDICATOR_SHIFT     = 3,
    CTRL_FLAG_C2_LOST        = 0x40,  // bit 6
    CTRL_FLAG_RESERVED_MASK  = 0x80,  // bit 7, must be zero
};

// Status-indicator states. The same enum is used for the Jetson's REQUEST
// (CONTROL) and the controller's REPORT of what is actually displayed
// (TELEM_STATE), so a mismatch between the two is directly visible.
enum : uint8_t {
    INDICATOR_OFF        = 0,
    INDICATOR_TELEOP     = 1,  // under operator control
    INDICATOR_AUTONOMOUS = 2,  // driving itself
    INDICATOR_ARRIVED    = 3,  // goal reached / waypoint complete
    INDICATOR_FAULT      = 4,  // something is wrong, see fault_status
    INDICATOR_MAX        = 7,  // 3 bits on the wire
};

// ---------------------------------------------------------------------------
// Fault flags (TelemetryPower::fault_status).
//
// WIDENED TO uint16 in v0.4. The uint8 version had exactly one bit left, and
// this revision needs several. Bits 0x0001-0x0040 keep their v0.3 values so
// existing code and docs stay valid, including 0x0080 for the C2-link fault.
//
// Powers of two so faults combine: COMM_TIMEOUT | OVER_CURRENT == 0x0003 and
// both survive.
// ---------------------------------------------------------------------------

enum : uint16_t {
    FAULT_COMM_TIMEOUT   = 0x0001,  // no valid CONTROL frame within the watchdog
    FAULT_OVER_CURRENT   = 0x0002,
    FAULT_ESTOP_ACTIVE   = 0x0004,
    FAULT_ENCODER_FAULT  = 0x0008,
    FAULT_UNDERVOLTAGE   = 0x0010,
    FAULT_FIRMWARE_FAULT = 0x0020,  // this boot followed a watchdog reset
    FAULT_PROTOCOL_ERROR = 0x0040,  // a frame on our id could not be interpreted
    FAULT_C2_LINK_LOST   = 0x0080,  // the last CONTROL frame reported C2 lost
    FAULT_SEQ_GAP        = 0x0100,  // CONTROL frames were lost or reordered
    FAULT_CRC_ERROR      = 0x0200,  // app-layer CRC mismatch on our id
};

// Link health, reported for each link independently.
//
// The Jetson link and the C2 link are SEPARATE and must never be collapsed:
// the 2027 autonomy course deliberately includes areas with no C2
// line-of-sight while onboard autonomy keeps working normally.
enum : uint8_t {
    LINK_OK           = 0,
    LINK_DEGRADED     = 1,  // intermittent: losses or gaps seen recently
    LINK_LOST         = 2,
    LINK_NOT_REPORTED = 3,  // nobody has told us about this link, OR what we
                            // were told has gone stale -- see c2LinkState()
};

// Low-level controller (motor driver / ESC) aggregate health.
enum : uint8_t {
    CTRL_HEALTH_OK           = 0,
    CTRL_HEALTH_DEGRADED     = 1,  // at least one channel reporting a problem
    CTRL_HEALTH_FAULT        = 2,  // at least one channel unusable
    CTRL_HEALTH_NOT_REPORTED = 3,  // no driver telemetry wired up yet
};

// Command age. Two distinct reserved values, because "you have never spoken to
// me" is a wiring or bus-config problem while "you stopped speaking 65 seconds
// ago" is something that died mid-mission.
enum : uint16_t {
    CMD_AGE_UNKNOWN = 0xFFFF,
    CMD_AGE_MAX     = 0xFFFE,
};

// Every frame is 8 bytes: 6 payload + 1 sequence + 1 CRC.
enum : uint8_t {
    FRAME_DLC        = 8,
    FRAME_PAYLOAD    = 6,  // bytes 0..5
    FRAME_SEQ_OFFSET = 6,
    FRAME_CRC_OFFSET = 7,
};

static const int32_t INT32_MIN_V = -2147483647 - 1;
static const int32_t INT32_MAX_V = 2147483647;

bool isKnownMode(uint8_t mode);
bool modePermitsMotion(uint8_t mode);
bool isKnownIndicator(uint8_t indicator);

// ---------------------------------------------------------------------------
// CRC-8 / SAE-J1850 -- poly 0x1D, init 0xFF, no reflection, final XOR 0xFF.
// Known-answer check value over "123456789" is 0x4B, pinned in tests.
// (This is what AUTOSAR E2E profiles 1 and 2 use.)
// ---------------------------------------------------------------------------

// Running CRC with no init and no final XOR, so callers can chain it across
// several buffers. This is the only one with a bit loop in it.
uint8_t crc8Update(uint8_t crc, const uint8_t* data, size_t len);

// Complete CRC over one buffer: init 0xFF, chain, final XOR 0xFF.
uint8_t crc8(const uint8_t* data, size_t len);

// Frame CRC, seeded with the message identifier the way AUTOSAR seeds with a
// Data ID. This ties the CRC to WHICH message it is, so a payload delivered on
// the wrong CAN id fails validation instead of being silently accepted. The id
// is chained low byte first, matching the protocol's little-endian convention.
uint8_t frameCrc8(uint32_t can_id, const uint8_t* payload, size_t len);

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

// Wrap an unbounded accumulator into int32 range. Encoder counters grow
// without limit, and letting an int32 overflow is UNDEFINED BEHAVIOUR in C++
// -- not a wrap, not an exception, but a compiler free to do anything. Counts
// are relative anyway; the receiver handles rollover when differencing.
int32_t wrapI32(int64_t value);

// Clamp a measured age to the wire range, keeping CMD_AGE_UNKNOWN reserved.
uint16_t clampCmdAgeMs(int64_t age_ms);

// Which fault names are set, for logging. Writes up to `cap` pointers into
// `out` and returns how many were written.
size_t faultNames(uint16_t bitmask, const char** out, size_t cap);

// Difference between two wrapping uint8 sequence numbers: how many frames
// elapsed from `previous` to `current`. 1 is the healthy case, 0 is a
// duplicate, >1 means frames were lost. Correct across the 255->0 wrap.
uint8_t seqDelta(uint8_t previous, uint8_t current);

// ---------------------------------------------------------------------------
// Messages
//
// Every struct below is 6 bytes on the wire. encode* appends the sequence
// number at byte 6 and the CRC at byte 7, giving DLC 8.
//
// Full field documentation -- type, units, range, meaning, update rate -- is
// in PROTOCOL.md section 3. The comments here are the short form.
// ---------------------------------------------------------------------------

// Jetson -> controller. CAN_ID_CONTROL, DLC 8, 20 Hz.
struct ControlMsg {
    int16_t drive_cmd;         // -1000..1000, tenths of a percent of full effort
    int16_t steer_cmd;         // -1000..1000, tenths of a percent of full range
    uint8_t mode;              // MODE_*
    bool    stop;              // stop now, regardless of mode
    bool    autonomy_abort;    // abandon the current autonomous task
    bool    return_request;    // begin the return-to-base behaviour
    uint8_t indicator_request; // INDICATOR_*, 0..7
    bool    c2_lost;           // the Jetson reports the base-station link down
    uint8_t seq;               // increments once per frame, wraps 255->0
};

// Controller -> Jetson, four frames, all 20 Hz, all sharing one cycle's seq.

struct TelemetryDriveL {
    int32_t enc_left;      // cumulative ticks, WRAPS -- treat as relative
    int16_t steer_fb;      // measured steering, same scale as steer_cmd
};

struct TelemetryDriveR {
    int32_t enc_right;     // cumulative ticks, WRAPS
    uint16_t cmd_age_ms;   // ms since the last valid CONTROL frame; see CMD_AGE_*
};

struct TelemetryPower {
    int16_t  current_ca;   // SIGNED centiamps (1 cA = 10 mA), +/-327.67 A
    int16_t  voltage_cv;   // SIGNED centivolts (1 cV = 10 mV), +/-327.67 V
    uint16_t fault_status; // FAULT_* bitmask
};

struct TelemetryState {
    uint8_t mode;              // MODE_* the controller believes it is in --
                               // an ECHO, so the operator can confirm the
                               // controller agrees with what was commanded
    uint8_t jetson_link;       // LINK_*
    uint8_t c2_link;           // LINK_*, from the c2_lost bit the Jetson
                               // forwards; LINK_NOT_REPORTED when that bit is
                               // absent or stale -- see c2LinkState()
    uint8_t controller_health; // CTRL_HEALTH_*
    uint8_t indicator_state;   // INDICATOR_* actually being displayed
    uint8_t reserved;          // transmit as 0
};

// ---------------------------------------------------------------------------
// Encode / decode.
//
// decode* returns a REASON rather than a bool, so the caller can report the
// right fault: a CRC mismatch and an undefined mode are both "this sender
// disagrees with us", but they are worth distinguishing in telemetry when
// you are debugging a link at a competition.
//
// The CAN id is an input to decoding because the CRC is seeded with it. A
// payload delivered on the wrong id therefore fails, rather than being
// accepted as a different message.
// ---------------------------------------------------------------------------

enum DecodeResult : uint8_t {
    DECODE_OK = 0,
    DECODE_BAD_DLC,        // wrong frame length
    DECODE_BAD_CRC,        // app-layer CRC mismatch
    DECODE_BAD_MODE,       // mode value this firmware does not define
    DECODE_BAD_FLAGS,      // reserved flag bits were set
    DECODE_BAD_INDICATOR,  // indicator request out of range
};

const char* decodeResultName(DecodeResult r);

uint8_t      encodeControl(const ControlMsg& msg, uint8_t* buf);
DecodeResult decodeControl(uint32_t can_id, const uint8_t* buf, uint8_t len,
                           ControlMsg& out);

uint8_t      encodeTelemetryDriveL(const TelemetryDriveL& msg, uint8_t seq, uint8_t* buf);
DecodeResult decodeTelemetryDriveL(uint32_t can_id, const uint8_t* buf, uint8_t len,
                                   TelemetryDriveL& out, uint8_t& seq_out);

uint8_t      encodeTelemetryDriveR(const TelemetryDriveR& msg, uint8_t seq, uint8_t* buf);
DecodeResult decodeTelemetryDriveR(uint32_t can_id, const uint8_t* buf, uint8_t len,
                                   TelemetryDriveR& out, uint8_t& seq_out);

uint8_t      encodeTelemetryPower(const TelemetryPower& msg, uint8_t seq, uint8_t* buf);
DecodeResult decodeTelemetryPower(uint32_t can_id, const uint8_t* buf, uint8_t len,
                                  TelemetryPower& out, uint8_t& seq_out);

uint8_t      encodeTelemetryState(const TelemetryState& msg, uint8_t seq, uint8_t* buf);
DecodeResult decodeTelemetryState(uint32_t can_id, const uint8_t* buf, uint8_t len,
                                  TelemetryState& out, uint8_t& seq_out);

}  // namespace rover

#endif  // ROVER_PROTOCOL_H
