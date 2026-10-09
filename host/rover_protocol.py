"""
rover_protocol.py -- Jetson-side codec for the rover CAN messages.

This is the ONLY part of the protocol that exists in two languages. It has to:
the Teensy runs C++ and the Jetson runs Python, and neither can import the
other's source.

So it is pinned rather than trusted. tests/host/test_golden_vectors.py runs the
native C++ encoder and asserts, byte for byte, that this file produces
identical output for the same inputs, and that both sides agree on every
validation predicate. If anyone edits one side's field order, width,
endianness or CRC, CI fails immediately instead of the two sides silently
disagreeing on a rover at competition.

The safety logic is NOT duplicated here. It lives once, in C++, in
firmware/src/rover_controller.*, and the simulator the host tests run against
is that same compiled code driven over a pipe -- see tools/rover_sim.cpp.

Mirrors firmware/src/rover_protocol.h. Read that file for the design
rationale; PROTOCOL.md section 3 has the full field documentation.
"""

import struct

# --- CAN identifiers. Also the bus priority: lowest id wins arbitration. ---
CAN_ID_CONTROL       = 0x100
CAN_ID_TELEM_DRIVE_L = 0x200
CAN_ID_TELEM_DRIVE_R = 0x201
CAN_ID_TELEM_POWER   = 0x202
CAN_ID_TELEM_STATE   = 0x203

TELEM_IDS = (CAN_ID_TELEM_DRIVE_L, CAN_ID_TELEM_DRIVE_R,
             CAN_ID_TELEM_POWER, CAN_ID_TELEM_STATE)

# --- operating modes ---
MODE_DISABLED   = 0
MODE_MANUAL     = 1
MODE_AUTONOMOUS = 2
MODE_NAMES = {MODE_DISABLED: "DISABLED", MODE_MANUAL: "MANUAL",
              MODE_AUTONOMOUS: "AUTONOMOUS"}

KNOWN_MODES  = (MODE_DISABLED, MODE_MANUAL, MODE_AUTONOMOUS)
MOTION_MODES = (MODE_MANUAL, MODE_AUTONOMOUS)

# --- CONTROL flags byte (wire byte 5) ---
CTRL_FLAG_STOP           = 0x01
CTRL_FLAG_AUTONOMY_ABORT = 0x02
CTRL_FLAG_RETURN_REQUEST = 0x04
CTRL_INDICATOR_MASK      = 0x38   # bits 3-5
CTRL_INDICATOR_SHIFT     = 3
CTRL_FLAG_C2_LOST        = 0x40   # bit 6: the Jetson reports the C2 link down
CTRL_FLAG_RESERVED_MASK  = 0x80   # bit 7, must be zero

# --- status indicator ---
INDICATOR_OFF        = 0
INDICATOR_TELEOP     = 1
INDICATOR_AUTONOMOUS = 2
INDICATOR_ARRIVED    = 3
INDICATOR_FAULT      = 4
INDICATOR_MAX        = 7
INDICATOR_NAMES = {INDICATOR_OFF: "OFF", INDICATOR_TELEOP: "TELEOP",
                   INDICATOR_AUTONOMOUS: "AUTONOMOUS",
                   INDICATOR_ARRIVED: "ARRIVED", INDICATOR_FAULT: "FAULT"}

# --- fault flags, uint16 since v0.4 ---
FAULT_COMM_TIMEOUT   = 0x0001
FAULT_OVER_CURRENT   = 0x0002
FAULT_ESTOP_ACTIVE   = 0x0004
FAULT_ENCODER_FAULT  = 0x0008
FAULT_UNDERVOLTAGE   = 0x0010
FAULT_FIRMWARE_FAULT = 0x0020
FAULT_PROTOCOL_ERROR = 0x0040
FAULT_C2_LINK_LOST   = 0x0080
FAULT_SEQ_GAP        = 0x0100
FAULT_CRC_ERROR      = 0x0200

FAULT_NAMES = [
    (FAULT_COMM_TIMEOUT, "COMM_TIMEOUT"),
    (FAULT_OVER_CURRENT, "OVER_CURRENT"),
    (FAULT_ESTOP_ACTIVE, "ESTOP_ACTIVE"),
    (FAULT_ENCODER_FAULT, "ENCODER_FAULT"),
    (FAULT_UNDERVOLTAGE, "UNDERVOLTAGE"),
    (FAULT_FIRMWARE_FAULT, "FIRMWARE_FAULT"),
    (FAULT_PROTOCOL_ERROR, "PROTOCOL_ERROR"),
    (FAULT_C2_LINK_LOST, "C2_LINK_LOST"),
    (FAULT_SEQ_GAP, "SEQ_GAP"),
    (FAULT_CRC_ERROR, "CRC_ERROR"),
]

# --- link health. Jetson and C2 are SEPARATE and never collapsed. ---
LINK_OK           = 0
LINK_DEGRADED     = 1
LINK_LOST         = 2
LINK_NOT_REPORTED = 3
LINK_NAMES = {LINK_OK: "OK", LINK_DEGRADED: "DEGRADED",
              LINK_LOST: "LOST", LINK_NOT_REPORTED: "NOT_REPORTED"}

# --- low-level controller health ---
CTRL_HEALTH_OK           = 0
CTRL_HEALTH_DEGRADED     = 1
CTRL_HEALTH_FAULT        = 2
CTRL_HEALTH_NOT_REPORTED = 3
CTRL_HEALTH_NAMES = {CTRL_HEALTH_OK: "OK", CTRL_HEALTH_DEGRADED: "DEGRADED",
                     CTRL_HEALTH_FAULT: "FAULT",
                     CTRL_HEALTH_NOT_REPORTED: "NOT_REPORTED"}

# --- command age sentinels ---
CMD_AGE_UNKNOWN = 0xFFFF
CMD_AGE_MAX     = 0xFFFE

# --- frame geometry: 6 payload bytes + seq + CRC = 8 ---
FRAME_DLC        = 8
FRAME_PAYLOAD    = 6
FRAME_SEQ_OFFSET = 6
FRAME_CRC_OFFSET = 7

# Payload layouts. '<' = little-endian, no padding, matching the explicit
# byte-order handling in rover_protocol.cpp.
CONTROL_FMT = "<hhBB"   # drive_cmd, steer_cmd, mode, flags
DRIVE_L_FMT = "<ih"     # enc_left, steer_fb
DRIVE_R_FMT = "<iH"     # enc_right, cmd_age_ms
POWER_FMT   = "<hhH"    # current_ca, voltage_cv, fault_status
STATE_FMT   = "<6B"     # mode, jetson_link, c2_link, ctrl_health, indicator, rsvd

INT32_MIN = -(2 ** 31)
INT32_MAX = 2 ** 31 - 1

# --- decode results, mirroring enum DecodeResult ---
DECODE_OK            = 0
DECODE_BAD_DLC       = 1
DECODE_BAD_CRC       = 2
DECODE_BAD_MODE      = 3
DECODE_BAD_FLAGS     = 4
DECODE_BAD_INDICATOR = 5
DECODE_NAMES = {DECODE_OK: "OK", DECODE_BAD_DLC: "BAD_DLC",
                DECODE_BAD_CRC: "BAD_CRC", DECODE_BAD_MODE: "BAD_MODE",
                DECODE_BAD_FLAGS: "BAD_FLAGS",
                DECODE_BAD_INDICATOR: "BAD_INDICATOR"}


# ---------------------------------------------------------------------------
# Predicates and helpers -- mirror the C++ exactly
# ---------------------------------------------------------------------------

def fault_names(bitmask):
    return [name for bit, name in FAULT_NAMES if bitmask & bit]


def is_known_mode(mode):
    """Is this a mode the protocol defines? DISABLED counts: it is a legitimate
    command that happens to mean "do not move"."""
    return mode in KNOWN_MODES


def mode_permits_motion(mode):
    """May the rover move in this mode? A WHITELIST, deliberately -- `mode` is a
    uint8, so 256 values fit where 3 are defined."""
    return mode in MOTION_MODES


def is_known_indicator(indicator):
    return 0 <= indicator <= INDICATOR_FAULT


def wrap_i32(value):
    """Wrap an unbounded accumulator into int32 range; mirrors wrapI32()."""
    return ((value - INT32_MIN) % (2 ** 32)) + INT32_MIN


def clamp_cmd_age_ms(age_ms):
    if age_ms < 0:
        return 0
    return age_ms if age_ms < CMD_AGE_MAX else CMD_AGE_MAX


def seq_delta(previous, current):
    """Frames elapsed from `previous` to `current`, correct across the 255->0
    wrap. 1 is healthy, 0 is a duplicate, >1 means frames were lost."""
    return (current - previous) & 0xFF


# ---------------------------------------------------------------------------
# CRC-8 / SAE-J1850: poly 0x1D, init 0xFF, no reflection, final XOR 0xFF.
# Known-answer check value over b"123456789" is 0x4B.
#
# NOTE: for a CRC-8 the byte is XORed into the WHOLE register. For a CRC-16 it
# goes into the high byte. Copying a CRC-16 loop is the classic way to get a
# plausible-looking wrong answer.
# ---------------------------------------------------------------------------

def crc8_update(crc, data):
    """Running CRC with no init and no final XOR, so it can be chained."""
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1D) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc


def crc8(data):
    """Complete CRC over one buffer: init 0xFF, final XOR 0xFF."""
    return crc8_update(0xFF, data) ^ 0xFF


def frame_crc8(can_id, payload):
    """Frame CRC seeded with the message identifier, AUTOSAR Data-ID style, so
    a payload delivered on the wrong CAN id fails validation. The id is chained
    low byte first, matching the protocol's little-endian convention."""
    id_bytes = bytes((can_id & 0xFF, (can_id >> 8) & 0xFF,
                      (can_id >> 16) & 0xFF, (can_id >> 24) & 0xFF))
    return crc8_update(crc8_update(0xFF, id_bytes), payload) ^ 0xFF


def _seal(can_id, payload, seq):
    """Append the sequence number and CRC to a 6-byte payload -> 8 bytes."""
    assert len(payload) == FRAME_PAYLOAD, "payload must be 6 bytes"
    body = payload + bytes((seq & 0xFF,))
    return body + bytes((frame_crc8(can_id, body),))


def _open(can_id, frame):
    """Validate length and CRC. Returns (result, payload, seq)."""
    if len(frame) != FRAME_DLC:
        return DECODE_BAD_DLC, None, None
    body = frame[:FRAME_CRC_OFFSET]
    if frame_crc8(can_id, body) != frame[FRAME_CRC_OFFSET]:
        return DECODE_BAD_CRC, None, None
    return DECODE_OK, frame[:FRAME_PAYLOAD], frame[FRAME_SEQ_OFFSET]


# ---------------------------------------------------------------------------
# CONTROL
# ---------------------------------------------------------------------------

def pack_control_flags(c2_lost, stop=False, autonomy_abort=False,
                       return_request=False, indicator_request=INDICATOR_OFF):
    flags = 0
    if stop:            flags |= CTRL_FLAG_STOP
    if autonomy_abort:  flags |= CTRL_FLAG_AUTONOMY_ABORT
    if return_request:  flags |= CTRL_FLAG_RETURN_REQUEST
    if c2_lost:         flags |= CTRL_FLAG_C2_LOST
    flags |= (indicator_request << CTRL_INDICATOR_SHIFT) & CTRL_INDICATOR_MASK
    return flags


def encode_control(drive_cmd, steer_cmd, mode, c2_lost, stop=False, seq=0,
                   autonomy_abort=False, return_request=False,
                   indicator_request=INDICATOR_OFF, raw_flags=None):
    """Encode a CONTROL frame. `raw_flags` bypasses pack_control_flags so tests
    can put an arbitrary byte on the wire, the way a faulty sender would.

    `c2_lost` has NO DEFAULT on purpose. It is the only field in this frame
    the controller cannot determine for itself, and a caller that forgot it
    would silently report the base-station link as healthy -- which is the one
    direction of error that lets a rover keep driving in TELEOP with nobody
    able to reach it. A missing argument is a loud failure instead.
    """
    flags = pack_control_flags(c2_lost, stop, autonomy_abort, return_request,
                               indicator_request) if raw_flags is None else raw_flags
    payload = struct.pack(CONTROL_FMT, drive_cmd, steer_cmd, mode, flags)
    return _seal(CAN_ID_CONTROL, payload, seq)


def decode_control(frame, can_id=CAN_ID_CONTROL):
    """Returns (result, dict or None). Mirrors decodeControl() exactly."""
    result, payload, seq = _open(can_id, frame)
    if result != DECODE_OK:
        return result, None
    drive, steer, mode, flags = struct.unpack(CONTROL_FMT, payload)
    if not is_known_mode(mode):
        return DECODE_BAD_MODE, None
    if flags & CTRL_FLAG_RESERVED_MASK:
        return DECODE_BAD_FLAGS, None
    indicator = (flags & CTRL_INDICATOR_MASK) >> CTRL_INDICATOR_SHIFT
    if not is_known_indicator(indicator):
        return DECODE_BAD_INDICATOR, None
    return DECODE_OK, {
        "drive_cmd": drive, "steer_cmd": steer, "mode": mode,
        "stop": bool(flags & CTRL_FLAG_STOP),
        "autonomy_abort": bool(flags & CTRL_FLAG_AUTONOMY_ABORT),
        "return_request": bool(flags & CTRL_FLAG_RETURN_REQUEST),
        "c2_lost": bool(flags & CTRL_FLAG_C2_LOST),
        "indicator_request": indicator, "seq": seq,
    }


# ---------------------------------------------------------------------------
# TELEMETRY
# ---------------------------------------------------------------------------

def encode_telemetry_drive_l(enc_left, steer_fb, seq=0):
    return _seal(CAN_ID_TELEM_DRIVE_L,
                 struct.pack(DRIVE_L_FMT, wrap_i32(enc_left), steer_fb), seq)


def encode_telemetry_drive_r(enc_right, cmd_age_ms, seq=0):
    return _seal(CAN_ID_TELEM_DRIVE_R,
                 struct.pack(DRIVE_R_FMT, wrap_i32(enc_right), cmd_age_ms), seq)


def encode_telemetry_power(current_ca, voltage_cv, fault_status, seq=0):
    return _seal(CAN_ID_TELEM_POWER,
                 struct.pack(POWER_FMT, current_ca, voltage_cv, fault_status), seq)


def encode_telemetry_state(mode, jetson_link, c2_link, controller_health,
                           indicator_state, seq=0, reserved=0):
    return _seal(CAN_ID_TELEM_STATE,
                 struct.pack(STATE_FMT, mode, jetson_link, c2_link,
                             controller_health, indicator_state, reserved), seq)


def decode_telemetry_drive_l(frame, can_id=CAN_ID_TELEM_DRIVE_L):
    result, payload, seq = _open(can_id, frame)
    if result != DECODE_OK:
        return result, None
    enc_left, steer_fb = struct.unpack(DRIVE_L_FMT, payload)
    return DECODE_OK, {"enc_left": enc_left, "steer_fb": steer_fb, "seq": seq}


def decode_telemetry_drive_r(frame, can_id=CAN_ID_TELEM_DRIVE_R):
    result, payload, seq = _open(can_id, frame)
    if result != DECODE_OK:
        return result, None
    enc_right, cmd_age = struct.unpack(DRIVE_R_FMT, payload)
    return DECODE_OK, {"enc_right": enc_right, "cmd_age_ms": cmd_age, "seq": seq}


def decode_telemetry_power(frame, can_id=CAN_ID_TELEM_POWER):
    result, payload, seq = _open(can_id, frame)
    if result != DECODE_OK:
        return result, None
    current_ca, voltage_cv, faults = struct.unpack(POWER_FMT, payload)
    return DECODE_OK, {
        "current_ca": current_ca, "current_a": current_ca / 100.0,
        "voltage_cv": voltage_cv, "voltage_v": voltage_cv / 100.0,
        "fault_status": faults, "seq": seq,
    }


def decode_telemetry_state(frame, can_id=CAN_ID_TELEM_STATE):
    result, payload, seq = _open(can_id, frame)
    if result != DECODE_OK:
        return result, None
    mode, jlink, c2link, health, indicator, reserved = struct.unpack(STATE_FMT, payload)
    return DECODE_OK, {
        "mode": mode, "jetson_link": jlink, "c2_link": c2link,
        "controller_health": health, "indicator_state": indicator,
        "reserved": reserved, "seq": seq,
    }


DECODERS = {
    CAN_ID_TELEM_DRIVE_L: decode_telemetry_drive_l,
    CAN_ID_TELEM_DRIVE_R: decode_telemetry_drive_r,
    CAN_ID_TELEM_POWER:   decode_telemetry_power,
    CAN_ID_TELEM_STATE:   decode_telemetry_state,
}
