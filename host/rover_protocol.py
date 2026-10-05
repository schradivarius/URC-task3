"""
rover_protocol.py -- Jetson-side codec for the rover CAN messages.

This is the ONLY part of the protocol that exists in two languages. It has to:
the Teensy runs C++ and the Jetson runs Python, and neither can import the
other's source.

So it is pinned rather than trusted. tests/host/test_golden_vectors.py runs the
native C++ encoder and asserts, byte for byte, that this file produces
identical output for the same inputs. If anyone edits one side's field order,
width or endianness, CI fails immediately instead of the two sides silently
disagreeing on a rover at competition.

The safety logic is NOT duplicated here. It lives once, in C++, in
firmware/src/rover_controller.*, and the simulator used by the host tests is
that same compiled code driven over a pipe -- see tools/rover_sim.cpp.

Mirrors firmware/src/rover_protocol.h. Read that file for the design rationale.
"""

import os
import struct
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

# CAN ids, modes, valid command ranges and timing live in rover_config.py.
# Re-exported here so existing callers can keep writing rp.CAN_ID_CONTROL.
from rover_config import (  # noqa: E402,F401
    CAN_ID_CONTROL, CAN_ID_TELEM_MOTION, CAN_ID_TELEM_STATUS,
    MODE_DISABLED, MODE_MANUAL, MODE_AUTONOMOUS, MODE_MAX,
    CMD_MIN, CMD_MAX, PROTOCOL_VERSION,
)

MODE_NAMES = {MODE_DISABLED: "DISABLED", MODE_MANUAL: "MANUAL",
              MODE_AUTONOMOUS: "AUTONOMOUS"}

INDICATOR_OFF = 0       
INDICATOR_BLUE = 1
INDICATOR_RED = 2
INDICATOR_GREEN_FLASH = 3      

# --- fault bitmask ---
FAULT_COMM_TIMEOUT   = 0x01
FAULT_OVER_CURRENT   = 0x02
FAULT_ESTOP_ACTIVE   = 0x04
FAULT_ENCODER_FAULT  = 0x08
FAULT_UNDERVOLTAGE   = 0x10
FAULT_FIRMWARE_FAULT = 0x20
FAULT_PROTOCOL_ERROR = 0x40

FAULT_NAMES = [
    (FAULT_COMM_TIMEOUT, "COMM_TIMEOUT"),
    (FAULT_OVER_CURRENT, "OVER_CURRENT"),
    (FAULT_ESTOP_ACTIVE, "ESTOP_ACTIVE"),
    (FAULT_ENCODER_FAULT, "ENCODER_FAULT"),
    (FAULT_UNDERVOLTAGE, "UNDERVOLTAGE"),
    (FAULT_FIRMWARE_FAULT, "FIRMWARE_FAULT"),
    (FAULT_PROTOCOL_ERROR, "PROTOCOL_ERROR"),
]

# --- command age sentinels ---
CMD_AGE_UNKNOWN = 0xFFFF   # no valid CONTROL frame has EVER arrived
CMD_AGE_MAX     = 0xFFFE   # saturation ceiling for a real measurement

# --- payload layouts. '<' = little-endian, no padding, matching the explicit
# byte-order handling in rover_protocol.cpp. Every message is <= 8 bytes so the
# protocol runs on Classic CAN as well as CAN FD.
CONTROL_FMT      = "<hhBBB"    # drive_cmd, steer_cmd, mode, stop, indicator_request
TELEM_MOTION_FMT = "<ii"      # enc_left, enc_right
TELEM_STATUS_FMT = "<hhBHB"    # steer_fb, current_ca, fault_status, cmd_age_ms, indicator_request

CONTROL_DLC      = struct.calcsize(CONTROL_FMT)       # 7
TELEM_MOTION_DLC = struct.calcsize(TELEM_MOTION_FMT)  # 8
TELEM_STATUS_DLC = struct.calcsize(TELEM_STATUS_FMT)  # 8

VALID_DLC = {
    CAN_ID_CONTROL:      CONTROL_DLC,
    CAN_ID_TELEM_MOTION: TELEM_MOTION_DLC,
    CAN_ID_TELEM_STATUS: TELEM_STATUS_DLC,
}

INT32_MIN = -(2 ** 31)
INT32_MAX = 2 ** 31 - 1


KNOWN_MODES = (MODE_DISABLED, MODE_MANUAL, MODE_AUTONOMOUS)
KNOWN_INDICATORS = (INDICATOR_OFF, INDICATOR_BLUE, INDICATOR_RED, INDICATOR_GREEN_FLASH)
MOTION_MODES = (MODE_MANUAL, MODE_AUTONOMOUS)


def fault_names(bitmask):
    return [name for bit, name in FAULT_NAMES if bitmask & bit]


def is_known_mode(mode):
    """Is this a mode value the protocol defines? DISABLED counts: it is a
    legitimate command that happens to mean "do not move"."""
    return mode in KNOWN_MODES


def mode_permits_motion(mode):
    """May the rover move in this mode? A WHITELIST, deliberately.

    `mode` is a uint8, so 256 values fit where 3 are defined. Asking only
    "is it DISABLED?" let every undefined value read as drivable (issue #4).
    Mirrors modePermitsMotion() in firmware/src/rover_protocol.cpp.
    """
    return mode in MOTION_MODES


def wrap_i32(value):
    """Wrap an unbounded accumulator into int32 range, matching wrapI32()."""
    return ((value - INT32_MIN) % (2 ** 32)) + INT32_MIN


def clamp_cmd_age_ms(age_ms):
    """Clamp to the wire range, keeping CMD_AGE_UNKNOWN reserved."""
    if age_ms < 0:
        return 0
    return age_ms if age_ms < CMD_AGE_MAX else CMD_AGE_MAX


# --- encode ---------------------------------------------------------------

def encode_control(drive_cmd, steer_cmd, mode, stop, indicator_request = INDICATOR_OFF):
    return struct.pack(CONTROL_FMT, drive_cmd, steer_cmd, mode, 1 if stop else 0, indicator_request)


def encode_telemetry_motion(enc_left, enc_right):
    return struct.pack(TELEM_MOTION_FMT, wrap_i32(enc_left), wrap_i32(enc_right))


def encode_telemetry_status(steer_fb, current_ca, fault_status, cmd_age_ms, indicator_state = INDICATOR_OFF):
    return struct.pack(TELEM_STATUS_FMT, steer_fb, current_ca,
                       fault_status, cmd_age_ms, indicator_state)


# --- decode. Each validates the DLC first, exactly as the C++ side does: CAN
# guarantees the frame arrived intact, but not that the sender agrees on what
# the bytes mean. A DLC mismatch is the cheap signal of a version mismatch.

def decode_control(payload):
    if len(payload) != CONTROL_DLC:
        return None
    drive, steer, mode, stop, indicator_request = struct.unpack(CONTROL_FMT, payload)
    # An undefined mode means the sender disagrees with us about the protocol,
    # exactly like a wrong DLC. Reject the frame rather than returning a value
    # the caller cannot reason about. Mirrors decodeControl() in C++.
    if not is_known_mode(mode):
        return None
    if indicator_request not in KNOWN_INDICATORS:
        return None 
    return {"drive_cmd": drive, "steer_cmd": steer, "mode": mode, "stop": bool(stop), "indicator_request": indicator_request}


def decode_telemetry_motion(payload):
    if len(payload) != TELEM_MOTION_DLC:
        return None
    enc_left, enc_right = struct.unpack(TELEM_MOTION_FMT, payload)
    return {"enc_left": enc_left, "enc_right": enc_right}


def decode_telemetry_status(payload):
    if len(payload) != TELEM_STATUS_DLC:
        return None
    steer_fb, current_ca, faults, age, state = struct.unpack(TELEM_STATUS_FMT, payload)
    if state not in KNOWN_INDICATORS:                                                     # ADD
        state = None          # "the MCU sent something I don't understand"
    return {"steer_fb": steer_fb, "current_ca": current_ca,
            "current_a": current_ca / 100.0,
            "fault_status": faults, "cmd_age_ms": age, "indicator_state": state}
