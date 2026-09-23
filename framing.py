"""
framing.py -- Rover Jetson<->Pico link: byte-level framing, CRC, and message
(de)serialization.

This module is the reference implementation of the wire format described in
PROTOCOL.md. It is used directly by the Jetson-side code (jetson_test.py,
pico_sim.py, demo.py). The Pico firmware (pico_main.py) contains a
byte-for-byte equivalent implementation, because MicroPython on the Pico
does not share this file over the serial link -- pico_main.py is a
standalone file you copy onto the board. Keep the two in sync if you ever
change the wire format; PROTOCOL.md is the source of truth for the format
itself.

Everything here uses only `struct`, which is available in both CPython and
MicroPython, specifically so the packing/unpacking code can look identical
on both sides of the link.
"""

import struct

# ---------------------------------------------------------------------------
# Wire-level constants
# ---------------------------------------------------------------------------

START_BYTE = 0xAA

MSG_CONTROL = 0x01     # Jetson -> Pico
MSG_TELEMETRY = 0x02   # Pico -> Jetson

# Operating modes (CONTROL.mode)
MODE_DISABLED = 0
MODE_MANUAL = 1
MODE_AUTONOMOUS = 2

MODE_NAMES = {MODE_DISABLED: "DISABLED", MODE_MANUAL: "MANUAL", MODE_AUTONOMOUS: "AUTONOMOUS"}

# Fault bitmask (TELEMETRY.fault_status)
FAULT_COMM_TIMEOUT = 0x01
FAULT_OVER_CURRENT = 0x02
FAULT_ESTOP_ACTIVE = 0x04
FAULT_ENCODER_FAULT = 0x08
FAULT_UNDERVOLTAGE = 0x10

FAULT_NAMES = [
    (FAULT_COMM_TIMEOUT, "COMM_TIMEOUT"),
    (FAULT_OVER_CURRENT, "OVER_CURRENT"),
    (FAULT_ESTOP_ACTIVE, "ESTOP_ACTIVE"),
    (FAULT_ENCODER_FAULT, "ENCODER_FAULT"),
    (FAULT_UNDERVOLTAGE, "UNDERVOLTAGE"),
]

CMD_AGE_UNKNOWN = 0xFFFF  # sentinel: "no valid CONTROL frame received yet"

# Payload struct formats. '<' = little-endian, no padding.
CONTROL_FMT = "<hhBB"      # drive_cmd, steer_cmd, mode, stop
TELEMETRY_FMT = "<iihHBH"  # enc_left, enc_right, steer_fb, current_ma, fault_status, cmd_age_ms

CONTROL_LEN = struct.calcsize(CONTROL_FMT)      # 6 bytes
TELEMETRY_LEN = struct.calcsize(TELEMETRY_FMT)  # 15 bytes


def fault_names(bitmask):
    """Human-readable list of fault flags set in a fault_status bitmask."""
    return [name for bit, name in FAULT_NAMES if bitmask & bit]


# ---------------------------------------------------------------------------
# CRC16-CCITT (XModem variant: poly 0x1021, init 0xFFFF, no reflection)
# Deliberately a plain bit-loop, not a lookup table: at 6-15 byte payloads
# and ~20 Hz, this costs microseconds even on the Pico's 133 MHz core, and
# a bit-loop is trivial to verify by hand and port to any platform.
# ---------------------------------------------------------------------------

def crc16_ccitt(data, crc=0xFFFF):
    for byte in data:
        crc ^= (byte << 8) & 0xFFFF
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc & 0xFFFF


# ---------------------------------------------------------------------------
# Frame encode
#
# Wire format (see PROTOCOL.md section 3 for the full diagram):
#   [START:1][MSG_ID:1][LEN:1][PAYLOAD:LEN][CRC16:2 little-endian]
#   CRC is computed over MSG_ID + LEN + PAYLOAD (NOT the START byte).
# ---------------------------------------------------------------------------

def encode_frame(msg_id, payload):
    if len(payload) > 255:
        raise ValueError("payload too long for 1-byte LEN field")
    body = bytes([msg_id, len(payload)]) + payload
    crc = crc16_ccitt(body)
    return bytes([START_BYTE]) + body + struct.pack("<H", crc)


def encode_control(drive_cmd, steer_cmd, mode, stop):
    payload = struct.pack(CONTROL_FMT, drive_cmd, steer_cmd, mode, 1 if stop else 0)
    return encode_frame(MSG_CONTROL, payload)


def encode_telemetry(enc_left, enc_right, steer_fb, current_ma, fault_status, cmd_age_ms):
    payload = struct.pack(
        TELEMETRY_FMT, enc_left, enc_right, steer_fb, current_ma, fault_status, cmd_age_ms
    )
    return encode_frame(MSG_TELEMETRY, payload)


def decode_control(payload):
    drive_cmd, steer_cmd, mode, stop = struct.unpack(CONTROL_FMT, payload)
    return {"drive_cmd": drive_cmd, "steer_cmd": steer_cmd, "mode": mode, "stop": bool(stop)}


def decode_telemetry(payload):
    enc_left, enc_right, steer_fb, current_ma, fault_status, cmd_age_ms = struct.unpack(
        TELEMETRY_FMT, payload
    )
    return {
        "enc_left": enc_left,
        "enc_right": enc_right,
        "steer_fb": steer_fb,
        "current_ma": current_ma,
        "fault_status": fault_status,
        "cmd_age_ms": cmd_age_ms,
    }


# ---------------------------------------------------------------------------
# Stream parser
#
# Robustness strategy (see PROTOCOL.md section 5):
#  - Frames are found by scanning for START_BYTE, then trusting LEN to know
#    exactly how many bytes to wait for -- no byte-stuffing/escaping needed.
#  - Every candidate frame is CRC-checked before being accepted. A CRC
#    mismatch (real corruption, OR a data byte that happened to equal
#    START_BYTE and caused a false-positive sync) is handled the same way:
#    drop just the leading START byte and resume scanning. This makes the
#    parser self-resynchronizing without needing a special "resync" mode.
#  - A stale partial frame (bytes arrived, but the rest never showed up)
#    is dropped after `inter_byte_timeout_ms` of silence, via check_timeout().
# ---------------------------------------------------------------------------

class FrameParser:
    def __init__(self, now_ms, inter_byte_timeout_ms=50):
        """
        now_ms: zero-arg callable returning a millisecond timestamp
                (time.monotonic() * 1000 on CPython, utime.ticks_ms() on
                MicroPython -- see pico_main.py for the ticks_diff-safe
                version needed there).
        """
        self.buf = bytearray()
        self.now_ms = now_ms
        self.timeout = inter_byte_timeout_ms
        self._last_byte_time = None
        self.stats = {"frames_ok": 0, "crc_errors": 0, "timeouts": 0}

    def feed(self, data):
        """Feed newly-received bytes in. Returns a list of (msg_id, payload) tuples
        for every complete, CRC-valid frame found."""
        if not data:
            return []
        self.buf.extend(data)
        self._last_byte_time = self.now_ms()
        return self._extract_frames()

    def check_timeout(self):
        """Call periodically (e.g. once per main loop iteration) even when no
        new bytes arrived. Clears a stuck partial frame after a silence gap,
        so a truncated transmission can't wedge the parser forever."""
        if self.buf and self._last_byte_time is not None:
            if (self.now_ms() - self._last_byte_time) > self.timeout:
                self.buf = bytearray()
                self._last_byte_time = None
                self.stats["timeouts"] += 1
                return True
        return False

    def _extract_frames(self):
        frames = []
        while True:
            while self.buf and self.buf[0] != START_BYTE:
                del self.buf[0]
            if len(self.buf) < 3:
                break
            msg_id = self.buf[1]
            length = self.buf[2]
            total = 3 + length + 2
            if len(self.buf) < total:
                break  # wait for more bytes
            body = bytes(self.buf[1:3 + length])
            crc_recv = struct.unpack("<H", bytes(self.buf[3 + length:total]))[0]
            if crc16_ccitt(body) == crc_recv:
                frames.append((msg_id, bytes(self.buf[3:3 + length])))
                del self.buf[0:total]
                self.stats["frames_ok"] += 1
            else:
                del self.buf[0]  # slide by one byte and keep scanning
                self.stats["crc_errors"] += 1
        return frames
