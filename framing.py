"""
framing.py -- Rover onboard-computer <-> embedded-controller link: byte-level
framing, CRC, and message (de)serialization.

This is the SINGLE shared implementation of the wire format described in
PROTOCOL.md. It runs unmodified on:

  * CPython on the Jetson / a dev laptop (jetson_test.py, mcu_sim.py, demo.py)
  * CircuitPython on the Feather RP2040 RFM9x (feather_main.py)

It deliberately imports only `struct`, which both runtimes provide, so there
is exactly ONE copy of the CRC, the struct formats, and the parser. An
earlier version of this project hand-duplicated all of that into the
firmware file; that is a wire-format-drift hazard with no upside, since
copying two files onto the board is no harder than copying one.

To deploy: copy this file onto CIRCUITPY alongside the firmware.
PROTOCOL.md remains the source of truth for the format itself.
"""

import struct

# ---------------------------------------------------------------------------
# Wire-level constants
# ---------------------------------------------------------------------------

START_BYTE = 0xAA

MSG_CONTROL = 0x01     # onboard computer -> embedded controller
MSG_TELEMETRY = 0x02   # embedded controller -> onboard computer

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
FAULT_FIRMWARE_FAULT = 0x20   # main loop raised; see feather_main.py safe_stop()

FAULT_NAMES = [
    (FAULT_COMM_TIMEOUT, "COMM_TIMEOUT"),
    (FAULT_OVER_CURRENT, "OVER_CURRENT"),
    (FAULT_ESTOP_ACTIVE, "ESTOP_ACTIVE"),
    (FAULT_ENCODER_FAULT, "ENCODER_FAULT"),
    (FAULT_UNDERVOLTAGE, "UNDERVOLTAGE"),
    (FAULT_FIRMWARE_FAULT, "FIRMWARE_FAULT"),
]

# Command age. 0xFFFF means "no valid CONTROL frame has EVER arrived";
# 0xFFFE is the saturation ceiling for a real-but-very-stale measurement.
# These are deliberately distinct so the onboard computer can tell
# "never talked to me" apart from "talked to me over 65 seconds ago".
CMD_AGE_UNKNOWN = 0xFFFF
CMD_AGE_MAX = 0xFFFE

# Payload struct formats. '<' = little-endian, no padding.
CONTROL_FMT = "<hhBB"      # drive_cmd, steer_cmd, mode, stop
TELEMETRY_FMT = "<iihhBH"  # enc_left, enc_right, steer_fb, current_ca,
                           # fault_status, cmd_age_ms

CONTROL_LEN = struct.calcsize(CONTROL_FMT)      # 6 bytes
TELEMETRY_LEN = struct.calcsize(TELEMETRY_FMT)  # 15 bytes

# Every payload length this protocol version considers legal, by message id.
# The parser uses this to reject an implausible LEN byte IMMEDIATELY rather
# than blocking until that many bytes arrive -- see FrameParser for why that
# matters a great deal.
VALID_PAYLOAD_LENS = {
    MSG_CONTROL: CONTROL_LEN,
    MSG_TELEMETRY: TELEMETRY_LEN,
}

# Headroom for message types not yet defined. An unknown MSG_ID with a LEN
# at or below this is still buffered and CRC-checked (so a future firmware
# can add messages without this parser rejecting them outright), but a LEN
# above it is treated as certain corruption.
MAX_PAYLOAD_LEN = 32
MAX_FRAME_LEN = 3 + MAX_PAYLOAD_LEN + 2

# Encoder counters are int32 on the wire and wrap. The receiver must treat
# them as relative and handle rollover when differencing.
INT32_MIN = -(2 ** 31)
INT32_MAX = 2 ** 31 - 1


def fault_names(bitmask):
    """Human-readable list of fault flags set in a fault_status bitmask."""
    return [name for bit, name in FAULT_NAMES if bitmask & bit]


def wrap_i32(value):
    """Wrap an unbounded accumulator into int32 range.

    Encoder tick counters grow without limit. Packing an out-of-range value
    raises struct.error, which on a microcontroller means the main loop dies
    with the motor outputs still energized -- so wrapping is a safety
    measure, not just tidiness. Counts are relative anyway.
    """
    return ((value - INT32_MIN) % (2 ** 32)) + INT32_MIN


def clamp_cmd_age_ms(age_ms):
    """Clamp a measured command age to the wire range, keeping CMD_AGE_UNKNOWN
    reserved for 'never received'."""
    if age_ms < 0:
        return 0
    return age_ms if age_ms < CMD_AGE_MAX else CMD_AGE_MAX


# ---------------------------------------------------------------------------
# CRC-16/CCITT-FALSE: poly 0x1021, init 0xFFFF, no reflection, no final XOR.
# (Also catalogued as CRC-16/IBM-3740.)
#
# NAMING, because this bites people: this is NOT CRC-16/XMODEM, which an
# earlier revision of this file and of PROTOCOL.md both mislabelled it as.
# XMODEM uses the same polynomial but initialises to 0x0000, which produces
# completely different output. Anyone implementing "CRC-16/XMODEM" from a
# standard library against this link would have every frame rejected.
#   CRC-16/CCITT-FALSE check value: 0x29B1   <-- this is us
#   CRC-16/XMODEM     check value: 0x31C3
# Init 0xFFFF is the better choice of the two here because it makes the CRC
# sensitive to leading zero bytes, which init 0x0000 is not.
#
# Deliberately a plain bit-loop, not a lookup table: at 6-15 byte payloads
# and ~20 Hz this costs microseconds even on the RP2040, and a bit-loop is
# trivial to verify by hand and port to any platform.
# Known-answer vector: crc16_ccitt(b"123456789") == 0x29B1 (enforced in tests/).
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
# Frame encode / decode
#
# Wire format (see PROTOCOL.md section 3 for the full diagram):
#   [START:1][MSG_ID:1][LEN:1][PAYLOAD:LEN][CRC16:2 little-endian]
#   CRC is computed over MSG_ID + LEN + PAYLOAD (NOT the START byte).
# ---------------------------------------------------------------------------

def encode_frame(msg_id, payload):
    if len(payload) > MAX_PAYLOAD_LEN:
        raise ValueError(
            "payload of %d bytes exceeds MAX_PAYLOAD_LEN (%d); raise the limit "
            "on BOTH sides of the link before sending it" % (len(payload), MAX_PAYLOAD_LEN)
        )
    body = bytes([msg_id, len(payload)]) + payload
    crc = crc16_ccitt(body)
    return bytes([START_BYTE]) + body + struct.pack("<H", crc)


def encode_control(drive_cmd, steer_cmd, mode, stop):
    payload = struct.pack(CONTROL_FMT, drive_cmd, steer_cmd, mode, 1 if stop else 0)
    return encode_frame(MSG_CONTROL, payload)


def encode_telemetry(enc_left, enc_right, steer_fb, current_ca, fault_status, cmd_age_ms):
    payload = struct.pack(
        TELEMETRY_FMT,
        wrap_i32(enc_left), wrap_i32(enc_right),
        steer_fb, current_ca, fault_status, cmd_age_ms,
    )
    return encode_frame(MSG_TELEMETRY, payload)


def decode_control(payload):
    drive_cmd, steer_cmd, mode, stop = struct.unpack(CONTROL_FMT, payload)
    return {"drive_cmd": drive_cmd, "steer_cmd": steer_cmd, "mode": mode, "stop": bool(stop)}


def decode_telemetry(payload):
    enc_left, enc_right, steer_fb, current_ca, fault_status, cmd_age_ms = struct.unpack(
        TELEMETRY_FMT, payload
    )
    return {
        "enc_left": enc_left,
        "enc_right": enc_right,
        "steer_fb": steer_fb,
        "current_ca": current_ca,
        "current_a": current_ca / 100.0,
        "fault_status": fault_status,
        "cmd_age_ms": cmd_age_ms,
    }


# ---------------------------------------------------------------------------
# Stream parser
#
# Robustness strategy (see PROTOCOL.md section 5):
#  - Frames are found by scanning for START_BYTE, then trusting LEN to know
#    exactly how many bytes to wait for -- no byte-stuffing/escaping needed.
#  - LEN and MSG_ID are SANITY-CHECKED BEFORE the parser agrees to wait for
#    that many bytes. This is the important one: without it, a single bit
#    flip turning a LEN byte into 0xFF makes the parser block for
#    3+255+2 = 260 bytes before it even gets to check the CRC. At 20 Hz with
#    11-byte CONTROL frames that is ~1.2 s of blackout -- four times the
#    300 ms watchdog -- so one corrupted byte would stop the rover. With the
#    check, the same corruption costs a single byte of resync.
#  - Every candidate frame is CRC-checked before being accepted. A CRC
#    mismatch (real corruption, OR a data byte that happened to equal
#    START_BYTE and caused a false-positive sync) is handled the same way:
#    drop just the leading START byte and resume scanning. This makes the
#    parser self-resynchronizing without needing a special "resync" mode.
#  - A stale partial frame (bytes arrived, but the rest never showed up) is
#    dropped after `inter_byte_timeout_ms` of silence, via check_timeout().
#  - The buffer is hard-capped, so a peer that never sends a valid frame
#    cannot grow it without bound on a memory-constrained board.
# ---------------------------------------------------------------------------

class FrameParser:
    def __init__(self, now_ms, inter_byte_timeout_ms=50, max_buffer=None):
        """
        now_ms: zero-arg callable returning a millisecond timestamp.
                CPython:        lambda: time.monotonic() * 1000
                CircuitPython:  lambda: time.monotonic_ns() // 1_000_000
                Both are monotonic and do not wrap within any realistic
                mission duration, so plain subtraction is safe here. (On
                MicroPython you would need utime.ticks_diff() instead --
                ticks_ms() wraps. CircuitPython avoids that whole class of
                bug, which is one reason to prefer it.)
        """
        self.buf = bytearray()
        self.now_ms = now_ms
        self.timeout = inter_byte_timeout_ms
        self.max_buffer = max_buffer if max_buffer is not None else MAX_FRAME_LEN * 4
        self._last_byte_time = None
        self.stats = {
            "frames_ok": 0,
            "crc_errors": 0,
            "timeouts": 0,
            "bad_headers": 0,     # LEN/MSG_ID rejected before buffering
            "overflows": 0,       # buffer cap hit
            "bytes_discarded": 0,
        }

    def feed(self, data):
        """Feed newly-received bytes in. Returns a list of (msg_id, payload)
        tuples for every complete, CRC-valid frame found."""
        if not data:
            return []
        self.buf.extend(data)
        self._last_byte_time = self.now_ms()
        frames = self._extract_frames()
        # Anything left over after extraction is either a genuine partial
        # frame or unrecognizable noise. Either way it must stay bounded.
        if len(self.buf) > self.max_buffer:
            self.stats["overflows"] += 1
            self.stats["bytes_discarded"] += len(self.buf) - MAX_FRAME_LEN
            del self.buf[:-MAX_FRAME_LEN]
        return frames

    def check_timeout(self):
        """Call periodically (e.g. once per main loop iteration) even when no
        new bytes arrived. Clears a stuck partial frame after a silence gap,
        so a truncated transmission can't wedge the parser forever."""
        if self.buf and self._last_byte_time is not None:
            if (self.now_ms() - self._last_byte_time) > self.timeout:
                self.stats["bytes_discarded"] += len(self.buf)
                self.buf = bytearray()
                self._last_byte_time = None
                self.stats["timeouts"] += 1
                return True
        return False

    def _plausible_header(self, msg_id, length):
        """Would a frame with this MSG_ID/LEN be worth waiting for?

        A known message id must carry exactly its defined payload length. An
        unknown id is tolerated up to MAX_PAYLOAD_LEN so a newer peer can
        introduce message types without this parser choking on them.
        """
        expected = VALID_PAYLOAD_LENS.get(msg_id)
        if expected is not None:
            return length == expected
        return length <= MAX_PAYLOAD_LEN

    def _extract_frames(self):
        frames = []
        while True:
            # Discard anything ahead of the next plausible start byte.
            if self.buf and self.buf[0] != START_BYTE:
                idx = self.buf.find(START_BYTE)
                if idx < 0:
                    self.stats["bytes_discarded"] += len(self.buf)
                    del self.buf[:]
                    break
                self.stats["bytes_discarded"] += idx
                del self.buf[:idx]

            if len(self.buf) < 3:
                break

            msg_id = self.buf[1]
            length = self.buf[2]

            if not self._plausible_header(msg_id, length):
                # Corrupt or false sync. Do NOT wait for `length` bytes.
                del self.buf[0]
                self.stats["bad_headers"] += 1
                self.stats["bytes_discarded"] += 1
                continue

            total = 3 + length + 2
            if len(self.buf) < total:
                break  # plausible header, wait for the rest

            body = bytes(self.buf[1:3 + length])
            crc_recv = struct.unpack("<H", bytes(self.buf[3 + length:total]))[0]
            if crc16_ccitt(body) == crc_recv:
                frames.append((msg_id, bytes(self.buf[3:3 + length])))
                del self.buf[0:total]
                self.stats["frames_ok"] += 1
            else:
                del self.buf[0]  # slide by one byte and keep scanning
                self.stats["crc_errors"] += 1
                self.stats["bytes_discarded"] += 1
        return frames
