"""
pico_main.py -- Rover embedded controller firmware (Raspberry Pi Pico / RP2040).

Copy this ONE file onto the Pico's filesystem as main.py (e.g. with Thonny
or `mpremote cp pico_main.py :main.py`) and it runs automatically on boot.
It is self-contained (only stdlib MicroPython modules) so nothing else
needs to be uploaded for this early version.

This implements the Pico side of the wire format in PROTOCOL.md, and is a
byte-for-byte match with framing.py (same START byte, same CRC16-CCITT,
same struct formats) -- they're just written separately because the Pico
can't `import` a file that only exists on the Jetson side of a serial link.

--- What this does, at a glance --------------------------------------------
  1. Reads bytes from UART as they arrive (non-blocking).
  2. Parses CONTROL frames (drive_cmd, steer_cmd, mode, stop).
  3. Runs a watchdog: if no valid CONTROL frame has arrived within
     WATCHDOG_TIMEOUT_MS, the rover is forced to a safe stopped state and
     a COMM_TIMEOUT fault is reported -- this is what "command age" is for.
  4. Sends TELEMETRY frames at a fixed rate (encoders, steering feedback,
     current, faults, command age).
  5. Motor/sensor I/O is stubbed behind clearly-marked functions so this
     runs and is testable *before* the final motor drivers/encoders are
     wired up (SIMULATE_SENSORS=True fakes plausible values so you can
     watch cmd_age_ms and the simulated encoders respond to real CONTROL
     frames sent from jetson_test.py --port ...).
-----------------------------------------------------------------------------
"""

import struct
import utime
from machine import UART, Pin

# ---------------------------------------------------------------------------
# Wire format constants -- MUST match framing.py exactly.
# ---------------------------------------------------------------------------

START_BYTE = 0xAA
MSG_CONTROL = 0x01
MSG_TELEMETRY = 0x02

MODE_DISABLED = 0
MODE_MANUAL = 1
MODE_AUTONOMOUS = 2

FAULT_COMM_TIMEOUT = 0x01
FAULT_OVER_CURRENT = 0x02
FAULT_ESTOP_ACTIVE = 0x04
FAULT_ENCODER_FAULT = 0x08
FAULT_UNDERVOLTAGE = 0x10

CMD_AGE_UNKNOWN = 0xFFFF

CONTROL_FMT = "<hhBB"
TELEMETRY_FMT = "<iihHBH"
CONTROL_LEN = struct.calcsize(CONTROL_FMT)
TELEMETRY_LEN = struct.calcsize(TELEMETRY_FMT)

# ---------------------------------------------------------------------------
# Configuration -- adjust these to match actual wiring once hardware is final.
# ---------------------------------------------------------------------------

UART_ID = 0
UART_TX_PIN = 0   # GP0, UART0 TX -- confirm against final pinout
UART_RX_PIN = 1   # GP1, UART0 RX
UART_BAUD = 115200

TELEMETRY_RATE_HZ = 20
TELEMETRY_PERIOD_MS = 1000 // TELEMETRY_RATE_HZ
WATCHDOG_TIMEOUT_MS = 300  # no valid CONTROL frame in this long -> fail-safe

# Flip to False once real encoders/current sensing/steering feedback are
# wired in -- see read_sensors() below for where to put the real code.
SIMULATE_SENSORS = True


# ---------------------------------------------------------------------------
# CRC16-CCITT (XModem: poly 0x1021, init 0xFFFF) -- identical to framing.py
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


def encode_frame(msg_id, payload):
    body = bytes([msg_id, len(payload)]) + payload
    crc = crc16_ccitt(body)
    return bytes([START_BYTE]) + body + struct.pack("<H", crc)


def decode_control(payload):
    drive_cmd, steer_cmd, mode, stop = struct.unpack(CONTROL_FMT, payload)
    return {"drive_cmd": drive_cmd, "steer_cmd": steer_cmd, "mode": mode, "stop": bool(stop)}


# ---------------------------------------------------------------------------
# Stream parser. Same resync strategy as framing.FrameParser: scan for
# START_BYTE, trust LEN, verify CRC, and on any mismatch drop one byte and
# keep scanning rather than discarding the whole buffer. See PROTOCOL.md
# section 5 for why this needs no byte-stuffing/escaping.
#
# Uses utime.ticks_ms()/ticks_diff() rather than plain subtraction because
# ticks_ms() wraps around -- ticks_diff() handles that wraparound correctly,
# plain subtraction eventually would not. This is a classic embedded gotcha.
# ---------------------------------------------------------------------------

class FrameParser:
    def __init__(self, inter_byte_timeout_ms=50):
        self.buf = bytearray()
        self.timeout = inter_byte_timeout_ms
        self._last_byte_time = None

    def feed(self, data):
        if not data:
            return []
        self.buf.extend(data)
        self._last_byte_time = utime.ticks_ms()
        return self._extract_frames()

    def check_timeout(self):
        if self.buf and self._last_byte_time is not None:
            if utime.ticks_diff(utime.ticks_ms(), self._last_byte_time) > self.timeout:
                self.buf = bytearray()
                self._last_byte_time = None

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
                break
            body = bytes(self.buf[1:3 + length])
            crc_recv = struct.unpack("<H", bytes(self.buf[3 + length:total]))[0]
            if crc16_ccitt(body) == crc_recv:
                frames.append((msg_id, bytes(self.buf[3:3 + length])))
                del self.buf[0:total]
            else:
                del self.buf[0]
        return frames


# ---------------------------------------------------------------------------
# Sensor / motor I/O -- placeholder implementations.
#
# These are the ONLY functions you should need to touch once real hardware
# (motor drivers, quadrature encoders, current-sense, steering pot) is
# available. Everything above this line is the protocol and shouldn't need
# to change for that.
# ---------------------------------------------------------------------------

_sim_enc_left = 0
_sim_enc_right = 0
_sim_steer_fb = 0


def read_sensors(last_control, effective_stop):
    """Return (enc_left, enc_right, steer_fb, current_ma, fault_bits).
    TODO once hardware is available: replace the SIMULATE_SENSORS branch
    with real reads, e.g.:
      - enc_left/right: counters incremented in a GPIO IRQ handler on each
        encoder pulse (attach with Pin(pin).irq(trigger=Pin.IRQ_RISING, ...))
      - steer_fb: ADC.read_u16() on the steering feedback potentiometer,
        scaled into the same -1000..1000 range as steer_cmd
      - current_ma: ADC.read_u16() on a current-sense amplifier output,
        scaled to milliamps per that amplifier's datasheet
      - fault_bits: OR in FAULT_OVER_CURRENT / FAULT_ENCODER_FAULT /
        FAULT_ESTOP_ACTIVE based on real readings and any E-stop input pin
    """
    global _sim_enc_left, _sim_enc_right, _sim_steer_fb

    if not SIMULATE_SENSORS:
        # --- real sensor reads go here ---
        return (0, 0, 0, 0, 0)

    drive = 0 if effective_stop else last_control["drive_cmd"]
    target_steer = 0 if effective_stop else last_control["steer_cmd"]

    _sim_enc_left += drive // 10
    _sim_enc_right += drive // 10
    if _sim_steer_fb < target_steer:
        _sim_steer_fb = min(target_steer, _sim_steer_fb + 25)
    elif _sim_steer_fb > target_steer:
        _sim_steer_fb = max(target_steer, _sim_steer_fb - 25)

    current_ma = 0 if effective_stop else min(20000, abs(drive) * 15)
    return (_sim_enc_left, _sim_enc_right, _sim_steer_fb, current_ma, 0)


def set_motor_outputs(drive_cmd, steer_cmd, effective_stop):
    """TODO once hardware is available: drive real motor controller outputs
    here (e.g. PWM duty cycle to an ESC or H-bridge for drive_cmd, and a
    servo/PWM position command for steer_cmd). When effective_stop is True,
    this MUST de-energize the drive motors regardless of drive_cmd -- that
    is the whole point of the watchdog/stop path."""
    pass


# ---------------------------------------------------------------------------
# Main loop
# ---------------------------------------------------------------------------

def main():
    uart = UART(UART_ID, baudrate=UART_BAUD, tx=Pin(UART_TX_PIN), rx=Pin(UART_RX_PIN))
    parser = FrameParser()

    try:
        led = Pin("LED", Pin.OUT)  # Pico W
    except (TypeError, ValueError):
        led = Pin(25, Pin.OUT)     # Pico (non-W): onboard LED is GP25

    last_control = {"drive_cmd": 0, "steer_cmd": 0, "mode": MODE_DISABLED, "stop": True}
    last_control_time = None
    next_telemetry_ms = utime.ticks_ms()

    while True:
        now = utime.ticks_ms()

        if uart.any():
            data = uart.read(uart.any())
            for msg_id, payload in parser.feed(data):
                if msg_id == MSG_CONTROL and len(payload) == CONTROL_LEN:
                    last_control = decode_control(payload)
                    last_control_time = now
        parser.check_timeout()

        if last_control_time is None:
            cmd_age_ms = CMD_AGE_UNKNOWN
        else:
            cmd_age_ms = min(CMD_AGE_UNKNOWN, utime.ticks_diff(now, last_control_time))

        watchdog_tripped = cmd_age_ms >= WATCHDOG_TIMEOUT_MS
        # Fail-safe: comm timeout, explicit stop flag, or DISABLED mode all
        # force the rover to a stopped state -- this is the safety-critical
        # line in the whole file.
        effective_stop = (
            watchdog_tripped or last_control["stop"] or last_control["mode"] == MODE_DISABLED
        )

        enc_left, enc_right, steer_fb, current_ma, sensor_faults = read_sensors(
            last_control, effective_stop
        )
        set_motor_outputs(last_control["drive_cmd"], last_control["steer_cmd"], effective_stop)

        if utime.ticks_diff(now, next_telemetry_ms) >= 0:
            fault_status = sensor_faults | (FAULT_COMM_TIMEOUT if watchdog_tripped else 0)
            payload = struct.pack(
                TELEMETRY_FMT, enc_left, enc_right, steer_fb, current_ma, fault_status, cmd_age_ms
            )
            uart.write(encode_frame(MSG_TELEMETRY, payload))
            led.toggle()
            next_telemetry_ms = utime.ticks_add(next_telemetry_ms, TELEMETRY_PERIOD_MS)

        utime.sleep_ms(2)


if __name__ == "__main__":
    main()
