"""
feather_main.py -- Rover embedded-controller firmware.
Target: Adafruit Feather RP2040 RFM9x, running CircuitPython.

DEPLOY
  Copy TWO files onto the CIRCUITPY drive:
      feather_main.py  ->  code.py
      framing.py       ->  framing.py     (shared protocol module, unmodified)
  And, for the USB data channel, also copy:
      boot.py          ->  boot.py        (then power-cycle the board once)

  The protocol layer is imported from framing.py rather than duplicated here,
  so the firmware and the host tools are provably running the same wire
  format. tests/test_protocol.py enforces that.

THE JETSON LINK: WHY usb_cdc AND NOT /dev/ttyACM0 DIRECTLY
  On CircuitPython the default USB serial device (/dev/ttyACM0) is the REPL
  console. Frames written there get fed to the Python interpreter, not to
  this program. boot.py calls usb_cdc.enable(console=True, data=True), which
  exposes a SECOND CDC endpoint -- typically /dev/ttyACM1 -- carrying nothing
  but our frames. That is the channel this firmware reads.

  If no data channel is available (boot.py missing, or you would rather run
  over real wires), this falls back to hardware UART0 on the Feather's
  TX/RX pads, which is what you would use with a USB-TTL adapter.

SAFETY STRUCTURE
  1. Hardware watchdog. If this loop ever stops feeding it, the RP2040
     resets, which drives every pin to high-Z and de-energizes the drives.
     Without this, a firmware exception leaves the PWM registers holding
     their last value -- i.e. a runaway rover with a dead controller.
  2. Exception guard. Any exception escaping the loop body calls safe_stop()
     BEFORE anything else, then re-raises so the watchdog completes the reset.
     It cannot report the fault itself -- the board is about to reset -- so
     instead the NEXT boot inspects microcontroller.cpu.reset_reason and
     raises FAULT_FIRMWARE_FAULT in its telemetry (see detect_boot_fault).
     Otherwise a watchdog reset is invisible to the onboard computer except
     as a brief gap in telemetry.
  3. Command watchdog. No valid CONTROL frame within WATCHDOG_TIMEOUT_MS
     forces a stop and raises FAULT_COMM_TIMEOUT.
  4. Encoder counters are wrapped into int32 range before packing. An
     unbounded counter would eventually raise struct.error mid-loop, which
     is failure mode (2) with extra steps.

I/O BUDGET WARNING -- READ BEFORE WIRING
  This board is NOT a drop-in for a Pico's pin count. GPIO16-23 are consumed
  by the RFM9x module and are not broken out at all. After reserving the
  Jetson UART (GP0/GP1), I2C (GP2/GP3), radio SPI (GP8/GP14/GP15), the
  NeoPixel (GP4) and the boot button (GP7), roughly 13 GPIO remain, with
  only 4 ADC channels (A0-A3 = GP26-29).

  A 6-wheel rover with 4-corner steering needs ~29 pins on direct GPIO.
  It does not fit. Plan on pushing I/O off-board: motor controllers over one
  serial link, servos via PCA9685 (I2C), current sense via ADS1115 (I2C),
  encoders via RP2040 PIO or dedicated counter chips. The stub functions at
  the bottom of this file are where that lands.
"""

import time

import board
import busio
import digitalio
import microcontroller

import controller
import framing

# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------

UART_BAUD = 115200          # only used on the hardware-UART fallback path

TELEMETRY_RATE_HZ = 20
TELEMETRY_PERIOD_MS = 1000 // TELEMETRY_RATE_HZ

WATCHDOG_TIMEOUT_MS = 300   # no valid CONTROL frame in this long -> fail-safe
HW_WATCHDOG_TIMEOUT_S = 1.0 # loop must feed this often or the board resets

OVER_CURRENT_LIMIT_CA = 4000  # 40.00 A, placeholder -- set from real hardware

# Flip to False once real encoders / current sensing / steering feedback are
# wired in. See read_sensors() for where the real code goes.
SIMULATE_SENSORS = True


def now_ms():
    """Monotonic milliseconds. time.monotonic_ns() is an integer and does not
    wrap within any realistic mission, so plain subtraction is safe -- unlike
    MicroPython's ticks_ms(), which wraps and needs ticks_diff()."""
    return time.monotonic_ns() // 1_000_000


# ---------------------------------------------------------------------------
# Link setup
# ---------------------------------------------------------------------------

def open_link():
    """Return (link, description). Prefers the USB CDC data channel; falls
    back to hardware UART0 on the TX/RX pads."""
    try:
        import usb_cdc
        if usb_cdc.data is not None:
            usb_cdc.data.timeout = 0
            return usb_cdc.data, "usb_cdc.data (second CDC endpoint)"
    except ImportError:
        pass
    uart = busio.UART(board.TX, board.RX, baudrate=UART_BAUD, timeout=0)
    return uart, "busio.UART on board.TX/board.RX (GP0/GP1)"


def link_read(link):
    """Non-blocking read of whatever is waiting. Both usb_cdc.Serial and
    busio.UART expose in_waiting, so one path covers both."""
    waiting = getattr(link, "in_waiting", 0)
    if not waiting:
        return b""
    data = link.read(waiting)
    return data if data else b""


# ---------------------------------------------------------------------------
# Sensor / motor I/O -- placeholder implementations.
#
# These are the ONLY functions that should need touching once real hardware
# (motor drivers, quadrature encoders, current sense, steering feedback) is
# available. Everything above is protocol and should not change for that.
# ---------------------------------------------------------------------------

_sim_enc_left = 0
_sim_enc_right = 0
_sim_steer_fb = 0


def read_sensors(last_control, effective_stop):
    """Return (enc_left, enc_right, steer_fb, current_ca, fault_bits).

    TODO once hardware exists, replace the SIMULATE_SENSORS branch:
      - enc_left/right: counters from RP2040 PIO quadrature decoders, or an
        I2C/SPI counter chip. Direct GPIO interrupts will not scale to six
        encoders on this board's pin budget.
      - steer_fb: ADS1115 channel on the steering feedback pot, scaled into
        the same -1000..1000 range as steer_cmd.
      - current_ca: ADS1115 channel on a current-sense amplifier, scaled to
        CENTIAMPS (1 cA = 10 mA). Signed, so regen/reverse current reads
        negative.
      - fault_bits: OR in FAULT_OVER_CURRENT / FAULT_ENCODER_FAULT /
        FAULT_ESTOP_ACTIVE / FAULT_UNDERVOLTAGE from real readings.
    """
    global _sim_enc_left, _sim_enc_right, _sim_steer_fb

    if not SIMULATE_SENSORS:
        # --- real sensor reads go here ---
        return (0, 0, 0, 0, 0)

    drive = 0 if effective_stop else last_control["drive_cmd"]
    target_steer = 0 if effective_stop else last_control["steer_cmd"]

    # framing.wrap_i32 keeps these inside int32 range forever. Without it the
    # accumulators overflow after ~12 hours of driving and struct.pack raises
    # mid-loop, killing the firmware with the motors still energized.
    _sim_enc_left = framing.wrap_i32(_sim_enc_left + drive // 10)
    _sim_enc_right = framing.wrap_i32(_sim_enc_right + drive // 10)

    if _sim_steer_fb < target_steer:
        _sim_steer_fb = min(target_steer, _sim_steer_fb + 25)
    elif _sim_steer_fb > target_steer:
        _sim_steer_fb = max(target_steer, _sim_steer_fb - 25)

    # Signed centiamps: 15 cA per unit of drive is an arbitrary placeholder.
    current_ca = 0 if effective_stop else int(abs(drive) * 1.5)
    fault_bits = framing.FAULT_OVER_CURRENT if current_ca > OVER_CURRENT_LIMIT_CA else 0
    return (_sim_enc_left, _sim_enc_right, _sim_steer_fb, current_ca, fault_bits)


def set_motor_outputs(drive_cmd, steer_cmd, effective_stop):
    """TODO once hardware exists: drive the real motor/servo outputs here.

    When effective_stop is True this MUST de-energize the drive motors
    regardless of drive_cmd -- that is the entire point of the watchdog and
    stop paths. The caller already zeroes the commands before calling, so a
    future implementation cannot accidentally honour a stale command, but
    keep honouring the flag anyway.
    """
    pass


def detect_boot_fault():
    """Fault bits to report for the whole of this run, based on why the board
    last reset.

    A watchdog reset means the previous run of this firmware stopped feeding
    the watchdog -- a hang or an unhandled exception. The onboard computer
    needs to know that happened: telemetry resuming normally after an
    unexplained gap otherwise looks like a transient link problem rather than
    a controller that restarted mid-drive.

    The bit latches for the session. It clears only on a clean power-on,
    because "this controller rebooted unexpectedly" stays true for as long as
    that boot lasts.
    """
    try:
        if microcontroller.cpu.reset_reason == microcontroller.ResetReason.WATCHDOG:
            return framing.FAULT_FIRMWARE_FAULT
    except (AttributeError, NotImplementedError):
        pass  # port does not expose a reset reason; nothing to report
    return 0


def safe_stop():
    """Unconditional stop. Called from the exception guard, so it must not
    assume any particular state and must not raise."""
    try:
        set_motor_outputs(0, 0, True)
    except Exception:
        pass


# ---------------------------------------------------------------------------
# Main loop
#
# The command/safety state machine lives in controller.py, shared verbatim
# with mcu_sim.py. This function is only the hardware wiring around it.
# ---------------------------------------------------------------------------

def run(link, led, watchdog, boot_faults=0):
    ctl = controller.RoverController(
        now_ms=now_ms,
        watchdog_timeout_ms=WATCHDOG_TIMEOUT_MS,
        telemetry_period_ms=TELEMETRY_PERIOD_MS,
    )

    while True:
        if watchdog is not None:
            watchdog.feed()

        ctl.ingest(link_read(link))

        now = now_ms()
        effective_stop = ctl.effective_stop(now)
        drive, steer = ctl.commanded_outputs(now)

        enc_left, enc_right, steer_fb, current_ca, sensor_faults = read_sensors(
            ctl.last_control, effective_stop
        )
        set_motor_outputs(drive, steer, effective_stop)

        if ctl.telemetry_due(now):
            link.write(ctl.telemetry_frame(
                enc_left=enc_left,
                enc_right=enc_right,
                steer_fb=steer_fb,
                current_ca=current_ca,
                sensor_faults=sensor_faults,
                extra_faults=boot_faults,
                now=now,
            ))
            led.value = not led.value

        time.sleep(0.002)


def main():
    link, link_desc = open_link()

    led = digitalio.DigitalInOut(board.LED)  # GP13 on the Feather RP2040 RFM9x
    led.direction = digitalio.Direction.OUTPUT
    led.value = False

    # Hardware watchdog. Wrapped because not every build exposes it; if it is
    # missing we still boot, but the exception guard below is then the only
    # backstop, so the startup banner says so out loud.
    watchdog = None
    try:
        from watchdog import WatchDogMode
        watchdog = microcontroller.watchdog
        watchdog.timeout = HW_WATCHDOG_TIMEOUT_S
        watchdog.mode = WatchDogMode.RESET
    except (ImportError, AttributeError, NotImplementedError, ValueError):
        watchdog = None

    boot_faults = detect_boot_fault()

    print("rover firmware up: link = %s, hw watchdog = %s%s"
          % (link_desc, "on" if watchdog else "UNAVAILABLE",
             ", RECOVERED FROM WATCHDOG RESET" if boot_faults else ""))

    try:
        run(link, led, watchdog, boot_faults)
    except Exception:
        # Stop the drives FIRST, before logging or anything else that could
        # itself fail. Then let it propagate: with the hardware watchdog armed
        # the board resets shortly after, which is the cleanest recovery from
        # an unknown firmware state. Without this guard, an exception here
        # leaves the PWM registers holding their last value.
        safe_stop()
        led.value = False
        raise


if __name__ == "__main__":
    main()
