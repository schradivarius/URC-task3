"""
tests/test_firmware.py -- Exercises feather_main.py, the file that actually
gets flashed, under stubbed CircuitPython hardware.

Run:  python3 -m unittest discover -s tests -v

The firmware is the most expensive place to have a bug and the hardest to
inspect once it is on the board, so it should not be the one file with no
test coverage. tests/fake_hardware.py supplies just enough of board/busio/
digitalio/microcontroller/usb_cdc to let CPython import and drive it.
"""

import os
import sys
import unittest

# Two entries, both needed:
#   the repo root  -> framing / controller / feather_main
#   this directory -> fake_hardware, a sibling test helper
# `unittest discover -s tests` happens to add this directory itself, but
# `python3 -m unittest tests.test_firmware` does not, so relying on that
# would make the module importable one way and not the other.
_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(_HERE))
sys.path.insert(0, _HERE)

import fake_hardware  # noqa: E402  (sibling module, see sys.path note above)

FAKE_WATCHDOG = fake_hardware.install()

import controller  # noqa: E402
import feather_main  # noqa: E402  -- import AFTER the fakes are installed
import framing  # noqa: E402


class LoopBreak(Exception):
    """Sentinel used to exit the firmware's infinite loop deterministically."""


class ScriptedLink:
    """A fake serial endpoint that hands the firmware a fixed set of inbound
    bytes, records everything written, and raises LoopBreak after a set number
    of polls so run() terminates."""

    def __init__(self, inbound=b"", max_polls=12):
        self._rx = bytearray(inbound)
        self.written = bytearray()
        self.polls = 0
        self.max_polls = max_polls

    @property
    def in_waiting(self):
        self.polls += 1
        if self.polls > self.max_polls:
            raise LoopBreak()
        return len(self._rx)

    def read(self, n):
        out = bytes(self._rx[:n])
        del self._rx[:n]
        return out

    def write(self, data):
        self.written.extend(data)
        return len(data)

    def telemetry(self):
        """Decode every TELEMETRY frame the firmware emitted."""
        p = framing.FrameParser(now_ms=lambda: 0)
        return [framing.decode_telemetry(payload)
                for msg_id, payload in p.feed(bytes(self.written))
                if msg_id == framing.MSG_TELEMETRY]


def fresh_led():
    import digitalio
    import board
    led = digitalio.DigitalInOut(board.LED)
    led.direction = digitalio.Direction.OUTPUT
    return led


class TestFirmwareLoop(unittest.TestCase):
    def setUp(self):
        # The firmware's simulated plant is module-level state; reset it so
        # tests do not leak encoder counts into each other.
        feather_main._sim_enc_left = 0
        feather_main._sim_enc_right = 0
        feather_main._sim_steer_fb = 0

    def run_firmware(self, inbound=b"", max_polls=12, watchdog=None):
        link = ScriptedLink(inbound, max_polls)
        led = fresh_led()
        with self.assertRaises(LoopBreak):
            feather_main.run(link, led, watchdog)
        return link, led

    def test_emits_telemetry(self):
        link, _ = self.run_firmware(max_polls=30)
        frames = link.telemetry()
        self.assertGreater(len(frames), 0, "firmware emitted no telemetry")

    def test_boots_into_a_safe_state_with_no_traffic(self):
        """With nothing on the link, the firmware must report COMM_TIMEOUT and
        an unknown command age -- never a quiet, movable idle."""
        link, _ = self.run_firmware(max_polls=30)
        first = link.telemetry()[0]
        self.assertEqual(first["cmd_age_ms"], framing.CMD_AGE_UNKNOWN)
        self.assertTrue(first["fault_status"] & framing.FAULT_COMM_TIMEOUT)
        self.assertEqual(first["current_ca"], 0)
        self.assertEqual(first["enc_left"], 0, "encoders moved while stopped")

    def test_acts_on_a_valid_control_frame(self):
        control = framing.encode_control(600, 0, framing.MODE_MANUAL, stop=False)
        link, _ = self.run_firmware(inbound=control, max_polls=30)
        frames = link.telemetry()
        self.assertTrue(any(f["cmd_age_ms"] != framing.CMD_AGE_UNKNOWN for f in frames),
                        "never registered the CONTROL frame")
        self.assertTrue(any(f["current_ca"] > 0 for f in frames),
                        "drive command produced no current draw")
        self.assertTrue(any(f["enc_left"] != 0 for f in frames),
                        "drive command did not advance the encoders")

    def test_ignores_a_corrupt_control_frame(self):
        bad = bytearray(framing.encode_control(600, 0, framing.MODE_MANUAL, False))
        bad[4] ^= 0xFF
        link, _ = self.run_firmware(inbound=bytes(bad), max_polls=30)
        for frame in link.telemetry():
            self.assertEqual(frame["cmd_age_ms"], framing.CMD_AGE_UNKNOWN,
                             "a CRC-invalid frame was treated as a command")
            self.assertTrue(frame["fault_status"] & framing.FAULT_COMM_TIMEOUT)

    def test_stop_flag_keeps_the_rover_stopped(self):
        control = framing.encode_control(1000, 500, framing.MODE_MANUAL, stop=True)
        link, _ = self.run_firmware(inbound=control, max_polls=30)
        for frame in link.telemetry():
            self.assertEqual(frame["current_ca"], 0, "drew current under stop")
            self.assertEqual(frame["enc_left"], 0, "moved under stop")

    def test_feeds_the_hardware_watchdog_every_iteration(self):
        wd = fake_hardware.FakeWatchdog()
        self.run_firmware(max_polls=20, watchdog=wd)
        self.assertGreaterEqual(wd.feeds, 15,
                                "loop is not feeding the watchdog; the board "
                                "would reset mid-operation")

    def test_runs_without_a_watchdog(self):
        """A build with no watchdog support must still boot, not crash."""
        link, _ = self.run_firmware(max_polls=15, watchdog=None)
        self.assertGreater(len(link.telemetry()), 0)

    def test_toggles_the_status_led(self):
        _, led = self.run_firmware(max_polls=40)
        self.assertGreater(led.toggles, 1, "status LED never toggled")


class TestFirmwareSensorStubs(unittest.TestCase):
    def setUp(self):
        feather_main._sim_enc_left = 0
        feather_main._sim_enc_right = 0

    def test_encoder_accumulator_wraps_instead_of_overflowing(self):
        """REGRESSION. An unbounded accumulator overflows int32 after ~12 h and
        raises inside struct.pack, killing the loop with the motors live."""
        feather_main._sim_enc_left = framing.INT32_MAX - 5
        feather_main._sim_enc_right = framing.INT32_MAX - 5
        control = {"drive_cmd": 1000, "steer_cmd": 0,
                   "mode": framing.MODE_MANUAL, "stop": False}
        for _ in range(10):
            enc_left, enc_right, _, _, _ = feather_main.read_sensors(control, False)
            self.assertGreaterEqual(enc_left, framing.INT32_MIN)
            self.assertLessEqual(enc_left, framing.INT32_MAX)
            # and the result must always be packable
            framing.encode_telemetry(enc_left, enc_right, 0, 0, 0, 0)

    def test_stopped_state_draws_no_current_and_does_not_move(self):
        control = {"drive_cmd": 1000, "steer_cmd": 500,
                   "mode": framing.MODE_MANUAL, "stop": False}
        enc_left, _, steer_fb, current_ca, _ = feather_main.read_sensors(control, True)
        self.assertEqual(current_ca, 0)
        self.assertEqual(enc_left, 0)
        self.assertEqual(steer_fb, 0)

    def test_safe_stop_never_raises(self):
        """safe_stop runs from the exception handler, so it must be
        unconditionally safe to call."""
        original = feather_main.set_motor_outputs
        try:
            def explode(*args, **kwargs):
                raise RuntimeError("motor driver is on fire")
            feather_main.set_motor_outputs = explode
            feather_main.safe_stop()   # must swallow it
        finally:
            feather_main.set_motor_outputs = original


class TestFirmwareStartup(unittest.TestCase):
    def test_arms_the_hardware_watchdog_in_reset_mode(self):
        calls = []
        original_run = feather_main.run
        try:
            feather_main.run = lambda *a, **kw: calls.append(a)
            feather_main.main()
        finally:
            feather_main.run = original_run
        self.assertEqual(FAKE_WATCHDOG.timeout, feather_main.HW_WATCHDOG_TIMEOUT_S)
        self.assertEqual(FAKE_WATCHDOG.mode, "RESET")
        self.assertEqual(len(calls), 1)

    def test_exception_guard_stops_the_motors_before_propagating(self):
        """Without this guard an exception leaves the PWM registers holding
        their last value -- a runaway rover with a dead controller."""
        stops = []
        original_run = feather_main.run
        original_set = feather_main.set_motor_outputs
        try:
            def boom(*a, **kw):
                raise RuntimeError("simulated firmware fault")
            feather_main.run = boom
            feather_main.set_motor_outputs = lambda d, s, stop: stops.append((d, s, stop))
            with self.assertRaises(RuntimeError):
                feather_main.main()
        finally:
            feather_main.run = original_run
            feather_main.set_motor_outputs = original_set
        self.assertEqual(stops, [(0, 0, True)],
                         "motors were not commanded to stop on a firmware fault")

    def test_prefers_the_usb_data_channel_when_present(self):
        """/dev/ttyACM0 is the REPL console; frames must go over the second CDC
        endpoint that boot.py enables."""
        import usb_cdc
        sentinel = ScriptedLink()
        usb_cdc.data = sentinel
        try:
            link, desc = feather_main.open_link()
        finally:
            usb_cdc.data = None
        self.assertIs(link, sentinel)
        self.assertIn("usb_cdc", desc)

    def test_falls_back_to_hardware_uart(self):
        import usb_cdc
        usb_cdc.data = None
        link, desc = feather_main.open_link()
        self.assertIsInstance(link, fake_hardware.FakeUART)
        self.assertIn("UART", desc)
        self.assertEqual(link.baudrate, 115200)

    def test_uses_the_feather_led_pin_not_the_pico_one(self):
        """Feather RP2040 RFM9x: onboard red LED is GP13. The Pico's was GP25."""
        import board
        self.assertEqual(board.LED.name, "GP13")
        led = fresh_led()
        self.assertIs(led.pin, board.LED)

    def test_firmware_timeouts_agree_with_the_shared_defaults(self):
        self.assertEqual(feather_main.WATCHDOG_TIMEOUT_MS,
                         controller.DEFAULT_WATCHDOG_TIMEOUT_MS)
        self.assertEqual(feather_main.TELEMETRY_PERIOD_MS,
                         controller.DEFAULT_TELEMETRY_PERIOD_MS)


if __name__ == "__main__":
    unittest.main()


class TestBootFaultReporting(unittest.TestCase):
    """A watchdog reset must not be invisible to the onboard computer.

    The exception guard cannot report the fault itself -- the board resets
    immediately after it runs -- so the next boot inspects the reset reason
    and raises FAULT_FIRMWARE_FAULT. Without this, telemetry resuming after an
    unexplained gap looks like a transient link problem rather than a
    controller that restarted mid-drive.
    """

    def setUp(self):
        feather_main._sim_enc_left = 0
        feather_main._sim_enc_right = 0
        self._cpu = getattr(sys.modules["microcontroller"], "cpu", None)

    def tearDown(self):
        import microcontroller
        if self._cpu is not None:
            microcontroller.cpu = self._cpu
        elif hasattr(microcontroller, "cpu"):
            del microcontroller.cpu

    def set_reset_reason(self, reason):
        import microcontroller
        import types
        if reason is None:
            if hasattr(microcontroller, "cpu"):
                del microcontroller.cpu
        else:
            microcontroller.cpu = types.SimpleNamespace(reset_reason=reason)

    def test_clean_power_on_reports_no_firmware_fault(self):
        self.set_reset_reason("POWER_ON")
        self.assertEqual(feather_main.detect_boot_fault(), 0)

    def test_watchdog_reset_is_detected(self):
        self.set_reset_reason("WATCHDOG")
        self.assertEqual(feather_main.detect_boot_fault(),
                         framing.FAULT_FIRMWARE_FAULT)

    def test_missing_reset_reason_is_tolerated(self):
        """A port that does not expose a reset reason must not crash the boot."""
        self.set_reset_reason(None)
        self.assertEqual(feather_main.detect_boot_fault(), 0)

    def test_firmware_fault_appears_in_every_telemetry_frame(self):
        """The bit latches for the session: the reboot stays true all run."""
        link = ScriptedLink(b"", max_polls=30)
        with self.assertRaises(LoopBreak):
            feather_main.run(link, fresh_led(), None,
                             boot_faults=framing.FAULT_FIRMWARE_FAULT)
        frames = link.telemetry()
        self.assertGreater(len(frames), 0)
        for frame in frames:
            self.assertIn("FIRMWARE_FAULT",
                          framing.fault_names(frame["fault_status"]))

    def test_firmware_fault_coexists_with_other_faults(self):
        """It must not mask the comm timeout that is also genuinely present."""
        link = ScriptedLink(b"", max_polls=30)
        with self.assertRaises(LoopBreak):
            feather_main.run(link, fresh_led(), None,
                             boot_faults=framing.FAULT_FIRMWARE_FAULT)
        names = framing.fault_names(link.telemetry()[0]["fault_status"])
        self.assertIn("FIRMWARE_FAULT", names)
        self.assertIn("COMM_TIMEOUT", names)

    def test_clean_boot_reports_no_firmware_fault_in_telemetry(self):
        link = ScriptedLink(b"", max_polls=30)
        with self.assertRaises(LoopBreak):
            feather_main.run(link, fresh_led(), None, boot_faults=0)
        for frame in link.telemetry():
            self.assertNotIn("FIRMWARE_FAULT",
                             framing.fault_names(frame["fault_status"]))
