"""
test_integration.py -- end-to-end Jetson <-> controller over the simulated bus.

The "controller" here is tools/rover_sim, which links the REAL
firmware/src/rover_controller.cpp. There is no Python reimplementation of the
safety logic to drift from the firmware, which is the mistake this project
already made once and fixed.

These tests involve a real subprocess and real time, so they use generous
margins (watchdog is 300 ms; cuts are 700 ms+) and assert on eventual state
rather than exact frame counts. A flaky safety test gets ignored, which is
worse than no test.
"""

import os
import sys
import time
import unittest

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(REPO, "host"))

import can_link  # noqa: E402
import rover_protocol as rp  # noqa: E402
from jetson_test import JetsonLink  # noqa: E402


class RoverFixture(unittest.TestCase):
    def setUp(self):
        try:
            self.link = can_link.SimLink()
        except RuntimeError as exc:
            self.skipTest(str(exc))
        self.jl = JetsonLink(self.link)

    def tearDown(self):
        self.link.close()

    def drive(self, seconds, drive=400, steer=100,
              mode=rp.MODE_MANUAL, stop=False, send=True):
        """Send CONTROL at 20 Hz (or stay silent) for `seconds`, polling."""
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            if send:
                self.jl.send_control(drive, steer, mode, stop)
            self.jl.poll()
            time.sleep(0.02)
        self.jl.poll()

    def faults(self):
        self.assertIsNotNone(self.jl.status, "no telemetry received at all")
        return rp.fault_names(self.jl.status["fault_status"])


def send_raw_control(fixture, drive, steer, mode, seconds):
    """Send a correctly sized CONTROL frame with arbitrary field values for
    `seconds`, bypassing any validation a well-behaved sender would apply."""
    import struct
    payload = struct.pack(rp.CONTROL_FMT, drive, steer, mode, 0, rp.INDICATOR_OFF)
    assert len(payload) == rp.CONTROL_DLC
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        fixture.link.send(rp.CAN_ID_CONTROL, payload)
        fixture.jl.poll()
        time.sleep(0.02)
    fixture.jl.poll()


class TestEndToEnd(RoverFixture):
    def test_normal_operation(self):
        self.drive(0.6)
        self.assertGreater(self.jl.motion_count, 5)
        self.assertGreater(self.jl.status_count, 5)
        self.assertEqual(self.faults(), [])
        self.assertGreater(self.jl.motion["enc_left"], 0, "rover never moved")
        self.assertGreater(self.jl.status["current_a"], 0)
        self.assertEqual(self.jl.bad_dlc_frames, 0)
        self.assertEqual(self.jl.unknown_frames, 0)

    def test_link_cut_trips_comm_timeout_then_recovers(self):
        self.drive(0.6)
        self.assertEqual(self.faults(), [])
        moving = self.jl.motion["enc_left"]

        self.drive(0.7, send=False)          # silence, well past the 300ms watchdog
        self.assertIn("COMM_TIMEOUT", self.faults())
        self.assertEqual(self.jl.status["current_ca"], 0,
                         "still drawing current after the watchdog tripped")
        frozen = self.jl.motion["enc_left"]
        self.assertGreater(frozen, moving)

        self.drive(0.4, send=False)          # stays stopped while silent
        self.assertEqual(self.jl.motion["enc_left"], frozen,
                         "kept moving while commands were stale")

        self.drive(0.5)                      # commands resume
        self.assertEqual(self.faults(), [], "fault did not clear on its own")
        self.assertGreater(self.jl.motion["enc_left"], frozen)

    def test_stop_flag_is_honoured_end_to_end(self):
        self.drive(0.5)
        running = self.jl.motion["enc_left"]
        self.drive(0.5, drive=1000, steer=500, stop=True)
        halted = self.jl.motion["enc_left"]
        self.assertEqual(self.jl.status["current_ca"], 0)
        self.drive(0.4, drive=1000, steer=500, stop=True)
        self.assertEqual(self.jl.motion["enc_left"], halted,
                         "moved while stop was asserted")
        # Unlike the in-process C++ tests, this one runs against a real
        # subprocess, so the previously-commanded drive is still in effect for
        # the iteration or two before the stop frame lands. The property that
        # matters is a BOUNDED coast followed by a complete freeze (asserted
        # above), not a single-cycle stop. With drive=1000 commanded alongside
        # the stop, ignoring the flag would advance this by many thousands.
        self.assertLess(halted - running, 500,
                        "coasted far too long after stop was asserted")

        self.drive(0.5)                      # releasing stop resumes motion,
        self.assertGreater(self.jl.motion["enc_left"], halted)   # no mode change

    def test_disabled_mode_never_moves(self):
        self.drive(0.6, drive=1000, steer=1000, mode=rp.MODE_DISABLED)
        self.assertEqual(self.jl.motion["enc_left"], 0)
        self.assertEqual(self.jl.status["current_ca"], 0)


class TestUndefinedMode(RoverFixture):
    """Issue #4, end to end against the real compiled controller.

    `mode` is a uint8 carrying 256 possible values where 3 are defined. Before
    the fix, any undefined value read as "not DISABLED" and permitted full
    throttle. These drive the actual firmware logic, not a model of it.
    """

    def send_raw_mode(self, mode, seconds, drive=1000, steer=500):
        """Put an arbitrary mode byte on the bus, the way a mismatched or
        faulty sender would.

        Packs with rp.CONTROL_FMT so the frame has the CORRECT length. A
        hand-written format here once fell a byte short after the indicator
        field was added, so these tests were passing on a wrong-DLC rejection
        rather than on the undefined mode they claim to test.
        """
        send_raw_control(self, drive, steer, mode, seconds)

    def test_undefined_mode_never_moves_the_rover(self):
        for mode in (3, 42, 255):
            with self.subTest(mode=mode):
                self.setUp()                       # fresh controller per mode
                try:
                    self.send_raw_mode(mode, 0.6)
                    self.assertEqual(self.jl.motion["enc_left"], 0,
                                     "rover moved in undefined mode %d" % mode)
                    self.assertEqual(self.jl.status["current_ca"], 0)
                finally:
                    self.link.close()
        self.setUp()                               # leave a live fixture for tearDown

    def test_undefined_mode_is_reported_as_a_protocol_error(self):
        """The stop must be explainable. A rover that halts while telemetry
        reads "no faults, link healthy" is its own hazard."""
        self.send_raw_mode(7, 0.5)
        self.assertIn("PROTOCOL_ERROR", self.faults())

    def test_undefined_mode_does_not_keep_the_watchdog_alive(self):
        self.drive(0.4)                            # healthy first
        self.send_raw_mode(7, 0.7)                 # then nothing but bad modes
        self.assertIn("COMM_TIMEOUT", self.faults(),
                      "undefined modes refreshed the command watchdog")

    def test_recovery_after_the_sender_is_fixed(self):
        self.send_raw_mode(7, 0.5)
        self.assertIn("PROTOCOL_ERROR", self.faults())
        self.drive(0.5)                            # valid commands resume
        self.assertEqual(self.faults(), [],
                         "PROTOCOL_ERROR latched instead of self-clearing")
        self.assertGreater(self.jl.motion["enc_left"], 0)

    def test_disabled_mode_is_still_accepted_as_a_valid_command(self):
        """DISABLED is KNOWN and VALID -- a legitimate command meaning "do not
        move". It must not be rejected as a protocol error; conflating the two
        is what caused the bug."""
        self.drive(0.5, drive=1000, mode=rp.MODE_DISABLED)
        self.assertEqual(self.jl.motion["enc_left"], 0)
        self.assertNotIn("PROTOCOL_ERROR", self.faults())
        self.assertNotIn("COMM_TIMEOUT", self.faults())


class TestCommandRange(RoverFixture):
    """drive/steer outside CMD_MIN..CMD_MAX, end to end against the real
    compiled controller. Rejected, never clamped to full scale."""

    def test_out_of_range_drive_never_moves_the_rover(self):
        send_raw_control(self, rp.CMD_MAX + 1, 0, rp.MODE_MANUAL, 0.6)
        self.assertEqual(self.jl.motion["enc_left"], 0,
                         "rover moved on an out-of-range drive command")
        self.assertEqual(self.jl.status["current_ca"], 0)

    def test_out_of_range_command_is_reported_as_a_protocol_error(self):
        send_raw_control(self, 0, rp.CMD_MIN - 1, rp.MODE_MANUAL, 0.5)
        self.assertIn("PROTOCOL_ERROR", self.faults())

    def test_out_of_range_command_does_not_keep_the_watchdog_alive(self):
        self.drive(0.4)                            # healthy first
        send_raw_control(self, 30000, 0, rp.MODE_MANUAL, 0.7)
        self.assertIn("COMM_TIMEOUT", self.faults(),
                      "out-of-range commands refreshed the command watchdog")

    def test_full_scale_is_still_accepted(self):
        self.drive(0.5, drive=rp.CMD_MAX, steer=rp.CMD_MIN)
        self.assertEqual(self.faults(), [])
        self.assertGreater(self.jl.motion["enc_left"], 0)


class TestBusHygiene(RoverFixture):
    """A shared CAN bus carries motor-controller and payload traffic too."""

    def test_foreign_can_id_does_not_command_the_rover(self):
        self.drive(0.4)
        moving = self.jl.motion["enc_left"]

        # Someone else's frame, carrying what looks like a full-throttle
        # command, on an id that is not ours. It must be ignored entirely --
        # including for watchdog purposes.
        end = time.monotonic() + 0.7
        while time.monotonic() < end:
            self.link.send(0x321, rp.encode_control(1000, 0, rp.MODE_MANUAL, False))
            self.jl.poll()
            time.sleep(0.02)
        self.jl.poll()

        self.assertIn("COMM_TIMEOUT", self.faults(),
                      "a foreign id kept the command watchdog alive")
        self.assertEqual(self.jl.status["current_ca"], 0)
        self.assertGreater(self.jl.motion["enc_left"], moving - 1)

    def test_wrong_dlc_on_our_id_does_not_command_the_rover(self):
        """A peer on a mismatched protocol version must not keep us alive."""
        self.drive(0.4)
        end = time.monotonic() + 0.7
        while time.monotonic() < end:
            self.link.send(rp.CAN_ID_CONTROL, b"\x00\x00\x00\x00")   # 4 bytes, not CONTROL_DLC
            self.jl.poll()
            time.sleep(0.02)
        self.jl.poll()
        self.assertIn("COMM_TIMEOUT", self.faults(),
                      "a wrong-DLC frame refreshed the command watchdog")


if __name__ == "__main__":
    unittest.main()
