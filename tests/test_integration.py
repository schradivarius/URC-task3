"""
tests/test_integration.py -- End-to-end CONTROL/TELEMETRY exchange across the
mock link.

Deliberately single-threaded and manually pumped rather than sleep-based: a
threaded timing test would flake in CI, and flaky tests get ignored, which is
worse than no test. mock_link's fault injection supplies the line noise.
"""

import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import controller  # noqa: E402
import framing  # noqa: E402
import mock_link  # noqa: E402
from jetson_test import JetsonLink  # noqa: E402


class Clock:
    def __init__(self):
        self.t = 0

    def __call__(self):
        return self.t

    def advance(self, ms):
        self.t += ms


class Rig:
    """Host and controller wired through the mock link, stepped by hand."""

    def __init__(self, **pair_kw):
        self.host_side, self.mcu_side = mock_link.make_pair(**pair_kw)
        self.clock = Clock()
        self.host = JetsonLink(self.host_side)
        self.mcu = controller.RoverController(
            now_ms=self.clock, watchdog_timeout_ms=300, telemetry_period_ms=50)
        self.enc = 0

    def step(self, ms=50, send=None):
        """Advance time by ms. If `send` is a dict, the host transmits that
        CONTROL frame first. Then the controller services the link and replies
        with telemetry, and the host reads whatever came back."""
        self.clock.advance(ms)
        if send is not None:
            self.host.send_control(**send)
        self.mcu.ingest(self.mcu_side.read(self.mcu_side.in_waiting))
        if not self.mcu.effective_stop():
            self.enc += 100
        if self.mcu.telemetry_due():
            self.mcu_side.write(self.mcu.telemetry_frame(
                enc_left=self.enc, enc_right=self.enc, steer_fb=0,
                current_ca=150 if not self.mcu.effective_stop() else 0))
        self.host.poll()


MANUAL = {"drive_cmd": 400, "steer_cmd": 100,
          "mode": framing.MODE_MANUAL, "stop": False}


class TestEndToEnd(unittest.TestCase):
    def test_normal_operation(self):
        rig = Rig()
        for _ in range(10):
            rig.step(send=MANUAL)
        self.assertEqual(rig.host.telemetry_count, 10)
        self.assertEqual(rig.host.unknown_frames, 0)
        latest = rig.host.latest_telemetry
        self.assertEqual(framing.fault_names(latest["fault_status"]), [])
        self.assertGreater(latest["enc_left"], 0)
        self.assertEqual(rig.host.parser.stats["crc_errors"], 0)

    def test_link_cut_trips_comm_timeout_then_recovers(self):
        rig = Rig()
        for _ in range(5):
            rig.step(send=MANUAL)
        self.assertEqual(framing.fault_names(rig.host.latest_telemetry["fault_status"]), [])
        moving_enc = rig.host.latest_telemetry["enc_left"]

        # Host goes quiet for 600 ms, well past the 300 ms watchdog.
        for _ in range(12):
            rig.step(send=None)
        stalled = rig.host.latest_telemetry
        self.assertIn("COMM_TIMEOUT", framing.fault_names(stalled["fault_status"]))
        self.assertGreater(stalled["cmd_age_ms"], 300)
        self.assertEqual(stalled["current_ca"], 0, "still drawing current after timeout")

        # Encoders must have stopped advancing during the cut.
        frozen = stalled["enc_left"]
        for _ in range(4):
            rig.step(send=None)
        self.assertEqual(rig.host.latest_telemetry["enc_left"], frozen,
                         "kept moving while commands were stale")
        self.assertGreater(frozen, moving_enc)

        # Commands resume: the fault must clear on its own.
        for _ in range(3):
            rig.step(send=MANUAL)
        recovered = rig.host.latest_telemetry
        self.assertEqual(framing.fault_names(recovered["fault_status"]), [],
                         "fault did not clear after the link was restored")
        self.assertLess(recovered["cmd_age_ms"], 300)

    def test_link_survives_injected_line_noise(self):
        """With 15% of writes corrupted, some frames are lost -- but the link
        must keep delivering and must never hand up a bad frame as good."""
        rig = Rig(corrupt_rate=0.15)
        for _ in range(120):
            rig.step(send=MANUAL)

        stats = rig.host.parser.stats
        self.assertGreater(rig.host.telemetry_count, 60,
                           "link collapsed under noise it should tolerate")
        self.assertGreater(stats["crc_errors"] + stats["bad_headers"], 0,
                           "no corruption was injected; the test proves nothing")
        self.assertEqual(rig.host.unknown_frames, 0,
                         "a corrupt frame was accepted as a valid message")

    def test_stop_command_is_honoured_end_to_end(self):
        rig = Rig()
        for _ in range(4):
            rig.step(send=MANUAL)
        running = rig.host.latest_telemetry["enc_left"]

        stop_cmd = dict(MANUAL, stop=True)
        for _ in range(4):
            rig.step(send=stop_cmd)
        self.assertEqual(rig.host.latest_telemetry["current_ca"], 0)
        halted = rig.host.latest_telemetry["enc_left"]

        for _ in range(4):
            rig.step(send=stop_cmd)
        self.assertEqual(rig.host.latest_telemetry["enc_left"], halted,
                         "moved while stop was asserted")
        # stop takes effect on the very cycle it arrives, so motion ceases
        # immediately rather than coasting for a frame.
        self.assertEqual(halted, running, "coasted after stop was asserted")

        # And releasing stop must resume motion without a mode change.
        for _ in range(3):
            rig.step(send=MANUAL)
        self.assertGreater(rig.host.latest_telemetry["enc_left"], halted)

    def test_disabled_mode_is_honoured_end_to_end(self):
        rig = Rig()
        disabled = dict(MANUAL, mode=framing.MODE_DISABLED)
        for _ in range(6):
            rig.step(send=disabled)
        self.assertEqual(rig.host.latest_telemetry["current_ca"], 0)
        self.assertEqual(rig.host.latest_telemetry["enc_left"], 0)


if __name__ == "__main__":
    unittest.main()
