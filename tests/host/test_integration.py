"""
test_integration.py -- end-to-end Jetson <-> controller over the simulated bus.

The "controller" here is tools/rover_sim, which links the REAL
firmware/src/rover_controller.cpp. There is no Python reimplementation of the
safety logic to drift from the firmware, which is the mistake this project
already made once and fixed.

These tests involve a real subprocess and real time, so they use generous
margins (the watchdog is 300 ms; cuts are 700 ms+) and assert on eventual
state rather than exact frame counts. A flaky safety test gets ignored, which
is worse than no test.
"""

import os
import struct
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

    def drive(self, seconds, send=True, **kw):
        """Send CONTROL at 20 Hz (or stay silent) for `seconds`, polling."""
        kw.setdefault("drive_cmd", 400)
        kw.setdefault("steer_cmd", 100)
        kw.setdefault("mode", rp.MODE_MANUAL)
        kw.setdefault("c2_lost", False)
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            if send:
                self.jl.send_control(**kw)
            self.jl.poll()
            time.sleep(0.02)
        self.jl.poll()

    def send_raw(self, frame, seconds):
        """Put arbitrary bytes on the CONTROL id, the way a faulty or
        mismatched sender would."""
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            self.link.send(rp.CAN_ID_CONTROL, frame)
            self.jl.poll()
            time.sleep(0.02)
        self.jl.poll()

    def snap(self):
        self.assertIsNotNone(self.jl.snapshot,
                             "no complete telemetry snapshot was assembled")
        return self.jl.snapshot

    def faults(self):
        return rp.fault_names(self.snap()["fault_status"])


class TestSnapshotAssembly(RoverFixture):
    """The four telemetry frames must reassemble into coherent snapshots."""

    def test_all_four_frames_arrive_and_assemble(self):
        self.drive(0.6)
        self.assertGreater(self.jl.tele.complete_count, 5)
        for can_id in rp.TELEM_IDS:
            self.assertGreater(self.jl.tele.frames_by_id[can_id], 5,
                               "no frames on id 0x%03x" % can_id)
        self.assertEqual(self.jl.tele.decode_errors, {})
        self.assertEqual(self.jl.unknown_frames, 0)

    def test_snapshots_are_not_torn(self):
        # All four frames of a cycle share one sequence number. Over a clean
        # link every snapshot should be coherent.
        self.drive(1.0)
        self.assertEqual(self.jl.tele.torn_count, 0,
                         "snapshots were torn across cycles on a clean link")

    def test_the_snapshot_sequence_number_advances(self):
        self.drive(0.4)
        first = self.snap()["seq"]
        self.drive(0.4)
        self.assertNotEqual(self.snap()["seq"], first)

    def test_a_snapshot_carries_every_required_field(self):
        # The task enumerates these; this is the end-to-end check that each one
        # actually reaches the Jetson rather than merely being defined.
        self.drive(0.6)
        s = self.snap()
        for field in ("seq", "enc_left", "enc_right", "steer_fb", "current_ca",
                      "voltage_cv", "mode", "fault_status", "cmd_age_ms",
                      "jetson_link", "c2_link", "controller_health",
                      "indicator_state"):
            self.assertIn(field, s, "telemetry is missing %r" % field)


class TestEndToEnd(RoverFixture):
    def test_normal_operation(self):
        self.drive(0.6, indicator_request=rp.INDICATOR_TELEOP)
        s = self.snap()
        self.assertEqual(self.faults(), [])
        self.assertGreater(s["enc_left"], 0, "rover never moved")
        self.assertGreater(s["current_a"], 0)
        self.assertGreater(s["voltage_v"], 20.0)
        self.assertEqual(s["mode"], rp.MODE_MANUAL)
        self.assertEqual(s["jetson_link"], rp.LINK_OK)
        self.assertEqual(s["indicator_state"], rp.INDICATOR_TELEOP)

    def test_link_cut_trips_comm_timeout_then_recovers(self):
        self.drive(0.6)
        self.assertEqual(self.faults(), [])
        moving = self.snap()["enc_left"]

        self.drive(0.7, send=False)
        self.assertIn("COMM_TIMEOUT", self.faults())
        self.assertEqual(self.snap()["current_ca"], 0)
        self.assertEqual(self.snap()["jetson_link"], rp.LINK_LOST)
        # A faulted rover must not display a cheerful indicator.
        self.assertEqual(self.snap()["indicator_state"], rp.INDICATOR_FAULT)
        frozen = self.snap()["enc_left"]
        self.assertGreater(frozen, moving)

        self.drive(0.4, send=False)
        self.assertEqual(self.snap()["enc_left"], frozen,
                         "kept moving while commands were stale")

        self.drive(0.6)
        self.assertEqual(self.faults(), [], "fault did not clear on its own")
        self.assertGreater(self.snap()["enc_left"], frozen)

    def test_stop_flag_is_honoured(self):
        self.drive(0.5)
        running = self.snap()["enc_left"]
        self.drive(0.5, drive_cmd=1000, steer_cmd=500, stop=True)
        halted = self.snap()["enc_left"]
        self.assertEqual(self.snap()["current_ca"], 0)
        self.drive(0.4, drive_cmd=1000, steer_cmd=500, stop=True)
        self.assertEqual(self.snap()["enc_left"], halted,
                         "moved while stop was asserted")
        # Real subprocess, real latency: the previous command is still in
        # effect for an iteration or two. Bounded coast then a full freeze is
        # the property that matters, not a single-cycle stop.
        self.assertLess(halted - running, 500, "coasted far too long")
        self.drive(0.5)
        self.assertGreater(self.snap()["enc_left"], halted)

    def test_disabled_mode_never_moves(self):
        self.drive(0.6, drive_cmd=1000, steer_cmd=1000, mode=rp.MODE_DISABLED)
        self.assertEqual(self.snap()["enc_left"], 0)
        self.assertEqual(self.snap()["current_ca"], 0)
        self.assertEqual(self.snap()["mode"], rp.MODE_DISABLED)


class TestNewCommandFields(RoverFixture):
    def test_autonomy_abort_stops_an_autonomous_rover(self):
        self.drive(0.5, drive_cmd=800, mode=rp.MODE_AUTONOMOUS)
        moving = self.snap()["enc_left"]
        self.assertGreater(moving, 0)
        self.drive(0.5, drive_cmd=800, mode=rp.MODE_AUTONOMOUS,
                   autonomy_abort=True)
        halted = self.snap()["enc_left"]
        self.drive(0.4, drive_cmd=800, mode=rp.MODE_AUTONOMOUS,
                   autonomy_abort=True)
        self.assertEqual(self.snap()["enc_left"], halted,
                         "kept driving autonomously after an abort")
        self.assertEqual(self.snap()["current_ca"], 0)

    def test_autonomy_abort_does_not_stop_a_teleoperated_rover(self):
        # In MANUAL the operator is already driving; there is no autonomous
        # task to abort, and stopping would surprise them mid-manoeuvre.
        self.drive(0.6, drive_cmd=600, mode=rp.MODE_MANUAL, autonomy_abort=True)
        self.assertGreater(self.snap()["enc_left"], 0)
        self.assertGreater(self.snap()["current_ca"], 0)

    def test_the_indicator_reports_what_the_rover_is_actually_doing(self):
        # URC scores this light on the rover's real state, so the controller's
        # own mode overrides the Jetson's request. Comparing request against
        # state is how an operator sees whether it was honoured.
        for requested in (rp.INDICATOR_TELEOP, rp.INDICATOR_AUTONOMOUS,
                          rp.INDICATOR_ARRIVED):
            self.drive(0.4, mode=rp.MODE_MANUAL, indicator_request=requested)
            self.assertEqual(self.snap()["indicator_state"], rp.INDICATOR_TELEOP,
                             "a teleoperated rover showed %d" % requested)

        self.drive(0.4, mode=rp.MODE_AUTONOMOUS,
                   indicator_request=rp.INDICATOR_TELEOP)
        self.assertEqual(self.snap()["indicator_state"], rp.INDICATOR_AUTONOMOUS)

        # ARRIVED is the one request the controller cannot derive itself.
        self.drive(0.4, mode=rp.MODE_AUTONOMOUS,
                   indicator_request=rp.INDICATOR_ARRIVED)
        self.assertEqual(self.snap()["indicator_state"], rp.INDICATOR_ARRIVED)

        self.drive(0.4, mode=rp.MODE_DISABLED,
                   indicator_request=rp.INDICATOR_AUTONOMOUS)
        self.assertEqual(self.snap()["indicator_state"], rp.INDICATOR_OFF)

    def test_mode_is_echoed_back(self):
        # Without the echo a dropped or misread mode change is invisible.
        for mode in (rp.MODE_MANUAL, rp.MODE_AUTONOMOUS, rp.MODE_DISABLED):
            self.drive(0.4, mode=mode)
            self.assertEqual(self.snap()["mode"], mode)


class TestLinkSeparation(RoverFixture):
    """C2 loss and Jetson loss must never be treated as equivalent."""

    def test_c2_link_is_reported_not_reported_before_any_frame(self):
        # Honest "nobody told us" rather than a misleading OK. The controller
        # has no radio, so this is the only truthful answer until the Jetson
        # forwards its verdict.
        self.drive(0.3, send=False)
        snap = self.snap()
        self.assertEqual(snap["c2_link"], rp.LINK_NOT_REPORTED)

    def test_the_forwarded_c2_verdict_reaches_telemetry(self):
        self.drive(0.6, c2_lost=False)
        self.assertEqual(self.snap()["c2_link"], rp.LINK_OK)
        self.drive(0.6, mode=rp.MODE_AUTONOMOUS, c2_lost=True)
        self.assertEqual(self.snap()["c2_link"], rp.LINK_LOST)
        self.assertTrue(self.snap()["fault_status"] & rp.FAULT_C2_LINK_LOST)

    def test_a_healthy_jetson_link_does_not_imply_a_healthy_c2_link(self):
        self.drive(0.6, mode=rp.MODE_AUTONOMOUS, c2_lost=True)
        snap = self.snap()
        self.assertEqual(snap["jetson_link"], rp.LINK_OK)
        self.assertEqual(snap["c2_link"], rp.LINK_LOST,
                         "C2 was reported OK purely because the Jetson link was")

    def test_c2_loss_stops_teleop_but_not_autonomy(self):
        # The asymmetry the autonomy course requires: line of sight to the
        # base station drops while onboard autonomy keeps running.
        self.drive(0.6, drive_cmd=800, mode=rp.MODE_AUTONOMOUS, c2_lost=True)
        self.assertNotEqual(self.snap()["current_a"], 0.0,
                            "an autonomous rover was stopped by C2 loss alone")

        self.drive(0.6, drive_cmd=800, mode=rp.MODE_MANUAL, c2_lost=True)
        self.assertEqual(self.snap()["current_a"], 0.0,
                         "a teleoperated rover kept driving with C2 lost")

    def test_a_stale_c2_bit_is_reported_as_not_reported(self):
        # Once the Jetson goes silent the last c2_lost value is stale. A clear
        # bit from before the silence does not mean C2 is fine now.
        self.drive(0.4, mode=rp.MODE_AUTONOMOUS, c2_lost=False)
        self.assertEqual(self.snap()["c2_link"], rp.LINK_OK)
        self.drive(0.6, send=False)              # past the 300 ms watchdog
        snap = self.snap()
        self.assertTrue(snap["fault_status"] & rp.FAULT_COMM_TIMEOUT)
        self.assertEqual(snap["c2_link"], rp.LINK_NOT_REPORTED)
        self.assertFalse(snap["fault_status"] & rp.FAULT_C2_LINK_LOST,
                         "a stale C2 verdict was reported as a live fault")

    def test_the_two_link_fields_are_independent_on_the_wire(self):
        for jl in (rp.LINK_OK, rp.LINK_DEGRADED, rp.LINK_LOST):
            for c2 in (rp.LINK_OK, rp.LINK_DEGRADED, rp.LINK_LOST,
                       rp.LINK_NOT_REPORTED):
                frame = rp.encode_telemetry_state(
                    rp.MODE_MANUAL, jl, c2, rp.CTRL_HEALTH_OK,
                    rp.INDICATOR_TELEOP, seq=1)
                _, d = rp.decode_telemetry_state(frame)
                self.assertEqual(d["jetson_link"], jl)
                self.assertEqual(d["c2_link"], c2)


class TestBusHygiene(RoverFixture):
    def test_foreign_can_id_does_not_command_the_rover(self):
        self.drive(0.4)
        end = time.monotonic() + 0.7
        while time.monotonic() < end:
            self.link.send(0x321, rp.encode_control(1000, 0, rp.MODE_MANUAL, False))
            self.jl.poll()
            time.sleep(0.02)
        self.jl.poll()
        self.assertIn("COMM_TIMEOUT", self.faults(),
                      "a foreign id kept the command watchdog alive")
        self.assertEqual(self.snap()["current_ca"], 0)

    def test_a_corrupted_frame_is_rejected_and_reported(self):
        # Corrupted AFTER the CRC is computed -- exactly the software-path
        # failure CAN's own CRC cannot see.
        self.drive(0.4)
        good = rp.encode_control(900, 0, rp.MODE_MANUAL, False, seq=50)
        bad = bytearray(good)
        bad[0] ^= 0xFF
        self.send_raw(bytes(bad), 0.7)
        self.assertIn("CRC_ERROR", self.faults())
        self.assertIn("COMM_TIMEOUT", self.faults(),
                      "a CRC-invalid frame refreshed the command watchdog")
        self.assertEqual(self.snap()["current_ca"], 0)

    def test_undefined_mode_is_rejected_and_does_not_move_the_rover(self):
        payload = struct.pack(rp.CONTROL_FMT, 1000, 500, 7, 0)
        body = payload + bytes((1,))
        frame = body + bytes((rp.frame_crc8(rp.CAN_ID_CONTROL, body),))
        self.send_raw(frame, 0.7)
        self.assertEqual(self.snap()["enc_left"], 0,
                         "rover moved in an undefined mode")
        self.assertIn("COMM_TIMEOUT", self.faults())

    def test_a_sequence_gap_is_detected_and_reported(self):
        self.drive(0.4)
        for seq in (10, 60):          # a 50-frame jump
            self.link.send(rp.CAN_ID_CONTROL,
                           rp.encode_control(400, 0, rp.MODE_MANUAL, False, seq=seq))
            time.sleep(0.08)
            self.jl.poll()
        self.assertIn("SEQ_GAP", self.faults())
        self.assertEqual(self.snap()["jetson_link"], rp.LINK_DEGRADED,
                         "a sequence gap did not degrade the reported link")


if __name__ == "__main__":
    unittest.main()
