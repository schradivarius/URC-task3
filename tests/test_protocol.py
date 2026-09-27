"""
tests/test_protocol.py -- Wire-format, parser and safety-logic tests.

Run:  python3 -m unittest discover -s tests -v
      (or `pytest tests/` if you prefer; these are plain unittest cases)

Stdlib unittest on purpose: CI needs no install step, and a firmware team
should be able to run the suite on any machine with Python.
"""

import os
import struct
import sys
import time
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import controller  # noqa: E402
import framing  # noqa: E402


class FakeClock:
    """Manually advanced millisecond clock, so watchdog behaviour is tested
    deterministically instead of with sleeps."""

    def __init__(self, start=0):
        self.t = start

    def __call__(self):
        return self.t

    def advance(self, ms):
        self.t += ms


def parser(clock=None, **kw):
    clock = clock or FakeClock()
    return framing.FrameParser(now_ms=clock, **kw), clock


# ---------------------------------------------------------------------------
# CRC
# ---------------------------------------------------------------------------

class TestCRC(unittest.TestCase):
    def test_known_answer_vector(self):
        """CRC-16/CCITT-FALSE of b"123456789" is 0x29B1.

        This test exists because the protocol was originally documented as
        "CRC-16/XModem", which is a DIFFERENT variant (same polynomial, but
        init 0x0000 instead of 0xFFFF, check value 0x31C3). Anyone writing a
        second implementation from that description -- a base station, a
        bench tool -- would have had every frame rejected. Pinning the check
        value here means the spec and the code cannot drift apart silently.
        """
        self.assertEqual(framing.crc16_ccitt(b"123456789"), 0x29B1)

    def test_is_not_xmodem(self):
        """Guard against someone "helpfully" changing init to 0x0000."""
        self.assertNotEqual(framing.crc16_ccitt(b"123456789"), 0x31C3)

    def test_empty_input_is_init_value(self):
        self.assertEqual(framing.crc16_ccitt(b""), 0xFFFF)

    def test_detects_single_bit_flip(self):
        base = framing.crc16_ccitt(b"rover")
        for bit in range(8):
            mutated = bytes([ord("r") ^ (1 << bit)]) + b"over"
            if mutated == b"rover":
                continue
            self.assertNotEqual(framing.crc16_ccitt(mutated), base)


# ---------------------------------------------------------------------------
# Round-trip
# ---------------------------------------------------------------------------

class TestRoundTrip(unittest.TestCase):
    def test_control_round_trip(self):
        for drive, steer, mode, stop in [
            (0, 0, framing.MODE_DISABLED, True),
            (1000, -1000, framing.MODE_AUTONOMOUS, False),
            (-1000, 1000, framing.MODE_MANUAL, True),
            (-32768, 32767, framing.MODE_MANUAL, False),  # int16 extremes
        ]:
            frame = framing.encode_control(drive, steer, mode, stop)
            p, _ = parser()
            got = p.feed(frame)
            self.assertEqual(len(got), 1, "frame not accepted")
            msg_id, payload = got[0]
            self.assertEqual(msg_id, framing.MSG_CONTROL)
            self.assertEqual(framing.decode_control(payload), {
                "drive_cmd": drive, "steer_cmd": steer, "mode": mode, "stop": stop})

    def test_telemetry_round_trip(self):
        frame = framing.encode_telemetry(
            enc_left=-123456, enc_right=123456, steer_fb=-1000,
            current_ca=-2500, fault_status=framing.FAULT_OVER_CURRENT,
            cmd_age_ms=42)
        p, _ = parser()
        got = p.feed(frame)
        self.assertEqual(len(got), 1)
        tm = framing.decode_telemetry(got[0][1])
        self.assertEqual(tm["enc_left"], -123456)
        self.assertEqual(tm["steer_fb"], -1000)
        self.assertEqual(tm["current_ca"], -2500)
        self.assertAlmostEqual(tm["current_a"], -25.0)
        self.assertEqual(framing.fault_names(tm["fault_status"]), ["OVER_CURRENT"])

    def test_current_is_signed_for_regen(self):
        """Current must be signed: a drive motor braking pushes current back."""
        frame = framing.encode_telemetry(0, 0, 0, -10000, 0, 0)
        tm = framing.decode_telemetry(parser()[0].feed(frame)[0][1])
        self.assertEqual(tm["current_a"], -100.0)

    def test_declared_lengths_match_formats(self):
        self.assertEqual(framing.CONTROL_LEN, struct.calcsize(framing.CONTROL_FMT))
        self.assertEqual(framing.TELEMETRY_LEN, struct.calcsize(framing.TELEMETRY_FMT))
        self.assertEqual(framing.VALID_PAYLOAD_LENS[framing.MSG_CONTROL],
                         framing.CONTROL_LEN)
        self.assertEqual(framing.VALID_PAYLOAD_LENS[framing.MSG_TELEMETRY],
                         framing.TELEMETRY_LEN)

    def test_frames_split_across_reads_reassemble(self):
        """A real serial read can land anywhere, including mid-frame."""
        frame = framing.encode_control(250, -250, framing.MODE_MANUAL, False)
        for split in range(1, len(frame)):
            p, _ = parser()
            self.assertEqual(p.feed(frame[:split]), [], "accepted a partial frame")
            self.assertEqual(len(p.feed(frame[split:])), 1,
                             "failed to reassemble at split %d" % split)


# ---------------------------------------------------------------------------
# Parser robustness
# ---------------------------------------------------------------------------

class TestParserRobustness(unittest.TestCase):
    def test_corrupted_payload_rejected_and_resyncs(self):
        bad = bytearray(framing.encode_control(500, -200, framing.MODE_MANUAL, False))
        bad[4] ^= 0xFF  # flip a payload byte; CRC no longer matches
        good = framing.encode_telemetry(42, 41, -190, 120, 0, 10)

        p, _ = parser()
        got = p.feed(bytes(bad) + good)
        self.assertEqual(len(got), 1)
        self.assertEqual(got[0][0], framing.MSG_TELEMETRY)
        self.assertGreaterEqual(p.stats["crc_errors"], 1)

    def test_corrupted_len_does_not_stall_the_link(self):
        """REGRESSION. A parser that trusts LEN blindly blocks for 3+255+2=260
        bytes before it can even check the CRC. At 20 Hz with 11-byte CONTROL
        frames that is ~1.2 s of blackout -- four times the 300 ms watchdog --
        so a single corrupted byte would stop the rover. One byte of resync is
        the only acceptable cost."""
        control = framing.encode_control(100, 0, framing.MODE_MANUAL, False)
        bad = bytearray(control)
        bad[2] = 0xFF  # LEN 6 -> 255

        p, _ = parser()
        got = p.feed(bytes(bad) + control * 5)
        self.assertEqual(len(got), 5, "valid frames were stalled behind a bad LEN")
        self.assertEqual(p.stats["bad_headers"], 1)

    def test_wrong_len_for_known_msg_id_rejected(self):
        """A CONTROL frame claiming any length but 6 is corrupt by definition."""
        for bogus_len in (0, 5, 7, 15, 32):
            p, _ = parser()
            p.feed(bytes([framing.START_BYTE, framing.MSG_CONTROL, bogus_len]))
            self.assertEqual(p.stats["bad_headers"], 1,
                             "LEN=%d accepted for CONTROL" % bogus_len)

    def test_unknown_msg_id_tolerated_for_forward_compatibility(self):
        """A future firmware may add message types. A plausible unknown frame
        should be parsed and handed up (for the caller to ignore), not treated
        as corruption -- otherwise adding a message breaks old peers."""
        future = framing.encode_frame(0x7E, b"\x01\x02\x03\x04")
        p, _ = parser()
        got = p.feed(future)
        self.assertEqual(len(got), 1)
        self.assertEqual(got[0][0], 0x7E)
        self.assertEqual(p.stats["bad_headers"], 0)

    def test_unknown_msg_id_with_absurd_len_rejected(self):
        p, _ = parser()
        p.feed(bytes([framing.START_BYTE, 0x7E, framing.MAX_PAYLOAD_LEN + 1]))
        self.assertEqual(p.stats["bad_headers"], 1)

    def test_start_byte_inside_payload_is_harmless(self):
        """0xAA is not escaped, so it appears in real payloads. drive_cmd
        0xAAAA and steer 0x00AA both embed it."""
        frame = framing.encode_control(-21846, 170, framing.MODE_MANUAL, False)
        self.assertIn(framing.START_BYTE, frame[3:])  # precondition
        p, _ = parser()
        got = p.feed(frame)
        self.assertEqual(len(got), 1)
        self.assertEqual(framing.decode_control(got[0][1])["drive_cmd"], -21846)

    def test_leading_garbage_discarded(self):
        frame = framing.encode_telemetry(1, 2, 3, 4, 0, 5)
        p, _ = parser()
        got = p.feed(b"\x00\x01\x02hello world" + frame)
        self.assertEqual(len(got), 1)
        self.assertGreater(p.stats["bytes_discarded"], 0)

    def test_inter_byte_timeout_clears_partial_frame(self):
        clock = FakeClock()
        p, _ = parser(clock, inter_byte_timeout_ms=50)
        frame = framing.encode_telemetry(1, 2, 3, 4, 0, 5)
        p.feed(frame[:6])                      # truncated transmission
        self.assertFalse(p.check_timeout())    # not yet stale
        clock.advance(51)
        self.assertTrue(p.check_timeout())     # cleared
        self.assertEqual(p.stats["timeouts"], 1)
        self.assertEqual(len(p.buf), 0)
        # and a following whole frame parses normally
        self.assertEqual(len(p.feed(frame)), 1)

    def test_buffer_is_bounded_under_junk_flood(self):
        """A peer that never sends a valid frame must not grow the buffer
        without bound on a memory-constrained board."""
        p, _ = parser()
        for _ in range(200):
            p.feed(bytes([framing.START_BYTE]) * 64)
        self.assertLessEqual(len(p.buf), p.max_buffer)

    def test_back_to_back_frames_all_parsed(self):
        stream = b"".join(
            framing.encode_control(i, -i, framing.MODE_MANUAL, False) for i in range(20))
        p, _ = parser()
        self.assertEqual(len(p.feed(stream)), 20)
        self.assertEqual(p.stats["frames_ok"], 20)


# ---------------------------------------------------------------------------
# Overflow / sentinel handling
# ---------------------------------------------------------------------------

class TestRangeHandling(unittest.TestCase):
    def test_encoder_overflow_does_not_raise(self):
        """REGRESSION. Unbounded encoder counters used to reach int32 range
        after ~12 h of driving, at which point struct.pack raised inside the
        main loop. On the board that kills the firmware with the motor outputs
        still energized -- a runaway rover. Wrapping is a safety measure."""
        for value in (2 ** 31, 2 ** 31 + 5, -(2 ** 31) - 1, 2 ** 40, -(2 ** 40)):
            frame = framing.encode_telemetry(value, value, 0, 0, 0, 0)
            tm = framing.decode_telemetry(parser()[0].feed(frame)[0][1])
            self.assertEqual(tm["enc_left"], framing.wrap_i32(value))

    def test_wrap_i32_is_identity_in_range(self):
        for value in (0, 1, -1, framing.INT32_MAX, framing.INT32_MIN, 123456):
            self.assertEqual(framing.wrap_i32(value), value)

    def test_wrap_i32_wraps_at_the_boundary(self):
        self.assertEqual(framing.wrap_i32(framing.INT32_MAX + 1), framing.INT32_MIN)
        self.assertEqual(framing.wrap_i32(framing.INT32_MIN - 1), framing.INT32_MAX)

    def test_cmd_age_sentinel_distinct_from_saturation(self):
        """"never received" and "very stale" must be distinguishable, or the
        onboard computer cannot tell a controller that has never heard from it
        apart from one it stopped talking to a minute ago."""
        self.assertNotEqual(framing.CMD_AGE_UNKNOWN, framing.CMD_AGE_MAX)
        self.assertEqual(framing.clamp_cmd_age_ms(10 ** 9), framing.CMD_AGE_MAX)
        self.assertLess(framing.clamp_cmd_age_ms(10 ** 9), framing.CMD_AGE_UNKNOWN)

    def test_clamp_cmd_age_floors_at_zero(self):
        self.assertEqual(framing.clamp_cmd_age_ms(-5), 0)

    def test_oversized_payload_rejected_at_encode(self):
        with self.assertRaises(ValueError):
            framing.encode_frame(framing.MSG_CONTROL,
                                 b"\x00" * (framing.MAX_PAYLOAD_LEN + 1))


# ---------------------------------------------------------------------------
# Safety state machine (controller.py -- shared by firmware and simulator)
# ---------------------------------------------------------------------------

class TestControllerSafety(unittest.TestCase):
    def make(self, **kw):
        clock = FakeClock()
        return controller.RoverController(now_ms=clock, **kw), clock

    def drive(self, ctl, drive=500, steer=100, mode=framing.MODE_MANUAL, stop=False):
        ctl.ingest(framing.encode_control(drive, steer, mode, stop))

    def test_boots_stopped_and_disabled(self):
        """Before any command arrives the rover must not be able to move."""
        ctl, _ = self.make()
        self.assertTrue(ctl.effective_stop())
        self.assertTrue(ctl.watchdog_tripped())
        self.assertEqual(ctl.cmd_age_ms(), framing.CMD_AGE_UNKNOWN)
        self.assertEqual(ctl.commanded_outputs(), (0, 0))
        self.assertEqual(ctl.last_control["mode"], framing.MODE_DISABLED)

    def test_valid_control_releases_the_stop(self):
        ctl, _ = self.make()
        self.drive(ctl)
        self.assertFalse(ctl.effective_stop())
        self.assertEqual(ctl.commanded_outputs(), (500, 100))

    def test_watchdog_trips_after_timeout(self):
        ctl, clock = self.make(watchdog_timeout_ms=300)
        self.drive(ctl)
        clock.advance(299)
        self.assertFalse(ctl.watchdog_tripped(), "tripped early")
        clock.advance(1)
        self.assertTrue(ctl.watchdog_tripped(), "failed to trip at the boundary")
        self.assertTrue(ctl.effective_stop())
        self.assertEqual(ctl.commanded_outputs(), (0, 0))

    def test_watchdog_clears_when_commands_resume(self):
        ctl, clock = self.make(watchdog_timeout_ms=300)
        self.drive(ctl)
        clock.advance(500)
        self.assertTrue(ctl.effective_stop())
        self.drive(ctl)
        self.assertFalse(ctl.effective_stop(), "did not recover after link restored")

    def test_explicit_stop_flag_forces_stop_in_any_mode(self):
        for mode in (framing.MODE_MANUAL, framing.MODE_AUTONOMOUS):
            ctl, _ = self.make()
            self.drive(ctl, mode=mode, stop=True)
            self.assertTrue(ctl.effective_stop())
            self.assertEqual(ctl.commanded_outputs(), (0, 0))

    def test_stop_is_releasable_without_a_mode_change(self):
        """stop is a separate field rather than a third mode precisely so an
        e-stop can be asserted AND released without a mode round trip."""
        ctl, _ = self.make()
        self.drive(ctl, stop=True)
        self.assertTrue(ctl.effective_stop())
        self.drive(ctl, stop=False)
        self.assertFalse(ctl.effective_stop())

    def test_disabled_mode_forces_stop_even_with_nonzero_drive(self):
        ctl, _ = self.make()
        self.drive(ctl, drive=1000, mode=framing.MODE_DISABLED, stop=False)
        self.assertTrue(ctl.effective_stop())
        self.assertEqual(ctl.commanded_outputs(), (0, 0))

    def test_corrupt_control_does_not_refresh_the_watchdog(self):
        """A frame that fails CRC must NOT count as a valid command, or
        corruption would keep the watchdog alive while the rover acts on
        stale data."""
        ctl, clock = self.make(watchdog_timeout_ms=300)
        self.drive(ctl)
        clock.advance(290)
        bad = bytearray(framing.encode_control(0, 0, framing.MODE_MANUAL, False))
        bad[4] ^= 0xFF
        self.assertEqual(ctl.ingest(bytes(bad)), 0, "corrupt frame was accepted")
        clock.advance(11)
        self.assertTrue(ctl.watchdog_tripped(), "corrupt frame refreshed the watchdog")

    def test_telemetry_reports_comm_timeout_and_command_age(self):
        ctl, clock = self.make(watchdog_timeout_ms=300)
        self.drive(ctl)
        clock.advance(400)
        frame = ctl.telemetry_frame(enc_left=1, enc_right=2, steer_fb=3, current_ca=0)
        tm = framing.decode_telemetry(parser()[0].feed(frame)[0][1])
        self.assertTrue(tm["fault_status"] & framing.FAULT_COMM_TIMEOUT)
        self.assertEqual(tm["cmd_age_ms"], 400)

    def test_telemetry_age_is_unknown_before_first_command(self):
        ctl, _ = self.make()
        frame = ctl.telemetry_frame(0, 0, 0, 0)
        tm = framing.decode_telemetry(parser()[0].feed(frame)[0][1])
        self.assertEqual(tm["cmd_age_ms"], framing.CMD_AGE_UNKNOWN)
        self.assertTrue(tm["fault_status"] & framing.FAULT_COMM_TIMEOUT)

    def test_sensor_faults_are_preserved_alongside_comm_timeout(self):
        ctl, _ = self.make()
        frame = ctl.telemetry_frame(0, 0, 0, 0,
                                    sensor_faults=framing.FAULT_OVER_CURRENT)
        tm = framing.decode_telemetry(parser()[0].feed(frame)[0][1])
        self.assertEqual(sorted(framing.fault_names(tm["fault_status"])),
                         ["COMM_TIMEOUT", "OVER_CURRENT"])

    def test_telemetry_due_paces_at_the_configured_period(self):
        ctl, clock = self.make(telemetry_period_ms=50)
        self.assertTrue(ctl.telemetry_due())      # first call is due immediately
        self.assertFalse(ctl.telemetry_due())
        clock.advance(50)
        self.assertTrue(ctl.telemetry_due())

    def test_telemetry_does_not_burst_after_a_stall(self):
        """After a long stall the scheduler must resync to now rather than
        firing once per missed period in a burst."""
        ctl, clock = self.make(telemetry_period_ms=50)
        ctl.telemetry_due()
        clock.advance(5000)                       # simulate a 5 s stall
        self.assertTrue(ctl.telemetry_due())
        self.assertFalse(ctl.telemetry_due(), "burst-fired after a stall")


# ---------------------------------------------------------------------------
# Drift guards: the firmware and the host tools must share ONE implementation
# ---------------------------------------------------------------------------

class TestNoDuplicatedProtocol(unittest.TestCase):
    """The wire format used to be hand-copied into the firmware, with a comment
    asking future maintainers to "keep the two in sync". These tests make that
    structurally impossible instead of merely requested."""

    REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

    def source(self, name):
        with open(os.path.join(self.REPO, name)) as handle:
            return handle.read()

    def test_firmware_does_not_redefine_the_crc(self):
        self.assertNotIn("def crc16_ccitt", self.source("feather_main.py"))

    def test_firmware_does_not_redefine_struct_formats(self):
        src = self.source("feather_main.py")
        for token in ("CONTROL_FMT =", "TELEMETRY_FMT =", "START_BYTE ="):
            self.assertNotIn(token, src, "%s redefined in the firmware" % token)

    def test_firmware_does_not_reimplement_the_parser(self):
        self.assertNotIn("class FrameParser", self.source("feather_main.py"))

    def test_firmware_imports_the_shared_modules(self):
        src = self.source("feather_main.py")
        self.assertIn("import framing", src)
        self.assertIn("import controller", src)

    def test_simulator_shares_the_watchdog_constant(self):
        """The simulator must not carry its own copy of the timeout, or the
        demo could pass while the real firmware misbehaves."""
        self.assertIn("controller.DEFAULT_WATCHDOG_TIMEOUT_MS",
                      self.source("mcu_sim.py"))

    def test_simulator_does_not_reimplement_the_stop_rule(self):
        src = self.source("mcu_sim.py")
        self.assertNotIn("watchdog_tripped or", src)
        self.assertIn("controller.RoverController", src)

    def test_no_stale_pico_firmware_left_behind(self):
        """The board is a Feather RP2040 RFM9x; a leftover pico_main.py would
        be a second, diverging firmware."""
        for stale in ("pico_main.py", "pico_sim.py"):
            self.assertFalse(os.path.exists(os.path.join(self.REPO, stale)),
                             "%s still present" % stale)
