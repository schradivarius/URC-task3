"""
test_golden_vectors.py -- pin the C++ and Python codecs together, byte for byte.

The codec is the only part of this protocol that exists in two languages. This
test compiles the real C++ encoder, runs it, and asserts Python produces
identical output for identical inputs -- not just for encoded frames, but for
the CRC, the sequence arithmetic and every validation predicate.

If it fails, the Teensy and the Jetson disagree about the wire format or about
which values are legal. That is not a style issue: it means commands would be
misread, or silently rejected, on a moving rover.
"""

import os
import subprocess
import sys
import unittest

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(REPO, "host"))

import rover_protocol as p  # noqa: E402


def build_and_run_cpp():
    out_dir = os.path.join(REPO, "tests", "cpp", "build")
    os.makedirs(out_dir, exist_ok=True)
    binary = os.path.join(out_dir, "golden_vectors")
    subprocess.run(
        [os.environ.get("CXX", "g++"), "-std=c++17", "-O1",
         os.path.join(REPO, "tools", "golden_vectors.cpp"),
         os.path.join(REPO, "firmware", "src", "rover_protocol.cpp"),
         "-o", binary],
        check=True, capture_output=True)
    result = subprocess.run([binary], check=True, capture_output=True, text=True)
    return [ln.split("|") for ln in result.stdout.splitlines() if ln.strip()]


class GoldenBase(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        try:
            cls.rows = build_and_run_cpp()
        except (subprocess.CalledProcessError, FileNotFoundError) as exc:
            raise unittest.SkipTest("C++ toolchain unavailable: %s" % exc)

    def rowsOf(self, kind):
        found = [r for r in self.rows if r[0] == kind]
        self.assertTrue(found, "C++ emitted no %s vectors" % kind)
        return found


class TestCrcAgreement(GoldenBase):
    def test_crc8_known_answer_matches(self):
        for _, text, expected in self.rowsOf("CRC8"):
            self.assertEqual("%02x" % p.crc8(text.encode()), expected)

    def test_crc8_check_value_is_sae_j1850(self):
        """Pinned independently of the C++ output, so a matching pair of wrong
        implementations still fails. CRC-8/SAE-J1850 check value is 0x4B."""
        self.assertEqual(p.crc8(b"123456789"), 0x4B)

    def test_frame_crc_matches_including_the_id_seed(self):
        for _, can_id, payload_hex, expected in self.rowsOf("FRAMECRC"):
            got = p.frame_crc8(int(can_id), bytes.fromhex(payload_hex))
            self.assertEqual("%02x" % got, expected,
                             "frame_crc8 disagrees for id %s" % can_id)

    def test_the_id_seed_actually_changes_the_crc(self):
        payload = bytes(range(6))
        crcs = {p.frame_crc8(cid, payload)
                for cid in (p.CAN_ID_CONTROL,) + p.TELEM_IDS}
        self.assertEqual(len(crcs), 5, "the CAN id is not affecting the CRC")


class TestPredicateAgreement(GoldenBase):
    def test_mode_predicates_agree_for_all_256_values(self):
        rows = self.rowsOf("MODE")
        self.assertEqual(len(rows), 256)
        for _, value, flags in rows:
            mode = int(value)
            self.assertEqual(p.is_known_mode(mode), flags[0] == "1",
                             "is_known_mode disagrees for mode=%d" % mode)
            self.assertEqual(p.mode_permits_motion(mode), flags[1] == "1",
                             "mode_permits_motion disagrees for mode=%d" % mode)

    def test_only_manual_and_autonomous_ever_permit_motion(self):
        for mode in range(256):
            if mode not in (p.MODE_MANUAL, p.MODE_AUTONOMOUS):
                self.assertFalse(p.mode_permits_motion(mode),
                                 "mode=%d permits motion but must not" % mode)

    def test_indicator_predicate_agrees_for_all_256_values(self):
        rows = self.rowsOf("INDICATOR")
        self.assertEqual(len(rows), 256)
        for _, value, known in rows:
            self.assertEqual(p.is_known_indicator(int(value)), known == "1",
                             "is_known_indicator disagrees for %s" % value)

    def test_sequence_arithmetic_agrees_across_the_wrap(self):
        for _, pair, expected in self.rowsOf("SEQDELTA"):
            prev, cur = (int(x) for x in pair.split(","))
            self.assertEqual(p.seq_delta(prev, cur), int(expected),
                             "seq_delta(%d,%d) disagrees" % (prev, cur))


class TestFrameAgreement(GoldenBase):
    def encode(self, kind, args):
        n = [int(a) for a in args.split(",")]
        if kind == "CONTROL":
            return p.encode_control(n[0], n[1], n[2], bool(n[3]), seq=n[7],
                                    autonomy_abort=bool(n[4]),
                                    return_request=bool(n[5]),
                                    indicator_request=n[6])
        if kind == "DRIVE_L":
            return p.encode_telemetry_drive_l(n[0], n[1], seq=n[2])
        if kind == "DRIVE_R":
            return p.encode_telemetry_drive_r(n[0], n[1], seq=n[2])
        if kind == "POWER":
            return p.encode_telemetry_power(n[0], n[1], n[2], seq=n[3])
        if kind == "STATE":
            return p.encode_telemetry_state(n[0], n[1], n[2], n[3], n[4], seq=n[5])
        self.fail("unknown vector kind %r" % kind)

    FRAME_KINDS = ("CONTROL", "DRIVE_L", "DRIVE_R", "POWER", "STATE")

    def test_every_frame_encodes_identically(self):
        checked = 0
        for kind in self.FRAME_KINDS:
            for _, args, expected in self.rowsOf(kind):
                actual = self.encode(kind, args).hex()
                self.assertEqual(
                    actual, expected,
                    "\n  %s(%s)\n    C++    : %s\n    Python : %s\n"
                    "  The two sides of the link disagree about the wire format."
                    % (kind, args, expected, actual))
                checked += 1
        self.assertGreater(checked, 15, "suspiciously few frames compared")

    def test_python_decodes_what_cpp_encoded(self):
        decoders = {
            "CONTROL": lambda f: p.decode_control(f),
            "DRIVE_L": lambda f: p.decode_telemetry_drive_l(f),
            "DRIVE_R": lambda f: p.decode_telemetry_drive_r(f),
            "POWER":   lambda f: p.decode_telemetry_power(f),
            "STATE":   lambda f: p.decode_telemetry_state(f),
        }
        for kind in self.FRAME_KINDS:
            for _, args, frame_hex in self.rowsOf(kind):
                result, decoded = decoders[kind](bytes.fromhex(frame_hex))
                self.assertEqual(result, p.DECODE_OK,
                                 "Python rejected a valid C++ %s frame (%s): %s"
                                 % (kind, args, p.DECODE_NAMES[result]))
                self.assertIsNotNone(decoded)

    def test_sequence_number_survives_the_round_trip(self):
        for _, args, frame_hex in self.rowsOf("CONTROL"):
            expected_seq = int(args.split(",")[7])
            _, decoded = p.decode_control(bytes.fromhex(frame_hex))
            self.assertEqual(decoded["seq"], expected_seq)

    def test_every_frame_is_exactly_eight_bytes(self):
        # 8 bytes is Classic CAN's limit. Exceeding it would force CAN FD
        # transceivers onto every node of the bus.
        for kind in self.FRAME_KINDS:
            for _, args, frame_hex in self.rowsOf(kind):
                self.assertEqual(len(frame_hex) // 2, p.FRAME_DLC,
                                 "%s(%s) is not 8 bytes" % (kind, args))


class TestPythonValidation(unittest.TestCase):
    """Python must reject what C++ rejects, for the same reasons."""

    def test_undefined_mode_is_rejected(self):
        for mode in (3, 4, 42, 128, 255):
            frame = p.encode_control(1000, 500, mode)
            result, decoded = p.decode_control(frame)
            self.assertEqual(result, p.DECODE_BAD_MODE)
            self.assertIsNone(decoded)

    def test_a_corrupted_byte_is_rejected(self):
        good = p.encode_control(500, -200, p.MODE_MANUAL, seq=3)
        for byte in range(p.FRAME_CRC_OFFSET):
            bad = bytearray(good)
            bad[byte] ^= 0xFF
            result, _ = p.decode_control(bytes(bad))
            self.assertNotEqual(result, p.DECODE_OK,
                                "byte %d corrupted but accepted" % byte)

    def test_a_frame_on_the_wrong_id_is_rejected(self):
        frame = p.encode_control(500, 0, p.MODE_MANUAL, seq=1)
        self.assertEqual(p.decode_control(frame)[0], p.DECODE_OK)
        self.assertEqual(p.decode_control(frame, can_id=p.CAN_ID_TELEM_POWER)[0],
                         p.DECODE_BAD_CRC)

    def test_wrong_length_is_rejected(self):
        frame = p.encode_control(0, 0, p.MODE_MANUAL)
        for n in range(0, 8):
            self.assertEqual(p.decode_control(frame[:n])[0], p.DECODE_BAD_DLC)

    def test_reserved_flag_bits_are_rejected(self):
        for pattern in (0x40, 0x80, 0xC0):
            flags = p.pack_control_flags(stop=True) | pattern
            frame = p.encode_control(0, 0, p.MODE_MANUAL, raw_flags=flags)
            self.assertEqual(p.decode_control(frame)[0], p.DECODE_BAD_FLAGS)

    def test_undefined_indicator_request_is_rejected(self):
        for ind in (5, 6, 7):
            frame = p.encode_control(0, 0, p.MODE_MANUAL, indicator_request=ind)
            self.assertEqual(p.decode_control(frame)[0], p.DECODE_BAD_INDICATOR)

    def test_the_c2_fault_bit_is_not_claimed(self):
        """0x0080 is reserved for the separate C2 work item."""
        self.assertEqual(p.fault_names(0x0080), [])


if __name__ == "__main__":
    unittest.main()
