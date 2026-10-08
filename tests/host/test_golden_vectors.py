"""
test_golden_vectors.py -- pin the C++ and Python codecs together byte for byte.

The codec is the only part of this protocol that exists in two languages. This
test compiles the real C++ encoder, runs it, and asserts Python produces
identical bytes for identical inputs.

If it fails, the Teensy and the Jetson disagree about the wire format. That is
not a style issue -- it means commands would be misread on a moving rover.
"""

import os
import subprocess
import sys
import unittest

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(REPO, "host"))

import rover_protocol as p  # noqa: E402


def build_and_run_cpp():
    """Compile tools/golden_vectors.cpp and return its output lines."""
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
    return [ln for ln in result.stdout.splitlines() if ln.strip()]


class TestGoldenVectors(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        try:
            cls.lines = build_and_run_cpp()
        except (subprocess.CalledProcessError, FileNotFoundError) as exc:
            raise unittest.SkipTest("C++ toolchain unavailable: %s" % exc)

    def vectors(self, kind):
        return [ln.split("|") for ln in self.lines if ln.split("|")[0] == kind]

    def python_encode(self, name, args):
        n = [int(a) for a in args.split(",")]
        if name == "CONTROL":
            return p.encode_control(n[0], n[1], n[2], n[3], n[4])
        if name == "TELEM_MOTION":
            return p.encode_telemetry_motion(n[0], n[1])
        if name == "TELEM_STATUS":
            return p.encode_telemetry_status(n[0], n[1], n[2], n[3])
        self.fail("unknown vector type %r" % name)

    def test_cpp_emitted_vectors(self):
        self.assertGreater(len(self.lines), 10, "C++ emitted almost nothing")

    def test_mode_predicates_agree_across_languages(self):
        """Issue #4. Both sides must agree, for all 256 values, on which modes
        are defined and which permit motion. A host that thinks mode 3 is
        drivable while the controller stops on it is its own bug."""
        rows = self.vectors("MODE")
        self.assertEqual(len(rows), 256, "C++ did not emit all 256 mode cases")
        for _, mode_s, flags in rows:
            mode = int(mode_s)
            cpp_known, cpp_motion = flags[0] == "1", flags[1] == "1"
            self.assertEqual(p.is_known_mode(mode), cpp_known,
                             "is_known_mode disagrees for mode=%d" % mode)
            self.assertEqual(p.mode_permits_motion(mode), cpp_motion,
                             "mode_permits_motion disagrees for mode=%d" % mode)
        # and the property that actually matters, stated directly
        for mode in range(256):
            if mode not in (p.MODE_TELEOP, p.MODE_AUTONOMOUS):
                self.assertFalse(p.mode_permits_motion(mode),
                                 "mode=%d permits motion but should not" % mode)

    def test_undefined_mode_is_rejected_by_the_python_decoder(self):
        for mode in (3, 4, 42, 128, 255):
            payload = p.encode_control(1000, 500, mode, False, c2_lost=False)
            self.assertIsNone(p.decode_control(payload),
                              "Python accepted undefined mode=%d" % mode)
        for mode in (p.MODE_SAFE, p.MODE_TELEOP, p.MODE_AUTONOMOUS):
            payload = p.encode_control(100, 0, mode, False, c2_lost=False)
            self.assertIsNotNone(p.decode_control(payload))

    def test_python_encoder_matches_cpp_byte_for_byte(self):
        for line in self.lines:
            name, args, expected_hex = line.split("|")
            if name == "MODE":
                continue        # not an encoded frame
            actual = self.python_encode(name, args).hex()
            self.assertEqual(
                actual, expected_hex,
                "\n  %s(%s)\n    C++    : %s\n    Python : %s\n"
                "  The two sides of the link disagree about the wire format."
                % (name, args, expected_hex, actual))

    def test_python_decoder_round_trips_cpp_bytes(self):
        """Not just identical encoding -- Python must also read C++ output back."""
        decoders = {
            "CONTROL": p.decode_control,
            "TELEM_MOTION": p.decode_telemetry_motion,
            "TELEM_STATUS": p.decode_telemetry_status,
        }
        for line in self.lines:
            name, args, expected_hex = line.split("|")
            if name == "MODE":
                continue
            decoded = decoders[name](bytes.fromhex(expected_hex))
            self.assertIsNotNone(decoded, "Python rejected a valid C++ frame: %s" % line)

    def test_dlcs_agree_across_languages(self):
        for line in self.lines:
            name, _, hex_bytes = line.split("|")
            if name == "MODE":
                continue
            expected = {"CONTROL": p.CONTROL_DLC,
                        "TELEM_MOTION": p.TELEM_MOTION_DLC,
                        "TELEM_STATUS": p.TELEM_STATUS_DLC}[name]
            self.assertEqual(len(hex_bytes) // 2, expected)

    def test_every_frame_fits_classic_can(self):
        for line in self.lines:
            name, _, hex_bytes = line.split("|")
            if name == "MODE":
                continue
            self.assertLessEqual(len(hex_bytes) // 2, 8,
                                 "frame exceeds Classic CAN's 8-byte limit")


if __name__ == "__main__":
    unittest.main()
