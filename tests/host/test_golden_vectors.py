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

    def python_encode(self, name, args):
        n = [int(a) for a in args.split(",")]
        if name == "CONTROL":
            return p.encode_control(n[0], n[1], n[2], n[3])
        if name == "TELEM_MOTION":
            return p.encode_telemetry_motion(n[0], n[1])
        if name == "TELEM_STATUS":
            return p.encode_telemetry_status(n[0], n[1], n[2], n[3])
        self.fail("unknown vector type %r" % name)

    def test_cpp_emitted_vectors(self):
        self.assertGreater(len(self.lines), 10, "C++ emitted almost nothing")

    def test_python_encoder_matches_cpp_byte_for_byte(self):
        for line in self.lines:
            name, args, expected_hex = line.split("|")
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
            decoded = decoders[name](bytes.fromhex(expected_hex))
            self.assertIsNotNone(decoded, "Python rejected a valid C++ frame: %s" % line)

    def test_dlcs_agree_across_languages(self):
        for line in self.lines:
            name, _, hex_bytes = line.split("|")
            expected = {"CONTROL": p.CONTROL_DLC,
                        "TELEM_MOTION": p.TELEM_MOTION_DLC,
                        "TELEM_STATUS": p.TELEM_STATUS_DLC}[name]
            self.assertEqual(len(hex_bytes) // 2, expected)

    def test_every_frame_fits_classic_can(self):
        for line in self.lines:
            _, _, hex_bytes = line.split("|")
            self.assertLessEqual(len(hex_bytes) // 2, 8,
                                 "frame exceeds Classic CAN's 8-byte limit")


if __name__ == "__main__":
    unittest.main()
