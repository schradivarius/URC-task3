"""
test_config_sync.py -- pin host/rover_config.py to firmware/src/rover_config.h.

The constants exist in both languages because the Teensy runs C++ and the
Jetson runs Python. If someone changes a timeout, an id or a payload size on
one side only, the two ends would silently disagree on a moving rover. This
test turns that into a CI failure instead.

It reads the C++ header as text rather than compiling it, so it needs no
toolchain and runs anywhere the host tests run.
"""

import os
import re
import sys
import unittest

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(REPO, "host"))

import c2_link  # noqa: E402
import rover_config as cfg  # noqa: E402
import rover_protocol as rp  # noqa: E402

HEADER_PATH = os.path.join(REPO, "firmware", "src", "rover_config.h")

# Python name -> C++ name, for every constant that exists on both sides.
SHARED = {
    "PROTOCOL_VERSION":    "PROTOCOL_VERSION",
    "CAN_BITRATE_HZ":      "CAN_BITRATE_HZ",
    "WATCHDOG_TIMEOUT_MS": "DEFAULT_WATCHDOG_TIMEOUT_MS",
    "TELEMETRY_PERIOD_MS": "DEFAULT_TELEMETRY_PERIOD_MS",
    "CAN_ID_CONTROL":      "CAN_ID_CONTROL",
    "CAN_ID_TELEM_MOTION": "CAN_ID_TELEM_MOTION",
    "CAN_ID_TELEM_STATUS": "CAN_ID_TELEM_STATUS",
    "MODE_DISABLED":       "MODE_DISABLED",
    "MODE_MANUAL":         "MODE_MANUAL",
    "MODE_AUTONOMOUS":     "MODE_AUTONOMOUS",
    "CMD_MIN":             "CMD_MIN",
    "CMD_MAX":             "CMD_MAX",
}


def cpp_constants():
    """Every `NAME = <integer literal>` in rover_config.h, comments stripped."""
    with open(HEADER_PATH) as f:
        text = re.sub(r"//[^\n]*", "", f.read())
    found = {}
    for name, value in re.findall(
            r"\b([A-Z][A-Z0-9_]*)\s*=\s*(-?\s*(?:0[xX][0-9A-Fa-f]+|\d+))\b", text):
        found[name] = int(value.replace(" ", ""), 0)
    return found


class TestConfigSync(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.cpp = cpp_constants()

    def test_header_was_parsed(self):
        # Guards the test itself: a regex that matched nothing would make
        # every comparison below vacuous.
        for cpp_name in SHARED.values():
            self.assertIn(cpp_name, self.cpp,
                          "%s not found in rover_config.h" % cpp_name)

    def test_python_config_matches_cpp_config(self):
        for py_name, cpp_name in SHARED.items():
            self.assertEqual(
                getattr(cfg, py_name), self.cpp[cpp_name],
                "%s differs: rover_config.py=%r, rover_config.h=%r -- the "
                "Jetson and the Teensy disagree"
                % (py_name, getattr(cfg, py_name), self.cpp[cpp_name]))

    def test_python_payload_sizes_match_cpp(self):
        # Python derives its sizes from the struct formats; C++ states them.
        # A field added on one side only shows up here first.
        self.assertEqual(rp.CONTROL_DLC, self.cpp["CONTROL_DLC"])
        self.assertEqual(rp.TELEM_MOTION_DLC, self.cpp["TELEM_MOTION_DLC"])
        self.assertEqual(rp.TELEM_STATUS_DLC, self.cpp["TELEM_STATUS_DLC"])

    def test_mode_max_is_the_highest_defined_mode(self):
        # C++ defines MODE_MAX as MODE_AUTONOMOUS (checked by name above).
        self.assertEqual(cfg.MODE_MAX, max(rp.KNOWN_MODES))

    def test_link_timeout_is_longer_than_a_telemetry_period(self):
        # Otherwise the Jetson would report the link down between two
        # perfectly on-time telemetry frames.
        self.assertGreater(cfg.LINK_TIMEOUT_S * 1000, cfg.TELEMETRY_PERIOD_MS)

    def test_watchdog_tolerates_a_missed_control_frame(self):
        # The controller must not stop the rover over a single late frame.
        self.assertGreater(cfg.WATCHDOG_TIMEOUT_MS, 2 * 1000 // cfg.CONTROL_RATE_HZ)

    def test_firmware_only_constants_are_in_the_config_header(self):
        # These have no Python counterpart, so the comparison above cannot
        # catch them drifting back out into rover_firmware.ino.
        for name in ("HW_WATCHDOG_MS", "OVER_CURRENT_CA"):
            self.assertIn(name, self.cpp,
                          "%s belongs in rover_config.h" % name)

    def test_c2_timeout_is_longer_than_the_command_watchdog(self):
        # A radio drops packets far more often than a CAN bus, so the C2
        # timeout must not be as tight as the command watchdog. If these ever
        # converge, a single radio hiccup starts stopping the rover in MANUAL.
        self.assertGreater(cfg.C2_TIMEOUT_S * 1000, cfg.WATCHDOG_TIMEOUT_MS)

    def test_c2_monitor_defaults_to_the_configured_timeout(self):
        # The constant is worthless if the only caller hardcodes its own.
        self.assertEqual(c2_link.C2Monitor().C2_timeout_s, cfg.C2_TIMEOUT_S)


if __name__ == "__main__":
    unittest.main()
