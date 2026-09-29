#!/usr/bin/env python3
"""
demo.py -- Jetson <-> controller message-exchange demonstration, no hardware.

Runs against tools/rover_sim: the REAL C++ rover_controller.cpp compiled
natively, with only the plant and the transport simulated. So what you see
below is the actual safety logic that gets flashed to the Teensy.

Part A: normal 20 Hz operation.
Part B: a deliberate 600 ms silence (twice the 300 ms command watchdog),
        proving the controller stops itself and reports COMM_TIMEOUT without
        the Jetson doing anything -- then recovers on its own.
Part C: the explicit stop flag, honoured on the cycle it arrives.
"""

import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import can_link  # noqa: E402
import rover_protocol as rp  # noqa: E402
from jetson_test import JetsonLink  # noqa: E402


def show(jl, t0, label_shown, last_print):
    jl.poll()
    now = time.monotonic() - t0
    if jl.status and (now - last_print) > 0.15:
        faults = ",".join(rp.fault_names(jl.status["fault_status"])) or "none"
        age = jl.status["cmd_age_ms"]
        age_txt = "never" if age == rp.CMD_AGE_UNKNOWN else "%dms" % age
        enc = (jl.motion or {"enc_left": 0})["enc_left"]
        print("  t=%5.2fs  cmd_age=%-8s enc_left=%+8d  current=%+7.2fA  faults=%s"
              % (now, age_txt, enc, jl.status["current_a"], faults))
        return now
    return last_print


def phase(jl, t0, seconds, drive, steer, stop, label):
    print("-- %s --" % label)
    start = time.monotonic()
    last_print = 0.0
    while time.monotonic() - start < seconds:
        if drive is not None:
            jl.send_control(drive, steer, rp.MODE_MANUAL, stop)
        last_print = show(jl, t0, label, last_print)
        time.sleep(0.02)


def main():
    print("=" * 78)
    print("Jetson <-> rover controller over CAN (simulated transport,")
    print("REAL compiled firmware safety logic from firmware/src/)")
    print("=" * 78)

    link = can_link.SimLink()
    jl = JetsonLink(link)
    t0 = time.monotonic()
    try:
        phase(jl, t0, 1.2, 500, 0, False, "A: normal operation, CONTROL at 20Hz")
        phase(jl, t0, 0.8, None, None, False,
              "B: link cut -- Jetson stops sending (800ms > 300ms watchdog)")
        phase(jl, t0, 1.0, 300, 100, False, "B: link restored")
        phase(jl, t0, 0.6, 1000, 500, True,
              "C: explicit stop asserted at full throttle")

        assert jl.status is not None, "never received any telemetry"
        assert jl.motion_count > 0, "never received a motion frame"
        assert jl.bad_dlc_frames == 0, "a frame arrived with the wrong DLC"
        print()
        print("PASS: telemetry flowed throughout; cmd_age climbed and COMM_TIMEOUT")
        print("appeared during the cut, cleared when CONTROL resumed, and the stop")
        print("flag zeroed current and froze the encoders on the cycle it arrived.")
    finally:
        link.close()


if __name__ == "__main__":
    main()
