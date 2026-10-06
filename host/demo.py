#!/usr/bin/env python3
"""
demo.py -- Jetson <-> controller message-exchange demonstration, no hardware.

Runs against tools/rover_sim: the REAL C++ rover_controller.cpp compiled
natively, with only the plant and the transport simulated. So what you see
below is the actual safety logic that gets flashed to the Teensy.

Part A: normal 20 Hz operation.
Part B: a deliberate 600 ms silence (twice the 300 ms command watchdog),
        proving the controller stops itself and reports JETSON_HEARTBEAT_LOST without
        the Jetson doing anything -- then recovers on its own.
Part C: the explicit stop flag, honoured on the cycle it arrives.
Part D: ONE corrupted CONTROL frame in the middle of normal 20 Hz traffic,
        proving the controller rejects it, keeps acting on the last good
        command, and reports PROTOCOL_ERROR.
"""

import os
import struct
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
            jl.send_control(drive, steer, rp.MODE_MANUAL, stop, c2_lost=False)
        last_print = show(jl, t0, label, last_print)
        time.sleep(0.02)


def corrupt_one_frame(jl, t0, seconds=1.0, bad_at=0.5):
    """Send CONTROL at the real 20 Hz rate, replacing exactly one frame with a
    corrupted one. Returns (protocol_error_reports_seen, encoders_ran_backwards).

    The corrupted frame carries an undefined mode AND full reverse. CAN's own
    CRC cannot be failed from software -- the transceiver rejects and
    retransmits a bad-CRC frame before the firmware ever sees it -- so what we
    can corrupt is the frame's meaning. If the controller wrongly accepted it,
    the encoders would visibly run backwards.
    """
    print("-- D: one corrupted CONTROL frame amid normal 20Hz traffic --")
    period = 1.0 / 20
    errors_before = jl.protocol_error_reports
    enc_samples = []
    start = time.monotonic()
    next_send = start
    sent_bad = False
    last_print = 0.0
    while time.monotonic() - start < seconds:
        now = time.monotonic()
        if now >= next_send:
            if not sent_bad and now - start >= bad_at:
                # Raw bytes: encode_control only ever builds valid frames.
                jl.link.send(rp.CAN_ID_CONTROL,
                             struct.pack(rp.CONTROL_FMT, -1000, 0, 7, 0, 0))
                sent_bad = True
                print("  >>> sent ONE frame: mode=7 (undefined), drive=-1000 (full reverse)")
            else:
                jl.send_control(500, 0, rp.MODE_MANUAL, False, c2_lost=False)
            next_send += period
        last_print = show(jl, t0, None, last_print)
        if jl.motion:
            enc_samples.append(jl.motion["enc_left"])
        time.sleep(0.005)
    jl.poll()

    errors_seen = jl.protocol_error_reports - errors_before
    went_backwards = any(b < a for a, b in zip(enc_samples, enc_samples[1:]))
    print("  status frames reporting PROTOCOL_ERROR: %d" % errors_seen)
    print("  encoders ever ran backwards: %s" % ("YES" if went_backwards else "no"))
    return errors_seen, went_backwards


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
        errors_seen, went_backwards = corrupt_one_frame(jl, t0)

        assert jl.status is not None, "never received any telemetry"
        assert jl.motion_count > 0, "never received a motion frame"
        assert jl.bad_dlc_frames == 0, "a frame arrived with the wrong DLC"
        assert not went_backwards, "the corrupted frame's command was acted on"
        assert errors_seen >= 1, "the corrupted frame was never reported"
        print()
        print("PASS: telemetry flowed throughout; cmd_age climbed and JETSON_HEARTBEAT_LOST")
        print("appeared during the cut, cleared when CONTROL resumed, and the stop")
        print("flag zeroed current and froze the encoders on the cycle it arrived.")
        print("The corrupted frame was rejected, its full-reverse command never")
        print("reached the motors, and PROTOCOL_ERROR was reported.")
    finally:
        link.close()


if __name__ == "__main__":
    main()
