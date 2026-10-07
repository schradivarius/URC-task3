#!/usr/bin/env python3
"""
jetson_test.py -- Jetson-side test harness for the rover CAN link.

    # Against the compiled simulator (no hardware, no CAN drivers):
    python3 host/jetson_test.py --mock --duration 5

    # Against a Linux virtual CAN bus:
    #   sudo modprobe vcan && sudo ip link add dev vcan0 type vcan
    #   sudo ip link set up vcan0
    python3 host/jetson_test.py --channel vcan0

    # Against the real Teensy on a real bus:
    python3 host/jetson_test.py --channel can0 --bitrate 500000

    # Cut the base station 2s in, to watch C2_LINK_LOST appear and stop it:
    python3 host/jetson_test.py --mock --duration 6 --drop-c2-at 2

    # Same cut in autonomy, where it must NOT stop the rover:
    python3 host/jetson_test.py --mock --duration 6 --drop-c2-at 2 \
        --mode autonomous

Sends CONTROL at 20 Hz and reports the two telemetry frames as they arrive.
Declares the LINK down if no telemetry has arrived within LINK_TIMEOUT_S --
which is separate from, and no substitute for, the controller's own command-age
watchdog. The rover's safety never depends on the Jetson noticing anything.
"""

import argparse
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import can_link  # noqa: E402
import rover_protocol as rp  # noqa: E402
import c2_link  # noqa: E402

CONTROL_RATE_HZ = 20
LINK_TIMEOUT_S = 0.5


class JetsonLink:
    """Sends CONTROL, accumulates the two telemetry frames into one view."""

    def __init__(self, link):
        self.link = link
        self.motion = None
        self.status = None
        self.frames_sent = 0
        self.motion_count = 0
        self.status_count = 0
        self.unknown_frames = 0
        self.bad_dlc_frames = 0
        # PROTOCOL_ERROR self-clears on the next good CONTROL frame, so it can
        # appear in a single status frame. Counted here, per frame, so a caller
        # sampling self.status at its own pace cannot miss it.
        self.protocol_error_reports = 0
        self.last_rx_time = None

    def send_control(self, drive_cmd, steer_cmd, mode, stop, c2_lost):
        self.link.send(rp.CAN_ID_CONTROL,
                       rp.encode_control(drive_cmd, steer_cmd, mode, stop, c2_lost))
        self.frames_sent += 1

    def poll(self):
        for can_id, payload in self.link.recv():
            if can_id == rp.CAN_ID_TELEM_MOTION:
                decoded = rp.decode_telemetry_motion(payload)
                if decoded is None:
                    self.bad_dlc_frames += 1
                    continue
                self.motion = decoded
                self.motion_count += 1
            elif can_id == rp.CAN_ID_TELEM_STATUS:
                decoded = rp.decode_telemetry_status(payload)
                if decoded is None:
                    self.bad_dlc_frames += 1
                    continue
                self.status = decoded
                self.status_count += 1
                if decoded["fault_status"] & rp.FAULT_PROTOCOL_ERROR:
                    self.protocol_error_reports += 1
            else:
                # A shared bus carries motor-controller and payload traffic.
                # Counted rather than silently dropped, so an unexpected id is
                # visible instead of invisible.
                self.unknown_frames += 1
                continue
            self.last_rx_time = time.monotonic()

    @property
    def link_up(self):
        if self.last_rx_time is None:
            return False
        return (time.monotonic() - self.last_rx_time) < LINK_TIMEOUT_S

    def format_status(self, elapsed):
        if self.status is None:
            return "[t=%5.1fs] waiting for telemetry..." % elapsed
        faults = rp.describe_faults(self.status["fault_status"])
        age = self.status["cmd_age_ms"]
        age_txt = ("never" if age == rp.CMD_AGE_UNKNOWN
                   else ">%dms" % rp.CMD_AGE_MAX if age == rp.CMD_AGE_MAX
                   else "%dms" % age)
        enc = self.motion or {"enc_left": 0, "enc_right": 0}
        return ("[t=%5.1fs] link=%-4s sent=%4d rx=%4d enc=(%+9d,%+9d) "
                "steer_fb=%+5d current=%+7.2fA cmd_age=%-8s faults=%-28s"
                % (elapsed, "UP" if self.link_up else "DOWN", self.frames_sent,
                   self.motion_count + self.status_count,
                   enc["enc_left"], enc["enc_right"], self.status["steer_fb"],
                   self.status["current_a"], age_txt, faults))


def run(link, c2, duration_s, verbose=True, mode=rp.MODE_MANUAL,
        drop_c2_at=None, restore_c2_at=None):
    """Drive CONTROL at 20 Hz for duration_s.

    drop_c2_at / restore_c2_at are seconds from the start at which to cut and
    restore the simulated base-station link. They exist so the C2 loss path can
    actually be walked end to end: before this, open_c2_link() connected and
    nothing ever disconnected, so c2_lost could only ever read False and the
    whole branch was dead code at runtime.
    """
    monitor = c2_link.C2Monitor()
    jl = JetsonLink(link)
    period = 1.0 / CONTROL_RATE_HZ
    t_start = time.monotonic()
    t_next = t_start
    last_printed = -1
    dropped = False
    restored = False

    while (time.monotonic() - t_start) < duration_s:
        now = time.monotonic()
        elapsed = now - t_start

        # Scripted link events, applied once each.
        if drop_c2_at is not None and not dropped and elapsed >= drop_c2_at:
            c2.disconnect()
            dropped = True
            if verbose:
                print("\n>>> C2 link cut at t=%.1fs" % elapsed)
        if restore_c2_at is not None and not restored and elapsed >= restore_c2_at:
            c2.connect()
            restored = True
            if verbose:
                print("\n>>> C2 link restored at t=%.1fs" % elapsed)

        if now >= t_next:
            drive = 300 if int(elapsed) % 4 < 2 else -300
            steer = int(200 * ((elapsed % 2) - 1))
            c2_loss = c2_link.poll_c2_lost(c2, monitor, now)
            jl.send_control(drive, steer, mode, stop=False, c2_lost=c2_loss)
            t_next += period
            if now - t_next > period:
                t_next = now + period          # do not burst after a stall
        jl.poll()

        total = jl.motion_count + jl.status_count
        if verbose and jl.status and total != last_printed:
            last_printed = total
            print("\r" + jl.format_status(now - t_start), end="", flush=True)
        time.sleep(0.005)

    if verbose:
        print()
        print("sent=%d motion=%d status=%d unknown_id=%d bad_dlc=%d"
              % (jl.frames_sent, jl.motion_count, jl.status_count,
                 jl.unknown_frames, jl.bad_dlc_frames))
    return jl


def open_can_link(args):
    if args.mock:
        return can_link.SimLink()
    return can_link.SocketCanLink(channel=args.channel, bitrate=args.bitrate)

def open_c2_link(args):
    # Both paths are the simulated link for now; swap in the real base-station
    # link here once it exists. The monitor above does not care which it is --
    # it only ever sees whether recv() produced anything.
    link = c2_link.SimC2Link()
    link.connect()
    return link


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--mock", action="store_true",
                    help="run against the compiled C++ simulator (no hardware)")
    ap.add_argument("--channel", help="CAN interface, e.g. can0 or vcan0")
    ap.add_argument("--bitrate", type=int, default=500000)
    ap.add_argument("--duration", type=float, default=10.0)
    ap.add_argument("--mode", choices=("manual", "autonomous"), default="manual",
                    help="CONTROL mode to send; C2 loss stops the rover in "
                         "manual but deliberately not in autonomous")
    ap.add_argument("--drop-c2-at", type=float, metavar="SECONDS",
                    help="cut the simulated base-station link this many "
                         "seconds in, to exercise the C2-loss path")
    ap.add_argument("--restore-c2-at", type=float, metavar="SECONDS",
                    help="restore the base-station link this many seconds in")
    args = ap.parse_args()

    if not args.mock and not args.channel:
        print("Specify --mock for the simulator, or --channel can0 for a real bus.",
              file=sys.stderr)
        sys.exit(1)

    link = open_can_link(args)
    c2 = open_c2_link(args)
    try:
        mode = (rp.MODE_AUTONOMOUS if args.mode == "autonomous"
                else rp.MODE_MANUAL)
        run(link, c2, args.duration, mode=mode,
            drop_c2_at=args.drop_c2_at, restore_c2_at=args.restore_c2_at)
    finally:
        link.close()
        c2.disconnect()


if __name__ == "__main__":
    main()
