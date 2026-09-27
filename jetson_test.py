#!/usr/bin/env python3
"""
jetson_test.py -- Onboard-computer-side test harness for the rover
CONTROL/TELEMETRY link.

Usage:
    # Against real hardware, once the Feather is running feather_main.py.
    # NOTE the port: with boot.py installed, ttyACM0 is the CircuitPython
    # REPL console and ttyACM1 is the data channel carrying our frames.
    python3 jetson_test.py --port /dev/ttyACM1

    # Over a USB-TTL adapter wired to the Feather's TX/RX pads instead:
    python3 jetson_test.py --port /dev/ttyUSB0

    # Against the no-hardware mock loopback (runs mcu_sim.py in-process):
    python3 jetson_test.py --mock --duration 5

Sends CONTROL frames at a fixed rate, parses incoming TELEMETRY, and prints a
live status line. Declares the LINK down if no valid TELEMETRY has arrived
within LINK_TIMEOUT_S -- which is separate from, and does not substitute for,
the controller's own command-age watchdog. The rover's safety never depends
on the onboard computer noticing anything.
"""

import argparse
import sys
import time

import framing

CONTROL_RATE_HZ = 20
LINK_TIMEOUT_S = 0.5  # no telemetry for this long -> consider the link down


class JetsonLink:
    def __init__(self, port):
        self.port = port
        self.parser = framing.FrameParser(now_ms=lambda: time.monotonic() * 1000)
        self.last_telemetry_time = None
        self.latest_telemetry = None
        self.frames_sent = 0
        self.telemetry_count = 0
        self.unknown_frames = 0

    def send_control(self, drive_cmd, steer_cmd, mode, stop):
        self.port.write(framing.encode_control(drive_cmd, steer_cmd, mode, stop))
        self.frames_sent += 1

    def poll(self):
        """Call frequently. Reads whatever bytes are available, parses any
        complete TELEMETRY frames, and updates link-timeout state."""
        n = getattr(self.port, "in_waiting", 0)
        data = self.port.read(n) if n else b""
        for msg_id, payload in self.parser.feed(data):
            if msg_id == framing.MSG_TELEMETRY and len(payload) == framing.TELEMETRY_LEN:
                self.latest_telemetry = framing.decode_telemetry(payload)
                self.last_telemetry_time = time.monotonic()
                self.telemetry_count += 1
            else:
                # Counted rather than silently dropped: a rising count here
                # means a version mismatch between the two sides.
                self.unknown_frames += 1
        self.parser.check_timeout()

    @property
    def link_up(self):
        if self.last_telemetry_time is None:
            return False
        return (time.monotonic() - self.last_telemetry_time) < LINK_TIMEOUT_S

    def format_status(self, elapsed):
        tm = self.latest_telemetry
        faults = ",".join(framing.fault_names(tm["fault_status"])) or "none"
        age = tm["cmd_age_ms"]
        age_txt = "never" if age == framing.CMD_AGE_UNKNOWN else (
            ">%dms" % framing.CMD_AGE_MAX if age == framing.CMD_AGE_MAX else "%dms" % age
        )
        return (
            "[t=%5.1fs] link=%-4s sent=%4d rx=%4d "
            "enc=(%+8d,%+8d) steer_fb=%+5d current=%+7.2fA "
            "cmd_age=%-8s faults=%-28s"
            % (elapsed, "UP" if self.link_up else "DOWN", self.frames_sent,
               self.telemetry_count, tm["enc_left"], tm["enc_right"],
               tm["steer_fb"], tm["current_a"], age_txt, faults)
        )


def open_real_port(path, baud):
    import serial  # lazy import so --mock does not require pyserial
    return serial.Serial(path, baudrate=baud, timeout=0)


def run(port, duration_s, verbose=True):
    link = JetsonLink(port)
    period = 1.0 / CONTROL_RATE_HZ
    t_start = time.monotonic()
    t_next_send = t_start
    last_printed = -1

    while (time.monotonic() - t_start) < duration_s:
        now = time.monotonic()

        if now >= t_next_send:
            # Simple demo motion profile: gentle sweep on drive and steering.
            elapsed = now - t_start
            drive = 300 if int(elapsed) % 4 < 2 else -300
            steer = int(200 * ((elapsed % 2) - 1))
            link.send_control(drive_cmd=drive, steer_cmd=steer,
                              mode=framing.MODE_MANUAL, stop=False)
            t_next_send += period
            if now - t_next_send > period:
                t_next_send = now + period  # do not burst after a stall

        link.poll()

        if verbose and link.latest_telemetry and link.telemetry_count != last_printed:
            last_printed = link.telemetry_count
            print("\r" + link.format_status(now - t_start), end="", flush=True)

        time.sleep(0.005)

    if verbose:
        print()
        print("parser stats: %s" % (link.parser.stats,))
        if link.unknown_frames:
            print("WARNING: %d unrecognized frames -- check both sides are on "
                  "the same protocol version" % link.unknown_frames)
    return link


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", help="serial device, e.g. /dev/ttyACM1 (see note above)")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--mock", action="store_true",
                    help="use the in-process mock loopback (runs mcu_sim.py) "
                         "instead of real hardware")
    ap.add_argument("--duration", type=float, default=10.0, help="seconds to run")
    args = ap.parse_args()

    if args.mock:
        import threading
        import mock_link
        import mcu_sim

        jetson_side, mcu_side = mock_link.make_pair()
        sim = mcu_sim.McuSim(mcu_side)
        thread = threading.Thread(target=sim.run_forever, daemon=True)
        thread.start()
        try:
            run(jetson_side, args.duration)
        finally:
            sim.stop()
            thread.join(timeout=1)
    elif args.port:
        run(open_real_port(args.port, args.baud), args.duration)
    else:
        print("Specify --port /dev/ttyXXX for real hardware, or --mock for the "
              "loopback demo.", file=sys.stderr)
        sys.exit(1)


if __name__ == "__main__":
    main()
