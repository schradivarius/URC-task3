#!/usr/bin/env python3
"""
jetson_test.py -- Jetson-side test harness for the rover CONTROL/TELEMETRY link.

Usage:
    # Against real hardware once the Pico is flashed with pico_main.py:
    python3 jetson_test.py --port /dev/ttyACM0 --baud 115200

    # Against the no-hardware mock loopback (talks to pico_sim.py in-process):
    python3 jetson_test.py --mock --duration 5

Sends CONTROL frames at a fixed rate, parses incoming TELEMETRY frames, and
prints a live status line. Declares the LINK itself down if no valid
TELEMETRY frame has arrived within LINK_TIMEOUT_S (separate from the
Pico's own command-age watchdog, which protects the rover if the Jetson
goes quiet).
"""

import argparse
import sys
import time

import framing

CONTROL_RATE_HZ = 20
LINK_TIMEOUT_S = 0.5  # if no telemetry for this long, consider the link down


class JetsonLink:
    def __init__(self, port):
        self.port = port
        self.parser = framing.FrameParser(now_ms=lambda: time.monotonic() * 1000)
        self.last_telemetry_time = None
        self.latest_telemetry = None
        self.frames_sent = 0
        self.telemetry_count = 0

    def send_control(self, drive_cmd, steer_cmd, mode, stop):
        frame = framing.encode_control(drive_cmd, steer_cmd, mode, stop)
        self.port.write(frame)
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
            # unknown/short frames are silently ignored here; a production
            # version should log framing.stats for diagnostics
        self.parser.check_timeout()

    @property
    def link_up(self):
        if self.last_telemetry_time is None:
            return False
        return (time.monotonic() - self.last_telemetry_time) < LINK_TIMEOUT_S


def open_real_port(path, baud):
    import serial  # imported lazily so --mock doesn't require pyserial installed
    return serial.Serial(path, baudrate=baud, timeout=0)


def run(port, duration_s, verbose=True):
    link = JetsonLink(port)
    period = 1.0 / CONTROL_RATE_HZ
    t_start = time.monotonic()
    t_next_send = t_start

    last_printed_count = -1
    while (time.monotonic() - t_start) < duration_s:
        now = time.monotonic()

        if now >= t_next_send:
            # Simple demo motion profile: gentle sweep on drive/steer.
            elapsed = now - t_start
            drive = int(300 * (1 if int(elapsed) % 4 < 2 else -1))
            steer = int(200 * ((elapsed % 2) - 1))
            link.send_control(drive_cmd=drive, steer_cmd=steer,
                               mode=framing.MODE_MANUAL, stop=False)
            t_next_send += period

        link.poll()

        if verbose and link.latest_telemetry and link.telemetry_count != last_printed_count:
            last_printed_count = link.telemetry_count
            tm = link.latest_telemetry
            faults = framing.fault_names(tm["fault_status"]) or ["none"]
            status = "UP" if link.link_up else "DOWN"
            print(
                f"\r[t={now - t_start:5.1f}s] link={status:4s} sent={link.frames_sent:4d} "
                f"enc=({tm['enc_left']:+6d},{tm['enc_right']:+6d}) "
                f"steer_fb={tm['steer_fb']:+5d} current={tm['current_ma']:5d}mA "
                f"cmd_age={tm['cmd_age_ms']:5d}ms faults={','.join(faults):<20s}",
                end="", flush=True,
            )

        time.sleep(0.005)

    if verbose:
        print()
    return link


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--port", help="serial device, e.g. /dev/ttyACM0")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--mock", action="store_true",
                     help="use the in-process mock loopback (runs pico_sim.py) instead of real hardware")
    ap.add_argument("--duration", type=float, default=10.0, help="seconds to run")
    args = ap.parse_args()

    if args.mock:
        import threading
        import mock_link
        import pico_sim

        jetson_side, pico_side = mock_link.make_pair()
        sim = pico_sim.PicoSim(pico_side)
        t = threading.Thread(target=sim.run_forever, daemon=True)
        t.start()
        run(jetson_side, args.duration)
        sim.stop()
    elif args.port:
        port = open_real_port(args.port, args.baud)
        run(port, args.duration)
    else:
        print("Specify --port /dev/ttyXXX for real hardware, or --mock for the loopback demo.",
              file=sys.stderr)
        sys.exit(1)


if __name__ == "__main__":
    main()
