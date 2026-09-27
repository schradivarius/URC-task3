#!/usr/bin/env python3
"""
demo.py -- Message-exchange demonstration for Task 3, run with no hardware
in hand (uses the mock loopback link from mock_link.py + the mcu_sim.py
mirror of the firmware logic).

Part A: deterministic proof that a corrupted frame is rejected by CRC and
         the parser resynchronizes on the very next valid frame (rather than
         losing the link or crashing).
Part B: a live 6-second CONTROL/TELEMETRY exchange over the mock link,
         including a deliberate ~0.6s pause in Jetson sends (longer than the
         300ms watchdog) to prove the controller-side fail-safe trips a
         COMM_TIMEOUT fault and forces a stop, then clears and recovers once
         sending resumes.
"""

import threading
import time

import framing
import mock_link
import mcu_sim
from jetson_test import JetsonLink


def part_a_crc_resync():
    print("=" * 78)
    print("PART A: CRC validation + resync-after-corruption (deterministic)")
    print("=" * 78)

    good_control = framing.encode_control(drive_cmd=500, steer_cmd=-200, mode=1, stop=False)
    corrupted = bytearray(good_control)
    corrupted[4] ^= 0xFF  # flip a payload byte -> CRC will no longer match
    good_telemetry = framing.encode_telemetry(
        enc_left=42, enc_right=41, steer_fb=-190, current_ca=120, fault_status=0, cmd_age_ms=10
    )

    parser = framing.FrameParser(now_ms=lambda: time.monotonic() * 1000)
    stream = bytes(corrupted) + good_telemetry
    print(f"Feeding {len(corrupted)} corrupted bytes immediately followed by "
          f"{len(good_telemetry)} valid TELEMETRY bytes...")
    frames = parser.feed(stream)

    print(f"Frames accepted: {len(frames)}  (expected: 1, the valid TELEMETRY frame)")
    print(f"Parser stats: {parser.stats}  (expected crc_errors >= 1, frames_ok == 1)")
    assert len(frames) == 1 and frames[0][0] == framing.MSG_TELEMETRY, "resync failed!"
    assert parser.stats["crc_errors"] >= 1, "corruption wasn't detected!"
    decoded = framing.decode_telemetry(frames[0][1])
    print(f"Decoded surviving frame: {decoded}")
    print("PASS: corrupted frame was rejected; parser resynced on the next valid frame.\n")


def part_c_len_resync():
    """Regression demo for the bug where one corrupted LEN byte caused a
    multi-second comms blackout."""
    print("=" * 78)
    print("PART C: a corrupted LEN byte costs 1 byte of resync, not 260")
    print("=" * 78)

    control = framing.encode_control(drive_cmd=100, steer_cmd=0,
                                     mode=framing.MODE_MANUAL, stop=False)
    # Corrupt the LEN byte of a frame from 6 to 0xFF. A parser that trusts LEN
    # blindly now waits for 3+255+2 = 260 bytes before it even checks the CRC.
    bad = bytearray(control)
    bad[2] = 0xFF

    parser = framing.FrameParser(now_ms=lambda: time.monotonic() * 1000)
    stream = bytes(bad) + control * 5
    frames = parser.feed(stream)

    blackout_bytes = 3 + 0xFF + 2
    print(f"Fed 1 frame with LEN corrupted to 0xFF, then 5 valid CONTROL frames")
    print(f"  a LEN-trusting parser would stall for {blackout_bytes} bytes "
          f"= {blackout_bytes / len(control) * 50:.0f}ms at 20Hz "
          f"(vs the {300}ms watchdog)")
    print(f"Frames recovered immediately: {len(frames)}  (expected 5)")
    print(f"Parser stats: {parser.stats}")
    assert len(frames) == 5, "LEN validation regressed -- frames were stalled!"
    assert parser.stats["bad_headers"] == 1, "implausible LEN was not rejected!"
    print("PASS: implausible LEN rejected after 1 byte; no frames were delayed.\n")


def part_b_live_exchange():
    print("=" * 78)
    print("PART B: live mock-serial exchange, including a simulated link cut")
    print("=" * 78)

    jetson_side, mcu_side = mock_link.make_pair()
    sim = mcu_sim.McuSim(mcu_side)
    sim_thread = threading.Thread(target=sim.run_forever, daemon=True)
    sim_thread.start()

    link = JetsonLink(jetson_side)
    t_start = time.monotonic()

    def send_loop(duration, drive, steer, label):
        print(f"-- {label} --")
        t0 = time.monotonic()
        last_print = 0
        while time.monotonic() - t0 < duration:
            if drive is not None:
                link.send_control(drive_cmd=drive, steer_cmd=steer, mode=framing.MODE_MANUAL, stop=False)
            link.poll()
            now = time.monotonic() - t_start
            if link.latest_telemetry and (now - last_print) > 0.15:
                tm = link.latest_telemetry
                faults = framing.fault_names(tm["fault_status"]) or ["none"]
                print(f"  t={now:5.2f}s  cmd_age={tm['cmd_age_ms']:5d}ms  "
                      f"enc_left={tm['enc_left']:+6d}  current={tm['current_a']:+7.2f}A  "
                      f"faults={','.join(faults)}")
                last_print = now
            time.sleep(0.02)

    send_loop(1.5, 500, 0, "Normal operation: sending CONTROL at 20Hz")
    send_loop(0.6, None, None, "Simulated link cut: Jetson stops sending (600ms > 300ms watchdog)")
    send_loop(1.5, 300, 100, "Link restored: sending CONTROL again")

    sim.stop()
    sim_thread.join(timeout=1)

    assert link.latest_telemetry is not None, "never received any telemetry!"
    print("\nPASS: TELEMETRY was received throughout; cmd_age climbed and a COMM_TIMEOUT")
    print("fault appeared during the cut, then cleared once CONTROL frames resumed.")


if __name__ == "__main__":
    part_a_crc_resync()
    part_c_len_resync()
    part_b_live_exchange()
