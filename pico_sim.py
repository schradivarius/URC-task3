"""
pico_sim.py -- DEMO ONLY. A pure-Python stand-in for the Pico side of the
link, used so the full CONTROL/TELEMETRY exchange can be demonstrated on
any machine without real Pico hardware.

This mirrors the protocol logic in pico_main.py (same watchdog timeout,
same fault bit, same message formats) but fakes the physical world instead
of reading real encoders/current sensors, and runs as a Python thread
instead of the Pico's bare-metal main loop. pico_main.py is the file you
actually flash to the board -- read that one to see the real firmware.
"""

import time

import framing

TELEMETRY_RATE_HZ = 20
WATCHDOG_TIMEOUT_MS = 300  # matches pico_main.py; see PROTOCOL.md section 6


class PicoSim:
    def __init__(self, endpoint):
        self.endpoint = endpoint
        self.parser = framing.FrameParser(now_ms=lambda: time.monotonic() * 1000)
        self._running = False

        # Simulated plant state
        self.enc_left = 0
        self.enc_right = 0
        self.steer_fb = 0
        self.last_control = {"drive_cmd": 0, "steer_cmd": 0, "mode": framing.MODE_DISABLED, "stop": True}
        self.last_control_time = None  # monotonic seconds of last VALID control frame

    def stop(self):
        self._running = False

    def run_forever(self):
        self._running = True
        period = 1.0 / TELEMETRY_RATE_HZ
        t_next_send = time.monotonic()

        while self._running:
            now = time.monotonic()

            n = getattr(self.endpoint, "in_waiting", 0)
            data = self.endpoint.read(n) if n else b""
            for msg_id, payload in self.parser.feed(data):
                if msg_id == framing.MSG_CONTROL and len(payload) == framing.CONTROL_LEN:
                    self.last_control = framing.decode_control(payload)
                    self.last_control_time = now
            self.parser.check_timeout()

            cmd_age_ms = self._compute_cmd_age_ms(now)
            watchdog_tripped = cmd_age_ms >= WATCHDOG_TIMEOUT_MS

            self._step_plant(watchdog_tripped)

            if now >= t_next_send:
                fault = 0
                if watchdog_tripped:
                    fault |= framing.FAULT_COMM_TIMEOUT
                current_ma = min(20000, abs(self.last_control["drive_cmd"]) * 15)
                frame = framing.encode_telemetry(
                    enc_left=self.enc_left,
                    enc_right=self.enc_right,
                    steer_fb=self.steer_fb,
                    current_ma=0 if self._effective_stop else current_ma,
                    fault_status=fault,
                    cmd_age_ms=cmd_age_ms,
                )
                self.endpoint.write(frame)
                t_next_send += period

            time.sleep(0.005)

    def _compute_cmd_age_ms(self, now):
        if self.last_control_time is None:
            return framing.CMD_AGE_UNKNOWN
        return min(framing.CMD_AGE_UNKNOWN, int((now - self.last_control_time) * 1000))

    def _step_plant(self, watchdog_tripped):
        # Fail-safe: watchdog expired -> force effective stop, same as pico_main.py
        effective_stop = watchdog_tripped or self.last_control["stop"] or (
            self.last_control["mode"] == framing.MODE_DISABLED
        )
        self._effective_stop = effective_stop # expose for run_forever to use
        drive = 0 if effective_stop else self.last_control["drive_cmd"]
        steer_target = 0 if effective_stop else self.last_control["steer_cmd"]

        # Fake encoder integration: ticks accumulate proportional to drive command
        self.enc_left += drive // 10
        self.enc_right += drive // 10

        # Fake steering servo slewing toward target
        if self.steer_fb < steer_target:
            self.steer_fb = min(steer_target, self.steer_fb + 25)
        elif self.steer_fb > steer_target:
            self.steer_fb = max(steer_target, self.steer_fb - 25)
