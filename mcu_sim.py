"""
mcu_sim.py -- DEMO ONLY. Stand-in for the embedded controller, so the full
CONTROL/TELEMETRY exchange can be demonstrated on any laptop with no Feather
in hand.

This is a genuine mirror, not a hand-copy: the command/safety state machine
is imported from controller.py -- the exact same module feather_main.py runs.
Only the plant is fake. The previous version of this file re-implemented the
watchdog timeout and fault logic separately, which meant the demo could pass
while the real firmware misbehaved.

What is still simulated rather than shared:
  * encoders / steering / current, which are faked instead of read
  * a Python thread instead of the board's bare-metal loop

Read feather_main.py for the code that actually gets flashed.
"""

import time

import controller
import framing

TELEMETRY_RATE_HZ = 20
TELEMETRY_PERIOD_MS = 1000 // TELEMETRY_RATE_HZ
WATCHDOG_TIMEOUT_MS = controller.DEFAULT_WATCHDOG_TIMEOUT_MS


def now_ms():
    return time.monotonic_ns() // 1_000_000


class McuSim:
    def __init__(self, endpoint):
        self.endpoint = endpoint
        self._running = False
        self.ctl = controller.RoverController(
            now_ms=now_ms,
            watchdog_timeout_ms=WATCHDOG_TIMEOUT_MS,
            telemetry_period_ms=TELEMETRY_PERIOD_MS,
        )

        # Simulated plant state
        self.enc_left = 0
        self.enc_right = 0
        self.steer_fb = 0
        self.current_ca = 0
        self.effective_stop = True   # initialized here, not lazily in a method

    @property
    def last_control(self):
        return self.ctl.last_control

    def stop(self):
        self._running = False

    def run_forever(self):
        self._running = True
        while self._running:
            n = getattr(self.endpoint, "in_waiting", 0)
            self.ctl.ingest(self.endpoint.read(n) if n else b"")

            now = now_ms()
            self.effective_stop = self.ctl.effective_stop(now)
            drive, steer = self.ctl.commanded_outputs(now)
            self._step_plant(drive, steer)

            if self.ctl.telemetry_due(now):
                self.endpoint.write(self.ctl.telemetry_frame(
                    enc_left=self.enc_left,
                    enc_right=self.enc_right,
                    steer_fb=self.steer_fb,
                    current_ca=self.current_ca,
                    now=now,
                ))

            time.sleep(0.005)

    def _step_plant(self, drive, steer_target):
        """Fake physics. `drive`/`steer_target` already have the fail-safe
        applied by controller.commanded_outputs(), so there is no second copy
        of the stop rule here."""
        # wrap_i32 mirrors the firmware: an unbounded counter would overflow
        # int32 and raise inside struct.pack.
        self.enc_left = framing.wrap_i32(self.enc_left + drive // 10)
        self.enc_right = framing.wrap_i32(self.enc_right + drive // 10)

        if self.steer_fb < steer_target:
            self.steer_fb = min(steer_target, self.steer_fb + 25)
        elif self.steer_fb > steer_target:
            self.steer_fb = max(steer_target, self.steer_fb - 25)

        self.current_ca = int(abs(drive) * 1.5)
