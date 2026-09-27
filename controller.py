"""
controller.py -- The rover's command/safety state machine, with no hardware
dependencies whatsoever.

WHY THIS MODULE EXISTS
  The watchdog timeout, the command-age computation and the three-way
  effective-stop decision are safety-critical. They used to be written out
  twice: once in the firmware and once in the simulator that "mirrors" it.
  Two hand-maintained copies of a safety rule is how you end up with a
  simulator that passes while the real firmware doesn't -- the tests would
  have been testing the wrong copy.

  So the rule lives here, once. feather_main.py drives it with real I/O;
  mcu_sim.py drives it with a fake plant. Both get identical behaviour by
  construction rather than by careful copy-editing.

  Runs on CPython and CircuitPython alike: only `framing` is imported, and
  time is injected as a callable.
"""

import framing

DEFAULT_WATCHDOG_TIMEOUT_MS = 300
DEFAULT_TELEMETRY_PERIOD_MS = 50   # 20 Hz


class RoverController:
    def __init__(self, now_ms,
                 watchdog_timeout_ms=DEFAULT_WATCHDOG_TIMEOUT_MS,
                 telemetry_period_ms=DEFAULT_TELEMETRY_PERIOD_MS,
                 inter_byte_timeout_ms=50):
        self.now_ms = now_ms
        self.watchdog_timeout_ms = watchdog_timeout_ms
        self.telemetry_period_ms = telemetry_period_ms
        self.parser = framing.FrameParser(now_ms=now_ms,
                                          inter_byte_timeout_ms=inter_byte_timeout_ms)

        # Boot state is the safest available: stopped, disabled, and with no
        # command ever received -- so cmd_age reads CMD_AGE_UNKNOWN, the
        # watchdog is tripped from the first cycle, and the rover cannot move
        # until the onboard computer actually asks it to.
        self.last_control = {"drive_cmd": 0, "steer_cmd": 0,
                             "mode": framing.MODE_DISABLED, "stop": True}
        self.last_control_ms = None
        self.control_frames_accepted = 0
        self._next_telemetry_ms = now_ms()

    # -- receive ------------------------------------------------------------

    def ingest(self, data):
        """Feed received bytes. Returns the number of CONTROL frames accepted.
        Also services the parser's inter-byte timeout."""
        accepted = 0
        for msg_id, payload in self.parser.feed(data):
            if msg_id == framing.MSG_CONTROL and len(payload) == framing.CONTROL_LEN:
                self.last_control = framing.decode_control(payload)
                self.last_control_ms = self.now_ms()
                self.control_frames_accepted += 1
                accepted += 1
        self.parser.check_timeout()
        return accepted

    # -- safety decision ----------------------------------------------------

    def cmd_age_ms(self, now=None):
        if self.last_control_ms is None:
            return framing.CMD_AGE_UNKNOWN
        now = self.now_ms() if now is None else now
        return framing.clamp_cmd_age_ms(int(now - self.last_control_ms))

    def watchdog_tripped(self, now=None):
        if self.last_control_ms is None:
            return True
        return self.cmd_age_ms(now) >= self.watchdog_timeout_ms

    def effective_stop(self, now=None):
        """The single fail-safe decision. Comm timeout, an explicit stop flag,
        and DISABLED mode all force a stop through this one path."""
        return (
            self.watchdog_tripped(now)
            or self.last_control["stop"]
            or self.last_control["mode"] == framing.MODE_DISABLED
        )

    def commanded_outputs(self, now=None):
        """(drive, steer) actually permitted right now -- zeroed under stop, so
        callers cannot accidentally honour a stale command."""
        if self.effective_stop(now):
            return (0, 0)
        return (self.last_control["drive_cmd"], self.last_control["steer_cmd"])

    # -- transmit scheduling ------------------------------------------------

    def telemetry_due(self, now=None):
        now = self.now_ms() if now is None else now
        if (now - self._next_telemetry_ms) < 0:
            return False
        self._next_telemetry_ms += self.telemetry_period_ms
        # Never let a stalled loop accumulate a backlog it then bursts out.
        if (now - self._next_telemetry_ms) > self.telemetry_period_ms:
            self._next_telemetry_ms = now + self.telemetry_period_ms
        return True

    def telemetry_frame(self, enc_left, enc_right, steer_fb, current_ca,
                        sensor_faults=0, extra_faults=0, now=None):
        fault_status = sensor_faults | extra_faults
        if self.watchdog_tripped(now):
            fault_status |= framing.FAULT_COMM_TIMEOUT
        return framing.encode_telemetry(
            enc_left=enc_left,
            enc_right=enc_right,
            steer_fb=steer_fb,
            current_ca=current_ca,
            fault_status=fault_status,
            cmd_age_ms=self.cmd_age_ms(now),
        )
