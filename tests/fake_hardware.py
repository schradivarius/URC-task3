"""
tests/fake_hardware.py -- Minimal stand-ins for the CircuitPython hardware
modules, so feather_main.py (the file that actually gets flashed) can be
imported and exercised under CPython in CI.

Without this, the firmware would be the one file no test ever touches -- which
is exactly the file where a bug is most expensive. These fakes implement only
what feather_main.py actually uses.

Call install() BEFORE importing feather_main.
"""

import sys
import types


class FakePin:
    def __init__(self, name):
        self.name = name

    def __repr__(self):
        return "FakePin(%s)" % self.name


class FakeDigitalInOut:
    def __init__(self, pin):
        self.pin = pin
        self.direction = None
        self.value = False
        self.toggles = 0

    def __setattr__(self, key, val):
        if key == "value" and "value" in self.__dict__ and self.__dict__["value"] != val:
            self.__dict__["toggles"] = self.__dict__.get("toggles", 0) + 1
        object.__setattr__(self, key, val)


class FakeUART:
    def __init__(self, tx, rx, baudrate=115200, timeout=0):
        self.tx, self.rx, self.baudrate, self.timeout = tx, rx, baudrate, timeout
        self.written = bytearray()
        self._rx = bytearray()

    @property
    def in_waiting(self):
        return len(self._rx)

    def read(self, n):
        out = bytes(self._rx[:n])
        del self._rx[:n]
        return out

    def write(self, data):
        self.written.extend(data)
        return len(data)

    def inject(self, data):
        """Test helper: pretend these bytes arrived from the other end."""
        self._rx.extend(data)


class FakeWatchdog:
    def __init__(self):
        self.timeout = None
        self.mode = None
        self.feeds = 0

    def feed(self):
        self.feeds += 1


def install(usb_data=None, watchdog_available=True, reset_reason="POWER_ON"):
    """Insert the fake modules into sys.modules. Returns the fake watchdog so
    tests can assert it was configured and fed.

    reset_reason models microcontroller.cpu.reset_reason: pass "WATCHDOG" to
    simulate booting after a watchdog reset, or None to simulate a port that
    does not expose one.
    """
    board = types.ModuleType("board")
    board.LED = FakePin("GP13")   # Feather RP2040 RFM9x onboard red LED
    board.TX = FakePin("GP0")
    board.RX = FakePin("GP1")

    digitalio = types.ModuleType("digitalio")
    digitalio.DigitalInOut = FakeDigitalInOut
    digitalio.Direction = types.SimpleNamespace(OUTPUT="output", INPUT="input")

    busio = types.ModuleType("busio")
    busio.UART = FakeUART

    fake_wd = FakeWatchdog()
    microcontroller = types.ModuleType("microcontroller")
    microcontroller.ResetReason = types.SimpleNamespace(
        POWER_ON="POWER_ON", WATCHDOG="WATCHDOG", SOFTWARE="SOFTWARE",
        RESET_PIN="RESET_PIN", BROWNOUT="BROWNOUT")
    if reset_reason is not None:
        microcontroller.cpu = types.SimpleNamespace(reset_reason=reset_reason)
    watchdog_mod = types.ModuleType("watchdog")
    if watchdog_available:
        microcontroller.watchdog = fake_wd
        watchdog_mod.WatchDogMode = types.SimpleNamespace(RESET="RESET", RAISE="RAISE")
    else:
        watchdog_mod.WatchDogMode = types.SimpleNamespace(RESET="RESET")

    usb_cdc = types.ModuleType("usb_cdc")
    usb_cdc.data = usb_data
    usb_cdc.console = None

    for mod in (board, digitalio, busio, microcontroller, watchdog_mod, usb_cdc):
        sys.modules[mod.__name__] = mod
    return fake_wd


def uninstall():
    for name in ("board", "digitalio", "busio", "microcontroller", "watchdog", "usb_cdc"):
        sys.modules.pop(name, None)
