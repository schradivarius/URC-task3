"""
mock_link.py -- software loopback "serial cable" with no hardware involved.

Creates a pair of connected endpoints (like two ends of a real UART cable).
Each endpoint exposes .write(bytes) and .read(n) so it can be swapped in for
a real serial.Serial object -- jetson_test.py doesn't need to know or care
whether it's talking to a real port or a MockEndpoint.

Also supports optional fault injection (byte corruption, dropped frames,
latency) so the demo can prove the CRC/resync/watchdog logic actually does
something, not just that the happy path works.
"""

import queue
import random
import time


class MockEndpoint:
    def __init__(self, inbox, outbox, corrupt_rate=0.0, drop_rate=0.0, latency_s=0.0):
        self._inbox = inbox    # bytes flow in here (from the other side)
        self._outbox = outbox  # bytes we write go out here (to the other side)
        self.corrupt_rate = corrupt_rate
        self.drop_rate = drop_rate
        self.latency_s = latency_s
        self._rx_buf = bytearray()

    def write(self, data):
        if self.latency_s:
            time.sleep(self.latency_s)
        if self.drop_rate and random.random() < self.drop_rate:
            return len(data)  # pretend it was sent; it never arrives
        if self.corrupt_rate and random.random() < self.corrupt_rate and data:
            data = bytearray(data)
            idx = random.randrange(len(data))
            data[idx] ^= 0xFF  # flip a byte to simulate line noise
            data = bytes(data)
        self._outbox.put(data)
        return len(data)

    @property
    def in_waiting(self):
        self._drain()
        return len(self._rx_buf)

    def read(self, size=1):
        self._drain()
        n = min(size, len(self._rx_buf))
        out = bytes(self._rx_buf[:n])
        del self._rx_buf[:n]
        return out

    def _drain(self):
        try:
            while True:
                self._rx_buf.extend(self._inbox.get_nowait())
        except queue.Empty:
            pass

    def close(self):
        pass


def make_pair(corrupt_rate=0.0, drop_rate=0.0, latency_s=0.0):
    """Returns (jetson_side, pico_side) MockEndpoints wired to each other.
    corrupt_rate/drop_rate/latency_s apply to bytes written by either side,
    for exercising the CRC/resync/watchdog paths under simulated line noise."""
    a_to_b = queue.Queue()
    b_to_a = queue.Queue()
    jetson_side = MockEndpoint(b_to_a, a_to_b, corrupt_rate, drop_rate, latency_s)
    pico_side = MockEndpoint(a_to_b, b_to_a, corrupt_rate, drop_rate, latency_s)
    return jetson_side, pico_side
