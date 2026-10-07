"""
test_c2_link.py -- unit tests for the Jetson-side C2 link monitor.

C2Monitor takes the current time as an argument rather than reading a clock,
so every timeout here is exercised with time the test controls. No sleeps, no
flakiness.

The distinction these tests protect, because it is the whole point of the
task: C2 LOST and C2 UNKNOWN come from two DIFFERENT timeouts.

  * C2 timeout     (here, C2Monitor, 1.0 s default)
        no base-station heartbeat -> the Jetson KNOWS C2 is down -> LOST.
        This is the state that must still let autonomy keep driving.

  * Jetson timeout (the MCU's 300 ms command watchdog, tested elsewhere)
        no CONTROL frame -> the MCU has lost its only SOURCE of C2
        information -> UNKNOWN.

Collapsing them would destroy the ability to say "C2 is down but autonomy is
fine", which is exactly the situation the 2027 course is built around.
"""

import os
import sys
import unittest

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(REPO, "host"))

import c2_link  # noqa: E402


class TestSimC2Link(unittest.TestCase):
    def test_starts_disconnected(self):
        link = c2_link.SimC2Link()
        self.assertFalse(link.is_connected())
        self.assertEqual(link.recv(), [])

    def test_connect_and_disconnect_change_what_recv_yields(self):
        link = c2_link.SimC2Link()
        link.connect()
        self.assertTrue(link.is_connected())
        self.assertEqual(len(link.recv()), 1)
        link.disconnect()
        self.assertFalse(link.is_connected())
        self.assertEqual(link.recv(), [])


class TestC2Monitor(unittest.TestCase):
    def test_reports_lost_before_anything_has_ever_arrived(self):
        # Fail-safe at startup: never having heard from the base station is
        # not the same as being connected, and must not read as connected.
        mon = c2_link.C2Monitor()
        self.assertTrue(mon.bool_no_connection(0.0))
        self.assertTrue(mon.bool_no_connection(1000.0))

    def test_a_fresh_update_clears_the_lost_state(self):
        mon = c2_link.C2Monitor(C2_timeout_s=1.0)
        mon.on_link_update(10.0)
        self.assertFalse(mon.bool_no_connection(10.0))
        self.assertFalse(mon.bool_no_connection(10.5))

    def test_the_timeout_boundary_is_exact(self):
        mon = c2_link.C2Monitor(C2_timeout_s=1.0)
        mon.on_link_update(10.0)
        self.assertFalse(mon.bool_no_connection(10.999), "tripped early")
        self.assertTrue(mon.bool_no_connection(11.0), "did not trip on time")
        self.assertTrue(mon.bool_no_connection(30.0), "recovered on its own")

    def test_a_later_update_restores_the_connection(self):
        # The base station coming back must clear the state without a restart.
        mon = c2_link.C2Monitor(C2_timeout_s=1.0)
        mon.on_link_update(10.0)
        self.assertTrue(mon.bool_no_connection(12.0))
        mon.on_link_update(12.0)
        self.assertFalse(mon.bool_no_connection(12.0))

    def test_the_timeout_is_configurable(self):
        # 1.0 s is an initial design value for testing, documented as tunable.
        # It must actually be a parameter, not a constant in disguise.
        fast = c2_link.C2Monitor(C2_timeout_s=0.2)
        fast.on_link_update(0.0)
        self.assertTrue(fast.bool_no_connection(0.25))
        slow = c2_link.C2Monitor(C2_timeout_s=5.0)
        slow.on_link_update(0.0)
        self.assertFalse(slow.bool_no_connection(0.25))


class TestPollC2Lost(unittest.TestCase):
    """poll_c2_lost is what the harness actually calls each control cycle."""

    def test_a_connected_link_never_reports_lost(self):
        link = c2_link.SimC2Link()
        link.connect()
        mon = c2_link.C2Monitor(C2_timeout_s=1.0)
        for t in (0.0, 0.5, 5.0, 60.0):
            self.assertFalse(c2_link.poll_c2_lost(link, mon, t),
                             "reported lost at t=%s while connected" % t)

    def test_a_dropped_link_reports_lost_after_the_timeout(self):
        link = c2_link.SimC2Link()
        link.connect()
        mon = c2_link.C2Monitor(C2_timeout_s=1.0)
        self.assertFalse(c2_link.poll_c2_lost(link, mon, 0.0))

        link.disconnect()
        self.assertFalse(c2_link.poll_c2_lost(link, mon, 0.5),
                         "reported lost before the timeout elapsed")
        self.assertTrue(c2_link.poll_c2_lost(link, mon, 1.0),
                        "did not report lost after the timeout")

    def test_reconnecting_clears_it(self):
        link = c2_link.SimC2Link()
        link.connect()
        mon = c2_link.C2Monitor(C2_timeout_s=1.0)
        c2_link.poll_c2_lost(link, mon, 0.0)
        link.disconnect()
        self.assertTrue(c2_link.poll_c2_lost(link, mon, 2.0))
        link.connect()
        self.assertFalse(c2_link.poll_c2_lost(link, mon, 2.0),
                         "stayed lost after the link came back")

    def test_a_link_that_never_connects_reports_lost_from_the_start(self):
        link = c2_link.SimC2Link()          # never connected
        mon = c2_link.C2Monitor()
        self.assertTrue(c2_link.poll_c2_lost(link, mon, 0.0))


if __name__ == "__main__":
    unittest.main()
