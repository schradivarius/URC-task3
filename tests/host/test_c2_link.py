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
import rover_protocol as rp  # noqa: E402


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


class TestC2StateDerivation(unittest.TestCase):
    """rp.c2_state() turns the fault word into OK / LOST / UNKNOWN.

    The tricky case is both bits set at once. The MCU only learns about C2
    from CONTROL.c2_lost, so if the Jetson has gone silent the C2 bit it is
    still reporting is stale -- it is whatever the last CONTROL frame said,
    which may be minutes old. UNKNOWN therefore has to outrank LOST, and
    equally it has to outrank OK: reporting a stale "C2 is fine" is the
    dangerous direction, because it tells the operator the link is healthy
    when in truth nobody can see it at all.
    """

    def test_no_faults_is_ok(self):
        self.assertEqual(rp.c2_state(0), rp.C2_STATE_OK)

    def test_the_c2_bit_alone_is_lost(self):
        self.assertEqual(rp.c2_state(rp.FAULT_C2_LINK_LOST), rp.C2_STATE_LOST)

    def test_a_silent_jetson_is_unknown_not_ok(self):
        # The C2 bit is CLEAR here, i.e. the last frame said "C2 fine".
        # Trusting that while the Jetson is silent is the unsafe reading.
        self.assertEqual(rp.c2_state(rp.FAULT_JETSON_HEARTBEAT_LOST),
                         rp.C2_STATE_UNKNOWN)

    def test_a_silent_jetson_outranks_a_stale_lost_bit(self):
        both = rp.FAULT_JETSON_HEARTBEAT_LOST | rp.FAULT_C2_LINK_LOST
        self.assertEqual(rp.c2_state(both), rp.C2_STATE_UNKNOWN)

    def test_unrelated_faults_do_not_affect_the_c2_state(self):
        self.assertEqual(rp.c2_state(rp.FAULT_OVER_CURRENT), rp.C2_STATE_OK)
        self.assertEqual(
            rp.c2_state(rp.FAULT_OVER_CURRENT | rp.FAULT_C2_LINK_LOST),
            rp.C2_STATE_LOST)

    def test_describe_faults_hides_the_stale_bit_and_says_unknown(self):
        both = rp.FAULT_JETSON_HEARTBEAT_LOST | rp.FAULT_C2_LINK_LOST
        text = rp.describe_faults(both)
        self.assertIn("C2_UNKNOWN", text)
        self.assertNotIn("C2_LINK_LOST", text,
                         "printed a stale C2 claim the Jetson cannot back up")

    def test_describe_faults_keeps_unrelated_faults_visible(self):
        # Suppressing the C2 bit must not swallow anything else in the word.
        text = rp.describe_faults(rp.FAULT_JETSON_HEARTBEAT_LOST
                                  | rp.FAULT_C2_LINK_LOST
                                  | rp.FAULT_OVER_CURRENT)
        self.assertIn("OVER_CURRENT", text)
        self.assertIn("C2_UNKNOWN", text)


if __name__ == "__main__":
    unittest.main()
