#!/usr/bin/env python3
"""Host-side regression checks for Motion Lab trajectory definitions.

Run from firmware/: python3 tools/motion_lab/test_trajectory.py
"""

import unittest


def linear(t: float) -> float:
    return t


def cubic_ease(t: float) -> float:
    return t * t * (3.0 - 2.0 * t)


def minimum_jerk(t: float) -> float:
    return t * t * t * (10.0 + t * (-15.0 + 6.0 * t))


class TrajectoryTest(unittest.TestCase):
    def test_endpoints(self) -> None:
        for curve in (linear, cubic_ease, minimum_jerk):
            self.assertEqual(curve(0.0), 0.0)
            self.assertEqual(curve(1.0), 1.0)

    def test_monotonic(self) -> None:
        for curve in (linear, cubic_ease, minimum_jerk):
            values = [curve(step / 100.0) for step in range(101)]
            self.assertEqual(values, sorted(values))

    def test_minimum_jerk_has_zero_endpoint_slope(self) -> None:
        epsilon = 1e-4
        self.assertLess(minimum_jerk(epsilon) / epsilon, 0.001)
        self.assertLess((1.0 - minimum_jerk(1.0 - epsilon)) / epsilon, 0.001)

    def test_deadband_does_not_discard_sub_tick_progress(self) -> None:
        # The 2 ms controller takes sub-deadband steps. It must retain those
        # internally and only suppress the corresponding bus writes.
        filtered_deg = 0.0
        sent_deg = 0.0
        for _ in range(100):
            filtered_deg += 45.0 * 0.002
            if abs(filtered_deg - sent_deg) >= 0.250:
                sent_deg = filtered_deg
        self.assertGreater(sent_deg, 1.0)


if __name__ == "__main__":
    unittest.main()
