#!/usr/bin/env python3
"""Regression checks for the offline conditioning feasibility diagnostic."""

from __future__ import annotations

import math
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parent))
import conditioning_trajectory as conditioning  # noqa: E402


def main() -> int:
    distorted = conditioning.evaluate(conditioning.ConditioningConfig(
        amplitude_deg=5.0,
        transition_ms=1000,
        max_velocity_deg_s=8.0,
        max_acceleration_deg_s2=30.0,
    ))
    assert distorted["feasible"] is False
    assert distorted["limits"]["velocity_active"] is True
    assert distorted["limits"]["acceleration_active"] is True
    assert distorted["generated"]["positive_endpoint_counts"] != 17
    assert distorted["generated"]["positive_hold_error_counts"] > 1.0
    assert distorted["generated"]["overshoot_negative_counts"] > 1.0

    selected = conditioning.evaluate(conditioning.ConditioningConfig(
        amplitude_deg=8.0,
        transition_ms=4000,
        max_velocity_deg_s=8.0,
        max_acceleration_deg_s2=30.0,
    ))
    assert selected["feasible"] is True
    assert selected["total_ms"] == 14000
    assert selected["requested"]["positive_counts"] == 8.0 * conditioning.COUNTS_PER_DEG
    assert selected["generated"]["positive_endpoint_counts"] == 27
    assert selected["generated"]["negative_endpoint_counts"] == 27
    assert selected["generated"]["positive_hold_error_counts"] <= 1.0
    assert selected["generated"]["negative_hold_error_counts"] <= 1.0
    assert selected["generated"]["overshoot_positive_counts"] == 0.0
    assert selected["generated"]["overshoot_negative_counts"] == 0.0
    assert selected["limits"]["velocity_active"] is False
    assert selected["limits"]["acceleration_active"] is False
    assert math.isclose(selected["analytical"]["reverse_peak_velocity_deg_s"], 7.5)
    assert math.isclose(
        selected["analytical"]["reverse_peak_acceleration_deg_s2"],
        10.0 * math.sqrt(3.0) / 3.0,
    )
    assert all(
        math.isfinite(float(row[field]))
        for row in conditioning.simulate(conditioning.ConditioningConfig(
            amplitude_deg=8.0,
            transition_ms=4000,
        ))
        for field in ("desired_deg", "command_deg", "command_velocity_deg_s", "command_acceleration_deg_s2")
    )
    print("conditioning trajectory checks: 2 passed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
