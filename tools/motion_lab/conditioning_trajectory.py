#!/usr/bin/env python3
"""Offline feasibility check for the Motion Lab reversal conditioning path.

The implementation mirrors ``main/boards/arm/motion_lab.cc`` for the
minimum-jerk reversal sweep and its bounded-acceleration command integrator.
It never opens a serial port and is intended to catch a distorted reference
before a supervised hardware run.
"""

from __future__ import annotations

import argparse
import json
import math
from dataclasses import dataclass, asdict


COUNTS_PER_DEG = 1024.0 / 300.0


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def minimum_jerk(normalized: float) -> float:
    """Return the standard quintic 0..1 minimum-jerk position curve."""

    t = clamp(float(normalized), 0.0, 1.0)
    return t * t * t * (10.0 + t * (-15.0 + 6.0 * t))


def reversal_reference(elapsed_ms: int, transition_ms: int, hold_ms: int) -> float:
    """Return the signed -1..1 reversal shape used by the firmware."""

    transition_ms = int(transition_ms)
    hold_ms = int(hold_ms)
    first_hold_end = transition_ms + hold_ms
    reverse_end = first_hold_end + transition_ms
    second_hold_end = reverse_end + hold_ms
    finish = second_hold_end + transition_ms
    if elapsed_ms < transition_ms:
        return minimum_jerk(elapsed_ms / transition_ms)
    if elapsed_ms < first_hold_end:
        return 1.0
    if elapsed_ms < reverse_end:
        return 1.0 - 2.0 * minimum_jerk((elapsed_ms - first_hold_end) / transition_ms)
    if elapsed_ms < second_hold_end:
        return -1.0
    if elapsed_ms < finish:
        return -1.0 + minimum_jerk((elapsed_ms - second_hold_end) / transition_ms)
    return 0.0


@dataclass(frozen=True)
class ConditioningConfig:
    amplitude_deg: float = 5.0
    transition_ms: int = 2500
    hold_ms: int = 1000
    max_velocity_deg_s: float = 8.0
    max_acceleration_deg_s2: float = 30.0
    dt_ms: int = 2

    @property
    def total_ms(self) -> int:
        return self.transition_ms * 3 + self.hold_ms * 2


def analytical_peaks(config: ConditioningConfig) -> dict[str, float]:
    """Return ideal minimum-jerk peaks for the 5° and 10° legs."""

    # max |d s/dt| for 10t^3-15t^4+6t^5 is 1.875.
    # max |d²s/dt²| is 10*sqrt(3)/3 ~= 5.7735027.
    peak_accel_shape = 10.0 * math.sqrt(3.0) / 3.0
    return {
        "positive_peak_velocity_deg_s": 1.875 * abs(config.amplitude_deg) / (config.transition_ms / 1000.0),
        "reverse_peak_velocity_deg_s": 1.875 * (2.0 * abs(config.amplitude_deg)) / (config.transition_ms / 1000.0),
        "positive_peak_acceleration_deg_s2": peak_accel_shape * abs(config.amplitude_deg) / (config.transition_ms / 1000.0) ** 2,
        "reverse_peak_acceleration_deg_s2": peak_accel_shape * (2.0 * abs(config.amplitude_deg)) / (config.transition_ms / 1000.0) ** 2,
    }


def simulate(config: ConditioningConfig) -> list[dict[str, float]]:
    """Simulate the firmware's internal command position and velocity."""

    if config.dt_ms <= 0 or config.transition_ms <= 0:
        raise ValueError("dt_ms and transition_ms must be positive")
    dt_s = config.dt_ms / 1000.0
    command_deg = 0.0
    command_velocity = 0.0
    rows: list[dict[str, float]] = []
    steps = math.ceil(config.total_ms / config.dt_ms)
    for index in range(steps):
        elapsed_ms = index * config.dt_ms
        if elapsed_ms >= config.total_ms:
            break
        desired = config.amplitude_deg * reversal_reference(
            elapsed_ms, config.transition_ms, config.hold_ms
        )
        velocity_target = clamp(
            (desired - command_deg) / dt_s,
            -config.max_velocity_deg_s,
            config.max_velocity_deg_s,
        )
        max_velocity_step = config.max_acceleration_deg_s2 * dt_s
        command_velocity += clamp(
            velocity_target - command_velocity,
            -max_velocity_step,
            max_velocity_step,
        )
        command_deg += command_velocity * dt_s
        rows.append({
            "elapsed_ms": float(elapsed_ms),
            "desired_deg": desired,
            "command_deg": command_deg,
            "command_velocity_deg_s": command_velocity,
        })
    for previous, current in zip(rows, rows[1:]):
        current["command_acceleration_deg_s2"] = (
            current["command_velocity_deg_s"] - previous["command_velocity_deg_s"]
        ) / dt_s
    if rows:
        rows[0]["command_acceleration_deg_s2"] = 0.0
    return rows


def _segment(rows: list[dict[str, float]], start_ms: int, end_ms: int) -> list[dict[str, float]]:
    return [row for row in rows if start_ms <= row["elapsed_ms"] < end_ms]


def evaluate(config: ConditioningConfig) -> dict:
    """Evaluate endpoint fidelity, overshoot, and active limits."""

    rows = simulate(config)
    peaks = analytical_peaks(config)
    positive_hold = _segment(rows, config.transition_ms, config.transition_ms + config.hold_ms)
    negative_hold_start = config.transition_ms * 2 + config.hold_ms
    negative_hold = _segment(rows, negative_hold_start, negative_hold_start + config.hold_ms)
    positive_endpoint = max((row["command_deg"] for row in positive_hold), default=0.0)
    negative_endpoint = min((row["command_deg"] for row in negative_hold), default=0.0)
    positive_hold_error = max(
        (abs(row["command_deg"] - config.amplitude_deg) for row in positive_hold),
        default=float("inf"),
    )
    negative_hold_error = max(
        (abs(row["command_deg"] + config.amplitude_deg) for row in negative_hold),
        default=float("inf"),
    )
    max_command = max((row["command_deg"] for row in rows), default=0.0)
    min_command = min((row["command_deg"] for row in rows), default=0.0)
    peak_velocity = max((abs(row["command_velocity_deg_s"]) for row in rows), default=0.0)
    peak_acceleration = max((abs(row.get("command_acceleration_deg_s2", 0.0)) for row in rows), default=0.0)
    command_count_positive = math.trunc(positive_endpoint * COUNTS_PER_DEG)
    command_count_negative = math.trunc(abs(negative_endpoint) * COUNTS_PER_DEG)
    endpoint_tolerance_counts = max(1.0, COUNTS_PER_DEG * 0.10)
    return {
        "config": asdict(config),
        "total_ms": config.total_ms,
        "requested": {
            "positive_deg": abs(config.amplitude_deg),
            "negative_deg": abs(config.amplitude_deg),
            "positive_counts": abs(config.amplitude_deg) * COUNTS_PER_DEG,
            "negative_counts": abs(config.amplitude_deg) * COUNTS_PER_DEG,
        },
        "generated": {
            "positive_endpoint_deg": positive_endpoint,
            "negative_endpoint_deg": abs(negative_endpoint),
            "positive_endpoint_counts": command_count_positive,
            "negative_endpoint_counts": command_count_negative,
            "positive_hold_error_deg": positive_hold_error,
            "negative_hold_error_deg": negative_hold_error,
            "positive_hold_error_counts": positive_hold_error * COUNTS_PER_DEG,
            "negative_hold_error_counts": negative_hold_error * COUNTS_PER_DEG,
            "overshoot_positive_deg": max(0.0, max_command - abs(config.amplitude_deg)),
            "overshoot_negative_deg": max(0.0, abs(min_command) - abs(config.amplitude_deg)),
            "overshoot_positive_counts": max(0.0, max_command - abs(config.amplitude_deg)) * COUNTS_PER_DEG,
            "overshoot_negative_counts": max(0.0, abs(min_command) - abs(config.amplitude_deg)) * COUNTS_PER_DEG,
        },
        "analytical": peaks,
        "simulated_peak_velocity_deg_s": peak_velocity,
        "simulated_peak_acceleration_deg_s2": peak_acceleration,
        "limits": {
            "velocity_active": peaks["reverse_peak_velocity_deg_s"] > config.max_velocity_deg_s + 1e-9,
            "acceleration_active": peaks["reverse_peak_acceleration_deg_s2"] > config.max_acceleration_deg_s2 + 1e-9,
            "simulated_velocity_active": peak_velocity >= config.max_velocity_deg_s - 1e-6,
            "simulated_acceleration_active": peak_acceleration >= config.max_acceleration_deg_s2 - 1e-6,
        },
        "feasible": (
            peaks["reverse_peak_velocity_deg_s"] <= config.max_velocity_deg_s + 1e-9
            and peaks["reverse_peak_acceleration_deg_s2"] <= config.max_acceleration_deg_s2 + 1e-9
            and positive_hold_error * COUNTS_PER_DEG <= endpoint_tolerance_counts
            and negative_hold_error * COUNTS_PER_DEG <= endpoint_tolerance_counts
            and max(0.0, max_command - abs(config.amplitude_deg)) * COUNTS_PER_DEG <= endpoint_tolerance_counts
            and max(0.0, abs(min_command) - abs(config.amplitude_deg)) * COUNTS_PER_DEG <= endpoint_tolerance_counts
        ),
        "endpoint_tolerance_counts": endpoint_tolerance_counts,
    }


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--transition-ms", type=int, default=2500)
    parser.add_argument("--hold-ms", type=int, default=1000)
    parser.add_argument("--amplitude-deg", type=float, default=5.0)
    parser.add_argument("--max-velocity-deg-s", type=float, default=8.0)
    parser.add_argument("--max-acceleration-deg-s2", type=float, default=30.0)
    parser.add_argument("--dt-ms", type=int, default=2)
    parser.add_argument("--json", action="store_true", help="emit one JSON document")
    return parser


def main() -> int:
    args = _parser().parse_args()
    result = evaluate(ConditioningConfig(
        amplitude_deg=args.amplitude_deg,
        transition_ms=args.transition_ms,
        hold_ms=args.hold_ms,
        max_velocity_deg_s=args.max_velocity_deg_s,
        max_acceleration_deg_s2=args.max_acceleration_deg_s2,
        dt_ms=args.dt_ms,
    ))
    if args.json:
        print(json.dumps(result, indent=2))
    else:
        print(f"conditioning {'FEASIBLE' if result['feasible'] else 'DISTORTED'}")
        print(f"  total duration: {result['total_ms']} ms")
        print(f"  requested: +/-{result['requested']['positive_deg']:.3f} deg ({result['requested']['positive_counts']:.3f} counts)")
        generated = result["generated"]
        print(f"  generated endpoints: +{generated['positive_endpoint_deg']:.4f} / -{generated['negative_endpoint_deg']:.4f} deg")
        print(f"  generated endpoint counts: +{generated['positive_endpoint_counts']} / -{generated['negative_endpoint_counts']}")
        print(f"  peak velocity: {result['simulated_peak_velocity_deg_s']:.4f} deg/s")
        print(f"  peak acceleration: {result['simulated_peak_acceleration_deg_s2']:.4f} deg/s^2")
        print(f"  hold error: +{generated['positive_hold_error_counts']:.4f} / -{generated['negative_hold_error_counts']:.4f} counts")
        print(f"  overshoot: +{generated['overshoot_positive_counts']:.4f} / -{generated['overshoot_negative_counts']:.4f} counts")
        print(f"  limits active: velocity={result['limits']['velocity_active']} acceleration={result['limits']['acceleration_active']}")
    return 0 if result["feasible"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
