#!/usr/bin/env python3
"""Small dependency-free regression checks for characterize_dynamics.py."""

from __future__ import annotations

import math
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parent))
from characterize_dynamics import (  # noqa: E402
    _local_polynomial_velocity,
    _reversal_proxy,
    analyze_run,
)
from analyze_stutter import parse_capture  # noqa: E402


def _row(index: int, command: float, feedback: float) -> dict[str, float]:
    timestamp = float((index + 1) * 50)
    return {
        "ts_ms": timestamp,
        "elapsed_ms": timestamp,
        "cmd_deg": command / (1024.0 / 300.0),
        "cmd_pos": command,
        "fb_pos": feedback,
        "fb_speed_raw": 10.0,
        "fb_load_raw": 100.0,
        "fb_age_ms": 20.0,
        "fb_ts_ms": timestamp,
        "stale": 0.0,
        "voltage_v": 8.0,
        "cmd_velocity_deg_s": 5.0,
        "fb_delta_counts": 1.0,
        "dt_ms": 50.0,
    }


def main() -> int:
    capture = Path(__file__).with_name(".synthetic-mlab-capture.log")
    rows_for_capture = [
        "MLAB,100,4,2,0,600,0|0|0|0|0,0|0|0|0|0,0|0|0|0|0,0|0|0|0|0,0|0|0|0|0,100|100|100|100|100,0|0|0|0|0,0|0|0|0|0,8.0,0|0|0|0|0",
        "MLAB,150,4,2,50,600,1|0|0|0|0,3|0|0|0|0,2|0|0|0|0,4|0|0|0|0,101|100|100|100|100,150|100|100|100|100,0|0|0|0|0,0|0|0|0|0,8.0,1|0|0|0|0",
    ]
    capture.write_text(
        "MLAB,ts_ms,experiment,trajectory,elapsed_ms,total_ms,cmd_deg[5],cmd_pos[5],fb_pos[5],fb_speed_raw[5],fb_load_raw[5],fb_ts_ms[5],fb_age_ms[5],fb_stale[5],servo_voltage_v,speed_cmd_raw\n"
        + "\n".join(rows_for_capture)
        + "\n",
        encoding="utf-8",
    )
    try:
        parsed = parse_capture(capture, joint=0)
        assert parsed[1]["fb_ts_ms"] == 150.0
        assert parsed[1]["speed_cmd_raw"] == 1.0
    finally:
        capture.unlink()

    rows = [_row(index, command, feedback) for index, (command, feedback) in enumerate(
        ((0, 0), (4, 2), (8, 6), (12, 10), (16, 15), (16, 16), (16, 16), (16, 16), (0, 1), (0, 0))
    )]
    metadata = {
        "run": "synthetic",
        "tier": "gentle",
        "experiment": 4,
        "joint": 2,
        "direction": "positive",
        "amplitude_deg": 3,
        "transition_ms": 200,
        "duration_ms": 600,
        "hold_ms": 100,
        "repetition": 1,
    }
    metric, _events = analyze_run(rows, metadata)
    assert metric["feedback_valid_rate"] == 1.0
    assert metric["stale_fraction"] == 0.0
    assert metric["motion_onset_latency_ms"] is not None
    assert metric["observed_peak_velocity_deg_s"] is not None
    assert all(math.isfinite(value) for value in _local_polynomial_velocity([(0.0, 0.0), (20.0, 1.0), (40.0, 2.0)]))

    reversal_rows = [_row(index, command, feedback) for index, (command, feedback) in enumerate(
        ((0, 0), (10, 5), (20, 12), (10, 12), (0, 9), (-10, 5), (-20, -1), (-10, -2), (0, 0))
    )]
    for index, row in enumerate(reversal_rows):
        row["cmd_velocity_deg_s"] = (reversal_rows[index]["cmd_pos"] - reversal_rows[index - 1]["cmd_pos"]) if index else 0.0
    assert _reversal_proxy(reversal_rows)
    print("characterize_dynamics checks passed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
