#!/usr/bin/env python3
"""Run a small, repeatable joint-0 direction-speed characterization matrix.

This is deliberately a bounded diagnostic, not an auto-calibration framework.
It keeps the 10-degree minimum-jerk command and duration fixed, varies only the
Motion Lab velocity limit, and writes raw captures plus a compact JSON summary.
Run with the ESP-IDF Python environment so capture_serial.py can import pyserial.
"""

from __future__ import annotations

import argparse
import json
import math
import subprocess
import sys
from pathlib import Path


def parse_rows(path: Path) -> list[dict[str, float]]:
    rows: list[dict[str, float]] = []
    for line in path.read_text(errors="replace").splitlines():
        if not line.startswith("MLAB,") or line.startswith("MLAB,ts_ms,"):
            continue
        fields = line.split(",")
        if len(fields) < 16:
            continue
        try:
            rows.append(
                {
                    "elapsed_ms": float(fields[4]),
                    "cmd_deg": float(fields[6].split("|")[0]),
                    "cmd_pos": float(fields[7].split("|")[0]),
                    "fb_pos": float(fields[8].split("|")[0]),
                    "fb_load": float(fields[10].split("|")[0]),
                    "fb_age_ms": float(fields[12].split("|")[0]),
                    "stale": float(fields[13].split("|")[0]),
                    "voltage_v": float(fields[14]),
                    "speed_cmd_raw": float(fields[15]),
                }
            )
        except (IndexError, ValueError):
            continue
    return rows


def summarize(rows: list[dict[str, float]], velocity: int, repetition: int) -> dict:
    if len(rows) < 2:
        return {"velocity_deg_s": velocity, "repetition": repetition, "rows": len(rows)}
    baseline_cmd_deg = rows[0]["cmd_deg"]
    errors = [row["fb_pos"] - row["cmd_pos"] for row in rows]
    jumps: list[dict[str, float]] = []
    for previous, current in zip(rows, rows[1:]):
        command_delta = current["cmd_pos"] - previous["cmd_pos"]
        feedback_delta = current["fb_pos"] - previous["fb_pos"]
        # Analyze the outbound (positive/CW) half only.  The return half is
        # the known smoother CCW direction and would dilute the symptom.
        # Require command motion in the same direction and a feedback jump
        # materially larger than the commanded sample-to-sample step; this
        # avoids counting ordinary encoder quantization as a stutter event.
        if (current["elapsed_ms"] / 5000.0 <= 0.5 and command_delta >= 0.5 and
                feedback_delta >= max(4.0, command_delta * 2.0)):
            jumps.append(
                {
                    "elapsed_ms": current["elapsed_ms"],
                    "elapsed_fraction": current["elapsed_ms"] / 5000.0,
                    "command_offset_deg": current["cmd_deg"] - baseline_cmd_deg,
                    "command_delta_counts": command_delta,
                    "delta_counts": feedback_delta,
                    "load_raw": current["fb_load"],
                }
            )
    return {
        "velocity_deg_s": velocity,
        "repetition": repetition,
        "rows": len(rows),
        "cmd_min_deg": min(row["cmd_deg"] for row in rows),
        "cmd_max_deg": max(row["cmd_deg"] for row in rows),
        "baseline_cmd_deg": baseline_cmd_deg,
        "feedback_min_counts": min(row["fb_pos"] for row in rows),
        "feedback_max_counts": max(row["fb_pos"] for row in rows),
        "tracking_error_median_counts": sorted(abs(error) for error in errors)[len(errors) // 2],
        "tracking_error_max_counts": max(abs(error) for error in errors),
        "load_median_raw": sorted(row["fb_load"] for row in rows)[len(rows) // 2],
        "load_max_raw": max(row["fb_load"] for row in rows),
        "voltage_min_v": min(row["voltage_v"] for row in rows),
        "voltage_max_v": max(row["voltage_v"] for row in rows),
        "stale_rows": sum(1 for row in rows if row["stale"] != 0),
        "jumps": jumps,
    }


def classify(summaries: list[dict]) -> dict:
    events = [jump for summary in summaries for jump in summary.get("jumps", [])]
    if not events:
        return {"classification": "no-jumps-detected", "event_count": 0}
    positions = [event["command_offset_deg"] for event in events]
    times = [event["elapsed_fraction"] for event in events]
    elapsed_ms = [event["elapsed_ms"] for event in events]
    magnitudes = [abs(event["delta_counts"]) for event in events]
    mean_position = sum(positions) / len(positions)
    position_spread = math.sqrt(sum((value - mean_position) ** 2 for value in positions) / len(positions))
    mean_time = sum(times) / len(times)
    time_spread = math.sqrt(sum((value - mean_time) ** 2 for value in times) / len(times))
    mean_elapsed_ms = sum(elapsed_ms) / len(elapsed_ms)
    elapsed_spread_ms = math.sqrt(
        sum((value - mean_elapsed_ms) ** 2 for value in elapsed_ms) / len(elapsed_ms)
    )
    by_speed: dict[str, dict[str, float]] = {}
    for summary in summaries:
        speed = str(summary["velocity_deg_s"])
        by_speed.setdefault(speed, {"runs": 0, "jump_count": 0, "jump_magnitude_sum": 0.0})
        by_speed[speed]["runs"] += 1
        by_speed[speed]["jump_count"] += len(summary.get("jumps", []))
        by_speed[speed]["jump_magnitude_sum"] += sum(abs(j["delta_counts"]) for j in summary.get("jumps", []))
    for value in by_speed.values():
        value["jump_count_per_run"] = value["jump_count"] / value["runs"]
        value["jump_magnitude_mean"] = value["jump_magnitude_sum"] / max(value["jump_count"], 1)
    speed_counts = [value["jump_count_per_run"] for value in by_speed.values()]
    speed_span = max(speed_counts) - min(speed_counts) if speed_counts else 0.0
    speed_means = [value["jump_count_per_run"] for value in by_speed.values()]
    speed_trend = all(left <= right for left, right in zip(speed_means, speed_means[1:]))
    if position_spread <= 1.5:
        classification = "position-locked-candidate"
    elif speed_trend and speed_span >= max(1.0, max(speed_counts, default=0.0) * 0.25):
        classification = "velocity-dependent-candidate"
    elif elapsed_spread_ms <= 250.0:
        classification = "time-frequency-locked-candidate"
    else:
        classification = "mixed-or-under-sampled"
    return {
        "classification": classification,
        "event_count": len(events),
        "command_offset_mean_deg": mean_position,
        "command_offset_spread_deg": position_spread,
        "elapsed_fraction_mean": mean_time,
        "elapsed_fraction_spread": time_spread,
        "elapsed_ms_mean": mean_elapsed_ms,
        "elapsed_ms_spread": elapsed_spread_ms,
        "jump_rate_span_per_run": speed_span,
        "jump_rate_is_monotonic_with_speed": speed_trend,
        "jump_magnitude_mean_counts": sum(magnitudes) / len(magnitudes),
        "per_speed": by_speed,
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--repetitions", type=int, default=3)
    parser.add_argument(
        "--analyze-only",
        action="store_true",
        help="reuse existing raw logs in output-dir without touching the hardware",
    )
    args = parser.parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)
    capture = Path(__file__).with_name("capture_serial.py")
    summaries: list[dict] = []
    for velocity in (8, 15, 30):
        for repetition in range(1, args.repetitions + 1):
            output = args.output_dir / f"stutter_cw_v{velocity}_r{repetition}.log"
            command = f"mlab run 0 2 0 10 5000 0 {velocity} 60 0"
            if not args.analyze_only:
                subprocess.run(
                    [
                        sys.executable,
                        str(capture),
                        "--port",
                        args.port,
                        "--command",
                        command,
                        "--duration-ms",
                        "5000",
                        "--output",
                        str(output),
                        "--ready-timeout-s",
                        # The ready banner is emitted only at boot and is not
                        # repeated for every capture.  Keep a short grace period
                        # so a live board is not delayed by the full timeout.
                        "0.5",
                        "--tail-s",
                        "1.5",
                    ],
                    check=True,
                )
            rows = parse_rows(output)
            if not rows:
                raise RuntimeError(
                    f"capture produced no telemetry rows: {output}; "
                    "check the serial session before interpreting the matrix"
                )
            summaries.append(summarize(rows, velocity, repetition))
    report = {
        "protocol": {
            "joint": 0,
            "direction": "positive / physical clockwise",
            "amplitude_deg": 10,
            "trajectory": "minimum-jerk",
            "duration_ms": 5000,
            "max_acceleration_deg_s2": 60,
            "velocities_deg_s": [8, 15, 30],
            "repetitions": args.repetitions,
        },
        "runs": summaries,
        "classification": classify(summaries),
    }
    report_path = args.output_dir / "stutter_characterization_report.json"
    report_path.write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report["classification"], indent=2))
    print(f"report: {report_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
