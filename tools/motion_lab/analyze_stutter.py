#!/usr/bin/env python3
"""Analyze Motion Lab UART captures for low-cost servo motion-quality events.

The analyzer is intentionally dependency-free: it emits JSON/CSV/JSONL and
small SVG plots, so a calibration run is inspectable on a fresh WSL install.
It consumes the manifest written by auto_calibrate.py and never talks to the
hardware.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import statistics
from pathlib import Path


def _array(value: str, cast=float) -> list:
    return [cast(part) for part in value.split("|")]


def parse_capture(path: Path, joint: int = 0) -> list[dict[str, float]]:
    rows: list[dict[str, float]] = []
    for line in path.read_text(errors="replace").splitlines():
        if not line.startswith("MLAB,") or line.startswith("MLAB,ts_ms,"):
            continue
        fields = line.split(",")
        if len(fields) < 16:
            continue
        try:
            rows.append({
                "ts_ms": float(fields[1]),
                "elapsed_ms": float(fields[4]),
                "cmd_deg": _array(fields[6])[joint],
                "cmd_pos": _array(fields[7])[joint],
                "fb_pos": _array(fields[8])[joint],
                "fb_speed_raw": _array(fields[9])[joint],
                "fb_load_raw": _array(fields[10])[joint],
                "fb_ts_ms": _array(fields[11])[joint],
                "fb_age_ms": _array(fields[12])[joint],
                "stale": _array(fields[13], int)[joint],
                "voltage_v": float(fields[14]),
                "speed_cmd_raw": _array(fields[15])[joint],
            })
        except (IndexError, ValueError):
            continue
    if len(rows) < 2:
        raise ValueError(f"{path}: expected at least two telemetry rows")
    baseline = rows[0]["cmd_deg"]
    for previous, current in zip(rows, rows[1:]):
        dt_ms = max(current["ts_ms"] - previous["ts_ms"], 1.0)
        current["cmd_offset_deg"] = current["cmd_deg"] - baseline
        current["cmd_velocity_deg_s"] = (current["cmd_deg"] - previous["cmd_deg"]) / (dt_ms / 1000.0)
        current["fb_delta_counts"] = current["fb_pos"] - previous["fb_pos"]
        current["dt_ms"] = dt_ms
    rows[0]["cmd_offset_deg"] = rows[0]["cmd_deg"] - baseline
    rows[0]["cmd_velocity_deg_s"] = 0.0
    rows[0]["fb_delta_counts"] = 0.0
    rows[0]["dt_ms"] = 0.0
    return rows


def resample_fixed_dt(rows: list[dict[str, float]], dt_ms: float = 50.0) -> list[dict[str, float]]:
    """Linearly resample comparable signals onto a fixed grid.

    This follows BAM's preprocessing discipline. Raw rows remain the source for
    event timing so interpolation cannot erase a short dwell/jump.
    """
    if len(rows) < 2:
        return rows[:]
    start = rows[0]["ts_ms"]
    end = rows[-1]["ts_ms"]
    fields = ("cmd_deg", "cmd_pos", "fb_pos", "fb_speed_raw", "fb_load_raw", "fb_age_ms", "voltage_v")
    normalized: list[dict[str, float]] = []
    source_index = 0
    sample_ts = start
    while sample_ts <= end + 0.001:
        while source_index + 1 < len(rows) and rows[source_index + 1]["ts_ms"] < sample_ts:
            source_index += 1
        left = rows[source_index]
        right = rows[min(source_index + 1, len(rows) - 1)]
        span = max(right["ts_ms"] - left["ts_ms"], 1.0)
        alpha = min(1.0, max(0.0, (sample_ts - left["ts_ms"]) / span))
        row = {"ts_ms": sample_ts, "elapsed_ms": sample_ts - start, "stale": int(left["stale"])}
        for field in fields:
            row[field] = left[field] + alpha * (right[field] - left[field])
        normalized.append(row)
        sample_ts += dt_ms
    for previous, current in zip(normalized, normalized[1:]):
        current["cmd_offset_deg"] = current["cmd_deg"] - normalized[0]["cmd_deg"]
        current["cmd_velocity_deg_s"] = (current["cmd_deg"] - previous["cmd_deg"]) / (dt_ms / 1000.0)
        current["fb_delta_counts"] = current["fb_pos"] - previous["fb_pos"]
        current["dt_ms"] = dt_ms
    if normalized:
        normalized[0]["cmd_offset_deg"] = 0.0
        normalized[0]["cmd_velocity_deg_s"] = 0.0
        normalized[0]["fb_delta_counts"] = 0.0
        normalized[0]["dt_ms"] = 0.0
    return normalized


def reversal_metrics(rows: list[dict[str, float]]) -> tuple[list[float], list[float]]:
    """Estimate reversal response delay and command travel before feedback moves.

    The latter is a backlash-style proxy, not a mechanical free-play claim: the
    arm has no external force fixture and the feedback itself is quantized.
    """
    delays_ms: list[float] = []
    backlash_deg: list[float] = []
    for index in range(1, len(rows)):
        previous = rows[index - 1]
        current = rows[index]
        old_velocity = previous["cmd_velocity_deg_s"]
        new_velocity = current["cmd_velocity_deg_s"]
        if abs(old_velocity) < 0.5 or abs(new_velocity) < 0.5 or old_velocity * new_velocity >= 0:
            continue
        new_sign = 1.0 if new_velocity > 0 else -1.0
        command_travel_counts = 0.0
        reversal_feedback_pos = current["fb_pos"]
        response_ts: float | None = None
        for later_index in range(index, len(rows)):
            later = rows[later_index]
            prior_later = rows[max(index, later_index - 1)]
            command_travel_counts += abs(later["cmd_pos"] - prior_later["cmd_pos"])
            if new_sign * (later["fb_pos"] - reversal_feedback_pos) >= 2.0:
                response_ts = later["ts_ms"]
                break
        if response_ts is not None:
            delays_ms.append(max(0.0, response_ts - current["ts_ms"]))
            backlash_deg.append(command_travel_counts * 300.0 / 1024.0)
    return delays_ms, backlash_deg


def _percentile(values: list[float], fraction: float) -> float:
    if not values:
        return 0.0
    ordered = sorted(values)
    index = min(len(ordered) - 1, max(0, int(round((len(ordered) - 1) * fraction))))
    return ordered[index]


def analyze_run(rows: list[dict[str, float]], metadata: dict) -> tuple[dict, list[dict]]:
    amplitude = float(metadata["amplitude_deg"])
    direction = "CW" if amplitude > 0 else "CCW"
    sign = 1.0 if amplitude > 0 else -1.0
    events: list[dict] = []
    dwell_start: float | None = None
    dwell_ms = 0.0
    for previous, current in zip(rows, rows[1:]):
        command_motion = sign * current["cmd_velocity_deg_s"] > 0.35
        feedback_motion = sign * current["fb_delta_counts"]
        if command_motion and abs(current["fb_delta_counts"]) <= 1.0:
            if dwell_start is None:
                dwell_start = previous["ts_ms"]
            dwell_ms = current["ts_ms"] - dwell_start
            continue
        if (dwell_start is not None and command_motion and feedback_motion >= 4.0):
            events.append({
                "type": "dwell_jump",
                "direction": direction,
                "timestamp_ms": current["ts_ms"],
                "position_deg": current["cmd_offset_deg"],
                "commanded_velocity_deg_s": current["cmd_velocity_deg_s"],
                "measured_velocity_raw": current["fb_speed_raw"],
                "load_raw": current["fb_load_raw"],
                "dwell_ms": max(0.0, dwell_ms),
                "jump_counts": feedback_motion,
            })
        dwell_start = None
        dwell_ms = 0.0

    normalized = resample_fixed_dt(rows)
    command_errors = [row["fb_pos"] - row["cmd_pos"] for row in normalized]
    moving = [row for row in normalized if abs(row["cmd_velocity_deg_s"]) > 0.35]
    hold_rows = [row for row in normalized if abs(row["cmd_velocity_deg_s"]) <= 0.35]
    hold_positions = [row["fb_pos"] for row in hold_rows]
    endpoint = max(abs(amplitude), 1.0)
    signed_feedback = [sign * (row["fb_pos"] - rows[0]["fb_pos"]) / (1024.0 / 300.0) for row in rows]
    overshoot_deg = max(0.0, max(signed_feedback, default=0.0) - endpoint)
    jitter_counts = 0.0
    if hold_positions:
        jitter_counts = max(hold_positions) - min(hold_positions)
    reversal_delays_ms, backlash_deg = reversal_metrics(rows)
    metric = {
        "run": metadata["run"],
        "candidate": metadata.get("candidate", ""),
        "servo_profile": metadata.get("servo_profile", "factory"),
        "runtime_profile": metadata.get("runtime_profile", "off"),
        "repetition": int(metadata.get("repetition", 0)),
        "direction": direction,
        "amplitude_deg": amplitude,
        "duration_ms": int(metadata["duration_ms"]),
        "max_velocity_deg_s": float(metadata["max_velocity_deg_s"]),
        "rows": len(rows),
        "events": len(events),
        "dwell_ms_total": sum(event["dwell_ms"] for event in events),
        "dwell_ms_p95": _percentile([event["dwell_ms"] for event in events], 0.95),
        "jump_counts_total": sum(event["jump_counts"] for event in events),
        "jump_counts_max": max((event["jump_counts"] for event in events), default=0.0),
        "tracking_error_abs_median_counts": statistics.median(abs(value) for value in command_errors),
        "tracking_error_abs_p95_counts": _percentile([abs(value) for value in command_errors], 0.95),
        "static_hold_jitter_counts": jitter_counts,
        "endpoint_overshoot_deg": overshoot_deg,
        "moving_load_median_raw": statistics.median(row["fb_load_raw"] for row in moving) if moving else 0.0,
        "voltage_min_v": min(row["voltage_v"] for row in rows),
        "voltage_max_v": max(row["voltage_v"] for row in rows),
        "stale_rows": sum(int(row["stale"]) for row in rows),
        "feedback_age_p95_ms": _percentile([row["fb_age_ms"] for row in rows], 0.95),
        "fixed_dt_ms": 50.0,
        "reversal_count": len(reversal_delays_ms),
        "reversal_delay_median_ms": statistics.median(reversal_delays_ms) if reversal_delays_ms else 0.0,
        "reversal_command_travel_median_deg": statistics.median(backlash_deg) if backlash_deg else 0.0,
    }
    # A normalized guardrail score. Lower is better, but it is not a perceptual
    # selector and is deliberately not used to overwrite a profile.
    metric["quality_score"] = (
        metric["dwell_ms_total"] / 1000.0
        + metric["events"]
        + metric["jump_counts_total"] / 10.0
        + metric["tracking_error_abs_median_counts"] / 20.0
        + metric["static_hold_jitter_counts"] / 10.0
        + metric["endpoint_overshoot_deg"]
    )
    return metric, events


def classify(metrics: list[dict], events: list[dict]) -> dict:
    if not metrics:
        return {"primary": "insufficient-data", "categories": [], "confidence": 0.0}
    positive = [metric for metric in metrics if metric["direction"] == "CW"]
    negative = [metric for metric in metrics if metric["direction"] == "CCW"]
    event_positions = [event["position_deg"] for event in events if event["direction"] == "CW"]
    position_spread = statistics.pstdev(event_positions) if len(event_positions) > 1 else float("inf")
    low_speed = [metric for metric in positive if metric["max_velocity_deg_s"] <= 10]
    high_speed = [metric for metric in positive if metric["max_velocity_deg_s"] >= 15]
    low_score = statistics.mean(metric["quality_score"] for metric in low_speed) if low_speed else 0.0
    high_score = statistics.mean(metric["quality_score"] for metric in high_speed) if high_speed else 0.0
    categories: list[dict] = []
    if len(event_positions) >= 3 and position_spread <= 1.0:
        categories.append({"name": "position-locked", "confidence": 0.7, "position_spread_deg": position_spread})
    if low_speed and high_speed and low_score >= high_score * 1.35:
        categories.append({"name": "velocity-dependent", "confidence": 0.7, "low_speed_score": low_score, "high_speed_score": high_score})
    intervals = []
    for direction in ("CW", "CCW"):
        stamps = sorted(event["timestamp_ms"] for event in events if event["direction"] == direction)
        intervals.extend(b - a for a, b in zip(stamps, stamps[1:]) if b > a)
    if len(intervals) >= 4:
        interval_mean = statistics.mean(intervals)
        interval_cv = statistics.pstdev(intervals) / interval_mean if interval_mean else 1.0
        if interval_cv <= 0.25:
            categories.append({"name": "time-frequency-locked", "confidence": 0.55, "interval_cv": interval_cv})
    if not categories:
        categories.append({"name": "mixed-or-hardware-limited", "confidence": 0.45})
    primary = max(categories, key=lambda category: category["confidence"])
    return {
        "primary": primary["name"],
        "categories": categories,
        "confidence": primary["confidence"],
        "cw_event_position_spread_deg": position_spread if math.isfinite(position_spread) else None,
        "cw_runs": len(positive),
        "ccw_runs": len(negative),
    }


def _svg_plot(path: Path, runs: list[tuple[dict, list[dict]]]) -> None:
    width, height = 1000, 520
    plot_left, plot_top, plot_width, plot_height = 60, 30, 900, 430
    series: list[tuple[str, str, list[tuple[float, float]]]] = []
    for index, (metadata, rows) in enumerate(runs):
        color = "#b23a48" if float(metadata["amplitude_deg"]) > 0 else "#2878b5"
        series.append((f"{metadata['run']} cmd", "#222222", [(r["elapsed_ms"], r["cmd_offset_deg"]) for r in rows]))
        series.append((f"{metadata['run']} fb", color, [(r["elapsed_ms"], (r["fb_pos"] - rows[0]["fb_pos"]) / (1024.0 / 300.0)) for r in rows]))
    max_x = max((point[0] for _, _, points in series for point in points), default=1.0)
    values = [point[1] for _, _, points in series for point in points]
    min_y, max_y = min(values, default=-1.0), max(values, default=1.0)
    if max_y - min_y < 1.0:
        min_y -= 0.5
        max_y += 0.5
    def xy(point: tuple[float, float]) -> str:
        x = plot_left + point[0] / max_x * plot_width
        y = plot_top + (max_y - point[1]) / (max_y - min_y) * plot_height
        return f"{x:.1f},{y:.1f}"
    lines = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}">',
             '<rect width="100%" height="100%" fill="white"/>',
             '<text x="60" y="20" font-family="sans-serif" font-size="16">Joint 0 command / feedback</text>']
    for label, color, points in series:
        lines.append(f'<polyline fill="none" stroke="{color}" stroke-width="1.4" points="{" ".join(xy(point) for point in points)}"/>')
    lines.extend([f'<line x1="{plot_left}" y1="{plot_top + plot_height}" x2="{plot_left + plot_width}" y2="{plot_top + plot_height}" stroke="#555"/>',
                  f'<line x1="{plot_left}" y1="{plot_top}" x2="{plot_left}" y2="{plot_top + plot_height}" stroke="#555"/>',
                  '<text x="70" y="500" font-family="sans-serif" font-size="12">black=command, red=CW feedback, blue=CCW feedback</text>', '</svg>'])
    path.write_text("\n".join(lines) + "\n")


def analyze_directory(input_dir: Path, manifest_path: Path, output_dir: Path) -> dict:
    manifest = json.loads(manifest_path.read_text())
    all_metrics: list[dict] = []
    all_events: list[dict] = []
    plot_runs: list[tuple[dict, list[dict]]] = []
    normalized_rows: list[dict] = []
    for metadata in manifest["runs"]:
        rows = parse_capture(input_dir / metadata["log"])
        metric, events = analyze_run(rows, metadata)
        all_metrics.append(metric)
        all_events.extend(events)
        plot_runs.append((metadata, rows))
        for row in resample_fixed_dt(rows):
            normalized_rows.append({
                "run": metadata["run"],
                "direction": metadata.get("direction", ""),
                "ts_ms": row["ts_ms"],
                "elapsed_ms": row["elapsed_ms"],
                "cmd_deg": row["cmd_deg"],
                "cmd_pos": row["cmd_pos"],
                "fb_pos": row["fb_pos"],
                "cmd_velocity_deg_s": row["cmd_velocity_deg_s"],
                "fb_speed_raw": row["fb_speed_raw"],
                "fb_load_raw": row["fb_load_raw"],
                "fb_age_ms": row["fb_age_ms"],
                "voltage_v": row["voltage_v"],
                "stale": row["stale"],
            })
    output_dir.mkdir(parents=True, exist_ok=True)
    with (output_dir / "metrics.csv").open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=sorted(all_metrics[0]))
        writer.writeheader()
        writer.writerows(all_metrics)
    with (output_dir / "events.jsonl").open("w") as handle:
        for event in all_events:
            handle.write(json.dumps(event, sort_keys=True) + "\n")
    with (output_dir / "normalized.csv").open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(normalized_rows[0]))
        writer.writeheader()
        writer.writerows(normalized_rows)
    classification = classify(all_metrics, all_events)
    summary = {"classification": classification, "runs": all_metrics, "event_count": len(all_events)}
    (output_dir / "classification.json").write_text(json.dumps(summary, indent=2) + "\n")
    _svg_plot(output_dir / "joint0_motion.svg", plot_runs)
    return summary


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--input-dir", type=Path, required=True)
    parser.add_argument("--manifest", type=Path)
    parser.add_argument("--output-dir", type=Path)
    args = parser.parse_args()
    manifest = args.manifest or args.input_dir / "manifest.json"
    output = args.output_dir or args.input_dir / "analysis"
    summary = analyze_directory(args.input_dir, manifest, output)
    print(json.dumps(summary["classification"], indent=2))
    print(f"analysis: {output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
