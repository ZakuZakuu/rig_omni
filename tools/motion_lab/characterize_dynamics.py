#!/usr/bin/env python3
"""Supervised SCS009 usable-dynamics characterization harness.

The default invocation is a manifest-only dry run.  Physical movement requires
both ``--execute`` and ``--confirm-hardware``.  Raw UART captures are never
overwritten by this tool; analysis-only mode can reprocess an existing manifest
without opening a serial port.

This is a bounded characterization protocol, not an auto-calibrator.  It keeps
the factory servo parameters and runtime compensation unchanged, runs one
selected joint, and stops before the next dynamics tier if a safety gate fails.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import re
import statistics
import subprocess
import sys
import time
from dataclasses import dataclass, asdict
from datetime import date
from pathlib import Path
from typing import Iterable

try:
    from analyze_stutter import parse_capture
except ImportError:  # pragma: no cover - import path used by package runners
    from tools.motion_lab.analyze_stutter import parse_capture


COUNTS_PER_DEG = 1024.0 / 300.0
DEG_PER_COUNT = 1.0 / COUNTS_PER_DEG
# A commanded excursion of roughly four degrees is only about fourteen
# encoder counts on this servo. Treat runs at or below that scale as
# quantization-sensitive rather than implying sub-count precision.
QUANTIZATION_SENSITIVE_MAX_COUNTS = 4.0 * COUNTS_PER_DEG
DEFAULT_JOINT = 2
DEFAULT_AMPLITUDES = (3, 5, 10)
DEFAULT_HOLD_MS = 1000
DEFAULT_DEADBAND_MDEG = 250
DEFAULT_POLL_PERIOD_MS = 5
CONDITIONING_AMPLITUDE_DEG = 5
CONDITIONING_TRANSITION_MS = 1000
CONDITIONING_HOLD_MS = DEFAULT_HOLD_MS
CONDITIONING_SETTLE_MS = 2000
CONDITIONING_MAX_VELOCITY_DEG_S = 8
CONDITIONING_MAX_ACCELERATION_DEG_S2 = 30
CONDITIONING_MIN_EXCURSION_COUNTS = 0.5 * CONDITIONING_AMPLITUDE_DEG * COUNTS_PER_DEG
CONDITIONING_RETURN_TOLERANCE_COUNTS = 5.0
ONSET_THRESHOLD_COUNTS = 3.0
ONSET_COMMAND_THRESHOLD_COUNTS = 1.0
SUSTAINED_ONSET_SAMPLES = 3
VALID_AGE_MAX_MS = 250.0
# The installed-arm telemetry previously clustered around 8 V.  This is an
# observational baseline used to require human review, not a validated servo
# operating range or electrical safety envelope.
VOLTAGE_BASELINE_V = 8.0
VOLTAGE_DEVIATION_MAX_V = 0.5
TRACKING_ERROR_MIN_DEG = 1.5
TRACKING_ERROR_SCALE = 0.75
TRACKING_ERROR_MAX_DEG = 6.0


@dataclass(frozen=True)
class DynamicsTier:
    name: str
    transition_ms: int
    max_velocity_deg_s: int
    max_acceleration_deg_s2: int

    @property
    def total_ms(self) -> int:
        return self.transition_ms * 2 + DEFAULT_HOLD_MS


DEFAULT_TIERS = (
    DynamicsTier("gentle", 3000, 8, 30),
    DynamicsTier("moderate", 2200, 15, 60),
    DynamicsTier("brisk", 1400, 30, 120),
    DynamicsTier("expressive", 1000, 45, 180),
)


class SafetyGateAbort(RuntimeError):
    """Raised when a run invalidates the progressive matrix."""


def _tracking_error_guard(amplitude_deg: float) -> float:
    """Return a small-motion sanity threshold, not a capability claim."""

    return min(
        TRACKING_ERROR_MAX_DEG,
        max(TRACKING_ERROR_MIN_DEG, abs(float(amplitude_deg)) * TRACKING_ERROR_SCALE),
    )


def _percentile(values: Iterable[float], fraction: float) -> float | None:
    ordered = sorted(float(value) for value in values if math.isfinite(float(value)))
    if not ordered:
        return None
    index = min(len(ordered) - 1, max(0, int(round((len(ordered) - 1) * fraction))))
    return ordered[index]


def _mean(values: Iterable[float]) -> float | None:
    finite = [float(value) for value in values if math.isfinite(float(value))]
    return statistics.mean(finite) if finite else None


def _parse_int_list(value: str, name: str) -> tuple[int, ...]:
    try:
        values = tuple(int(part.strip()) for part in value.split(",") if part.strip())
    except ValueError as exc:
        raise argparse.ArgumentTypeError(f"{name} must be comma-separated integers") from exc
    if not values:
        raise argparse.ArgumentTypeError(f"{name} must not be empty")
    return values


def _firmware_context() -> dict[str, str]:
    root = Path(__file__).resolve().parents[2]
    def git(*args: str) -> str:
        try:
            return subprocess.check_output(["git", "-C", str(root), *args], text=True).strip()
        except (OSError, subprocess.CalledProcessError):
            return "unknown"
    return {"commit": git("rev-parse", "HEAD"), "branch": git("branch", "--show-current")}


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _read_capture_text(path: Path) -> str:
    return path.read_text(errors="replace")


def _parse_status_capture(path: Path) -> dict:
    """Parse the read-only ``mlab status`` snapshot fail-closed."""

    records: list[dict] = []
    malformed = 0
    for line in _read_capture_text(path).splitlines():
        if not line.startswith("MLAB_SERVO_STATUS,") or line.startswith("MLAB_SERVO_STATUS,id,"):
            continue
        fields = line.split(",")
        if len(fields) != 10:
            malformed += 1
            continue
        try:
            records.append({
                "id": int(fields[1]),
                "error_hex": int(fields[2], 16),
                "last_error_hex": int(fields[3], 16),
                "last_error_ts_ms": int(fields[4]),
                "error_count": int(fields[5]),
                "fb_pos": int(fields[6]),
                "fb_speed_raw": int(fields[7]),
                "fb_load_raw": int(fields[8]),
                "stale": int(fields[9]),
            })
        except ValueError:
            malformed += 1
    records.sort(key=lambda record: record["id"])
    expected_ids = list(range(1, 6))
    missing_ids = [servo_id for servo_id in expected_ids if servo_id not in {record["id"] for record in records}]
    status_errors = [
        {
            "id": record["id"],
            "error_hex": record["error_hex"],
            "last_error_hex": record["last_error_hex"],
            "stale": record["stale"],
        }
        for record in records
        if record["error_hex"] != 0 or record["last_error_hex"] != 0 or record["stale"] != 0
    ]
    return {
        "records": records,
        "record_count": len(records),
        "missing_ids": missing_ids,
        "malformed_rows": malformed,
        "errors": status_errors,
        "parse_ok": len(records) == 5 and not missing_ids and malformed == 0,
    }


def _parse_voltage_capture(path: Path) -> dict:
    """Parse the read-only ID-1 voltage response and require a valid reply."""

    pattern = re.compile(
        r"^MLAB_VOLTAGE,ts_ms=(?P<ts>\d+),id=(?P<id>\d+),"
        r"voltage_v=(?P<voltage>-?\d+(?:\.\d+)?),request_sent=(?P<sent>[01])$"
    )
    matches: list[dict] = []
    for line in _read_capture_text(path).splitlines():
        match = pattern.match(line.strip())
        if match is None:
            continue
        matches.append({
            "ts_ms": int(match.group("ts")),
            "id": int(match.group("id")),
            "voltage_v": float(match.group("voltage")),
            "request_sent": int(match.group("sent")),
        })
    record = next((item for item in reversed(matches) if item["id"] == 1), None)
    parse_ok = (
        record is not None
        and record["request_sent"] == 1
        and math.isfinite(record["voltage_v"])
        and record["voltage_v"] > 0.0
    )
    return {"record": record, "records": matches, "parse_ok": parse_ok}


def _parse_params_capture(path: Path) -> dict:
    """Validate that the immutable parameter snapshot completed."""

    text = _read_capture_text(path)
    rows = [line for line in text.splitlines() if line.startswith("SCS009_PARAM,")]
    return {
        "row_count": len(rows),
        "read_only_banner": "SCS009_PARAM dump: read-only" in text,
        "complete": "SCS009_PARAM dump: complete" in text,
        "parse_ok": bool(rows) and "SCS009_PARAM dump: complete" in text,
    }


def _preflight_decision(status_path: Path, params_path: Path, voltage_path: Path) -> dict:
    """Return a persisted, fail-closed decision before any movement command."""

    reasons: list[str] = []
    try:
        status = _parse_status_capture(status_path)
    except (OSError, UnicodeError) as error:
        status = {"parse_ok": False, "records": [], "errors": [], "missing_ids": [], "malformed_rows": 0}
        reasons.append(f"status capture unreadable: {error}")
    if not status.get("parse_ok"):
        reasons.append("status capture missing or incomplete for servo IDs 1..5")
    if status.get("errors"):
        reasons.append("status reports a current, latched, or stale servo error")

    try:
        voltage = _parse_voltage_capture(voltage_path)
    except (OSError, UnicodeError) as error:
        voltage = {"parse_ok": False, "record": None, "records": []}
        reasons.append(f"voltage capture unreadable: {error}")
    if not voltage.get("parse_ok"):
        reasons.append("voltage response missing, invalid, or request was not acknowledged")
    voltage_record = voltage.get("record") or {}
    voltage_value = voltage_record.get("voltage_v")
    if voltage_value is not None and abs(voltage_value - VOLTAGE_BASELINE_V) > VOLTAGE_DEVIATION_MAX_V:
        reasons.append(
            f"observed voltage {voltage_value:.2f} V differs from the prior ~{VOLTAGE_BASELINE_V:.1f} V baseline; user review required"
        )

    try:
        params = _parse_params_capture(params_path)
    except (OSError, UnicodeError) as error:
        params = {"parse_ok": False, "row_count": 0, "read_only_banner": False, "complete": False}
        reasons.append(f"parameter snapshot unreadable: {error}")
    if not params.get("parse_ok"):
        reasons.append("read-only SCS009 parameter snapshot missing or incomplete")

    return {
        "passed": not reasons,
        "reasons": reasons,
        "status": status,
        "voltage": voltage,
        "parameters": params,
        "voltage_policy": {
            "baseline_v": VOLTAGE_BASELINE_V,
            "review_deviation_v": VOLTAGE_DEVIATION_MAX_V,
            "note": "observational preflight guard; not a validated SCS009 operating or safety range",
        },
    }


def _capture_command(port: str, command: str, output: Path, duration_ms: int, *, tail_s: float = 1.5) -> None:
    """Run the existing raw serial capture helper without overwriting output."""

    if output.exists():
        raise FileExistsError(f"refusing to overwrite immutable capture: {output}")
    script = Path(__file__).with_name("capture_serial.py")
    subprocess.run(
        [
            sys.executable,
            str(script),
            "--port",
            port,
            "--command",
            command,
            "--duration-ms",
            str(duration_ms),
            "--output",
            str(output),
            "--ready-timeout-s",
            "0.8",
            "--tail-s",
            str(tail_s),
        ],
        check=True,
    )


def _new_output(path: Path) -> Path:
    if path.exists():
        raise FileExistsError(f"refusing to overwrite existing artifact: {path}")
    return path


def _raw_rows(path: Path, joint: int) -> list[dict[str, float]]:
    return parse_capture(path, joint=joint)


def _valid_feedback_rows(rows: list[dict[str, float]]) -> list[dict[str, float]]:
    return [
        row
        for row in rows
        if row["stale"] == 0 and row["fb_ts_ms"] > 0 and row["fb_age_ms"] < 1_000_000_000
    ]


def _unique_feedback_samples(rows: list[dict[str, float]]) -> list[tuple[float, float]]:
    samples: dict[float, float] = {}
    for row in _valid_feedback_rows(rows):
        samples[row["fb_ts_ms"]] = row["fb_pos"]
    return sorted(samples.items())


def _resample(samples: list[tuple[float, float]], step_ms: float = 20.0) -> list[tuple[float, float]]:
    if len(samples) < 2:
        return samples[:]
    start, end = samples[0][0], samples[-1][0]
    output: list[tuple[float, float]] = []
    index = 0
    timestamp = start
    while timestamp <= end + 1e-6:
        while index + 1 < len(samples) and samples[index + 1][0] < timestamp:
            index += 1
        left = samples[index]
        right = samples[min(index + 1, len(samples) - 1)]
        span = max(right[0] - left[0], 1.0)
        alpha = min(1.0, max(0.0, (timestamp - left[0]) / span))
        output.append((timestamp, left[1] + alpha * (right[1] - left[1])))
        timestamp += step_ms
    return output


def _solve_linear(matrix: list[list[float]], vector: list[float]) -> list[float]:
    """Solve a small dense system for the local polynomial fit."""

    augmented = [row[:] + [value] for row, value in zip(matrix, vector)]
    size = len(augmented)
    for column in range(size):
        pivot = max(range(column, size), key=lambda row: abs(augmented[row][column]))
        if abs(augmented[pivot][column]) < 1e-12:
            raise ValueError("singular local polynomial window")
        augmented[column], augmented[pivot] = augmented[pivot], augmented[column]
        divisor = augmented[column][column]
        augmented[column] = [value / divisor for value in augmented[column]]
        for row in range(size):
            if row == column:
                continue
            factor = augmented[row][column]
            augmented[row] = [a - factor * b for a, b in zip(augmented[row], augmented[column])]
    return [augmented[row][-1] for row in range(size)]


def _local_polynomial_velocity(samples: list[tuple[float, float]], window: int = 5) -> list[float]:
    """Return count/s from conservative local quadratic fits.

    This deliberately avoids differentiating raw quantized, irregular samples
    directly.  The samples are first linearly resampled at 20 ms, then each
    point uses a centered local polynomial in seconds.  The output is an
    observed velocity estimate, not a servo-calibrated speed.
    """

    if len(samples) < 3:
        return []
    width = max(3, window | 1)
    half = width // 2
    velocities: list[float] = []
    for index, (center_time, _position) in enumerate(samples):
        left = max(0, index - half)
        right = min(len(samples), index + half + 1)
        local = samples[left:right]
        degree = min(2, len(local) - 1)
        matrix = [[0.0 for _ in range(degree + 1)] for _ in range(degree + 1)]
        vector = [0.0 for _ in range(degree + 1)]
        for timestamp, position in local:
            x = (timestamp - center_time) / 1000.0
            powers = [x**power for power in range(degree + 1)]
            for row in range(degree + 1):
                vector[row] += powers[row] * position
                for column in range(degree + 1):
                    matrix[row][column] += powers[row] * powers[column]
        try:
            coefficients = _solve_linear(matrix, vector)
        except ValueError:
            continue
        velocities.append(coefficients[1] if len(coefficients) > 1 else 0.0)
    return velocities


def _sustained_onset(rows: list[dict[str, float]], baseline: float, sign: float) -> float | None:
    valid = _valid_feedback_rows(rows)
    for index in range(len(valid) - SUSTAINED_ONSET_SAMPLES + 1):
        window = valid[index : index + SUSTAINED_ONSET_SAMPLES]
        # A stale/duplicated feedback sample must not count as sustained
        # motion.  The high-rate path can emit several telemetry rows between
        # actual servo replies, so require strictly advancing servo timestamps.
        if any(later["fb_ts_ms"] <= earlier["fb_ts_ms"] for earlier, later in zip(window, window[1:])):
            continue
        displacements = [sign * (sample["fb_pos"] - baseline) for sample in window]
        if all(value >= ONSET_THRESHOLD_COUNTS for value in displacements):
            return window[0]["fb_ts_ms"]
    return None


def _crossing_time(rows: list[dict[str, float]], baseline: float, sign: float, target_counts: float, fraction: float) -> float | None:
    threshold = target_counts * fraction
    for row in _valid_feedback_rows(rows):
        if sign * (row["fb_pos"] - baseline) >= threshold:
            return row["fb_ts_ms"]
    return None


def _reversal_proxy(rows: list[dict[str, float]]) -> list[dict[str, float]]:
    """Measure command reversal response as lost-motion, not pure backlash."""

    if len(rows) < 3:
        return []
    reversals: list[dict[str, float]] = []
    for index in range(1, len(rows)):
        previous, current = rows[index - 1], rows[index]
        old_velocity = previous["cmd_velocity_deg_s"]
        new_velocity = current["cmd_velocity_deg_s"]
        if abs(old_velocity) < 0.5 or abs(new_velocity) < 0.5 or old_velocity * new_velocity >= 0:
            continue
        sign = 1.0 if new_velocity > 0 else -1.0
        reference_feedback = current["fb_pos"]
        travel_counts = 0.0
        response_ts: float | None = None
        for later_index in range(index, len(rows)):
            later = rows[later_index]
            prior = rows[max(index, later_index - 1)]
            travel_counts += abs(later["cmd_pos"] - prior["cmd_pos"])
            if later["stale"] == 0 and sign * (later["fb_pos"] - reference_feedback) >= ONSET_THRESHOLD_COUNTS:
                response_ts = later["fb_ts_ms"]
                break
        if response_ts is not None:
            reversals.append(
                {
                    "command_reversal_ts_ms": current["ts_ms"],
                    "feedback_response_ts_ms": response_ts,
                    "response_delay_ms": max(0.0, response_ts - current["ts_ms"]),
                    "command_travel_before_feedback_counts": travel_counts,
                    "command_travel_before_feedback_deg": travel_counts * DEG_PER_COUNT,
                }
            )
    return reversals


def analyze_run(rows: list[dict[str, float]], metadata: dict) -> tuple[dict, list[dict]]:
    if len(rows) < 2:
        raise ValueError(f"{metadata.get('run', 'run')}: fewer than two telemetry rows")
    amplitude = float(metadata["amplitude_deg"])
    sign = 1.0 if amplitude >= 0 else -1.0
    baseline_cmd = rows[0]["cmd_pos"]
    baseline_feedback = rows[0]["fb_pos"]
    valid = _valid_feedback_rows(rows)
    command_displacements = [sign * (row["cmd_pos"] - baseline_cmd) for row in rows]
    feedback_displacements = [sign * (row["fb_pos"] - baseline_feedback) for row in valid]
    command_excursion_counts = max(command_displacements, default=0.0)
    achieved_excursion_counts = max(feedback_displacements, default=0.0)
    errors_deg = [(row["fb_pos"] - row["cmd_pos"]) * DEG_PER_COUNT for row in valid]
    feedback_samples = _unique_feedback_samples(rows)
    resampled = _resample(feedback_samples)
    observed_velocity = _local_polynomial_velocity(resampled)
    transition_ms = float(metadata.get("transition_ms", metadata.get("duration_ms", 0)))
    hold_end_ms = transition_ms + float(metadata.get("hold_ms", DEFAULT_HOLD_MS))
    command_onset = next(
        (row["ts_ms"] for row in rows if abs(row["cmd_pos"] - baseline_cmd) >= ONSET_COMMAND_THRESHOLD_COUNTS),
        None,
    )
    feedback_onset = _sustained_onset(rows, baseline_feedback, sign)
    onset_latency = None if command_onset is None or feedback_onset is None else max(0.0, feedback_onset - command_onset)
    target_counts = max(command_excursion_counts, 1.0)
    crossing_10 = _crossing_time(rows, baseline_feedback, sign, target_counts, 0.10)
    crossing_90 = _crossing_time(rows, baseline_feedback, sign, target_counts, 0.90)
    time_10_90 = None if crossing_10 is None or crossing_90 is None else max(0.0, crossing_90 - crossing_10)
    hold_rows = [row for row in valid if transition_ms <= row["elapsed_ms"] <= hold_end_ms]
    target_command_pos = baseline_cmd + sign * target_counts
    settling_tolerance_counts = max(5.0, target_counts * 0.05)
    last_bad = max(
        (row["fb_ts_ms"] for row in hold_rows if abs(row["fb_pos"] - target_command_pos) > settling_tolerance_counts),
        default=None,
    )
    settling_time = None if crossing_90 is None or last_bad is None else max(0.0, last_bad - crossing_90)
    signed_peak = max((sign * (row["fb_pos"] - baseline_feedback) for row in valid), default=0.0)
    overshoot_counts = max(0.0, signed_peak - target_counts)
    age_values = [row["fb_age_ms"] for row in rows if row["fb_age_ms"] < 1_000_000_000]
    stale_rows = sum(1 for row in rows if row["stale"] != 0)
    feedback_rate = None
    if len(feedback_samples) >= 2 and feedback_samples[-1][0] > feedback_samples[0][0]:
        feedback_rate = (len(feedback_samples) - 1) / ((feedback_samples[-1][0] - feedback_samples[0][0]) / 1000.0)
    reversals = _reversal_proxy(rows) if metadata.get("experiment") == 5 else []
    metric = {
        **metadata,
        "rows": len(rows),
        "valid_feedback_rows": len(valid),
        "feedback_valid_rate": len(valid) / len(rows),
        "measured_feedback_sample_rate_hz": feedback_rate,
        "feedback_age_p50_ms": _percentile(age_values, 0.50),
        "feedback_age_p95_ms": _percentile(age_values, 0.95),
        "feedback_age_max_ms": max(age_values, default=None),
        "stale_rows": stale_rows,
        "stale_fraction": stale_rows / len(rows),
        "encoder_resolution_deg_per_count": DEG_PER_COUNT,
        "command_excursion_counts": command_excursion_counts,
        "achieved_excursion_counts": achieved_excursion_counts,
        "command_excursion_deg": command_excursion_counts * DEG_PER_COUNT,
        "achieved_excursion_deg": achieved_excursion_counts * DEG_PER_COUNT,
        "quantization_sensitive": (
            command_excursion_counts <= QUANTIZATION_SENSITIVE_MAX_COUNTS
            or achieved_excursion_counts <= QUANTIZATION_SENSITIVE_MAX_COUNTS
        ),
        "quantization_note": (
            "Encoder resolution is approximately %.4f deg/count; this run is "
            "marked quantization-sensitive when commanded or achieved excursion "
            "is at most %.2f counts."
            % (DEG_PER_COUNT, QUANTIZATION_SENSITIVE_MAX_COUNTS)
        ),
        "tracking_rms_error_deg": math.sqrt(sum(value * value for value in errors_deg) / len(errors_deg)) if errors_deg else None,
        "tracking_peak_error_deg": max((abs(value) for value in errors_deg), default=None),
        "tracking_error_guard_deg": _tracking_error_guard(amplitude),
        "motion_onset_latency_ms": onset_latency,
        "motion_onset_latency_note": (
            "Latency is measured from the first command sample changing by at "
            "least %.1f count to the first set of %d strictly advancing feedback "
            "samples each displaced by at least %.1f counts. The thresholds are "
            "deliberately different and the result is not pure transport or "
            "actuation latency; it also includes command quantization, feedback "
            "age, and mechanical response."
            % (ONSET_COMMAND_THRESHOLD_COUNTS, SUSTAINED_ONSET_SAMPLES, ONSET_THRESHOLD_COUNTS)
        ),
        "motion_10_90_time_ms": time_10_90,
        "observed_peak_velocity_deg_s": max((abs(value) * DEG_PER_COUNT for value in observed_velocity), default=None),
        "velocity_estimator": "20ms linear resample + centered local quadratic fit",
        "observed_peak_velocity_calibrated": False,
        "observed_peak_velocity_note": (
            "Derived from quantized feedback after resampling and a local "
            "quadratic fit; not a calibrated actuator velocity limit."
        ),
        "settling_time_ms": settling_time,
        "settling_tolerance_counts": settling_tolerance_counts,
        "endpoint_overshoot_deg": overshoot_counts * DEG_PER_COUNT,
        "voltage_min_v": min((row["voltage_v"] for row in rows), default=None),
        "voltage_max_v": max((row["voltage_v"] for row in rows), default=None),
        "raw_load_min": min((row["fb_load_raw"] for row in valid), default=None),
        "raw_load_max": max((row["fb_load_raw"] for row in valid), default=None),
        "raw_speed_min": min((row["fb_speed_raw"] for row in valid), default=None),
        "raw_speed_max": max((row["fb_speed_raw"] for row in valid), default=None),
        "reversal_count": len(reversals),
        "reversal_proxy": reversals,
    }
    events = []
    for previous, current in zip(rows, rows[1:]):
        if current["stale"] == 0 and abs(current["fb_pos"] - previous["fb_pos"]) >= 4.0:
            events.append({
                "timestamp_ms": current["ts_ms"],
                "feedback_jump_counts": abs(current["fb_pos"] - previous["fb_pos"]),
                "direction": "positive" if current["fb_pos"] > previous["fb_pos"] else "negative",
            })
    metric["feedback_jump_events"] = len(events)
    return metric, events


def _safety_reasons(metric: dict, max_load_raw: float) -> list[str]:
    reasons: list[str] = []
    if metric["feedback_valid_rate"] < 0.80:
        reasons.append("feedback valid rate below 80%")
    if metric["stale_fraction"] > 0.0:
        reasons.append("stale feedback present")
    if metric["feedback_age_p95_ms"] is not None and metric["feedback_age_p95_ms"] > VALID_AGE_MAX_MS:
        reasons.append(f"feedback age p95 exceeds {VALID_AGE_MAX_MS:g} ms")
    if metric["voltage_min_v"] is not None and (
        abs(metric["voltage_min_v"] - VOLTAGE_BASELINE_V) > VOLTAGE_DEVIATION_MAX_V
        or abs(metric["voltage_max_v"] - VOLTAGE_BASELINE_V) > VOLTAGE_DEVIATION_MAX_V
    ):
        reasons.append(
            "voltage deviates from the observational ~8 V device baseline; this is not a validated electrical safety limit"
        )
    tracking_guard = _tracking_error_guard(float(metric["amplitude_deg"]))
    if metric["tracking_peak_error_deg"] is not None and metric["tracking_peak_error_deg"] > tracking_guard:
        reasons.append(
            f"tracking error exceeds the {tracking_guard:.2f} deg small-motion sanity guard (not a capability limit)"
        )
    if metric["raw_load_max"] is not None and metric["raw_load_max"] > max_load_raw:
        reasons.append(
            f"raw load exceeds provisional {max_load_raw:g} observational anomaly gate; raw encoding is not a physical load limit"
        )
    return reasons


def _direction_asymmetry(metrics: list[dict]) -> list[dict]:
    groups: dict[tuple[str, int, int], dict[str, list[dict]]] = {}
    for metric in metrics:
        key = (str(metric.get("tier", "")), abs(int(metric["amplitude_deg"])), int(metric.get("repetition", 0)))
        direction = "positive" if float(metric["amplitude_deg"]) >= 0 else "negative"
        groups.setdefault(key, {}).setdefault(direction, []).append(metric)
    output: list[dict] = []
    for (tier, amplitude, repetition), directions in sorted(groups.items()):
        positive = directions.get("positive", [])
        negative = directions.get("negative", [])
        if not positive or not negative:
            continue
        fields = ("motion_onset_latency_ms", "tracking_rms_error_deg", "observed_peak_velocity_deg_s", "endpoint_overshoot_deg")
        item = {"tier": tier, "amplitude_deg": amplitude, "repetition": repetition}
        for field in fields:
            positive_value = _mean(metric[field] for metric in positive if metric[field] is not None)
            negative_value = _mean(metric[field] for metric in negative if metric[field] is not None)
            item[f"positive_{field}"] = positive_value
            item[f"negative_{field}"] = negative_value
            item[f"positive_minus_negative_{field}"] = None if positive_value is None or negative_value is None else positive_value - negative_value
        output.append(item)
    return output


def _conditioning_entry(args: argparse.Namespace, direction: str) -> dict:
    """Build the optional directional center-preconditioning motion."""

    if direction not in ("positive", "negative"):
        raise ValueError("conditioning direction must be positive or negative")
    sign = 1 if direction == "positive" else -1
    return {
        "tier": "conditioning",
        "experiment": 5,
        "trajectory": 2,
        "joint": args.joint,
        "amplitude_deg": sign * CONDITIONING_AMPLITUDE_DEG,
        "transition_ms": CONDITIONING_TRANSITION_MS,
        "hold_ms": CONDITIONING_HOLD_MS,
        "duration_ms": CONDITIONING_TRANSITION_MS * 3 + CONDITIONING_HOLD_MS * 2,
        "stagger_ms": 0,
        "max_velocity_deg_s": CONDITIONING_MAX_VELOCITY_DEG_S,
        "max_acceleration_deg_s2": CONDITIONING_MAX_ACCELERATION_DEG_S2,
        "deadband_mdeg": args.deadband_mdeg,
        "direction": direction,
        "description": (
            "center -> +5 -> -5 -> center"
            if direction == "positive"
            else "center -> -5 -> +5 -> center"
        ),
    }


def _conditioning_metrics(
    rows: list[dict[str, float]],
    metadata: dict,
    center_count: float,
    settled_feedback_count: float | None,
    settle_status: dict,
) -> dict:
    """Summarize conditioning separately from formal motion metrics."""

    valid = _valid_feedback_rows(rows)
    feedback_samples = _unique_feedback_samples(rows)
    command_center_count = rows[0]["cmd_pos"] if rows else None
    command_displacement = (
        []
        if command_center_count is None
        else [row["cmd_pos"] - command_center_count for row in rows]
    )
    displacement = [row["fb_pos"] - center_count for row in valid]
    age_values = [row["fb_age_ms"] for row in rows if row["fb_age_ms"] < 1_000_000_000]
    target_counts = abs(float(metadata["amplitude_deg"])) * COUNTS_PER_DEG
    feedback_rate = None
    if len(feedback_samples) >= 2 and feedback_samples[-1][0] > feedback_samples[0][0]:
        feedback_rate = (len(feedback_samples) - 1) / (
            (feedback_samples[-1][0] - feedback_samples[0][0]) / 1000.0
        )
    return_error_counts = (
        None if settled_feedback_count is None else settled_feedback_count - center_count
    )
    return {
        "conditioning_direction": metadata["direction"],
        "conditioning_amplitude_deg": abs(float(metadata["amplitude_deg"])),
        "conditioning_target_counts": target_counts,
        "conditioning_rows": len(rows),
        "conditioning_valid_feedback_rows": len(valid),
        "conditioning_feedback_valid_rate": len(valid) / len(rows) if rows else 0.0,
        "conditioning_feedback_unique_samples": len(feedback_samples),
        "conditioning_feedback_sample_rate_hz": feedback_rate,
        "conditioning_feedback_age_p95_ms": _percentile(age_values, 0.95),
        "conditioning_feedback_age_max_ms": max(age_values, default=None),
        "conditioning_stale_rows": sum(1 for row in rows if row["stale"] != 0),
        "conditioning_command_center_count": command_center_count,
        "conditioning_commanded_positive_counts": max(
            0.0, max(command_displacement, default=0.0)
        ),
        "conditioning_commanded_negative_counts": max(
            0.0, -min(command_displacement, default=0.0)
        ),
        "conditioning_commanded_positive_deg": (
            max(0.0, max(command_displacement, default=0.0)) * DEG_PER_COUNT
        ),
        "conditioning_commanded_negative_deg": (
            max(0.0, -min(command_displacement, default=0.0)) * DEG_PER_COUNT
        ),
        "conditioning_achieved_positive_counts": max(0.0, max(displacement, default=0.0)),
        "conditioning_achieved_negative_counts": max(0.0, -min(displacement, default=0.0)),
        "conditioning_achieved_positive_deg": max(0.0, max(displacement, default=0.0)) * DEG_PER_COUNT,
        "conditioning_achieved_negative_deg": max(0.0, -min(displacement, default=0.0)) * DEG_PER_COUNT,
        "conditioning_return_feedback_count": settled_feedback_count,
        "conditioning_return_error_counts": return_error_counts,
        "conditioning_return_error_deg": (
            None if return_error_counts is None else return_error_counts * DEG_PER_COUNT
        ),
        "conditioning_voltage_min_v": min((row["voltage_v"] for row in rows), default=None),
        "conditioning_voltage_max_v": max((row["voltage_v"] for row in rows), default=None),
        "conditioning_raw_load_max": max((row["fb_load_raw"] for row in valid), default=None),
        "conditioning_settle_status_parse_ok": bool(settle_status.get("parse_ok")),
        "conditioning_settle_status_errors": settle_status.get("errors", []),
        "conditioning_passed": False,
        "conditioning_health_reasons": [],
    }


def _conditioning_health_reasons(summary: dict, args: argparse.Namespace) -> list[str]:
    """Return fail-closed health reasons before a formal run may start."""

    reasons: list[str] = []
    if summary["conditioning_feedback_valid_rate"] < 0.80:
        reasons.append("conditioning feedback valid rate below 80%")
    if summary["conditioning_feedback_unique_samples"] < 2:
        reasons.append("conditioning feedback has fewer than two advancing samples")
    if summary["conditioning_stale_rows"] > 0:
        reasons.append("conditioning stale feedback present")
    age_p95 = summary["conditioning_feedback_age_p95_ms"]
    if age_p95 is None:
        reasons.append("conditioning feedback age is unavailable")
    elif age_p95 > VALID_AGE_MAX_MS:
        reasons.append(f"conditioning feedback age p95 exceeds {VALID_AGE_MAX_MS:g} ms")
    for field in ("conditioning_voltage_min_v", "conditioning_voltage_max_v"):
        value = summary[field]
        if value is None:
            reasons.append("conditioning servo voltage telemetry is unavailable")
            break
        if abs(value - VOLTAGE_BASELINE_V) > VOLTAGE_DEVIATION_MAX_V:
            reasons.append(
                "conditioning voltage differs from the observational ~8 V baseline; "
                "this is not a validated electrical safety limit"
            )
            break
    if summary["conditioning_achieved_positive_counts"] < CONDITIONING_MIN_EXCURSION_COUNTS:
        reasons.append("conditioning positive excursion did not pass the breakaway health gate")
    if summary["conditioning_achieved_negative_counts"] < CONDITIONING_MIN_EXCURSION_COUNTS:
        reasons.append("conditioning negative excursion did not pass the breakaway health gate")
    if not summary["conditioning_settle_status_parse_ok"]:
        reasons.append("conditioning settle status missing or incomplete")
    if summary["conditioning_settle_status_errors"]:
        reasons.append("conditioning settle status reports a servo error or stale state")
    return_error = summary["conditioning_return_error_counts"]
    if return_error is None:
        reasons.append("conditioning return position is unavailable")
    elif abs(return_error) > CONDITIONING_RETURN_TOLERANCE_COUNTS:
        reasons.append(
            f"conditioning return error exceeds {CONDITIONING_RETURN_TOLERANCE_COUNTS:g} counts"
        )
    raw_load_max = summary["conditioning_raw_load_max"]
    if raw_load_max is None:
        reasons.append("conditioning raw load telemetry is unavailable")
    elif raw_load_max > args.max_load_raw:
        reasons.append(
            "conditioning raw load exceeds the provisional observational anomaly gate"
        )
    return reasons


def _write_normalized_csv(path: Path, runs: list[tuple[dict, list[dict[str, float]]]]) -> None:
    fields = (
        "run", "tier", "direction", "repetition", "ts_ms", "elapsed_ms", "fb_ts_ms", "fb_age_ms", "stale",
        "cmd_pos", "fb_pos", "cmd_deg", "cmd_offset_deg", "fb_speed_raw", "fb_load_raw", "voltage_v",
        "cmd_velocity_deg_s", "fb_delta_counts", "fb_position_deg",
    )
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        for metadata, rows in runs:
            baseline = rows[0]["fb_pos"] if rows else 0.0
            for row in rows:
                output = {field: row.get(field, "") for field in fields}
                output.update({
                    "run": metadata["run"],
                    "tier": metadata.get("tier", ""),
                    "direction": metadata.get("direction", ""),
                    "repetition": metadata.get("repetition", ""),
                    "fb_position_deg": (row["fb_pos"] - baseline) * DEG_PER_COUNT,
                })
                writer.writerow(output)


def _build_plan(args: argparse.Namespace) -> tuple[list[dict], list[dict]]:
    amplitudes = tuple(args.amplitudes_deg)
    directions = (1, -1)
    tiers_by_name = {tier.name: tier for tier in DEFAULT_TIERS}
    selected_tiers = [tiers_by_name[name] for name in args.tiers]
    groups: list[dict] = []
    # The first planned condition is intentionally the documented gentle,
    # positive 3-degree run. Low-speed probes and reversal are follow-up
    # observations, never an accidental first hardware movement.
    for tier in selected_tiers:
        entries = []
        for amplitude in amplitudes:
            for direction in directions:
                for repetition in range(1, args.repetitions + 1):
                    entries.append({
                        "tier": tier.name, "experiment": 4, "joint": args.joint,
                        "amplitude_deg": direction * amplitude,
                        "transition_ms": tier.transition_ms, "hold_ms": DEFAULT_HOLD_MS,
                        "duration_ms": tier.total_ms, "max_velocity_deg_s": tier.max_velocity_deg_s,
                        "max_acceleration_deg_s2": tier.max_acceleration_deg_s2,
                        "deadband_mdeg": args.deadband_mdeg, "repetition": repetition,
                    })
        groups.append({"name": tier.name, "entries": entries})
    if not args.skip_low_speed:
        for speed in args.low_speed_deg_s:
            tier = DynamicsTier(f"low_speed_{speed}", 4000, speed, max(20, speed * 4))
            entries = []
            for direction in directions:
                for repetition in range(1, args.repetitions + 1):
                    entries.append({
                        "tier": tier.name, "experiment": 4, "joint": args.joint,
                        "amplitude_deg": direction * args.low_speed_amplitude_deg,
                        "transition_ms": tier.transition_ms, "hold_ms": DEFAULT_HOLD_MS,
                        "duration_ms": tier.total_ms, "max_velocity_deg_s": tier.max_velocity_deg_s,
                        "max_acceleration_deg_s2": tier.max_acceleration_deg_s2,
                        "deadband_mdeg": args.deadband_mdeg, "repetition": repetition,
                    })
            groups.append({"name": tier.name, "entries": entries})
    if not args.skip_reversal:
        tier = tiers_by_name[args.reversal_tier]
        entries = []
        for repetition in range(1, args.repetitions + 1):
            entries.append({
                "tier": f"reversal_{tier.name}", "experiment": 5, "joint": args.joint,
                "amplitude_deg": args.reversal_amplitude_deg, "transition_ms": tier.transition_ms,
                "hold_ms": DEFAULT_HOLD_MS, "duration_ms": tier.transition_ms * 3 + DEFAULT_HOLD_MS * 2,
                "max_velocity_deg_s": tier.max_velocity_deg_s,
                "max_acceleration_deg_s2": tier.max_acceleration_deg_s2,
                "deadband_mdeg": args.deadband_mdeg, "repetition": repetition,
            })
        groups.append({"name": f"reversal_{tier.name}", "entries": entries})
    planned = []
    for group in groups:
        planned.extend(group["entries"])
    return groups, planned


def _execution_entries(groups: list[dict], max_runs: int) -> list[tuple[str, dict]]:
    """Flatten the deterministic plan to the bounded set executed this run."""

    selected: list[tuple[str, dict]] = []
    for group in groups:
        for entry in group["entries"]:
            if len(selected) >= max_runs:
                return selected
            selected.append((group["name"], entry))
    return selected


def _command_for(entry: dict) -> str:
    return (
        f"mlab run {entry['experiment']} 2 {entry['joint']} {entry['amplitude_deg']} "
        f"{entry['transition_ms']} 0 {entry['max_velocity_deg_s']} "
        f"{entry['max_acceleration_deg_s2']} {entry['deadband_mdeg']}"
    )


def _print_plan(args: argparse.Namespace, groups: list[dict], planned: list[dict]) -> None:
    print("SCS009 usable-dynamics characterization plan")
    print(f"  selected joint: J{args.joint} (J2 default: visible forearm pitch, neutral-centered and not near its model limits)")
    print("  parameters: factory/current read-only; no EEPROM writes; runtime compensation disabled")
    print(f"  amplitudes: {args.amplitudes_deg}; directions: positive and negative; repetitions: {args.repetitions}")
    print(f"  hold: {DEFAULT_HOLD_MS} ms before return; selected feedback poll: {args.poll_period_ms} ms")
    if args.precondition:
        conditioning = _conditioning_entry(args, args.precondition)
        print(
            f"  conditioning: {conditioning['description']}; "
            f"{CONDITIONING_SETTLE_MS} ms settle before formal run"
        )
        if args.conditioning_only:
            print("  mode: conditioning-only; formal dynamics captures disabled")
    else:
        print("  conditioning: disabled (cold/unconditioned protocol)")
    print(f"  planned movement captures: {len(planned)}")
    print(f"  execution limit: {args.max_runs} movement(s); first supervised run is the only default run")
    for group in groups:
        first = group["entries"][0]
        print(f"  {group['name']}: {len(group['entries'])} runs; first command: {_command_for(first)}")
    print("  raw captures are immutable; normalized CSV/JSON are derived artifacts")
    print("  safety: stop on stale feedback, age, voltage, load, or tracking guardrail failure")


def _initial_manifest(args: argparse.Namespace, planned: list[dict]) -> dict:
    return {
        "protocol": "scs009-usable-dynamics-v0.1",
        "status": "planned",
        "firmware": _firmware_context(),
        "joint_selection": {
            "joint": args.joint,
            "reason": "J2 is a visible forearm-pitch joint, away from model limits at the neutral pose; no current fault is assumed.",
        },
        "safety_policy": {
            "voltage_baseline_v": VOLTAGE_BASELINE_V,
            "voltage_review_deviation_v": VOLTAGE_DEVIATION_MAX_V,
            "voltage_note": "observational preflight/run guard; not a validated SCS009 operating or electrical safety range",
            "valid_age_p95_max_ms": VALID_AGE_MAX_MS,
            "onset_threshold_counts": ONSET_THRESHOLD_COUNTS,
            "max_load_raw": args.max_load_raw,
            "load_note": "provisional raw anomaly threshold; fb_load_raw encoding is not independently validated as physical load",
            "tracking_error_min_deg": TRACKING_ERROR_MIN_DEG,
            "tracking_error_scale": TRACKING_ERROR_SCALE,
            "tracking_error_max_deg": TRACKING_ERROR_MAX_DEG,
            "tracking_error_note": "small-motion abort sanity guard, not a measured servo capability",
            "parameters_modified": False,
        },
        "telemetry": {
            "poll_period_ms": args.poll_period_ms,
            "resample_dt_ms": 20,
            "velocity_estimator": "20ms linear resample + centered local quadratic fit",
            "raw_speed_is_calibrated": False,
        },
        "preflight": {"captures": [], "decision": None},
        "conditioning": {
            "enabled": args.precondition is not None,
            "direction": args.precondition,
            "settle_ms": CONDITIONING_SETTLE_MS,
            "formal_run_started": False,
            "conditioning_passed": None,
            "captures": [],
            "metrics": None,
        },
        "execution": {
            "max_runs": args.max_runs,
            "conditioning_only": args.conditioning_only,
            "executed_runs": 0,
            "stop_reason": "not_started",
        },
        "planned_runs": planned,
        "runs": [],
    }


def _write_json(path: Path, value: dict) -> None:
    path.write_text(json.dumps(value, indent=2, sort_keys=False) + "\n", encoding="utf-8")


def _analyze_manifest(output_dir: Path, manifest_path: Path, *, max_load_raw: float) -> dict:
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    metrics: list[dict] = []
    events: list[dict] = []
    run_rows: list[tuple[dict, list[dict[str, float]]]] = []
    for metadata in manifest.get("runs", []):
        path = output_dir / metadata["log"]
        rows = _raw_rows(path, int(metadata["joint"]))
        metric, run_events = analyze_run(rows, metadata)
        metric["safety_reasons"] = _safety_reasons(metric, max_load_raw)
        metric["safety_passed"] = not metric["safety_reasons"]
        metrics.append(metric)
        events.extend([{**event, "run": metadata["run"]} for event in run_events])
        run_rows.append((metadata, rows))
        metadata["sha256"] = _sha256(path)
        metadata["rows"] = len(rows)
    normalized = output_dir / "normalized.csv"
    _write_normalized_csv(normalized, run_rows)
    report = {
        "protocol": manifest.get("protocol"),
        "firmware": manifest.get("firmware", {}),
        "joint_selection": manifest.get("joint_selection", {}),
        "safety_policy": manifest.get("safety_policy", {}),
        "preflight": manifest.get("preflight", {"captures": [], "decision": None}),
        "conditioning": manifest.get("conditioning", {"enabled": False}),
        "execution": manifest.get("execution", {}),
        "runs": metrics,
        "feedback_jump_events": events,
        "direction_asymmetry": _direction_asymmetry(metrics),
        "low_speed_observations": [
            {
                "run": metric["run"],
                "tier": metric.get("tier"),
                "direction": metric["direction"],
                "observed_peak_velocity_deg_s": metric["observed_peak_velocity_deg_s"],
                "motion_onset_latency_ms": metric["motion_onset_latency_ms"],
                "feedback_jump_events": metric["feedback_jump_events"],
                "settling_time_ms": metric["settling_time_ms"],
            }
            for metric in metrics
            if str(metric.get("tier", "")).startswith("low_speed_")
        ],
        "reversal_lost_motion_proxy": [
            {"run": metric["run"], "reversal_proxy": metric["reversal_proxy"]}
            for metric in metrics
            if metric.get("reversal_proxy")
        ],
        "data_quality": {
            "raw_captures_unchanged": True,
            "speed_cmd_raw_forms": ["scalar", "historical five-element array"],
            "speed_cmd_raw_note": (
                "The parser accepts the current scalar sync-write field and the "
                "historical per-joint array form."
            ),
            "feedback_speed_encoding_verified": False,
            "derived_velocity_is_calibrated": False,
        },
        "summary": {
            "all_runs_passed_safety": all(metric["safety_passed"] for metric in metrics) if metrics else False,
            "raw_capture_count": len(run_rows),
            "normalized_csv": str(normalized),
            "raw_speed_note": "fb_speed_raw is reported only as a raw servo value; it is not treated as calibrated angular velocity.",
            "acceleration_note": "No physical acceleration or jerk limit is inferred from sparse quantized feedback.",
            "metric_interpretation": {
                "encoder_resolution_deg_per_count": DEG_PER_COUNT,
                "quantization_sensitive_max_counts": QUANTIZATION_SENSITIVE_MAX_COUNTS,
                "peak_velocity_is_calibrated": False,
                "onset_latency_is_pure_transport_latency": False,
            },
        },
    }
    _write_json(output_dir / "dynamics_report.json", report)
    manifest["analysis"] = {
        "report": "dynamics_report.json",
        "normalized_csv": "normalized.csv",
        "raw_sha256": {metadata["run"]: metadata.get("sha256", "") for metadata in manifest.get("runs", [])},
    }
    _write_json(manifest_path, manifest)
    return report


def _run_conditioning(args: argparse.Namespace, output_dir: Path, manifest: dict) -> dict:
    """Run and gate the optional directional conditioning prelude."""

    if not args.precondition:
        return manifest.get("conditioning", {"enabled": False})
    entry = _conditioning_entry(args, args.precondition)
    command = _command_for(entry)
    motion_name = "conditioning_motion.log"
    settle_name = "conditioning_settle_status.log"
    motion_path = _new_output(output_dir / motion_name)
    settle_path = _new_output(output_dir / settle_name)
    preflight_status = manifest["preflight"]["decision"]["status"]
    center_record = next(
        (record for record in preflight_status.get("records", []) if record["id"] == args.joint + 1),
        None,
    )
    if center_record is None:
        raise SafetyGateAbort("conditioning center feedback is unavailable after preflight")
    center_count = float(center_record["fb_pos"])
    metadata = {
        **entry,
        "run": "conditioning",
        "log": motion_name,
        "command": command,
    }
    print(
        f"conditioning {entry['direction']}: {entry['description']}; "
        f"{command} (physical motion; supervise the arm)",
        flush=True,
    )
    _capture_command(args.port, command, motion_path, int(entry["duration_ms"]))

    # Settling is an explicit wall-clock observation after the reversal sweep.
    # No command is sent during this interval; the subsequent status query is
    # kept separate from the conditioning telemetry capture.
    time.sleep(CONDITIONING_SETTLE_MS / 1000.0)
    _capture_command(args.port, "mlab status", settle_path, 500, tail_s=0.5)
    settle_status = _parse_status_capture(settle_path)
    settle_record = next(
        (record for record in settle_status.get("records", []) if record["id"] == args.joint + 1),
        None,
    )
    settled_feedback_count = None if settle_record is None else float(settle_record["fb_pos"])
    summary = _conditioning_metrics(
        _raw_rows(motion_path, args.joint),
        metadata,
        center_count,
        settled_feedback_count,
        settle_status,
    )
    reasons = _conditioning_health_reasons(summary, args)
    summary["conditioning_health_reasons"] = reasons
    summary["conditioning_passed"] = not reasons
    return {
        "enabled": True,
        "direction": entry["direction"],
        "description": entry["description"],
        "settle_ms": CONDITIONING_SETTLE_MS,
        "formal_run_started": False,
        "conditioning_passed": summary["conditioning_passed"],
        "captures": [
            {
                "name": motion_name,
                "command": command,
                "duration_ms": entry["duration_ms"],
                "log": motion_name,
                "sha256": _sha256(motion_path),
            },
            {
                "name": settle_name,
                "command": "mlab status",
                "duration_ms": 500,
                "log": settle_name,
                "sha256": _sha256(settle_path),
            },
        ],
        "metrics": summary,
    }


def _run_hardware(args: argparse.Namespace, output_dir: Path, manifest: dict, groups: list[dict]) -> None:
    if not args.port:
        raise ValueError("--port is required with --execute")
    if not args.confirm_hardware:
        raise ValueError("physical execution requires --confirm-hardware")
    output_dir.mkdir(parents=True, exist_ok=True)
    executed_runs = 0
    try:
        # Establish factory-like runtime context and preserve read-only identity
        # captures before the first movement.
        preflight = []
        for name, command, duration in (
            ("preflight_poll_off.log", "mlab poll off", 350),
            ("preflight_comp_off.log", "mlab comp off", 350),
            ("preflight_status.log", "mlab status", 500),
            # The snapshot reads every relevant register for all five IDs. It
            # is intentionally given time to complete before any movement.
            ("preflight_params.log", "mlab params", 15000),
            ("preflight_voltage.log", "mlab voltage", 500),
            ("preflight_poll_on.log", f"mlab poll {args.joint} {args.poll_period_ms}", 350),
        ):
            try:
                path = _new_output(output_dir / name)
                _capture_command(args.port, command, path, duration, tail_s=0.5)
            except Exception as error:
                manifest["preflight"] = {
                    "captures": preflight,
                    "decision": {
                        "passed": False,
                        "reasons": [f"preflight command {command!r} failed: {error}"],
                    },
                }
                manifest["status"] = "preflight_failed"
                manifest["execution"]["stop_reason"] = "preflight_capture"
                _write_json(output_dir / "manifest.json", manifest)
                raise SafetyGateAbort(f"preflight capture failed for {command}: {error}") from error
            preflight.append({
                "name": name,
                "command": command,
                "duration_ms": duration,
                "log": name,
                "sha256": _sha256(path),
            })
        manifest["preflight"] = {"captures": preflight, "decision": None}
        decision = _preflight_decision(
            output_dir / "preflight_status.log",
            output_dir / "preflight_params.log",
            output_dir / "preflight_voltage.log",
        )
        manifest["preflight"]["decision"] = decision
        if not decision["passed"]:
            manifest["status"] = "preflight_failed"
            manifest["execution"]["stop_reason"] = "preflight_gate"
            _write_json(output_dir / "manifest.json", manifest)
            raise SafetyGateAbort("preflight gate failed: " + "; ".join(decision["reasons"]))
        if args.precondition:
            try:
                conditioning = _run_conditioning(args, output_dir, manifest)
            except SafetyGateAbort as error:
                # A missing center snapshot or an equivalent conditioning
                # prelude failure is itself a failed health gate. Persist that
                # decision so the report is explicit and never looks like a
                # generic user abort.
                conditioning = {
                    **manifest.get("conditioning", {}),
                    "enabled": True,
                    "direction": args.precondition,
                    "conditioning_passed": False,
                    "formal_run_started": False,
                    "metrics": {
                        "conditioning_passed": False,
                        "conditioning_health_reasons": [str(error)],
                    },
                }
                manifest["conditioning"] = conditioning
                manifest["status"] = "conditioning_failed"
                manifest["execution"]["stop_reason"] = "conditioning_gate"
                _write_json(output_dir / "manifest.json", manifest)
                raise
            except Exception as error:
                # Serial/capture failures are also fail-closed. No formal
                # movement may begin after an incomplete conditioning prelude.
                conditioning = {
                    **manifest.get("conditioning", {}),
                    "enabled": True,
                    "direction": args.precondition,
                    "conditioning_passed": False,
                    "formal_run_started": False,
                    "metrics": {
                        "conditioning_passed": False,
                        "conditioning_health_reasons": [
                            f"conditioning capture failed: {error}"
                        ],
                    },
                }
                manifest["conditioning"] = conditioning
                manifest["status"] = "conditioning_failed"
                manifest["execution"]["stop_reason"] = "conditioning_capture"
                _write_json(output_dir / "manifest.json", manifest)
                raise SafetyGateAbort(f"conditioning capture failed: {error}") from error
            # No conditioning helper may claim that the formal phase started
            # before this gate returns; the only assignment that flips this
            # field is immediately below, after a successful non-only gate.
            conditioning["formal_run_started"] = False
            manifest["conditioning"] = conditioning
            _write_json(output_dir / "manifest.json", manifest)
            if not conditioning["conditioning_passed"]:
                manifest["status"] = "conditioning_failed"
                manifest["execution"]["stop_reason"] = "conditioning_gate"
                _write_json(output_dir / "manifest.json", manifest)
                raise SafetyGateAbort(
                    "conditioning health gate failed: "
                    + "; ".join(conditioning["metrics"]["conditioning_health_reasons"])
                )
            if args.conditioning_only:
                manifest["status"] = "conditioning_complete"
                manifest["execution"]["stop_reason"] = "conditioning_only"
                manifest["conditioning"]["formal_run_started"] = False
                _write_json(output_dir / "manifest.json", manifest)
                print("conditioning-only complete; formal dynamics run not started", flush=True)
                return
            manifest["conditioning"]["formal_run_started"] = True
        manifest["status"] = "running"
        manifest["execution"]["stop_reason"] = "running"
        _write_json(output_dir / "manifest.json", manifest)
        last_group = None
        for group_name, entry in _execution_entries(groups, args.max_runs):
            if group_name != last_group:
                print(f"starting supervised tier: {group_name}", flush=True)
                last_group = group_name
            name = f"j{entry['joint']}_{entry['tier']}_{'pos' if entry['amplitude_deg'] > 0 else 'neg'}_r{entry['repetition']}"
            metadata = {**entry, "run": name, "log": f"{name}.log", "command": _command_for(entry)}
            print(f"  {name}: {metadata['command']}  (physical motion; supervise the arm)", flush=True)
            _capture_command(
                args.port,
                metadata["command"],
                _new_output(output_dir / metadata["log"]),
                int(entry["duration_ms"]),
            )
            rows = _raw_rows(output_dir / metadata["log"], args.joint)
            metric, _events = analyze_run(rows, metadata)
            reasons = _safety_reasons(metric, args.max_load_raw)
            metadata["sha256"] = _sha256(output_dir / metadata["log"])
            metadata["safety_reasons"] = reasons
            metadata["safety_passed"] = not reasons
            manifest["runs"].append(metadata)
            executed_runs += 1
            manifest["execution"]["executed_runs"] = executed_runs
            _write_json(output_dir / "manifest.json", manifest)
            if reasons:
                manifest["execution"]["stop_reason"] = "run_safety_gate"
                raise SafetyGateAbort(f"{name}: safety gate failed: {'; '.join(reasons)}")
        manifest["execution"]["stop_reason"] = "max_runs" if executed_runs >= args.max_runs else "plan_complete"
        print(f"execution limit reached after {executed_runs} movement(s); returning control", flush=True)
        manifest["status"] = "complete"
    except (KeyboardInterrupt, SafetyGateAbort):
        if manifest.get("status") not in ("preflight_failed", "conditioning_failed"):
            manifest["status"] = "aborted"
        raise
    finally:
        # Restore normal polling and Motion Lab ownership even on user abort or
        # a failed gate. These are runtime commands; they do not touch EEPROM.
        cleanup_commands = (
            ("cleanup_stop.log", "mlab stop"),
            ("cleanup_poll_off.log", "mlab poll off"),
            ("cleanup_comp_off.log", "mlab comp off"),
        )
        for name, command in cleanup_commands:
            try:
                _capture_command(args.port, command, _new_output(output_dir / name), 350, tail_s=0.5)
            except Exception as error:  # pragma: no cover - hardware cleanup path
                print(f"warning: cleanup command failed ({command}): {error}", file=sys.stderr)
        manifest["execution"]["executed_runs"] = executed_runs
        _write_json(output_dir / "manifest.json", manifest)


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", help="serial port; required only with --execute")
    parser.add_argument("--output-dir", type=Path, default=Path("backups") / f"motion-lab-dynamics-{date.today().isoformat()}")
    parser.add_argument("--joint", type=int, default=DEFAULT_JOINT, choices=range(5))
    parser.add_argument("--amplitudes-deg", type=lambda value: _parse_int_list(value, "amplitudes-deg"), default=DEFAULT_AMPLITUDES)
    parser.add_argument("--repetitions", type=int, default=3)
    parser.add_argument(
        "--max-runs",
        type=int,
        default=1,
        help="maximum physical movement captures for this invocation (default: 1; use a larger value only after review)",
    )
    parser.add_argument("--tiers", type=lambda value: tuple(part.strip() for part in value.split(",") if part.strip()), default=tuple(tier.name for tier in DEFAULT_TIERS))
    parser.add_argument("--low-speed-deg-s", type=lambda value: _parse_int_list(value, "low-speed-deg-s"), default=(2, 4, 8))
    parser.add_argument("--low-speed-amplitude-deg", type=int, default=3)
    parser.add_argument("--reversal-tier", choices=tuple(tier.name for tier in DEFAULT_TIERS), default="moderate")
    parser.add_argument("--reversal-amplitude-deg", type=int, default=3)
    parser.add_argument("--poll-period-ms", type=int, default=DEFAULT_POLL_PERIOD_MS)
    parser.add_argument("--deadband-mdeg", type=int, default=DEFAULT_DEADBAND_MDEG)
    parser.add_argument(
        "--precondition",
        choices=("positive", "negative"),
        help=(
            "optionally run a directional center -> +/-5 -> -/+5 -> center "
            "health motion before formal measurements; disabled by default"
        ),
    )
    parser.add_argument(
        "--conditioning-only",
        action="store_true",
        help=(
            "run exactly one selected conditioning prelude and settle/status "
            "capture; never start a formal dynamics run"
        ),
    )
    parser.add_argument("--max-load-raw", type=float, default=3000.0)
    parser.add_argument("--skip-low-speed", action="store_true")
    parser.add_argument("--skip-reversal", action="store_true")
    parser.add_argument("--execute", action="store_true", help="run supervised hardware captures; dry-run is the default")
    parser.add_argument("--confirm-hardware", action="store_true", help="required with --execute")
    parser.add_argument("--manifest-only", action="store_true", help="explicitly request the default no-hardware plan")
    parser.add_argument("--analyze-only", action="store_true", help="analyze an existing manifest/capture directory only")
    return parser


def main() -> int:
    parser = _build_parser()
    args = parser.parse_args()
    if args.repetitions < 1 or args.repetitions > 5:
        parser.error("repetitions must be 1..5")
    if args.max_runs < 1 or args.max_runs > 1000:
        parser.error("max-runs must be 1..1000")
    if any(abs(amplitude) < 1 or abs(amplitude) > 10 for amplitude in args.amplitudes_deg):
        parser.error("amplitudes must remain within 1..10 degrees")
    if args.poll_period_ms < 5 or args.poll_period_ms > 100:
        parser.error("poll period must remain 5..100 ms")
    if args.max_load_raw <= 0:
        parser.error("max-load-raw must be positive")
    if args.execute and args.manifest_only:
        parser.error("--execute and --manifest-only are mutually exclusive")
    if args.execute and not args.port:
        parser.error("--port is required with --execute")
    if args.execute and not args.confirm_hardware:
        parser.error("physical execution requires --confirm-hardware")
    if args.conditioning_only and not args.execute:
        parser.error("--conditioning-only requires --execute")
    if args.conditioning_only and not args.precondition:
        parser.error("--conditioning-only requires --precondition positive|negative")
    if any(speed < 1 or speed > 45 for speed in args.low_speed_deg_s):
        parser.error("low-speed values must remain 1..45 deg/s")
    tiers_by_name = {tier.name for tier in DEFAULT_TIERS}
    unknown_tiers = set(args.tiers) - tiers_by_name
    if unknown_tiers:
        parser.error(f"unknown tier(s): {', '.join(sorted(unknown_tiers))}")
    groups, planned = _build_plan(args)
    if not planned:
        parser.error("the plan contains no movement captures; select at least one tier or probe")
    output_dir = args.output_dir
    output_dir.mkdir(parents=True, exist_ok=True)
    manifest_path = output_dir / "manifest.json"
    if args.analyze_only:
        if not manifest_path.exists():
            parser.error(f"--analyze-only requires {manifest_path}")
        report = _analyze_manifest(output_dir, manifest_path, max_load_raw=args.max_load_raw)
        print(json.dumps(report["summary"], indent=2))
        print(f"report: {output_dir / 'dynamics_report.json'}")
        return 0
    if manifest_path.exists():
        parser.error(f"refusing to overwrite existing manifest: {manifest_path}; choose a new output directory or use --analyze-only")
    manifest = _initial_manifest(args, planned)
    _print_plan(args, groups, planned)
    _write_json(manifest_path, manifest)
    if not args.execute:
        print(f"dry-run only; manifest: {manifest_path}")
        print("To run only the first movement under direct human supervision, repeat with --execute --confirm-hardware --max-runs 1 --port <PORT>.")
        print(f"The first proposed movement is: {_command_for(planned[0])}")
        return 0
    exit_code = 0
    try:
        _run_hardware(args, output_dir, manifest, groups)
    except SafetyGateAbort as error:
        print(f"ABORTED: {error}", file=sys.stderr)
        exit_code = 2
    except KeyboardInterrupt:
        print("ABORTED: user interrupt", file=sys.stderr)
        exit_code = 130
    finally:
        if manifest.get("runs") or manifest.get("conditioning", {}).get("enabled"):
            report = _analyze_manifest(output_dir, manifest_path, max_load_raw=args.max_load_raw)
            print(json.dumps(report["summary"], indent=2))
            print(f"report: {output_dir / 'dynamics_report.json'}")
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
