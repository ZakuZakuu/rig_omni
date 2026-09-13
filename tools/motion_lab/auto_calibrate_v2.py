#!/usr/bin/env python3
"""Bounded Auto Calibration v0.2 for the repeatable joint-0 asymmetry.

This is a practical screen, not a physical friction-identification system. It
tries a small set of reversible SCS009/runtime profiles using the exact same
10-degree, 2-second minimum-jerk motion in both physical directions. The
existing analyzer supplies guardrails and a ranking; a human still chooses the
final profile by visual A/B/C comparison.
"""

from __future__ import annotations

import argparse
import csv
import json
import statistics
import subprocess
import sys
from dataclasses import dataclass
from pathlib import Path


RUN_DURATION_MS = 2000
RUN_TAIL_S = "1.5"
RUN_VELOCITY = 15
RUN_ACCELERATION = 60
RUN_DEADBAND = 250


@dataclass(frozen=True)
class Candidate:
    name: str
    servo_profile: str
    tune_commands: tuple[str, ...]
    runtime_profile: str
    runtime_command: str | None


CANDIDATES = (
    Candidate("A_factory", "factory", (), "off", None),
    Candidate("B_cw_deadband0", "cw_deadband=0", ("mlab tune cw_deadband 0",), "off", None),
    Candidate(
        "C_runtime_floor",
        "factory",
        (),
        "cw_db125_min1000_scale1100",
        "mlab comp profile 0 125 250 1000 0 1100 1000",
    ),
    Candidate(
        "D_p12_runtime",
        "p=12",
        ("mlab tune p 12",),
        "cw_db125_min1000_scale1100",
        "mlab comp profile 0 125 250 1000 0 1100 1000",
    ),
    Candidate(
        "E_startup32_runtime",
        "startup=32",
        ("mlab tune startup 32",),
        "cw_db125_min1000_scale1100",
        "mlab comp profile 0 125 250 1000 0 1100 1000",
    ),
    Candidate(
        "F_ccw_deadband0_runtime",
        "ccw_deadband=0",
        ("mlab tune ccw_deadband 0",),
        "cw_db125_min1000_scale1100",
        "mlab comp profile 0 125 0 1000 0 1100 1000",
    ),
)


def capture(port: str, command: str, output: Path, duration_ms: int, tail_s: str = RUN_TAIL_S) -> None:
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
            tail_s,
        ],
        check=True,
    )


def apply_candidate(port: str, output_dir: Path, candidate: Candidate) -> None:
    # Restore before every candidate so no EEPROM setting leaks across groups.
    capture(port, "mlab tune restore", output_dir / f"{candidate.name}_restore.log", 350, "0.5")
    for index, command in enumerate(candidate.tune_commands):
        capture(port, command, output_dir / f"{candidate.name}_tune_{index}.log", 350, "0.5")
    profile_command = candidate.runtime_command or "mlab comp off"
    capture(port, profile_command, output_dir / f"{candidate.name}_profile.log", 350, "0.5")
    capture(port, "mlab params", output_dir / f"{candidate.name}_params.log", 350, "0.5")
    capture(port, "mlab comp show", output_dir / f"{candidate.name}_comp_show.log", 350, "0.5")


def aggregate_metrics(input_dir: Path, manifest: dict) -> list[dict]:
    metrics_path = input_dir / "analysis" / "metrics.csv"
    with metrics_path.open(newline="") as handle:
        rows = list(csv.DictReader(handle))
    by_candidate: dict[str, list[dict]] = {}
    for row in rows:
        by_candidate.setdefault(row["candidate"], []).append(row)
    aggregates: list[dict] = []
    for candidate in CANDIDATES:
        values = by_candidate.get(candidate.name, [])
        if not values:
            continue
        def mean(key: str) -> float:
            return statistics.mean(float(item[key]) for item in values)
        def maximum(key: str) -> float:
            return max(float(item[key]) for item in values)
        aggregates.append({
            "candidate": candidate.name,
            "servo_profile": candidate.servo_profile,
            "runtime_profile": candidate.runtime_profile,
            "runs": len(values),
            "quality_score_mean": mean("quality_score"),
            "quality_score_p95": sorted(float(item["quality_score"]) for item in values)[min(len(values) - 1, int(round((len(values) - 1) * 0.95)))],
            "events_total": sum(int(item["events"]) for item in values),
            "dwell_ms_total": sum(float(item["dwell_ms_total"]) for item in values),
            "tracking_error_abs_p95_max_counts": maximum("tracking_error_abs_p95_counts"),
            "endpoint_overshoot_max_deg": maximum("endpoint_overshoot_deg"),
            "static_hold_jitter_max_counts": maximum("static_hold_jitter_counts"),
            "stale_rows": sum(int(item["stale_rows"]) for item in values),
            "voltage_min_v": min(float(item["voltage_min_v"]) for item in values),
            "voltage_max_v": max(float(item["voltage_max_v"]) for item in values),
            "cw_quality_score_mean": mean_direction(values, "CW"),
            "ccw_quality_score_mean": mean_direction(values, "CCW"),
        })
    return aggregates


def mean_direction(rows: list[dict], direction: str) -> float:
    values = [float(row["quality_score"]) for row in rows if row["direction"] == direction]
    return statistics.mean(values) if values else 0.0


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--repetitions", type=int, default=2)
    args = parser.parse_args()
    if args.repetitions < 1 or args.repetitions > 4:
        parser.error("repetitions must be 1..4")
    args.output_dir.mkdir(parents=True, exist_ok=True)
    runs: list[dict] = []
    try:
        for candidate in CANDIDATES:
            print(f"v0.2 screen: {candidate.name} ({candidate.servo_profile}; {candidate.runtime_profile})", flush=True)
            apply_candidate(args.port, args.output_dir, candidate)
            for repetition in range(1, args.repetitions + 1):
                for direction, amplitude in (("CW", 10), ("CCW", -10)):
                    run_name = f"{candidate.name}_{direction.lower()}_r{repetition}"
                    print(f"  run {run_name}: physical {direction}, +/−10 deg endpoint, 2 s", flush=True)
                    log_name = f"{run_name}.log"
                    command = (
                        f"mlab run 0 2 0 {amplitude} {RUN_DURATION_MS} 0 {RUN_VELOCITY} "
                        f"{RUN_ACCELERATION} {RUN_DEADBAND}"
                    )
                    capture(args.port, command, args.output_dir / log_name, RUN_DURATION_MS)
                    runs.append({
                        "run": run_name,
                        "candidate": candidate.name,
                        "log": log_name,
                        "joint": 0,
                        "direction": direction,
                        "amplitude_deg": amplitude,
                        "duration_ms": RUN_DURATION_MS,
                        "max_velocity_deg_s": RUN_VELOCITY,
                        "max_acceleration_deg_s2": RUN_ACCELERATION,
                        "deadband_mdeg": RUN_DEADBAND,
                        "repetition": repetition,
                        "servo_profile": candidate.servo_profile,
                        "runtime_profile": candidate.runtime_profile,
                    })
    finally:
        # Persistent factory restore and RAM compensation off are both required
        # even when a serial capture fails midway through the screen.
        try:
            capture(args.port, "mlab tune restore", args.output_dir / "final_factory_restore.log", 350, "0.5")
            capture(args.port, "mlab comp off", args.output_dir / "final_comp_off.log", 350, "0.5")
        except Exception as error:  # pragma: no cover - hardware cleanup path
            print(f"warning: failed to restore factory-like state: {error}", file=sys.stderr)

    manifest = {
        "protocol": {
            "version": "auto-calibration-v0.2",
            "joint": 0,
            "trajectory": "minimum-jerk",
            "amplitude_deg": 10,
            "duration_ms": RUN_DURATION_MS,
            "max_velocity_deg_s": RUN_VELOCITY,
            "max_acceleration_deg_s2": RUN_ACCELERATION,
            "command_deadband_mdeg": RUN_DEADBAND,
            "directions": ["CW", "CCW"],
            "repetitions": args.repetitions,
            "candidate_count": len(CANDIDATES),
            "d_note": "D gain not screened: no endpoint ringing was established in v0.1.",
        },
        "runs": runs,
    }
    manifest_path = args.output_dir / "manifest.json"
    manifest_path.write_text(json.dumps(manifest, indent=2) + "\n")
    analyzer = Path(__file__).with_name("analyze_stutter.py")
    subprocess.run([sys.executable, str(analyzer), "--input-dir", str(args.output_dir)], check=True)
    aggregates = aggregate_metrics(args.output_dir, manifest)
    baseline = next((item for item in aggregates if item["candidate"] == "A_factory"), None)
    if baseline is None:
        raise RuntimeError("factory baseline was not captured")
    for item in aggregates:
        reasons: list[str] = []
        if item["stale_rows"]:
            reasons.append("stale feedback")
        if item["voltage_min_v"] < 7.5 or item["voltage_max_v"] > 8.8:
            reasons.append("voltage outside 7.5..8.8 V")
        if item["tracking_error_abs_p95_max_counts"] > max(50.0, baseline["tracking_error_abs_p95_max_counts"] * 1.5):
            reasons.append("tracking error guardrail")
        if item["endpoint_overshoot_max_deg"] > max(1.5, baseline["endpoint_overshoot_max_deg"] + 1.0):
            reasons.append("overshoot guardrail")
        item["rejected"] = bool(reasons)
        item["reject_reason"] = "; ".join(reasons)
        # Score is a telemetry guardrail only. The final choice remains visual.
        item["guardrail_score"] = item["quality_score_mean"]
    safe = sorted((item for item in aggregates if not item["rejected"]), key=lambda item: item["guardrail_score"])
    report = {
        "protocol": manifest["protocol"],
        "candidates": aggregates,
        "safe_ranked_candidates": [item["candidate"] for item in safe[:3]],
        "baseline": baseline,
        "note": "Ranking rejects unsafe regressions but does not claim perceptual superiority; compare the top candidates visually.",
    }
    report_path = args.output_dir / "auto_calibration_v0.2_report.json"
    report_path.write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps({"safe_ranked_candidates": report["safe_ranked_candidates"]}, indent=2))
    print(f"report: {report_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
