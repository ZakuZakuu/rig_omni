#!/usr/bin/env python3
"""Search a small, reversible CW command-deadband compensation set.

This is intentionally a bounded screen, not an auto-calibration framework.  It
changes only the Motion Lab RAM profile; SCS009 EEPROM parameters are untouched.
The resulting candidates still require a human visual A/B comparison.
"""

from __future__ import annotations

import argparse
import json
import subprocess
import sys
from pathlib import Path

from characterize_stutter import parse_rows, summarize


CAPTURE_DURATION_MS = 5000
CAPTURE_TAIL_S = "1.5"
RUN_ARGS = "0 2 0 10 5000 0 15 60 250"


def capture(port: str, command: str, output: Path, duration_ms: int) -> None:
    capture_script = Path(__file__).with_name("capture_serial.py")
    subprocess.run(
        [
            sys.executable,
            str(capture_script),
            "--port",
            port,
            "--command",
            command,
            "--duration-ms",
            str(duration_ms),
            "--output",
            str(output),
            "--ready-timeout-s",
            "0.5",
            "--tail-s",
            CAPTURE_TAIL_S,
        ],
        check=True,
    )


def apply_profile(port: str, command: str, output: Path) -> None:
    capture(port, command, output, 300)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--repetitions", type=int, default=2)
    args = parser.parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)

    # Keep CCW at the normal 250 mdeg host deadband and screen only small CW
    # reductions.  The baseline is included as candidate A for fair comparison.
    candidates = [
        ("A_factory", None, 250),
        ("B_cw0", 0, 250),
        ("C_cw125", 125, 250),
    ]
    results: list[dict] = []
    try:
        for name, cw_mdeg, ccw_mdeg in candidates:
            profile_command = "mlab comp off" if cw_mdeg is None else (
                f"mlab comp 0 {cw_mdeg} {ccw_mdeg}"
            )
            apply_profile(args.port, profile_command, args.output_dir / f"{name}_profile.log")
            summaries = []
            for repetition in range(1, args.repetitions + 1):
                raw = args.output_dir / f"{name}_r{repetition}.log"
                capture(args.port, f"mlab run {RUN_ARGS}", raw, CAPTURE_DURATION_MS)
                rows = parse_rows(raw)
                if not rows:
                    raise RuntimeError(f"capture produced no telemetry rows: {raw}")
                summaries.append(summarize(rows, 15, repetition))
            stale = sum(item["stale_rows"] for item in summaries)
            voltage_min = min(item["voltage_min_v"] for item in summaries)
            voltage_max = max(item["voltage_max_v"] for item in summaries)
            jump_count = sum(len(item["jumps"]) for item in summaries)
            median_error = sum(item["tracking_error_median_counts"] for item in summaries) / len(summaries)
            max_error = max(item["tracking_error_max_counts"] for item in summaries)
            rejected = stale != 0 or voltage_min < 7.5 or voltage_max > 8.8
            # Tracking error is a guardrail, not the objective.  The score only
            # ranks clearly safe candidates before the human visual comparison.
            score = jump_count + 0.10 * median_error + 0.01 * max_error
            results.append({
                "candidate": name,
                "cw_deadband_mdeg": cw_mdeg,
                "ccw_deadband_mdeg": ccw_mdeg,
                "rejected": rejected,
                "reject_reason": "stale feedback or voltage out of range" if rejected else "",
                "jump_count": jump_count,
                "tracking_error_median_counts": median_error,
                "tracking_error_max_counts": max_error,
                "voltage_min_v": voltage_min,
                "voltage_max_v": voltage_max,
                "runs": summaries,
                "guardrail_score": score,
            })
    finally:
        # Leave the diagnostic profile disabled even if a capture fails.
        try:
            apply_profile(args.port, "mlab comp off", args.output_dir / "restore_comp_off.log")
        except Exception as error:  # pragma: no cover - hardware cleanup path
            print(f"warning: failed to restore compensation off: {error}", file=sys.stderr)

    safe = sorted((item for item in results if not item["rejected"]), key=lambda item: item["guardrail_score"])
    report = {
        "protocol": {
            "joint": 0,
            "direction": "positive / physical clockwise outbound half",
            "trajectory": "minimum-jerk",
            "amplitude_deg": 10,
            "duration_ms": CAPTURE_DURATION_MS,
            "max_velocity_deg_s": 15,
            "max_acceleration_deg_s2": 60,
            "host_deadband_mdeg": 250,
            "repetitions": args.repetitions,
        },
        "candidates": results,
        "safe_ranked_candidates": [item["candidate"] for item in safe[:3]],
        "note": "Telemetry ranking is only a guardrail; choose the final profile by human visual A/B.",
    }
    report_path = args.output_dir / "compensation_search_report.json"
    report_path.write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps({"safe_ranked_candidates": report["safe_ranked_candidates"]}, indent=2))
    print(f"report: {report_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
