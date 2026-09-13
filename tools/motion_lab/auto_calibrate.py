#!/usr/bin/env python3
"""Run a bounded, bidirectional joint-0 calibration capture matrix.

The script only issues Motion Lab commands and read-only parameter snapshots.
It never writes servo EEPROM and always disables the RAM compensation profile in
its cleanup path.  Analysis is delegated to analyze_stutter.py.
"""

from __future__ import annotations

import argparse
import json
import subprocess
import sys
from pathlib import Path


def capture(port: str, command: str, output: Path, duration_ms: int, tail_s: str = "1.5") -> None:
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
            "0.5",
            "--tail-s",
            tail_s,
        ],
        check=True,
    )


def ints(value: str) -> list[int]:
    return [int(part.strip()) for part in value.split(",") if part.strip()]


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--repetitions", type=int, default=2)
    parser.add_argument("--durations-ms", default="3000,5000")
    parser.add_argument("--velocities-deg-s", default="8,15")
    parser.add_argument("--amplitude-deg", type=int, default=10)
    parser.add_argument("--acceleration-deg-s2", type=int, default=60)
    parser.add_argument("--deadband-mdeg", type=int, default=250)
    args = parser.parse_args()
    if args.amplitude_deg < 8 or args.amplitude_deg > 10:
        parser.error("amplitude must stay in the conservative 8..10 degree range")
    if args.repetitions < 1 or args.repetitions > 5:
        parser.error("repetitions must be 1..5")
    durations = ints(args.durations_ms)
    velocities = ints(args.velocities_deg_s)
    if any(duration < 2000 or duration > 10000 for duration in durations):
        parser.error("durations must be 2000..10000 ms")
    if any(1 > velocity or velocity > 60 for velocity in velocities):
        parser.error("velocity limits must be 1..60 deg/s")
    args.output_dir.mkdir(parents=True, exist_ok=True)

    # Ensure the runtime-only profile cannot bias the characterization.  This
    # does not touch the factory servo registers.
    capture(args.port, "mlab comp off", args.output_dir / "comp_off.log", 300, "0.5")
    capture(args.port, "mlab params", args.output_dir / "factory_params.log", 300, "0.5")
    runs: list[dict] = []
    try:
        for duration_ms in durations:
            for velocity in velocities:
                for direction in (1, -1):
                    direction_name = "cw" if direction > 0 else "ccw"
                    amplitude = direction * args.amplitude_deg
                    for repetition in range(1, args.repetitions + 1):
                        name = f"j0_{direction_name}_d{duration_ms}_v{velocity}_r{repetition}"
                        log = f"{name}.log"
                        command = (
                            f"mlab run 0 2 0 {amplitude} {duration_ms} 0 {velocity} "
                            f"{args.acceleration_deg_s2} {args.deadband_mdeg}"
                        )
                        capture(args.port, command, args.output_dir / log, duration_ms, "1.5")
                        runs.append({
                            "run": name,
                            "log": log,
                            "joint": 0,
                            "direction": direction_name,
                            "amplitude_deg": amplitude,
                            "duration_ms": duration_ms,
                            "max_velocity_deg_s": velocity,
                            "max_acceleration_deg_s2": args.acceleration_deg_s2,
                            "deadband_mdeg": args.deadband_mdeg,
                            "repetition": repetition,
                        })
    finally:
        # Leave factory-like runtime behavior even if a serial capture fails.
        try:
            capture(args.port, "mlab comp off", args.output_dir / "comp_off_final.log", 300, "0.5")
        except Exception as error:  # pragma: no cover - hardware cleanup path
            print(f"warning: failed to disable runtime compensation: {error}", file=sys.stderr)

    manifest = {
        "protocol": {
            "joint": 0,
            "amplitude_deg": args.amplitude_deg,
            "durations_ms": durations,
            "velocities_deg_s": velocities,
            "directions": ["CW", "CCW"],
            "repetitions": args.repetitions,
            "trajectory": "minimum-jerk",
            "acceleration_limiter_deg_s2": args.acceleration_deg_s2,
            "command_deadband_mdeg": args.deadband_mdeg,
        },
        "runs": runs,
    }
    manifest_path = args.output_dir / "manifest.json"
    manifest_path.write_text(json.dumps(manifest, indent=2) + "\n")
    analyzer = Path(__file__).with_name("analyze_stutter.py")
    subprocess.run(
        [sys.executable, str(analyzer), "--input-dir", str(args.output_dir)],
        check=True,
    )
    print(f"manifest: {manifest_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
