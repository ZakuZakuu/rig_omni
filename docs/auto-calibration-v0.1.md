# Joint-0 auto-calibration prototype (v0.1)

This prototype is deliberately small: it characterizes the root joint with
repeatable Motion Lab captures, emits inspectable metrics/plots, and keeps the
result in a replaceable profile file. It is not a research-grade actuator
identification system and it never adds an external PID loop.

## Run

Start from the verified factory servo snapshot and leave Motion Lab idle. With
the ESP-IDF Python environment active:

```bash
python tools/motion_lab/auto_calibrate.py \
  --port "$RIG_PORT" \
  --output-dir backups/motion-lab-YYYY-MM-DD-auto-calibration
```

The default matrix is joint 0, fixed 10° endpoint, minimum-jerk, 3 s and 5 s
out-and-back durations, velocity limits 8 and 15°/s, both physical directions,
and two repetitions per condition. The acceleration limiter is fixed at 60°/s²
and the host command deadband at 0.25°. These values keep the arm well inside
the conservative diagnostic range and avoid the old 1.5 s endpoint overshoot.
Use `--repetitions`, `--durations-ms`, and `--velocities-deg-s` only for a
clearly documented follow-up matrix.

The script first captures `mlab params` (read-only), disables the RAM-only
Motion Lab compensation profile, drains each serial session to a raw log, and
always attempts to disable the profile again on exit. It does not issue any
SCS009 EEPROM write.

## Outputs

- `manifest.json`: exact protocol and per-run metadata;
- `*.log`: immutable UART captures;
- `analysis/metrics.csv`: per-run low-level motion metrics;
- `analysis/normalized.csv`: 50 ms fixed-grid derived telemetry for comparable
  velocity/error metrics (raw event timing is kept separately);
- `analysis/events.jsonl`: dwell/jump records with position, direction,
  commanded velocity, measured speed raw, load, dwell duration, jump size, and
  timestamp. Metrics also include reversal delay and command travel until the
  feedback moves two counts in the new direction (a backlash-style proxy, not a
  mechanical free-play measurement);
- `analysis/classification.json`: machine-readable category candidates and
  confidence;
- `analysis/joint0_motion.svg`: dependency-free command/feedback plot.

The analyzer can be rerun without touching hardware:

```bash
python tools/motion_lab/analyze_stutter.py \
  --input-dir backups/motion-lab-YYYY-MM-DD-auto-calibration
```

Event detection is conservative. A dwell requires continuing command motion
with feedback nearly stationary; a jump then requires a same-direction feedback
step of at least four counts. Ordinary one-count quantization is not counted.
Scores combine dwell, jump frequency/magnitude, tracking error, static hold
jitter, and endpoint overshoot. They reject clearly bad telemetry but do not
choose the perceptual winner.

## Classification and compensation policy

- `position-locked`: repeated event positions cluster across durations; only
  then consider a small direction/position LUT;
- `velocity-dependent`: low-speed scores are materially worse; consider a
  direction-specific minimum smooth velocity or timing adjustment;
- `time-frequency-locked`: event interval is stable despite changed speed and
  position; investigate internal servo deadband/P behavior before compensation;
- `mixed-or-hardware-limited`: do not invent a model; retain the factory
  profile and document the practical hardware limit.

The reusable starting profile is
`calibration/profiles/joint0.yaml`. It is intentionally all-null/factory. Any
candidate runtime profile must remain bypassable and be presented as no more
than three clearly labelled A/B/C motions for human visual evaluation. A
numerically lower score is only a guardrail; the final profile is not selected
without the human comparison.

## Safety and rollback

Keep the arm supported and away from pinch zones. Stop the experiment with
`mlab stop`, then disable runtime compensation with `mlab comp off`. Restore
factory servo registers with the already verified `mlab tune restore` path only
when explicitly authorized; this is not part of the normal auto-calibration
capture. Never alter I or add an external PID loop. If the same profile is not
materially better in both directions, stop and move on to Creature Motion.
