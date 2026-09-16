# SCS009 usable dynamics characterization

Status: protocol and harness implemented; supervised hardware data pending.
This PR does not change servo EEPROM, persistent speed/time registers, P/I/D,
deadband, startup force, simulation limits, or production Ruckig defaults.

## Research question

The goal is the usable dynamic envelope of the assembled RIG-Arm: repeatable
small/medium expressive motion with acceptable tracking, feedback freshness,
voltage, load, and mechanical behavior. This is not a destructive maximum-speed
test and does not convert the vendor's no-load speed into a hardware limit.

The source review, including the SCS009/C013 identity and the unresolved
7.9–8.1 V versus 7.4 V nominal-range discrepancy, is in
[`docs/research/scs009-dynamics-characterization.md`](research/scs009-dynamics-characterization.md).

## Current protocol

The first representative axis is J2: a visible forearm-pitch joint whose
validated neutral pose is away from the model limits. It is not selected merely
because earlier J0 work found a stutter. The harness keeps the current factory
parameters and begins with ±3°, then ±5°, then ±10° minimum-jerk motions. Each
condition has at least three repetitions and a one-second endpoint hold before
return. Tiers are deliberately progressive:

| Tier | Transition | Velocity limit | Acceleration limit |
| --- | ---: | ---: | ---: |
| gentle | 3.0 s | 8°/s | 30°/s² |
| moderate | 2.2 s | 15°/s | 60°/s² |
| brisk | 1.4 s | 30°/s | 120°/s² |
| expressive | 1.0 s | 45°/s | 180°/s² |

Three additional ±3° low-speed probes at 2/4/8°/s estimate the practical
continuous-motion floor. A separate experiment-5 reversal run executes
center → +3° → −3° → center at the validated moderate tier. The reversal
result is explicitly a **lost-motion/backlash proxy**, not pure mechanical gear
backlash.

The first proposed hardware command is:

```text
mlab run 4 2 2 3 3000 0 8 30 250
```

This harness lives on the PR branch, not the parent repository's unchanged
submodule SHA. Before a hardware run, check out the reviewed branch in the
firmware submodule:

```bash
git fetch origin feat/scs009-dynamics-characterization
git switch feat/scs009-dynamics-characterization
```

It moves J2 +3° over 3 seconds, holds for 1 second, and returns. The arm must
be supervised with hands and cables clear. The tool will not issue this command
unless invoked with both `--execute` and `--confirm-hardware`.

## Reproduction and analysis

Create a plan without hardware access (the default execution limit is one
movement):

```bash
python3 tools/motion_lab/characterize_dynamics.py \
  --manifest-only --output-dir backups/motion-lab-dynamics-YYYY-MM-DD --joint 2
```

The deterministic plan always starts with the gentle J2 `+3°` condition. Run
only that first condition from a supervised terminal after reviewing the
printed plan:

```bash
python3 tools/motion_lab/characterize_dynamics.py \
  --execute --confirm-hardware --port "$RIG_PORT" \
  --output-dir backups/motion-lab-dynamics-YYYY-MM-DD --joint 2 --max-runs 1
```

`--max-runs 1` is the recommended first hardware invocation. The harness
performs read-only status, parameter, and voltage preflight, executes exactly
one movement, cleans up polling/Motion-Lab ownership, analyzes that capture,
and returns control. A larger `--max-runs N` is available only after the
previous captures have been reviewed.

Existing captures can be re-analyzed without opening a port:

```bash
python3 tools/motion_lab/characterize_dynamics.py \
  --analyze-only --output-dir backups/motion-lab-dynamics-YYYY-MM-DD
```

Each output directory contains immutable raw UART logs, a `manifest.json` with
firmware SHA/branch, protocol and parameters, hashed preflight snapshots and
the persisted pass/fail decision, SHA-256 hashes for movement captures,
`normalized.csv`, and `dynamics_report.json`. Cleanup sends `mlab stop`,
`mlab poll off`, and `mlab comp off` even when preflight or a run is aborted.

## Metrics and definitions

The report contains, per run and direction:

- valid feedback rate, measured timestamp-derived sample rate, age percentiles,
  and stale fraction;
- commanded and achieved excursion;
- tracking RMS and peak error;
- robust onset latency, using three sustained feedback samples above a
  three-count threshold rather than a one-count quantization change;
- 10–90% motion time where the endpoint is reached;
- observed peak velocity from a 20 ms linear resample followed by a centered
  local quadratic fit;
- endpoint settling and overshoot;
- positive/negative direction asymmetry;
- reversal command travel before sustained opposite feedback;
- voltage min/max, raw load range, and raw servo-speed range.

Raw `fb_speed_raw` is retained and compared but is not treated as calibrated
angular velocity. Physical acceleration and jerk are not inferred from sparse,
quantized feedback. A stale sample, unexpected status error, unsafe target, or
unexpected tracking error invalidates the current run and stops progression.
Voltage is preserved and compared against the previously observed ~8 V device
baseline; the deviation check is an observational preflight/run guard and is
not a validated SCS009 operating or electrical safety range. `fb_load_raw` is
also only a provisional anomaly diagnostic because its physical encoding is not
independently validated.

## Results

No supervised hardware run has been performed by this implementation pass.
Populate this section only from the hashed report artifacts:

| Joint | Direction | Tier/amplitude | Usable peak velocity | Onset | Tracking RMS/peak | Settling | Lost-motion proxy | Voltage/load/rate |
| --- | --- | --- | ---: | ---: | ---: | ---: | ---: | --- |
| pending | pending | pending | pending | pending | pending | pending | pending | pending |

## Proposed simulation profile

`SCS009_CHARACTERIZED_SIM_PROFILE` remains a proposal until the supervised
captures are reviewed. It must include only a conservative velocity value and,
if sample density supports it, an acceleration range with uncertainty. No
measured maximum jerk is claimed. The current Ruckig/simulation defaults remain
unchanged in this PR.
