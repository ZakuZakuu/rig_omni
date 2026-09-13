# Auto Calibration v0.2 — bounded joint-0 screen

v0.1 established a repeatable directional asymmetry but did not produce a
calibration profile. v0.2 is intentionally a small bounded screen for a
practical runtime improvement; it does not attempt a friction model, external
PID, LUT, or exhaustive identification.

## Protocol

`tools/motion_lab/auto_calibrate_v2.py` tests six labelled candidates. Before
each candidate it restores the five-servo factory values, applies at most one
joint-0 EEPROM group, and configures an optional RAM-only directional profile.
Every candidate uses the identical joint-0 test in both physical directions:

- signed 10° endpoint, minimum-jerk, 2,000 ms out-and-back;
- 15°/s velocity limit, 60°/s² acceleration limit, 250 mdeg host deadband;
- two repetitions per direction;
- I remains unchanged and D is not screened because endpoint ringing was not
  established in v0.1.

Candidates are:

| Label | Persistent joint-0 change | RAM-only runtime change |
| --- | --- | --- |
| A | factory | off |
| B | CW dead zone = 0 | off |
| C | factory | CW/CCW deadband 125/250 mdeg, CW floor 1°/s, CW scale 1.10 |
| D | P = 12 | same as C |
| E | startup force = 32 | same as C |
| F | CCW dead zone = 0 | C with CCW deadband 0 |

Telemetry rejects stale feedback, voltage outside 7.5–8.8 V, large tracking
error, or excessive endpoint overshoot. The score combines dwell/jump events,
tracking error, static jitter, and overshoot and is only a safety/ranking
guardrail; it does not replace visual judgment.

Run it with the ESP-IDF Python environment (the system Python may not have
`pyserial`):

```bash
PY=/home/lenovo/.espressif/python_env/idf5.5_py3.12_env/bin/python
"$PY" tools/motion_lab/auto_calibrate_v2.py \
  --port "$RIG_PORT" \
  --output-dir backups/motion-lab-YYYY-MM-DD-auto-calibration-v0.2 \
  --repetitions 2
```

The run writes an immutable manifest, raw UART captures, normalized telemetry,
events, and `auto_calibration_v0.2_report.json`. It always performs a final
factory restore and `mlab comp off`; read back `mlab params` before any later
experiment. The restore path spaces EEPROM writes to avoid dropping the P
register during a multi-register restore.

## Current screen result

The valid rerun is preserved outside Git under
`backups/motion-lab-2026-09-13-auto-calibration-v0.2-rerun/`. All 24 motion
runs had fresh feedback and 7.8–8.2 V. The telemetry ranking was:

1. E — startup force 32 + runtime profile (score 7.34);
2. D — P 12 + runtime profile (score 9.12);
3. A — factory/null control (score 9.17).

This ranking is not a perceptual win. E, D, and A must be run as repeated,
clearly labelled visual candidates. Only after that comparison should a profile
be persisted in `calibration/profiles/joint0.yaml`; otherwise document the
hardware-dominated limit and keep the verified factory profile.
