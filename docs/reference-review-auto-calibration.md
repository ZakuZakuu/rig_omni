# Reference review: auto-calibration v0.1

Reviewed 2026-09-13 before extending the prototype. The goal is to reuse the
measurement architecture and definitions that fit RIG-Arm, not to import a
simulation or a large optimizer.

## What transfers

### BAM / bam-feetech

[BAM's identification guide](https://bam.readthedocs.io/en/latest/identification/index.html)
separates the workflow into actuator assumptions, standardized trajectory
recording, fixed-timestep processing, parameter fitting, and validation. The
`bam-feetech` README makes the same separation explicit: record predefined
trajectories, process them onto a constant `dt`, fit on one set, and validate on
held-out logs. BAM also treats communication delay as a rig parameter and its
models can express directional, Coulomb/Stribeck, viscous, quadratic, and
load-dependent effects.

We can reuse the discipline: a manifest must describe every excitation; raw
captures remain immutable; derived data use a fixed time grid; reversal and
direction are retained rather than averaged away; candidate profiles are scored
on held-out repetitions; and command delay is reported separately from actuator
friction. This is more useful here than copying BAM's CMA-ES fitting loop.

The pendulum bench, known mass/length, current/voltage motor equations, and
MuJoCo trajectory reproduction do not transfer. RIG-Arm has a position-loop
SCS009 with unknown internal PWM/current law, no calibrated torque sensor or
known external load, coarse feedback, and a different servo family from the
published STS3215 models. Therefore v0.1 must not claim fitted physical
friction coefficients.

### UART-servo backlash study

[Robonine's backlash project](https://github.com/roboninecom/Measuring-Backlash-in-Popular-UART-Servos)
uses a repeatable home/sweep sequence, explicit phase masks, timestamped CSV
logging, and separate relaxed/stretched segments. Its analysis reports segment
averages and MAE/RMSE rather than treating a whole mixed trace as one sample.
The useful RIG adaptation is a reversal protocol with a sign change, a measured
reversal delay/backlash interval, per-direction distributions, and overlays of
target versus present position.

Its puller-servo test stand and controlled external force are not available on
the assembled arm. Load is consequently a diagnostic proxy from SCS009 feedback,
not a ground-truth torque measurement; posture/load claims must remain modest.

### Klipper calibration architecture

Klipper's
[resonance calibration flow](https://www.klipper3d.org/Measuring_Resonances.html)
is a good process template: standardized excitation → raw CSV capture →
offline analysis/plots → candidate scoring → persisted settings, with generated
settings shown for human review rather than silently applied. The project does
not copy input shaping: RIG-Arm has no accelerometer-based resonance signal and
the observed symptom is directional stick-slip, not a verified structural
resonance.

### LeRobot Feetech tables

[LeRobot's SCS table](https://github.com/huggingface/lerobot/blob/main/src/lerobot/motors/feetech/tables.py)
confirms protocol-1 SCS009 (`model_number=1284`, 1024-count resolution) and the
control-table semantics used here: P/I/D at 21/23/22, startup force at 24 (two
bytes), CW/CCW dead zones at 26/27, SRAM goal position/time/velocity at 42/44/46,
and feedback position/velocity/load/voltage at 56/58/60/62. It also documents
that SCS byte order differs from STS/SMS and that raw sign encoding should not be
borrowed between families.

The existing firmware addresses match these SCS entries for the tuned fields and
preserves raw bytes in `mlab params`. We will keep persistent speed/time values
read-only and separate from runtime `speed_cmd_raw`; unknown model-specific
factory entries remain raw until verified against an SCS009 table.

## Changes to our v0.1

1. Keep the standardized bidirectional minimum-jerk matrix and immutable raw
   UART logs.
2. Add fixed-grid derived output for comparable velocity/error metrics, while
   detecting short dwell/jump events on the raw timestamp stream so interpolation
   cannot erase a jump.
3. Add explicit reversal-delay and backlash-style metrics, reported separately
   for CW and CCW and never converted into a correction unless repeatability is
   demonstrated.
4. Keep the small normalized guardrail score and human A/B/C gate; do not add
   CMA-ES, a Stribeck fit, resonance FFT/input shaping, or an external PID.
5. Keep `calibration/profiles/joint0.yaml` as the persisted artifact. A profile
   may contain a direction-specific lead/minimum velocity only after held-out
   repetitions show a material improvement; otherwise it remains factory/null.
