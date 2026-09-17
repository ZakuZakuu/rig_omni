# Servo Characterization

This phase starts only after the Motion Lab baseline. Its first firmware build
improves measurement freshness and adds a read-only SCS009 parameter snapshot;
it does not change servo EEPROM values or add an external PID loop.

## Deployment/readiness gate

The host checkout and the image actually running on the ESP32 are separate
evidence sources. After a firmware change, use `rig_env`, run
`idf.py reconfigure` and `idf.py build`, then `idf.py -p "$RIG_PORT" flash monitor`.
Wait for `MLAB_CONSOLE ready`, query `mlab caps`, `mlab help`, and `mlab status`,
then exit the monitor before starting an automated capture. The capability
response records the device image identity and must advertise the experiment
required by the selected protocol (experiment 5 plus reversal for conditioning).
`capture_serial.py` is a capture/command helper only; it never flashes and its
success cannot prove deployment.

Keep these failure classes separate in manifests and reports:

- **Deployment failure** — wrong or stale image, missing/malformed capability,
  unsupported experiment, or an `invalid config` response before any telemetry.
- **Preflight failure** — status, voltage, or read-only parameter snapshot fails
  before motion.
- **Conditioning failure** — the conditioning motion started, but its feedback,
  load, return, or health gates failed.
- **Formal failure** — a formal motion started and then failed tracking or
  safety checks.

An invalid configuration with no `MLAB` rows is not evidence of servo travel;
it must be recorded with `physical_motion_started=false` and the host must stop
without automatically trying another movement.

## Feedback contract

The servo bus is polled by `xgo_feedback_poll`, not by the 2 ms command loop.
One ID is requested every 20 ms, so the nominal complete five-ID cycle is 100 ms
(about 10 Hz per joint). Motion Lab telemetry remains 20 Hz and adds:

- `fb_ts_ms[5]`: the millisecond timestamp at which each status packet was
  accepted;
- `fb_age_ms[5]`: telemetry time minus that sample timestamp;
- `fb_pos[5]`, `fb_speed_raw[5]`, `fb_load_raw[5]`: unchanged raw feedback
  fields. Load is an effort proxy, not a calibrated force.
- `fb_stale[5]`: explicit stale marker. A joint is marked stale only after
  three unanswered request attempts; a valid status packet clears the marker.
- `servo_voltage_v`: the most recent read-only SCS009 present-voltage sample from
  ID 1, in volts. It is refreshed at most once per second so it cannot starve a
  pending position/status response. This is a bus-voltage diagnostic, not a
  calibrated current or transient droop measurement.
- `speed_cmd_raw`: the runtime velocity field sent in the existing `0x2A`
  position sync-write. It is not a persistent EEPROM speed/time parameter.

For a suspected single-servo torque loss, use the read-only diagnostic command
`mlab status`. It prints the latest protocol error byte, the last non-zero error
byte retained since boot, its timestamp, and an occurrence count for each ID.
This is intentionally separate from the normal telemetry CSV and does not send
any write or torque command.

The console `mlab run` amplitude accepts either sign (`-10..-1` or `1..10`),
allowing a mirrored physical-direction test without changing any servo register.
The safety check applies the signed target to the current feedback position and
rejects either direction outside the conservative count range.

The normal poller uses a 60 ms response timeout and three total attempts per
request. After the third timeout it records a skip, marks that joint stale, and
continues with the next ID. This bounds the damage from an unplugged or
unresponsive servo instead of blocking the entire round-robin.

For stick-slip characterization, enable selected-joint high-rate mode from the
monitor (joint numbers are zero-based):

```text
mlab poll 0 5       # prioritize joint 0, request no faster than every 5 ms
mlab poll stats     # print valid count, measured rate, max response gap, skips
mlab poll off       # restore the normal five-joint round-robin
```

The selected joint is prioritized while the other joints remain background
samples at roughly 100 ms. The receive parser also runs at the 5 ms diagnostic
cadence. The configured period is only a request schedule; use the `rate_mHz`
and `max_gap_ms` values from `mlab poll stats` as the measured reliable rate.
In the final flashed test, a 5 ms request schedule produced approximately
45.1 Hz valid responses with a 1.224 s worst gap under normal system load, so
the result is not treated as a guaranteed 200 Hz feedback stream. Background
slots did run (unlike the earlier scheduler bug), but the shared system load
still caused occasional bounded skips.

Reject or annotate samples with unexpectedly high age before using them for
stick-slip or tracking conclusions.

## Read-only factory snapshot

With the firmware running in the ESP-IDF monitor, and with no Motion Lab motion
active, enter:

```text
mlab params
```

The monitor prints `SCS009_PARAM` CSV records for IDs 1–5. `raw_hex` preserves
the bytes returned by the servo; no byte order or scale is applied. The command
does not unlock the EPROM area and sends no write instruction. Save the complete
monitor output as an immutable baseline together with the firmware commit and
date. Do not run it while holding the arm or while a motion experiment is active.

The snapshot covers the SCS-series identity, communication, limits, P/D/I,
minimum startup force, dead zones, and protection entries used for later
characterization. The addresses follow the SCS-series control table and the
FEETECH SCS protocol; SCS009 is a potentiometer SCS servo, so raw bytes are kept
until the model-specific table is verified.

The current factory snapshot is also the tuning guardrail. Before any internal
parameter experiment, compare a fresh `mlab params` capture against the saved
baseline. Motion Lab does not write a persistent speed/time parameter: its
`speed_cmd_raw` field is a runtime command value. The reversible tuning interface
is intentionally staged as deadband, P, D, then startup force; I is not exposed.
No tuning value should be changed until the repeatable duration baseline and
voltage check are complete, and each group must be restored and read back before
the next group.

References:

- [FEETECH communication protocol and memory-table downloads](https://www.feetechrc.com/en/letter-of-agreement.html)
- [FEETECH SCS009 product specification](https://www.feetechrc.com/en/6v-23kg-cm-dual-axis-serial-bus-steering-gear.html)
- [FEETECH protocol manual mirror](https://files.seeedstudio.com/wiki/robotics/Actuator/feetech/Communication_Protocol_Manual.pdf)

## Automatic CW-stutter screen

The bounded screen is reproducible from the firmware directory:

```bash
python tools/motion_lab/characterize_stutter.py \
  --port "$RIG_PORT" \
  --output-dir backups/motion-lab-YYYY-MM-DD-auto-characterization
```

It repeats the same positive/CW 10° minimum-jerk out-and-back three times at
8, 15, and 30°/s velocity limits. It analyzes only the outbound half and
requires a feedback step larger than the commanded sample step, so ordinary
encoder quantization is not reported as a jump. Re-running the classifier over
existing captures without touching the arm is supported with `--analyze-only`.

The 2026-09-13 run produced 28 candidate jump events across nine valid captures.
Relative command-position spread was 2.22°, absolute event-time spread was
359 ms, and event rates were 2.67/3.33/3.33 per run at 8/15/30°/s. This is
`mixed-or-under-sampled`, not evidence for a position LUT or a strong velocity
law; normalized event phase alone is not treated as time locking.

For a small reversible follow-up screen, the firmware exposes a RAM-only
directional host command deadband:

```text
mlab comp off
mlab comp 0 <cw_deadband_mdeg> <ccw_deadband_mdeg>
mlab comp show
```

`tools/motion_lab/search_compensation.py` tests factory (A), CW=0 (B), and
CW=125 mdeg (C), with CCW fixed at 250 mdeg, using two identical 10° runs per
candidate. It restores `mlab comp off` in a cleanup path. The profile does not
write servo EEPROM and is disabled by default; telemetry only rejects stale or
out-of-voltage runs and cannot select the final human-visible winner.

The 2026-09-13 automated screen had no stale rows and 8.0–8.1 V for all
candidates; telemetry scores were effectively tied (C marginally lower). The
human A/B/C captures are under
`backups/motion-lab-2026-09-13-auto-compensation/human_ab/`. The profile is
currently left disabled pending visual choice. If all three look equivalent,
retain factory behavior and move on rather than escalating actuator tuning.

## Characterization matrix

Use minimum-jerk, one root/high-load joint at a time, fixed amplitude and limits.
Begin with the visible 10-degree step/hold/return test, then use smaller safe
amplitudes only after feedback age is verified. For each condition collect at
least three out-and-back repetitions:

| Factor | Initial levels | Hold constant |
| --- | --- | --- |
| trajectory duration | 2.8 s, 3 s, 6 s | amplitude, joint, posture |
| safe posture/load | unloaded reference, light supported load, normal working load | duration, amplitude |
| direction | positive and negative | all above |

Record posture/load setup, servo supply condition, commit, and the raw log file.
Do not approach mechanical limits or intentionally stall a joint. The first
question is whether visible stutter tracks load and very-low-speed portions of
the same minimum-jerk profile; only after this baseline may one internal
parameter (dead zone, P, D, or startup force) be changed and restored.

For unattended captures, `tools/motion_lab/capture_serial.py` continuously
drains UART0 and writes raw bytes to a file, avoiding monitor/PTY backpressure.
For example:

```bash
python3 tools/motion_lab/capture_serial.py \
  --port "$RIG_PORT" \
  --command 'mlab run 0 2 0 10 3000 0 15 30 0' \
  --duration-ms 3000 \
  --output /tmp/rig_motion_3000ms.log
```

The output is intentionally not committed; keep it with the experiment notes
and record its path, firmware commit, posture, and load separately.

The selected-joint mode is an observability aid, not a closed-loop controller.
Do not interpret a high `fb_speed_raw` value as calibrated angular speed until
the SCS009 scale is independently verified.

## Capture parser and small-motion interpretation

`speed_cmd_raw` is emitted by the current firmware as one scalar because the
same runtime value is used by the position sync-write. The host parser accepts
that scalar and also accepts the historical five-element array form, selecting
the requested joint in either case. The raw UART capture is never rewritten.

Every dynamics metric reports commanded and achieved excursion in both degrees
and encoder counts. With the current conversion (`1024 / 300` counts per
degree), excursions at or below about 14 counts (roughly 4 degrees) are marked
`quantization_sensitive`; this is an interpretation flag, not a failure.

`motion_onset_latency_ms` is a robust observation, not pure bus or actuator
latency. It starts at the first command sample that changes by at least one
count and ends only after three strictly advancing feedback samples each move
at least three counts in the requested direction. The different thresholds
reduce one-count noise but also include command quantization, feedback age,
mechanical response, and any dwell/stick-slip.

The reported `observed_peak_velocity_deg_s` is derived from timestamped,
quantized feedback (20 ms resampling plus a local quadratic fit). It remains an
un-calibrated diagnostic estimate and must not be used as an actuator velocity
limit. Raw `fb_speed_raw` values are preserved separately until the installed
servo encoding is verified.

## Cold and conditioned small-signal protocols

The first r1/r2 J2 +3-degree captures are retained as **cold / unconditioned**
evidence: the mechanical state before each run was not standardized. They must
not be discarded or reinterpreted as a repeatability result.

The characterization harness has an optional, disabled-by-default conditioning
prelude. Enable it explicitly for a future formal run with
`--precondition positive` (or `negative`). The positive protocol sends:

```text
mlab run 5 2 <joint> 5 2500 0 8 30 250
```

which is the existing minimum-jerk reversal sweep:
`center -> +5° -> -5° -> center`. The mirrored negative protocol uses `-5°`
and follows `center -> -5° -> +5° -> center`. The selected 2,500 ms transition
is intentional: with the existing 8°/s and 30°/s² caps, the ideal reversal
peak is 7.5°/s and 9.24°/s², so the command generator does not distort either
endpoint. Total conditioning time is 9,500 ms, followed by a quiet 2-second
settle interval and a separate read-only `mlab status` snapshot before any
formal measurement. The conditioning excursion is inside the existing ±10°
Motion Lab envelope; it does not change compensation or persistent servo
parameters.

Before supervised hardware use, run the deterministic offline feasibility check
(no serial port is opened):

```bash
python3 tools/motion_lab/conditioning_trajectory.py --transition-ms 1000
python3 tools/motion_lab/conditioning_trajectory.py --transition-ms 2500
```

The first command is expected to report `DISTORTED`; the selected profile must
report `FEASIBLE`. It reports nominal/requested endpoints, generated command
endpoints, peak command velocity/acceleration, endpoint-hold error, and
overshoot in degrees and encoder counts.

The deterministic comparison is recorded here for traceability: at 1,000 ms
the simulated command reaches approximately +5.689°/−6.049° (about +19/−20
counts), with velocity and acceleration caps active, endpoint-hold errors of
about 2.35/10.85 counts, and negative overshoot of about 3.58 counts. At the
selected 2,500 ms transition it reaches +5.000°/−5.000° (+17/−17 counts),
with 0-count hold error/overshoot and analytical peaks of 7.50°/s and
9.24°/s². These are command-reference diagnostics, not hardware capability
claims.

The harness remains dry-run by default. A future supervised positive run would
opt in explicitly, for example:

```bash
python tools/motion_lab/characterize_dynamics.py \
  --execute --confirm-hardware --max-runs 1 --joint 2 \
  --amplitudes-deg 3 --tiers gentle --skip-low-speed --skip-reversal \
  --repetitions 1 --precondition positive --port "$RIG_PORT" \
  --output-dir backups/motion-lab-conditioned-YYYY-MM-DD
```

`--max-runs` limits only formal measurement captures; the explicitly selected
conditioning prelude is always evaluated first and is never counted as a
formal `runs` entry.

To validate the preconditioning motion itself before coupling it to a formal
measurement, use `--conditioning-only` together with the same explicit
hardware gates:

```bash
python tools/motion_lab/characterize_dynamics.py \
  --execute --confirm-hardware --conditioning-only \
  --precondition positive --joint 2 --port "$RIG_PORT" \
  --output-dir backups/motion-lab-conditioning-only-YYYY-MM-DD
```

This mode performs preflight, exactly one reversal sweep, the 2-second settle
observation, and cleanup. It records `conditioning_complete` (or
`conditioning_failed`) and never starts a formal `mlab run 4`, regardless of
`--max-runs`.

Conditioning is a health/preload gate, not part of formal-run metrics. It must
show fresh feedback, telemetry-confirmed motion in both directions, no status
error or stale flag, voltage within the existing observational ~8 V review
band, raw load below the provisional anomaly gate, and return within five
encoder counts of the preconditioning center. The gate compares feedback with
the command endpoint actually generated and held (50% minimum), while also
recording the nominal requested ±5° endpoint. If the command endpoint itself
differs from the request by more than roughly 1.5 counts, the result is marked
an experimental-design distortion and is not a clean breakaway conclusion. If
any gate fails, the formal run is not started. Reports keep `conditioning`,
`preflight`, and formal `runs` as separate sections and record
`formal_run_started` explicitly.

Conditioning metrics report the command positions actually emitted in the
capture (positive/negative excursion in counts and degrees), the corresponding
feedback excursions, return error, unique feedback rate/age, peak raw load,
and voltage range. Thus the `+5° -> -5°` reversal leg is measured rather than
assumed to reach either nominal endpoint.

### Initial cold-baseline forensic comparison

The two supervised J2 +3-degree runs remain immutable evidence. Both delivered
the same ten-count commanded excursion (2.9297°), but r1 achieved seven counts
(2.0508°) while r2 achieved zero counts. The initial J2 feedback counts were
152 (r1) and 154 (r2), with command counts 153 and 154; final feedback was 150
and 154 respectively. Both command trajectories reached their ten-count
endpoint and held it for approximately one second, so host command delivery
appears valid. The feedback traces were 144–159 counts in r1 and 153–154 in
r2; r2 accumulated a ten-to-eleven-count endpoint error while held.

The J2 read-only parameter snapshots were identical: P=0x0F, I=0x00, D=0x0F,
CW/CCW dead zones=0x01/0x01, startup force=0x0018, and unchanged limits and
protection values. Feedback freshness was good in both runs (no stale rows;
sample rate about 16.8–17.0 Hz), while voltage stayed near 8 V and raw loads
remained below the provisional anomaly gate. The evidence therefore rules out
a simple host-delivery or telemetry-rate explanation, but does not distinguish
internal dead-zone/startup-force behavior from mechanical stiction/preload or
another unresolved state-dependent effect. No pure backlash or servo fault is
inferred from these two cold runs alone.

## Current baseline result

The earlier 1.5 s captures are retained as historical data, but are not directly
comparable: with `max_velocity=15` and `max_acceleration=30`, the acceleration
limiter allowed the internal command to reach about 13.2° before returning. The
strict comparison matrix therefore replaces 1.5 s with 2.8 s. A protocol capture
at 2.8 s reached `-4.984..5.019°` (10.003° excursion), confirming the endpoint
is now comparable with the 3 s and 6 s conditions. A safe posture/load comparison
was skipped because no explicit verified compact/extended pose command exists yet.
The ID-1 voltage check measured 8.00 V at idle and 8.00–8.10 V during motion;
all rows reported `speed_cmd_raw=350`. These observations support continuing with
factory settings and do not justify PID or EEPROM tuning by themselves.

The voltage-enabled rerun on commit `3d276c2` repeated each duration three times
and preserved `speed_cmd_raw=350` in all 9 captures. Voltage stayed within
7.90–8.10 V, with no stale rows. The raw logs are preserved outside Git under
`backups/motion-lab-2026-09-13-voltage/` with a `SHA256SUMS` manifest.

The first human-in-the-loop startup-force trial tested ID1 values 24 (A), 32 (B),
and 40 (C) with the identical 10°/2.8 s minimum-jerk sequence. B improved the
return but introduced visibly segmented forward motion; C was approximately the
same as B. Neither was a whole-motion improvement, so the final reversible
profile is factory startup=24 with factory P/D/I and dead zones. Startup-force
tuning is considered inconclusive/negative for this joint and is paused rather
than being escalated.

The directional asymmetry now makes separate deadband investigation the next
meaningful parameter group. The console exposes `mlab tune cw_deadband <raw>`
and `mlab tune ccw_deadband <raw>` so one SCS009 register can be changed at a
time; the legacy `mlab tune deadband` command remains a paired write for
backward compatibility. Begin from factory `1/1`, use at most two conservative
CW candidates, and restore/read back the pair before any other parameter group.

The follow-up direction comparison used the factory baseline and three repeated
`+10°` and three repeated `−10°` minimum-jerk sweeps at 2.8 s with identical
velocity/acceleration limits. The raw captures are under
`backups/motion-lab-2026-09-13-voltage/direction_*10_r*.log`. The visual result
must be judged by physical direction versus outbound/return phase; telemetry is
provided to confirm sign and amplitude but does not choose the interpretation.
