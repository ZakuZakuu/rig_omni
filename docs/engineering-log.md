# RIG-Arm engineering log

This is the chronological, versioned record of bring-up decisions, experiments,
observations, and follow-up work. Firmware commits remain the source of exact
code changes; this log records why those changes were made and what was actually
observed on hardware.

## 2026-09-13 — Motion Quality Baseline complete

### Software milestones

- `0d44344` added the controlled Motion Lab path and trajectory test harness.
- `932fe20` added the UART0 `mlab` console so experiments can be triggered from
  the ESP-IDF monitor without relying on a web client to control the board.
- `e89bfd9` buffered console input until Enter. The monitor is configured as a
  raw terminal, so a line-oriented parser otherwise interpreted each character
  as a separate command.
- `ad3ccd0` changed telemetry timestamps to a portable 32-bit millisecond field.
  The ESP-IDF newlib configuration did not support the earlier `%lld` format,
  which shifted CSV fields.
- `feb1976` kept the trajectory's internal command accumulator advancing while
  a deadband suppressed a bus write. Without this separation a sub-deadband
  move could stall permanently.
- `68cb856` added the visible 10-degree step/hold/return experiment used to
  establish an observable hardware motion before judging subtle motion quality.

### Decisions and tradeoffs

- Minimum-jerk is now the trajectory baseline for characterization. Linear and
  cubic trajectories remain available for comparison and regression tests, but
  no additional easing variants will be added to the baseline.
- Motion Lab temporarily disables stock idle/preset ownership and restores it
  when the experiment ends. This preserves upstream behavior outside the
  explicitly isolated diagnostic window.
- The first visual sanity check uses approximately 10 degrees, not 3 degrees.
  Three degrees is useful later for subtle-motion analysis, but is too easy to
  hide in backlash, static friction, and camera perspective during bring-up.
- Direct-joint commands are used only for this diagnostic seam. Production IK,
  limits, and calibration remain unchanged.

### Hardware observations

- A visible single-joint 10-degree step produced roughly 35 encoder counts of
  command travel and a corresponding feedback excursion on joint 0.
- Linear, cubic, and minimum-jerk single-joint tests all produced command and
  feedback motion. Synchronized and staggered five-joint tests also tracked the
  commanded sequence.
- Feedback was visibly stale in the logs: the existing request schedule queried
  one of five joints only about every 100 ms, so a full round took roughly
  500 ms. This is the next characterization blocker; no closed-loop tuning
  should be inferred from those stale samples.

## 2026-09-13 — Servo Characterization phase opened

The next phase is deliberately staged:

1. move feedback polling out of the 2 ms control path and timestamp each valid
   status sample;
2. report feedback age at a useful, documented rate;
3. capture all relevant SCS009 factory control registers read-only;
4. compare the root/high-load joint under safe loads/postures and several
   minimum-jerk durations;
5. only then test one internal parameter at a time, reversibly, with a saved
   baseline. No external PID loop is planned.

The current working hypothesis is that visible stick-slip may correlate with
  low-speed motion and load, but this is not yet a conclusion. The experiment
  matrix must keep trajectory shape and amplitude fixed while changing one
  physical condition at a time.

### Implementation status

- Feedback polling now runs outside the control loop at one ID per 20 ms; valid
  samples carry per-joint timestamps and telemetry exposes sample age.
- `mlab params` performs a read-only SCS009 factory/control-table snapshot for
  IDs 1–5. It is intentionally raw and reversible: no EPROM unlock and no
  parameter write are part of this milestone.
- The next hardware action is to flash this build, run `mlab params`, and save
  the complete UART output before any internal parameter experiment.

### Validation record

- Firmware commit: `0930849`.
- ESP-IDF: v5.5.3, target `esp32s3`, `CONFIG_BOARD_TYPE_ARM=y`.
- Host trajectory tests: 5 passed.
- Firmware build: `ninja -C build all` passed; application binary has 26% free
  space in the smallest app partition.
- Hardware validation: pending because no `$RIG_PORT`/USB serial device was
  attached to this engineering session. No flash or motion command was issued.
- Follow-up after the user reattached the device: the Codex sandbox still had
  no `/dev/ttyACM*`; `rig_env` reached the Windows USB attach helper but WSL
  returned `UtilBindVsockAnyPort: socket failed 1`. Direct serial capture is
  therefore delegated to the already-connected user monitor for this session.
