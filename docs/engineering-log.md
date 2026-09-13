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

## 2026-09-13 — First live SCS009 snapshot and freshness check

The user ran the new firmware without another flash from this session. The
read-only snapshot returned all 24 requested entries for every ID (120 records,
no timeouts). All five servos reported the same identity and core control values:

- firmware `00.16`, model bytes `05 04`;
- baud register `01` (the configured 500 kbit/s setting);
- position limits `0x0014..0x03EB` (20..1003 counts);
- maximum torque limit `0x03E8` (1000 raw counts);
- P=`0x0F`, D=`0x0F`, I=`0x00`;
- minimum startup force `0x0018` (raw value 24);
- IDs 1–4 have clockwise/counter-clockwise dead zones `01/01`.

ID 5 differs at dead zone `04/04`; this is an observation to preserve, not a
reason to tune yet. Protection entries were also identical (`0x14`, `0xC8`).
The raw dump remains the authoritative baseline until units/bitfields are
verified for this exact SCS009 revision.

The visible 10-degree joint-0 step/hold/return completed its 4-second window:
command position moved approximately 489→523 counts and feedback followed about
489→523 counts. Across 78 telemetry rows, feedback age medians were roughly
50–62 ms. There were occasional spikes up to 165–214 ms (and one initial stale
sample at 588 ms immediately after the parameter dump), so freshness is improved
but not yet clean enough to use for closed-loop conclusions.

The next code revision keeps a poll ID unchanged when the UART send lock rejects
a status request. This prevents a dropped request from silently advancing the
round-robin schedule. It builds successfully as the pending commit after this
log entry; it has not yet been flashed or hardware-tested.

## 2026-09-13 — Follow-up visible run exposes long freshness stalls

The next pasted `mlab visible 0` log covered 47 rows and the full 4-second
experiment. Joint 0 commanded position moved `555..590` counts and feedback
reached `556..585`, so the visible motion still tracked. However, feedback ages
were not yet acceptable for characterization: per-joint medians were 60–78 ms,
but maxima reached 773–912 ms, with a broad 1.8–2.6 s stale interval. A Wi-Fi
TLS error was printed during the same run, and the log has no firmware commit
marker, so the cause is not yet isolated between the older flashed build,
UART-send contention, and system-load/serial logging effects.

This run is therefore a useful observability failure record, not a servo-quality
measurement. Do not tune internal P/D/dead-zone values from it. The retry-on-
dropped-poll build (`44773a3`) must be flashed and repeated under the same
experiment before deciding whether more bus scheduling work is needed.

## 2026-09-13 — Hold test confirms a shared feedback stall

The subsequent `mlab hold` log held all five command positions constant for a
10-second experiment. Most samples were in the expected roughly 0–110 ms range,
but around elapsed 2.4–3.4 s all joints simultaneously became stale, with ages
up to approximately 610–696 ms, before recovering. Because the command was held,
this cannot be attributed to the root joint's motion or load. It points to a
shared receive/scheduling/bus interruption that must be isolated before servo
parameter tuning.

The console banner now identifies the intended diagnostic build as
`feedback_poll=20ms; retry=on`, making future hardware logs traceable even when
the Git commit is not included in the monitor capture.

The banner-confirmed build still showed the same class of stall: hold-test age
medians were about 52–54 ms, while maxima reached 300–807 ms. The next revision
therefore changes the polling invariant from “request sent” to “matching valid
response received”; only the latter advances the ID, with a 60 ms retry timeout.

The next banner revision adds `ack=on` so the response-gated round-robin build
can be distinguished from the earlier send-retry build in a pasted monitor log.
It is a traceability marker only; this revision has not yet been flashed or
hardware-tested.

## 2026-09-13 — Response-gated polling hold result

The user then ran a 10-second `mlab hold` with the banner
`feedback_poll=20ms; ack=on; retry=on`. After the initial warm-up, feedback ages
were generally within the expected 0–120 ms range. Ignoring the first 500 ms of
startup, observed maxima were approximately 123, 145, 128, 152, and 125 ms for
joints 1–5 respectively. This is a substantial improvement over the previous
300–807 ms stalls, although occasional samples still exceed the target window
and should be monitored during motion tests.

One row reported `4294967295` ms for joint 2. The corresponding feedback
timestamp was one millisecond newer than the telemetry task's `now_ms`, which
is a benign cross-task snapshot race that wrapped an unsigned subtraction. The
telemetry code now snapshots each timestamp once and clamps a future timestamp
to age zero; this is a logging correctness fix, not a servo-control change.

The hold test is now sufficient to proceed to a cautious root-joint motion
freshness check, but internal servo parameter tuning remains deferred until
motion logs confirm that the improved age bound persists under load.

## 2026-09-13 — Motion exposes command/feedback bus contention

The first directly controlled root-joint test was run from Codex over the
newly available `/dev/ttyACM0` device using:

```text
mlab run 0 2 0 10 4000 0 15 30 0
```

The firmware banner confirmed `ack=on`. During the stationary portions of the
run, feedback was present, but once the trajectory began the reported age grew
continuously into the 0.8–1.2 s range across multiple joints. The command path
was writing a five-joint sync packet on essentially every 2 ms control tick
because the experiment explicitly used zero deadband. This saturated the
half-duplex bus enough to starve the feedback poller; it is a scheduling issue,
not evidence of a servo parameter failure.

The Motion Lab output path now keeps trajectory integration at 2 ms but limits
sync-write packets to 50 Hz (20 ms minimum spacing). This preserves the
requested trajectory shape while reserving bus time for feedback queries. The
next hardware run should use the same root-joint test so the before/after
feedback-age comparison is controlled.

The 50 Hz build was flashed directly through the now-visible `/dev/ttyACM0`
port and the same experiment was repeated. Joint 0 moved from approximately
463 to 498 counts and back to 462–468 counts, matching the commanded
approximately 10-degree minimum-jerk excursion. Feedback ages during motion
were mostly 0–120 ms; isolated samples reached roughly 170–225 ms, with no
return of the earlier 800–1200 ms accumulation. An unrelated Wi-Fi TLS receive
error appeared after the experiment and did not prevent the motion log from
completing.

This establishes 50 Hz sync writes as the current Motion Lab bus-rate baseline.
It is sufficient to proceed with controlled duration/load comparisons, while
logs should continue to retain and report occasional samples above 120 ms.
