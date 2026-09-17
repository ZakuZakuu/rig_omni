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

The same root-joint test at 8 s duration revealed a separate system-level
failure mode. Most samples remained below roughly 200 ms, but one shared stall
around elapsed 5.0–6.0 s drove feedback ages above 1 s (maxima after warm-up:
about 1136, 1111, 1442, 471, and 1081 ms for joints 1–5). A Wi-Fi TLS receive
error was printed during that interval. Because the command was a single-joint
test and the ages rose across all joints together, this is not evidence of
root-joint stick-slip or servo tuning behavior. Longer-duration characterization
is blocked until UART receive/poll scheduling is isolated from asynchronous
Wi-Fi/camera work.

The 50 Hz command cadence shares the same 20 ms period as the feedback poller,
so a persistent phase collision is also possible. The next diagnostic build
uses a 25 ms command period (40 Hz) while keeping the 2 ms trajectory update;
this deliberately de-synchronizes command writes and feedback queries before
adding deeper UART instrumentation.

## 2026-09-13 — Bounded retries and measured selected-joint feedback rate

The next diagnostic revision made the response-gated poller finite: each status
request has a 60 ms timeout and three total attempts. On the final timeout the
corresponding `Motor::FbStale` flag is set, a per-ID skip counter is incremented,
and the scheduler continues with the next servo. A valid status packet clears
the stale flag. Telemetry now includes `fb_stale[5]` so a stale sample cannot be
mistaken for fresh data.

The same revision added an explicit `mlab poll <joint> <period_ms>` mode. It
prioritizes one selected ID (initially joint 0), keeps the remaining IDs as
approximately 100 ms background samples, and shortens both the poll and receive
task cadence to 5 ms when configured at the minimum period. `mlab poll stats`
reports valid count, measured rate, maximum response gap, and skip counts.

Hardware validation after flashing the build:

- `mlab poll 0 5` with no active motion: 594 valid selected-joint responses over
  13,154 ms, measured 45.1 Hz, maximum gap 1,224 ms, skip counts 4|1|1|1|1.
  The configured 5 ms period is therefore not a guaranteed 200 Hz stream; the
  long gap remains a system-load/transport issue to isolate. The high-rate
  scheduler was then corrected so background slots cannot be starved by a
  perpetually due selected-joint request.
- With high-rate mode disabled, the controlled test
  `mlab run 0 2 0 10 4000 0 15 30 0` completed and joint 0 tracked roughly
  531..562 counts and back. The final run's five-joint feedback ages were
  generally 0..170 ms and all `fb_stale` fields remained zero.

These results improve observability and failure containment but do not justify
PID, dead-zone, or startup-torque changes. Duration/load comparisons remain the
next characterization step, with high-rate stats retained alongside each raw
log.

## 2026-09-13 — Unloaded minimum-jerk duration sweep

Using the UART-draining `tools/motion_lab/capture_serial.py` helper and the
final flashed build, joint 0 was tested with the same 10-degree minimum-jerk
single-joint sweep at 1.5 s, 3 s, and 6 s complete out-and-back durations.
All runs used `max_velocity=15`, `max_acceleration=30`, zero command deadband,
the current 40 Hz Motion Lab sync-write cadence, and no added mechanical load.

| duration | telemetry rows | command range (deg) | feedback range (counts) | max age (j0..j4 ms) | stale rows |
| ---: | ---: | ---: | ---: | --- | --- |
| 1.5 s | 26 | 4.12..17.30 | 522..562 | 254, 307, 287, 308, 273 | 0, 0, 0, 0, 0 |
| 3 s | 53 | -7.62..2.38 | 483..514 | 163, 137, 165, 140, 150 | 0, 0, 0, 0, 0 |
| 6 s | 106 | -0.88..9.12 | 491..537 | 141, 171, 142, 165, 143 | 0, 0, 0, 0, 0 |

The longer profiles naturally provide more low-command-velocity samples near
the minimum-jerk endpoints (approximately 1, 4, and 15 samples under the
current 20 Hz telemetry and a 0.5 deg/s finite-difference threshold). The raw
root load medians in these three captures were approximately 1093, 114, and 99
respectively, with maxima 1153, 1168, and 1243. Because the load field is an
unsigned/raw effort encoding and the captures begin at different poses, this is
not a monotonic duration relationship and must not be treated as evidence of a
servo-parameter effect.

Conclusion: unloaded duration changes are mechanically safe and observable, but
they do not yet separate low-speed stick-slip from posture/load or system-level
transport effects. A controlled safe-load/posture comparison is still required
before touching internal P/D/dead-zone/startup-force parameters.

## 2026-09-13 — Repeatability, parameter readback, and supply-voltage check

The duration baseline was repeated three times per condition before actuator
tuning. Every run used joint 0, a 10-degree minimum-jerk command, zero command
deadband, `max_velocity=15`, `max_acceleration=30`, and the current 40 Hz sync
write cadence. The table reports root-joint feedback excursion in raw counts;
the coefficient of variation across repetitions was approximately 5–6%.

| duration | feedback excursion, repetitions (counts) | max joint-0 age (ms) | stale rows |
| ---: | --- | ---: | ---: |
| 1.5 s | 46, 43, 41 | 144, 145, 166 | 0, 0, 0 |
| 3 s | 36, 36, 40 | 194, 302, 170 | 0, 0, 0 |
| 6 s | 41, 39, 37 | 229, 166, 172 | 0, 0, 0 |

The optional compact-versus-extended posture comparison was not run: the current
Motion Lab console has no explicit, verified safe static-pose command, so guessing
an IK pose would add avoidable mechanical risk. This is recorded as a skipped
factor rather than treating the unloaded sweep as a load result.

### Persistent speed/time audit

The Motion Lab path still calls the existing `SetMotorAngle(command_deg,
motor_speed)` function. `SetMotorPos` sends the upstream runtime sync-write at
address `0x2A`; its velocity field is the existing in-RAM `motor_speed` value.
The Motion Lab telemetry now records `speed_cmd_raw`, which was `350` in the
3-second capture. No Motion Lab path sends a persistent speed/time register or a
servo EEPROM write. A source comparison against the upstream baseline confirms
that the runtime command packet and default `motor_speed=350` are unchanged.

The fresh read-only parameter dump after the restore check returned the same
control values as the pre-test baseline for all five servos: P=`0x0F`, D=`0x0F`,
I=`0x00`, startup force=`0x0018`, dead zones `0x01/0x01` (ID 1–4) and
`0x04/0x04` (ID 5), baud=`0x01`, and return delay=`0x00`. The complete post-check
capture is kept locally at
`backups/scs009-factory-params-2026-09-13-post-restore.log` with SHA-256
`db11fe3d25a12012abdae0f4d7237bedbf6044b24d0a2ec3e036770faf268045`.

### Servo voltage telemetry

SCS009 present-voltage feedback is now sampled read-only from ID 1 at most once
per second when the bus is otherwise free. It is included in every Motion Lab
row as `servo_voltage_v`; `mlab voltage` provides an explicit idle reading. The
observed idle value was `8.00 V`. During the 3-second minimum-jerk run it stayed
at `8.00 V` initially and rose only to `8.10 V` in later samples, with no motion
correlation or brownout signature. This is servo-reported bus voltage, not a
direct oscilloscope measurement of transient current or rail droop, so it does
not eliminate very short supply disturbances; it does rule out a persistent
low-voltage or speed-setting explanation for the observed slowdown.

The reversible tuning command path is present for the next phase, but no new
deadband, P, D, or startup-force value has been applied. The one restore command
used for this check wrote the captured factory values back and the subsequent
read-only dump verified that there was no net parameter change. I remains
untouched. Continue characterization with the factory values before any tuning.

### Voltage-enabled repeat captures

To ensure every characterization record carries the new supply and runtime-speed
fields, the same three-duration matrix was rerun on the flashed `3d276c2` build.
All nine captures reported `speed_cmd_raw=350`, zero stale rows, and the voltage
ranges below. The small 7.90–8.10 V spread is the servo's 0.1 V reporting
resolution; it is not a sustained sag.

| duration | repetition | rows | root feedback range (counts) | max j0 age (ms) | servo voltage (V) |
| ---: | ---: | ---: | ---: | ---: | ---: |
| 1.5 s | 1 / 2 / 3 | 25 / 25 / 26 | 464–511 / 529–578 / 556–595 | 143 / 166 / 206 | 8.00–8.10 / 8.10 / 8.00–8.10 |
| 3 s | 1 / 2 / 3 | 51 / 52 / 51 | 467–502 / 471–507 / 477–510 | 164 / 140 / 147 | 8.10 / 8.10 / 8.10 |
| 6 s | 1 / 2 / 3 | 104 / 103 / 103 | 481–517 / 479–515 / 478–511 | 141 / 172 / 197 | 8.10 / 7.90–8.10 / 8.00–8.10 |

These captures confirm the telemetry path is carrying voltage and command-speed
context without changing the factory control values. The root-joint feedback
excursions remain repeatable enough for baseline comparison, but the different
starting poses mean their absolute encoder ranges should not be compared as a
load estimate.

## 2026-09-13 — Human-in-the-loop startup-force A/B/C trial

Because the stationary hold did not show sustained hunting and the working visual
symptom is low-speed dwell-then-jump, the first meaningful tuning group was
startup force. Only joint 0 was changed; P, D, I, and both dead zones remained at
factory values. Each candidate used the identical visible sequence:
`mlab run 0 2 0 10 2800 0 15 30 0` (minimum-jerk, 10° excursion, 2.8 s,
out-and-back).

| label | ID1 startup force | capture |
| --- | ---: | --- |
| A | `0x0018` (24, factory) | `backups/motion-lab-2026-09-13-voltage/startup_A_factory.log` |
| B | `0x0020` (32) | `backups/motion-lab-2026-09-13-voltage/startup_B_32.log` |
| C | `0x0028` (40) | `backups/motion-lab-2026-09-13-voltage/startup_C_40.log` |

The three logs are preserved with SHA-256 hashes in the experiment directory.
The post-C readback confirmed ID1 startup=`0x0028`, P=`0x0F`, D=`0x0F`, I=`0x00`,
and dead zones `0x01/0x01`; no other control group was changed. At this point the
robot was paused at C pending the human visual choice. Telemetry is retained for
diagnosis but is not the selector.

### Human visual result

The user repeated A three times, then watched B and C with explicit notices before
each parameter switch. A showed noticeable start-of-travel stick-slip and a
slightly better return. B made the return substantially smoother, but converted
the forward motion into roughly three discrete jumps. C looked approximately the
same as B, with no visible improvement. Because neither candidate improved the
whole motion and the benefit was marginal, C was rejected and ID1 startup was
returned to A (`0x0018`). The final readback confirmed factory startup, P/D/I, and
dead zones. No further startup-force candidates will be tested.

## 2026-09-13 — Unexpected ID2 torque loss; tuning paused

Immediately after the A/B/C sequence, the user reported that the second servo
from the base became limp. `mlab stop` found no active experiment. The servo
recovered its holding torque after a robot restart, so the event is treated as a
latched protection/drive-state incident until proven otherwise; no further motion
test is authorized yet.

Read-only checks after the restart found ID2 still responding and its complete
control/protection snapshot unchanged from factory (temperature limit `0x37`,
voltage limits `0x64/0x28`, max torque `0x03E8`, unloading/LED alarm `0x25`,
protective torque `0x14`, protection time `0xC8`, P/D/I `0F/0F/00`, startup
`0x0018`). ID1 reported 8.10 V. These checks do not prove which protection cause
latched because the current parser does not expose a servo error/alarm byte.

The temporary C setting was restored to factory values (`MLAB_TUNE
restore_factory=1`) and read back: ID1 and ID2 both report startup `0x0018` with
factory P/D/I and dead zones. The post-event captures are preserved under
`backups/` with SHA-256 hashes. Startup-force optimization is paused; inspect
servo temperature/protection indicators and reproduce only under direct human
supervision after the cause is understood.

To make a future transient observable without expanding the normal telemetry
rate, the firmware now retains each servo status packet's protocol error byte,
last non-zero error timestamp, and count. The read-only `mlab status` command
prints this snapshot. This diagnostic build compiles successfully; it should be
flashed only when the arm is supported and observed, and no motion test should
be run until the torque-loss cause is understood.

## 2026-09-13 — Comparable endpoint protocol and hold classification

The original 1.5 s condition is excluded from speed comparisons. With the fixed
`max_velocity=15` and `max_acceleration=30` limits, its acceleration-limited
internal command reached about 13.2° instead of the requested 10°. A 2.8 s
minimum-jerk replacement was tested with the same limits and produced a command
range of `-4.984..5.019°`, i.e. a 10.003° excursion. The strict duration matrix
is now 2.8 s, 3 s, and 6 s; all three use the same 10° amplitude and limiter
settings.

The fresh 10-second stationary `mlab hold` run does not show persistent static
hunting after warm-up. Joint 0 stayed at 500–501 counts for almost the entire
hold (a one-count quantization/jitter band); the initial 488→500 transition is
the command handoff from the previous experiment. There was no repeated large
correction while holding. The current dominant symptom is therefore not static
hunting; based on the prior visible motion reports and the discrete low-speed
feedback steps, the working category is in-motion dwell-then-jump / stick-slip.
Endpoint ringing is not established by the hold test and will be judged during
the visible A/B/C motion sequence. No automatic telemetry-based winner will be
selected.

## 2026-09-13 — Mirrored physical-direction comparison

The factory baseline was restored and read back before this comparison: all five
servos had startup=`0x0018`, P/D/I=`0F/0F/00`, and their original dead zones.
Motion Lab was extended to accept signed amplitudes so the same trajectory could
be run in the opposite physical direction. No servo parameter was changed during
the comparison.

The protocol was three repeated `+10°` sweeps followed by three repeated `−10°`
sweeps, each `joint 0`, minimum-jerk, 2.8 s, `max_velocity=15`,
`max_acceleration=30`, zero command deadband. The captures are:

- `direction_plus10_r1.log`, `direction_plus10_r2.log`, `direction_plus10_r3.log`
- `direction_minus10_r1.log`, `direction_minus10_r2.log`, `direction_minus10_r3.log`

All six runs completed with zero stale rows. Telemetry confirmed the signed
command ranges and roughly 10° excursions; voltage stayed at 7.9–8.0 V and
`speed_cmd_raw` remained 350. The user's visual comparison of + versus − is the
decisive result: if one physical sign is consistently worse, investigate
direction-dependent friction/backlash or CW/CCW servo behavior; if both signs show
the same outbound/return asymmetry, return to trajectory/start-state effects.

The aggregate telemetry does not contradict the visual result, but it is too
coarse to select a parameter automatically: the three `+` runs had median
absolute root tracking error 4–5 counts and median load 54–84 raw, while the
three `−` runs had 5–6 counts and 69–99 raw. Sampled feedback plateaus were
common in both directions, with maximum single-sample steps of 10–14 counts (+)
and 13–16 counts (−). The repeatable physical-direction asymmetry is therefore
treated as an actuator/mechanical effect, not as a trajectory-duration issue.

The next firmware revision adds separate `cw_deadband` and `ccw_deadband`
commands. This preserves the factory pair (`0x01/0x01`) and makes it possible to
change only the direction-specific register before comparing the same physical
direction again. No P, D, or startup-force value is changed by this revision.

## 2026-09-13 — Bounded automatic CW-stutter screen

The first automatic characterization attempt produced empty files because the
WSL serial session briefly lost its stream. `esptool chip_id` still responded,
and a fresh UART session recovered normal output; the empty files are not treated
as experimental data. The capture script now fails immediately when no telemetry
rows are present and supports `--analyze-only` for offline reclassification.

Nine valid captures (three repetitions at each 8, 15, and 30°/s limit) were then
collected with joint 0, positive/CW 10°, minimum-jerk, 5 s out-and-back, and the
same 60°/s² acceleration limit. The conservative outbound jump detector found
28 events. Relative command-position spread was 2.22°, event-time spread was
359 ms, and event rates were 2.67, 3.33, and 3.33 per run. The result is
`mixed-or-under-sampled`; no position LUT, velocity law, or time-lock claim is
justified from this matrix.

To close the issue quickly without another EEPROM experiment, a RAM-only Motion
Lab profile was added for per-joint CW/CCW host command deadbands. It is disabled
by default and does not affect IK/action control or SCS009 registers. A bounded
screen compared A=factory (250/250 mdeg), B=CW 0/CCW 250 mdeg, and C=CW
125/CCW 250 mdeg, two repetitions each. All runs had valid feedback, no stale
rows, and 8.0–8.1 V; telemetry scores were effectively tied, with only a
marginal numerical edge for C. Human visual choice therefore remains decisive.
The raw matrix is under
`backups/motion-lab-2026-09-13-auto-compensation/`; the explicit A/B/C motion
captures are under its `human_ab/` subdirectory. The profile was restored to
`mlab comp off` after the comparison. No further automatic actuator tuning is
planned unless one candidate is visibly and repeatably better.

## 2026-09-13 — Auto-calibration prototype v0.1

The next phase was implemented as a host-side, dependency-free prototype rather
than a new controller. `tools/motion_lab/auto_calibrate.py` captured joint 0 in
both physical directions at fixed 10° endpoints, 3 s/5 s durations, 8/15°/s
velocity limits, 60°/s² acceleration limiting, and two repetitions per cell.
The script recorded a read-only factory parameter snapshot, voltage and
feedback-age telemetry, and always disabled the RAM compensation profile in
cleanup. No SCS009 EEPROM register was written.

`analyze_stutter.py` emitted `metrics.csv`, `events.jsonl`,
`classification.json`, and a dependency-free SVG command/feedback plot. The
16-run matrix contained 65 conservative dwell/jump events. CW quality score
averaged 11.109 versus 8.663 CCW, but CW event-position spread was 2.45° and
the combined result classified as `mixed-or-hardware-limited` (confidence
0.45). Every run had fresh feedback and 8.0–8.2 V. The data do not justify a
position LUT, minimum-smooth-velocity law, or internal PID change; the practical
next step is Creature Motion with the factory profile rather than an elaborate
calibration model.

The reusable profile is `calibration/profiles/joint0.yaml`, kept separate from
Motion Engine source. It records the baseline metrics and leaves directional
compensation null/disabled. The raw matrix and analysis are preserved outside
Git under `backups/motion-lab-2026-09-13-auto-calibration/`.

## 2026-09-13 — Reference review before further identification

Before adding more custom identification math, the reference review in
`docs/reference-review-auto-calibration.md` compared BAM/bam-feetech,
Robonine's UART-servo backlash study, Klipper's calibration architecture, and
LeRobot's SCS table. The implementation now follows the reusable parts: an
excitation manifest, immutable raw captures, fixed-grid derived data, separate
reversal metrics, candidate scoring, and a persisted-but-bypassable profile.
The pendulum/CMA-ES friction fit, external backlash fixture, input-shaper FFT,
and unverified SCS009 physical parameters remain explicitly out of scope.

## 2026-09-13 — Auto-calibration v0.2 bounded screen

The v0.1 result remained factory/null, so the next step was a deliberately
small practical screen rather than another physical-model exercise. Motion Lab
now supports a RAM-only per-joint directional profile with independent command
deadbands, minimum smooth velocities, and velocity scales. The profile is
disabled outside the explicit diagnostic path and cannot be changed while a
Motion Lab run is active.

The host runner `tools/motion_lab/auto_calibrate_v2.py` compared six profiles
using identical 10°/2 s minimum-jerk joint-0 motions, both directions, two
repetitions. It screened CW/CCW deadband, P, startup force, and runtime
direction compensation; D was intentionally skipped because no endpoint
ringing was established. All valid rerun captures had fresh feedback and
7.8–8.2 V. The telemetry ranking was E (startup=32 plus runtime compensation),
D (P=12 plus runtime compensation), then factory A. The ranking is only a
guardrail; the human visual comparison is still required.

During the first screen, repeated restore commands exposed that a tightly
packed EEPROM write sequence could leave joint-0 P at the prior candidate even
when later fields restored. The restore implementation now inserts a bounded
delay between each register write, and a write/readback test confirmed P=15,
D=15, startup=24, and dead zones 1/1 after restoring from P=12. This is a
reproducibility and safety fix, not a motion-quality claim.

The required human comparison was completed afterward. Candidate E
(startup=32 plus runtime compensation), candidate D (P=12 plus the same
runtime compensation), and the factory/null candidate A were each run with
joint-0 10° minimum-jerk motions in both directions. The user reported no
meaningful perceptual difference; all remained visibly jerky. A final readback
confirmed joint 0 P/D/I=`0F/0F/00`, startup=`0x0018`, dead zones=`0x01/0x01`,
and `MLAB_COMP enabled=0`. v0.2 therefore closes as hardware-dominated with
the factory profile retained; no candidate is promoted and the next milestone
is Creature Motion rather than more low-level actuator tuning.

## 2026-09-16 — Deployment contract before conditioned characterization

The immutable cold J2 `+3°` evidence remains the first hardware record: r1
commanded 10 counts (2.9297°) and achieved 7 counts (2.0508°), while r2
commanded the same 10 counts and achieved 0 counts. Those runs were collected
with firmware that already supported experiment 4, and remain valid cold-run
evidence; they are not a repeatability claim.

The first conditioning-only attempt was saved under
`backups/motion-lab-conditioning-only-2026-09-16-positive-c0`. The host checkout
contained experiment-5 code, but the device was not redeployed: after the
preflight, the device returned `MLAB_CONSOLE run: invalid config` and emitted no
`MLAB` telemetry. No physical conditioning motion or formal run was therefore
confirmed. c0 is classified as a deployment/capability failure, not a servo or
conditioning result; `formal_run_started=false` and cleanup completed.

The process is now explicit: capture host branch/commit and device-reported
image identity separately, flash and verify with ESP-IDF monitor, query
`mlab caps`, and only then allow a capability-matched experiment. A malformed
capability response, unsupported experiment, invalid config without telemetry,
or any failed readiness/safety gate stops the protocol without an automatic
retry or follow-up movement.

## 2026-09-17 — First valid supervised conditioning-only run

The deployment gate passed on host firmware commit
`55bf4b3499df88d8fc9e857f5fee5d8063433750`. The device reported protocol 2,
experiments `0|1|2|3|4|5`, reversal support, and image ELF SHA
`24083392406d518d8a86ff748236167b29b24d3f86590fe59defad4ae9b0640f`.
Read-only preflight passed for all five servo IDs, the parameter snapshot was
complete, and the observed voltage was 8.0 V.

Exactly one supervised conditioning command was sent:

```text
mlab run 5 2 2 5 1000 0 8 30 250
```

The immutable capture is under
`backups/motion-lab-conditioning-only-2026-09-17-positive-hw1/`. The measured
commanded excursions were +19 counts (+5.5664°) and −21 counts (−6.1523°).
Feedback achieved +8 counts (+2.3438°) and −23 counts (−6.7383°), then returned
to the starting feedback count exactly (0 counts / 0° return error). Feedback
valid rate was 100%, unique sample rate 16.29 Hz, feedback-age P50/P95/max was
3/49/107 ms, stale fraction was 0%, voltage was 8.0–8.1 V, and peak raw load
was 1243. Raw speed values remain uncalibrated diagnostics.

The manifest records `physical_motion_started=true`,
`conditioning_passed=false`, `formal_run_started=false`, and `executed_runs=0`;
cleanup (`mlab stop`, polling off, compensation off) completed. The original
health result must not be read as a clean positive-direction breakaway failure:
the 1,000 ms minimum-jerk reference is dynamically infeasible at an 8°/s cap.
The ideal reversal leg alone requires about 18.75°/s, and a faithful offline
simulation of the firmware command integrator produces distorted endpoints
(approximately +5.69° / −6.05°, with velocity and acceleration limits active).
That explains the emitted +19/−21 counts versus the nominal ±17.07 counts;
the +8-count feedback excursion is therefore evidence from a confounded
experiment, not an actuator capability verdict. The raw telemetry still
confirms physical reversal response, healthy feedback, and exact return. No
formal +3° run, negative conditioning, or automatic retry was performed. No
human visual observation was supplied in the terminal record.

The conditioning protocol is corrected offline before another hardware run:
the same 8°/s and 30°/s² limits use a 2,500 ms transition, giving a 9,500 ms
center → +5° → −5° → center sequence with no simulated endpoint overshoot.

## 2026-09-17 — Stock idle-motion isolation check

The supervised conditioning-only capture
`backups/motion-lab-conditioning-only-2026-09-17-positive-hw1/` was run while
the stock idle behavior was visibly enabled before the experiment. The
firmware's direct-control branch disables idle motion as soon as it takes
ownership and restores the previous state when the run finishes. The
immutable UART capture confirms that this isolation held: during the 86-row
conditioning window, J0 and J4 command positions were constant (0-count
span), while only the selected J2 command changed. The observed J2
under-travel therefore must not be attributed to q0/q4 idle offsets.

To make the state controllable from the ESP-IDF monitor without an MCP client,
the UART console now provides `mlab idle off|on|status`. This is a RAM-only
switch; reboot restores the upstream default (enabled), and it does not write
servo EEPROM or alter Motion Lab's temporary isolation/restore behavior.

## 2026-09-17 — Corrected conditioning-only validation (c1)

The firmware branch `feat/scs009-dynamics-characterization` was verified at
host commit `0f6c2f8956c84a07eaeddbc8381233c2f096ea27`. It was rebuilt with
ESP-IDF v5.5.3, flashed through the ESP-IDF monitor, and verified before the
experiment. The device reported protocol 2, experiments `0|1|2|3|4|5`,
`reversal=1`, and image ELF SHA
`ec3fd0859ea989e336c6024a2ca766770448f8edabe1d1767a1d0dcccdb4986c`.
Read-only `mlab status` reported all five IDs online with zero current or
latched errors and no stale flags. `mlab idle status` reported enabled, then
`mlab idle off` was used only to simplify observation; Motion Lab direct-joint
ownership independently isolates idle during the run.

Exactly one supervised physical command was sent after the deployment gate:

```text
mlab run 5 2 2 5 2500 0 8 30 250
```

The immutable capture is under
`backups/motion-lab-conditioning-only-2026-09-17-positive-c1/`. The protocol
was the corrected minimum-jerk sequence:
`center → +5° → −5° → center`, 2,500 ms transitions, 1,000 ms holds, an 8°/s
velocity cap, and a 30°/s² acceleration cap (9,500 ms total).

Requested excursions were ±17.0667 counts (±5.0000°). The generated command
held +17 counts (+4.9805°) and −18 counts (−5.2734°): endpoint errors were
0.0667 counts (0.0195°) positive and 0.9333 counts (0.2734°) negative, both
inside the 1.5-count fidelity tolerance. The sampled command therefore had no
continuous-profile overshoot; the extra negative count is quantization-level
endpoint error. The offline-faithful profile predicts peak 7.50°/s and
9.2376°/s² on the 10° reversal leg, with neither configured limit active.
Finite differences of the sparse UART command samples showed apparent spikes
up to about 10.46°/s and 96.6°/s²; these are quantization/sampling artifacts,
not calibrated actuator limits.

Endpoint-hold feedback reached +10 counts (+2.9297°) and −22 counts
(−6.4453°); the raw negative peak was −23 counts (−6.7383°). The settled
return was 2 counts (0.5859°) below the starting feedback count, within the
5-count return gate. Feedback validity was 100%, with 162 unique samples at
17.10 Hz; feedback age was P50/P95/max 3/26/78 ms and stale fraction 0%.
Voltage was 8.0–8.1 V, peak raw load was 1273, and raw speed fields remain
uncalibrated diagnostics (`speed_cmd_raw=350` is a runtime command field).

The manifest records `status=conditioning_complete`,
`physical_motion_started=true`, `conditioning_passed=true`,
`formal_run_started=false`, and cleanup completed (`mlab stop`, polling off,
compensation off). The corrected command is therefore suitable as a
preconditioning motion for a later supervised experiment. Compared with the
older hw1 capture, c1 must be treated as a separate protocol: hw1 used the
infeasible 1,000 ms profile. C1 has faithful command endpoints and passes the
health gate, while the positive-direction feedback remains weaker in the
telemetry (10 versus 22 counts at the endpoint), so directional actuator
asymmetry remains the next hypothesis rather than a trajectory-generation
failure.

No human visual observation was included in the terminal record after this
run, so smoothness, sound/vibration, and subjective direction visibility are
intentionally left unclassified rather than inferred. No formal +3° run,
negative conditioning, retry, or EEPROM/PID/dead-zone/startup-force change was
performed.

## 2026-09-17 — Conditioned J2 +3° gate stopped before formal run (c1)

The host firmware checkout was at
`feat/scs009-dynamics-characterization` commit
`c91de1ded8b553ca3b03953f7a91b7e28782cca0`; the only difference from the
flashed runtime commit was the engineering-log documentation commit. The
device was not reflashed. ESP-IDF monitor readiness was rechecked first: the
device reported protocol 2, experiments `0|1|2|3|4|5`, `reversal=1`, all five
servos online, zero errors/stale flags, and ELF SHA
`ec3fd0859ea989e336c6024a2ca766770448f8edabe1d1767a1d0dcccdb4986c`.

The immutable capture is under
`backups/motion-lab-conditioned-j2-plus3-2026-09-17-c1/`. Exactly one
conditioning command was attempted:

```text
mlab run 5 2 2 5 2500 0 8 30 250
```

The generated command remained faithful to the corrected profile: requested
±17.0667 counts (±5°), held +17 counts (+4.9805°) and −18 counts (−5.2734°),
with endpoint errors 0.0667 and 0.9333 counts respectively. Feedback reached
only +4 counts (+1.1719°) at the positive endpoint and −20 counts (−5.8594°)
at the negative endpoint (raw negative peak −21 counts). The settled return
was +1 count (+0.2930°) from the starting feedback count.

Conditioning telemetry remained healthy: 100% valid rows, 163 unique samples
at 17.12 Hz, feedback-age P50/P95/max 3/17/78 ms, stale fraction 0%, voltage
7.9–8.1 V, and peak raw load 1303. Raw speed values remain uncalibrated; the
runtime `speed_cmd_raw` field was 350. The health gate failed only because the
positive feedback excursion (4 counts) was below 50% of the actual +17-count
command endpoint (`4.00 < 8.50`).

The harness stopped fail-closed before the formal command: the manifest records
`status=conditioning_failed`, `physical_motion_started=true`,
`conditioning_passed=false`, `formal_run_started=false`, and
`executed_runs=0`. No `mlab run 4 ...` command was sent. Cleanup completed
(`mlab stop`, polling off, compensation off), and the serial port was released.
The corrected conditioning trajectory itself is suitable and command-faithful,
but this run shows that the positive-direction conditioning response is not
repeatable: the prior corrected c1 conditioning capture reached +10 counts
under the same nominal protocol, whereas this capture reached +4 counts. A
conditioned formal +3° comparison against cold r1/r2 is therefore not yet
valid; the next hypothesis is direction-dependent/stiction or intermittent
actuator response, not command-profile distortion.

No human visual observation was supplied in the terminal record, so visibility,
smoothness, sound/vibration, and return quality are left unclassified. No
formal +3° run, retry, parameter tuning, EEPROM write, or additional physical
motion was performed.

## 2026-09-17 — First supervised ±8° conditioning / health-motion validation

The host plan was verified at PR #1 commit
`f3de25d29b221d2363b58b228d29a39035fbf26d`; only host-side harness,
documentation, and tests differ from the already flashed runtime image. The
device was checked through ESP-IDF monitor before motion and still reported
protocol 2, experiments `0|1|2|3|4|5`, `reversal=1`, all five servos online,
zero errors/stale flags, and ELF SHA
`ec3fd0859ea989e336c6024a2ca766770448f8edabe1d1767a1d0dcccdb4986c`.

Exactly one supervised conditioning-only command was sent:

```text
mlab run 5 2 2 8 4000 0 8 30 250
```

The immutable capture is under
`backups/motion-lab-conditioning-only-2026-09-17-positive-health8-hw1/`.
The requested ±8° endpoints are ±27.3067 counts. The generated command held
+27 counts (+7.9102°) and −28 counts (−8.2031°), with endpoint errors of
0.3067 and 0.6933 counts (0.0898° and 0.2031°); both are inside the 1.5-count
fidelity tolerance and show no material command overshoot beyond quantization.

Feedback reached +17 counts (+4.9805°) and −32 counts (−9.3750°); the raw
negative peak was −33 counts. The settled return was −3 counts (−0.8789°)
from the starting feedback count, within the 5-count return gate. Feedback was
100% valid with 237 unique samples at 16.90 Hz; age P50/P95/max was 3/31/73 ms
and stale fraction 0%. Voltage during the motion was 7.6–7.9 V, peak raw load
was 1273, and raw speed remains an uncalibrated diagnostic (`speed_cmd_raw=350`).

The unchanged 50% health gate required at least 13.5 counts (3.9551°) for the
quantized +27-count positive endpoint and 14.0 counts (4.1016°) for the
quantized −28-count endpoint. Both directions passed, so the manifest records
`status=conditioning_complete`, `physical_motion_started=true`,
`conditioning_passed=true`, `formal_run_started=false`, and cleanup completed
(`mlab stop`, polling off, compensation off). This is evidence that the ±8°
profile is a command-faithful and telemetry-healthy health motion; it is not a
small-signal capability measurement, and no formal +3° run was started.

The terminal record contains no post-run human description of visibility,
directional smoothness, sound/vibration, or return appearance. Visual
acceptance is therefore left pending rather than inferred from telemetry. If
the user confirms both directions were clearly visible and physically normal,
the ±8° motion can be adopted as the standard preconditioner; otherwise stop
preconditioning characterization and investigate the J2 direction-dependent
response without increasing amplitude automatically.

## 2026-09-17 — Conditioning / health-motion protocol revised to ±8°

The corrected ±5° conditioning command was valid offline, but it was not a
reliable health motion on J2. Two identical corrected-profile captures emitted
+17/−18 counts; feedback reached +10/−22 counts in one and only +4/−20 counts
in the next. The user also reported that the movement was barely visible.
Because ±5° overlaps the positive small-signal/breakaway phenomenon we are
trying to measure, it is retired as the standard preconditioner. The prior
captures remain immutable evidence and are not reinterpreted or deleted.

The new conditioning / health-motion candidate is evaluated offline only in
this change (no flash and no physical motion):

```text
center -> +8° -> −8° -> center
transition = 4000 ms; hold = 1000 ms
max velocity = 8°/s; max acceleration = 30°/s²
total duration = 14000 ms
```

The faithful deterministic simulator produces requested ±8.000° (±27.3067
counts), generated command endpoints +8.000°/−8.000° (+27/−27 counts), zero
endpoint-hold error, zero command overshoot, and exact return to center. The
16° reversal leg has analytical peak velocity 7.5000°/s and peak acceleration
5.7735°/s²; neither configured limit is active. The ±8° excursion remains
inside the existing ±10° Motion Lab safety envelope.

The health gate still compares feedback with the actual generated command
endpoint, keeping requested, commanded, and achieved excursions separate. At
the nominal ±8° request, the unchanged 50% threshold is 13.6533 counts,
equivalent to 4.0000°. For a quantized ±27-count command endpoint it is 13.5
counts (3.9551°). This motion is intended to break frictional history, establish
directional preload, verify obvious bidirectional motion, and return to center
from the negative side before a positive formal +3° test; it is not used to
estimate small-signal capability.

## 2026-09-17 — Conditioned J2 +3° after ±8° health motion (health8-c1)

The host checkout was verified at PR #1 branch
`feat/scs009-dynamics-characterization`, commit
`c6783288678292a5fbad17661438aeab764dbcd1`. The device was not reflashed;
ESP-IDF monitor readiness confirmed protocol 2, experiments `0|1|2|3|4|5`,
`reversal=1`, all five servos online, zero error/stale flags, and the known
runtime ELF SHA
`ec3fd0859ea989e336c6024a2ca766770448f8edabe1d1767a1d0dcccdb4986c`.

The immutable capture is under
`backups/motion-lab-conditioned-j2-plus3-2026-09-17-health8-c1/`. Exactly one
conditioning-plus-formal attempt was made with `--max-runs 1`:

```text
conditioning: mlab run 5 2 2 8 4000 0 8 30 250
formal:       mlab run 4 2 2 3 3000 0 8 30 250
```

The ±8° conditioning motion passed its existing health gate. Its generated
commands reached +27/−28 counts (+7.9102/−8.2031°); feedback reached +16/−31
counts (+4.6875/−9.0820°), and the settled return was exactly 0 counts from
the conditioning start. Conditioning feedback was 100% valid at 16.96 Hz;
feedback age P50 was 3 ms, P95 35 ms, and maximum 78 ms; stale rows were 0.
Voltage was 7.7–8.0 V and peak raw load was 1303. These values are kept
separate from the formal small-signal metrics.

The formal J2 positive +3° command requested 10.24 counts (3.0000°) and
actually generated a +10-count (+2.9297°) endpoint. Feedback achieved only
+1 count (+0.2930°), an achieved/commanded ratio of 0.10; during the command
hold, feedback remained approximately 146–151 while the commanded endpoint
was 160 (starting feedback 150). Final feedback was 149, or −1 count
(−0.2930°) from the starting position. No command overshoot or feedback jump
event was reported, but settling/onset were not meaningful after the safety
abort. Tracking RMS error was 2.4683°, peak error 4.1016°, exceeding the
2.25° small-motion sanity guard. The run therefore stopped fail-closed after
the single formal command; this guard is not a calibrated actuator limit.

Formal telemetry was 100% valid at 16.78 Hz; feedback age P50/P95/max was
3/36/117 ms and stale fraction 0%. Voltage was 7.9–8.1 V, peak raw load was
1243, and raw speed values (0–32818) remain uncalibrated diagnostics. The
manifest records `conditioning_passed=true`, `formal_run_started=true`,
`status=aborted`, and cleanup completed (`mlab stop`, polling off,
compensation off). No retry, negative formal test, EEPROM write, or servo
parameter change was performed.

Compared only with the formal cold baselines, the conditioned result was
`1/10` feedback counts versus cold r1 `7/10` and cold r2 `0/10`. One run is
not a repeatability claim. The result does not support an improvement from
the ±8° preconditioner; substantial small-signal loss remains, so internal
dead-zone/startup-force or low-speed stiction/control behavior remain stronger
hypotheses. The conditioning motion itself passed, but the formal run's
tracking guard failure means no further physical experiment should be started
automatically.

The previous ±8° visual observation is preserved: both directions showed a
similar stick/static-friction-like hesitation, with no obvious qualitative
direction difference by eye. No new post-run human visual description was
available in the terminal record, so visibility, smoothness, sound/vibration,
and subjective comparison of this run remain unclassified rather than
inferred.

## 2026-09-17 — Nano printf telemetry correction and read-only revalidation

The Creature Stream firmware was rebuilt from `00b291ad98879be09c59103e07aa1dbabfc31526`
on `feat/creature-stream-runtime` and flashed using ESP-IDF 5.5.3. Device ELF
SHA-256:
`3a0a8b98101024d3ea0c0d2888698812f6bb18f4982f0e832b9070adfa399c2a`.

The preceding malformed `CREATURE_STATE`/`MLAB_SERVO_STATUS` lines were caused
by `CONFIG_LIBC_NEWLIB_NANO_FORMAT=y`: Nano `printf` does not implement the
64-bit integer formatters used for diagnostic timestamps. Internal watchdog,
freshness, and feedback timestamps remain 64-bit. Console timestamps now use
explicit `uint32` millisecond projections and `%lu`; the source tree has a
regression guard against reintroducing `%ll`/`PRI*64` formatters in the ARM
console sources.

After flashing, only read-only monitor commands were sent (`mlab caps`,
`mlab status`, `creature caps`, and `creature state`, with a repeated state
sample). No `creature take`, target, Motion Lab command, or physical movement
was performed. The device reported `MLAB_CAPS` protocol 2 with experiments
`0|1|2|3|4|5` and `reversal=1`; `CREATURE_CAPS` reported five joints. The
corrected state lines contained numeric `last_target_ms=0`, five feedback
positions/mdeg values, `fb_stale=0|0|0|0|0`, `servo_error=0|0|0|0|0`, and
`voltage_v=7.90`. Feedback ages remained finite (224|162|69|62|1 ms in the
first sample and 94|74|54|12|135 ms in the repeat captured after boot settling),
so the earlier shifted fields and `0.00 V` were formatting corruption rather
than evidence of a new actuator or supply fault. The monitor exited cleanly
and `/dev/ttyACM0` was free afterward.
