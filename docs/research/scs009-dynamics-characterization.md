# SCS009 usable-dynamics characterization: source review

Status: research note for PR #6. This note is source review and experiment
context only; it does not change firmware, servo registers, simulation limits,
or production motion behavior.

Review date: 2026-09-16

## Identity and electrical context

The RIG-Arm secondary-development guide identifies the arm as a five-axis arm
using five `scs009` serial-bus servos, addressed as IDs 1--5 over the arm UART.
That establishes the family and installed quantity, but it does not by itself
prove the exact commercial suffix in the assembled hardware. See the [RIG-Arm
development guide](https://wiki.xgorobot.com/kb/pet-arm/f7ae72301c174cbf915284b728655236),
especially the hardware table and servo communication section.

The closest first-party identity match for the dual-shaft hardware is FEETECH
`SC-0090-C013`; the corresponding official specification document names the
model `SCS0009-C013`. FEETECH's product page describes it as a 6 V, 2.3 kg-cm
dual-axis serial-bus servo and lists the same small form factor. The official
datasheet should be treated as the more precise source for characterization
conditions:

| Item | Official source value | Characterization implication |
| --- | --- | --- |
| Nominal test voltage | 6 V typical | Use vendor speed/torque values only as a sanity reference. |
| Product-page operating range | 4.8--7.4 V | Do not assume the RIG-Arm rail is within this range without measuring it. |
| Datasheet input range | 4.0--7.4 V; typical entries at 4.8 V and 6 V | The wider datasheet input line does not make higher voltage a validated operating point. |
| No-load speed | 0.125 s/60° at 4.8 V; vendor materials cite roughly 0.07--0.10 s/60° at 6 V (the current C013 sheet lists 0.1 s/60° / 100 RPM; older/common SCS009 references cite about 0.07 s/60°) | The documents are not a single validated installed-arm measurement. No-load speed is not an assembled-arm usable-speed limit; do not convert it directly into a Ruckig limit. |
| Rated / stall torque | 0.75 / 2.3 kg-cm at 6 V | Not a target for a stall or maximum-performance test. |
| Feedback | position, speed, load, input voltage, temperature | The available fields are useful diagnostics, but the raw speed/load encodings still need model-specific interpretation. |
| Resolution / travel | 0.293° (300°/1024), 300° command range | Quantization is material for small-amplitude tests; onset and velocity estimation must use sustained movement and timestamped samples. |
| Mechanical size | Product page: 23.2 × 12 × 25.5 mm; datasheet drawing/spec: 23.2 × 12.1 × 25.25 mm | Treat these as nominal variant dimensions, not as the assembled printed-body measurement. |
| Backlash | Datasheet: gear backlash ≤ 0.5° | A reversal measurement is a lost-motion proxy, not pure gear backlash. |
| Protection | Datasheet describes over-voltage protection above 9 V or below 4.5 V, plus overload/over-temperature protection | This is a protection behavior, not permission to characterize around the protection threshold. |

Sources: [FEETECH SC-0090-C013 product page](https://www.feetechrc.com/en/6v-23kg-cm-dual-axis-serial-bus-steering-gear.html)
and the [official SCS0009-C013 product specification PDF](https://www.feetechrc.com/Data/feetechrc/upload/file/20260622/6391771868598044632953633.pdf),
pages 3--5 and 8.

### Two source ambiguities to preserve

1. FEETECH's product-page prose says “25T output shaft”, while its parameter
   table says `20T/OD3.95mm`; the official C013 datasheet also says
   `20T/OD3.95mm`. The characterization harness does not rely on horn spline
   count, but assembly/CAD work should use the datasheet value provisionally
   and validate it against the physical part.
2. The installed RIG-Arm hardware has previously reported approximately
   7.9--8.1 V through servo present-voltage telemetry. That is above the
   product-page/datasheet stated normal input range, even though it remains
   below the datasheet's stated over-voltage protection threshold. It is an
   unresolved electrical-context discrepancy, not a valid basis for scaling
   the 6 V no-load speed. Preserve voltage in every run and do not change the
   supply or persistent servo parameters in this PR.

The prior observation is recorded in
[`firmware/docs/engineering-log.md`](../engineering-log.md),
under “Servo voltage telemetry” and “Voltage-enabled repeat captures”. It was
reported by the servo at 0.1 V resolution, not measured with an oscilloscope;
very short rail transients therefore remain outside that evidence.

## What the current firmware actually measures and commands

The current top-level checkout points the `firmware` submodule at
`0037330761a0b0ed26d3eb739b9e0a94e3c8656b`, branch
`feat/creature-runtime` (the Motion-Lab-containing branch), not the firmware
`main` branch. Verify this relationship before creating the PR branch:

```bash
git -C firmware rev-parse HEAD
git -C firmware branch --show-current
git -C firmware log -1 --oneline
```

The existing Motion Lab path is already the appropriate execution seam:

- `firmware/main/boards/arm/motion_lab.{h,cc}` owns isolated direct-joint
  experiments, minimum-jerk/other baseline trajectories, signed amplitudes,
  safety windows, command deadband, and command cadence.
- `firmware/main/boards/arm/xgo.cc` keeps feedback polling outside the 2 ms
  command loop, bounds retries, marks stale joints, and exposes selected-joint
  high-rate polling through `xgo_feedback_poll_config()` and
  `xgo_feedback_poll_print_stats()` in `xgo.h`.
- The UART telemetry contract emits timestamp/age, position, raw speed, raw
  load, stale flags, servo-reported voltage, and `speed_cmd_raw`. The latter is
  the runtime velocity field in the existing sync write; it is not a persistent
  EEPROM speed/time setting. See
  [`firmware/docs/servo-characterization.md`](../servo-characterization.md).
- `firmware/tools/motion_lab/capture_serial.py` drains the serial stream into an
  immutable raw file. `analyze_stutter.py` parses rows, conservatively resamples
  timestamped data, preserves raw event timing, and already computes tracking,
  stale/age, voltage/load, dwell/jump, overshoot, direction, and reversal
  lost-motion proxy metrics.

The read-only `mlab params` command in `xgo.cc` captures the current SCS-series
control-table bytes for all five IDs, including P/D/I, minimum startup force,
dead zones, position/voltage/temperature limits, and protection entries. It
does not unlock EEPROM or issue a write. The saved factory snapshot in the
firmware engineering log is the restore/audit baseline. PR #6 must not repeat
the prior P/D/deadband/startup-force screening and must not write these
registers.

## Dynamics interpretation and protocol boundaries

The vendor's no-load speed is a sanity check only. The assembled arm adds
printed-link inertia, posture-dependent gravity/inertia, gearbox friction,
direction asymmetry, command/feedback bus latency, and quantized feedback. A
usable expressive envelope must therefore be reported per joint, direction,
amplitude, posture, and duration/repetition—not as one conversion from
`0.1 s/60°` to a universal joint speed.

For PR #6, use the current factory parameters, current supply, and existing
Motion Lab safety gates. The first accepted condition should be one visible,
centered representative joint, not automatically joint 0. Progress from 3° to
5° to 10° in both physical directions, with at least three repetitions and a
hold before return. A tier is accepted only after feedback remains valid,
tracking and load are ordinary, and the arm stays away from limits. Do not
intentionally stall or approach the voltage/protection thresholds.

Report these as measured observables: valid/sample rate, age/stale fraction,
command/achieved excursion, RMS/peak tracking error, robust motion-onset
latency, 10--90% movement time, observed peak velocity, settling, overshoot,
direction asymmetry, reversal lost-motion proxy, voltage range, raw load range,
and raw servo-speed range. Do not call raw `fb_speed_raw` calibrated angular
velocity until its C013 encoding is independently verified. Likewise, sparse
quantized position feedback cannot support a trustworthy physical jerk claim;
acceleration should be labeled low confidence and derived only where sample
density supports it.

The proposed `SCS009_CHARACTERIZED_SIM_PROFILE` should remain a proposal in
this PR. It may contain a conservative velocity value and an acceleration
range with uncertainty, but it must not change the current simulation/Ruckig
defaults until the hardware evidence is reviewed. Do not invent a measured
maximum jerk from this telemetry.

## References

- [FEETECH SC-0090-C013 product page](https://www.feetechrc.com/en/6v-23kg-cm-dual-axis-serial-bus-steering-gear.html)
- [FEETECH SCS0009-C013 official datasheet](https://www.feetechrc.com/Data/feetechrc/upload/file/20260622/6391771868598044632953633.pdf)
- [Luwu RIG-Arm secondary-development guide](https://wiki.xgorobot.com/kb/pet-arm/f7ae72301c174cbf915284b728655236)
- [Firmware Motion Lab guide](../motion-lab.md)
- [Firmware servo-characterization guide](../servo-characterization.md)
- [Firmware engineering log](../engineering-log.md)
- [`motion_lab.cc`](../../main/boards/arm/motion_lab.cc)
- [`xgo.h`](../../main/boards/arm/xgo.h)
- [`xgo.cc`](../../main/boards/arm/xgo.cc)
- [`capture_serial.py`](../../tools/motion_lab/capture_serial.py)
- [`analyze_stutter.py`](../../tools/motion_lab/analyze_stutter.py)
