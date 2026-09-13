# ADR 0003: Verify supply voltage and runtime speed before servo tuning

## Status

Accepted — 2026-09-13

## Context

The root joint looked slower and showed visible stick-slip during very-low-speed
minimum-jerk motion. A persistent servo speed/time register change or supply-voltage
drop could mimic an actuator-control problem, so those causes must be separated
before changing P, D, deadband, or startup force.

## Decision

- Treat the existing `0x2A` sync-write velocity field as a runtime command and
  log it as `speed_cmd_raw` for every Motion Lab row.
- Do not add or modify a persistent speed/time parameter. Use the raw SCS009
  factory snapshot as the EEPROM/control-table baseline.
- Sample ID-1 present voltage read-only at a bounded 1 Hz diagnostic rate and
  include the latest value in experiment telemetry.
- Keep internal servo tuning deferred until repeatability and voltage checks are
  complete; restore and read back the factory snapshot between later parameter
  groups.

## Evidence

- `speed_cmd_raw` remained `350` during the 3-second minimum-jerk capture.
- ID-1 reported `8.00 V` at idle and `8.00–8.10 V` during motion.
- Post-check P/D/I, startup force, dead zones, baud, and return delay matched the
  saved factory snapshot for all five servos.

## Consequences

The slowdown is not explained by a persistent speed-setting change or a sustained
low-voltage condition. Very short supply transients and mechanical friction remain
possible; future experiments should retain voltage, feedback age, and load fields
when evaluating those hypotheses.
