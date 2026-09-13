# ADR 0001: Isolate the Motion Lab and use minimum-jerk as its baseline

- Status: accepted
- Date: 2026-09-13

## Context

The stock firmware can continuously add idle motions or execute preset actions,
which makes a small servo experiment ambiguous. Early 3-degree visual tests were
also below a reliable observation threshold. Several trajectory shapes are
useful for comparison, but continuing to add variants would expand the test
surface before the actuator and feedback path are understood.

## Decision

The Motion Lab owns direct joint targets only while an experiment is active. It
temporarily disables stock idle/preset ownership, restores it on completion, and
keeps the existing safety/calibration limits. Minimum-jerk is the characterization
baseline; linear and cubic remain regression/comparison options, not new product
behavior.

## Consequences

- A 10-degree visible step/hold/return test is available for safe bring-up.
- All later load, posture, and duration comparisons can use one fixed trajectory
  definition.
- Direct-joint experiments must remain explicitly diagnostic and must not silently
  replace the production IK path.
