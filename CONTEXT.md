# RIG-Arm engineering context

This glossary records the terms used by the low-level motion and characterization
work. It is intentionally about the system domain, not the implementation.

## Motion Lab

A controlled diagnostic surface for repeatable joint experiments. It owns direct
joint targets only while an experiment is active and reports commanded and
feedback values for later analysis.

## Command position

The position requested by the experiment in the servo's calibrated count space.
It is distinct from the last packet that happened to be transmitted when a
deadband suppresses a redundant write.

## Feedback sample

A position, speed, and load observation returned by one servo in one status
packet. A sample is useful only when its acquisition time and age are known.

## Feedback age

The elapsed time between the telemetry timestamp and the most recent valid
feedback sample for a joint. Age is a freshness measure, not a motion estimate.

## Servo characterization

A measurement phase that relates repeatable trajectories and safe mechanical
conditions to the observed servo response before changing internal servo
parameters.

## Factory parameter snapshot

A read-only record of the control and safety registers currently stored in each
servo. It is a baseline for comparison and is not itself a tuning operation.

## Direct-joint experiment

An intentionally isolated diagnostic motion that bypasses IK and stock idle or
preset actions. It is not the production behavior interface.
