# Creature Stream UART protocol

Creature Stream is the firmware-side execution owner for PC-generated joint
targets. It is deliberately separate from Motion Lab: Motion Lab remains a
diagnostic direct-joint path, while Creature Stream forwards final joint-space
targets and never runs IK, canned emotional clips, extra smoothing, EEPROM
writes, or actuator compensation.

## Deployment and monitor workflow

From WSL, use `rig_env` (or source `~/esp/esp-idf/export.sh` and export
`RIG_PORT`) in `firmware/`:

```bash
idf.py reconfigure
idf.py build
idf.py -p "$RIG_PORT" flash monitor
```

Wait for `MLAB_CONSOLE ready`, then issue the read-only checks:

```text
mlab caps
mlab status
creature caps
creature state
```

Exit monitor cleanly and verify the port is free before starting the PC
backend. Never run monitor and `pc.SerialHardwareBackend` concurrently.

## Commands and responses

```text
creature caps
creature take
creature target <seq> <q0_mdeg> <q1_mdeg> <q2_mdeg> <q3_mdeg> <q4_mdeg>
creature state
creature stop
creature release
creature help
```

`creature caps` emits `CREATURE_CAPS` with `protocol=1`, `joints=5`,
`units=mdeg`, `max_stream_hz=40`, `watchdog_ms=250`, project/version, and the
ESP-IDF app ELF SHA-256. The target parser accepts exactly one uint32 sequence
and five signed int32 millidegree values; malformed, out-of-order, and
out-of-range targets are rejected. The PC demo sends a deterministic 33.333 Hz
schedule (every third 100 Hz sample); firmware still enforces the independent
25 ms physical write ceiling and never lets an incoming target bypass it.

`creature take` requires calibration, teach, and Motion Lab to be idle and all
five feedback samples healthy. It snapshots stock-idle state, disables idle
and preset actions, initializes the target from current feedback, and enters
`hold`; taking ownership is inert and does not start the watchdog. The first
accepted target transitions `hold → active` and starts the 250 ms watchdog.
The ACK includes ownership, sequence, target age and feedback freshness.

`CREATURE_STATE` exposes both `fb_pos` (raw absolute SCS009 counts) and
`fb_mdeg` (the calibrated **model-space** joint angle). The conversion first
computes the installed-servo angle relative to each servo's `ZeroPos`, then
applies the authoritative installation mapping `[+1,+1,-1,+1,-1]`. Host code
must use `fb_mdeg` for `q_rad`; raw counts and installed-servo signs are
diagnostic/boundary details only. `target_mdeg` uses the same model-space
convention and is converted back to installed-servo angles only immediately
before a bus command.

The internal target timestamp remains a 64-bit microsecond value for watchdog
and freshness calculations. The machine-readable `last_target_ms` field is an
explicitly narrowed `uint32` millisecond diagnostic projection (and may wrap
after a long uptime) so it remains compatible with the ESP-IDF Nano `printf`
configuration. Do not widen this formatter back to a 64-bit `%ll` conversion;
preserve the internal timestamp type instead.

`creature stop` enters HOLD and continues forwarding the current measured
posture when available. It does not snap to IK or neutral. `creature release`
ends ownership, cancels action flags, and restores the idle enable state that
was captured by `take`; it is not an emergency command.

## Watchdog and owner priority

If no valid target arrives for 250 ms, the stream enters `timed_out` HOLD. If
feedback becomes stale or a servo reports a non-zero error while active, it
enters `fault_hold`. In both cases it prefers fresh measured joint positions
per joint and otherwise retains the last safe target, never resumes stock
idle/IK automatically, and requires an explicit retake. The xgo boundary is:

```text
parameter dump > Motion Lab > Creature Stream > teach/calibration > normal IK/actions/idle
```

The 2 ms xgo task only checks the stream state and writes a target when its
40 Hz cadence is due; feedback and telemetry remain outside that critical
path.

## Host use

`pc.SerialHardwareBackend` converts the project convention `q_rad[5]` to
integer mdeg, verifies protocol/joint/unit/rate/watchdog capabilities, performs
the hold-aware caps/state/take handshake, enforces simulation position limits
and send-rate bounds, and fails closed on stale feedback, servo errors, owner
changes, watchdog state, malformed state, or rejected targets. Normal success
uses explicit `normal_stop_release()`; abnormal paths use
`emergency_hold_close()` and never release ownership. Feedback is queried
periodically for diagnostics and is never used to create a second controller.

Firmware independently validates each target against the authoritative IK
limits (J0 ±2.62, J1 ±1.57, J2 −0.50..2.50, J3 ±1.50, J4 −1.20..1.30 rad)
before converting it to servo counts. Failed runs retain their manifest,
command/feedback traces, event trace, and summary for post-mortem analysis.
