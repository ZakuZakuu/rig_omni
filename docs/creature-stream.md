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
out-of-range targets are rejected. Valid targets are swapped atomically and
forwarded at at most 40 Hz.

`creature take` requires calibration, teach, and Motion Lab to be idle and all
five feedback samples fresh. It snapshots stock-idle state, disables idle and
preset actions, and initializes the target from current feedback, so taking
ownership is inert. The ACK includes ownership, sequence, target age and
feedback freshness.

`creature stop` enters HOLD and continues forwarding the current measured
posture when available. It does not snap to IK or neutral. `creature release`
ends ownership, cancels action flags, and restores the idle enable state that
was captured by `take`; it is not an emergency command.

## Watchdog and owner priority

If no valid target arrives for 250 ms, the stream enters `timed_out` HOLD. It
prefers fresh measured joint positions, otherwise the last valid target, and
never resumes stock idle/IK automatically. A host must explicitly retake
ownership. The xgo boundary is:

```text
parameter dump > Motion Lab > Creature Stream > teach/calibration > normal IK/actions/idle
```

The 2 ms xgo task only checks the stream state and writes a target when its
40 Hz cadence is due; feedback and telemetry remain outside that critical
path.

## Host use

`pc.SerialHardwareBackend` converts the project convention `q_rad[5]` to
integer mdeg, performs caps/state/take handshake, enforces simulation position
limits and send-rate bounds, and provides `stop`/`release` cleanup on errors.
Feedback is queried periodically for diagnostics and is never used to create a
second controller.

