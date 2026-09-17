# Motion Quality Baseline

The `feat/creature-runtime` firmware branch carries a deliberately isolated
diagnostic path for the RIG-Arm.
It does not replace IK or the stock action system. The path is inactive unless
explicitly started through its MCP tools.

## Safety model

- The experiment starts from the latest servo feedback positions, not a hard-coded pose.
- It rejects the run until all five joints have valid feedback.
- Direct joint targets are limited to a conservative 100--923 count diagnostic window,
  in addition to the stock `[1, 1023]` command clamp.
- A run captures and disables idle motion, clears `Action_ID`, and gives Motion Lab
  exclusive direct-joint control. On finish or stop it sends the start pose and restores
  the prior idle-motion setting.
- The initial UI limits amplitude to 10 degrees. Begin at 3 degrees or less.
- Do not run any experiment with hands, cables, or objects in the arm's pinch zones.

Motion Lab does not write servo EEPROM, zero calibration, mechanical limits, PID, or NVS.

## Build and flash

After changing branches or pulling this module into an existing build directory,
refresh CMake once, then build from `firmware/`:

```bash
idf.py reconfigure && idf.py build
```

After the build succeeds, flash the same image and keep the ESP-IDF monitor open
for the boot/readiness check. A host checkout or build log alone does not prove
which image is running on the device:

```bash
rig_env
idf.py -p "$RIG_PORT" flash monitor
```

Wait for the boot banner and `MLAB_CONSOLE ready`, then query the read-only
capability contract before any experiment:

```text
mlab caps
mlab help
mlab status
```

`mlab caps` reports the capability protocol, supported experiment IDs, reversal
support, and the running image identity (project/version/build timestamp/ELF
SHA). Experiment 5 and reversal must be advertised before a conditioned
characterization run. If the response is missing, malformed, stale, or does
not advertise the required experiment, stop and flash/verify the intended
image; do not send `mlab run`.

Only after those read-only checks should you exit the monitor and run an
automated capture helper. Never run `idf.py monitor` and
`tools/motion_lab/capture_serial.py` against the same port at the same time.
The helper only sends the command and captures UART; it does not flash firmware
and its output is not evidence that the intended image is installed.

## Local serial console (recommended)

The USB serial monitor is the direct human control interface. After the boot
logs show `MLAB_CONSOLE ready`, type a command in the `idf.py monitor` window
and press Enter. This does not require a cloud service, an MCP client, or a
browser connection.

```text
mlab help
mlab caps
mlab idle off|on|status
mlab hold
mlab visible [joint]
mlab run <experiment> <trajectory> <joint> <amplitude_deg> <duration_ms> <stagger_ms> <max_velocity_deg_s> <max_acceleration_deg_s2> <deadband_mdeg>
mlab stop
```

`mlab idle off` disables the stock q0/q4 idle micro-motion in RAM and leaves
the arm on its normal IK pose; `mlab idle on` restores it. `mlab idle status`
is read-only. These settings are not EEPROM parameters and currently reset to
the upstream default (enabled) after reboot. Motion Lab itself always
temporarily disables idle motion while a run owns the direct-joint path, then
restores the state that was active before the run.

The first controlled test is simply:

```text
mlab hold
```

It holds the current feedback pose for ten seconds with stock idle motion
disabled. It is the safe way to establish whether there is background chatter.

For visible bring-up validation, run `mlab visible 0`. It uses the direct
single-joint path (no IK): 10 degrees out over 1.5 seconds, holds one second,
then returns over 1.5 seconds. It remains inside the diagnostic safety window.

## MCP tools (optional)

The firmware also registers `self.arm.motion_lab.run` for a future MCP client
that is actually connected to the device. A normal ChatGPT browser tab is not
such a client, so use the local serial console above for this milestone.

The MCP run tool accepts the following integer fields. `mlab run` accepts every
field except `hold_ms`; its step-hold-return mode uses the safe one-second
default, and `mlab visible` is the recommended visible test.

| Field | Meaning | Safe initial value |
| --- | --- | --- |
| `experiment` | `0` single joint, `1` synchronized five-joint, `2` staggered five-joint, `3` hold, `4` step-hold-return, `5` reversal lost-motion proxy | `0` |
| `trajectory` | `0` linear, `1` cubic ease, `2` minimum-jerk | `2` |
| `joint` | zero-based joint for experiment `0` | `0` |
| `amplitude_deg` | positive direct-joint offset; must be `0` for hold | `3` |
| `duration_ms` | complete out-and-back time, or hold time | `2000` |
| `hold_ms` | peak hold time for experiment `4` or each endpoint of experiment `5` | `1000` |
| `stagger_ms` | start delay between joints for experiment `2` | `120` |
| `max_velocity_deg_s` | trajectory output velocity limit | `45` |
| `max_acceleration_deg_s2` | trajectory output acceleration limit | `180` |
| `deadband_mdeg` | command change threshold in millidegrees | `250` |

Use `mlab stop` (or `self.arm.motion_lab.stop` from a connected client) to end
a run early and return to its start pose.

## Reproducible hardware baseline

Wait at least one second after boot before starting so all five feedback slots have been
polled. Run one experiment at a time and save the monitor output for each run.

1. Fixed-pose chatter control: type `mlab hold`.
   This distinguishes stock idle motion from servo, power, or mechanical chatter.
2. Visible mapping/tracking validation: `mlab visible 0`. Confirm `cmd_pos[0]`
   rises by about 34 counts and `fb_pos[0]` follows before any trajectory tuning.
3. Single-joint trajectory comparison: `mlab run 0 0 0 10 3000 120 20 60 250`,
   then repeat with trajectory `1` and `2` in the second field.
4. Synchronized sweep: `mlab run 1 2 0 5 2500 120 45 180 250`.
5. Staggered sweep: `mlab run 2 2 0 5 2500 120 45 180 250`.

The serial monitor emits a CSV header and `MLAB` rows at 20 Hz while a run is active:

```text
MLAB,ts_ms,experiment,trajectory,elapsed_ms,total_ms,cmd_deg[5],cmd_pos[5],fb_pos[5],fb_speed_raw[5],fb_load_raw[5]
```

Fields separated by `|` are ordered joint 0 through joint 4. `fb_load_raw` is the
servo's raw feedback torque/load proxy and must not be interpreted as calibrated force.
The stock protocol feedback arrives one servo at a time, so the five feedback values in
a row have different ages (up to roughly 500 ms in the existing polling design).

## Host curve regression test

```bash
python3 tools/motion_lab/test_trajectory.py
```

This verifies endpoint and monotonic properties of linear, cubic-ease, and minimum-jerk
curves. Firmware build validation remains `idf.py build`.

## Usable dynamics characterization (PR #6)

The reusable supervised harness is
`tools/motion_lab/characterize_dynamics.py`. It reuses the raw capture and
telemetry contract above; it does not write servo registers or change the
simulation/Ruckig defaults. The default invocation is manifest-only and never
opens a serial port:

```bash
python3 tools/motion_lab/characterize_dynamics.py \
  --manifest-only \
  --output-dir backups/motion-lab-dynamics-YYYY-MM-DD \
  --joint 2
```

J2 is the documented first representative joint: it is a visible forearm-pitch
axis, the validated neutral pose is away from its model limits, and this choice
does not assume that the earlier J0 stutter is the whole-arm limit. The harness
plans 3/5/10 degree positive and negative minimum-jerk step/hold/return runs,
progressive gentle/moderate/brisk/expressive tiers, three repetitions, selected
joint high-rate feedback, low-speed probes, and a separate experiment-5
reversal lost-motion proxy. It prints the exact first command before any
hardware action.

Only a human-supervised invocation may move the arm. The first invocation is
strictly bounded to one movement:

```bash
python3 tools/motion_lab/characterize_dynamics.py \
  --execute --confirm-hardware --port "$RIG_PORT" \
  --output-dir backups/motion-lab-dynamics-YYYY-MM-DD \
  --joint 2 --max-runs 1
```

The harness is on the reviewed firmware branch. If the parent repository still
has its unchanged submodule checkout, select the branch first:

```bash
git fetch origin feat/scs009-dynamics-characterization
git switch feat/scs009-dynamics-characterization
```

The default execution limit is also one movement. Increase `--max-runs` only
after reviewing the previous capture; the plan itself remains deterministic
and begins with the gentle J2 `+3°` condition before low-speed, reversal, or
faster tiers.

The conditioning / health-motion prelude uses the feasible reversal profile
`mlab run 5 2 <joint> 8 4000 0 8 30 250` (total 14 s), preserving
`center -> +8° -> -8° -> center` without velocity/acceleration distortion.
The ±8° amplitude is intentionally inside the existing ±10° diagnostic
envelope but is large enough to establish bidirectional motion and a known
preload state. The retired ±5° motion was command-faithful yet overlapped the
J2 positive-direction breakaway regime and was too subtle to observe reliably;
it is retained only as historical evidence. The first formal movement remains a small J2 `+3` degree endpoint using
`mlab run 4 2 2 3 3000 0 8 30 250`, followed by a one-second hold and a
return. Before it, the harness requires parseable status for IDs 1–5, a
completed read-only `mlab params` snapshot, and an acknowledged voltage reply.
It stops before any later movement if preflight fails. Raw UART files are
immutable and are hashed in `manifest.json`; derived `normalized.csv` and
`dynamics_report.json` retain the firmware SHA and experiment parameters.
Voltage is compared with the observed ~8 V device baseline, not presented as a
validated electrical safety range. Raw load and speed values remain
observational diagnostics, not calibrated physical limits.

Before that preflight, the harness sends `mlab caps` and records the running
device identity separately from the host Git commit. A conditioned run requires
capability protocol 2, experiment 5, and reversal support; a cold formal run
requires experiment 4. Missing/malformed capabilities or a capability mismatch
are deployment/readiness failures and no physical `mlab run` is sent. If a
device replies `invalid config` and emits no `MLAB` telemetry, the manifest
records `physical_motion_started=false` and classifies the result as deployment
failure rather than actuator evidence.

To analyze a prior capture directory without touching hardware:

```bash
python3 tools/motion_lab/characterize_dynamics.py \
  --analyze-only --output-dir backups/motion-lab-dynamics-YYYY-MM-DD
```

The dedicated reversal experiment executes center → signed endpoint → opposite
endpoint → center. Its command travel before sustained opposite feedback is a
lost-motion proxy that includes servo deadband, quantization, bus delay, and
control behavior; it is not a claim of pure gear backlash. Cleanup always sends
`mlab stop`, `mlab poll off`, and `mlab comp off` so normal Motion Lab ownership
and polling are restored after a run or user abort.
