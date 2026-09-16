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

Because a full flash takes longer than the Codex terminal allowance, use an interactive
WSL terminal to flash and monitor:

```bash
rig_env
idf.py -p "$RIG_PORT" flash monitor
```

## Local serial console (recommended)

The USB serial monitor is the direct human control interface. After the boot
logs show `MLAB_CONSOLE ready`, type a command in the `idf.py monitor` window
and press Enter. This does not require a cloud service, an MCP client, or a
browser connection.

```text
mlab help
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
| `experiment` | `0` single joint, `1` synchronized five-joint, `2` staggered five-joint, `3` hold, `4` step-hold-return | `0` |
| `trajectory` | `0` linear, `1` cubic ease, `2` minimum-jerk | `2` |
| `joint` | zero-based joint for experiment `0` | `0` |
| `amplitude_deg` | positive direct-joint offset; must be `0` for hold | `3` |
| `duration_ms` | complete out-and-back time, or hold time | `2000` |
| `hold_ms` | peak hold time for experiment `4` | `1000` |
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
