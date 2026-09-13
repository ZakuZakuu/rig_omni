# Servo Characterization

This phase starts only after the Motion Lab baseline. Its first firmware build
improves measurement freshness and adds a read-only SCS009 parameter snapshot;
it does not change servo EEPROM values or add an external PID loop.

## Feedback contract

The servo bus is polled by `xgo_feedback_poll`, not by the 2 ms command loop.
One ID is requested every 20 ms, so the nominal complete five-ID cycle is 100 ms
(about 10 Hz per joint). Motion Lab telemetry remains 20 Hz and adds:

- `fb_ts_ms[5]`: the millisecond timestamp at which each status packet was
  accepted;
- `fb_age_ms[5]`: telemetry time minus that sample timestamp;
- `fb_pos[5]`, `fb_speed_raw[5]`, `fb_load_raw[5]`: unchanged raw feedback
  fields. Load is an effort proxy, not a calibrated force.

Reject or annotate samples with unexpectedly high age before using them for
stick-slip or tracking conclusions.

## Read-only factory snapshot

With the firmware running in the ESP-IDF monitor, and with no Motion Lab motion
active, enter:

```text
mlab params
```

The monitor prints `SCS009_PARAM` CSV records for IDs 1–5. `raw_hex` preserves
the bytes returned by the servo; no byte order or scale is applied. The command
does not unlock the EPROM area and sends no write instruction. Save the complete
monitor output as an immutable baseline together with the firmware commit and
date. Do not run it while holding the arm or while a motion experiment is active.

The snapshot covers the SCS-series identity, communication, limits, P/D/I,
minimum startup force, dead zones, and protection entries used for later
characterization. The addresses follow the SCS-series control table and the
FEETECH SCS protocol; SCS009 is a potentiometer SCS servo, so raw bytes are kept
until the model-specific table is verified.

References:

- [FEETECH communication protocol and memory-table downloads](https://www.feetechrc.com/en/letter-of-agreement.html)
- [FEETECH SCS009 product specification](https://www.feetechrc.com/en/6v-23kg-cm-dual-axis-serial-bus-steering-gear.html)
- [FEETECH protocol manual mirror](https://files.seeedstudio.com/wiki/robotics/Actuator/feetech/Communication_Protocol_Manual.pdf)

## Characterization matrix

Use minimum-jerk, one root/high-load joint at a time, fixed amplitude and limits.
Begin with the visible 10-degree step/hold/return test, then use smaller safe
amplitudes only after feedback age is verified. For each condition collect at
least three out-and-back repetitions:

| Factor | Initial levels | Hold constant |
| --- | --- | --- |
| trajectory duration | 1.5 s, 3 s, 6 s | amplitude, joint, posture |
| safe posture/load | unloaded reference, light supported load, normal working load | duration, amplitude |
| direction | positive and negative | all above |

Record posture/load setup, servo supply condition, commit, and the raw log file.
Do not approach mechanical limits or intentionally stall a joint. The first
question is whether visible stutter tracks load and very-low-speed portions of
the same minimum-jerk profile; only after this baseline may one internal
parameter (dead zone, P, D, or startup force) be changed and restored.
