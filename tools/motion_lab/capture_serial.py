#!/usr/bin/env python3
"""Capture a Motion Lab experiment without monitor/PTY backpressure."""

import argparse
import sys
import time

import serial


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", required=True)
    parser.add_argument("--command", required=True)
    parser.add_argument("--duration-ms", type=int, required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--ready-timeout-s", type=float, default=8.0)
    parser.add_argument("--tail-s", type=float, default=2.0)
    args = parser.parse_args()

    command = (args.command.rstrip("\r\n") + "\n").encode("ascii")
    started = time.monotonic()
    ready_deadline = started + args.ready_timeout_s
    finish_deadline = None
    ready_seen = False
    command_sent = False
    banner_window = bytearray()

    # Do not toggle flow-control lines: opening the port must not intentionally
    # reset a board that is already running the tested firmware.
    with serial.Serial(
        args.port,
        115200,
        timeout=0.1,
        dsrdtr=False,
        rtscts=False,
    ) as port, open(args.output, "wb") as output:
        print(f"capture: {args.port} -> {args.output}", flush=True)
        while True:
            chunk = port.read(4096)
            if chunk:
                output.write(chunk)
                output.flush()
                banner_window.extend(chunk)
                if len(banner_window) > 4096:
                    del banner_window[:-4096]
                if b"MLAB_CONSOLE ready" in banner_window:
                    ready_seen = True

            now = time.monotonic()
            if not command_sent and (ready_seen or now >= ready_deadline):
                port.write(command)
                port.flush()
                command_sent = True
                finish_deadline = now + args.duration_ms / 1000.0 + args.tail_s
                print(f"capture: sent {args.command}", flush=True)
            if command_sent and now >= finish_deadline:
                break

    if not command_sent:
        print("capture: command was not sent", file=sys.stderr)
        return 2
    print("capture: complete", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
