#!/usr/bin/env python3
"""Guard the ESP-IDF Nano printf-compatible telemetry format subset."""

from pathlib import Path
import re


ROOT = Path(__file__).resolve().parents[1]
CONSOLE_SOURCES = (
    ROOT / "main/boards/arm/arm_board.cc",
    ROOT / "main/boards/arm/xgo.cc",
    ROOT / "main/boards/arm/creature_stream.cc",
    ROOT / "main/boards/arm/motion_lab.cc",
)
UNSUPPORTED = re.compile(r"%ll|PRI(?:u|d|i|x|o)?64")


def main() -> None:
    violations = []
    for path in CONSOLE_SOURCES:
        text = path.read_text(encoding="utf-8")
        if UNSUPPORTED.search(text):
            violations.append(str(path.relative_to(ROOT)))
    if violations:
        raise SystemExit(
            "Nano printf-incompatible 64-bit formatter found in: "
            + ", ".join(violations)
        )

    arm_board = (ROOT / "main/boards/arm/arm_board.cc").read_text(encoding="utf-8")
    xgo = (ROOT / "main/boards/arm/xgo.cc").read_text(encoding="utf-8")
    assert "last_target_ms=%lu" in arm_board
    assert "%02X,%lu,%lu,%d,%d,%d,%d" in xgo
    print("Nano printf format guard passed")


if __name__ == "__main__":
    main()
