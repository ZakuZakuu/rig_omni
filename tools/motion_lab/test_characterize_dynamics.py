#!/usr/bin/env python3
"""Small dependency-free regression checks for characterize_dynamics.py."""

from __future__ import annotations

import math
import json
import tempfile
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parent))
import characterize_dynamics as dynamics  # noqa: E402
from characterize_dynamics import (  # noqa: E402
    DeploymentGateAbort,
    SafetyGateAbort,
    _build_plan,
    _build_parser,
    _command_for,
    _analyze_manifest,
    _conditioning_entry,
    _conditioning_health_reasons,
    _conditioning_metrics,
    _execution_entries,
    _initial_manifest,
    _local_polynomial_velocity,
    _preflight_decision,
    _reversal_proxy,
    _run_hardware,
    _safety_reasons,
    _write_json,
    analyze_run,
)
from analyze_stutter import parse_capture  # noqa: E402


def _row(index: int, command: float, feedback: float) -> dict[str, float]:
    timestamp = float((index + 1) * 50)
    return {
        "ts_ms": timestamp,
        "elapsed_ms": timestamp,
        "cmd_deg": command / (1024.0 / 300.0),
        "cmd_pos": command,
        "fb_pos": feedback,
        "fb_speed_raw": 10.0,
        "fb_load_raw": 100.0,
        "fb_age_ms": 20.0,
        "fb_ts_ms": timestamp,
        "stale": 0.0,
        "voltage_v": 8.0,
        "cmd_velocity_deg_s": 5.0,
        "fb_delta_counts": 1.0,
        "dt_ms": 50.0,
    }


def _status_capture() -> str:
    return "MLAB_SERVO_STATUS,id,error_hex,last_error_hex,last_error_ts_ms,error_count,fb_pos,fb_speed_raw,fb_load_raw,stale\n" + "\n".join(
        f"MLAB_SERVO_STATUS,{servo_id},00,00,0,0,512,0,100,0" for servo_id in range(1, 6)
    ) + "\n"


def _params_capture() -> str:
    return "SCS009_PARAM dump: read-only; no EEPROM unlock/write performed\nSCS009_PARAM,1,p_coefficient,0x15,1,18,100\nSCS009_PARAM dump: complete\n"


def _voltage_capture(voltage: float = 8.0) -> str:
    return f"MLAB_VOLTAGE,ts_ms=100,id=1,voltage_v={voltage:.2f},request_sent=1\n"


def _caps_capture(
    *,
    experiments: str = "0|1|2|3|4|5",
    protocol: int = 2,
    reversal: int = 1,
) -> str:
    return (
        f"MLAB_CAPS,protocol={protocol},experiments={experiments},reversal={reversal},"
        "project=rig-arm,version=test,build_date=2026-09-16,build_time=12:00:00,"
        "elf_sha256=device-test-sha\n"
    )


def _mlab_capture(feedback: float) -> str:
    header = "MLAB,ts_ms,experiment,trajectory,elapsed_ms,total_ms,cmd_deg[5],cmd_pos[5],fb_pos[5],fb_speed_raw[5],fb_load_raw[5],fb_ts_ms[5],fb_age_ms[5],fb_stale[5],servo_voltage_v,speed_cmd_raw\n"
    rows = []
    for timestamp, elapsed, command, position in ((100, 0, 0, 0), (150, 50, 1, feedback)):
        command_array = ["0", "0", str(command), "0", "0"]
        position_array = ["0", "0", str(position), "0", "0"]
        arrays = [
            "|".join(command_array),
            "|".join(command_array),
            "|".join(position_array),
            "0|0|0|0|0",
            "100|100|100|100|100",
            "|".join([str(timestamp)] * 5),
            "0|0|0|0|0",
            "0|0|0|0|0",
            "8.0",
            "1|0|0|0|0",
        ]
        rows.append(f"MLAB,{timestamp},4,2,{elapsed},600," + ",".join(arrays))
    return header + "\n".join(rows) + "\n"


def _conditioning_capture() -> str:
    header = "MLAB,ts_ms,experiment,trajectory,elapsed_ms,total_ms,cmd_deg[5],cmd_pos[5],fb_pos[5],fb_speed_raw[5],fb_load_raw[5],fb_ts_ms[5],fb_age_ms[5],fb_stale[5],servo_voltage_v,speed_cmd_raw\n"
    rows = []
    for timestamp, elapsed, command, feedback in (
        (100, 0, 512, 512),
        (2600, 2500, 529, 522),
        (6600, 6500, 495, 502),
        (9100, 9000, 512, 512),
    ):
        command_array = ["0", "0", str(command), "0", "0"]
        position_array = ["512", "512", str(feedback), "512", "512"]
        arrays = [
            "|".join(command_array),
            "|".join(command_array),
            "|".join(position_array),
            "0|0|0|0|0",
            "100|100|100|100|100",
            "|".join([str(timestamp)] * 5),
            "0|0|0|0|0",
            "0|0|0|0|0",
            "8.0",
            "350",
        ]
        rows.append(f"MLAB,{timestamp},5,2,{elapsed},9500," + ",".join(arrays))
    return header + "\n".join(rows) + "\n"


def _hardware_args(output_dir: Path, max_runs: int = 1):
    return _build_parser().parse_args([
        "--execute", "--confirm-hardware", "--port", "/dev/test",
        "--output-dir", str(output_dir), "--joint", "2", "--max-runs", str(max_runs),
        "--repetitions", "1",
    ])


def _run_with_fake_capture(
    args,
    manifest,
    groups,
    *,
    status_text: str,
    movement_feedback: float | None,
    caps_text: str | None = None,
    invalid_conditioning: bool = False,
):
    calls: list[str] = []
    original = dynamics._capture_command

    def fake_capture(_port, command, output, _duration_ms, **_kwargs):
        calls.append(command)
        if command == "mlab caps":
            output.write_text(caps_text or _caps_capture(), encoding="utf-8")
        elif command == "mlab status":
            output.write_text(status_text, encoding="utf-8")
        elif command == "mlab params":
            output.write_text(_params_capture(), encoding="utf-8")
        elif command == "mlab voltage":
            output.write_text(_voltage_capture(), encoding="utf-8")
        elif command.startswith("mlab run"):
            if command.startswith("mlab run 5"):
                output.write_text(
                    "MLAB_CONSOLE run: invalid config\r\n"
                    if invalid_conditioning
                    else _conditioning_capture(),
                    encoding="utf-8",
                )
            else:
                output.write_text(_mlab_capture(movement_feedback or 0.0), encoding="utf-8")
        else:
            output.write_text("ack\n", encoding="utf-8")

    dynamics._capture_command = fake_capture
    try:
        error = None
        try:
            dynamics._run_hardware(args, args.output_dir, manifest, groups)
        except SafetyGateAbort as caught:
            error = caught
        return calls, error
    finally:
        dynamics._capture_command = original


def main() -> int:
    capture = Path(__file__).with_name(".synthetic-mlab-capture.log")
    rows_for_capture = [
        "MLAB,100,4,2,0,600,0|0|0|0|0,0|0|0|0|0,0|0|0|0|0,0|0|0|0|0,0|0|0|0|0,100|100|100|100|100,0|0|0|0|0,0|0|0|0|0,8.0,0|0|0|0|0",
        "MLAB,150,4,2,50,600,1|0|0|0|0,3|0|0|0|0,2|0|0|0|0,4|0|0|0|0,101|100|100|100|100,150|100|100|100|100,0|0|0|0|0,0|0|0|0|0,8.0,1|0|0|0|0",
    ]
    capture.write_text(
        "MLAB,ts_ms,experiment,trajectory,elapsed_ms,total_ms,cmd_deg[5],cmd_pos[5],fb_pos[5],fb_speed_raw[5],fb_load_raw[5],fb_ts_ms[5],fb_age_ms[5],fb_stale[5],servo_voltage_v,speed_cmd_raw\n"
        + "\n".join(rows_for_capture)
        + "\n",
        encoding="utf-8",
    )
    try:
        parsed = parse_capture(capture, joint=0)
        assert parsed[1]["fb_ts_ms"] == 150.0
        assert parsed[1]["speed_cmd_raw"] == 1.0

        # Current firmware emits speed_cmd_raw as one scalar for the shared
        # sync-write. The parser must expose that same value for whichever
        # selected joint is being analyzed.
        scalar_capture = capture.with_name(".synthetic-mlab-scalar-capture.log")
        scalar_capture.write_text(
            capture.read_text(encoding="utf-8").replace(
                ",1|0|0|0|0\n", ",350\n"
            ),
            encoding="utf-8",
        )
        try:
            scalar_rows = parse_capture(scalar_capture, joint=2)
            assert scalar_rows[1]["speed_cmd_raw"] == 350.0
        finally:
            scalar_capture.unlink()

        caps_capture = capture.with_name(".synthetic-mlab-caps.log")
        caps_capture.write_text(_caps_capture(), encoding="utf-8")
        try:
            capabilities = dynamics._parse_caps_capture(caps_capture, required_experiment=5)
            assert capabilities["passed"] is True
            assert capabilities["protocol"] == 2
            assert capabilities["experiments"] == [0, 1, 2, 3, 4, 5]
            assert capabilities["device_identity"]["elf_sha256"] == "device-test-sha"
        finally:
            caps_capture.unlink()

        malformed_caps = capture.with_name(".synthetic-mlab-malformed-caps.log")
        malformed_caps.write_text(
            "MLAB_CAPS,protocol=1,experiments=0|1|2|3|4,reversal=0\n",
            encoding="utf-8",
        )
        try:
            rejected = dynamics._parse_caps_capture(malformed_caps, required_experiment=5)
            assert rejected["passed"] is False
            assert any("protocol" in reason for reason in rejected["reasons"])
            assert any("experiment 5" in reason for reason in rejected["reasons"])
        finally:
            malformed_caps.unlink()
    finally:
        capture.unlink()

    plan_args = _build_parser().parse_args(["--joint", "2", "--repetitions", "1"])
    plan_groups, planned = _build_plan(plan_args)
    assert _command_for(planned[0]) == "mlab run 4 2 2 3 3000 0 8 30 250"
    assert planned[0]["tier"] == "gentle"
    assert planned[0]["amplitude_deg"] == 3
    assert len(_execution_entries(plan_groups, 1)) == 1
    assert len(_execution_entries(plan_groups, 2)) == 2

    conditioned_args = _build_parser().parse_args(["--joint", "2", "--precondition", "positive"])
    positive_conditioning = _conditioning_entry(conditioned_args, "positive")
    negative_conditioning = _conditioning_entry(conditioned_args, "negative")
    assert positive_conditioning["description"] == "center -> +5 -> -5 -> center"
    assert negative_conditioning["description"] == "center -> -5 -> +5 -> center"
    assert positive_conditioning["duration_ms"] == 9500
    assert positive_conditioning["offline_profile"]["feasible"] is True
    assert _command_for(positive_conditioning) == "mlab run 5 2 2 5 2500 0 8 30 250"
    assert _command_for(negative_conditioning) == "mlab run 5 2 2 -5 2500 0 8 30 250"
    conditioning_rows = [_row(index, command, feedback) for index, (command, feedback) in enumerate(
        ((0, 100), (17, 120), (-17, 80), (0, 100))
    )]
    conditioning_metadata = {**positive_conditioning, "run": "conditioning"}
    conditioning_summary = _conditioning_metrics(
        conditioning_rows,
        conditioning_metadata,
        center_count=100.0,
        settled_feedback_count=100.0,
        settle_status={"parse_ok": True, "errors": []},
    )
    assert conditioning_summary["conditioning_achieved_positive_counts"] == 20.0
    assert conditioning_summary["conditioning_achieved_negative_counts"] == 20.0
    assert conditioning_summary["conditioning_commanded_positive_counts"] == 17.0
    assert conditioning_summary["conditioning_commanded_negative_counts"] == 17.0
    assert conditioning_summary["conditioning_commanded_positive_deg"] > 0.0
    assert conditioning_summary["conditioning_commanded_negative_deg"] > 0.0
    assert conditioning_summary["conditioning_commanded_endpoint_within_tolerance"]
    assert conditioning_summary["conditioning_requested_positive_counts"] > 17.0
    assert not _conditioning_health_reasons(conditioning_summary, conditioned_args)
    distorted_rows = [_row(index, command, feedback) for index, (command, feedback) in enumerate(
        ((0, 100), (19, 108), (-21, 77), (0, 100))
    )]
    distorted_summary = _conditioning_metrics(
        distorted_rows,
        conditioning_metadata,
        center_count=100.0,
        settled_feedback_count=100.0,
        settle_status={"parse_ok": True, "errors": []},
    )
    distorted_reasons = _conditioning_health_reasons(distorted_summary, conditioned_args)
    assert not distorted_summary["conditioning_commanded_endpoint_within_tolerance"]
    assert any("experimental-design distortion" in reason for reason in distorted_reasons)
    failed_conditioning = dict(conditioning_summary)
    failed_conditioning["conditioning_achieved_positive_counts"] = 0.0
    assert _conditioning_health_reasons(failed_conditioning, conditioned_args)

    with tempfile.TemporaryDirectory(prefix="rig-dynamics-separate-metrics-") as directory:
        output_dir = Path(directory)
        formal_log = output_dir / "formal.log"
        formal_log.write_text(_mlab_capture(2.0), encoding="utf-8")
        formal_entry = {
            **planned[0],
            "run": "formal",
            "log": formal_log.name,
            "direction": "positive",
            "command": _command_for(planned[0]),
        }
        report_manifest = _initial_manifest(conditioned_args, [formal_entry])
        report_manifest["runs"] = [formal_entry]
        report_manifest["conditioning"] = {
            "enabled": True,
            "direction": "positive",
            "conditioning_passed": True,
            "formal_run_started": True,
            "metrics": conditioning_summary,
        }
        manifest_path = output_dir / "manifest.json"
        _write_json(manifest_path, report_manifest)
        report = _analyze_manifest(output_dir, manifest_path, max_load_raw=3000.0)
        assert report["conditioning"]["enabled"] is True
        assert report["conditioning"]["formal_run_started"] is True
        assert [run["run"] for run in report["runs"]] == ["formal"]
        assert "conditioning_achieved_positive_counts" not in report["runs"][0]

    with tempfile.TemporaryDirectory(prefix="rig-dynamics-readiness-") as directory:
        output_dir = Path(directory)
        status_path = output_dir / "status.log"
        params_path = output_dir / "params.log"
        voltage_path = output_dir / "voltage.log"
        status_path.write_text(_status_capture(), encoding="utf-8")
        params_path.write_text(_params_capture(), encoding="utf-8")
        voltage_path.write_text(_voltage_capture(), encoding="utf-8")
        decision = _preflight_decision(status_path, params_path, voltage_path)
        assert decision["passed"]
        voltage_path.write_text(_voltage_capture(7.0), encoding="utf-8")
        voltage_decision = _preflight_decision(status_path, params_path, voltage_path)
        assert not voltage_decision["passed"]
        assert any("baseline" in reason for reason in voltage_decision["reasons"])
        status_path.write_text(_status_capture().replace(",00,00,0,0,512", ",01,00,0,0,512", 1), encoding="utf-8")
        status_decision = _preflight_decision(status_path, params_path, voltage_path)
        assert not status_decision["passed"]
        assert any("status" in reason for reason in status_decision["reasons"])

    # The first 3-degree run must use a meaningful small-motion tracking guard,
    # rather than the old 12-degree floor.
    first_run_rows = [_row(index, command, feedback) for index, (command, feedback) in enumerate(((0, 0), (4, 50)))]
    first_run_metadata = {
        "run": "first", "tier": "gentle", "experiment": 4, "joint": 2,
        "direction": "positive", "amplitude_deg": 3, "transition_ms": 3000,
        "duration_ms": 7000, "hold_ms": 1000, "repetition": 1,
    }
    first_metric, _ = analyze_run(first_run_rows, first_run_metadata)
    first_reasons = _safety_reasons(first_metric, 3000.0)
    assert first_metric["tracking_error_guard_deg"] < 12.0
    assert any("small-motion sanity guard" in reason for reason in first_reasons)
    assert first_metric["command_excursion_counts"] == 4.0
    assert first_metric["achieved_excursion_counts"] == 50.0
    assert first_metric["quantization_sensitive"] is True
    assert first_metric["observed_peak_velocity_calibrated"] is False
    assert "threshold" in first_metric["motion_onset_latency_note"]

    with tempfile.TemporaryDirectory(prefix="rig-dynamics-preflight-") as directory:
        output_dir = Path(directory)
        args = _hardware_args(output_dir)
        groups, planned = _build_plan(args)
        manifest = _initial_manifest(args, planned)
        calls, error = _run_with_fake_capture(
            args, manifest, groups, status_text="garbled status\n", movement_feedback=None
        )
        assert isinstance(error, SafetyGateAbort)
        assert not any(command.startswith("mlab run") for command in calls)
        assert "mlab stop" in calls and "mlab poll off" in calls and "mlab comp off" in calls
        persisted = json.loads((output_dir / "manifest.json").read_text(encoding="utf-8"))
        assert not persisted["preflight"]["decision"]["passed"]

    # A stale image that does not advertise the required reversal experiment
    # must fail before status/preflight or any physical movement command.
    with tempfile.TemporaryDirectory(prefix="rig-dynamics-capability-mismatch-") as directory:
        output_dir = Path(directory)
        args = _build_parser().parse_args([
            "--execute", "--confirm-hardware", "--port", "/dev/test",
            "--output-dir", str(output_dir), "--joint", "2", "--max-runs", "1",
            "--repetitions", "1", "--precondition", "positive",
        ])
        groups, planned = _build_plan(args)
        manifest = _initial_manifest(args, planned)
        calls, error = _run_with_fake_capture(
            args,
            manifest,
            groups,
            status_text=_status_capture(),
            movement_feedback=None,
            caps_text="MLAB_CAPS,protocol=2,experiments=0|1|2|3|4,reversal=0\n",
        )
        assert isinstance(error, DeploymentGateAbort)
        assert calls[0] == "mlab caps"
        assert not any(command.startswith("mlab status") for command in calls)
        assert not any(command.startswith("mlab run") for command in calls)
        persisted = json.loads((output_dir / "manifest.json").read_text(encoding="utf-8"))
        assert persisted["status"] == "deployment_failed"
        assert persisted["deployment"]["run_rejection"] if "run_rejection" in persisted["deployment"] else True

    # Device identity is a separate evidence source from the host checkout
    # recorded in the manifest's firmware section.
    with tempfile.TemporaryDirectory(prefix="rig-dynamics-identity-") as directory:
        output_dir = Path(directory)
        caps_path = output_dir / "caps.log"
        caps_path.write_text(_caps_capture(), encoding="utf-8")
        manifest = _initial_manifest(_hardware_args(output_dir), [])
        manifest["firmware"]["commit"] = "host-commit"
        manifest["deployment"]["device_identity"] = dynamics._parse_caps_capture(caps_path)["device_identity"]
        manifest_path = output_dir / "manifest.json"
        _write_json(manifest_path, manifest)
        report = _analyze_manifest(output_dir, manifest_path, max_load_raw=3000.0)
        assert report["firmware"]["commit"] == "host-commit"
        assert report["deployment"]["device_identity"]["elf_sha256"] == "device-test-sha"

    with tempfile.TemporaryDirectory(prefix="rig-dynamics-conditioning-") as directory:
        output_dir = Path(directory)
        args = _build_parser().parse_args([
            "--execute", "--confirm-hardware", "--port", "/dev/test",
            "--output-dir", str(output_dir), "--joint", "2", "--max-runs", "1",
            "--repetitions", "1", "--precondition", "positive",
        ])
        groups, planned = _build_plan(args)
        manifest = _initial_manifest(args, planned)
        original_conditioning = dynamics._run_conditioning

        def failed_conditioning(_args, _output_dir, _manifest):
            return {
                "enabled": True,
                "direction": "positive",
                "conditioning_passed": False,
                "metrics": {"conditioning_health_reasons": ["synthetic health failure"]},
            }

        dynamics._run_conditioning = failed_conditioning
        try:
            calls, error = _run_with_fake_capture(
                args, manifest, groups, status_text=_status_capture(), movement_feedback=None
            )
        finally:
            dynamics._run_conditioning = original_conditioning
        assert isinstance(error, SafetyGateAbort)
        assert not any(command.startswith("mlab run 4") for command in calls)
        assert "mlab stop" in calls and "mlab poll off" in calls and "mlab comp off" in calls
        persisted = json.loads((output_dir / "manifest.json").read_text(encoding="utf-8"))
        assert persisted["status"] == "conditioning_failed"
        assert persisted["conditioning"]["formal_run_started"] is False

    # An on-device invalid-config response with no MLAB rows is a deployment
    # readiness failure, never an actuator/conditioning result.
    with tempfile.TemporaryDirectory(prefix="rig-dynamics-invalid-config-") as directory:
        output_dir = Path(directory)
        args = _build_parser().parse_args([
            "--execute", "--confirm-hardware", "--port", "/dev/test",
            "--output-dir", str(output_dir), "--joint", "2", "--max-runs", "1",
            "--repetitions", "1", "--precondition", "positive",
        ])
        groups, planned = _build_plan(args)
        manifest = _initial_manifest(args, planned)
        original_sleep = dynamics.time.sleep
        dynamics.time.sleep = lambda _seconds: None
        try:
            calls, error = _run_with_fake_capture(
                args,
                manifest,
                groups,
                status_text=_status_capture(),
                movement_feedback=None,
                invalid_conditioning=True,
            )
        finally:
            dynamics.time.sleep = original_sleep
        assert isinstance(error, DeploymentGateAbort)
        assert any(command.startswith("mlab run 5") for command in calls)
        assert not any(command.startswith("mlab run 4") for command in calls)
        persisted = json.loads((output_dir / "manifest.json").read_text(encoding="utf-8"))
        assert persisted["status"] == "deployment_failed"
        assert persisted["deployment"]["run_rejection"]["physical_motion_started"] is False
        assert persisted["conditioning"]["formal_run_started"] is False

    with tempfile.TemporaryDirectory(prefix="rig-dynamics-conditioning-only-") as directory:
        output_dir = Path(directory)
        args = _build_parser().parse_args([
            "--execute", "--confirm-hardware", "--port", "/dev/test",
            "--output-dir", str(output_dir), "--joint", "2", "--max-runs", "7",
            "--repetitions", "1", "--precondition", "positive", "--conditioning-only",
        ])
        groups, planned = _build_plan(args)
        manifest = _initial_manifest(args, planned)
        original_sleep = dynamics.time.sleep
        dynamics.time.sleep = lambda _seconds: None
        try:
            calls, error = _run_with_fake_capture(
                args, manifest, groups, status_text=_status_capture(), movement_feedback=100.0
            )
        finally:
            dynamics.time.sleep = original_sleep
        assert error is None
        assert "mlab run 5 2 2 5 2500 0 8 30 250" in calls
        assert not any(command.startswith("mlab run 4") for command in calls)
        persisted = json.loads((output_dir / "manifest.json").read_text(encoding="utf-8"))
        assert persisted["status"] == "conditioning_complete"
        assert persisted["conditioning"]["formal_run_started"] is False
        assert persisted["conditioning"]["conditioning_passed"] is True
        assert persisted["execution"]["conditioning_only"] is True
        assert "mlab stop" in calls and "mlab poll off" in calls and "mlab comp off" in calls

    # The ordinary --precondition path still starts its formal run after a
    # passing prelude; conditioning-only is an explicit opt-out from that step.
    with tempfile.TemporaryDirectory(prefix="rig-dynamics-conditioning-normal-") as directory:
        output_dir = Path(directory)
        args = _build_parser().parse_args([
            "--execute", "--confirm-hardware", "--port", "/dev/test",
            "--output-dir", str(output_dir), "--joint", "2", "--max-runs", "1",
            "--repetitions", "1", "--precondition", "positive",
        ])
        groups, planned = _build_plan(args)
        manifest = _initial_manifest(args, planned)
        original_sleep = dynamics.time.sleep
        dynamics.time.sleep = lambda _seconds: None
        try:
            calls, error = _run_with_fake_capture(
                args, manifest, groups, status_text=_status_capture(), movement_feedback=100.0
            )
        finally:
            dynamics.time.sleep = original_sleep
        assert isinstance(error, SafetyGateAbort)
        assert any(command.startswith("mlab run 5") for command in calls)
        assert any(command.startswith("mlab run 4") for command in calls)
        assert "mlab stop" in calls and "mlab poll off" in calls and "mlab comp off" in calls

    # The confirmation flag remains an independent hard gate: a caller cannot
    # reach the serial capture path merely by supplying --execute and a port.
    with tempfile.TemporaryDirectory(prefix="rig-dynamics-no-confirm-") as directory:
        output_dir = Path(directory)
        args = _build_parser().parse_args([
            "--execute", "--port", "/dev/test", "--output-dir", str(output_dir),
        ])
        groups, planned = _build_plan(args)
        try:
            _run_hardware(args, output_dir, _initial_manifest(args, planned), groups)
        except ValueError as error:
            assert "confirm-hardware" in str(error)
        else:  # pragma: no cover - protects the safety contract
            raise AssertionError("hardware execution must require --confirm-hardware")

    with tempfile.TemporaryDirectory(prefix="rig-dynamics-run-abort-") as directory:
        output_dir = Path(directory)
        args = _hardware_args(output_dir)
        groups, planned = _build_plan(args)
        manifest = _initial_manifest(args, planned)
        calls, error = _run_with_fake_capture(
            args, manifest, groups, status_text=_status_capture(), movement_feedback=100.0
        )
        assert isinstance(error, SafetyGateAbort)
        movement_calls = [command for command in calls if command.startswith("mlab run")]
        assert movement_calls == ["mlab run 4 2 2 3 3000 0 8 30 250"]
        assert "mlab stop" in calls and "mlab poll off" in calls and "mlab comp off" in calls

    rows = [_row(index, command, feedback) for index, (command, feedback) in enumerate(
        ((0, 0), (4, 2), (8, 6), (12, 10), (16, 15), (16, 16), (16, 16), (16, 16), (0, 1), (0, 0))
    )]
    metadata = {
        "run": "synthetic",
        "tier": "gentle",
        "experiment": 4,
        "joint": 2,
        "direction": "positive",
        "amplitude_deg": 3,
        "transition_ms": 200,
        "duration_ms": 600,
        "hold_ms": 100,
        "repetition": 1,
    }
    metric, _events = analyze_run(rows, metadata)
    assert metric["feedback_valid_rate"] == 1.0
    assert metric["stale_fraction"] == 0.0
    assert metric["motion_onset_latency_ms"] is not None
    assert metric["observed_peak_velocity_deg_s"] is not None
    assert all(math.isfinite(value) for value in _local_polynomial_velocity([(0.0, 0.0), (20.0, 1.0), (40.0, 2.0)]))

    reversal_rows = [_row(index, command, feedback) for index, (command, feedback) in enumerate(
        ((0, 0), (10, 5), (20, 12), (10, 12), (0, 9), (-10, 5), (-20, -1), (-10, -2), (0, 0))
    )]
    for index, row in enumerate(reversal_rows):
        row["cmd_velocity_deg_s"] = (reversal_rows[index]["cmd_pos"] - reversal_rows[index - 1]["cmd_pos"]) if index else 0.0
    assert _reversal_proxy(reversal_rows)
    print("characterize_dynamics checks passed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
