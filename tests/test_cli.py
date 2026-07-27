"""Unit tests for the CLI layer with a mocked daemon (no socket, no process)."""

from __future__ import annotations

import json

import pytest
from typer.testing import CliRunner

from mavctl.cli.app import app
from mavctl.daemon import process
from mavctl.daemon.client import DaemonNotRunningError
from mavctl.models import DaemonResponse, ExitCode

runner = CliRunner()

_CALL_DAEMON = "mavctl.cli.app.call_daemon"


def test_status_daemon_down_exits_3(monkeypatch: pytest.MonkeyPatch) -> None:
    def boom(*_args: object, **_kwargs: object) -> DaemonResponse:
        raise DaemonNotRunningError("no socket")

    monkeypatch.setattr(_CALL_DAEMON, boom)
    result = runner.invoke(app, ["status", "--json"])

    assert result.exit_code == ExitCode.DAEMON_NOT_RUNNING


def test_status_success_json(monkeypatch: pytest.MonkeyPatch) -> None:
    payload = {"connected": True, "flight_mode": "GUIDED", "armed": False}
    monkeypatch.setattr(_CALL_DAEMON, lambda *a, **k: DaemonResponse.success(payload))
    result = runner.invoke(app, ["status", "--json"])

    assert result.exit_code == ExitCode.SUCCESS
    assert json.loads(result.stdout)["flight_mode"] == "GUIDED"


def test_status_success_human(monkeypatch: pytest.MonkeyPatch) -> None:
    payload = {"connected": True, "flight_mode": "LOITER", "connection_string": "udp:x"}
    monkeypatch.setattr(_CALL_DAEMON, lambda *a, **k: DaemonResponse.success(payload))
    result = runner.invoke(app, ["status"])

    assert result.exit_code == ExitCode.SUCCESS
    assert "CONNECTED" in result.stdout
    assert "LOITER" in result.stdout


def test_telemetry_not_connected_exits_4(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setattr(
        _CALL_DAEMON,
        lambda *a, **k: DaemonResponse.failure(
            ExitCode.VEHICLE_NOT_CONNECTED, "vehicle not connected"
        ),
    )
    result = runner.invoke(app, ["telemetry", "--json"])

    assert result.exit_code == ExitCode.VEHICLE_NOT_CONNECTED


def test_daemon_start_already_running(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setattr(process, "is_running", lambda: True)
    monkeypatch.setattr(process, "read_pid", lambda: 4242)
    result = runner.invoke(app, ["daemon", "start", "--connect", "udp:x", "--json"])

    assert result.exit_code == ExitCode.SUCCESS
    assert json.loads(result.stdout) == {"status": "already_running", "pid": 4242}


def test_daemon_start_spawns(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setattr(process, "is_running", lambda: False)
    monkeypatch.setattr(process, "spawn", lambda connect, heartbeat_timeout: 999)
    result = runner.invoke(app, ["daemon", "start", "--connect", "udp:127.0.0.1:14550", "--json"])

    assert result.exit_code == ExitCode.SUCCESS
    body = json.loads(result.stdout)
    assert body["status"] == "started"
    assert body["pid"] == 999


def test_daemon_stop_when_down_exits_3(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setattr(process, "is_running", lambda: False)
    result = runner.invoke(app, ["daemon", "stop", "--json"])

    assert result.exit_code == ExitCode.DAEMON_NOT_RUNNING


def test_daemon_status_running(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setattr(process, "is_running", lambda: True)
    monkeypatch.setattr(process, "read_pid", lambda: 7)
    result = runner.invoke(app, ["daemon", "status", "--json"])

    assert result.exit_code == ExitCode.SUCCESS
    assert json.loads(result.stdout)["running"] is True


def test_missing_required_connect_is_usage_error() -> None:
    result = runner.invoke(app, ["daemon", "start"])
    assert result.exit_code == ExitCode.USAGE_ERROR


# -- flight-control commands ------------------------------------------------


def test_arm_passes_params_and_formats(monkeypatch: pytest.MonkeyPatch) -> None:
    captured: dict[str, object] = {}

    def fake_call(method: str, params=None, timeout: float = 5.0):  # type: ignore[no-untyped-def]
        captured["method"] = method
        captured["params"] = params
        return DaemonResponse.success(
            {"action": "arm", "executed": True, "outcome": {"result_name": "ACCEPTED"}}
        )

    monkeypatch.setattr(_CALL_DAEMON, fake_call)
    result = runner.invoke(app, ["arm", "--confirm"])

    assert result.exit_code == ExitCode.SUCCESS
    assert captured["method"] == "arm"
    assert captured["params"] == {"confirm": True, "force": False, "dry_run": False}
    assert "ACCEPTED" in result.stdout


def test_arm_guard_rejection_exit_5_with_hint(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setattr(
        _CALL_DAEMON,
        lambda *a, **k: DaemonResponse.failure(
            ExitCode.SAFETY_REJECTED,
            "arm requires explicit confirmation",
            {"reason": "confirmation_required", "hint": "re-run with --confirm"},
        ),
    )
    result = runner.invoke(app, ["arm"])
    assert result.exit_code == ExitCode.SAFETY_REJECTED


def test_takeoff_guard_rejection_maps_exit_code(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setattr(
        _CALL_DAEMON,
        lambda *a, **k: DaemonResponse.failure(
            ExitCode.SAFETY_REJECTED, "takeoff requires GUIDED", {"reason": "wrong_mode"}
        ),
    )
    result = runner.invoke(app, ["takeoff", "--alt", "10", "--confirm"])
    assert result.exit_code == ExitCode.SAFETY_REJECTED


def test_mode_dry_run_json(monkeypatch: pytest.MonkeyPatch) -> None:
    payload = {
        "dry_run": True,
        "action": "mode",
        "would_execute": True,
        "already_satisfied": False,
        "checks": [{"name": "confirm", "passed": True, "detail": "confirmed"}],
    }
    monkeypatch.setattr(_CALL_DAEMON, lambda *a, **k: DaemonResponse.success(payload))
    result = runner.invoke(app, ["mode", "LOITER", "--confirm", "--dry-run", "--json"])

    assert result.exit_code == ExitCode.SUCCESS
    assert json.loads(result.stdout)["would_execute"] is True


def test_takeoff_requires_alt_option() -> None:
    result = runner.invoke(app, ["takeoff", "--confirm"])
    assert result.exit_code == ExitCode.USAGE_ERROR


def test_nack_maps_exit_6(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setattr(
        _CALL_DAEMON,
        lambda *a, **k: DaemonResponse.failure(
            ExitCode.NACK_TIMEOUT, "arm not accepted by vehicle: FAILED", {}
        ),
    )
    result = runner.invoke(app, ["arm", "--confirm"])
    assert result.exit_code == ExitCode.NACK_TIMEOUT
