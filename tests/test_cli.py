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
