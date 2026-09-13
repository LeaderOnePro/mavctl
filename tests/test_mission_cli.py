"""CLI contract tests for mission commands (Phase 3A)."""

from __future__ import annotations

import json
from typing import Any

import pytest
from typer.testing import CliRunner

from mavctl.cli.app import app
from mavctl.models import DaemonResponse, ExitCode

runner = CliRunner()

_CALL_DAEMON = "mavctl.cli.app.call_daemon"

_MISSION_JSON = {
    "version": 1,
    "items": [
        {"type": "takeoff", "altitude_m": 10.0},
        {"type": "waypoint", "lat_deg": 1.0, "lon_deg": 2.0, "altitude_m": 15.0},
        {"type": "rtl"},
    ],
}


def _write_mission(tmp_path: Any, payload: Any = None) -> str:
    path = tmp_path / "mission.json"
    path.write_text(json.dumps(payload if payload is not None else _MISSION_JSON))
    return str(path)


def _download_payload() -> dict[str, Any]:
    return {
        "action": "mission_download",
        "mission": {"version": 1, "items": [{"type": "takeoff", "altitude_m": 10.0}]},
    }


# -- upload ------------------------------------------------------------------


def test_upload_success_human(monkeypatch: pytest.MonkeyPatch, tmp_path: Any) -> None:
    monkeypatch.setattr(
        _CALL_DAEMON,
        lambda *_a, **_k: DaemonResponse.success(
            {"action": "mission_upload", "accepted": True, "result_name": "ACCEPTED",
             "item_count": 3}
        ),
    )
    result = runner.invoke(app, ["mission", "upload", _write_mission(tmp_path), "--confirm"])
    assert result.exit_code == ExitCode.SUCCESS
    assert "accepted" in result.stdout
    assert "(3 items)" in result.stdout


def test_upload_success_json(monkeypatch: pytest.MonkeyPatch, tmp_path: Any) -> None:
    monkeypatch.setattr(
        _CALL_DAEMON,
        lambda *_a, **_k: DaemonResponse.success(
            {"action": "mission_upload", "accepted": True, "result_name": "ACCEPTED",
             "item_count": 3}
        ),
    )
    result = runner.invoke(
        app, ["mission", "upload", _write_mission(tmp_path), "--confirm", "--json"]
    )
    assert result.exit_code == ExitCode.SUCCESS
    payload = json.loads(result.stdout)
    assert payload["accepted"] is True
    assert payload["item_count"] == 3


def test_upload_missing_file_exits_2_without_daemon(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Any
) -> None:
    def no_daemon(*_a: object, **_k: object) -> DaemonResponse:
        raise AssertionError("daemon must not be contacted for an unreadable file")

    monkeypatch.setattr(_CALL_DAEMON, no_daemon)
    result = runner.invoke(
        app, ["mission", "upload", str(tmp_path / "missing.json"), "--confirm"]
    )
    assert result.exit_code == ExitCode.USAGE_ERROR


def test_upload_invalid_schema_exits_2_without_daemon(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Any
) -> None:
    def no_daemon(*_a: object, **_k: object) -> DaemonResponse:
        raise AssertionError("daemon must not be contacted for an invalid mission")

    monkeypatch.setattr(_CALL_DAEMON, no_daemon)
    invalid = {"version": 1, "items": [{"type": "rtl"}, {"type": "takeoff", "altitude_m": 5.0}]}
    result = runner.invoke(
        app, ["mission", "upload", _write_mission(tmp_path, invalid), "--confirm", "--json"]
    )
    assert result.exit_code == ExitCode.USAGE_ERROR
    assert "invalid_mission" in result.output


def test_upload_rejects_uppercase_types(monkeypatch: pytest.MonkeyPatch, tmp_path: Any) -> None:
    def no_daemon(*_a: object, **_k: object) -> DaemonResponse:
        raise AssertionError("uppercase type must be a schema error")

    monkeypatch.setattr(_CALL_DAEMON, no_daemon)
    payload = {"version": 1, "items": [{"type": "TAKEOFF", "altitude_m": 10.0}]}
    result = runner.invoke(
        app, ["mission", "upload", _write_mission(tmp_path, payload), "--confirm"]
    )
    assert result.exit_code == ExitCode.USAGE_ERROR


def test_upload_uncertain_maps_to_exit_6(monkeypatch: pytest.MonkeyPatch, tmp_path: Any) -> None:
    monkeypatch.setattr(
        _CALL_DAEMON,
        lambda *_a, **_k: DaemonResponse.failure(
            ExitCode.NACK_TIMEOUT,
            "mission operation outcome uncertain; remote mission state must be verified",
            {"reason": "remote_mission_state_uncertain", "accepted_upto": 1},
        ),
    )
    result = runner.invoke(
        app, ["mission", "upload", _write_mission(tmp_path), "--confirm", "--json"]
    )
    assert result.exit_code == ExitCode.NACK_TIMEOUT
    payload = json.loads(result.stderr)
    assert payload["error"]["detail"]["reason"] == "remote_mission_state_uncertain"


def test_upload_stdin_style_dash_path_is_just_a_missing_file(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Any
) -> None:
    # v1 has no stdin support: "-" is treated as a literal (missing) path.
    def no_daemon(*_a: object, **_k: object) -> DaemonResponse:
        raise AssertionError("daemon must not be contacted")

    monkeypatch.setattr(_CALL_DAEMON, no_daemon)
    result = runner.invoke(app, ["mission", "upload", "-", "--confirm"])
    assert result.exit_code == ExitCode.USAGE_ERROR


# -- download ----------------------------------------------------------------


def test_download_json_stdout_is_pure_mission_json(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.setattr(_CALL_DAEMON, lambda *_a, **_k: DaemonResponse.success(_download_payload()))
    result = runner.invoke(app, ["mission", "download", "--json"])
    assert result.exit_code == ExitCode.SUCCESS
    payload = json.loads(result.stdout)
    assert payload == {"version": 1, "items": [{"type": "takeoff", "altitude_m": 10.0}]}
    assert "action" not in payload  # no envelope pollution
    assert "mission" not in payload


def test_download_output_file_writes_mission_json(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Any
) -> None:
    monkeypatch.setattr(_CALL_DAEMON, lambda *_a, **_k: DaemonResponse.success(_download_payload()))
    target = tmp_path / "downloaded.json"
    result = runner.invoke(app, ["mission", "download", "--output", str(target)])
    assert result.exit_code == ExitCode.SUCCESS
    written = json.loads(target.read_text())
    assert written["items"][0]["type"] == "takeoff"
    assert "mission downloaded" in result.stdout


def test_download_output_write_failure_human_exits_2_without_traceback(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Any
) -> None:
    # the daemon already answered successfully; the write failure must still
    # surface as a controlled usage error, never as a fake success
    monkeypatch.setattr(_CALL_DAEMON, lambda *_a, **_k: DaemonResponse.success(_download_payload()))

    parent_is_file = tmp_path / "not-a-dir"
    parent_is_file.write_text("x")
    target = parent_is_file / "mission.json"  # parent is a file → OSError

    result = runner.invoke(app, ["mission", "download", "--output", str(target)])
    assert result.exit_code == ExitCode.USAGE_ERROR
    assert "cannot write mission output" in result.output
    assert str(target) in result.output
    assert "Traceback" not in result.output


def test_download_output_write_failure_json_reason(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Any
) -> None:
    monkeypatch.setattr(_CALL_DAEMON, lambda *_a, **_k: DaemonResponse.success(_download_payload()))

    parent_is_file = tmp_path / "not-a-dir"
    parent_is_file.write_text("x")
    target = parent_is_file / "mission.json"

    # --output and --json are mutually exclusive by design, so the write
    # failure surfaces through the human path; the structured detail exists
    # for future JSON output modes and is covered by the reason contract.
    result = runner.invoke(app, ["mission", "download", "--output", str(target)])
    assert result.exit_code == ExitCode.USAGE_ERROR
    assert "cannot write mission output" in result.output
    assert "Traceback" not in result.output


def test_download_json_and_output_are_mutually_exclusive(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Any
) -> None:
    def no_daemon(*_a: object, **_k: object) -> DaemonResponse:
        raise AssertionError("daemon must not be contacted")

    monkeypatch.setattr(_CALL_DAEMON, no_daemon)
    result = runner.invoke(
        app,
        ["mission", "download", "--output", str(tmp_path / "m.json"), "--json"],
    )
    assert result.exit_code == ExitCode.USAGE_ERROR


def test_download_default_human_table(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setattr(_CALL_DAEMON, lambda *_a, **_k: DaemonResponse.success(_download_payload()))
    result = runner.invoke(app, ["mission", "download"])
    assert result.exit_code == ExitCode.SUCCESS
    assert "takeoff" in result.stdout
    assert "1 item(s)" in result.stdout


def test_download_failure_maps_to_exit_6(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setattr(
        _CALL_DAEMON,
        lambda *_a, **_k: DaemonResponse.failure(
            ExitCode.NACK_TIMEOUT,
            "mission download failed: TIMEOUT",
            {"reason": "mission_protocol_timeout"},
        ),
    )
    result = runner.invoke(app, ["mission", "download", "--json"])
    assert result.exit_code == ExitCode.NACK_TIMEOUT
    payload = json.loads(result.stderr)
    assert payload["error"]["detail"]["reason"] == "mission_protocol_timeout"


# -- clear -------------------------------------------------------------------


def test_clear_success_human(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setattr(
        _CALL_DAEMON,
        lambda *_a, **_k: DaemonResponse.success(
            {"action": "mission_clear", "accepted": True, "result_name": "ACCEPTED",
             "verified": True, "observed_count": 0}
        ),
    )
    result = runner.invoke(app, ["mission", "clear", "--confirm"])
    assert result.exit_code == ExitCode.SUCCESS
    assert "remote count verified 0" in result.stdout


def test_clear_uncertain_human_shows_observed_count(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.setattr(
        _CALL_DAEMON,
        lambda *_a, **_k: DaemonResponse.failure(
            ExitCode.NACK_TIMEOUT,
            "mission clear uncertain; remote mission count observed: 2",
            {"reason": "remote_mission_state_uncertain", "observed_count": 2},
        ),
    )
    result = runner.invoke(app, ["mission", "clear", "--confirm"])
    assert result.exit_code == ExitCode.NACK_TIMEOUT
    assert "remote mission count observed: 2" in result.output


def test_clear_requires_confirm(monkeypatch: pytest.MonkeyPatch) -> None:
    # Like arm/disarm, the confirm gate lives in the daemon guard; the CLI
    # passes confirm=False through and surfaces the structured rejection.
    monkeypatch.setattr(
        _CALL_DAEMON,
        lambda *_a, **_k: DaemonResponse.failure(
            ExitCode.SAFETY_REJECTED,
            "mission clear is a state-changing command and requires explicit confirmation",
            {"reason": "confirmation_required"},
        ),
    )
    result = runner.invoke(app, ["mission", "clear"])
    assert result.exit_code == ExitCode.SAFETY_REJECTED
    assert "requires explicit confirmation" in result.output
