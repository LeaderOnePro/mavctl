"""Daemon RPC contract tests for mission commands (Phase 3A)."""

from __future__ import annotations

import asyncio
from typing import Any

from mavctl.adapter.base import (
    MissionCountUnsupportedError,
    MissionItemUnsupportedError,
    MissionProtocolError,
    MissionStateUncertainError,
)
from mavctl.daemon.server import DaemonServer
from mavctl.models import (
    DownloadedMissionV1,
    ExitCode,
    GpsInfo,
    MissionOutcome,
    MissionV1,
    VehicleState,
)
from tests.test_server import FakeAdapter, _params


def _grounded_state(**kw: Any) -> VehicleState:
    base: dict[str, Any] = {
        "connected": True,
        "heartbeat_age_s": 0.2,
        "flight_mode": "STABILIZE",
        "armed": False,
        "landed_state": "on_ground",
        "relative_alt_m": 0.0,
        "telemetry_age_s": 0.2,
        "landed_state_age_s": 0.2,
        "gps": GpsInfo(fix_type=6, fix_label="rtk_fixed"),
    }
    base.update(kw)
    return VehicleState(**base)


def _mission_payload() -> dict[str, Any]:
    return MissionV1.model_validate(
        {
            "version": 1,
            "items": [
                {"type": "takeoff", "altitude_m": 10.0},
                {"type": "waypoint", "lat_deg": 1.0, "lon_deg": 2.0, "altitude_m": 15.0},
                {"type": "rtl"},
            ],
        }
    ).model_dump()


def _mission_server(**state_kw: Any) -> tuple[FakeAdapter, DaemonServer]:
    adapter = FakeAdapter(_grounded_state(**state_kw))
    return adapter, DaemonServer(adapter, "udp:127.0.0.1:14550")


class _ScriptedMissionAdapter(FakeAdapter):
    """FakeAdapter with scripted mission transaction outcomes."""

    def __init__(self, state: VehicleState, *, outcome: Any = None, error: Exception | None = None):
        super().__init__(state)
        self._outcome = outcome
        self._error = error

    def upload_mission(self, mission: MissionV1) -> MissionOutcome:
        self.calls.append(f"upload_mission({len(mission.items)} items)")
        if self._error is not None:
            raise self._error
        assert isinstance(self._outcome, MissionOutcome)
        return self._outcome

    def download_mission(self) -> DownloadedMissionV1:
        self.calls.append("download_mission")
        if self._error is not None:
            raise self._error
        assert isinstance(self._outcome, DownloadedMissionV1)
        return self._outcome

    def clear_mission(self, **_kw: Any) -> MissionOutcome:
        self.calls.append("clear_mission")
        if self._error is not None:
            raise self._error
        assert isinstance(self._outcome, MissionOutcome)
        return self._outcome


async def test_mission_upload_happy_path() -> None:
    adapter, server = _mission_server()
    response = await server._dispatch(
        _params(method="mission_upload", mission=_mission_payload(), confirm=True)
    )
    assert response.ok is True
    assert response.result is not None
    assert response.result["accepted"] is True
    assert response.result["item_count"] == 3
    assert adapter.calls == ["upload_mission"]


async def test_mission_upload_invalid_mission_is_exit_2() -> None:
    adapter, server = _mission_server()
    bad = {"version": 1, "items": []}  # upload requires non-empty
    response = await server._dispatch(_params(method="mission_upload", mission=bad, confirm=True))
    assert response.ok is False
    assert response.error is not None
    assert response.error.code == ExitCode.USAGE_ERROR
    assert response.error.detail["reason"] == "invalid_mission"
    assert adapter.calls == []  # adapter never reached


async def test_mission_upload_schema_cannot_be_bypassed_via_rpc() -> None:
    # A direct RPC with a raw MAVLink command in place of a semantic item
    # must be rejected at the daemon boundary.
    adapter, server = _mission_server()
    raw = {"version": 1, "items": [{"type": "takeoff", "altitude_m": 10.0, "command": 22}]}
    response = await server._dispatch(_params(method="mission_upload", mission=raw, confirm=True))
    assert response.ok is False
    assert response.error is not None
    assert response.error.code == ExitCode.USAGE_ERROR
    assert adapter.calls == []


async def test_mission_upload_requires_confirm() -> None:
    adapter, server = _mission_server()
    response = await server._dispatch(
        _params(method="mission_upload", mission=_mission_payload(), confirm=False)
    )
    assert response.ok is False
    assert response.error is not None
    assert response.error.code == ExitCode.SAFETY_REJECTED
    assert response.error.detail["reason"] == "confirmation_required"
    assert adapter.calls == []


async def test_mission_upload_requires_disarmed() -> None:
    adapter, server = _mission_server(armed=True)
    response = await server._dispatch(
        _params(method="mission_upload", mission=_mission_payload(), confirm=True)
    )
    assert response.ok is False
    assert response.error is not None
    assert response.error.code == ExitCode.SAFETY_REJECTED
    assert response.error.detail["reason"] == "mission_requires_disarmed"
    assert adapter.calls == []


async def test_mission_upload_dry_run_sends_nothing() -> None:
    adapter, server = _mission_server()
    response = await server._dispatch(
        _params(method="mission_upload", mission=_mission_payload(), confirm=True, dry_run=True)
    )
    assert response.ok is True
    assert response.result is not None
    assert response.result["dry_run"] is True
    assert response.result["would_execute"] is True
    assert adapter.calls == []  # guards ran, adapter untouched


async def test_mission_upload_uncertain_maps_to_exit_6() -> None:
    error = MissionStateUncertainError("vehicle vanished", sent_upto=1)
    adapter = _ScriptedMissionAdapter(_grounded_state(), error=error)
    server = DaemonServer(adapter, "udp:127.0.0.1:14550")
    response = await server._dispatch(
        _params(method="mission_upload", mission=_mission_payload(), confirm=True)
    )
    assert response.ok is False
    assert response.error is not None
    assert response.error.code == ExitCode.NACK_TIMEOUT
    assert response.error.detail["reason"] == "remote_mission_state_uncertain"
    assert response.error.detail["sent_upto"] == 1
    assert "internal" not in (response.error.message or "").lower()


async def test_mission_upload_sequence_gap_surfaces_expected_and_requested() -> None:
    error = MissionStateUncertainError("future item requested", sent_upto=0)
    error.expected_seq = 1  # type: ignore[attr-defined]
    error.requested_seq = 2  # type: ignore[attr-defined]
    adapter = _ScriptedMissionAdapter(_grounded_state(), error=error)
    server = DaemonServer(adapter, "udp:127.0.0.1:14550")
    response = await server._dispatch(
        _params(method="mission_upload", mission=_mission_payload(), confirm=True)
    )
    assert response.ok is False
    assert response.error is not None
    assert response.error.code == ExitCode.NACK_TIMEOUT
    assert response.error.detail["reason"] == "remote_mission_state_uncertain"
    assert response.error.detail["expected_seq"] == 1
    assert response.error.detail["requested_seq"] == 2
    assert response.error.detail["hint"] == (
        "verify the remote mission with 'mavctl mission download'"
    )


async def test_mission_upload_rejection_maps_to_exit_6() -> None:
    error = MissionProtocolError("no space", result_name="NO_SPACE")
    adapter = _ScriptedMissionAdapter(_grounded_state(), error=error)
    server = DaemonServer(adapter, "udp:127.0.0.1:14550")
    response = await server._dispatch(
        _params(method="mission_upload", mission=_mission_payload(), confirm=True)
    )
    assert response.ok is False
    assert response.error is not None
    assert response.error.code == ExitCode.NACK_TIMEOUT
    assert response.error.detail["reason"] == "mission_rejected"
    assert response.error.detail["result_name"] == "NO_SPACE"


async def test_mission_download_happy_path() -> None:
    mission = DownloadedMissionV1(version=1, items=[])
    adapter = _ScriptedMissionAdapter(_grounded_state(), outcome=mission)
    server = DaemonServer(adapter, "udp:127.0.0.1:14550")
    response = await server._dispatch(_params(method="mission_download"))
    assert response.ok is True
    assert response.result is not None
    assert response.result["mission"] == {"version": 1, "items": []}


async def test_mission_download_unsupported_item_maps_to_exit_6() -> None:
    error = MissionItemUnsupportedError("bad item", seq=2, command=999, frame=6)
    adapter = _ScriptedMissionAdapter(_grounded_state(), error=error)
    server = DaemonServer(adapter, "udp:127.0.0.1:14550")
    response = await server._dispatch(_params(method="mission_download"))
    assert response.ok is False
    assert response.error is not None
    assert response.error.code == ExitCode.NACK_TIMEOUT
    assert response.error.detail["reason"] == "mission_item_unsupported"
    assert response.error.detail["seq"] == 2
    assert response.error.detail["command"] == 999


async def test_mission_download_count_over_limit_maps_to_exit_6() -> None:
    # observed_count is the WIRE count (home slot included); the error detail
    # reports v1 items (wire - 1).
    error = MissionCountUnsupportedError(
        "remote mission has 102 items", observed_count=102, max_supported_items=100
    )
    adapter = _ScriptedMissionAdapter(_grounded_state(), error=error)
    server = DaemonServer(adapter, "udp:127.0.0.1:14550")
    response = await server._dispatch(_params(method="mission_download"))
    assert response.ok is False
    assert response.error is not None
    assert response.error.code == ExitCode.NACK_TIMEOUT
    assert response.error.detail["reason"] == "mission_item_unsupported"
    assert response.error.detail["observed_count"] == 101
    assert response.error.detail["max_supported_items"] == 100


async def test_mission_download_protocol_timeout_maps_to_exit_6() -> None:
    error = MissionProtocolError("vehicle silent", result_name="TIMEOUT")
    adapter = _ScriptedMissionAdapter(_grounded_state(), error=error)
    server = DaemonServer(adapter, "udp:127.0.0.1:14550")
    response = await server._dispatch(_params(method="mission_download"))
    assert response.ok is False
    assert response.error is not None
    assert response.error.code == ExitCode.NACK_TIMEOUT
    assert response.error.detail["reason"] == "mission_protocol_timeout"


async def test_mission_download_does_not_hold_command_lock() -> None:
    """A download must complete while a state-changing command holds the
    daemon _command_lock (read-only path, design §H)."""

    mission = DownloadedMissionV1(version=1, items=[])
    adapter = _ScriptedMissionAdapter(_grounded_state(), outcome=mission)
    server = DaemonServer(adapter, "udp:127.0.0.1:14550")

    async with server._command_lock:
        response = await server._dispatch(_params(method="mission_download"))
    assert response.ok is True
    assert response.result is not None
    assert response.result["mission"]["items"] == []


async def test_mission_upload_holds_command_lock() -> None:
    """An upload must wait for the daemon _command_lock (state-changing)."""

    outcome = MissionOutcome(action="mission_upload", accepted=True, item_count=3)
    adapter = _ScriptedMissionAdapter(_grounded_state(), outcome=outcome)
    server = DaemonServer(adapter, "udp:127.0.0.1:14550")

    request = _params(
        method="mission_upload", mission=_mission_payload(), confirm=True
    )
    async with server._command_lock:
        task = asyncio.create_task(server._dispatch(request))
        await asyncio.sleep(0.1)
        assert not task.done()  # waiting for the command lock
    response = await asyncio.wait_for(task, timeout=5.0)
    assert response.ok is True


async def test_mission_clear_happy_path_verified() -> None:
    outcome = MissionOutcome(action="mission_clear", accepted=True, verified=True, observed_count=0)
    adapter = _ScriptedMissionAdapter(_grounded_state(), outcome=outcome)
    server = DaemonServer(adapter, "udp:127.0.0.1:14550")
    response = await server._dispatch(_params(method="mission_clear", confirm=True))
    assert response.ok is True
    assert response.result is not None
    assert response.result["verified"] is True
    assert response.result["observed_count"] == 0


async def test_mission_clear_uncertain_without_observed_count() -> None:
    """When the read-back itself times out the count is unknown: the message
    must not claim an observed count and the detail must omit the key."""

    error = MissionStateUncertainError("read-back timeout", observed_count=None)
    adapter = _ScriptedMissionAdapter(_grounded_state(), error=error)
    server = DaemonServer(adapter, "udp:127.0.0.1:14550")
    response = await server._dispatch(_params(method="mission_clear", confirm=True))
    assert response.ok is False
    assert response.error is not None
    assert response.error.code == ExitCode.NACK_TIMEOUT
    assert response.error.detail["reason"] == "remote_mission_state_uncertain"
    assert "observed_count" not in response.error.detail
    assert "could not be observed" in (response.error.message or "")


async def test_mission_clear_uncertain_includes_observed_count() -> None:
    error = MissionStateUncertainError("count nonzero", observed_count=2)
    adapter = _ScriptedMissionAdapter(_grounded_state(), error=error)
    server = DaemonServer(adapter, "udp:127.0.0.1:14550")
    response = await server._dispatch(_params(method="mission_clear", confirm=True))
    assert response.ok is False
    assert response.error is not None
    assert response.error.code == ExitCode.NACK_TIMEOUT
    assert response.error.detail["reason"] == "remote_mission_state_uncertain"
    assert response.error.detail["observed_count"] == 2
    # P1: the message names the clear operation, never the upload
    assert "mission_clear" in (response.error.message or "")
    assert "mission_clear uncertain" in (response.error.message or "")
    assert "mission_upload" not in (response.error.message or "")


async def test_unexpected_mission_exception_still_internal_error() -> None:
    """A genuinely unexpected adapter bug must not be masked as a mission
    rejection."""

    adapter = _ScriptedMissionAdapter(
        _grounded_state(), error=RuntimeError("boom: adapter bug")
    )
    server = DaemonServer(adapter, "udp:127.0.0.1:14550")
    response = await server._dispatch(
        _params(method="mission_upload", mission=_mission_payload(), confirm=True)
    )
    assert response.ok is False
    assert response.error is not None
    assert response.error.code == ExitCode.GENERAL_ERROR
    assert "internal error" in (response.error.message or "")


async def test_mission_download_requires_connection() -> None:
    _adapter, server = _mission_server(connected=False)
    response = await server._dispatch(_params(method="mission_download"))
    assert response.ok is False
    assert response.error is not None
    assert response.error.code == ExitCode.VEHICLE_NOT_CONNECTED


async def test_mission_status_stays_readable_during_mission_call() -> None:
    """status (fast path) must remain callable while a mission transaction is
    in flight."""

    release = asyncio.Event()

    class _BlockingMissionAdapter(FakeAdapter):
        def upload_mission(self, mission: MissionV1) -> MissionOutcome:
            # runs in the executor; block until the test has read status
            import time as _time

            deadline = _time.monotonic() + 2.0
            while not release.is_set() and _time.monotonic() < deadline:
                _time.sleep(0.02)
            return MissionOutcome(action="mission_upload", accepted=True, item_count=3)

    adapter = _BlockingMissionAdapter(_grounded_state())
    server = DaemonServer(adapter, "udp:127.0.0.1:14550")
    task = asyncio.create_task(
        server._dispatch(_params(method="mission_upload", mission=_mission_payload(), confirm=True))
    )
    await asyncio.sleep(0.05)
    status = await server._dispatch(_params(method="status"))
    assert status.ok is True  # fast path not blocked by the mission call
    release.set()
    response = await asyncio.wait_for(task, timeout=5.0)
    assert response.ok is True
