"""Phase 3B-1 SITL execution conformance: `mavctl mission start` against a
live ArduCopter.

Safety contract for this module:

- loopback endpoint only (enforced at import);
- the only flight actions are: `mavctl arm --confirm` (to satisfy the
  mission-start armed guard), `mavctl mission start`, and the mission's own
  items (takeoff/waypoint/rtl) — plus `mavctl rtl --confirm --wait` as the
  bounded safety recovery;
- no `disarm --force`, no pause/resume/stop/cancel, no mode switches by
  mavctl other than the vehicle's own MISSION_START AUTO transition;
- every test leaves the vehicle disarmed/landed and the remote mission
  cleared with a verified empty read-back.

Requires an isolated no-MAVProxy SITL instance:

    sim_vehicle.py -v ArduCopter --instance 1 --no-mavproxy --no-rebuild

Endpoint: MAVCTL_SITL_CONNECT (default tcp:127.0.0.1:5770).
"""

from __future__ import annotations

import asyncio
import os
import shutil
import time
from collections.abc import Iterator
from typing import Any

import pytest

from mavctl.daemon import process
from mavctl.daemon.client import call_daemon
from mavctl.models.operation import OperationState

pytestmark = pytest.mark.sitl

_CONNECT = os.environ.get("MAVCTL_SITL_CONNECT", "tcp:127.0.0.1:5770")

if not (
    _CONNECT.startswith("udp:127.0.0.1")
    or _CONNECT.startswith("tcp:127.0.0.1")
    or _CONNECT.startswith("udp:localhost")
):
    raise RuntimeError(
        f"mission execution SITL tests require a loopback endpoint, got {_CONNECT!r}"
    )


def _status() -> dict[str, Any]:
    return call_daemon("status").result or {}


def _await(predicate: Any, timeout: float, fail_msg: str) -> dict[str, Any]:
    deadline = time.monotonic() + timeout
    state = _status()
    while time.monotonic() < deadline:
        if predicate(state):
            return state
        time.sleep(0.25)
        state = _status()
    pytest.fail(f"{fail_msg}: last state={_brief(state)}")


def _brief(state: dict[str, Any]) -> str:
    mission = state.get("mission") or {}
    return (
        f"connected={state.get('connected')} armed={state.get('armed')} "
        f"mode={state.get('flight_mode')} gps={state.get('gps', {}).get('fix_label')} "
        f"mission={mission.get('state')}/{mission.get('current_seq')}"
    )


def _await_connected(timeout: float = 30.0) -> dict[str, Any]:
    return _await(lambda s: bool(s.get("connected")), timeout, "no heartbeat from SITL")


def _await_gps_fix(timeout: float = 90.0) -> dict[str, Any]:
    return _await(
        lambda s: (s.get("gps") or {}).get("fix_label") in ("rtk_fixed", "dgps", "3d_fix"),
        timeout,
        "no 3D GPS fix from SITL",
    )


def _await_home(timeout: float = 60.0) -> dict[str, Any]:
    return _await(lambda s: bool(s.get("home_position")), timeout, "no HOME_POSITION")


def _await_ground_fresh(timeout: float = 60.0) -> dict[str, Any]:
    return _await(
        lambda s: bool(s.get("connected"))
        and (s.get("mission") or {}).get("age_s") is not None
        and (s.get("relative_alt_m") is not None)
        and (s.get("landed_state") in (None, "on_ground")),
        timeout,
        "no fresh ground evidence from SITL",
    )


def _await_armed(timeout: float = 30.0) -> None:
    _await(lambda s: s.get("armed") is True, timeout, "vehicle did not report armed")


def _await_disarmed_and_landed(timeout: float = 240.0) -> None:
    _await(
        lambda s: s.get("armed") is False
        and (s.get("relative_alt_m") or 0.0) < 1.0,
        timeout,
        "vehicle did not land and disarm after mission/RTL",
    )


def _await_mission_state(label: str, timeout: float) -> None:
    _await(
        lambda s: (s.get("mission") or {}).get("state") == label,
        timeout,
        f"mission state never reached {label!r}",
    )


def _download_mission() -> dict[str, Any]:
    response = call_daemon("mission_download", timeout=30)
    assert response.ok is True, f"mission download failed: {response.error}"
    return (response.result or {}).get("mission") or {}


def _clear_mission() -> None:
    response = call_daemon("mission_clear", {"confirm": True}, timeout=30)
    assert response.ok is True, f"mission clear failed: {response.error}"
    mission = _download_mission()
    assert mission == {"version": 1, "items": []}, "clear left mission items"


def _best_effort_cleanup() -> None:
    """Safety recovery: land (RTL) if airborne, then clear the mission."""

    try:
        if process.is_running():
            state = call_daemon("status").result or {}
            if state.get("armed") is True:
                rtl = call_daemon(
                    "rtl", {"confirm": True, "wait": True, "timeout": 180}, timeout=210
                )
                print(f"[cleanup] rtl accepted={rtl.ok}")
            cleared = call_daemon("mission_clear", {"confirm": True}, timeout=30)
            mission = call_daemon("mission_download", timeout=30)
            items = (mission.result or {}).get("mission", {}).get("items", [])
            print(
                f"[cleanup] clear accepted={cleared.ok} "
                f"remote items remaining={len(items)}"
            )
            if items:
                print(f"[cleanup] WARNING residual mission items: {items}")
    except Exception as exc:  # diagnostics only
        print(f"[cleanup] best-effort cleanup failed (diagnostic): {exc}")


@pytest.fixture
def execution_daemon(monkeypatch: pytest.MonkeyPatch) -> Iterator[None]:
    home = f"/tmp/mavctl_mission_exec_{os.getpid()}"
    monkeypatch.setenv("MAVCTL_HOME", home)
    if process.is_running():
        process.stop()
    process.spawn(_CONNECT, heartbeat_timeout=3.0, source_system=254)
    try:
        _await_connected()
        _await_gps_fix()
        _await_home()
        _await_ground_fresh()
        yield
    finally:
        _best_effort_cleanup()
        process.stop()
        shutil.rmtree(home, ignore_errors=True)


def _upload_execution_mission(lat0: float, lon0: float, *, near: bool = True) -> dict[str, Any]:
    """Mission: takeoff 10 m → near waypoint (hold 10 s) → RTL.

    ``near=False`` builds a longer leg (supersession test needs the mission
    ACTIVE for longer than the RTL takes to be accepted).
    """

    offset = 0.0003 if near else 0.0015
    payload = {
        "version": 1,
        "items": [
            {"type": "takeoff", "altitude_m": 10.0},
            {
                "type": "waypoint",
                "lat_deg": lat0 + offset,
                "lon_deg": lon0 + offset,
                "altitude_m": 10.0,
                "hold_s": 10.0 if not near else 0.0,
            },
            {"type": "rtl"},
        ],
    }
    response = call_daemon(
        "mission_upload", {"mission": payload, "confirm": True}, timeout=30
    )
    assert response.ok is True, f"mission upload failed: {response.error}"

    downloaded = _download_mission()
    assert downloaded["version"] == 1 and len(downloaded["items"]) == 3
    return payload


def _arm() -> None:
    """Arm for mission execution. ArduCopter refuses arming in AUTO, so the
    fixture first moves the vehicle to GUIDED (explicit fixture mode command,
    per the Phase 3B-1 task's allowed fixture actions)."""

    mode = call_daemon("mode", {"mode": "GUIDED", "confirm": True}, timeout=30)
    assert mode.ok is True, f"mode GUIDED failed: {mode.error}"
    response = call_daemon("arm", {"confirm": True}, timeout=30)
    assert response.ok is True, f"arm failed: {response.error}"
    _await_armed()


# -- Test 1: guard negatives (no flight-state change) ---------------------------


def test_mission_start_guards_negative_paths(execution_daemon: None) -> None:
    _await_mission_state("no_mission", timeout=30)

    # 1a. no --confirm → confirmation_required (exit 5)
    response = call_daemon("mission_start", {"confirm": False}, timeout=30)
    assert response.ok is False
    assert response.error is not None
    assert response.error.code == 5
    assert response.error.detail["reason"] == "confirmation_required"

    # 1b. no stored mission → mission_absent (exit 5); vehicle stays disarmed.
    # Robust to the ArduPilot home slot: after flight activity the vehicle
    # re-writes home on arming, so a cleared mission still answers
    # MISSION_COUNT == 1 — the guard treats wire count >= 2 as present.
    response = call_daemon("mission_start", {"confirm": True}, timeout=60)
    assert response.ok is False
    assert response.error is not None
    assert response.error.code == 5
    assert response.error.detail["reason"] == "mission_absent"
    state = _status()
    assert state.get("armed") is False  # no execution happened

    # 1c. mission uploaded but vehicle disarmed → mission_requires_armed
    lat0, lon0 = _await_position()
    upload = call_daemon(
        "mission_upload",
        {"mission": _execution_payload(lat0, lon0), "confirm": True},
        timeout=30,
    )
    assert upload.ok is True, f"upload failed: {upload.error}"
    response = call_daemon("mission_start", {"confirm": True}, timeout=60)
    assert response.ok is False
    assert response.error is not None
    assert response.error.code == 5
    assert response.error.detail["reason"] == "mission_requires_armed"
    # no flight happened
    assert _status().get("armed") is False

    _clear_mission()


def _await_position(timeout: float = 30.0) -> tuple[float, float]:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        telemetry = call_daemon("telemetry")
        if telemetry.ok:
            position = (telemetry.result or {}).get("position") or {}
            lat = position.get("lat_deg")
            lon = position.get("lon_deg")
            if lat is not None and lon is not None:
                return float(lat), float(lon)
        time.sleep(0.25)
    pytest.fail("no vehicle position from SITL within timeout")


def _execution_payload(lat0: float, lon0: float) -> dict[str, Any]:
    return {
        "version": 1,
        "items": [
            {"type": "takeoff", "altitude_m": 10.0},
            {
                "type": "waypoint",
                "lat_deg": lat0 + 0.0003,
                "lon_deg": lon0 + 0.0003,
                "altitude_m": 10.0,
                "hold_s": 10.0,
            },
            {"type": "rtl"},
        ],
    }


# -- Test 2: happy path -----------------------------------------------------------


def test_mission_start_happy_path_full_execution(execution_daemon: None) -> None:
    lat0, lon0 = _await_position()
    uploaded = call_daemon(
        "mission_upload",
        {"mission": _execution_payload(lat0, lon0), "confirm": True},
        timeout=30,
    )
    assert uploaded.ok is True, f"upload failed: {uploaded.error}"
    downloaded = _download_mission()
    assert len(downloaded["items"]) == 3  # lossless readback

    _arm()

    start = call_daemon(
        "mission_start",
        {"confirm": True, "wait": True, "timeout": 90},
        timeout=120,
    )
    assert start.ok is True, f"mission start failed: {start.error}"
    result = start.result or {}
    operation_id = result["operation_id"]
    assert operation_id.startswith("op-")

    # vehicle transitioned to AUTO and the mission is executing
    _await_mission_state("active", timeout=30)
    state = _status()
    assert state.get("flight_mode") == "AUTO"
    mission = state.get("mission") or {}
    assert mission.get("state") == "active"
    assert mission.get("mode") == "mission"
    assert mission.get("total") == 3  # home excluded
    assert mission.get("age_s") is not None

    # real execution evidence: the vehicle climbs (takeoff item), not just an ACK
    climbed = _await(
        lambda s: (s.get("relative_alt_m") or 0.0) >= 5.0,
        timeout=90,
        fail_msg="vehicle never climbed after mission start",
    )
    assert (climbed.get("relative_alt_m") or 0.0) >= 5.0

    # operation observed the milestone
    op = call_daemon("operation_get", {"operation_id": operation_id}, timeout=30)
    assert op.ok is True
    op_state = (op.result or {}).get("operation", {}).get("state")
    assert op_state in (OperationState.REACHED.value, "reached")

    # the mission's own RTL item ends the flight: lands + disarms + COMPLETE
    _await_mission_state("complete", timeout=240)
    _await_disarmed_and_landed(timeout=240)

    _clear_mission()


# -- Test 3: already running idempotent --------------------------------------------


def test_mission_start_already_running_idempotent(execution_daemon: None) -> None:
    lat0, lon0 = _await_position()
    _upload_execution_mission(lat0, lon0)
    _arm()

    start = call_daemon(
        "mission_start",
        {"confirm": True, "wait": True, "timeout": 90},
        timeout=120,
    )
    assert start.ok is True
    first_operation_id = (start.result or {}).get("operation_id")
    assert first_operation_id is not None
    _await_mission_state("active", timeout=30)
    # second start while AUTO + ACTIVE: idempotent no-op via the shared
    # already_satisfied shape — no new command, no new operation
    again = call_daemon("mission_start", {"confirm": True}, timeout=60)
    assert again.ok is True
    again_result = again.result or {}
    assert again_result.get("already_satisfied") is True
    assert again_result.get("executed") is False
    assert again_result.get("note") == "already running"
    assert again_result.get("operation_id") is None

    # no mission restart: state stays active and the first operation record
    # is unchanged (never superseded — nothing replaced it)
    state = _status()
    assert (state.get("mission") or {}).get("state") == "active"
    assert state.get("flight_mode") == "AUTO"
    first_operation = server_operations_get(first_operation_id)
    assert first_operation is not None
    assert first_operation["state"] == "reached"
    assert first_operation["superseded_by_operation_id"] is None

    # the mission finishes via its own RTL item
    _await_mission_state("complete", timeout=240)
    _await_disarmed_and_landed(timeout=240)
    _clear_mission()


# -- Test 4: RTL supersession -------------------------------------------------------


async def test_mission_start_superseded_by_rtl(execution_daemon: None) -> None:
    """§C.2.1-D at SITL: RTL's accepted ACK supersedes a still-WAITING
    mission_start observation. The RTL is dispatched immediately after the
    start command returns (the start operation is WAITING the moment the
    start ACK registers it), before the vehicle's MISSION_CURRENT ACTIVE
    observation typically lands. The superseded start operation can never
    report reached afterwards (epoch fencing)."""

    lat0, lon0 = _await_position()
    # hold 10 s at the waypoint keeps the mission executing long enough for
    # the superseded operation to stay superseded (no restart) during RTL
    _upload_execution_mission(lat0, lon0, near=False)
    _arm()

    start = call_daemon(
        "mission_start", {"confirm": True}, timeout=60
    )
    assert start.ok is True, f"mission start failed: {start.error}"
    start_op = (start.result or {}).get("operation_id")
    assert start_op is not None
    start_snapshot = server_operations_get(start_op)
    assert start_snapshot is not None
    assert start_snapshot["state"] == "waiting"

    # immediate RTL: its ACK supersedes the WAITING start observation
    rtl = call_daemon("rtl", {"confirm": True}, timeout=60)
    assert rtl.ok is True, f"rtl failed: {rtl.error}"
    rtl_result = rtl.result or {}
    rtl_id = rtl_result["operation_id"]

    start_snapshot = server_operations_get(start_op)
    assert start_snapshot is not None
    assert start_snapshot["state"] == "superseded"
    assert start_snapshot["superseded_by_operation_id"] == rtl_id
    assert start_snapshot["terminal_reason"] != "milestone reached"

    # the vehicle flies home and lands (the RTL command is executing); the
    # RTL operation's own watcher records REACHED once armed == false
    _await_disarmed_and_landed(timeout=240)
    rtl_snapshot = server_operations_get(rtl_id)
    for _ in range(50):
        assert rtl_snapshot is not None
        if rtl_snapshot["state"] == "reached":
            break
        await asyncio.sleep(0.1)
        rtl_snapshot = server_operations_get(rtl_id)
    assert rtl_snapshot is not None
    assert rtl_snapshot["state"] == "reached"

    _clear_mission()


def server_operations_get(operation_id: str) -> dict[str, Any] | None:
    op = call_daemon("operation_get", {"operation_id": operation_id}, timeout=30)
    if not op.ok:
        return None
    return (op.result or {}).get("operation")
