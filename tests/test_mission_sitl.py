"""Phase 3A mission protocol conformance tests against local ArduPilot SITL.

Safety contract for this module:

- the endpoint must be a loopback address (enforced at import);
- the tests never arm, take off, switch to AUTO, or start a mission —
  upload/download/clear only touch the stored mission plan, never flight
  control;
- the SITL vehicle must be disarmed; a vehicle that reports armed fails the
  module instead of being forcibly disarmed;
- every test cleans up: best-effort mission clear, read-back diagnostic,
  daemon stop, temporary runtime directory removal.

Environment requirement (SITL-observed [FACT]): the mission link must NOT
have a concurrent in-band GCS relaying mission traffic. sim_vehicle's
MAVProxy (default sysid 255, the same as mavctl's) races the upload — the
observed pattern is duplicated MISSION_REQUESTs and a premature
MISSION_ACK(INVALID_SEQUENCE) after only two items. Run SITL with
`--no-mavproxy` (dedicated instance) or point MAVCTL_SITL_CONNECT at a
MAVProxy-free loopback link.
"""

from __future__ import annotations

import os
import shutil
import time
from collections.abc import Iterator
from typing import Any

import pytest

from mavctl.daemon import process
from mavctl.daemon.client import call_daemon
from mavctl.models import (
    DownloadedMissionV1,
)

pytestmark = pytest.mark.sitl

# Default endpoint: a DEDICATED MAVProxy-free SITL instance:
#   sim_vehicle.py -v ArduCopter --instance 1 --no-mavproxy --no-rebuild
# (SERIAL0 tcp server on 127.0.0.1:5770). Override with MAVCTL_SITL_CONNECT.
# A shared sim_vehicle link (default udp:127.0.0.1:14550 with MAVProxy) is
# NOT usable for mission upload: its in-band mission module races mavctl
# (duplicated MISSION_REQUESTs, premature INVALID_SEQUENCE ACK after two
# items — verified by the raw-pymavlink probe during Phase 3A SITL bring-up).
_CONNECT = os.environ.get("MAVCTL_SITL_CONNECT", "tcp:127.0.0.1:5770")

if not (
    _CONNECT.startswith("udp:127.0.0.1")
    or _CONNECT.startswith("tcp:127.0.0.1")
    or _CONNECT.startswith("udp:localhost")
):
    raise RuntimeError(
        f"mission SITL tests require a loopback endpoint, got {_CONNECT!r}"
    )


def _status() -> dict[str, Any]:
    return call_daemon("status").result or {}


def _await_connected(timeout: float = 20.0) -> dict[str, Any]:
    deadline = time.monotonic() + timeout
    state = _status()
    while time.monotonic() < deadline and not state.get("connected"):
        time.sleep(0.5)
        state = _status()
    return state


def _await_position(timeout: float = 20.0) -> tuple[float, float]:
    """Wait for a vehicle position fix and return (lat_deg, lon_deg)."""

    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        telemetry = call_daemon("telemetry")
        if telemetry.ok:
            position = (telemetry.result or {}).get("position") or {}
            lat = position.get("lat_deg")
            lon = position.get("lon_deg")
            if lat is not None and lon is not None:
                return float(lat), float(lon)
        time.sleep(0.5)
    pytest.fail("no vehicle position from SITL within timeout")


def _download_mission() -> dict[str, Any]:
    response = call_daemon("mission_download", timeout=30)
    assert response.ok is True, f"mission download failed: {response.error}"
    return (response.result or {}).get("mission") or {}


def _upload_mission(payload: dict[str, Any]) -> dict[str, Any]:
    response = call_daemon(
        "mission_upload",
        {"mission": payload, "confirm": True},
        timeout=30,
    )
    assert response.ok is True, f"mission upload failed: {response.error}"
    return response.result or {}


def _clear_mission() -> dict[str, Any]:
    response = call_daemon("mission_clear", {"confirm": True}, timeout=30)
    assert response.ok is True, f"mission clear failed: {response.error}"
    return response.result or {}


def _best_effort_cleanup() -> None:
    """Best-effort post-test cleanup with explicit diagnostics.

    Never raises: a failed cleanup is printed as a diagnostic so the
    original test failure is not masked, and the daemon is always stopped.
    """

    try:
        if process.is_running():
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
        print(f"[cleanup] best-effort clear failed (diagnostic): {exc}")


@pytest.fixture
def daemon(monkeypatch: pytest.MonkeyPatch) -> Iterator[None]:
    # AF_UNIX paths are capped near 104 bytes on macOS: keep MAVCTL_HOME short.
    home = f"/tmp/mavctl_mission_sitl_{os.getpid()}"
    monkeypatch.setenv("MAVCTL_HOME", home)
    if process.is_running():
        process.stop()
    process.spawn(_CONNECT, heartbeat_timeout=3.0)
    try:
        state = _await_connected()
        assert state.get("connected") is True, "no heartbeat from SITL within 20s"
        if state.get("armed"):
            pytest.fail(
                "SITL must be disarmed for mission protocol tests; refusing to "
                "run against an armed vehicle"
            )
        yield
    finally:
        _best_effort_cleanup()
        process.stop()
        shutil.rmtree(home, ignore_errors=True)


def _mission_payload(lat0: float, lon0: float) -> dict[str, Any]:
    """Four-item mission near the vehicle's current position."""

    return {
        "version": 1,
        "items": [
            {"type": "takeoff", "altitude_m": 10.0},
            {
                "type": "waypoint",
                "lat_deg": lat0 + 0.0005,
                "lon_deg": lon0 + 0.0005,
                "altitude_m": 10.0,
                "hold_s": 0.0,
                "accept_radius_m": 0.0,
            },
            {
                "type": "waypoint",
                "lat_deg": lat0 + 0.001,
                "lon_deg": lon0 + 0.001,
                "altitude_m": 10.0,
            },
            {"type": "rtl"},
        ],
    }


def _current_position_payload(lat0: float, lon0: float) -> dict[str, Any]:
    return _mission_payload(lat0, lon0)


# -- conformance tests -------------------------------------------------------


def test_mission_upload_succeeds(daemon: None) -> None:
    lat0, lon0 = _await_position()
    result = _upload_mission(_current_position_payload(lat0, lon0))

    assert result.get("accepted") is True
    assert result.get("result_name") == "ACCEPTED"
    assert result.get("item_count") == 4
    assert result.get("sent_upto") == 3

    # NOTE: the immediate download after upload is intentionally NOT asserted
    # here — ArduCopter normalizes the uploaded takeoff into a GLOBAL-frame
    # waypoint, which mavctl v1 reports as mission_item_unsupported (see
    # test_mission_download_reports_normalized_takeoff_atomically).


def test_mission_download_reports_normalized_takeoff_atomically(
    daemon: None,
) -> None:
    """SITL-observed ArduCopter behavior: an uploaded NAV_TAKEOFF item is
    normalized on the vehicle into a GLOBAL-frame waypoint at the current
    position with the home MSL altitude — it does not survive as a takeoff.
    mavctl v1 cannot represent that item losslessly, so download must fail
    atomically with mission_item_unsupported (never emit partial/lossy
    JSON)."""

    lat0, lon0 = _await_position()
    _upload_mission(_mission_payload(lat0, lon0))

    response = call_daemon("mission_download", timeout=30)
    assert response.ok is False, (
        "unexpected success: the normalized takeoff should be atomically unsupported"
    )
    assert response.error is not None
    detail = response.error.detail
    assert detail.get("reason") == "mission_item_unsupported"
    assert detail.get("seq") == 0
    assert detail.get("command") == 16  # normalized to WAYPOINT by ArduCopter
    assert "frame" in detail

    # the daemon must remain healthy for subsequent operations
    assert call_daemon("ping").ok is True


def test_mission_clear_verified_by_readback(daemon: None) -> None:
    lat0, lon0 = _await_position()
    _upload_mission(_mission_payload(lat0, lon0))

    result = _clear_mission()
    assert result.get("accepted") is True
    assert result.get("verified") is True
    assert result.get("observed_count") == 0

    # never trust the clear ACK alone: read back the remote mission
    mission = _download_mission()
    assert mission == {"version": 1, "items": []}


def test_mission_download_empty_after_clear(daemon: None) -> None:
    _clear_mission()

    mission = _download_mission()
    assert mission == {"version": 1, "items": []}
    parsed = DownloadedMissionV1.model_validate(mission)
    assert parsed.items == []
    # an empty mission must not produce item requests — the download returned
    # promptly without errors, and the daemon is still responsive
    assert call_daemon("ping").ok is True


def test_cleanup_leaves_empty_mission_across_daemon_restart(daemon: None) -> None:
    lat0, lon0 = _await_position()
    _upload_mission(_mission_payload(lat0, lon0))

    # stop and respawn the daemon: the mission lives on the vehicle, not in
    # the daemon process
    process.stop()
    process.spawn(_CONNECT, heartbeat_timeout=3.0)
    state = _await_connected()
    assert state.get("connected") is True

    _clear_mission()
    mission = _download_mission()
    assert mission == {"version": 1, "items": []}, "cleanup left mission items"
