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

Identity: mavctl runs with the distinct GCS identity 254:190 (MAVProxy
1.8.74 defaults to 255:230), so the two do not share an on-wire identity
([FACT] ArduPilot rejects mission items whose sender identity differs from
the MISSION_COUNT sender). MAVProxy relays duplicate every packet in both
directions when it has more than one --out link; the adapter's upload
converges on that (request debounce + stray ACK tolerance) and the suite
runs against both topologies:

- default (MAVCTL_SITL_CONNECT unset): the standard shared sim_vehicle link
  `udp:127.0.0.1:14550` with MAVProxy in-band — the validated coexistence
  environment, same as tests/test_sitl.py;
- optional isolation: a DEDICATED MAVProxy-free instance
  (`sim_vehicle.py -v ArduCopter --instance 1 --no-mavproxy --no-rebuild`,
  SERIAL0 tcp:127.0.0.1:5770) via
  `MAVCTL_SITL_CONNECT=tcp:127.0.0.1:5770` — an optional protocol-isolation
  diagnostic, not a normal test prerequisite.

Real aircraft are out of scope: every endpoint must be loopback (enforced
below), and no flight control is ever exercised.
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

# Default endpoint: the standard shared sim_vehicle link with MAVProxy
# in-band (udp:127.0.0.1:14550) — same environment as tests/test_sitl.py;
# one plain `sim_vehicle.py -v ArduCopter` covers the whole -m sitl suite.
# Optional isolation: run a DEDICATED MAVProxy-free instance and point the
# env var at it (protocol-isolation diagnostic, not a test prerequisite):
#   sim_vehicle.py -v ArduCopter --instance 1 --no-mavproxy --no-rebuild
#   MAVCTL_SITL_CONNECT=tcp:127.0.0.1:5770 uv run pytest tests/test_mission_sitl.py -q
_CONNECT = os.environ.get("MAVCTL_SITL_CONNECT", "udp:127.0.0.1:14550")

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


def _await_ground_evidence(timeout: float = 20.0) -> dict[str, Any]:
    """Wait until the daemon caches guard-usable ground evidence.

    A freshly spawned daemon knows nothing until the first
    GLOBAL_POSITION_INT / EXTENDED_SYS_STATE arrives; the mission guards
    rightly refuse to act on an unknown ground state, so every test must
    wait for position/landed-state telemetry before exercising mission
    operations (a race here would surface as a spurious
    ``ground_state_unknown`` rejection).
    """

    deadline = time.monotonic() + timeout
    state = _status()
    while time.monotonic() < deadline:
        if (
            state.get("relative_alt_m") is not None
            or state.get("landed_state") is not None
        ):
            return state
        time.sleep(0.25)
        state = _status()
    pytest.fail("no ground evidence (position/landed_state) from SITL within timeout")


def _await_home_position(timeout: float = 20.0) -> dict[str, Any]:
    """Wait until the daemon caches the vehicle's HOME_POSITION.

    The download's home-slot verification must match the seq-0 item against
    the HOME_POSITION received from this vehicle (never a positional guess),
    so the tests wait for the verified home before exercising mission
    operations. The adapter requests a 1 Hz HOME_POSITION stream when the
    mission session opens (MAV_CMD_SET_MESSAGE_INTERVAL).
    """

    deadline = time.monotonic() + timeout
    state = _status()
    while time.monotonic() < deadline:
        if state.get("home_position") is not None:
            return state
        time.sleep(0.25)
        state = _status()
    pytest.fail("no HOME_POSITION cached by the daemon within timeout")


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
    process.spawn(_CONNECT, heartbeat_timeout=3.0, source_system=254)
    try:
        state = _await_connected()
        assert state.get("connected") is True, "no heartbeat from SITL within 20s"
        if state.get("armed"):
            pytest.fail(
                "SITL must be disarmed for mission protocol tests; refusing to "
                "run against an armed vehicle"
            )
        _await_ground_evidence()
        _await_home_position()
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


def _expected_round_trip(lat0: float, lon0: float) -> dict[str, Any]:
    """The exact mission the SITL upload should produce on download."""

    return {
        "version": 1,
        "items": [
            {"type": "takeoff", "altitude_m": 10.0},
            {
                "type": "waypoint",
                "lat_deg": round(lat0 + 0.0005, 7),
                "lon_deg": round(lon0 + 0.0005, 7),
                "altitude_m": 10.0,
                "hold_s": 0.0,
                "accept_radius_m": 0.0,
            },
            {
                "type": "waypoint",
                "lat_deg": round(lat0 + 0.001, 7),
                "lon_deg": round(lon0 + 0.001, 7),
                "altitude_m": 10.0,
                "hold_s": 0.0,
                "accept_radius_m": 0.0,
            },
            {"type": "rtl"},
        ],
    }


def test_mission_upload_succeeds(daemon: None) -> None:
    lat0, lon0 = _await_position()
    result = _upload_mission(_current_position_payload(lat0, lon0))

    assert result.get("accepted") is True
    assert result.get("result_name") == "ACCEPTED"
    assert result.get("item_count") == 4
    # sent_upto is the highest wire sequence sent: home slot (0) + 4 items
    assert result.get("sent_upto") == 4


def test_mission_download_round_trips_losslessly(daemon: None) -> None:
    """Upload → download returns exactly the v1 mission that was sent.

    ArduPilot's home-slot convention ([FACT] AP_Mission::add_cmd auto-inserts
    home at storage slot 0; get_item always exposes it as seq 0) means mavctl
    transfers its items at wire seqs 1..N and the download validates and
    excludes the home entry. The earlier "normalized takeoff" observation was
    this home slot being read as the first item, which also silently
    overwrote the real first item — both defects are fixed by the wire
    convention.
    """

    lat0, lon0 = _await_position()
    _upload_mission(_mission_payload(lat0, lon0))

    mission = _download_mission()
    assert mission == _expected_round_trip(lat0, lon0)

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


def test_mission_transfer_survives_relay_duplicated_traffic(daemon: None) -> None:
    """Regression: a relay with duplicated forward traffic (two MAVProxy --out
    links) duplicates the GCS's MISSION_COUNT, which re-initializes the
    vehicle's upload session mid-transfer; every item request then arrives
    twice. The upload must still converge (no INVALID_SEQUENCE abort, no
    premature-ACCEPTED abort) and the stored mission must be intact.

    Evidenced on the default shared sim_vehicle endpoint
    (udp:127.0.0.1:14550) where this is the live topology; on the optional
    MAVProxy-free endpoint the test still guards the adapter logic.
    """

    lat0, lon0 = _await_position()
    payload = _mission_payload(lat0, lon0)
    result = _upload_mission(payload)

    assert result.get("accepted") is True
    assert result.get("result_name") == "ACCEPTED"
    assert result.get("item_count") == 4

    # upload again without clearing: the second transfer replaces the first
    # under the same duplicated-traffic conditions and must also converge
    result2 = _upload_mission(payload)
    assert result2.get("accepted") is True
    assert result2.get("item_count") == 4

    # the stored mission is exactly what was sent, not a partial merge
    mission = _download_mission()
    assert mission == _expected_round_trip(lat0, lon0)


def test_cleanup_leaves_empty_mission_across_daemon_restart(daemon: None) -> None:
    lat0, lon0 = _await_position()
    _upload_mission(_mission_payload(lat0, lon0))

    # stop and respawn the daemon: the mission lives on the vehicle, not in
    # the daemon process
    process.stop()
    process.spawn(_CONNECT, heartbeat_timeout=3.0, source_system=254)
    state = _await_connected()
    assert state.get("connected") is True
    _await_ground_evidence()
    _await_home_position()

    _clear_mission()
    mission = _download_mission()
    assert mission == {"version": 1, "items": []}, "cleanup left mission items"
