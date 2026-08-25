"""SITL integration tests.

Require a running ArduPilot SITL streaming to ``udp:127.0.0.1:14550``:

    sim_vehicle.py -v ArduCopter --out udp:127.0.0.1:14550

Run explicitly with:  pytest -m sitl
Skipped by default via:  pytest -m "not sitl"
Override the endpoint with the MAVCTL_SITL_CONNECT env var.
"""

from __future__ import annotations

import os
import shutil
import time
from collections.abc import Iterator

import pytest

from mavctl.daemon import process
from mavctl.daemon.client import call_daemon
from mavctl.models import DaemonResponse

pytestmark = pytest.mark.sitl

_CONNECT = os.environ.get("MAVCTL_SITL_CONNECT", "udp:127.0.0.1:14550")


def _status() -> dict[str, object]:
    return call_daemon("status").result or {}


def _await_connected(timeout: float = 15.0) -> dict[str, object]:
    deadline = time.monotonic() + timeout
    state = _status()
    while time.monotonic() < deadline and not state.get("connected"):
        time.sleep(0.5)
        state = _status()
    return state


def _await_gps_fix(min_fix: int = 3, timeout: float = 15.0) -> dict[str, object]:
    """Wait until status reports a GPS fix (heartbeat can arrive before GPS_RAW_INT)."""

    deadline = time.monotonic() + timeout
    state = _status()
    while time.monotonic() < deadline:
        gps = state.get("gps") if isinstance(state.get("gps"), dict) else {}
        fix = gps.get("fix_type") if isinstance(gps, dict) else None
        if isinstance(fix, int) and fix >= min_fix:
            return state
        time.sleep(0.5)
        state = _status()
    return state


@pytest.fixture
def daemon(monkeypatch: pytest.MonkeyPatch) -> Iterator[None]:
    # AF_UNIX paths are capped near 104 bytes on macOS, so keep MAVCTL_HOME short.
    home = f"/tmp/mavctl_sitl_{os.getpid()}"
    monkeypatch.setenv("MAVCTL_HOME", home)
    if process.is_running():
        process.stop()
    process.spawn(_CONNECT, heartbeat_timeout=3.0)
    try:
        yield
    finally:
        process.stop()
        shutil.rmtree(home, ignore_errors=True)


def test_link_reports_connected_and_telemetry(daemon: None) -> None:
    state = _await_connected()
    assert state.get("connected") is True, "no heartbeat from SITL within 15s"
    assert state.get("flight_mode") is not None

    telemetry = call_daemon("telemetry")
    assert telemetry.ok is True
    assert telemetry.result is not None
    assert telemetry.result["position"]["lat_deg"] is not None


def test_guard_rejects_arm_without_confirm(daemon: None) -> None:
    _await_connected()
    resp = call_daemon("arm", {})
    assert resp.ok is False
    assert resp.error is not None
    assert resp.error.code == 5  # SAFETY_REJECTED
    assert resp.error.detail.get("reason") == "confirmation_required"


def test_full_flight_flow(daemon: None) -> None:
    state = _await_connected()
    assert state.get("connected") is True
    state = _await_gps_fix()
    gps_obj = state.get("gps")
    gps: dict[str, object] = gps_obj if isinstance(gps_obj, dict) else {}
    fix = gps.get("fix_type")
    assert isinstance(fix, int) and fix >= 3, f"no 3D GPS fix within timeout: {gps}"

    def ok(resp: DaemonResponse, what: str) -> DaemonResponse:
        assert resp.ok is True, f"{what} failed: {resp.error}"
        return resp

    # mode GUIDED -> arm -> takeoff 10 (wait) -> rtl (wait for disarm on land)
    ok(call_daemon("mode", {"mode": "GUIDED", "confirm": True, "wait": True, "timeout": 15},
                   timeout=30), "mode GUIDED")
    ok(call_daemon("arm", {"confirm": True}, timeout=25), "arm")

    # arm ACK precedes the armed heartbeat by ~1s; verify before takeoff.
    deadline = time.monotonic() + 10.0
    while time.monotonic() < deadline and not _status().get("armed"):
        time.sleep(0.5)
    assert _status().get("armed") is True, "vehicle did not report armed after arm"

    ok(call_daemon("takeoff", {"alt": 10.0, "confirm": True, "wait": True, "timeout": 45},
                   timeout=75), "takeoff")

    rel_alt = _status().get("relative_alt_m")
    assert isinstance(rel_alt, (int, float)) and rel_alt >= 9.0, f"expected ~10m, got {rel_alt}"

    ok(call_daemon("rtl", {"confirm": True, "wait": True, "timeout": 120}, timeout=150), "rtl")

    final = _status()
    assert final.get("armed") in (False, None), "vehicle should be disarmed after RTL/land"
