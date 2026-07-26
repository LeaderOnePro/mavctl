"""SITL integration test.

Requires a running ArduPilot SITL streaming to ``udp:127.0.0.1:14550``:

    sim_vehicle.py -v ArduCopter --out udp:127.0.0.1:14550

Run explicitly with:  pytest -m sitl
Skipped by default via:  pytest -m "not sitl"
"""

from __future__ import annotations

import os
import shutil
import time

import pytest

from mavctl.daemon import process
from mavctl.daemon.client import call_daemon

pytestmark = pytest.mark.sitl

_CONNECT = os.environ.get("MAVCTL_SITL_CONNECT", "udp:127.0.0.1:14550")


def test_end_to_end_link_against_sitl(monkeypatch: pytest.MonkeyPatch) -> None:
    # AF_UNIX paths are capped near 104 bytes on macOS, so keep MAVCTL_HOME
    # short rather than under pytest's deep tmp dir.
    home = f"/tmp/mavctl_sitl_{os.getpid()}"
    monkeypatch.setenv("MAVCTL_HOME", home)
    try:
        assert not process.is_running()
        process.spawn(_CONNECT, heartbeat_timeout=3.0)
        try:
            # Give the link a moment to receive the first heartbeats/telemetry.
            deadline = time.monotonic() + 15.0
            state = call_daemon("status").result or {}
            while time.monotonic() < deadline and not state.get("connected"):
                time.sleep(0.5)
                state = call_daemon("status").result or {}

            assert state.get("connected") is True, "no heartbeat from SITL within 15s"
            assert state.get("flight_mode") is not None

            telemetry = call_daemon("telemetry")
            assert telemetry.ok is True
            assert telemetry.result is not None
            assert telemetry.result["position"]["lat_deg"] is not None
        finally:
            process.stop()

        assert not process.is_running()
    finally:
        shutil.rmtree(home, ignore_errors=True)
