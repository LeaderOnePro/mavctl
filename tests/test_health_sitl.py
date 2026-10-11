"""Health observation conformance against a live ArduCopter SITL (Issue #4,
observe-only v1).

Safety contract for this module:

- loopback endpoint only (enforced at import);
- **no state-changing command is ever sent**: the only traffic is the
  daemon's own telemetry/stream setup plus `status` / `telemetry` queries;
  no arm, no mission, no `MAV_CMD_RUN_PREARM_CHECKS`;
- the assertions treat missing health data as `unknown`, never as a
  failure of the vehicle — but the estimator stream request is expected
  to work, so a fresh `observed` verdict must appear within the poll
  window on a healthy SITL vehicle;
- foreign-source filtering is covered by mock tests
  (`tests/test_health_observation.py`); it cannot be exercised here
  without injecting traffic.

Requires an isolated no-MAVProxy SITL instance:

    sim_vehicle.py -v ArduCopter --instance 1 --no-mavproxy --no-rebuild

Endpoint: MAVCTL_SITL_CONNECT (default tcp:127.0.0.1:5770).
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

pytestmark = pytest.mark.sitl

_CONNECT = os.environ.get("MAVCTL_SITL_CONNECT", "tcp:127.0.0.1:5770")

if not (
    _CONNECT.startswith("udp:127.0.0.1")
    or _CONNECT.startswith("tcp:127.0.0.1")
    or _CONNECT.startswith("udp:localhost")
):
    raise RuntimeError(
        f"health SITL tests require a loopback endpoint, got {_CONNECT!r}"
    )


def _status() -> dict[str, Any]:
    response = call_daemon("status", timeout=30)
    assert response.ok is True, f"status failed: {response.error}"
    return response.result or {}


def _await(predicate: Any, timeout: float, fail_msg: str) -> dict[str, Any]:
    deadline = time.monotonic() + timeout
    state = _status()
    while time.monotonic() < deadline:
        if predicate(state):
            return state
        time.sleep(0.25)
        state = _status()
    pytest.fail(f"{fail_msg}: last connected={state.get('connected')}")


@pytest.fixture
def health_daemon(monkeypatch: pytest.MonkeyPatch) -> Iterator[None]:
    home = f"/tmp/mavctl_health_sitl_{os.getpid()}"
    monkeypatch.setenv("MAVCTL_HOME", home)
    if process.is_running():
        process.stop()
    process.spawn(_CONNECT, heartbeat_timeout=3.0, source_system=254)
    try:
        _await(lambda s: bool(s.get("connected")), 30, "no heartbeat from SITL")
        _await(
            lambda s: (s.get("gps") or {}).get("fix_label") in ("rtk_fixed", "dgps", "3d_fix"),
            90,
            "no 3D GPS fix from SITL",
        )
        _await(lambda s: bool(s.get("home_position")), 60, "no HOME_POSITION")
        yield
    finally:
        process.stop()
        shutil.rmtree(home, ignore_errors=True)


# -- health block on a live vehicle ------------------------------------------------


def test_health_estimator_observed_on_live_sitl(health_daemon: None) -> None:
    """The adapter's observation-only EKF stream request yields a fresh
    estimator observation; raw fields are populated and the verdict stays
    inside the observe-only literal set."""

    def _estimator_fresh(state: dict[str, Any]) -> bool:
        estimator = (state.get("health") or {}).get("estimator") or {}
        return (
            estimator.get("status") == "observed"
            and estimator.get("last_status") == "observed"
            and estimator.get("age_s") is not None
            and estimator["age_s"] <= 5.0
        )

    state = _await(_estimator_fresh, 20, "estimator observation never went fresh")
    estimator = state["health"]["estimator"]
    assert isinstance(estimator["flags_mask"], int)
    assert isinstance(estimator["flags"], list)
    assert all(isinstance(name, str) for name in estimator["flags"])
    # EKF_STATUS_REPORT was requested at 2 Hz: the age must be well below
    # the staleness threshold on a healthy link.
    assert estimator["age_s"] <= 2.0
    for variance in (
        "velocity_variance",
        "pos_horiz_variance",
        "pos_vert_variance",
        "compass_variance",
        "terrain_alt_variance",
        "airspeed_variance",
    ):
        assert isinstance(estimator[variance], float), variance


def test_health_json_shape_and_honesty_on_live_sitl(health_daemon: None) -> None:
    state = _status()
    health = state.get("health") or {}
    assert set(health) == {"estimator", "sensors", "prearm"}
    assert health["estimator"]["status"] in ("unknown", "observed")
    assert "healthy" not in str(health).lower()
    assert "prearm_passed" not in str(health)

    # SYS_STATUS streams at 2 Hz on ArduCopter SITL: the raw sensor
    # bitmasks must be populated integers with a bounded age.
    def _sensors_fresh(inner: dict[str, Any]) -> bool:
        return (
            isinstance(inner.get("present"), int)
            and inner.get("age_s") is not None
            and inner["age_s"] <= 5.0
        )

    deadline = time.monotonic() + 15
    sensors = health["sensors"]
    while not _sensors_fresh(sensors) and time.monotonic() < deadline:
        time.sleep(0.25)
        sensors = (_status().get("health") or {}).get("sensors") or {}
    assert isinstance(sensors.get("present"), int)
    assert isinstance(sensors.get("enabled"), int)
    assert isinstance(sensors.get("health"), int)

    # STATUSTEXT: boot diagnostics may or may not include an arming-check
    # prefix on a healthy boot — both are honest outcomes; the field shape
    # is what the contract pins.
    prearm = health["prearm"]
    assert set(prearm) == {"last_message", "severity", "age_s"}


def test_health_unknown_is_honest_when_never_received(health_daemon: None) -> None:
    """A freshly spawned daemon may not have estimator data in its very
    first instants; before the first report the block must be the honest
    never-observed shape (all nulls), never a fabricated verdict."""

    # This is validated at mock level for determinism; here we only assert
    # the live JSON never contains an invented verdict field.
    state = _status()
    dumped = str(state.get("health"))
    assert "healthy" not in dumped
    assert '"status": "healthy"' not in dumped
