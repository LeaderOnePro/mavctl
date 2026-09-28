"""Guard tests for mission upload / clear (Phase 3A)."""

from __future__ import annotations

import pytest

from mavctl.daemon import guards
from mavctl.daemon.guards import GuardConfig
from mavctl.models import Battery, ExitCode, GpsInfo, MissionV1, VehicleState

_CFG = GuardConfig()


def _mission() -> MissionV1:
    return MissionV1.model_validate(
        {
            "version": 1,
            "items": [
                {"type": "takeoff", "altitude_m": 10.0},
                {"type": "waypoint", "lat_deg": 1.0, "lon_deg": 2.0, "altitude_m": 15.0},
                {"type": "rtl"},
            ],
        }
    )


def _state(**kw: object) -> VehicleState:
    base: dict[str, object] = {
        "connected": True,
        "heartbeat_age_s": 0.2,
        "flight_mode": "STABILIZE",
        "armed": False,
        "gps": GpsInfo(fix_type=6, fix_label="rtk_fixed"),
        "battery": Battery(voltage_v=12.6),
        "landed_state": "on_ground",
        "relative_alt_m": 0.0,
        "telemetry_age_s": 0.2,
        "landed_state_age_s": 0.2,
    }
    base.update(kw)
    return VehicleState(**base)  # type: ignore[arg-type]


def test_upload_without_confirm_rejected() -> None:
    d = guards.check_mission_upload(_state(), _mission(), confirm=False, config=_CFG)
    assert d.allowed is False
    assert d.reason == "confirmation_required"
    assert d.exit_code == ExitCode.SAFETY_REJECTED


def test_clear_without_confirm_rejected() -> None:
    d = guards.check_mission_clear(_state(), confirm=False, config=_CFG)
    assert d.allowed is False
    assert d.reason == "confirmation_required"


@pytest.mark.parametrize("armed", [True, None])
def test_upload_requires_explicitly_disarmed(armed: bool | None) -> None:
    # armed=None (unknown) must NOT be treated as safely disarmed.
    d = guards.check_mission_upload(
        _state(armed=armed), _mission(), confirm=True, config=_CFG
    )
    assert d.allowed is False
    assert d.reason == "mission_requires_disarmed"
    assert d.exit_code == ExitCode.SAFETY_REJECTED
    assert d.checks[-1].detail == f"armed={armed}"


def test_clear_requires_explicitly_disarmed() -> None:
    d = guards.check_mission_clear(_state(armed=True), confirm=True, config=_CFG)
    assert d.allowed is False
    assert d.reason == "mission_requires_disarmed"


def test_upload_stale_heartbeat_rejected_exit_4() -> None:
    d = guards.check_mission_upload(
        _state(connected=False, armed=False, flight_mode=None, heartbeat_age_s=None),
        _mission(),
        confirm=True,
        config=_CFG,
    )
    assert d.allowed is False
    assert d.reason == "not_connected"
    assert d.exit_code == ExitCode.VEHICLE_NOT_CONNECTED


def test_upload_airborne_rejected_in_flight() -> None:
    d = guards.check_mission_upload(
        _state(armed=False, landed_state="in_air", relative_alt_m=15.0),
        _mission(),
        confirm=True,
        config=_CFG,
    )
    assert d.allowed is False
    assert d.reason == "in_flight"


def test_clear_airborne_rejected_in_flight() -> None:
    d = guards.check_mission_clear(
        _state(landed_state=None, relative_alt_m=15.0), confirm=True, config=_CFG
    )
    assert d.allowed is False
    assert d.reason == "in_flight"


def test_upload_stale_ground_evidence_rejected() -> None:
    # both evidence streams stale: neither landed_state nor low telemetry is fresh
    d = guards.check_mission_upload(
        _state(landed_state_age_s=99.0, telemetry_age_s=99.0),
        _mission(),
        confirm=True,
        config=_CFG,
    )
    assert d.allowed is False
    assert d.reason == "ground_state_stale"
    assert d.exit_code == ExitCode.SAFETY_REJECTED


def test_clear_stale_ground_evidence_rejected() -> None:
    d = guards.check_mission_clear(
        _state(telemetry_age_s=99.0, landed_state=None, relative_alt_m=0.3),
        confirm=True,
        config=_CFG,
    )
    assert d.allowed is False
    assert d.reason == "ground_state_stale"


def test_upload_unknown_ground_state_rejected() -> None:
    d = guards.check_mission_upload(
        _state(landed_state=None, relative_alt_m=None, telemetry_age_s=None,
               landed_state_age_s=None),
        _mission(),
        confirm=True,
        config=_CFG,
    )
    assert d.allowed is False
    assert d.reason == "ground_state_unknown"


def test_upload_grounded_fresh_allowed() -> None:
    d = guards.check_mission_upload(_state(), _mission(), confirm=True, config=_CFG)
    assert d.allowed is True


def test_upload_low_fresh_altitude_allowed_without_landed_state() -> None:
    d = guards.check_mission_upload(
        _state(landed_state=None, relative_alt_m=0.3),
        _mission(),
        confirm=True,
        config=_CFG,
    )
    assert d.allowed is True


def test_clear_grounded_fresh_allowed() -> None:
    d = guards.check_mission_clear(_state(), confirm=True, config=_CFG)
    assert d.allowed is True


def test_upload_altitude_ceiling_rejected() -> None:
    mission = MissionV1.model_validate(
        {
            "version": 1,
            "items": [
                {"type": "takeoff", "altitude_m": 10.0},
                {"type": "waypoint", "lat_deg": 1.0, "lon_deg": 2.0,
                 "altitude_m": _CFG.max_takeoff_alt_m + 10.0},
                {"type": "rtl"},
            ],
        }
    )
    d = guards.check_mission_upload(_state(), mission, confirm=True, config=_CFG)
    assert d.allowed is False
    assert d.reason == "altitude_limit"
    assert d.exit_code == ExitCode.SAFETY_REJECTED
    assert "1" in (d.checks[-1].detail)  # item index 1


def test_upload_altitude_at_ceiling_allowed() -> None:
    mission = MissionV1.model_validate(
        {
            "version": 1,
            "items": [
                {"type": "takeoff", "altitude_m": 10.0},
                {"type": "waypoint", "lat_deg": 1.0, "lon_deg": 2.0,
                 "altitude_m": _CFG.max_takeoff_alt_m},
                {"type": "rtl"},
            ],
        }
    )
    d = guards.check_mission_upload(_state(), mission, confirm=True, config=_CFG)
    assert d.allowed is True


def test_clear_unknown_ground_state_rejected() -> None:
    d = guards.check_mission_clear(
        _state(landed_state=None, relative_alt_m=None, telemetry_age_s=None,
               landed_state_age_s=None),
        confirm=True,
        config=_CFG,
    )
    assert d.allowed is False
    assert d.reason == "ground_state_unknown"
