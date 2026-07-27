"""Unit tests for the safety-guard framework."""

from __future__ import annotations

from mavctl.daemon import guards
from mavctl.daemon.guards import GuardConfig
from mavctl.models import Battery, ExitCode, GpsInfo, VehicleState

_CFG = GuardConfig()
_MODES = ["GUIDED", "LOITER", "RTL", "LAND", "STABILIZE"]


def _state(**kw: object) -> VehicleState:
    base: dict[str, object] = {
        "connected": True,
        "flight_mode": "GUIDED",
        "armed": False,
        "gps": GpsInfo(fix_type=6, fix_label="rtk_fixed"),
        "battery": Battery(voltage_v=12.6),
    }
    base.update(kw)
    return VehicleState(**base)  # type: ignore[arg-type]


# -- confirm gate ----------------------------------------------------------


def test_arm_without_confirm_rejected() -> None:
    d = guards.check_arm(_state(), confirm=False, config=_CFG)
    assert d.allowed is False
    assert d.reason == "confirmation_required"
    assert d.exit_code == ExitCode.SAFETY_REJECTED
    assert d.hint  # must guide the agent


def test_every_command_requires_confirm() -> None:
    st = _state(armed=True, relative_alt_m=0.0)
    assert guards.check_disarm(st, confirm=False, force=False, config=_CFG).allowed is False
    assert guards.check_mode(st, "LOITER", _MODES, confirm=False).allowed is False
    assert guards.check_takeoff(st, 10.0, confirm=False, config=_CFG).allowed is False
    assert guards.check_land(st, confirm=False).allowed is False
    assert guards.check_rtl(st, confirm=False).allowed is False


# -- arm -------------------------------------------------------------------


def test_arm_allowed_with_good_gps() -> None:
    d = guards.check_arm(_state(), confirm=True, config=_CFG)
    assert d.allowed is True
    assert d.already_satisfied is False


def test_arm_rejected_on_poor_gps() -> None:
    poor = _state(gps=GpsInfo(fix_type=1, fix_label="no_fix"))
    d = guards.check_arm(poor, confirm=True, config=_CFG)
    assert d.allowed is False
    assert d.reason == "gps_not_ready"


def test_arm_idempotent_when_already_armed() -> None:
    d = guards.check_arm(_state(armed=True), confirm=True, config=_CFG)
    assert d.allowed is True
    assert d.already_satisfied is True
    assert d.note == "already armed"


# -- disarm ----------------------------------------------------------------


def test_disarm_rejected_in_flight() -> None:
    d = guards.check_disarm(
        _state(armed=True, relative_alt_m=15.0), confirm=True, force=False, config=_CFG
    )
    assert d.allowed is False
    assert d.reason == "in_flight"


def test_disarm_in_flight_allowed_with_force() -> None:
    d = guards.check_disarm(
        _state(armed=True, relative_alt_m=15.0), confirm=True, force=True, config=_CFG
    )
    assert d.allowed is True


def test_disarm_idempotent_when_disarmed() -> None:
    d = guards.check_disarm(_state(armed=False), confirm=True, force=False, config=_CFG)
    assert d.allowed is True
    assert d.already_satisfied is True


# -- mode ------------------------------------------------------------------


def test_mode_unknown_is_usage_error() -> None:
    d = guards.check_mode(_state(), "WARP", _MODES, confirm=True)
    assert d.allowed is False
    assert d.reason == "unknown_mode"
    assert d.exit_code == ExitCode.USAGE_ERROR


def test_mode_idempotent_when_already_in_mode() -> None:
    d = guards.check_mode(_state(flight_mode="GUIDED"), "GUIDED", _MODES, confirm=True)
    assert d.allowed is True
    assert d.already_satisfied is True


def test_mode_switch_allowed() -> None:
    d = guards.check_mode(_state(flight_mode="GUIDED"), "LOITER", _MODES, confirm=True)
    assert d.allowed is True
    assert d.already_satisfied is False


# -- takeoff ---------------------------------------------------------------


def test_takeoff_requires_positive_alt() -> None:
    d = guards.check_takeoff(_state(armed=True), 0.0, confirm=True, config=_CFG)
    assert d.allowed is False
    assert d.reason == "invalid_altitude"
    assert d.exit_code == ExitCode.USAGE_ERROR


def test_takeoff_altitude_limit() -> None:
    d = guards.check_takeoff(_state(armed=True), 999.0, confirm=True, config=_CFG)
    assert d.allowed is False
    assert d.reason == "altitude_limit"
    assert d.exit_code == ExitCode.SAFETY_REJECTED


def test_takeoff_requires_guided_mode() -> None:
    d = guards.check_takeoff(
        _state(flight_mode="STABILIZE", armed=True), 10.0, confirm=True, config=_CFG
    )
    assert d.allowed is False
    assert d.reason == "wrong_mode"
    assert "GUIDED" in (d.hint or "")


def test_takeoff_requires_armed() -> None:
    d = guards.check_takeoff(
        _state(flight_mode="GUIDED", armed=False), 10.0, confirm=True, config=_CFG
    )
    assert d.allowed is False
    assert d.reason == "not_armed"
    assert "arm" in (d.hint or "")


def test_takeoff_allowed_when_guided_and_armed() -> None:
    d = guards.check_takeoff(
        _state(flight_mode="GUIDED", armed=True), 10.0, confirm=True, config=_CFG
    )
    assert d.allowed is True


# -- land / rtl ------------------------------------------------------------


def test_land_allowed_in_flight() -> None:
    d = guards.check_land(_state(armed=True, relative_alt_m=10.0), confirm=True)
    assert d.allowed is True
    assert d.already_satisfied is False


def test_land_idempotent_on_ground() -> None:
    d = guards.check_land(_state(armed=False), confirm=True)
    assert d.allowed is True
    assert d.already_satisfied is True


def test_rtl_allowed_in_flight() -> None:
    d = guards.check_rtl(_state(armed=True, relative_alt_m=10.0), confirm=True)
    assert d.allowed is True
