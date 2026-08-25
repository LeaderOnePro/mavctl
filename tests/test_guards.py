"""Unit tests for the safety-guard framework."""

from __future__ import annotations

import pytest

from mavctl.daemon import guards
from mavctl.daemon.guards import GuardConfig
from mavctl.models import Battery, ExitCode, GpsInfo, VehicleState

_CFG = GuardConfig()
_MODES = ["GUIDED", "LOITER", "RTL", "LAND", "STABILIZE"]


def _state(**kw: object) -> VehicleState:
    base: dict[str, object] = {
        "connected": True,
        "heartbeat_age_s": 0.2,
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
    assert guards.check_mode(st, "LOITER", _MODES, confirm=False, config=_CFG).allowed is False
    assert guards.check_takeoff(st, 10.0, confirm=False, config=_CFG).allowed is False
    assert guards.check_land(st, confirm=False, config=_CFG).allowed is False
    assert guards.check_rtl(st, confirm=False, config=_CFG).allowed is False


# -- connection gate -------------------------------------------------------


def test_disconnected_rejected_exit_4() -> None:
    d = guards.check_arm(_state(connected=False), confirm=True, config=_CFG)
    assert d.allowed is False
    assert d.reason == "not_connected"
    assert d.exit_code == ExitCode.VEHICLE_NOT_CONNECTED
    assert d.hint


def test_stale_heartbeat_rejected_exit_4() -> None:
    # connected flag stale-but-true: the defensive freshness check still fires.
    d = guards.check_takeoff(
        _state(heartbeat_age_s=10.0, armed=True), 10.0, confirm=True, config=_CFG
    )
    assert d.allowed is False
    assert d.reason == "not_connected"
    assert d.exit_code == ExitCode.VEHICLE_NOT_CONNECTED


def test_missing_heartbeat_rejected() -> None:
    d = guards.check_mode(
        _state(heartbeat_age_s=None), "LOITER", _MODES, confirm=True, config=_CFG
    )
    assert d.allowed is False
    assert d.reason == "not_connected"


def test_confirm_gate_precedes_connection_gate() -> None:
    # Disconnected AND unconfirmed -> confirmation_required wins (order matters).
    d = guards.check_arm(_state(connected=False), confirm=False, config=_CFG)
    assert d.reason == "confirmation_required"


def test_all_commands_reject_when_disconnected() -> None:
    st = _state(connected=False, armed=True)
    reasons = [
        guards.check_arm(st, confirm=True, config=_CFG).reason,
        guards.check_disarm(st, confirm=True, force=False, config=_CFG).reason,
        guards.check_mode(st, "LOITER", _MODES, confirm=True, config=_CFG).reason,
        guards.check_takeoff(st, 10.0, confirm=True, config=_CFG).reason,
        guards.check_land(st, confirm=True, config=_CFG).reason,
        guards.check_rtl(st, confirm=True, config=_CFG).reason,
    ]
    assert all(r == "not_connected" for r in reasons)


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


def test_arm_unknown_armed_is_not_idempotent() -> None:
    # armed=None (unknown) must not short-circuit as already-armed; it proceeds
    # through the preconditions instead (safe side).
    d = guards.check_arm(_state(armed=None), confirm=True, config=_CFG)
    assert d.allowed is True
    assert d.already_satisfied is False


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


def test_disarm_unknown_armed_is_not_idempotent() -> None:
    # armed=None: do not treat as already-disarmed; fall through to the
    # ground-evidence check. A known low altitude still proves "on ground".
    d = guards.check_disarm(
        _state(armed=None, relative_alt_m=0.0), confirm=True, force=False, config=_CFG
    )
    assert d.allowed is True
    assert d.already_satisfied is False


def test_disarm_armed_with_no_ground_evidence_rejected() -> None:
    # armed but neither landed_state nor altitude is known: missing telemetry
    # must never be read as "on the ground" (P0).
    d = guards.check_disarm(
        _state(armed=True, landed_state=None, relative_alt_m=None),
        confirm=True,
        force=False,
        config=_CFG,
    )
    assert d.allowed is False
    assert d.reason == "ground_state_unknown"
    assert d.exit_code == ExitCode.SAFETY_REJECTED
    assert "armed" in (d.message or "")
    assert "--force" in (d.hint or "")


def test_disarm_unknown_armed_and_unknown_alt_rejected() -> None:
    d = guards.check_disarm(
        _state(armed=None, landed_state=None, relative_alt_m=None),
        confirm=True,
        force=False,
        config=_CFG,
    )
    assert d.allowed is False
    assert d.reason == "ground_state_unknown"
    assert d.exit_code == ExitCode.SAFETY_REJECTED


def test_disarm_allowed_on_explicit_on_ground() -> None:
    d = guards.check_disarm(
        _state(armed=True, landed_state="on_ground", relative_alt_m=None),
        confirm=True,
        force=False,
        config=_CFG,
    )
    assert d.allowed is True


def test_disarm_low_known_altitude_boundary() -> None:
    at_ceiling = guards.check_disarm(
        _state(armed=True, relative_alt_m=_CFG.max_on_ground_alt_m),
        confirm=True,
        force=False,
        config=_CFG,
    )
    assert at_ceiling.allowed is True
    just_above = guards.check_disarm(
        _state(armed=True, relative_alt_m=_CFG.max_on_ground_alt_m + 0.1),
        confirm=True,
        force=False,
        config=_CFG,
    )
    assert just_above.allowed is False
    assert just_above.reason == "ground_state_unknown"


def test_disarm_transition_states_are_not_ground() -> None:
    # takeoff/landing transitions prove neither air nor ground -> unknown band.
    for ls in ("takeoff", "landing"):
        d = guards.check_disarm(
            _state(armed=True, landed_state=ls, relative_alt_m=None),
            confirm=True,
            force=False,
            config=_CFG,
        )
        assert d.allowed is False
        assert d.reason == "ground_state_unknown"


def test_disarm_contradictory_ground_report_with_high_alt_rejected() -> None:
    # Air evidence wins over an on-ground report: reject safely.
    d = guards.check_disarm(
        _state(armed=True, landed_state="on_ground", relative_alt_m=15.0),
        confirm=True,
        force=False,
        config=_CFG,
    )
    assert d.allowed is False
    assert d.reason == "in_flight"


def test_disarm_force_allows_and_marks_checks() -> None:
    d = guards.check_disarm(
        _state(armed=True, landed_state=None, relative_alt_m=None),
        confirm=True,
        force=True,
        config=_CFG,
    )
    assert d.allowed is True
    forced = [c for c in d.checks if c.name == "on_ground"]
    assert forced and "forced" in forced[0].detail


def test_disarm_airborne_threshold_is_configurable() -> None:
    cfg = GuardConfig(airborne_alt_threshold_m=5.0)
    # Between the on-ground ceiling and the raised airborne threshold the
    # state is UNKNOWN, not ground: reject rather than allow.
    band = guards.check_disarm(
        _state(armed=True, relative_alt_m=3.0), confirm=True, force=False, config=cfg
    )
    above = guards.check_disarm(
        _state(armed=True, relative_alt_m=6.0), confirm=True, force=False, config=cfg
    )
    assert band.allowed is False
    assert band.reason == "ground_state_unknown"
    assert above.allowed is False and above.reason == "in_flight"


def test_disarm_on_ground_ceiling_is_configurable() -> None:
    # Default ceiling is 0.5 m while the airborne threshold stays at 1 m:
    # 0.7 m sits in the unknown band until the ceiling is raised above it.
    default = guards.check_disarm(
        _state(armed=True, relative_alt_m=0.7), confirm=True, force=False, config=_CFG
    )
    assert default.allowed is False
    assert default.reason == "ground_state_unknown"

    cfg = GuardConfig(max_on_ground_alt_m=0.8)
    raised = guards.check_disarm(
        _state(armed=True, relative_alt_m=0.7), confirm=True, force=False, config=cfg
    )
    assert raised.allowed is True


# -- mode ------------------------------------------------------------------


def test_mode_unknown_is_usage_error() -> None:
    d = guards.check_mode(_state(), "WARP", _MODES, confirm=True, config=_CFG)
    assert d.allowed is False
    assert d.reason == "unknown_mode"
    assert d.exit_code == ExitCode.USAGE_ERROR


def test_mode_idempotent_when_already_in_mode() -> None:
    d = guards.check_mode(_state(flight_mode="GUIDED"), "GUIDED", _MODES, confirm=True, config=_CFG)
    assert d.allowed is True
    assert d.already_satisfied is True


def test_mode_switch_allowed() -> None:
    d = guards.check_mode(_state(flight_mode="GUIDED"), "LOITER", _MODES, confirm=True, config=_CFG)
    assert d.allowed is True
    assert d.already_satisfied is False


def test_mode_map_unavailable_rejected_structurally() -> None:
    # Empty mode map must not degrade into a silent pass-through: the guard
    # rejects with a recoverable safety rejection (exit 5), never an internal
    # error, and the caller must not reach adapter.set_mode.
    d = guards.check_mode(_state(flight_mode="GUIDED"), "LOITER", [], confirm=True, config=_CFG)
    assert d.allowed is False
    assert d.reason == "mode_map_unavailable"
    assert d.exit_code == ExitCode.SAFETY_REJECTED
    assert d.message and "mode mapping" in d.message
    assert d.hint and "status" in d.hint


def test_mode_idempotent_even_when_map_unavailable() -> None:
    # Already in the target mode: nothing would execute, so a missing mode
    # map must not block the no-op.
    d = guards.check_mode(_state(flight_mode="LOITER"), "LOITER", [], confirm=True, config=_CFG)
    assert d.allowed is True
    assert d.already_satisfied is True


# -- takeoff ---------------------------------------------------------------


def test_takeoff_requires_positive_alt() -> None:
    d = guards.check_takeoff(_state(armed=True), 0.0, confirm=True, config=_CFG)
    assert d.allowed is False
    assert d.reason == "invalid_altitude"
    assert d.exit_code == ExitCode.USAGE_ERROR


@pytest.mark.parametrize("bad_alt", [float("nan"), float("inf"), float("-inf")])
def test_takeoff_non_finite_alt_rejected(bad_alt: float) -> None:
    # NaN would slip every comparison; it must be rejected explicitly.
    d = guards.check_takeoff(_state(armed=True), bad_alt, confirm=True, config=_CFG)
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


def test_takeoff_unknown_armed_rejected() -> None:
    # armed=None must not launch: require a positively-confirmed armed state.
    d = guards.check_takeoff(
        _state(flight_mode="GUIDED", armed=None), 10.0, confirm=True, config=_CFG
    )
    assert d.allowed is False
    assert d.reason == "not_armed"


def test_takeoff_allowed_when_guided_and_armed() -> None:
    d = guards.check_takeoff(
        _state(flight_mode="GUIDED", armed=True), 10.0, confirm=True, config=_CFG
    )
    assert d.allowed is True


# -- land / rtl ------------------------------------------------------------


def test_land_allowed_in_flight() -> None:
    d = guards.check_land(_state(armed=True, relative_alt_m=10.0), confirm=True, config=_CFG)
    assert d.allowed is True
    assert d.already_satisfied is False


def test_land_idempotent_on_ground() -> None:
    d = guards.check_land(_state(armed=False), confirm=True, config=_CFG)
    assert d.allowed is True
    assert d.already_satisfied is True


def test_land_unknown_armed_is_not_idempotent() -> None:
    d = guards.check_land(_state(armed=None, landed_state=None), confirm=True, config=_CFG)
    assert d.allowed is True
    assert d.already_satisfied is False


def test_rtl_allowed_in_flight() -> None:
    d = guards.check_rtl(_state(armed=True, relative_alt_m=10.0), confirm=True, config=_CFG)
    assert d.allowed is True
