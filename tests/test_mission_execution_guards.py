"""Guard tests for `mission start` (Phase 3B-1).

`mission_count` is the vehicle-verified count the daemon obtains via the
controlled `MISSION_REQUEST_LIST` probe before the guard runs; these tests
exercise the decision chain deterministically.
"""

from __future__ import annotations

import pytest

from mavctl.daemon.guards import GuardConfig, GuardDecision, check_mission_start
from mavctl.models import MissionExecutionState, VehicleState


def _state(
    *,
    armed: bool = True,
    mode: str = "GUIDED",
    mission_state: str | None = None,
    connected: bool = True,
) -> VehicleState:
    return VehicleState(
        connected=connected,
        heartbeat_age_s=0.2 if connected else None,
        flight_mode=mode,
        armed=armed,
        gps=__import__("mavctl.models", fromlist=["GpsInfo"]).GpsInfo(
            fix_type=6, fix_label="rtk_fixed"
        ),
        mission=MissionExecutionState(
            current_seq=None if mission_state is None else 1,
            total=None if mission_state is None else 4,
            state=mission_state,
            mode="mission" if mission_state == "active" else None,
            age_s=None if mission_state is None else 0.3,
        ),
    )


def _run(
    state: VehicleState,
    *,
    mission_count: int = 4,
    confirm: bool = True,
) -> GuardDecision:
    return check_mission_start(
        state, mission_count=mission_count, confirm=confirm, config=GuardConfig()
    )


def test_confirm_required() -> None:
    decision = _run(_state(), confirm=False)
    assert decision.allowed is False
    assert decision.reason == "confirmation_required"
    assert decision.exit_code == 5


def test_fresh_link_required() -> None:
    decision = _run(_state(connected=False))
    assert decision.allowed is False
    assert decision.reason == "not_connected"
    # link freshness is a precondition, not a safety rejection: exit 4
    assert decision.exit_code == 4


def test_mission_absent_rejects() -> None:
    decision = _run(_state(), mission_count=0)
    assert decision.allowed is False
    assert decision.reason == "mission_absent"
    assert decision.exit_code == 5
    assert any(c.name == "mission_present" and not c.passed for c in decision.checks)


def test_unarmed_rejects_mission_requires_armed() -> None:
    decision = _run(_state(armed=False), mission_count=4)
    assert decision.allowed is False
    assert decision.reason == "mission_requires_armed"
    assert decision.exit_code == 5


def test_armed_unknown_rejects() -> None:
    state = _state()
    state.armed = None
    decision = _run(state, mission_count=4)
    assert decision.allowed is False
    assert decision.reason == "mission_requires_armed"


def test_auto_active_is_idempotent_already_running() -> None:
    decision = _run(_state(armed=True, mode="AUTO", mission_state="active"))
    assert decision.allowed is True
    assert decision.already_satisfied is True
    assert decision.note == "already running"
    assert any(c.name == "already_running" and c.passed for c in decision.checks)


def test_auto_not_active_is_allowed_pending_sitl() -> None:
    """[DECIDED] v1 policy (§C.1.2): AUTO + non-ACTIVE is allowed —
    ArduCopter's MAV_CMD_MISSION_START handler calls start_or_resume() when
    not RUNNING (source-verified). SITL verification pending."""
    decision = _run(_state(armed=True, mode="AUTO", mission_state="not_started"))
    assert decision.allowed is True
    assert decision.already_satisfied is False


def test_non_auto_mode_is_allowed_command_transitions_to_auto() -> None:
    """The command itself may transition the vehicle to AUTO — that is
    explicit, confirmation-gated execution behavior, not a hidden side
    effect (Phase 3B design §C.1)."""
    decision = _run(_state(armed=True, mode="GUIDED"))
    assert decision.allowed is True


def test_mission_state_unknown_still_allows_start() -> None:
    """mission state unknown (no MISSION_CURRENT yet) must not block the
    start — the vehicle's own idempotent handling covers the already-running
    case; the observation milestone provides the outcome."""
    decision = _run(_state(armed=True, mode="GUIDED", mission_state=None))
    assert decision.allowed is True


@pytest.mark.parametrize("count", [-1, 0])
def test_non_positive_verified_count_rejects(count: int) -> None:
    decision = _run(_state(), mission_count=count)
    assert decision.allowed is False
    assert decision.reason == "mission_absent"
    assert decision.exit_code == 5


def test_dry_run_shape_is_guard_only() -> None:
    """The guard is pure: no adapter, no wire, no operation — the caller
    decides what a dry-run renders."""
    decision = _run(_state(), mission_count=4)
    assert decision.allowed is True
    assert all(c.passed for c in decision.checks)
