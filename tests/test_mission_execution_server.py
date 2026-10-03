"""Phase 3B-1 daemon tests: `mission start` operation integration.

Uses the FakeAdapter (mission count scriptable via ``mission_count``, mission
execution state controllable via the vehicle snapshot) and deterministic
async coordination — no sleeps-as-synchronization, no MAVLink endpoint.
Milestone semantics per docs/design/mission-execution-phase3b.md §C.2:
mission_state ACTIVE + flight mode AUTO.
"""

from __future__ import annotations

import asyncio
import contextlib

from mavctl.daemon import wire
from mavctl.daemon.server import DaemonServer
from mavctl.models import (
    CommandOutcome,
    ExitCode,
    GpsInfo,
    MissionExecutionState,
    VehicleState,
)
from mavctl.models.operation import OperationKind, OperationState
from tests.test_server import FakeAdapter


def _state(
    *,
    armed: bool = True,
    mode: str = "GUIDED",
    connected: bool = True,
    mission: MissionExecutionState | None = None,
) -> VehicleState:
    return VehicleState(
        connected=connected,
        heartbeat_age_s=0.2 if connected else None,
        flight_mode=mode,
        armed=armed,
        relative_alt_m=0.0,
        gps=GpsInfo(fix_type=6, fix_label="rtk_fixed"),
        mission=mission if mission is not None else MissionExecutionState(),
    )


class _MissionStartAdapter(FakeAdapter):
    """FakeAdapter with a controllable mission count and a stalled start
    observation (the vehicle never reports mission ACTIVE until released)."""

    def __init__(self, state: VehicleState, *, mission_count: int = 4) -> None:
        super().__init__(state, mission_count=mission_count)
        self.release_mission_active = False

    def start_mission(self) -> CommandOutcome:
        self.calls.append("start_mission")
        return CommandOutcome.from_ack(0, 1)


def _server(
    *,
    armed: bool = True,
    mode: str = "GUIDED",
    mission_count: int = 4,
    mission: MissionExecutionState | None = None,
) -> tuple[DatumServer, _MissionStartAdapter]:
    adapter = _MissionStartAdapter(
        _state(armed=armed, mode=mode, mission=mission),
        mission_count=mission_count,
    )
    server = DaemonServer(adapter, "udp:127.0.0.1:14550")
    return server, adapter


DatumServer = DaemonServer


def _start_params(**overrides: object) -> bytes:
    params: dict[str, object] = {"confirm": True}
    params.update(overrides)
    return wire.encode({"method": "mission_start", "params": params})


def _await_reached(
    server: DatumServer,
    operation_id: str,
    *,
    polls: int = 50,
) -> OperationState:
    """Poll the registry until the operation reaches a milestone."""

    import time as _time

    operation = server.operations.get(operation_id)
    for _ in range(polls):
        assert operation is not None
        if operation.state in (
            OperationState.REACHED,
            OperationState.SUPERSEDED,
            OperationState.LINK_LOST,
            OperationState.UNCERTAIN,
            OperationState.FAILED,
        ):
            return operation.state
        operation = server.operations.get(operation_id)
        _time.sleep(0.05)
    assert operation is not None
    return operation.state


# -- acceptance / registration ---------------------------------------------------


async def test_mission_start_accepted_registers_operation() -> None:
    server, adapter = _server()
    response = await server._dispatch(_start_params())

    assert response.ok is True
    result = response.result
    assert result is not None
    assert result["action"] == "mission_start"
    operation_id = result["operation_id"]
    assert operation_id.startswith("op-")
    operation = server.operations.get(operation_id)
    assert operation is not None
    assert operation.kind is OperationKind.MISSION_START
    assert operation.state is OperationState.WAITING
    assert "start_mission" in adapter.calls


async def test_mission_start_without_wait_reaches_in_background() -> None:
    server, adapter = _server()
    response = await server._dispatch(_start_params())
    result = response.result
    assert result is not None
    operation_id = result["operation_id"]

    # vehicle enters AUTO and reports mission ACTIVE
    adapter._state = _state(
        mode="AUTO",
        mission=MissionExecutionState(
            current_seq=1, total=4, state="active", mode="mission", age_s=0.1
        ),
    )
    operation = server.operations.get(operation_id)
    for _ in range(50):
        assert operation is not None
        if operation.state is OperationState.REACHED:
            break
        await asyncio.sleep(0.05)
        operation = server.operations.get(operation_id)
    assert operation is not None
    assert operation.state is OperationState.REACHED


async def test_mission_start_wait_milestone_reached() -> None:
    server, adapter = _server()

    async def _become_active() -> None:
        await asyncio.sleep(0.1)
        adapter._state = _state(
            mode="AUTO",
            mission=MissionExecutionState(
                current_seq=1,
                total=4,
                state="active",
                mode="mission",
                age_s=0.05,
            ),
        )

    state_task = asyncio.create_task(_become_active())
    response = await asyncio.wait_for(
        server._dispatch(_start_params(wait=True, timeout=5)), timeout=8.0
    )
    await state_task

    assert response.ok is True
    result = response.result
    assert result is not None
    assert result["waited"] is True
    operation = server.operations.get(result["operation_id"])
    assert operation is not None
    assert operation.state is OperationState.REACHED


# -- client timeout / supersession / link loss ------------------------------------


async def test_mission_start_wait_timeout_operation_continues() -> None:
    server, _adapter = _server()

    response = await asyncio.wait_for(
        server._dispatch(_start_params(wait=True, timeout=1)), timeout=10.0
    )
    assert response.ok is False
    error = response.error
    assert error is not None
    assert error.code == ExitCode.NACK_TIMEOUT
    assert error.detail["reason"] == "operation_wait_timeout"
    assert error.detail["operation_still_running"] is True
    operation = server.operations.get(error.detail["operation_id"])
    assert operation is not None
    assert operation.state is OperationState.WAITING  # still observing


async def test_mission_start_superseded_by_rtl() -> None:
    server, _adapter = _server()
    waiting = asyncio.create_task(
        server._dispatch(_start_params(wait=True, timeout=30))
    )
    await asyncio.sleep(0.15)  # ACK + registration; lock released

    rtl = await asyncio.wait_for(
        server._dispatch(wire.encode({"method": "rtl", "params": {"confirm": True}})),
        timeout=5.0,
    )
    assert rtl.ok is True
    rtl_result = rtl.result
    assert rtl_result is not None
    rtl_id = rtl_result["operation_id"]

    r_wait = await asyncio.wait_for(waiting, timeout=5.0)
    assert r_wait.ok is False
    error = r_wait.error
    assert error is not None
    assert error.code == ExitCode.NACK_TIMEOUT
    assert error.detail["reason"] == "operation_superseded"
    assert error.detail["superseded_by_operation_id"] == rtl_id
    assert "re-queried" in error.message

    operation = server.operations.get(error.detail["operation_id"])
    assert operation is not None
    assert operation.state is OperationState.SUPERSEDED


async def test_mission_start_link_lost_during_wait() -> None:
    server, adapter = _server()
    waiting = asyncio.create_task(
        server._dispatch(_start_params(wait=True, timeout=30))
    )
    await asyncio.sleep(0.15)
    adapter._state = _state(connected=False)

    r_wait = await asyncio.wait_for(waiting, timeout=5.0)
    assert r_wait.ok is False
    assert r_wait.error is not None
    assert r_wait.error.code == ExitCode.VEHICLE_NOT_CONNECTED
    assert r_wait.error.detail["reason"] == "link_lost_during_wait"


# -- guards through the daemon -----------------------------------------------------


async def test_mission_start_mission_absent_rejects() -> None:
    server, _adapter = _server(mission_count=0)
    response = await server._dispatch(_start_params())

    assert response.ok is False
    assert response.error is not None
    assert response.error.code == ExitCode.SAFETY_REJECTED
    assert response.error.detail["reason"] == "mission_absent"


async def test_mission_start_count_unverified_rejects() -> None:
    server, adapter = _server()
    from mavctl.adapter.base import MissionProtocolError

    def probe_error() -> int:
        raise MissionProtocolError(
            "vehicle did not answer MISSION_REQUEST_LIST", result_name="TIMEOUT"
        )

    adapter.get_mission_count = probe_error  # type: ignore[method-assign]
    response = await server._dispatch(_start_params())

    assert response.ok is False
    assert response.error is not None
    assert response.error.code == ExitCode.SAFETY_REJECTED
    assert response.error.detail["reason"] == "mission_count_unverified"


async def test_mission_start_unarmed_rejects() -> None:
    server, _adapter = _server(armed=False)
    response = await server._dispatch(_start_params())

    assert response.ok is False
    assert response.error is not None
    assert response.error.code == ExitCode.SAFETY_REJECTED
    assert response.error.detail["reason"] == "mission_requires_armed"


async def test_mission_start_already_running_is_idempotent() -> None:
    server, adapter = _server(
        mode="AUTO",
        mission=MissionExecutionState(
            current_seq=1, total=4, state="active", mode="mission", age_s=0.1
        ),
    )
    calls_before = list(adapter.calls)

    response = await server._dispatch(_start_params())
    assert response.ok is True
    result = response.result
    assert result is not None
    # the shared idempotent shape from _pre_execute
    assert result["already_satisfied"] is True
    assert result["executed"] is False
    assert result["note"] == "already running"
    assert server.operations.active() is None  # no operation created
    # the verified-count probe runs before the idempotent short-circuit —
    # that is the guard's evidence — but no MAV_CMD_MISSION_START is sent
    assert adapter.calls == [*calls_before, "get_mission_count"]


async def test_mission_start_dry_run_creates_no_operation() -> None:
    server, adapter = _server()
    response = await server._dispatch(_start_params(dry_run=True))

    assert response.ok is True
    assert server.operations.active() is None
    assert "start_mission" not in adapter.calls


async def test_mission_start_confirm_required() -> None:
    server, _adapter = _server()
    response = await server._dispatch(_start_params(confirm=False))

    assert response.ok is False
    assert response.error is not None
    assert response.error.code == ExitCode.SAFETY_REJECTED
    assert response.error.detail["reason"] == "confirmation_required"


# -- observation liveness ------------------------------------------------------------


async def test_status_live_during_mission_start_wait() -> None:
    server, _adapter = _server()
    waiting = asyncio.create_task(
        server._dispatch(_start_params(wait=True, timeout=30))
    )
    await asyncio.sleep(0.15)

    status = await asyncio.wait_for(
        server._dispatch(wire.encode({"method": "status"})), timeout=1.0
    )
    assert status.ok is True
    waiting.cancel()
    with contextlib.suppress(asyncio.CancelledError):
        await waiting
