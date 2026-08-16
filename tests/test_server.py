"""Unit tests for the daemon server dispatch, guards wiring, and concurrency."""

from __future__ import annotations

import asyncio
import threading

import pytest

from mavctl.daemon import wire
from mavctl.daemon.server import DaemonServer
from mavctl.models import (
    CommandOutcome,
    ExitCode,
    GpsInfo,
    Position,
    Telemetry,
    VehicleState,
)


class FakeAdapter:
    """VehicleAdapter double with a mutable snapshot and scripted outcomes.

    Command verbs mutate the snapshot (arm/disarm/set_mode) or telemetry
    (takeoff) so that --wait predicates can actually resolve.
    """

    def __init__(
        self,
        state: VehicleState,
        telemetry: Telemetry | None = None,
        outcome: CommandOutcome | None = None,
    ) -> None:
        self._state = state
        self._telemetry = telemetry or Telemetry()
        self._outcome = outcome or CommandOutcome.from_ack(0, 1)
        self.calls: list[str] = []

    def connect(self) -> None:
        self.calls.append("connect")

    def disconnect(self) -> None:
        self.calls.append("disconnect")

    def get_state(self) -> VehicleState:
        return self._state

    def get_telemetry(self) -> Telemetry:
        return self._telemetry

    def mode_names(self) -> list[str]:
        return ["GUIDED", "LOITER", "RTL", "LAND", "STABILIZE"]

    def arm(self, force: bool = False) -> CommandOutcome:
        self.calls.append(f"arm(force={force})")
        self._state = self._state.model_copy(update={"armed": True})
        return self._outcome

    def disarm(self, force: bool = False) -> CommandOutcome:
        self.calls.append(f"disarm(force={force})")
        self._state = self._state.model_copy(update={"armed": False})
        return self._outcome

    def set_mode(self, mode: str) -> CommandOutcome:
        self.calls.append(f"set_mode({mode})")
        self._state = self._state.model_copy(update={"flight_mode": mode})
        return self._outcome

    def takeoff(self, altitude_m: float) -> CommandOutcome:
        self.calls.append(f"takeoff({altitude_m})")
        self._telemetry = Telemetry(position=Position(relative_alt_m=altitude_m))
        return self._outcome

    def land(self) -> CommandOutcome:
        self.calls.append("land")
        self._state = self._state.model_copy(update={"armed": False})
        return self._outcome

    def rtl(self) -> CommandOutcome:
        self.calls.append("rtl")
        self._state = self._state.model_copy(update={"armed": False})
        return self._outcome


def _state(connected: bool = True, *, armed: bool = False, mode: str = "GUIDED") -> VehicleState:
    return VehicleState(
        connected=connected,
        heartbeat_age_s=0.2 if connected else None,
        flight_mode=mode,
        armed=armed,
        relative_alt_m=0.0,
        gps=GpsInfo(fix_type=6, fix_label="rtk_fixed"),
    )


def _server(
    connected: bool = True,
    *,
    armed: bool = False,
    mode: str = "GUIDED",
    outcome: CommandOutcome | None = None,
) -> DaemonServer:
    return DaemonServer(
        FakeAdapter(_state(connected, armed=armed, mode=mode), outcome=outcome),
        "udp:127.0.0.1:14550",
    )


def _params(**kw: object) -> bytes:
    return wire.encode({"method": kw.pop("method"), "params": kw})


# -- wire / fast methods ---------------------------------------------------


def test_wire_roundtrip() -> None:
    frame = wire.encode({"method": "status", "params": {}})
    assert frame.endswith(b"\n")
    assert wire.decode(frame) == {"method": "status", "params": {}}


async def test_dispatch_status_always_succeeds() -> None:
    response = await _server(connected=False)._dispatch(wire.encode({"method": "status"}))
    assert response.ok is True
    assert response.result is not None and response.result["connected"] is False


async def test_dispatch_telemetry_requires_connection() -> None:
    response = await _server(connected=False)._dispatch(wire.encode({"method": "telemetry"}))
    assert response.ok is False
    assert response.error is not None
    assert response.error.code == ExitCode.VEHICLE_NOT_CONNECTED


async def test_dispatch_unknown_method() -> None:
    response = await _server()._dispatch(wire.encode({"method": "nope"}))
    assert response.ok is False
    assert response.error is not None
    assert response.error.code == ExitCode.GENERAL_ERROR


# -- command methods -------------------------------------------------------


async def test_arm_without_confirm_rejected_exit_5() -> None:
    response = await _server()._dispatch(_params(method="arm"))
    assert response.ok is False
    assert response.error is not None
    assert response.error.code == ExitCode.SAFETY_REJECTED
    assert response.error.detail["reason"] == "confirmation_required"
    assert response.error.detail["hint"]


async def test_arm_not_connected_exit_4() -> None:
    server = _server(connected=False)
    response = await server._dispatch(_params(method="arm", confirm=True))
    assert response.ok is False
    assert response.error is not None
    assert response.error.code == ExitCode.VEHICLE_NOT_CONNECTED
    # Daemon-path _not_connected (not guard-shaped rejection).
    assert response.error.message == "vehicle not connected (no recent heartbeat)"
    assert response.error.detail == {"connection_string": "udp:127.0.0.1:14550"}
    assert server._adapter.calls == []  # type: ignore[attr-defined]


@pytest.mark.parametrize(
    ("method", "extra"),
    [
        ("arm", {"confirm": True}),
        ("disarm", {"confirm": True}),
        ("mode", {"confirm": True, "mode": "GUIDED"}),
        ("takeoff", {"confirm": True, "alt": 10.0}),
        ("land", {"confirm": True}),
        ("rtl", {"confirm": True}),
    ],
)
async def test_all_dangerous_rpcs_not_connected_exit_4(
    method: str, extra: dict[str, object]
) -> None:
    """Every state-changing RPC rejects disconnect inside the command lock."""

    server = _server(connected=False)
    response = await server._dispatch(_params(method=method, **extra))
    assert response.ok is False
    assert response.error is not None
    assert response.error.code == ExitCode.VEHICLE_NOT_CONNECTED
    assert response.error.message == "vehicle not connected (no recent heartbeat)"
    assert response.error.detail.get("connection_string") == "udp:127.0.0.1:14550"
    # Must not reach guards (no reason/hint) or the adapter.
    assert "reason" not in response.error.detail
    assert "hint" not in response.error.detail
    assert server._adapter.calls == []  # type: ignore[attr-defined]


async def test_arm_confirmed_executes() -> None:
    server = _server()
    response = await server._dispatch(_params(method="arm", confirm=True))
    assert response.ok is True
    assert response.result is not None
    assert response.result["executed"] is True
    assert response.result["outcome"]["result_name"] == "ACCEPTED"


async def test_arm_dry_run_does_not_execute() -> None:
    server = _server()
    response = await server._dispatch(_params(method="arm", confirm=True, dry_run=True))
    assert response.ok is True
    assert response.result is not None
    assert response.result["dry_run"] is True
    assert response.result["would_execute"] is True
    assert "arm(force=False)" not in server._adapter.calls  # type: ignore[attr-defined]


async def test_arm_idempotent_when_already_armed() -> None:
    server = _server(armed=True)
    response = await server._dispatch(_params(method="arm", confirm=True))
    assert response.ok is True
    assert response.result is not None
    assert response.result["already_satisfied"] is True
    assert response.result["executed"] is False


async def test_arm_nack_maps_exit_6() -> None:
    server = _server(outcome=CommandOutcome.from_ack(4, 1))  # FAILED
    response = await server._dispatch(_params(method="arm", confirm=True))
    assert response.ok is False
    assert response.error is not None
    assert response.error.code == ExitCode.NACK_TIMEOUT


async def test_takeoff_missing_alt_is_usage_error() -> None:
    server = _server(armed=True)
    response = await server._dispatch(_params(method="takeoff", confirm=True))
    assert response.ok is False
    assert response.error is not None
    assert response.error.code == ExitCode.USAGE_ERROR


async def test_takeoff_wrong_mode_rejected() -> None:
    server = _server(mode="STABILIZE", armed=True)
    response = await server._dispatch(_params(method="takeoff", confirm=True, alt=10.0))
    assert response.ok is False
    assert response.error is not None
    assert response.error.code == ExitCode.SAFETY_REJECTED
    assert response.error.detail["reason"] == "wrong_mode"


async def test_mode_dry_run_reports_checks() -> None:
    server = _server(mode="GUIDED")
    response = await server._dispatch(
        _params(method="mode", mode="LOITER", confirm=True, dry_run=True)
    )
    assert response.ok is True
    assert response.result is not None
    assert response.result["would_execute"] is True
    assert any(c["name"] == "mode_known" for c in response.result["checks"])


# -- IN_PROGRESS semantics (P1) --------------------------------------------


async def test_in_progress_is_accepted_not_nack() -> None:
    server = _server(outcome=CommandOutcome.from_ack(5, 1))  # IN_PROGRESS
    response = await server._dispatch(_params(method="arm", confirm=True))
    assert response.ok is True
    assert response.result is not None
    assert response.result["executed"] is True
    assert response.result["outcome"]["result_name"] == "IN_PROGRESS"


async def test_in_progress_with_wait_continues_to_target() -> None:
    server = _server(mode="GUIDED", outcome=CommandOutcome.from_ack(5, 1))
    response = await server._dispatch(
        _params(method="mode", mode="LOITER", confirm=True, wait=True, timeout=5)
    )
    assert response.ok is True
    assert response.result is not None
    assert response.result["waited"] is True
    assert response.result["outcome"]["result_name"] == "IN_PROGRESS"


async def test_denied_still_exit_6() -> None:
    server = _server(outcome=CommandOutcome.from_ack(2, 1))  # DENIED
    response = await server._dispatch(_params(method="arm", confirm=True))
    assert response.ok is False
    assert response.error is not None
    assert response.error.code == ExitCode.NACK_TIMEOUT
    assert response.error.detail["outcome"]["result_name"] == "DENIED"


# -- --wait link loss (P1) -------------------------------------------------


class _LinkDropAdapter(FakeAdapter):
    """Connected for the guard read, then drops the link during --wait."""

    def __init__(self, state: VehicleState, outcome: CommandOutcome) -> None:
        super().__init__(state, outcome=outcome)
        self._reads = 0

    def get_state(self) -> VehicleState:
        self._reads += 1
        if self._reads <= 1:
            return self._state  # first read (guard) sees a live link
        return self._state.model_copy(update={"connected": False, "heartbeat_age_s": 9.0})


async def test_wait_link_loss_returns_exit_4_not_timeout() -> None:
    adapter = _LinkDropAdapter(_state(armed=True, mode="GUIDED"), CommandOutcome.from_ack(0, 1))
    server = DaemonServer(adapter, "udp:127.0.0.1:14550")
    response = await server._dispatch(
        _params(method="takeoff", confirm=True, alt=10.0, wait=True, timeout=5)
    )
    assert response.ok is False
    assert response.error is not None
    assert response.error.code == ExitCode.VEHICLE_NOT_CONNECTED  # 4, not 6
    assert response.error.detail["reason"] == "link_lost_during_wait"


# -- command lock: serialization + concurrent status (P0) ------------------


class _BlockingArmAdapter(FakeAdapter):
    """arm() blocks until released, to hold the command lock deterministically."""

    def __init__(self, state: VehicleState) -> None:
        super().__init__(state)
        self.started = threading.Event()
        self.release = threading.Event()

    def arm(self, force: bool = False) -> CommandOutcome:
        self.started.set()
        self.release.wait(5.0)
        return CommandOutcome.from_ack(0, 1)


async def test_command_lock_serializes_and_status_stays_live() -> None:
    adapter = _BlockingArmAdapter(_state(armed=False))
    server = DaemonServer(adapter, "udp:127.0.0.1:14550")
    loop = asyncio.get_running_loop()

    first = asyncio.create_task(server._dispatch(_params(method="arm", confirm=True)))
    # Wait (off the event loop) until arm has entered the executor and holds the lock.
    await loop.run_in_executor(None, adapter.started.wait, 5.0)

    # status must still return promptly while the command holds the lock.
    status = await asyncio.wait_for(
        server._dispatch(wire.encode({"method": "status"})), timeout=2.0
    )
    assert status.ok is True

    # A second state-changing command must block on the command lock.
    second = asyncio.create_task(server._dispatch(_params(method="arm", confirm=True)))
    await asyncio.sleep(0.2)
    assert not second.done(), "second command ran while the first held the lock"

    adapter.release.set()
    r1 = await asyncio.wait_for(first, timeout=5.0)
    r2 = await asyncio.wait_for(second, timeout=5.0)
    assert r1.ok is True
    assert r2.ok is True


class _NeverReachesAltAdapter(FakeAdapter):
    """takeoff ACKs but altitude never reaches the target (holds --wait)."""

    def takeoff(self, altitude_m: float) -> CommandOutcome:
        self.calls.append(f"takeoff({altitude_m})")
        # Leave relative_alt at 0 so --wait keeps polling.
        return CommandOutcome.from_ack(0, 1)


async def test_status_stays_live_during_wait_and_second_command_blocks() -> None:
    """--wait stays inside the command lock; status must not block."""

    adapter = _NeverReachesAltAdapter(_state(armed=True, mode="GUIDED"))
    server = DaemonServer(adapter, "udp:127.0.0.1:14550")

    waiting = asyncio.create_task(
        server._dispatch(
            _params(method="takeoff", confirm=True, alt=10.0, wait=True, timeout=3)
        )
    )
    # Let the takeoff enter the --wait poll loop under the lock.
    await asyncio.sleep(0.15)
    assert not waiting.done()

    status = await asyncio.wait_for(
        server._dispatch(wire.encode({"method": "status"})), timeout=1.0
    )
    assert status.ok is True

    second = asyncio.create_task(server._dispatch(_params(method="arm", confirm=True)))
    await asyncio.sleep(0.15)
    assert not second.done(), "second command ran during takeoff --wait"

    # Waiting takeoff times out (exit 6); only then may the second command run.
    r_wait = await asyncio.wait_for(waiting, timeout=5.0)
    assert r_wait.ok is False
    assert r_wait.error is not None
    assert r_wait.error.code == ExitCode.NACK_TIMEOUT

    r2 = await asyncio.wait_for(second, timeout=5.0)
    # arm is already satisfied (adapter started armed) → success no-op, or executes.
    assert r2.ok is True
