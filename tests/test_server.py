"""Unit tests for the daemon server dispatch, guards wiring, and wire framing."""

from __future__ import annotations

from mavctl.daemon import wire
from mavctl.daemon.server import DaemonServer
from mavctl.models import CommandOutcome, ExitCode, Telemetry, VehicleState


class FakeAdapter:
    """VehicleAdapter double with preset snapshot and scripted command outcomes."""

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
        return self._outcome

    def disarm(self, force: bool = False) -> CommandOutcome:
        self.calls.append(f"disarm(force={force})")
        return self._outcome

    def set_mode(self, mode: str) -> CommandOutcome:
        self.calls.append(f"set_mode({mode})")
        return self._outcome

    def takeoff(self, altitude_m: float) -> CommandOutcome:
        self.calls.append(f"takeoff({altitude_m})")
        return self._outcome

    def land(self) -> CommandOutcome:
        self.calls.append("land")
        return self._outcome

    def rtl(self) -> CommandOutcome:
        self.calls.append("rtl")
        return self._outcome


def _server(
    connected: bool = True,
    *,
    armed: bool = False,
    mode: str = "GUIDED",
    outcome: CommandOutcome | None = None,
    rel_alt: float | None = 0.0,
) -> DaemonServer:
    state = VehicleState(
        connected=connected,
        heartbeat_age_s=0.2 if connected else None,
        flight_mode=mode,
        armed=armed,
        relative_alt_m=rel_alt,
    )
    from mavctl.models import GpsInfo  # local import keeps the fixture compact

    state.gps = GpsInfo(fix_type=6, fix_label="rtk_fixed")
    adapter = FakeAdapter(state, outcome=outcome)
    server = DaemonServer(adapter, "udp:127.0.0.1:14550")
    return server


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
    response = await _server(connected=False)._dispatch(_params(method="arm", confirm=True))
    assert response.ok is False
    assert response.error is not None
    assert response.error.code == ExitCode.VEHICLE_NOT_CONNECTED


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
