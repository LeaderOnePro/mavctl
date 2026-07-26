"""Unit tests for the daemon server dispatch and wire framing."""

from __future__ import annotations

from mavctl.daemon import wire
from mavctl.daemon.server import DaemonServer
from mavctl.models import ExitCode, Telemetry, VehicleState


class FakeAdapter:
    """Minimal VehicleAdapter double driven by preset snapshots."""

    def __init__(self, state: VehicleState, telemetry: Telemetry) -> None:
        self._state = state
        self._telemetry = telemetry
        self.connected_called = False
        self.disconnected_called = False

    def connect(self) -> None:
        self.connected_called = True

    def disconnect(self) -> None:
        self.disconnected_called = True

    def get_state(self) -> VehicleState:
        return self._state

    def get_telemetry(self) -> Telemetry:
        return self._telemetry


def _server(connected: bool) -> DaemonServer:
    state = VehicleState(connected=connected, flight_mode="GUIDED", armed=False)
    telemetry = Telemetry()
    return DaemonServer(FakeAdapter(state, telemetry), "udp:127.0.0.1:14550")


def test_wire_roundtrip() -> None:
    frame = wire.encode({"method": "status", "params": {}})
    assert frame.endswith(b"\n")
    assert wire.decode(frame) == {"method": "status", "params": {}}


def test_dispatch_ping() -> None:
    response = _server(connected=True)._dispatch(wire.encode({"method": "ping"}))
    assert response.ok is True
    assert response.result is not None and response.result["pong"] is True


def test_dispatch_status_always_succeeds() -> None:
    response = _server(connected=False)._dispatch(wire.encode({"method": "status"}))
    assert response.ok is True
    assert response.result is not None and response.result["connected"] is False


def test_dispatch_telemetry_requires_connection() -> None:
    response = _server(connected=False)._dispatch(wire.encode({"method": "telemetry"}))
    assert response.ok is False
    assert response.error is not None
    assert response.error.code == ExitCode.VEHICLE_NOT_CONNECTED


def test_dispatch_telemetry_ok_when_connected() -> None:
    response = _server(connected=True)._dispatch(wire.encode({"method": "telemetry"}))
    assert response.ok is True
    assert response.result is not None
    assert "position" in response.result


def test_dispatch_unknown_method() -> None:
    response = _server(connected=True)._dispatch(wire.encode({"method": "nope"}))
    assert response.ok is False
    assert response.error is not None
    assert response.error.code == ExitCode.GENERAL_ERROR


def test_dispatch_malformed_frame() -> None:
    response = _server(connected=True)._dispatch(b"{not json}\n")
    assert response.ok is False
    assert response.error is not None
    assert response.error.code == ExitCode.USAGE_ERROR
