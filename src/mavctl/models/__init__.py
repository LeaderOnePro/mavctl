"""Core pydantic data models shared across all layers.

This package is a leaf dependency: it must not import from ``cli``, ``daemon``,
or ``adapter``.
"""

from mavctl.models.commands import MAV_RESULT_NAMES, CommandOutcome
from mavctl.models.protocol import (
    DaemonResponse,
    ExitCode,
    RpcError,
    RpcRequest,
)
from mavctl.models.state import Battery, GpsInfo, HomePosition, VehicleState
from mavctl.models.telemetry import Attitude, Position, Telemetry, Velocity

__all__ = [
    "MAV_RESULT_NAMES",
    "Attitude",
    "Battery",
    "CommandOutcome",
    "DaemonResponse",
    "ExitCode",
    "GpsInfo",
    "HomePosition",
    "Position",
    "RpcError",
    "RpcRequest",
    "Telemetry",
    "VehicleState",
    "Velocity",
]
