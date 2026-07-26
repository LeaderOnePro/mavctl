"""Core pydantic data models shared across all layers.

This package is a leaf dependency: it must not import from ``cli``, ``daemon``,
or ``adapter``.
"""

from mavctl.models.protocol import (
    DaemonResponse,
    ExitCode,
    RpcError,
    RpcRequest,
)
from mavctl.models.state import Battery, GpsInfo, VehicleState
from mavctl.models.telemetry import Attitude, Position, Telemetry, Velocity

__all__ = [
    "Attitude",
    "Battery",
    "DaemonResponse",
    "ExitCode",
    "GpsInfo",
    "Position",
    "RpcError",
    "RpcRequest",
    "Telemetry",
    "VehicleState",
    "Velocity",
]
