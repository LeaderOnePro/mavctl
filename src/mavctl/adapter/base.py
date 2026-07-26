"""Abstract vehicle adapter interface.

No pymavlink import here — this module defines the contract the daemon
depends on. Concrete transports (e.g. :mod:`mavctl.adapter.pymavlink_adapter`)
implement it.
"""

from __future__ import annotations

from typing import Protocol, runtime_checkable

from mavctl.models import Telemetry, VehicleState


class AdapterError(Exception):
    """Base class for adapter-level failures."""


class ConnectionLostError(AdapterError):
    """Raised when the underlying link cannot be established or is lost."""


@runtime_checkable
class VehicleAdapter(Protocol):
    """Transport-agnostic view of a single MAVLink vehicle.

    Implementations maintain a continuously-updated snapshot of vehicle
    status and telemetry; :meth:`get_state` and :meth:`get_telemetry` are
    cheap, non-blocking reads of that snapshot.
    """

    def connect(self) -> None:
        """Open the link and begin ingesting messages.

        Raises:
            ConnectionLostError: if the link cannot be opened.
        """
        ...

    def disconnect(self) -> None:
        """Close the link and stop ingesting messages. Idempotent."""
        ...

    def get_state(self) -> VehicleState:
        """Return the latest cached vehicle status snapshot."""
        ...

    def get_telemetry(self) -> Telemetry:
        """Return the latest cached telemetry snapshot."""
        ...
