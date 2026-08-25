"""Abstract vehicle adapter interface.

No pymavlink import here — this module defines the contract the daemon
depends on. Concrete transports (e.g. :mod:`mavctl.adapter.pymavlink_adapter`)
implement it.
"""

from __future__ import annotations

from typing import Protocol, runtime_checkable

from mavctl.models import CommandOutcome, Telemetry, VehicleState


class AdapterError(Exception):
    """Base class for adapter-level failures."""


class ConnectionLostError(AdapterError):
    """Raised when the underlying link cannot be established or is lost."""


@runtime_checkable
class VehicleAdapter(Protocol):
    """Transport-agnostic view of a single MAVLink vehicle.

    Implementations maintain a continuously-updated snapshot of vehicle
    status and telemetry; :meth:`get_state` and :meth:`get_telemetry` are
    cheap, non-blocking reads of that snapshot. The command verbs
    (:meth:`arm` … :meth:`rtl`) send a COMMAND_LONG and block only until the
    vehicle's COMMAND_ACK (with retries), never until the manoeuvre finishes;
    completion is observed by polling the snapshot.
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

    def mode_names(self) -> list[str]:
        """Return the flight-mode names supported by the vehicle."""
        ...

    def arm(self, force: bool = False) -> CommandOutcome:
        """Send an arm command and await its ACK."""
        ...

    def disarm(self, force: bool = False) -> CommandOutcome:
        """Send a disarm command and await its ACK."""
        ...

    def set_mode(self, mode: str) -> CommandOutcome:
        """Switch flight mode by name (resolved via the vehicle mode map)."""
        ...

    def takeoff(self, altitude_m: float) -> CommandOutcome:
        """Command a takeoff to ``altitude_m`` metres relative altitude."""
        ...

    def land(self) -> CommandOutcome:
        """Command a land at the current position."""
        ...

    def rtl(self) -> CommandOutcome:
        """Command a return-to-launch."""
        ...

