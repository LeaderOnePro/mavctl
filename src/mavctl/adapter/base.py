"""Abstract vehicle adapter interface.

No pymavlink import here — this module defines the contract the daemon
depends on. Concrete transports (e.g. :mod:`mavctl.adapter.pymavlink_adapter`)
implement it.
"""

from __future__ import annotations

from typing import Protocol, runtime_checkable

from mavctl.models import (
    CommandOutcome,
    DownloadedMissionV1,
    MissionOutcome,
    MissionV1,
    Telemetry,
    VehicleState,
)


class AdapterError(Exception):
    """Base class for adapter-level failures."""


class ConnectionLostError(AdapterError):
    """Raised when the underlying link cannot be established or is lost."""


class ModeMappingUnavailableError(AdapterError):
    """Raised when a flight mode cannot be resolved against the vehicle's
    mode mapping.

    This is a transient vehicle state (the mapping has not populated yet, or
    changed between validation and send), not a user-input error: the daemon
    maps it to a structured, retryable safety rejection (exit 5
    ``mode_map_unavailable``) instead of an internal error. Unknown-mode user
    input is rejected earlier by the guard (exit 2 ``unknown_mode``).
"""


class MissionProtocolError(AdapterError):
    """MAVLink mission protocol failure: vehicle rejection, protocol
    violation, or a transaction timeout whose remote effect is known.

    ``result_name`` carries the ``MAV_MISSION_RESULT`` name (e.g.
    ``NO_SPACE``) or a diagnostic such as ``TIMEOUT``.
    """

    def __init__(self, message: str, *, result_name: str = "ERROR") -> None:
        super().__init__(message)
        self.result_name = result_name


class MissionStateUncertainError(MissionProtocolError):
    """The remote mission state may be partially modified and must be
    verified with ``mavctl mission download``.

    Raised only once ``MISSION_COUNT``/``MISSION_CLEAR_ALL`` has reached the
    vehicle; failures before that are plain :class:`MissionProtocolError`.

    ``sent_upto`` is the highest sequence number mavctl **locally sent** —
    it is not a vehicle-confirmed acceptance. It is expressed in the MAVLink
    wire sequence space, which on ArduPilot includes the vehicle-managed home
    slot at seq 0 (mavctl's v1 items occupy wire seqs 1..N).
    """

    def __init__(
        self,
        message: str,
        *,
        sent_upto: int | None = None,
        observed_count: int | None = None,
    ) -> None:
        super().__init__(message, result_name="UNCERTAIN")
        self.sent_upto = sent_upto
        self.observed_count = observed_count


class MissionCountUnsupportedError(MissionProtocolError):
    """The remote mission is larger than mavctl v1 can represent
    (``MISSION_COUNT`` exceeded the supported item limit); nothing was
    requested."""

    def __init__(self, message: str, *, observed_count: int, max_supported_items: int) -> None:
        super().__init__(message, result_name="NO_SPACE")
        self.observed_count = observed_count
        self.max_supported_items = max_supported_items


class MissionItemUnsupportedError(MissionProtocolError):
    """A received mission item cannot be represented in the v1 schema;
    download fails atomically rather than approximating it."""

    def __init__(self, message: str, *, seq: int, command: int, frame: int) -> None:
        super().__init__(message, result_name="UNSUPPORTED")
        self.seq = seq
        self.command = command
        self.frame = frame


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

    def arm(self) -> CommandOutcome:
        """Send an arm command and await its ACK.

        Force-arm (MAV_CMD_COMPONENT_ARM_DISARM param2=21196) is deliberately
        not part of this interface: pre-arm checks are never bypassed.
        """
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

    def upload_mission(self, mission: MissionV1) -> MissionOutcome:
        """Run the MAVLink mission upload transaction for a validated plan.

        Raises:
            MissionProtocolError: the vehicle rejected the mission or the
                transaction failed with a known remote outcome.
            MissionStateUncertainError: the transaction aborted after the
                vehicle may have stored a partial mission.
        """
        ...

    def download_mission(self) -> DownloadedMissionV1:
        """Read the remote mission atomically.

        Raises:
            MissionProtocolError: transaction timeout or vehicle denial —
                never a partial mission.
            MissionItemUnsupportedError: the remote mission contains items
                outside the v1 schema.
        """
        ...

    def clear_mission(self) -> MissionOutcome:
        """Clear the remote mission and verify the remote count is zero.

        Raises:
            MissionProtocolError: the vehicle rejected the clear.
            MissionStateUncertainError: the clear or its read-back could not
                be confirmed (``observed_count`` set when known).
        """
        ...

