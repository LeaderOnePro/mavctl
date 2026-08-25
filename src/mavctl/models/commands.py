"""Models describing the outcome of a vehicle command (COMMAND_ACK)."""

from __future__ import annotations

from enum import Enum

from pydantic import BaseModel

# MAV_RESULT code -> human label (MAVLink common enum).
MAV_RESULT_NAMES: dict[int, str] = {
    0: "ACCEPTED",
    1: "TEMPORARILY_REJECTED",
    2: "DENIED",
    3: "UNSUPPORTED",
    4: "FAILED",
    5: "IN_PROGRESS",
    6: "CANCELLED",
    7: "COMMAND_LONG_ONLY",
    8: "COMMAND_INT_ONLY",
    9: "COMMAND_UNSUPPORTED_MAV_FRAME",
}

# Results that mean "the vehicle took the command". ACCEPTED is terminal-OK;
# IN_PROGRESS means accepted and still executing (NOT a NACK). Both let the
# caller proceed (and, if --wait, keep polling for the target state).
MAV_RESULT_ACCEPTED = 0
MAV_RESULT_IN_PROGRESS = 5
_ACCEPTED_RESULTS = frozenset({MAV_RESULT_ACCEPTED, MAV_RESULT_IN_PROGRESS})


class CommandOutcome(BaseModel):
    """Result of sending a COMMAND_LONG and awaiting its COMMAND_ACK."""

    accepted: bool
    in_progress: bool = False
    timed_out: bool = False
    result_code: int | None = None
    result_name: str = "UNKNOWN"
    attempts: int = 0

    @classmethod
    def from_ack(cls, result_code: int, attempts: int) -> CommandOutcome:
        return cls(
            accepted=result_code in _ACCEPTED_RESULTS,
            in_progress=result_code == MAV_RESULT_IN_PROGRESS,
            timed_out=False,
            result_code=result_code,
            result_name=MAV_RESULT_NAMES.get(result_code, f"RESULT_{result_code}"),
            attempts=attempts,
        )

    @classmethod
    def timeout(cls, attempts: int) -> CommandOutcome:
        return cls(accepted=False, timed_out=True, result_name="TIMEOUT", attempts=attempts)


class WaitStatus(str, Enum):
    """Explicit outcome of a --wait phase (never a bare bool).

    NOT_WAITED  — caller did not request --wait.
    REACHED     — the target state was observed within the timeout.
    TIMEOUT     — the target state was not reached before the timeout.
    LINK_LOST   — the heartbeat went stale during the wait (link dropped).
    """

    NOT_WAITED = "not_waited"
    REACHED = "reached"
    TIMEOUT = "timeout"
    LINK_LOST = "link_lost"
