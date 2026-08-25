"""Models describing the outcome of a vehicle command (COMMAND_ACK)."""

from __future__ import annotations

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


class CommandOutcome(BaseModel):
    """Result of sending a COMMAND_LONG and awaiting its COMMAND_ACK."""

    accepted: bool
    timed_out: bool = False
    result_code: int | None = None
    result_name: str = "UNKNOWN"
    attempts: int = 0

    @classmethod
    def from_ack(cls, result_code: int, attempts: int) -> CommandOutcome:
        return cls(
            accepted=result_code == 0,
            timed_out=False,
            result_code=result_code,
            result_name=MAV_RESULT_NAMES.get(result_code, f"RESULT_{result_code}"),
            attempts=attempts,
        )

    @classmethod
    def timeout(cls, attempts: int) -> CommandOutcome:
        return cls(accepted=False, timed_out=True, result_name="TIMEOUT", attempts=attempts)
