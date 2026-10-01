"""Operation model: long-running flight-command observation records.

Phase 3B-0 foundation (docs/design/long-running-operation-interruption.md
§C.2.1/F): a `--wait`-capable flight command registers a daemon-owned
operation after its effecting command is ACKed, and a passive observer —
fenced by operation id + epoch — records the milestone outcome.

This module holds the pure data shapes only: the daemon registry lives in
`mavctl.daemon.operations` and pymavlink stays confined to the adapter layer.
"""

from __future__ import annotations

from enum import Enum

from pydantic import BaseModel, ConfigDict


class OperationKind(str, Enum):
    """Flight-command kinds that create operations (Phase 3B-0 set)."""

    TAKEOFF = "takeoff"
    LAND = "land"
    RTL = "rtl"


class OperationState(str, Enum):
    """Operation lifecycle states (design doc §F).

    Evidence levels: ``pending``/``command_sent`` are daemon-local;
    ``accepted`` is vehicle-confirmed (COMMAND_ACK); ``waiting``/``reached``/
    ``timed_out``/``link_lost`` are daemon-local observations of
    vehicle-confirmed predicates; ``superseded`` is a daemon-local
    active-owner replacement (never a vehicle-confirmed cancellation);
    ``uncertain`` means the vehicle effect/state cannot be established
    (restart / evidence loss); ``failed`` is vehicle-confirmed (NACK) or
    guard-local.
    """

    PENDING = "pending"
    COMMAND_SENT = "command_sent"
    ACCEPTED = "accepted"
    WAITING = "waiting"
    REACHED = "reached"
    TIMED_OUT = "timed_out"
    LINK_LOST = "link_lost"
    SUPERSEDED = "superseded"
    UNCERTAIN = "uncertain"
    FAILED = "failed"


TERMINAL_OPERATION_STATES = frozenset(
    {
        OperationState.REACHED,
        OperationState.TIMED_OUT,
        OperationState.LINK_LOST,
        OperationState.SUPERSEDED,
        OperationState.UNCERTAIN,
        OperationState.FAILED,
    }
)
"""States that end an operation; a fenced observer never writes into them."""


class OperationSnapshot(BaseModel):
    """Safe, externally visible view of an operation (``operation get``).

    Deliberately excludes the internal epoch and any adapter/exception
    internals: only evidence-classified fields cross the RPC boundary.
    """

    model_config = ConfigDict(extra="forbid")

    id: str
    kind: OperationKind
    state: OperationState
    created_age_s: float | None = None
    effect_sent_age_s: float | None = None
    ack_age_s: float | None = None
    superseded_by_operation_id: str | None = None
    terminal_reason: str | None = None
