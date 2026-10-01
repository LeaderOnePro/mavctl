"""Daemon-side operation registry (Phase 3B-0 foundation).

Implements the v1 `[DECIDED]` model from
docs/design/long-running-operation-interruption.md §C.2.1:

- **one active passive-observation operation per daemon/vehicle** (not per
  kind) — observation semantics on one vehicle cannot be split by command
  kind;
- a newly accepted operation **atomically supersedes** the previous active
  owner (the superseded operation stores
  `superseded_by_operation_id` and can never report `reached` again);
- every activation bumps the registry-wide **epoch**; a passive waiter may
  only record outcomes while its `operation_id + epoch` still matches the
  registry's active pair (stale-waiter fencing);
- the registry is **in-memory only** — a daemon restart loses it and prior
  operations become unknown/uncertain (no persistence in v1).

Concurrency: all registry mutations happen on the daemon's asyncio event
loop (handlers and observation tasks share one thread), so plain attribute
updates are atomic at ``await`` boundaries; no extra lock is added.
"""

from __future__ import annotations

import time
import uuid

from mavctl.models.operation import (
    TERMINAL_OPERATION_STATES,
    OperationKind,
    OperationSnapshot,
    OperationState,
)

_TERMINAL_REASON_SUPERSEDED = "superseded by a newer accepted command"


class Operation:
    """Mutable daemon-internal operation record (not exported via RPC)."""

    __slots__ = (
        "ack_monotonic",
        "created_monotonic",
        "effect_sent_monotonic",
        "epoch",
        "kind",
        "operation_id",
        "state",
        "superseded_by_operation_id",
        "terminal_reason",
    )

    def __init__(
        self,
        *,
        operation_id: str,
        epoch: int,
        kind: OperationKind,
        created_monotonic: float,
    ) -> None:
        self.operation_id = operation_id
        self.epoch = epoch
        self.kind = kind
        self.created_monotonic = created_monotonic
        self.effect_sent_monotonic: float | None = None
        self.ack_monotonic: float | None = None
        self.state = OperationState.PENDING
        self.superseded_by_operation_id: str | None = None
        self.terminal_reason: str | None = None

    def snapshot(self, now_monotonic: float) -> OperationSnapshot:
        """Safe external view: ages only, no internal epoch."""

        def _age(mono: float | None) -> float | None:
            return round(now_monotonic - mono, 3) if mono is not None else None

        return OperationSnapshot(
            id=self.operation_id,
            kind=self.kind,
            state=self.state,
            created_age_s=_age(self.created_monotonic),
            effect_sent_age_s=_age(self.effect_sent_monotonic),
            ack_age_s=_age(self.ack_monotonic),
            superseded_by_operation_id=self.superseded_by_operation_id,
            terminal_reason=self.terminal_reason,
        )


class OperationRegistry:
    """Single-active-owner registry with epoch fencing (in-memory, v1)."""

    def __init__(self) -> None:
        self._active: Operation | None = None
        self._operations: dict[str, Operation] = {}
        self._last_epoch = 0

    # -- registration ------------------------------------------------------

    def activate(
        self,
        kind: OperationKind,
        *,
        effect_sent_monotonic: float,
        ack_monotonic: float,
        now_monotonic: float | None = None,
    ) -> Operation:
        """Register a freshly ACKed operation as the active owner.

        Atomically supersedes any prior active operation. Must be called
        while the daemon command lock is held (linearization checkpoint,
        design doc §C.2.1-C).
        """

        now = time.monotonic() if now_monotonic is None else now_monotonic
        operation = Operation(
            operation_id=f"op-{uuid.uuid4()}",
            epoch=self._last_epoch + 1,
            kind=kind,
            created_monotonic=now,
        )
        operation.effect_sent_monotonic = effect_sent_monotonic
        operation.ack_monotonic = ack_monotonic
        operation.state = OperationState.WAITING

        previous = self._active
        if previous is not None and previous.state not in TERMINAL_OPERATION_STATES:
            previous.state = OperationState.SUPERSEDED
            previous.superseded_by_operation_id = operation.operation_id
            previous.terminal_reason = _TERMINAL_REASON_SUPERSEDED

        self._active = operation
        self._operations[operation.operation_id] = operation
        self._last_epoch = operation.epoch
        return operation

    # -- fencing -----------------------------------------------------------

    def is_active(self, operation_id: str, epoch: int) -> bool:
        """Fencing check: may this observer still record outcomes?"""

        active = self._active
        return (
            active is not None
            and active.operation_id == operation_id
            and active.epoch == epoch
        )

    # -- observation writes (fenced) ----------------------------------------

    def record_reached(self, operation_id: str, epoch: int) -> bool:
        """Record the milestone as reached; fenced, returns success.

        Terminal states are final: a fenced, terminal operation never
        transitions again (e.g. LINK_LOST → REACHED is impossible)."""

        if not self.is_active(operation_id, epoch):
            return False
        operation = self._operations[operation_id]
        if operation.state is OperationState.WAITING:
            operation.state = OperationState.REACHED
            operation.terminal_reason = "milestone reached"
            return True
        return False

    def record_link_lost(self, operation_id: str, epoch: int) -> bool:
        """Record heartbeat loss while still the active owner; fenced."""

        if not self.is_active(operation_id, epoch):
            return False
        operation = self._operations[operation_id]
        if operation.state is OperationState.WAITING:
            operation.state = OperationState.LINK_LOST
            operation.terminal_reason = "heartbeat lost during wait"
            return True
        return False

    def record_timed_out(self, operation_id: str, epoch: int) -> bool:
        """Record that the (client's) observation deadline elapsed while the
        operation was still the active owner; fenced. The operation stays
        active and keeps observing in the background (client timeout ≠
        vehicle command cancelled)."""

        if not self.is_active(operation_id, epoch):
            return False
        operation = self._operations[operation_id]
        if operation.state is OperationState.WAITING:
            operation.terminal_reason = (
                "client wait deadline elapsed; observation continues"
            )
            return True
        return False

    # -- lookup -------------------------------------------------------------

    def get(self, operation_id: str) -> Operation | None:
        return self._operations.get(operation_id)

    def active(self) -> Operation | None:
        return self._active

    def snapshot(
        self, operation_id: str, now_monotonic: float | None = None
    ) -> OperationSnapshot | None:
        operation = self._operations.get(operation_id)
        if operation is None:
            return None
        now = time.monotonic() if now_monotonic is None else now_monotonic
        return operation.snapshot(now)
