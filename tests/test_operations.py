"""Operation foundation tests: registry, epoch fencing, supersession.

Deterministic: every test drives the registry with explicit monotonic
stamps (no sleeps, no real clock). The daemon/CLI integration tests live
in tests/test_operation_server.py and tests/test_cli.py.
"""

from __future__ import annotations

from mavctl.daemon.operations import (
    MAX_RETAINED_TERMINAL_OPERATIONS,
    OperationRegistry,
)
from mavctl.models.operation import (
    OperationKind,
    OperationState,
)


def _activate(
    registry: OperationRegistry,
    kind: OperationKind = OperationKind.TAKEOFF,
    *,
    base: float = 100.0,
) -> tuple[str, int, float]:
    """Activate an operation as if its effecting command was just ACKed."""

    operation = registry.activate(
        kind,
        effect_sent_monotonic=base,
        ack_monotonic=base + 0.05,
        now_monotonic=base,
    )
    return operation.operation_id, operation.epoch, base


# -- registration / supersession ---------------------------------------------


def test_activate_registers_active_owner_with_ages() -> None:
    registry = OperationRegistry()
    operation_id, _epoch, base = _activate(registry)

    operation = registry.get(operation_id)
    assert operation is not None
    assert operation.state is OperationState.WAITING
    assert operation.effect_sent_monotonic == base
    assert operation.ack_monotonic == base + 0.05
    snapshot = registry.snapshot(operation_id, now_monotonic=base + 1.0)
    assert snapshot is not None
    assert snapshot.created_age_s == 1.0
    assert snapshot.ack_age_s == 0.95
    assert snapshot.superseded_by_operation_id is None


def test_second_accepted_operation_supersedes_first() -> None:
    registry = OperationRegistry()
    first_id, first_epoch, _ = _activate(registry, OperationKind.TAKEOFF)

    second_id, second_epoch, _ = _activate(registry, OperationKind.RTL, base=101.0)

    first = registry.get(first_id)
    assert first is not None
    assert first.state is OperationState.SUPERSEDED
    assert first.superseded_by_operation_id == second_id
    assert first.terminal_reason is not None
    # the new operation is the sole active owner
    assert registry.active() is not None
    assert registry.active().operation_id == second_id  # type: ignore[union-attr]
    assert second_epoch == first_epoch + 1  # epoch increments per activation


def test_active_owner_is_per_vehicle_not_per_kind() -> None:
    """A second operation of the SAME kind also supersedes (one active owner
    per vehicle — not per kind)."""

    registry = OperationRegistry()
    _first_id, first_epoch, _ = _activate(registry, OperationKind.TAKEOFF)
    second_id, second_epoch, _ = _activate(registry, OperationKind.TAKEOFF, base=101.0)

    assert second_epoch == first_epoch + 1
    assert registry.active() is not None
    assert registry.active().operation_id == second_id  # type: ignore[union-attr]


# -- epoch fencing -------------------------------------------------------------


def test_fenced_observer_cannot_mark_reached_after_supersession() -> None:
    """§C.2.1-E: a stale waiter whose operation was superseded must never
    write reached/success — even if its predicate becomes true later."""

    registry = OperationRegistry()
    first_id, first_epoch, _ = _activate(registry, OperationKind.TAKEOFF)
    _second_id, _second_epoch, _ = _activate(registry, OperationKind.RTL, base=101.0)

    # the stale waiter wakes up and its altitude predicate is true:
    assert registry.record_reached(first_id, first_epoch) is False

    first = registry.get(first_id)
    assert first is not None
    assert first.state is OperationState.SUPERSEDED  # never reached
    assert registry.active() is not None
    assert registry.active().state is OperationState.WAITING  # type: ignore[union-attr]


def test_active_owner_records_reached_within_epoch() -> None:
    registry = OperationRegistry()
    operation_id, epoch, _base = _activate(registry, OperationKind.TAKEOFF)

    assert registry.record_reached(operation_id, epoch) is True
    operation = registry.get(operation_id)
    assert operation is not None
    assert operation.state is OperationState.REACHED
    assert operation.terminal_reason == "milestone reached"


def test_epoch_mismatch_is_fenced_even_with_matching_id() -> None:
    registry = OperationRegistry()
    operation_id, epoch, _ = _activate(registry)
    _new_id, _new_epoch, _ = _activate(registry, OperationKind.RTL, base=101.0)

    # stale waiter replays with the right id but its remembered (old) epoch
    assert registry.is_active(operation_id, epoch) is False
    assert registry.record_reached(operation_id, epoch) is False


# -- link loss / timeout recording ---------------------------------------------


def test_link_lost_recorded_only_while_active() -> None:
    registry = OperationRegistry()
    operation_id, epoch, _ = _activate(registry)

    assert registry.record_link_lost(operation_id, epoch) is True
    assert registry.get(operation_id).state is OperationState.LINK_LOST  # type: ignore[union-attr]

    # fenced after terminal
    assert registry.record_reached(operation_id, epoch) is False
    assert registry.get(operation_id).state is OperationState.LINK_LOST  # type: ignore[union-attr]


def test_timed_out_keeps_operation_observing() -> None:
    """Client wait deadline elapsed: terminal_reason is recorded but the
    operation stays WAITING (it keeps observing in the background — client
    timeout ≠ vehicle command cancelled)."""

    registry = OperationRegistry()
    operation_id, epoch, _ = _activate(registry)

    assert registry.record_timed_out(operation_id, epoch) is True
    operation = registry.get(operation_id)
    assert operation is not None
    assert operation.state is OperationState.WAITING  # still observing
    assert "client wait deadline elapsed" in (operation.terminal_reason or "")

    # it can still reach afterwards
    assert registry.record_reached(operation_id, epoch) is True
    assert registry.get(operation_id).state is OperationState.REACHED  # type: ignore[union-attr]


# -- lookup / restart semantics -------------------------------------------------


def test_unknown_operation_returns_none() -> None:
    registry = OperationRegistry()
    assert registry.get("op-does-not-exist") is None
    assert registry.snapshot("op-does-not-exist") is None


def test_fresh_registry_has_no_active_operation() -> None:
    """Daemon restart semantics at registry level: a fresh registry owns
    nothing; prior operations are gone (unknown/uncertain to clients)."""

    registry = OperationRegistry()
    assert registry.active() is None
    operation_id, _epoch, _ = _activate(registry)
    del registry  # daemon restart
    fresh = OperationRegistry()
    assert fresh.active() is None
    assert fresh.get(operation_id) is None


# -- snapshot safety ------------------------------------------------------------


def test_snapshot_exposes_safe_fields_only() -> None:
    registry = OperationRegistry()
    operation_id, _epoch, base = _activate(registry)
    snapshot = registry.snapshot(operation_id, now_monotonic=base + 0.5)
    assert snapshot is not None
    data = snapshot.model_dump()
    assert set(data) == {
        "id",
        "kind",
        "state",
        "created_age_s",
        "effect_sent_age_s",
        "ack_age_s",
        "superseded_by_operation_id",
        "terminal_reason",
    }
    assert "epoch" not in data  # internal generation never crosses the RPC boundary

# -- bounded retention (P2) -----------------------------------------------------


def test_retention_constant_is_256() -> None:
    from mavctl.daemon import operations as operations_module

    assert operations_module.MAX_RETAINED_TERMINAL_OPERATIONS == 256


def test_retention_evicts_oldest_terminal_keeps_active_and_newest() -> None:
    registry = OperationRegistry()
    total = MAX_RETAINED_TERMINAL_OPERATIONS + 20
    ids: list[str] = []
    for i in range(total):
        operation_id, _epoch, _ = _activate(
            registry, OperationKind.TAKEOFF, base=100.0 + i
        )
        ids.append(operation_id)

    # every activation supersedes the previous one (each becomes terminal),
    # so the last activation is the active owner.
    active = registry.active()
    assert active is not None
    assert active.operation_id == ids[-1]

    # the oldest terminal operations are evicted; the newest are retained
    assert registry.get(ids[0]) is None  # oldest terminal evicted
    assert registry.get(ids[1]) is None
    assert registry.get(ids[-2]) is not None  # newest terminal retained
    assert registry.get(ids[-1]) is not None  # active owner retained

    # retention accounting: retained = 1 active + 256 terminal
    retained = sum(1 for op_id in ids if registry.get(op_id) is not None)
    assert retained == MAX_RETAINED_TERMINAL_OPERATIONS + 1


def test_retained_operations_keep_terminal_evidence() -> None:
    registry = OperationRegistry()
    ids: list[str] = []
    for i in range(MAX_RETAINED_TERMINAL_OPERATIONS + 5):
        operation_id, _epoch, _ = _activate(
            registry, OperationKind.TAKEOFF, base=100.0 + i
        )
        ids.append(operation_id)

    newest_terminal = registry.get(ids[-2])  # ids[-1] is the active owner
    assert newest_terminal is not None
    assert newest_terminal.state is OperationState.SUPERSEDED
    assert newest_terminal.superseded_by_operation_id == ids[-1]


def test_evicted_id_fencing_is_still_safe() -> None:
    """An evicted id's stale waiter is fenced: record_* returns False and
    nothing resurrects the evicted record."""

    registry = OperationRegistry()
    first_id, first_epoch, _ = _activate(registry)
    for i in range(MAX_RETAINED_TERMINAL_OPERATIONS + 10):
        _activate(registry, OperationKind.TAKEOFF, base=100.0 + i)

    assert registry.get(first_id) is None
    assert registry.record_reached(first_id, first_epoch) is False
    assert registry.record_link_lost(first_id, first_epoch) is False
    assert registry.record_uncertain(
        first_id, first_epoch, reason="operation_observation_failed"
    ) is False
    assert registry.get(first_id) is None
