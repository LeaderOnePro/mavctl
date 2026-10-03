"""Mock-first tests for Phase 3B-1 adapter behavior: `MAV_CMD_MISSION_START`,
`MISSION_CURRENT` observation, and the mission-count probe.

All wire traffic uses the existing FakeMaster/FakeMsg doubles; no MAVLink
endpoint is contacted. Deterministic via injected monotonic stamps.
"""

from __future__ import annotations

import pytest

from mavctl.adapter.pymavlink_adapter import PymavlinkAdapter
from mavctl.daemon.operations import OperationRegistry
from mavctl.models.operation import OperationKind, OperationState
from tests.fakes import FakeMaster, FakeMsg

_ARMED_FLAG = 0b10000000


def _hb(
    system: int = 1,
    component: int = 1,
    armed: bool = False,
    custom_mode: int = 3,  # AUTO
    system_status: int = 3,
) -> FakeMsg:
    return FakeMsg(
        "HEARTBEAT",
        src_system=system,
        src_component=component,
        base_mode=_ARMED_FLAG if armed else 0,
        custom_mode=custom_mode,
        system_status=system_status,
    )


def _mission_current(
    seq: int = 1,
    total: int = 4,
    mission_state: int = 3,
    mission_mode: int = 1,
    system: int = 1,
    component: int = 1,
) -> FakeMsg:
    return FakeMsg(
        "MISSION_CURRENT",
        src_system=system,
        src_component=component,
        seq=seq,
        total=total,
        mission_state=mission_state,
        mission_mode=mission_mode,
    )


def _adapter(armed: bool = True) -> tuple[PymavlinkAdapter, FakeMaster]:
    # Fast command timeouts so denied/timeout tests stay deterministic.
    adapter = PymavlinkAdapter(
        "udp:127.0.0.1:14550",
        command_ack_timeout_s=0.05,
        command_retries=1,
        command_ack_settle_s=0.0,
    )
    master = FakeMaster(flightmode="AUTO")
    adapter._master = master
    adapter._on_heartbeat(_hb(1, 1, armed=armed, custom_mode=3, system_status=3))
    return adapter, master


def _wire_mission_start_acks(
    adapter: PymavlinkAdapter,
    master: FakeMaster,
    results: list[int | None],
) -> None:
    """Script COMMAND_ACK results for subsequent start_mission() calls,
    delivered from the adapter's locked target (mirrors
    tests/test_adapter.py::_wire_acks)."""

    state = {"i": 0}

    def responder(
        _tsys: int, _tcomp: int, command: int, confirmation: int, *_params: float
    ) -> None:
        master.sent.append((command, confirmation, tuple(_params)))
        i = state["i"]
        state["i"] += 1
        result = results[i] if i < len(results) else None
        if result is not None:
            adapter._on_command_ack(
                FakeMsg(
                    "COMMAND_ACK",
                    src_system=1,
                    src_component=1,
                    command=command,
                    result=result,
                )
            )

    master.mav.command_long_send = responder
    # restore on registry reset is unnecessary: tests use fresh doubles


class _FakeMonotonicClock:
    def __init__(self, start: float = 1000.0) -> None:
        self.now = start

    def advance(self, seconds: float) -> None:
        self.now += seconds

    def __call__(self) -> float:
        return self.now


# -- MAV_CMD_MISSION_START ------------------------------------------------------


def test_start_mission_sends_zero_params() -> None:
    """ArduCopter answers MAV_RESULT_DENIED for non-zero param1/param2 — the
    start command must send the bare command with zeroed params."""
    adapter, master = _adapter()
    _wire_mission_start_acks(adapter, master, [0])
    adapter.start_mission()
    sent = [entry for entry in master.sent if entry[0] == 300]
    assert len(sent) == 1
    _command, _conf, params = sent[0]
    assert params[:2] == (0.0, 0.0)


def test_start_mission_ack_accepted() -> None:
    adapter, master = _adapter()
    _wire_mission_start_acks(adapter, master, [0])
    outcome = adapter.start_mission()
    assert outcome.accepted is True
    assert outcome.result_name == "ACCEPTED"
    assert outcome.attempts == 1


def test_start_mission_denied() -> None:
    adapter, master = _adapter()
    _wire_mission_start_acks(adapter, master, [2])  # MAV_RESULT_DENIED
    outcome = adapter.start_mission()
    assert outcome.accepted is False
    assert outcome.result_name == "DENIED"


def test_start_mission_timeout() -> None:
    adapter, master = _adapter()
    _wire_mission_start_acks(adapter, master, [None])  # no ACK arrives
    outcome = adapter.start_mission()
    assert outcome.accepted is False
    assert outcome.timed_out is True


# -- MISSION_CURRENT observation -------------------------------------------------


def test_mission_current_from_locked_source_updates_state(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    clock = _FakeMonotonicClock()
    monkeypatch.setattr("mavctl.adapter.pymavlink_adapter.time.monotonic", clock)
    adapter, _master = _adapter()

    adapter._on_mission_current(_mission_current(seq=1, total=4, mission_state=3))
    clock.advance(0.5)

    state = adapter.get_state()
    assert state.mission is not None
    assert state.mission.current_seq == 1
    assert state.mission.total == 4
    assert state.mission.state == "active"
    assert state.mission.mode == "mission"
    assert state.mission.age_s == pytest.approx(0.5)


def test_mission_current_progress_updates_seq_and_age(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    clock = _FakeMonotonicClock()
    monkeypatch.setattr("mavctl.adapter.pymavlink_adapter.time.monotonic", clock)
    adapter, _master = _adapter()

    adapter._on_mission_current(_mission_current(seq=1))
    clock.advance(2.0)
    adapter._on_mission_current(_mission_current(seq=2))

    state = adapter.get_state()
    assert state.mission is not None
    assert state.mission.current_seq == 2
    assert state.mission.age_s == pytest.approx(0.0)


def test_foreign_mission_current_never_pollutes_state(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    clock = _FakeMonotonicClock()
    monkeypatch.setattr("mavctl.adapter.pymavlink_adapter.time.monotonic", clock)
    adapter, _master = _adapter()
    adapter._on_mission_current(_mission_current(seq=1))  # locked source

    adapter._on_mission_current(_mission_current(seq=9, system=2, component=1))
    adapter._on_mission_current(_mission_current(seq=9, system=1, component=154))

    state = adapter.get_state()
    assert state.mission is not None
    assert state.mission.current_seq == 1  # unchanged


def test_mission_state_labels_cover_mav_mission_state(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    clock = _FakeMonotonicClock()
    monkeypatch.setattr("mavctl.adapter.pymavlink_adapter.time.monotonic", clock)
    adapter, _master = _adapter()
    for raw, label in (
        (0, "unknown"),
        (1, "no_mission"),
        (2, "not_started"),
        (3, "active"),
        (4, "paused"),
        (5, "complete"),
        (9, "mission_state_9"),
    ):
        adapter._on_mission_current(_mission_current(mission_state=raw))
        state = adapter.get_state()
        assert state.mission is not None
        assert state.mission.state == label, raw


def test_mission_state_survives_heartbeat_loss_but_flags_staleness(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """Cached mission observation is retained with a growing age; after
    heartbeat loss `connected` is False, so the observation must not be
    treated as a live fact by callers."""

    clock = _FakeMonotonicClock()
    monkeypatch.setattr("mavctl.adapter.pymavlink_adapter.time.monotonic", clock)
    adapter, _master = _adapter()
    adapter._on_mission_current(_mission_current(seq=2, mission_state=3))

    clock.advance(60.0)
    state = adapter.get_state()
    assert state.mission is not None
    assert state.mission.current_seq == 2  # cache retained
    assert state.mission.age_s == pytest.approx(60.0)  # staleness visible
    assert state.connected is False  # not a live fact


def test_mission_current_absent_yields_all_none() -> None:
    adapter, _master = _adapter()
    state = adapter.get_state()
    assert state.mission is not None
    assert state.mission.current_seq is None
    assert state.mission.total is None
    assert state.mission.state is None
    assert state.mission.mode is None
    assert state.mission.age_s is None


# -- routing boundary --------------------------------------------------------------


def test_mission_current_does_not_enter_transfer_inbox() -> None:
    """MISSION_CURRENT is not a transfer-transaction message: it must never
    be routed into the mission session inbox or satisfy a transaction wait."""

    adapter, _master = _adapter()
    adapter._on_mission_current(_mission_current(seq=1))
    assert adapter._mission_inbox == []


# -- operation integration -----------------------------------------------------------


def test_mission_start_operation_is_supported_kind() -> None:
    """The registry accepts mission_start operations with the same
    ownership/epoch/supersession semantics as the flight commands."""

    registry = OperationRegistry()
    operation = registry.activate(
        OperationKind.MISSION_START,
        effect_sent_monotonic=100.0,
        ack_monotonic=100.05,
        now_monotonic=100.0,
    )
    assert operation.kind is OperationKind.MISSION_START
    assert operation.state is OperationState.WAITING
    assert registry.is_active(operation.operation_id, operation.epoch) is True
