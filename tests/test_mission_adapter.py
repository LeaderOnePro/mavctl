"""Mock MAVLink transaction tests for the mission protocol (Phase 3A).

The adapter transaction runs on a background thread (the GCS side); the test
plays the vehicle from the main thread by injecting FakeMsg traffic through
``adapter._handle_message``. All timeouts are shrunk via monkeypatched module
constants so failure paths stay fast and deterministic.
"""

from __future__ import annotations

import threading
import time
from collections.abc import Callable
from typing import Any

import pytest

from mavctl.adapter.base import (
    MissionItemUnsupportedError,
    MissionProtocolError,
    MissionStateUncertainError,
)
from mavctl.adapter.pymavlink_adapter import PymavlinkAdapter, _MissionSequenceGapError
from mavctl.models import DownloadedMissionV1, MissionV1
from tests.fakes import FakeMaster, FakeMsg

_ARMED_FLAG = 0b10000000


@pytest.fixture
def fast_timeouts(monkeypatch: pytest.MonkeyPatch) -> None:
    import mavctl.adapter.pymavlink_adapter as mod

    monkeypatch.setattr(mod, "_MISSION_REQUEST_TIMEOUT_S", 0.05)
    monkeypatch.setattr(mod, "_MISSION_RETRIES", 1)
    monkeypatch.setattr(mod, "_MISSION_TRANSACTION_TIMEOUT_S", 0.5)
    monkeypatch.setattr(mod, "_MISSION_ACK_WINDOW_S", 0.05)
    monkeypatch.setattr(mod, "_MISSION_CLEAR_RESENDS", 0)
    monkeypatch.setattr(mod, "_MISSION_COUNT_RESENDS", 0)


def _hb() -> FakeMsg:
    return FakeMsg("HEARTBEAT", base_mode=0, custom_mode=0, system_status=3)


def _msg(msg_type: str, **fields: Any) -> FakeMsg:
    return FakeMsg(msg_type, src_system=1, src_component=1, **fields)


def _adapter() -> tuple[PymavlinkAdapter, FakeMaster]:
    adapter = PymavlinkAdapter("udp:127.0.0.1:14550")
    master = FakeMaster(flightmode="GUIDED")
    adapter._master = master
    adapter._on_heartbeat(_hb())  # locks onto sys=1 comp=1
    return adapter, master


def _mission(items: int = 4) -> MissionV1:
    """Exactly ``items`` items: takeoff first, (n-2) waypoints, then rtl
    (omitted for a single-item mission — a lone takeoff is valid)."""

    payload_items: list[dict[str, Any]] = [{"type": "takeoff", "altitude_m": 10.0}]
    for index in range(items - 2):
        payload_items.append(
            {"type": "waypoint", "lat_deg": 1.0 + index, "lon_deg": 2.0 + index,
             "altitude_m": 15.0}
        )
    if items >= 2:
        payload_items.append({"type": "rtl"})
    return MissionV1.model_validate({"version": 1, "items": payload_items})


def _item(seq: int, command: int = 16, frame: int = 6, **fields: Any) -> FakeMsg:
    base: dict[str, Any] = {
        "seq": seq, "frame": frame, "command": command, "current": 0,
        "autocontinue": 1, "param1": 0.0, "param2": 0.0, "param3": 0.0,
        "param4": 0.0, "x": 0, "y": 0, "z": 10.0, "mission_type": 0,
    }
    base.update(fields)
    return _msg("MISSION_ITEM_INT", **base)


def _wait_until(predicate: Callable[[], bool], timeout: float = 5.0) -> bool:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(0.01)
    return predicate()


def _sent(master: FakeMaster, name: str) -> list[tuple[Any, ...]]:
    return [entry for entry in master.sent if entry[0] == name]


def _item_seqs(master: FakeMaster) -> list[int]:
    return [entry[1][2] for entry in _sent(master, "MISSION_ITEM_INT")]


def _request_seqs(master: FakeMaster) -> list[int]:
    return [entry[1][2] for entry in _sent(master, "MISSION_REQUEST_INT")]


class _Runner:
    """Run a transaction on a background thread; the test plays the vehicle."""

    def __init__(self, fn: Callable[[], object]) -> None:
        self.result: Any = None
        self.error: BaseException | None = None
        self._thread = threading.Thread(target=self._run, args=(fn,), daemon=True)

    def _run(self, fn: Callable[[], object]) -> None:
        try:
            self.result = fn()
        except BaseException as exc:  # captured for assertions
            self.error = exc

    def start(self) -> None:
        self._thread.start()

    def join(self, timeout: float = 10.0) -> bool:
        self._thread.join(timeout)
        return not self._thread.is_alive()


def _play_upload(
    adapter: PymavlinkAdapter, master: FakeMaster, count: int, *, ack_type: int = 0
) -> bool:
    """Play the vehicle through a full upload: request each item in order,
    then send the terminal ACK."""

    if not _wait_until(lambda: len(_sent(master, "MISSION_COUNT")) >= 1):
        return False
    for seq in range(count):
        adapter._handle_message(_msg("MISSION_REQUEST_INT", seq=seq, mission_type=0))

        def item_sent(expected: int = seq, _master: FakeMaster = master) -> bool:
            return expected in _item_seqs(_master)

        if not _wait_until(item_sent):
            return False
    adapter._handle_message(_msg("MISSION_ACK", type=ack_type, mission_type=0))
    return True


def _start_download(
    adapter: PymavlinkAdapter, master: FakeMaster, count: int
) -> bool:
    """Play the vehicle's download prologue: answer MISSION_REQUEST_LIST with
    MISSION_COUNT and confirm the first item request went out."""

    if not _wait_until(lambda: len(_sent(master, "MISSION_REQUEST_LIST")) >= 1):
        return False
    adapter._handle_message(_msg("MISSION_COUNT", count=count, mission_type=0))
    if count == 0:
        return True
    return _wait_until(lambda: _request_seqs(master)[:1] == [0])


# -- upload ------------------------------------------------------------------


def test_upload_happy_path() -> None:
    adapter, master = _adapter()
    mission = _mission(4)
    runner = _Runner(lambda: adapter.upload_mission(mission))
    runner.start()
    assert _play_upload(adapter, master, 4)
    assert runner.join()
    assert runner.error is None
    outcome = runner.result
    assert outcome is not None
    assert outcome.accepted is True
    assert outcome.result_name == "ACCEPTED"
    assert outcome.item_count == 4
    assert outcome.accepted_upto == 3
    # vehicle-paced upload: the GCS never sends item requests — the vehicle
    # does. The GCS sends COUNT and the requested items only.
    assert _request_seqs(master) == []
    assert _item_seqs(master) == [0, 1, 2, 3]
    # takeoff wire encoding: frame 6, command 22, x/y zero
    takeoff = _sent(master, "MISSION_ITEM_INT")[0]
    assert takeoff[1][3] == 6
    assert takeoff[1][4] == 22
    assert takeoff[1][11] == 0
    assert takeoff[1][12] == 0


def test_upload_single_item_mission() -> None:
    adapter, master = _adapter()
    runner = _Runner(lambda: adapter.upload_mission(_mission(1)))
    runner.start()
    assert _play_upload(adapter, master, 1)
    assert runner.join()
    assert runner.error is None
    assert _item_seqs(master) == [0]


def test_upload_duplicate_request_resends_without_advancing(
    fast_timeouts: None,
) -> None:
    adapter, master = _adapter()
    runner = _Runner(lambda: adapter.upload_mission(_mission(2)))
    runner.start()
    assert _wait_until(lambda: len(_sent(master, "MISSION_COUNT")) >= 1)
    # request 0 → send 0
    adapter._handle_message(_msg("MISSION_REQUEST_INT", seq=0, mission_type=0))
    assert _wait_until(lambda: _item_seqs(master) == [0])
    # request 0 again (packet loss of the item) → resend 0, no advance
    adapter._handle_message(_msg("MISSION_REQUEST_INT", seq=0, mission_type=0))
    assert _wait_until(lambda: _item_seqs(master) == [0, 0])
    # a third duplicate is still answered with item 0
    adapter._handle_message(_msg("MISSION_REQUEST_INT", seq=0, mission_type=0))
    assert _wait_until(lambda: _item_seqs(master) == [0, 0, 0])
    # request 1 → send 1 (the expectation only advanced after item 0 was sent)
    adapter._handle_message(_msg("MISSION_REQUEST_INT", seq=1, mission_type=0))
    assert _wait_until(lambda: _item_seqs(master) == [0, 0, 0, 1])
    adapter._handle_message(_msg("MISSION_ACK", type=0, mission_type=0))
    assert runner.join()
    assert runner.error is None
    assert _request_seqs(master) == []  # requests are injected by the vehicle
    assert _item_seqs(master) == [0, 0, 0, 1]


def test_upload_out_of_range_request_is_uncertain(fast_timeouts: None) -> None:
    adapter, master = _adapter()
    runner = _Runner(lambda: adapter.upload_mission(_mission(4)))
    runner.start()
    assert _wait_until(lambda: len(_sent(master, "MISSION_COUNT")) >= 1)
    adapter._handle_message(_msg("MISSION_REQUEST_INT", seq=99, mission_type=0))
    assert runner.join(timeout=10)
    assert isinstance(runner.error, MissionStateUncertainError)
    assert isinstance(runner.error, _MissionSequenceGapError)
    assert _item_seqs(master) == []  # nothing sent for an invalid request
    # the gap detail identifies both sides of the violation
    assert runner.error.expected_seq == 0
    assert runner.error.requested_seq == 99


def test_upload_future_request_aborts_strict_sequence(fast_timeouts: None) -> None:
    """request 0 → send 0; request 2 while expected 1 → no item 2 is sent,
    the transaction aborts as uncertain with expected/requested detail, and
    later stale requests cannot pollute a following transaction."""

    adapter, master = _adapter()
    runner = _Runner(lambda: adapter.upload_mission(_mission(4)))
    runner.start()
    assert _wait_until(lambda: len(_sent(master, "MISSION_COUNT")) >= 1)
    adapter._handle_message(_msg("MISSION_REQUEST_INT", seq=0, mission_type=0))
    assert _wait_until(lambda: _item_seqs(master) == [0])
    adapter._handle_message(_msg("MISSION_REQUEST_INT", seq=2, mission_type=0))
    assert runner.join(timeout=10)
    assert isinstance(runner.error, _MissionSequenceGapError)
    assert runner.error.expected_seq == 1
    assert runner.error.requested_seq == 2
    assert _item_seqs(master) == [0]  # item 2 was never sent

    # stale requests after the abort are dropped (delivery inactive)…
    adapter._handle_message(_msg("MISSION_REQUEST_INT", seq=2, mission_type=0))
    assert _item_seqs(master) == [0]

    # …and a fresh transaction starts clean: the second COUNT triggers a new
    # session and the full ordered play succeeds again.
    second = _Runner(lambda: adapter.upload_mission(_mission(4)))
    second.start()
    assert _wait_until(lambda: len(_sent(master, "MISSION_COUNT")) >= 2)
    for seq in range(4):
        adapter._handle_message(_msg("MISSION_REQUEST_INT", seq=seq, mission_type=0))

        def item_once(expected: int = seq, _master: FakeMaster = master) -> bool:
            return _item_seqs(_master).count(expected) == 1

        assert _wait_until(item_once)
    adapter._handle_message(_msg("MISSION_ACK", type=0, mission_type=0))
    assert second.join()
    assert second.error is None
    assert second.result.accepted is True


def test_upload_wrong_source_ignored(fast_timeouts: None) -> None:
    adapter, master = _adapter()
    runner = _Runner(lambda: adapter.upload_mission(_mission(4)))
    runner.start()
    assert _wait_until(lambda: len(_sent(master, "MISSION_COUNT")) >= 1)
    # foreign system and foreign component both ignored
    adapter._handle_message(
        FakeMsg("MISSION_REQUEST_INT", src_system=2, src_component=1, seq=0, mission_type=0)
    )
    adapter._handle_message(
        FakeMsg("MISSION_REQUEST_INT", src_system=1, src_component=154, seq=0, mission_type=0)
    )
    assert runner.join(timeout=10)
    assert isinstance(runner.error, MissionStateUncertainError)
    assert _item_seqs(master) == []


def test_upload_wrong_mission_type_ignored(fast_timeouts: None) -> None:
    adapter, master = _adapter()
    runner = _Runner(lambda: adapter.upload_mission(_mission(4)))
    runner.start()
    assert _wait_until(lambda: len(_sent(master, "MISSION_COUNT")) >= 1)
    adapter._handle_message(_msg("MISSION_REQUEST_INT", seq=0, mission_type=1))  # fence
    assert runner.join(timeout=10)
    assert isinstance(runner.error, MissionStateUncertainError)
    assert _item_seqs(master) == []


def test_upload_terminal_rejected_ack(fast_timeouts: None) -> None:
    adapter, master = _adapter()
    runner = _Runner(lambda: adapter.upload_mission(_mission(4)))
    runner.start()
    assert _play_upload(adapter, master, 4, ack_type=4)  # NO_SPACE
    assert runner.join()
    assert isinstance(runner.error, MissionProtocolError)
    assert runner.error.result_name == "NO_SPACE"


def test_upload_operation_cancelled_is_uncertain(fast_timeouts: None) -> None:
    adapter, master = _adapter()
    runner = _Runner(lambda: adapter.upload_mission(_mission(2)))
    runner.start()
    assert _wait_until(lambda: len(_sent(master, "MISSION_COUNT")) >= 1)
    adapter._handle_message(_msg("MISSION_REQUEST_INT", seq=0, mission_type=0))
    assert _wait_until(lambda: _item_seqs(master) == [0])
    # the vehicle's 8 s upload timer fires mid-transfer
    adapter._handle_message(_msg("MISSION_ACK", type=15, mission_type=0))
    assert runner.join(timeout=10)
    assert isinstance(runner.error, MissionStateUncertainError)


def test_upload_timeout_is_uncertain_after_count(fast_timeouts: None) -> None:
    adapter, master = _adapter()
    runner = _Runner(lambda: adapter.upload_mission(_mission(4)))
    runner.start()
    assert _wait_until(lambda: len(_sent(master, "MISSION_COUNT")) >= 1)
    assert runner.join(timeout=10)
    assert isinstance(runner.error, MissionStateUncertainError)
    # COUNT went out (possibly re-sent), but no item is ever sent unrequested
    assert len(_sent(master, "MISSION_COUNT")) >= 1
    assert _item_seqs(master) == []


def test_upload_u2_timeout_is_uncertain_without_blind_resend(
    fast_timeouts: None,
) -> None:
    """U2 stall (item 0 sent on request 0, then silence): mavctl must NOT
    blind-re-send item 0 or send item 1 — the GCS cannot know whether the
    item was lost, accepted with the next request lost, or the vehicle
    entered an error state. It waits until the overall deadline, then reports
    remote_mission_state_uncertain with accepted_upto == 0."""

    adapter, master = _adapter()
    runner = _Runner(lambda: adapter.upload_mission(_mission(2)))
    runner.start()
    assert _wait_until(lambda: len(_sent(master, "MISSION_COUNT")) >= 1)
    adapter._handle_message(_msg("MISSION_REQUEST_INT", seq=0, mission_type=0))
    assert _wait_until(lambda: _item_seqs(master) == [0])
    # vehicle stalls: no request 1, no terminal ACK
    assert runner.join(timeout=10)
    assert isinstance(runner.error, MissionStateUncertainError)
    assert "item 0" in str(runner.error)  # last sent sequence in the message
    assert runner.error.accepted_upto == 0
    # no second item 0 (no blind resend), no future item 1
    assert _item_seqs(master) == [0]


def test_upload_u1_retries_count_then_uncertain(fast_timeouts: None) -> None:
    """U1: COUNT is re-sent only on per-attempt timeouts (never on other
    paths), and exhaustion reports uncertain with no items sent."""

    import mavctl.adapter.pymavlink_adapter as mod

    adapter, master = _adapter()
    runner = _Runner(lambda: adapter.upload_mission(_mission(2)))
    runner.start()
    assert runner.join(timeout=10)
    assert isinstance(runner.error, MissionStateUncertainError)
    # initial send + one resend per _MISSION_RETRIES attempt, timeouts only
    assert len(_sent(master, "MISSION_COUNT")) == 1 + mod._MISSION_RETRIES
    assert _item_seqs(master) == []


def test_stale_mission_message_cannot_satisfy_later_transaction(
    fast_timeouts: None,
) -> None:
    adapter, master = _adapter()
    # stale request arrives while no transaction is running
    adapter._handle_message(_msg("MISSION_REQUEST_INT", seq=0, mission_type=0))
    runner = _Runner(lambda: adapter.upload_mission(_mission(4)))
    runner.start()
    assert _wait_until(lambda: len(_sent(master, "MISSION_COUNT")) >= 1)
    assert runner.join(timeout=10)
    # the stale request was drained at session start and must not produce an item
    assert isinstance(runner.error, MissionStateUncertainError)
    assert _item_seqs(master) == []


def test_command_ack_does_not_satisfy_mission_wait(fast_timeouts: None) -> None:
    adapter, master = _adapter()
    runner = _Runner(lambda: adapter.upload_mission(_mission(1)))
    runner.start()
    assert _wait_until(lambda: len(_sent(master, "MISSION_COUNT")) >= 1)
    # a stray COMMAND_ACK must be irrelevant to the mission session
    adapter._handle_message(_msg("COMMAND_ACK", command=16, result=0))
    adapter._handle_message(_msg("MISSION_REQUEST_INT", seq=0, mission_type=0))
    assert _wait_until(lambda: _item_seqs(master) == [0])
    # the stray COMMAND_ACK went to the command machinery only — the mission
    # session still required its own REQUEST_INT before any item was sent
    assert set(adapter._acks) == {16}
    adapter._handle_message(_msg("MISSION_ACK", type=0, mission_type=0))
    assert runner.join()
    assert runner.error is None
    assert runner.result.accepted is True


def test_snapshot_readable_during_upload(fast_timeouts: None) -> None:
    adapter, _master = _adapter()
    release = threading.Event()

    def stalled_upload() -> None:
        # Emulate a transaction stuck waiting for the vehicle: the mission
        # session is active and _mission_lock is held the whole time.
        with adapter._mission_lock:
            adapter._begin_mission_session()
            release.wait(timeout=2.0)
            adapter._end_mission_session()

    runner = _Runner(lambda: stalled_upload())
    runner.start()
    assert _wait_until(lambda: adapter._mission_active)
    # fast handlers never block on mission or command locks
    state = adapter.get_state()
    assert state.connected is True
    assert adapter.get_telemetry() is not None
    release.set()
    assert runner.join(timeout=5.0)


# -- download ----------------------------------------------------------------


def test_download_happy_path() -> None:
    adapter, master = _adapter()
    runner = _Runner(lambda: adapter.download_mission())
    runner.start()
    assert _wait_until(lambda: len(_sent(master, "MISSION_REQUEST_LIST")) >= 1)
    adapter._handle_message(_msg("MISSION_COUNT", count=2, mission_type=0))
    assert _wait_until(lambda: _request_seqs(master) == [0])
    adapter._handle_message(
        _item(0, command=16, x=-353632621, y=1491652374, z=20.0, param1=1.0, param2=2.0)
    )
    assert _wait_until(lambda: _request_seqs(master) == [0, 1])
    adapter._handle_message(_item(1, command=20))
    assert runner.join()
    assert runner.error is None
    mission = runner.result
    assert mission is not None
    assert mission.version == 1 and len(mission.items) == 2
    assert mission.items[0].type == "waypoint"
    assert mission.items[0].lat_deg == pytest.approx(-35.3632621)
    assert mission.items[1].type == "rtl"
    # courtesy terminal ACK was sent
    assert len(_sent(master, "MISSION_ACK")) == 1


def test_download_empty_mission() -> None:
    adapter, master = _adapter()
    runner = _Runner(lambda: adapter.download_mission())
    runner.start()
    assert _wait_until(lambda: len(_sent(master, "MISSION_REQUEST_LIST")) >= 1)
    adapter._handle_message(_msg("MISSION_COUNT", count=0, mission_type=0))
    assert runner.join()
    assert runner.error is None
    mission = runner.result
    assert mission.items == []
    assert _sent(master, "MISSION_ACK")  # courtesy ACK still sent
    assert _sent(master, "MISSION_REQUEST_INT") == []


def test_download_duplicate_item_not_refetched() -> None:
    adapter, master = _adapter()
    runner = _Runner(lambda: adapter.download_mission())
    runner.start()
    assert _start_download(adapter, master, count=2)
    adapter._handle_message(_item(0))
    adapter._handle_message(_item(0))  # duplicate retransmission
    assert _wait_until(lambda: _request_seqs(master) == [0, 1])
    adapter._handle_message(_item(1, command=20))
    assert runner.join()
    assert runner.error is None
    assert len(runner.result.items) == 2
    assert _request_seqs(master) == [0, 1]  # seq 0 never re-requested


def test_download_out_of_order_buffered() -> None:
    adapter, master = _adapter()
    runner = _Runner(lambda: adapter.download_mission())
    runner.start()
    assert _start_download(adapter, master, count=2)
    adapter._handle_message(_item(1, command=20))  # arrives early: buffered
    adapter._handle_message(_item(0))
    assert runner.join()
    assert runner.error is None
    assert len(runner.result.items) == 2
    # seq 1 was already buffered, so it is never requested
    assert _request_seqs(master) == [0]


def test_download_missing_item_times_out_atomically(fast_timeouts: None) -> None:
    adapter, master = _adapter()
    runner = _Runner(lambda: adapter.download_mission())
    runner.start()
    assert _start_download(adapter, master, count=2)
    adapter._handle_message(_item(0))
    assert _wait_until(lambda: _request_seqs(master) == [0, 1])
    # seq 1 never arrives
    assert runner.join(timeout=10)
    assert isinstance(runner.error, MissionProtocolError)
    assert runner.error.result_name == "TIMEOUT"
    # no partial mission was emitted anywhere
    assert not isinstance(runner.result, DownloadedMissionV1)


def test_download_unsupported_command_atomic(fast_timeouts: None) -> None:
    adapter, master = _adapter()
    runner = _Runner(lambda: adapter.download_mission())
    runner.start()
    assert _start_download(adapter, master, count=2)
    adapter._handle_message(_item(0, command=999))
    assert runner.join(timeout=10)
    assert isinstance(runner.error, MissionItemUnsupportedError)
    assert runner.error.seq == 0
    assert runner.error.command == 999


def test_download_unsupported_frame_atomic(fast_timeouts: None) -> None:
    adapter, master = _adapter()
    runner = _Runner(lambda: adapter.download_mission())
    runner.start()
    assert _start_download(adapter, master, count=2)
    adapter._handle_message(_item(0, frame=3))  # GLOBAL_RELATIVE_ALT (float msg)
    assert runner.join(timeout=10)
    assert isinstance(runner.error, MissionItemUnsupportedError)
    assert runner.error.frame == 3


def test_download_wrong_source_ignored(fast_timeouts: None) -> None:
    adapter, master = _adapter()
    runner = _Runner(lambda: adapter.download_mission())
    runner.start()
    assert _start_download(adapter, master, count=2)
    adapter._handle_message(
        FakeMsg("MISSION_ITEM_INT", src_system=9, src_component=9, seq=0, frame=6,
                command=16, current=0, autocontinue=1, param1=0.0, param2=0.0,
                param3=0.0, param4=0.0, x=0, y=0, z=5.0, mission_type=0)
    )
    assert runner.join(timeout=10)
    assert isinstance(runner.error, MissionProtocolError)  # TIMEOUT: item ignored


def test_download_denied_by_vehicle(fast_timeouts: None) -> None:
    adapter, master = _adapter()
    runner = _Runner(lambda: adapter.download_mission())
    runner.start()
    assert _wait_until(lambda: len(_sent(master, "MISSION_REQUEST_LIST")) >= 1)
    adapter._handle_message(_msg("MISSION_ACK", type=14, mission_type=0))  # DENIED
    assert runner.join(timeout=10)
    assert isinstance(runner.error, MissionProtocolError)
    assert runner.error.result_name == "DENIED"


def test_download_courtesy_ack_failure_is_non_fatal(fast_timeouts: None) -> None:
    adapter, master = _adapter()

    def broken_ack(*_args: Any) -> None:
        raise RuntimeError("no buffer space")

    master.mav.mission_ack_send = broken_ack
    runner = _Runner(lambda: adapter.download_mission())
    runner.start()
    assert _start_download(adapter, master, count=1)
    adapter._handle_message(_item(0, command=22))
    assert runner.join(timeout=10)
    # the fully received mission still succeeds even though the courtesy ACK failed
    assert runner.error is None
    assert len(runner.result.items) == 1


def test_concurrent_downloads_serialize() -> None:
    # Real timeouts: runner1 holds _mission_lock while waiting for COUNT, so
    # runner2 must still be blocked while the lock is held.
    adapter, master = _adapter()
    runner1 = _Runner(lambda: adapter.download_mission())
    runner1.start()
    assert _wait_until(lambda: len(_sent(master, "MISSION_REQUEST_LIST")) >= 1)
    runner2 = _Runner(lambda: adapter.download_mission())
    runner2.start()
    time.sleep(0.3)
    assert runner2._thread.is_alive()  # blocked behind _mission_lock
    assert runner1.join(timeout=10)
    assert isinstance(runner1.error, MissionProtocolError)  # timed out, released
    assert runner2.join(timeout=15)
    # runner2 then runs its own session (also times out with no vehicle)
    assert isinstance(runner2.error, MissionProtocolError)
