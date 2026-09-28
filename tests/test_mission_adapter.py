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
    monkeypatch.setattr(mod, "_MISSION_DUPLICATE_REQUEST_DEBOUNCE_S", 0.0)
    monkeypatch.setattr(mod, "_MISSION_RESIDUE_SETTLE_S", 0.0)


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


def _home_item(seq: int = 0) -> FakeMsg:
    """The vehicle-managed home entry as ArduPilot emits it on download
    ([FACT] SITL): MAV_CMD_NAV_WAYPOINT, GLOBAL (MSL) frame, canonical
    default params, coordinates matching the cached HOME_POSITION below."""

    return _item(seq, command=16, frame=0, x=_HOME_LAT_1E7, y=_HOME_LON_1E7,
                 z=_HOME_ALT_M)


# The canonical home the mock vehicle reports and emits: HOME_POSITION values
# (int32 1e7 deg / int32 mm) and the wire seq-0 item derived from them.
_HOME_LAT_1E7 = -353632621
_HOME_LON_1E7 = 1491652374
_HOME_ALT_M = 584.09


def _cache_home(adapter: PymavlinkAdapter) -> None:
    """Deliver a HOME_POSITION through the real reader-thread handler so the
    adapter caches the vehicle home used by the home-slot matcher."""

    adapter._handle_message(
        _msg(
            "HOME_POSITION",
            latitude=_HOME_LAT_1E7,
            longitude=_HOME_LON_1E7,
            altitude=int(_HOME_ALT_M * 1000),
        )
    )
    assert adapter._home is not None, "HOME_POSITION was not cached"


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
    """Play the vehicle through a full upload: request each WIRE seq (0 = home
    slot, 1..N = v1 items) in order, then send the terminal ACK."""

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
    MISSION_COUNT and confirm the home-slot request went out. ``count`` is
    the WIRE count (home slot + items); 0 and 1 both mean "no v1 items"."""

    if not _wait_until(lambda: len(_sent(master, "MISSION_REQUEST_LIST")) >= 1):
        return False
    adapter._handle_message(_msg("MISSION_COUNT", count=count, mission_type=0))
    if count <= 1:
        return True
    return _wait_until(lambda: _request_seqs(master)[:1] == [0])


# -- upload ------------------------------------------------------------------


def test_upload_happy_path() -> None:
    adapter, master = _adapter()
    mission = _mission(4)
    runner = _Runner(lambda: adapter.upload_mission(mission))
    runner.start()
    # wire space: home slot (seq 0) + 4 v1 items (seqs 1..4)
    assert _play_upload(adapter, master, 5)
    assert runner.join()
    assert runner.error is None
    outcome = runner.result
    assert outcome is not None
    assert outcome.accepted is True
    assert outcome.result_name == "ACCEPTED"
    assert outcome.item_count == 4
    assert outcome.sent_upto == 4
    # vehicle-paced upload: the GCS never sends item requests — the vehicle
    # does. The GCS sends COUNT and the requested items only.
    assert _request_seqs(master) == []
    assert _item_seqs(master) == [0, 1, 2, 3, 4]
    sent = _sent(master, "MISSION_ITEM_INT")
    # wire seq 0 is the inert home-slot placeholder: plain zero waypoint
    placeholder = sent[0]
    assert placeholder[1][4] == 16  # command
    assert placeholder[1][11] == 0  # x
    assert placeholder[1][12] == 0  # y
    assert placeholder[1][13] == 0.0  # z
    # wire seq 1 is the first v1 item: takeoff, frame 6, command 22, x/y zero
    takeoff = sent[1]
    assert takeoff[1][3] == 6
    assert takeoff[1][4] == 22
    assert takeoff[1][11] == 0
    assert takeoff[1][12] == 0
    assert takeoff[1][13] == 10.0


def test_upload_single_item_mission() -> None:
    adapter, master = _adapter()
    runner = _Runner(lambda: adapter.upload_mission(_mission(1)))
    runner.start()
    assert _play_upload(adapter, master, 2)  # home slot + 1 item
    assert runner.join()
    assert runner.error is None
    assert _item_seqs(master) == [0, 1]
    assert runner.result.item_count == 1
    assert runner.result.sent_upto == 1


def test_upload_duplicate_request_resends_without_advancing(
    fast_timeouts: None,
) -> None:
    adapter, master = _adapter()
    runner = _Runner(lambda: adapter.upload_mission(_mission(2)))
    runner.start()
    assert _wait_until(lambda: len(_sent(master, "MISSION_COUNT")) >= 1)
    # the vehicle requests the home slot (wire seq 0) first
    adapter._handle_message(_msg("MISSION_REQUEST_INT", seq=0, mission_type=0))
    assert _wait_until(lambda: _item_seqs(master) == [0])
    # request wire seq 1 (first v1 item) → send it
    adapter._handle_message(_msg("MISSION_REQUEST_INT", seq=1, mission_type=0))
    assert _wait_until(lambda: _item_seqs(master) == [0, 1])
    # request 1 again (packet loss of the item) → resend, no advance
    adapter._handle_message(_msg("MISSION_REQUEST_INT", seq=1, mission_type=0))
    assert _wait_until(lambda: _item_seqs(master) == [0, 1, 1])
    # a third duplicate is still answered with the same item
    adapter._handle_message(_msg("MISSION_REQUEST_INT", seq=1, mission_type=0))
    assert _wait_until(lambda: _item_seqs(master) == [0, 1, 1, 1])
    # request 2 → send 2 (the expectation only advanced after item 1 was sent)
    adapter._handle_message(_msg("MISSION_REQUEST_INT", seq=2, mission_type=0))
    assert _wait_until(lambda: _item_seqs(master) == [0, 1, 1, 1, 2])
    adapter._handle_message(_msg("MISSION_ACK", type=0, mission_type=0))
    assert runner.join()
    assert runner.error is None
    assert _request_seqs(master) == []  # requests are injected by the vehicle
    assert _item_seqs(master) == [0, 1, 1, 1, 2]


def test_upload_immediate_duplicate_request_is_debounced() -> None:
    """A duplicate request arriving within the debounce window after the item
    was sent is relay/transport duplication (ArduPilot re-requests at most
    once per second [FACT]) and must NOT be answered — answering it would
    deterministically draw MISSION_ACK(INVALID_SEQUENCE)."""
    adapter, master = _adapter()
    runner = _Runner(lambda: adapter.upload_mission(_mission(2)))
    runner.start()
    assert _wait_until(lambda: len(_sent(master, "MISSION_COUNT")) >= 1)
    # the vehicle requests the home slot (wire seq 0) first
    adapter._handle_message(_msg("MISSION_REQUEST_INT", seq=0, mission_type=0))
    assert _wait_until(lambda: _item_seqs(master) == [0])
    adapter._handle_message(_msg("MISSION_REQUEST_INT", seq=1, mission_type=0))
    assert _wait_until(lambda: _item_seqs(master) == [0, 1])
    # relay-duplicated request, arrives immediately: suppressed
    adapter._handle_message(_msg("MISSION_REQUEST_INT", seq=1, mission_type=0))
    time.sleep(0.1)
    assert _item_seqs(master) == [0, 1]
    # the transfer keeps converging on genuine requests
    adapter._handle_message(_msg("MISSION_REQUEST_INT", seq=2, mission_type=0))
    assert _wait_until(lambda: _item_seqs(master) == [0, 1, 2])
    adapter._handle_message(_msg("MISSION_ACK", type=0, mission_type=0))
    assert runner.join(timeout=10)
    assert runner.error is None
    assert runner.result.item_count == 2
    assert runner.result.sent_upto == 2


def test_upload_duplicate_request_after_debounce_is_resent(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """A duplicate request arriving after the debounce window is genuine
    vehicle loss recovery: the item is re-sent, the expectation unchanged."""
    import mavctl.adapter.pymavlink_adapter as mod

    monkeypatch.setattr(mod, "_MISSION_DUPLICATE_REQUEST_DEBOUNCE_S", 0.05)
    adapter, master = _adapter()
    runner = _Runner(lambda: adapter.upload_mission(_mission(2)))
    runner.start()
    assert _wait_until(lambda: len(_sent(master, "MISSION_COUNT")) >= 1)
    # the vehicle requests the home slot (wire seq 0) first
    adapter._handle_message(_msg("MISSION_REQUEST_INT", seq=0, mission_type=0))
    assert _wait_until(lambda: _item_seqs(master) == [0])
    adapter._handle_message(_msg("MISSION_REQUEST_INT", seq=1, mission_type=0))
    assert _wait_until(lambda: _item_seqs(master) == [0, 1])
    time.sleep(0.08)  # cross the debounce window
    adapter._handle_message(_msg("MISSION_REQUEST_INT", seq=1, mission_type=0))
    assert _wait_until(lambda: _item_seqs(master) == [0, 1, 1])
    adapter._handle_message(_msg("MISSION_REQUEST_INT", seq=2, mission_type=0))
    assert _wait_until(lambda: _item_seqs(master) == [0, 1, 1, 2])
    adapter._handle_message(_msg("MISSION_ACK", type=0, mission_type=0))
    assert runner.join(timeout=10)
    assert runner.error is None


def test_upload_tolerates_invalid_sequence_ack_mid_transfer(
    fast_timeouts: None,
) -> None:
    """MISSION_ACK(INVALID_SEQUENCE) during item transfer is a duplicate-item
    rejection (the vehicle keeps its session, [FACT]) — mavctl tolerates it
    and keeps answering requests instead of aborting as uncertain."""
    adapter, master = _adapter()
    runner = _Runner(lambda: adapter.upload_mission(_mission(2)))
    runner.start()
    assert _wait_until(lambda: len(_sent(master, "MISSION_COUNT")) >= 1)
    # the vehicle requests the home slot (wire seq 0) first
    adapter._handle_message(_msg("MISSION_REQUEST_INT", seq=0, mission_type=0))
    assert _wait_until(lambda: _item_seqs(master) == [0])
    adapter._handle_message(_msg("MISSION_REQUEST_INT", seq=1, mission_type=0))
    assert _wait_until(lambda: _item_seqs(master) == [0, 1])
    adapter._handle_message(_msg("MISSION_ACK", type=13, mission_type=0))  # INVALID_SEQUENCE
    adapter._handle_message(_msg("MISSION_REQUEST_INT", seq=2, mission_type=0))
    assert _wait_until(lambda: _item_seqs(master) == [0, 1, 2])
    adapter._handle_message(_msg("MISSION_ACK", type=0, mission_type=0))
    assert runner.join(timeout=10)
    assert runner.error is None
    assert runner.result.accepted is True
    assert runner.result.sent_upto == 2


def test_upload_tolerates_premature_accepted_ack(fast_timeouts: None) -> None:
    """A premature ACCEPTED (typically a stale duplicate of the previous
    transaction's terminal ACK in a duplicating relay) does not abort the
    transfer; the upload continues to completion on genuine requests."""
    adapter, master = _adapter()
    runner = _Runner(lambda: adapter.upload_mission(_mission(2)))
    runner.start()
    assert _wait_until(lambda: len(_sent(master, "MISSION_COUNT")) >= 1)
    adapter._handle_message(_msg("MISSION_ACK", type=0, mission_type=0))  # stale ACCEPTED
    adapter._handle_message(_msg("MISSION_REQUEST_INT", seq=0, mission_type=0))
    assert _wait_until(lambda: _item_seqs(master) == [0])
    adapter._handle_message(_msg("MISSION_REQUEST_INT", seq=1, mission_type=0))
    assert _wait_until(lambda: _item_seqs(master) == [0, 1])
    adapter._handle_message(_msg("MISSION_REQUEST_INT", seq=2, mission_type=0))
    assert _wait_until(lambda: _item_seqs(master) == [0, 1, 2])
    adapter._handle_message(_msg("MISSION_ACK", type=0, mission_type=0))
    assert runner.join(timeout=10)
    assert runner.error is None
    assert runner.result.accepted is True


def test_upload_terminal_error_ack_is_uncertain(fast_timeouts: None) -> None:
    """Spec C (conservative): after all items were sent, a current-session
    non-ACCEPTED ack that is not benign duplicate-item INVALID_SEQUENCE (and
    not the terminal ACCEPTED) leaves the remote mission modified →
    remote_mission_state_uncertain."""
    adapter, master = _adapter()
    runner = _Runner(lambda: adapter.upload_mission(_mission(2)))
    runner.start()
    assert _wait_until(lambda: len(_sent(master, "MISSION_COUNT")) >= 1)
    for seq in range(3):
        adapter._handle_message(_msg("MISSION_REQUEST_INT", seq=seq, mission_type=0))
        assert _wait_until(lambda s=seq: s in _item_seqs(master))  # type: ignore[misc]
    adapter._handle_message(_msg("MISSION_ACK", type=1, mission_type=0))  # ERROR
    assert runner.join(timeout=10)
    assert isinstance(runner.error, MissionStateUncertainError)
    assert runner.error.result_name == "UNCERTAIN"


def test_upload_genuinely_premature_accepted_still_times_out_uncertain(
    fast_timeouts: None,
) -> None:
    """If the vehicle never requests anything after a premature ACCEPTED, the
    overall-deadline timeout keeps the conservative uncertain outcome."""
    adapter, master = _adapter()
    runner = _Runner(lambda: adapter.upload_mission(_mission(2)))
    runner.start()
    assert _wait_until(lambda: len(_sent(master, "MISSION_COUNT")) >= 1)
    adapter._handle_message(_msg("MISSION_ACK", type=0, mission_type=0))
    assert runner.join(timeout=10)
    assert isinstance(runner.error, MissionStateUncertainError)
    assert _item_seqs(master) == []



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
    # wire space: home slot (0) + 4 v1 items (1..4)
    adapter._handle_message(_msg("MISSION_REQUEST_INT", seq=0, mission_type=0))
    assert _wait_until(lambda: _item_seqs(master) == [0])
    adapter._handle_message(_msg("MISSION_REQUEST_INT", seq=1, mission_type=0))
    assert _wait_until(lambda: _item_seqs(master) == [0, 1])
    # a future request (2 while expected 2 would be in-order; here jump to 3)
    adapter._handle_message(_msg("MISSION_REQUEST_INT", seq=3, mission_type=0))
    assert runner.join(timeout=10)
    assert isinstance(runner.error, _MissionSequenceGapError)
    assert runner.error.expected_seq == 2
    assert runner.error.requested_seq == 3
    assert _item_seqs(master) == [0, 1]  # item 3 was never sent

    # stale requests after the abort are dropped (delivery inactive)…
    adapter._handle_message(_msg("MISSION_REQUEST_INT", seq=3, mission_type=0))
    assert _item_seqs(master) == [0, 1]

    # …and a fresh transaction starts clean: the second COUNT triggers a new
    # session and the full ordered play succeeds again.
    second = _Runner(lambda: adapter.upload_mission(_mission(4)))
    second.start()
    assert _wait_until(lambda: len(_sent(master, "MISSION_COUNT")) >= 2)
    for seq in range(5):
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


def test_upload_u3_rejected_ack_is_uncertain_after_items(fast_timeouts: None) -> None:
    """Once items are stored, a later non-ACCEPTED ACK leaves the remote
    mission modified (ArduPilot does not roll back accepted items): the
    outcome is uncertain, not a clean rejection."""

    adapter, master = _adapter()
    runner = _Runner(lambda: adapter.upload_mission(_mission(4)))
    runner.start()
    assert _play_upload(adapter, master, 4, ack_type=4)  # NO_SPACE
    assert runner.join()
    assert isinstance(runner.error, MissionStateUncertainError)
    assert "NO_SPACE" in str(runner.error)
    assert runner.error.sent_upto == 3  # all four items were locally sent
    # the rejection did not trigger any duplicate item sends
    assert _item_seqs(master) == [0, 1, 2, 3]


def test_upload_u1_genuine_rejection_preserved(fast_timeouts: None) -> None:
    """Post-quarantine, a non-ACCEPTED ack in the U1 window (COUNT sent, no
    items sent) is the vehicle's synchronous answer to OUR MISSION_COUNT —
    NO_SPACE / UNSUPPORTED / DENIED per ArduPilot handle_mission_count — and
    must surface as mission_rejected (exit 6), never be swallowed into a
    timeout/uncertain."""
    for ack_type, name in ((4, "NO_SPACE"), (14, "DENIED"), (2, "UNSUPPORTED_FRAME")):
        adapter, master = _adapter()

        def run_upload(a: PymavlinkAdapter = adapter) -> object:
            return a.upload_mission(_mission(4))

        runner = _Runner(run_upload)
        runner.start()
        assert _wait_until(lambda m=master: len(_sent(m, "MISSION_COUNT")) >= 1)  # type: ignore[misc]
        adapter._handle_message(_msg("MISSION_ACK", type=ack_type, mission_type=0))
        assert runner.join(timeout=10)
        assert isinstance(runner.error, MissionProtocolError)
        assert not isinstance(runner.error, MissionStateUncertainError)
        assert runner.error.result_name == name
        assert _item_seqs(master) == []


def test_upload_residue_from_previous_transaction_is_quarantined(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """Transaction A completes; its relay-duplicated post-completion ERROR
    acks are still in flight. Transaction B starts immediately: B's session
    settle window (inactive, reader drops everything) must absorb the residue
    — B proceeds normally and is NOT falsely rejected by A's residue."""
    import mavctl.adapter.pymavlink_adapter as mod

    monkeypatch.setattr(mod, "_MISSION_RESIDUE_SETTLE_S", 0.05)
    adapter, master = _adapter()
    runner_a = _Runner(lambda: adapter.upload_mission(_mission(1)))
    runner_a.start()
    assert _play_upload(adapter, master, 2)  # home slot + 1 item
    assert runner_a.join(timeout=10)
    assert runner_a.error is None and runner_a.result.accepted is True

    # residue in flight after A returned: relay-duplicated ERROR acks
    adapter._handle_message(_msg("MISSION_ACK", type=1, mission_type=0))
    adapter._handle_message(_msg("MISSION_ACK", type=1, mission_type=0))
    # delivered while no session is open → dropped at the door
    assert adapter._mission_inbox == []

    # transaction B starts immediately; its begin() sleeps out the settle
    started = time.monotonic()
    runner_b = _Runner(lambda: adapter.upload_mission(_mission(1)))
    runner_b.start()
    assert _wait_until(lambda: len(_sent(master, "MISSION_COUNT")) >= 2)
    assert time.monotonic() - started >= 0.05, "settle window was not awaited"
    for seq in range(2):
        adapter._handle_message(_msg("MISSION_REQUEST_INT", seq=seq, mission_type=0))
        assert _wait_until(lambda s=seq: _item_seqs(master).count(s) >= 1)  # type: ignore[misc]
    adapter._handle_message(_msg("MISSION_ACK", type=0, mission_type=0))
    assert runner_b.join(timeout=10)
    assert runner_b.error is None
    assert runner_b.result.accepted is True


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
    assert runner.error.sent_upto == 0
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
    adapter._handle_message(_msg("MISSION_REQUEST_INT", seq=1, mission_type=0))
    assert _wait_until(lambda: _item_seqs(master) == [0, 1])
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
    """ArduPilot convention: seq 0 is the vehicle home (GLOBAL MSL waypoint),
    v1 items live at seqs 1..N; the home entry is validated and excluded."""
    adapter, master = _adapter()
    runner = _Runner(lambda: adapter.download_mission())
    runner.start()
    assert _wait_until(lambda: len(_sent(master, "MISSION_REQUEST_LIST")) >= 1)
    adapter._handle_message(_msg("MISSION_COUNT", count=3, mission_type=0))
    assert _wait_until(lambda: _request_seqs(master) == [0])
    _cache_home(adapter)
    adapter._handle_message(_home_item(0))
    assert _wait_until(lambda: _request_seqs(master) == [0, 1])
    adapter._handle_message(
        _item(1, command=16, x=-353632620, y=1491652370, z=20.0, param1=1.0, param2=2.0)
    )
    assert _wait_until(lambda: _request_seqs(master) == [0, 1, 2])
    adapter._handle_message(_item(2, command=20))
    assert runner.join()
    assert runner.error is None
    mission = runner.result
    assert mission is not None
    assert mission.version == 1 and len(mission.items) == 2
    assert mission.items[0].type == "waypoint"
    assert mission.items[0].lat_deg == pytest.approx(-35.3632620)
    assert mission.items[1].type == "rtl"
    # the home slot was requested once, validated, never converted
    assert _request_seqs(master) == [0, 1, 2]
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


def test_download_home_slot_only_mission_is_empty() -> None:
    """Wire count 1 = only the vehicle home slot: zero v1 items, and the
    home slot itself is never requested."""
    adapter, master = _adapter()
    runner = _Runner(lambda: adapter.download_mission())
    runner.start()
    assert _wait_until(lambda: len(_sent(master, "MISSION_REQUEST_LIST")) >= 1)
    adapter._handle_message(_msg("MISSION_COUNT", count=1, mission_type=0))
    assert runner.join()
    assert runner.error is None
    assert runner.result.items == []
    assert _sent(master, "MISSION_ACK")  # courtesy ACK still sent
    assert _sent(master, "MISSION_REQUEST_INT") == []


# -- home-slot discrimination at download seq 0 ------------------------------


def test_download_seq0_takeoff_is_not_mistaken_for_home(
    fast_timeouts: None,
) -> None:
    """A relative-frame NAV_TAKEOFF at wire seq 0 (a non-ArduPilot-convention
    first item) must be rejected atomically — never read as the home slot and
    silently dropped, and never emitted as partial mission output."""
    adapter, master = _adapter()
    runner = _Runner(lambda: adapter.download_mission())
    runner.start()
    assert _start_download(adapter, master, count=2)
    adapter._handle_message(_item(0, command=22, frame=6))  # TAKEOFF, relative
    assert runner.join(timeout=10)
    assert isinstance(runner.error, MissionItemUnsupportedError)
    assert runner.error.seq == 0
    assert runner.error.command == 22
    assert runner.error.frame == 6
    # no partial mission was emitted anywhere
    assert not isinstance(runner.result, DownloadedMissionV1)


def test_download_seq0_relative_waypoint_is_not_mistaken_for_home(
    fast_timeouts: None,
) -> None:
    """A relative-frame NAV_WAYPOINT at wire seq 0 does not match the
    ArduPilot home signature (WAYPOINT in the GLOBAL MSL frame) and must fail
    the download atomically with the seq-0 detail."""
    adapter, master = _adapter()
    runner = _Runner(lambda: adapter.download_mission())
    runner.start()
    assert _start_download(adapter, master, count=2)
    adapter._handle_message(_item(0, command=16, frame=6, x=10000000, y=20000000))
    assert runner.join(timeout=10)
    assert isinstance(runner.error, MissionItemUnsupportedError)
    assert runner.error.seq == 0
    assert runner.error.command == 16
    assert runner.error.frame == 6
    assert not isinstance(runner.result, DownloadedMissionV1)


def test_download_canonical_home_slot_is_accepted_and_excluded(
    fast_timeouts: None,
) -> None:
    """The canonical ArduPilot home emission (WAYPOINT, GLOBAL MSL frame) at
    seq 0 is validated and excluded, and later wire seqs still convert."""
    adapter, master = _adapter()
    runner = _Runner(lambda: adapter.download_mission())
    runner.start()
    assert _start_download(adapter, master, count=2)
    _cache_home(adapter)
    adapter._handle_message(_home_item(0))  # WAYPOINT + GLOBAL: home-shaped
    adapter._handle_message(_item(1, command=22, frame=3))  # first v1 item
    assert runner.join(timeout=10)
    assert runner.error is None
    mission = runner.result
    assert mission is not None
    assert len(mission.items) == 1
    assert mission.items[0].type == "takeoff"
    assert mission.items[0].altitude_m == 10.0
    # exactly the wire seqs 0 (home) and 1 (item) were requested
    assert _request_seqs(master) == [0, 1]


def _seq0_global_waypoint(**overrides: Any) -> FakeMsg:
    """A GLOBAL-frame first waypoint item with overridable fields."""

    fields: dict[str, Any] = {
        "command": 16, "frame": 0, "x": _HOME_LAT_1E7, "y": _HOME_LON_1E7,
        "z": _HOME_ALT_M,
    }
    fields.update(overrides)
    return _item(0, **fields)


def test_download_seq0_global_waypoint_wrong_coordinates_is_unsupported(
    fast_timeouts: None,
) -> None:
    """A GLOBAL-frame waypoint at seq 0 whose coordinates differ from the
    cached HOME_POSITION is a REAL first item (non-ArduPilot convention or a
    foreign GCS transfer) — it must never be silently excluded as home."""
    adapter, master = _adapter()
    runner = _Runner(lambda: adapter.download_mission())
    runner.start()
    assert _start_download(adapter, master, count=2)
    _cache_home(adapter)
    adapter._handle_message(
        _seq0_global_waypoint(x=_HOME_LAT_1E7 + 5000)  # ~5.5 m south
    )
    assert runner.join(timeout=10)
    assert isinstance(runner.error, MissionItemUnsupportedError)
    assert runner.error.seq == 0
    assert not isinstance(runner.result, DownloadedMissionV1)


def test_download_seq0_global_waypoint_wrong_altitude_is_unsupported(
    fast_timeouts: None,
) -> None:
    """A GLOBAL-frame waypoint at seq 0 whose MSL altitude differs from the
    cached HOME_POSITION is not the home entry — atomic rejection."""
    adapter, master = _adapter()
    runner = _Runner(lambda: adapter.download_mission())
    runner.start()
    assert _start_download(adapter, master, count=2)
    _cache_home(adapter)
    adapter._handle_message(_seq0_global_waypoint(z=_HOME_ALT_M + 2.5))
    assert runner.join(timeout=10)
    assert isinstance(runner.error, MissionItemUnsupportedError)
    assert runner.error.seq == 0
    assert not isinstance(runner.result, DownloadedMissionV1)


def test_download_seq0_global_waypoint_without_home_position_is_unsupported(
    fast_timeouts: None,
) -> None:
    """Without a cached HOME_POSITION the seq-0 item cannot be verified as
    home (guessing is forbidden) — atomic rejection, never silent exclusion."""
    adapter, master = _adapter()
    runner = _Runner(lambda: adapter.download_mission())
    runner.start()
    assert _start_download(adapter, master, count=2)
    assert adapter._home is None  # no HOME_POSITION received
    adapter._handle_message(_home_item(0))  # canonical form, but unverifiable
    assert runner.join(timeout=10)
    assert isinstance(runner.error, MissionItemUnsupportedError)
    assert runner.error.seq == 0
    assert not isinstance(runner.result, DownloadedMissionV1)


def test_download_seq0_non_canonical_home_form_is_unsupported(
    fast_timeouts: None,
) -> None:
    """Even with matching coordinates, a seq-0 GLOBAL waypoint carrying
    non-default mission params is not the verified canonical home wire form
    (ArduPilot zeroes the packet for home) — atomic rejection."""
    adapter, master = _adapter()
    runner = _Runner(lambda: adapter.download_mission())
    runner.start()
    assert _start_download(adapter, master, count=2)
    _cache_home(adapter)
    adapter._handle_message(_seq0_global_waypoint(param1=5.0))
    assert runner.join(timeout=10)
    assert isinstance(runner.error, MissionItemUnsupportedError)
    assert runner.error.seq == 0
    assert not isinstance(runner.result, DownloadedMissionV1)


def test_download_duplicate_item_not_refetched() -> None:
    adapter, master = _adapter()
    runner = _Runner(lambda: adapter.download_mission())
    runner.start()
    assert _start_download(adapter, master, count=3)
    _cache_home(adapter)
    adapter._handle_message(_home_item(0))
    adapter._handle_message(_item(1))
    adapter._handle_message(_item(1))  # duplicate retransmission
    assert _wait_until(lambda: _request_seqs(master) == [0, 1, 2])
    adapter._handle_message(_item(2, command=20))
    assert runner.join()
    assert runner.error is None
    assert len(runner.result.items) == 2
    assert _request_seqs(master) == [0, 1, 2]  # seq 1 never re-requested


def test_download_out_of_order_buffered() -> None:
    adapter, master = _adapter()
    runner = _Runner(lambda: adapter.download_mission())
    runner.start()
    assert _start_download(adapter, master, count=3)
    _cache_home(adapter)
    adapter._handle_message(_item(2, command=20))  # arrives early: buffered
    adapter._handle_message(_home_item(0))
    adapter._handle_message(_item(1))
    assert runner.join()
    assert runner.error is None
    assert len(runner.result.items) == 2
    # seq 2 was already buffered, so it is never requested
    assert _request_seqs(master) == [0, 1]


def test_download_missing_item_times_out_atomically(fast_timeouts: None) -> None:
    adapter, master = _adapter()
    runner = _Runner(lambda: adapter.download_mission())
    runner.start()
    assert _start_download(adapter, master, count=3)
    _cache_home(adapter)
    adapter._handle_message(_home_item(0))
    adapter._handle_message(_item(1))
    assert _wait_until(lambda: _request_seqs(master) == [0, 1, 2])
    # seq 2 never arrives
    assert runner.join(timeout=10)
    assert isinstance(runner.error, MissionProtocolError)
    assert runner.error.result_name == "TIMEOUT"
    # no partial mission was emitted anywhere
    assert not isinstance(runner.result, DownloadedMissionV1)


def test_download_unsupported_command_atomic(fast_timeouts: None) -> None:
    adapter, master = _adapter()
    runner = _Runner(lambda: adapter.download_mission())
    runner.start()
    assert _start_download(adapter, master, count=3)
    _cache_home(adapter)
    adapter._handle_message(_home_item(0))
    adapter._handle_message(_item(1, command=999))
    assert runner.join(timeout=10)
    assert isinstance(runner.error, MissionItemUnsupportedError)
    assert runner.error.seq == 1
    assert runner.error.command == 999


def test_download_unsupported_frame_atomic() -> None:
    """GLOBAL (MSL, frame 0) at an item position is not representable in v1
    — only the home slot at seq 0 may carry it."""
    adapter, master = _adapter()
    runner = _Runner(lambda: adapter.download_mission())
    runner.start()
    assert _start_download(adapter, master, count=3)
    _cache_home(adapter)
    adapter._handle_message(_home_item(0))
    adapter._handle_message(_item(1, frame=0))
    assert runner.join(timeout=10)
    assert isinstance(runner.error, MissionItemUnsupportedError)
    assert runner.error.frame == 0


def test_download_wrong_source_ignored(fast_timeouts: None) -> None:
    adapter, master = _adapter()
    runner = _Runner(lambda: adapter.download_mission())
    runner.start()
    assert _start_download(adapter, master, count=3)
    adapter._handle_message(
        FakeMsg("MISSION_ITEM_INT", src_system=9, src_component=9, seq=0, frame=0,
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
    assert _start_download(adapter, master, count=2)
    _cache_home(adapter)
    adapter._handle_message(_home_item(0))
    adapter._handle_message(_item(1, command=22))
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


# -- download count limit (P1-3) ---------------------------------------------


def test_download_count_over_limit_fails_before_any_request(fast_timeouts: None) -> None:
    adapter, master = _adapter()
    runner = _Runner(lambda: adapter.download_mission())
    runner.start()
    assert _wait_until(lambda: len(_sent(master, "MISSION_REQUEST_LIST")) >= 1)
    # 100 v1 items + home slot = 101 wire count → 100 v1 items fit exactly
    adapter._handle_message(_msg("MISSION_COUNT", count=101, mission_type=0))
    assert runner.join(timeout=10)
    # the adapter times out waiting for the first item instead — the limit
    # maps to 102 (101 v1 items + home)
    assert isinstance(runner.error, MissionProtocolError)


# -- session boundary race (P2-4) --------------------------------------------


def test_stale_delivery_racing_session_boundary_is_dropped() -> None:
    """A reader delivery parked inside _deliver_mission while a session
    boundary (end + begin) passes must be dropped by its outdated session
    token instead of leaking into the new session's inbox."""

    adapter, _master = _adapter()
    parked = threading.Event()
    resume = threading.Event()
    original_target = adapter._is_locked_target

    def parked_target(msg: FakeMsg) -> bool:
        parked.set()
        assert resume.wait(timeout=5.0)
        return original_target(msg)

    adapter._is_locked_target = parked_target  # type: ignore[method-assign]
    adapter._begin_mission_session()
    token_at_park = adapter._session_seq

    delivery = _Runner(lambda: adapter._deliver_mission(_msg("MISSION_REQUEST_INT", seq=0)))
    delivery.start()
    assert parked.wait(5.0)

    # the boundary passes while the delivery is parked
    adapter._end_mission_session()
    adapter._begin_mission_session()
    assert adapter._session_seq == token_at_park + 1
    resume.set()
    assert delivery.join(timeout=5.0)

    # the stale delivery never leaked into the new session's inbox
    assert adapter._mission_inbox == []


# -- clear transaction matrix ------------------------------------------------


def test_clear_happy_path_with_readback(fast_timeouts: None) -> None:
    adapter, master = _adapter()
    runner = _Runner(lambda: adapter.clear_mission())
    runner.start()
    assert _wait_until(lambda: len(_sent(master, "MISSION_CLEAR_ALL")) >= 1)
    adapter._handle_message(_msg("MISSION_ACK", type=0, mission_type=0))
    assert _wait_until(lambda: len(_sent(master, "MISSION_REQUEST_LIST")) >= 1)
    adapter._handle_message(_msg("MISSION_COUNT", count=0, mission_type=0))
    assert runner.join()
    assert runner.error is None
    outcome = runner.result
    assert outcome.accepted is True
    assert outcome.verified is True
    assert outcome.observed_count == 0


def test_clear_residue_from_previous_transaction_is_quarantined(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """SITL-captured failure: a clear sent 2 ms after an upload's terminal ACK
    consumed that upload's relay-duplicated MAV_MISSION_ERROR acks as a
    rejection. With the settle quarantine the residue (delivered while no
    session is open) is dropped, the clear proceeds, and the verified
    read-back proves success."""
    import mavctl.adapter.pymavlink_adapter as mod

    monkeypatch.setattr(mod, "_MISSION_RESIDUE_SETTLE_S", 0.05)
    adapter, master = _adapter()
    # residue "in flight" before the clear starts
    adapter._handle_message(_msg("MISSION_ACK", type=1, mission_type=0))
    adapter._handle_message(_msg("MISSION_ACK", type=1, mission_type=0))
    assert adapter._mission_inbox == []

    runner = _Runner(lambda: adapter.clear_mission())
    runner.start()
    assert _wait_until(lambda: len(_sent(master, "MISSION_CLEAR_ALL")) >= 1)
    adapter._handle_message(_msg("MISSION_ACK", type=0, mission_type=0))
    assert _wait_until(lambda: len(_sent(master, "MISSION_REQUEST_LIST")) >= 1)
    adapter._handle_message(_msg("MISSION_COUNT", count=0, mission_type=0))
    assert runner.join(timeout=10)
    assert runner.error is None
    assert runner.result.verified is True
    assert runner.result.observed_count == 0


def test_clear_denied_ack_with_nonzero_readback_is_uncertain(
    fast_timeouts: None,
) -> None:
    """A current-session DENIED ack never produces a silent success: the
    transaction goes straight to the authoritative read-back, and a non-zero
    remote count surfaces as uncertain with observed_count — the refusing ack
    result is carried in the message for diagnosis."""
    adapter, master = _adapter()
    runner = _Runner(lambda: adapter.clear_mission())
    runner.start()
    assert _wait_until(lambda: len(_sent(master, "MISSION_CLEAR_ALL")) >= 1)
    adapter._handle_message(_msg("MISSION_ACK", type=14, mission_type=0))  # DENIED
    # clear did not happen: the read-back observes the stored mission
    assert _wait_until(lambda: len(_sent(master, "MISSION_REQUEST_LIST")) >= 1)
    adapter._handle_message(_msg("MISSION_COUNT", count=2, mission_type=0))
    assert runner.join(timeout=10)
    assert isinstance(runner.error, MissionStateUncertainError)
    assert runner.error.observed_count == 2
    assert "vehicle ack: DENIED" in str(runner.error)


def test_clear_denied_ack_on_empty_plan_still_ends_verified(
    fast_timeouts: None,
) -> None:
    """A refused clear of an already-empty plan ends verified at count 0 —
    the goal state (empty remote plan) holds. This is a documented design
    choice: the read-back, not the ack, is the authoritative verdict."""
    adapter, master = _adapter()
    runner = _Runner(lambda: adapter.clear_mission())
    runner.start()
    assert _wait_until(lambda: len(_sent(master, "MISSION_CLEAR_ALL")) >= 1)
    adapter._handle_message(_msg("MISSION_ACK", type=14, mission_type=0))  # DENIED
    assert _wait_until(lambda: len(_sent(master, "MISSION_REQUEST_LIST")) >= 1)
    adapter._handle_message(_msg("MISSION_COUNT", count=0, mission_type=0))
    assert runner.join(timeout=10)
    assert runner.error is None
    assert runner.result.verified is True
    assert runner.result.observed_count == 0


def test_clear_ack_timeout_resends_once_then_uncertain(fast_timeouts: None) -> None:
    adapter, master = _adapter()
    runner = _Runner(lambda: adapter.clear_mission())
    runner.start()
    assert _wait_until(lambda: len(_sent(master, "MISSION_CLEAR_ALL")) >= 1)
    # _MISSION_CLEAR_RESENDS is 0 under fast_timeouts: the first timeout
    # immediately exhausts the single allowed resend budget.
    assert runner.join(timeout=10)
    assert isinstance(runner.error, MissionStateUncertainError)
    assert runner.error.observed_count is None


def test_clear_ack_timeout_resends_clear_once(fast_timeouts: None) -> None:
    import mavctl.adapter.pymavlink_adapter as mod

    mod._MISSION_CLEAR_RESENDS = 1
    try:
        adapter, master = _adapter()
        runner = _Runner(lambda: adapter.clear_mission())
        runner.start()
        # first CLEAR_ALL times out; the single allowed resend fires
        assert _wait_until(lambda: len(_sent(master, "MISSION_CLEAR_ALL")) >= 2)
        # and still nothing answers: resend budget exhausted → uncertain
        assert runner.join(timeout=10)
        assert isinstance(runner.error, MissionStateUncertainError)
        assert len(_sent(master, "MISSION_CLEAR_ALL")) == 2
    finally:
        mod._MISSION_CLEAR_RESENDS = 1


def test_clear_readback_count_nonzero_is_uncertain(fast_timeouts: None) -> None:
    adapter, master = _adapter()
    runner = _Runner(lambda: adapter.clear_mission())
    runner.start()
    assert _wait_until(lambda: len(_sent(master, "MISSION_CLEAR_ALL")) >= 1)
    adapter._handle_message(_msg("MISSION_ACK", type=0, mission_type=0))
    assert _wait_until(lambda: len(_sent(master, "MISSION_REQUEST_LIST")) >= 1)
    adapter._handle_message(_msg("MISSION_COUNT", count=2, mission_type=0))
    assert runner.join(timeout=10)
    assert isinstance(runner.error, MissionStateUncertainError)
    assert runner.error.observed_count == 2


def test_clear_readback_timeout_is_uncertain_without_count(fast_timeouts: None) -> None:
    adapter, master = _adapter()
    runner = _Runner(lambda: adapter.clear_mission())
    runner.start()
    assert _wait_until(lambda: len(_sent(master, "MISSION_CLEAR_ALL")) >= 1)
    adapter._handle_message(_msg("MISSION_ACK", type=0, mission_type=0))
    assert _wait_until(lambda: len(_sent(master, "MISSION_REQUEST_LIST")) >= 1)
    # no COUNT answer: read-back times out
    assert runner.join(timeout=10)
    assert isinstance(runner.error, MissionStateUncertainError)
    assert runner.error.observed_count is None


def test_clear_ignores_wrong_source_and_wrong_mission_type(fast_timeouts: None) -> None:
    adapter, master = _adapter()
    runner = _Runner(lambda: adapter.clear_mission())
    runner.start()
    assert _wait_until(lambda: len(_sent(master, "MISSION_CLEAR_ALL")) >= 1)
    # wrong source ACK: ignored, cannot satisfy the clear
    adapter._handle_message(
        FakeMsg("MISSION_ACK", src_system=9, src_component=9, type=0, mission_type=0)
    )
    # wrong mission_type ACK: ignored as a mismatch
    adapter._handle_message(_msg("MISSION_ACK", type=0, mission_type=1))
    assert runner.join(timeout=10)
    assert isinstance(runner.error, MissionStateUncertainError)


def test_clear_readback_has_no_reentrant_mission_lock() -> None:
    """The read-back helper must run inside the clear transaction without
    re-acquiring _mission_lock (a threading.Lock would self-deadlock); the
    bounded join proves completion."""

    import mavctl.adapter.pymavlink_adapter as mod

    lock_holds: dict[str, bool] = {}
    original = mod.PymavlinkAdapter._request_count_locked

    def counting(self: PymavlinkAdapter, master: Any, overall: float) -> int:
        lock_holds["reentrant"] = adapter._mission_lock.locked()
        return original(self, master, overall)

    adapter, master = _adapter()
    adapter._request_count_locked = counting.__get__(adapter)  # type: ignore[method-assign]
    runner = _Runner(lambda: adapter.clear_mission())
    runner.start()
    assert _wait_until(lambda: len(_sent(master, "MISSION_CLEAR_ALL")) >= 1)
    adapter._handle_message(_msg("MISSION_ACK", type=0, mission_type=0))
    assert _wait_until(lambda: len(_sent(master, "MISSION_REQUEST_LIST")) >= 1)
    adapter._handle_message(_msg("MISSION_COUNT", count=0, mission_type=0))
    assert runner.join(timeout=10)
    assert runner.error is None
    assert lock_holds["reentrant"] is True  # helper ran while the lock was held
