"""pymavlink-backed vehicle adapter.

This is the single module in the codebase permitted to import pymavlink.
A background thread performs the blocking MAVLink reads and updates a
lock-protected snapshot; :meth:`get_state` / :meth:`get_telemetry` are cheap
reads of that snapshot. Command verbs send a COMMAND_LONG and block only
until the matching COMMAND_ACK (captured by the reader thread), with retries.
"""

from __future__ import annotations

import contextlib
import math
import threading
import time
from collections.abc import Callable
from typing import Any

from pymavlink import mavutil

from mavctl.adapter.base import (
    ConnectionLostError,
    MissionCountUnsupportedError,
    MissionItemUnsupportedError,
    MissionProtocolError,
    MissionStateUncertainError,
    ModeMappingUnavailableError,
)
from mavctl.models import (
    MISSION_MAX_ITEMS,
    MISSION_TYPE_MISSION,
    Attitude,
    Battery,
    CommandOutcome,
    DownloadedMissionV1,
    GpsInfo,
    HomePosition,
    MissionItem,
    MissionOutcome,
    MissionV1,
    Position,
    Telemetry,
    VehicleState,
    Velocity,
    mission_item_from_remote,
    mission_item_to_int_fields,
    mission_result_name,
)

# Message types we subscribe to.
_SUBSCRIBED = (
    "HEARTBEAT",
    "SYS_STATUS",
    "GLOBAL_POSITION_INT",
    "ATTITUDE",
    "GPS_RAW_INT",
    "COMMAND_ACK",
    "EXTENDED_SYS_STATE",
    "HOME_POSITION",
    "MISSION_REQUEST",
    "MISSION_REQUEST_LIST",
    "MISSION_COUNT",
    "MISSION_CLEAR_ALL",
    "MISSION_REQUEST_INT",
    "MISSION_ITEM_INT",
    "MISSION_ACK",
    "MISSION_CURRENT",
    "MISSION_ITEM_REACHED",
)

# Mission messages that participate in upload/download/clear transactions.
# MISSION_CURRENT / MISSION_ITEM_REACHED are subscribed for completeness but
# intentionally unused by the Phase 3A transactions.
_MISSION_TRANSACTION_TYPES = frozenset({
    "MISSION_REQUEST",
    "MISSION_REQUEST_LIST",
    "MISSION_COUNT",
    "MISSION_CLEAR_ALL",
    "MISSION_REQUEST_INT",
    "MISSION_ITEM_INT",
    "MISSION_ACK",
})

class _MissionSequenceGapError(MissionStateUncertainError):
    """The vehicle requested a future item (``seq > expected``): a strict
    upload-ordering violation per the verified ArduPilot
    ``MissionItemProtocol`` ``request_i`` handling ([FACT]: items arriving
    out of order are answered ``MISSION_ACK(INVALID_SEQUENCE)``). The remote
    mission state must be treated as uncertain and read back."""

    def __init__(
        self,
        message: str,
        *,
        expected_seq: int,
        requested_seq: int,
        sent_upto: int | None = None,
    ) -> None:
        super().__init__(message, sent_upto=sent_upto)
        self.expected_seq = expected_seq
        self.requested_seq = requested_seq


# Mission transaction tuning (docs/design/mission-protocol-v1.md §D/E; the
# vehicle's own upload timer is 8 s — ArduPilot MissionItemProtocol).
_MISSION_REQUEST_TIMEOUT_S = 1.0
_MISSION_RETRIES = 3
_MISSION_TRANSACTION_TIMEOUT_S = 15.0
_MISSION_ACK_WINDOW_S = 2.0
_MISSION_CLEAR_RESENDS = 1
_MISSION_COUNT_RESENDS = 2

_GPS_FIX_LABELS = {
    0: "no_gps",
    1: "no_fix",
    2: "2d_fix",
    3: "3d_fix",
    4: "dgps",
    5: "rtk_float",
    6: "rtk_fixed",
    7: "static",
    8: "ppp",
}

_MAV_STATE_LABELS = {
    0: "uninit",
    1: "boot",
    2: "calibrating",
    3: "standby",
    4: "active",
    5: "critical",
    6: "emergency",
    7: "poweroff",
    8: "flight_termination",
}

_LANDED_STATE_LABELS = {
    0: "undefined",
    1: "on_ground",
    2: "in_air",
    3: "takeoff",
    4: "landing",
}

# Sentinels used by MAVLink to mean "field not populated".
_UINT16_MAX = 65535

# Magic param2 value for a forced DISARM (emergency motor stop). Arm never
# sends it: there is no force-arm path in mavctl by design.
_FORCE_ARM_MAGIC = 21196.0


class PymavlinkAdapter:
    """Concrete :class:`~mavctl.adapter.base.VehicleAdapter` over pymavlink.

    Args:
        connection_string: mavutil-style connection, e.g. ``udp:127.0.0.1:14550``.
        heartbeat_timeout_s: link is considered lost if no HEARTBEAT arrives
            within this many seconds.
        source_system: MAVLink source system id for this GCS.
        command_ack_timeout_s: seconds to wait for a COMMAND_ACK per attempt.
        command_retries: how many times to (re)send a command awaiting its ACK.
    """

    def __init__(
        self,
        connection_string: str,
        heartbeat_timeout_s: float = 3.0,
        source_system: int = 255,
        command_ack_timeout_s: float = 5.0,
        command_retries: int = 3,
        command_ack_settle_s: float = 1.0,
    ) -> None:
        self._connection_string = connection_string
        self._heartbeat_timeout_s = heartbeat_timeout_s
        self._source_system = source_system
        self._command_ack_timeout_s = command_ack_timeout_s
        self._command_retries = command_retries
        # Post-timeout quiet window for a command id (see _send_command).
        self._command_ack_settle_s = command_ack_settle_s

        self._master: Any | None = None
        self._reader: threading.Thread | None = None
        self._stop = threading.Event()
        self._lock = threading.Lock()
        self._send_lock = threading.Lock()
        # Serializes a whole command transaction (send -> await ACK -> retry)
        # so two commands can never interleave or steal each other's ACK.
        # Held only by executor threads running command verbs; the reader
        # thread and snapshot reads (get_state/get_telemetry) never take it.
        self._command_lock = threading.Lock()

        # COMMAND_ACK rendezvous: command id -> (result, recv_monotonic).
        #
        # MAVLink protocol limit: COMMAND_ACK carries the command id (and
        # result) but not confirmation/param1, so arm and disarm — which share
        # MAV_CMD_COMPONENT_ARM_DISARM — cannot be perfectly correlated to a
        # specific send. Transaction serialization + stale-ACK clear reduce
        # concurrent races; they cannot prove a late ACK after a timeout
        # belongs to the next same-id send. See quarantine below.
        self._ack_cond = threading.Condition()
        self._acks: dict[int, tuple[int, float]] = {}
        # command id -> monotonic deadline. After a full timeout we drop ACKs
        # for that id until the settle window ends, and the next same-id send
        # waits out the window first. Risk reduction only — not perfect
        # correlation.
        self._ack_quarantine_until: dict[int, float] = {}

        # Target ids, learned from (and then locked to) the first autopilot
        # heartbeat. Only this system+component may update the snapshot or
        # answer COMMAND_ACKs. Defaults are placeholders until lock; ACKs and
        # snapshot telemetry are rejected while unlocked.
        self._target_system = 1
        self._target_component = 1
        self._target_locked = False
        self._streams_requested = False

        # Snapshot fields, all guarded by ``_lock``.
        self._system_id: int | None = None
        self._component_id: int | None = None
        self._last_hb_monotonic: float | None = None
        self._last_hb_epoch: float | None = None
        self._flight_mode: str | None = None
        self._armed: bool | None = None
        self._system_status: str | None = None
        self._landed_state: str | None = None
        self._relative_alt_m: float | None = None
        self._battery = Battery()
        self._gps = GpsInfo()
        self._home: HomePosition | None = None
        self._position = Position()
        self._attitude = Attitude()
        self._velocity = Velocity()
        self._telemetry_ts: float | None = None
        # Freshness: time.monotonic() of the last accepted message per
        # stream (locked-autopilot source only). Monotonic, never epoch
        # wall-clock, so ages survive wall-clock adjustments. Ages keep
        # counting after heartbeat loss — the cached snapshot's staleness
        # must stay visible when the link is gone. Future guards may use
        # stream freshness as an additional input; current guard conditions
        # are unchanged.
        self._battery_ts_mono: float | None = None
        self._gps_ts_mono: float | None = None
        self._telemetry_ts_mono: float | None = None
        self._home_ts_mono: float | None = None
        self._landed_state_ts_mono: float | None = None

        # Mission protocol transactions. _mission_lock serializes whole
        # upload/download/clear sessions; _mission_cond + _mission_inbox are
        # the reader-thread -> transaction rendezvous. Messages are only
        # delivered while a session is active, so stale traffic can never
        # satisfy a later transaction.
        self._mission_lock = threading.Lock()
        self._mission_cond = threading.Condition()
        self._mission_active = False
        self._session_seq = 0
        self._mission_inbox: list[tuple[str, Any]] = []

    # -- lifecycle ---------------------------------------------------------

    def connect(self) -> None:
        """Open the link and start the background reader thread.

        Does not block waiting for a heartbeat; the vehicle may connect
        later. Raises :class:`ConnectionLostError` if the link cannot open.
        """

        if self._reader is not None and self._reader.is_alive():
            return
        try:
            self._master = mavutil.mavlink_connection(
                self._connection_string,
                source_system=self._source_system,
            )
        except Exception as exc:
            raise ConnectionLostError(
                f"could not open link {self._connection_string!r}: {exc}"
            ) from exc

        self._stop.clear()
        self._reader = threading.Thread(
            target=self._read_loop,
            name="mavctl-reader",
            daemon=True,
        )
        self._reader.start()

    def disconnect(self) -> None:
        """Stop the reader thread and close the link. Idempotent."""

        self._stop.set()
        reader = self._reader
        if reader is not None and reader.is_alive() and reader is not threading.current_thread():
            reader.join(timeout=3.0)
        self._reader = None
        if self._master is not None:
            with contextlib.suppress(Exception):
                self._master.close()
            self._master = None

    # -- snapshot reads ----------------------------------------------------

    def get_state(self) -> VehicleState:
        with self._lock:
            now = time.monotonic()

            def _age(ts_mono: float | None) -> float | None:
                # None = stream never received; otherwise elapsed monotonic
                # seconds, independent of the heartbeat/connected state.
                return round(now - ts_mono, 3) if ts_mono is not None else None

            age = self._heartbeat_age_locked()
            connected = age is not None and age <= self._heartbeat_timeout_s
            return VehicleState(
                connected=connected,
                connection_string=self._connection_string,
                system_id=self._system_id,
                component_id=self._component_id,
                last_heartbeat_ts=self._last_hb_epoch,
                heartbeat_age_s=round(age, 3) if age is not None else None,
                flight_mode=self._flight_mode if connected else None,
                armed=self._armed if connected else None,
                system_status=self._system_status if connected else None,
                landed_state=self._landed_state if connected else None,
                relative_alt_m=self._relative_alt_m,
                battery=self._battery.model_copy(),
                gps=self._gps.model_copy(),
                home_position=self._home.model_copy() if self._home is not None else None,
                telemetry_age_s=_age(self._telemetry_ts_mono),
                gps_age_s=_age(self._gps_ts_mono),
                battery_age_s=_age(self._battery_ts_mono),
                home_position_age_s=_age(self._home_ts_mono),
                landed_state_age_s=_age(self._landed_state_ts_mono),
            )

    def get_telemetry(self) -> Telemetry:
        with self._lock:
            return Telemetry(
                timestamp=self._telemetry_ts,
                position=self._position.model_copy(),
                attitude=self._attitude.model_copy(),
                velocity=self._velocity.model_copy(),
            )

    # -- commands ----------------------------------------------------------

    def mode_names(self) -> list[str]:
        return sorted(self._mode_mapping().keys())

    def arm(self) -> CommandOutcome:
        # param2 stays 0.0 unconditionally: mavctl never sends the 21196 magic
        # that would bypass the autopilot's pre-arm checks.
        return self._send_command(
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            [1.0, 0.0],
        )

    def disarm(self, force: bool = False) -> CommandOutcome:
        return self._send_command(
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            [0.0, _FORCE_ARM_MAGIC if force else 0.0],
        )

    def set_mode(self, mode: str) -> CommandOutcome:
        mapping = self._mode_mapping()
        number = mapping.get(mode.upper())
        if number is None:
            # The guard validates user input against mode_names(); reaching
            # here with an unresolvable target means the mapping vanished or
            # changed between validation and send — a transient vehicle
            # state, reported as a typed adapter error, never an internal
            # error.
            raise ModeMappingUnavailableError(
                f"flight mode {mode!r} cannot be resolved: the vehicle mode "
                f"mapping is unavailable or changed (available now: {sorted(mapping)})"
            )
        return self._send_command(
            mavutil.mavlink.MAV_CMD_DO_SET_MODE,
            [float(mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED), float(number)],
        )

    def takeoff(self, altitude_m: float) -> CommandOutcome:
        return self._send_command(
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, float(altitude_m)],
        )

    def land(self) -> CommandOutcome:
        return self._send_command(mavutil.mavlink.MAV_CMD_NAV_LAND, [])

    def rtl(self) -> CommandOutcome:
        return self._send_command(mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, [])

    # -- mission transactions (Phase 3A) -----------------------------------
    #
    # State machines per docs/design/mission-protocol-v1.md §D/E/F. All three
    # transactions serialize on _mission_lock; the reader thread delivers
    # locked-autopilot MISSION_* messages into the session inbox; the
    # COMMAND_ACK machinery is never involved.

    def upload_mission(self, mission: MissionV1) -> MissionOutcome:
        """Run the MAVLink mission upload transaction for a validated plan.

        Request ordering follows the verified ArduPilot strict-sequence
        protocol ([FACT] ``MissionItemProtocol.handle_mission_item``: an item
        whose ``seq`` does not equal the vehicle's expected ``request_i`` is
        answered ``MISSION_ACK(INVALID_SEQUENCE)``):

        - ``seq == expected_next_seq``: send exactly that item, advance;
        - ``seq < expected_next_seq``: re-send the already-sent item
          (packet-loss / retry compatible; the expectation does not advance);
        - ``seq > expected_next_seq``: future item — send nothing, abort with
          :class:`_MissionSequenceGapError` (remote state uncertain).

        Phase model (design §D): from ``MISSION_COUNT`` onward every abort
        raises :class:`MissionStateUncertainError` — the vehicle may hold a
        partial mission.
        """
        master = self._master
        if master is None:
            raise ConnectionLostError("link is not open")
        items = mission.items
        count = len(items)
        overall = time.monotonic() + _MISSION_TRANSACTION_TIMEOUT_S
        sent_upto: int | None = None
        items_sent = 0
        expected_next_seq = 0
        last_sent_seq: int | None = None

        def uncertain(message: str) -> MissionStateUncertainError:
            return MissionStateUncertainError(message, sent_upto=sent_upto)

        def accept_request_or_ack(msg_type: str, msg: Any) -> bool:
            # ArduPilot paces uploads with the FLOAT MISSION_REQUEST message
            # (MissionItemProtocol::queued_request_send never sends
            # MISSION_REQUEST_INT in the upload direction) — accept either
            # transport; both carry the same seq semantics.
            if msg_type not in ("MISSION_REQUEST_INT", "MISSION_REQUEST", "MISSION_ACK"):
                return False
            return getattr(msg, "mission_type", MISSION_TYPE_MISSION) == MISSION_TYPE_MISSION

        with self._mission_lock:
            self._begin_mission_session()
            try:
                self._send_mission_count(master, count)
                resends = 0
                while expected_next_seq < count:
                    if last_sent_seq is None:
                        # U1: COUNT sent, no request seen yet — COUNT is the
                        # only thing that can be safely retried here.
                        until = min(overall, time.monotonic() + _MISSION_REQUEST_TIMEOUT_S)
                    else:
                        # U2: at least one item was sent. A blind re-send is
                        # unsafe — the GCS cannot know whether the last item
                        # was lost, accepted with the next request lost, or
                        # the vehicle entered an error state (re-sending item
                        # k to a vehicle that advanced to k+1 hits
                        # INVALID_SEQUENCE). Wait for explicit vehicle
                        # traffic until the overall deadline; late requests
                        # are still classified and answered below.
                        until = overall
                    msg_type, msg = self._wait_mission(accept_request_or_ack, until)
                    if msg is None:
                        if time.monotonic() >= overall:
                            if last_sent_seq is None:
                                raise uncertain("vehicle did not request mission items")
                            raise uncertain(
                                f"vehicle stopped requesting after item "
                                f"{last_sent_seq}"
                            )
                        # U1 per-attempt timeout: retry COUNT per design.
                        resends += 1
                        if resends > _MISSION_RETRIES:
                            raise uncertain(
                                "vehicle did not request mission items after retries"
                            )
                        self._send_mission_count(master, count)
                        continue
                    if msg_type == "MISSION_ACK":
                        # Phase-aware ACK mapping. Before any item is sent the
                        # vehicle cannot have stored anything, so a rejection
                        # is clean; once items are stored the remote mission
                        # is already modified and the outcome must be treated
                        # as uncertain (ArduPilot does not roll back accepted
                        # items on a later error ACK).
                        result = int(msg.type)
                        if result == 0:
                            # Premature ACCEPTED: protocol anomaly — the
                            # vehicle accepted fewer items than announced.
                            raise uncertain(
                                f"premature mission ACK after {items_sent} item(s)"
                            )
                        if items_sent == 0:
                            raise MissionProtocolError(
                                f"mission upload rejected: {mission_result_name(result)}",
                                result_name=mission_result_name(result),
                            )
                        raise uncertain(
                            f"vehicle rejected the upload after {items_sent} "
                            f"item(s): {mission_result_name(result)}"
                        )
                    seq = int(msg.seq)
                    if seq >= count:
                        raise _MissionSequenceGapError(
                            f"vehicle requested out-of-range seq {seq} "
                            f"(expected {expected_next_seq})",
                            expected_seq=expected_next_seq,
                            requested_seq=seq,
                            sent_upto=sent_upto,
                        )
                    if seq > expected_next_seq:
                        raise _MissionSequenceGapError(
                            f"vehicle requested future seq {seq} "
                            f"(expected {expected_next_seq}): strict upload "
                            "ordering forbids sending items ahead of the "
                            "vehicle's request sequence",
                            expected_seq=expected_next_seq,
                            requested_seq=seq,
                            sent_upto=sent_upto,
                        )
                    # seq == expected: the expected item — send and advance.
                    # seq < expected: explicit duplicate request — re-send the
                    # already-sent item without advancing (the ONLY resend
                    # trigger; retry is request-driven).
                    self._send_mission_item(master, items[seq], seq)
                    last_sent_seq = seq
                    items_sent += 1
                    if seq == expected_next_seq:
                        sent_upto = seq
                        expected_next_seq = seq + 1
                    resends = 0
                sent_upto = count - 1
                # Terminal ACK. The vehicle sends it immediately after the last
                # item; keep listening until the overall deadline because its
                # 8 s timer can still deliver OPERATION_CANCELLED. Late
                # in-order requests are answered politely.
                while True:
                    until = min(overall, time.monotonic() + _MISSION_ACK_WINDOW_S)
                    msg_type, msg = self._wait_mission(accept_request_or_ack, until)
                    if msg is None:
                        if time.monotonic() >= overall:
                            raise uncertain("terminal mission ACK not received")
                        continue
                    if msg_type == "MISSION_ACK":
                        result = int(msg.type)
                        if result == 0:
                            return MissionOutcome(
                                action="mission_upload",
                                accepted=True,
                                result_name="ACCEPTED",
                                item_count=count,
                                sent_upto=count - 1,
                            )
                        # U3: every item was already sent and stored — any
                        # non-ACCEPTED result (including the vehicle's 8 s
                        # OPERATION_CANCELLED) leaves the remote mission
                        # modified. Phase-aware mapping: uncertain, never a
                        # clean rejection.
                        raise uncertain(
                            f"upload ended with {mission_result_name(result)} "
                            f"after {items_sent} item(s)"
                        )
                    seq = int(msg.seq)
                    if 0 <= seq < count:
                        self._send_mission_item(master, items[seq], seq)
            finally:
                self._end_mission_session()

    def download_mission(self) -> DownloadedMissionV1:
        """Read the remote mission atomically (design §E).

        Any failure raises: a partial mission is never emitted. The GCS
        terminal ACK is best-effort courtesy (ArduPilot does not consume it).
        """
        master = self._master
        if master is None:
            raise ConnectionLostError("link is not open")
        overall = time.monotonic() + _MISSION_TRANSACTION_TIMEOUT_S

        def accept_count_or_ack(msg_type: str, msg: Any) -> bool:
            if msg_type not in ("MISSION_COUNT", "MISSION_ACK"):
                return False
            return getattr(msg, "mission_type", MISSION_TYPE_MISSION) == MISSION_TYPE_MISSION

        def accept_item(msg_type: str, msg: Any) -> bool:
            return msg_type == "MISSION_ITEM_INT" and (
                getattr(msg, "mission_type", MISSION_TYPE_MISSION) == MISSION_TYPE_MISSION
            )

        with self._mission_lock:
            self._begin_mission_session()
            try:
                count: int | None = None
                resends = 0
                request_outstanding = False
                while count is None:
                    if not request_outstanding:
                        self._send_mission_request_list(master)
                        request_outstanding = True
                    until = min(overall, time.monotonic() + _MISSION_REQUEST_TIMEOUT_S)
                    msg_type, msg = self._wait_mission(accept_count_or_ack, until)
                    if msg is None:
                        if time.monotonic() >= overall:
                            raise MissionProtocolError(
                                "vehicle did not answer MISSION_REQUEST_LIST",
                                result_name="TIMEOUT",
                            )
                        resends += 1
                        if resends > _MISSION_COUNT_RESENDS:
                            raise MissionProtocolError(
                                "vehicle did not answer MISSION_REQUEST_LIST after retries",
                                result_name="TIMEOUT",
                            )
                        request_outstanding = False  # re-request on timeout
                        continue
                    if msg_type == "MISSION_ACK":
                        # Typically DENIED: a vehicle-side upload is in flight.
                        result = int(msg.type)
                        raise MissionProtocolError(
                            f"vehicle denied mission download: {mission_result_name(result)}",
                            result_name=mission_result_name(result),
                        )
                    count = int(msg.count)
                if count > MISSION_MAX_ITEMS:
                    # Fail before requesting anything: v1 cannot represent a
                    # mission this large, and issuing 100+ item requests would
                    # only churn the link before failing anyway.
                    raise MissionCountUnsupportedError(
                        f"remote mission has {count} items; mavctl v1 supports "
                        f"at most {MISSION_MAX_ITEMS}",
                        observed_count=count,
                        max_supported_items=MISSION_MAX_ITEMS,
                    )
                if count == 0:
                    self._send_mission_ack_accepted(master)
                    return DownloadedMissionV1(version=1, items=[])
                converted: list[MissionItem] = []
                received: dict[int, Any] = {}
                for seq in range(count):
                    if seq in received:
                        # buffered earlier by out-of-order delivery
                        converted.append(self._convert_remote_item(received[seq], seq))
                        continue
                    resends = 0
                    request_outstanding = False
                    while seq not in received:
                        if not request_outstanding:
                            self._send_mission_request_int(master, seq)
                            request_outstanding = True
                        until = min(overall, time.monotonic() + _MISSION_REQUEST_TIMEOUT_S)
                        msg_type, msg = self._wait_mission(accept_item, until)
                        if msg is None:
                            if time.monotonic() >= overall:
                                raise MissionProtocolError(
                                    f"mission download timed out at seq {seq} "
                                    f"(received: {sorted(received)})",
                                    result_name="TIMEOUT",
                                )
                            resends += 1
                            if resends > _MISSION_RETRIES:
                                raise MissionProtocolError(
                                    f"mission download timed out at seq {seq} after "
                                    f"retries (received: {sorted(received)})",
                                    result_name="TIMEOUT",
                                )
                            request_outstanding = False  # re-request on timeout
                            continue
                        iseq = int(msg.seq)
                        if iseq == seq:
                            # Convert immediately: an unsupported item fails the
                            # download atomically the moment it is identified.
                            converted.append(self._convert_remote_item(msg, seq))
                            received[seq] = msg
                        elif 0 <= iseq < count and iseq not in received:
                            received[iseq] = msg  # buffer out-of-order delivery
                        # duplicates and out-of-range seqs are dropped
                self._send_mission_ack_accepted(master)
                return DownloadedMissionV1(version=1, items=converted)
            finally:
                self._end_mission_session()

    def clear_mission(self) -> MissionOutcome:
        """Clear the remote mission and verify with a count read-back.

        The read-back is mandatory: a silent success is never reported. A
        non-zero observed count or a read-back timeout raises
        :class:`MissionStateUncertainError` with ``observed_count`` when
        known.
        """
        master = self._master
        if master is None:
            raise ConnectionLostError("link is not open")
        overall = time.monotonic() + _MISSION_TRANSACTION_TIMEOUT_S

        def accept_clear_ack(msg_type: str, msg: Any) -> bool:
            return msg_type == "MISSION_ACK" and (
                getattr(msg, "mission_type", MISSION_TYPE_MISSION) == MISSION_TYPE_MISSION
            )

        with self._mission_lock:
            self._begin_mission_session()
            try:
                self._send_mission_clear_all(master)
                resends = 0
                while True:
                    until = min(overall, time.monotonic() + _MISSION_REQUEST_TIMEOUT_S)
                    _, msg = self._wait_mission(accept_clear_ack, until)
                    if msg is None:
                        if time.monotonic() >= overall:
                            raise MissionStateUncertainError(
                                "vehicle did not acknowledge MISSION_CLEAR_ALL"
                            )
                        resends += 1
                        if resends > _MISSION_CLEAR_RESENDS:
                            raise MissionStateUncertainError(
                                "vehicle did not acknowledge MISSION_CLEAR_ALL "
                                "after resend"
                            )
                        self._send_mission_clear_all(master)
                        continue
                    result = int(msg.type)
                    if result != 0:
                        raise MissionProtocolError(
                            f"mission clear rejected: {mission_result_name(result)}",
                            result_name=mission_result_name(result),
                        )
                    break
                # Read-back verification inside the same mission session.
                observed = self._request_count_locked(master, overall)
                if observed != 0:
                    raise MissionStateUncertainError(
                        f"remote mission count {observed} after clear",
                        observed_count=observed,
                    )
                return MissionOutcome(
                    action="mission_clear",
                    accepted=True,
                    result_name="ACCEPTED",
                    verified=True,
                    observed_count=0,
                )
            finally:
                self._end_mission_session()

    def _convert_remote_item(self, msg: Any, seq: int) -> MissionItem:
        """Convert a received ``MISSION_ITEM_INT`` to a semantic v1 item.

        Raises :class:`MissionItemUnsupportedError` for frames/commands/fields
        outside the v1 schema — download fails atomically instead of emitting
        lossy JSON.
        """

        try:
            return mission_item_from_remote(
                seq=seq,
                command=int(msg.command),
                frame=int(msg.frame),
                param1=float(msg.param1),
                param2=float(msg.param2),
                param3=float(msg.param3),
                param4=float(msg.param4),
                x=int(msg.x),
                y=int(msg.y),
                z=float(msg.z),
            )
        except ValueError as exc:
            raise MissionItemUnsupportedError(
                str(exc), seq=seq, command=int(msg.command), frame=int(msg.frame)
            ) from exc

    def _request_count_locked(self, master: Any, overall: float) -> int:
        """Read back the remote mission count; ``_mission_lock`` is held."""

        def accept_count_or_ack(msg_type: str, msg: Any) -> bool:
            if msg_type not in ("MISSION_COUNT", "MISSION_ACK"):
                return False
            return getattr(msg, "mission_type", MISSION_TYPE_MISSION) == MISSION_TYPE_MISSION

        resends = 0
        request_outstanding = False
        while True:
            if not request_outstanding:
                self._send_mission_request_list(master)
                request_outstanding = True
            until = min(overall, time.monotonic() + _MISSION_REQUEST_TIMEOUT_S)
            msg_type, msg = self._wait_mission(accept_count_or_ack, until)
            if msg is None:
                if time.monotonic() >= overall:
                    raise MissionStateUncertainError("read-back timeout after mission clear")
                resends += 1
                if resends > _MISSION_COUNT_RESENDS:
                    raise MissionStateUncertainError(
                        "read-back timeout after mission clear (retries exhausted)"
                    )
                request_outstanding = False  # re-request on timeout
                continue
            if msg_type == "MISSION_ACK":
                result = int(msg.type)
                raise MissionStateUncertainError(
                    f"vehicle denied mission read-back: {mission_result_name(result)}"
                )
            return int(msg.count)

    def _send_command(
        self,
        command: int,
        params: list[float],
        ack_timeout: float | None = None,
        retries: int | None = None,
    ) -> CommandOutcome:
        master = self._master
        if master is None:
            raise ConnectionLostError("link is not open")
        timeout = ack_timeout if ack_timeout is not None else self._command_ack_timeout_s
        attempts = retries if retries is not None else self._command_retries
        padded = (params + [0.0] * 7)[:7]

        # One command transaction at a time: send + await-ACK + retries are
        # atomic so a concurrent command can neither interleave a send nor
        # consume this command's ACK.
        #
        # Protocol limit (not fully solvable here): COMMAND_ACK only names the
        # MAV_CMD id. Shared ids (arm/disarm) and late ACKs after a timeout
        # cannot be correlated to a specific send with certainty. Mitigations:
        # (1) this lock, (2) clear stale map entries at transaction start,
        # (3) after a full timeout, quarantine that command id for
        # ``command_ack_settle_s`` — drop ACKs and delay the next same-id
        # send. These lower risk; they do not provide perfect correlation.
        with self._command_lock:
            self._wait_ack_quarantine(command)
            # Drop any stale ACK for this command id left by a prior
            # transaction (arm/disarm share MAV_CMD_COMPONENT_ARM_DISARM).
            with self._ack_cond:
                self._acks.pop(command, None)
            for attempt in range(1, attempts + 1):
                send_ts = time.monotonic()
                with self._send_lock:
                    master.mav.command_long_send(
                        self._target_system,
                        self._target_component,
                        command,
                        attempt - 1,  # confirmation counter
                        *padded,
                    )
                result = self._await_ack(command, send_ts, timeout)
                if result is not None:
                    with self._ack_cond:
                        self._ack_quarantine_until.pop(command, None)
                    return CommandOutcome.from_ack(result, attempt)
            # Full timeout: open a settle window so a late ACK for this
            # failed transaction is less likely to satisfy the next same-id send.
            with self._ack_cond:
                self._acks.pop(command, None)
                if self._command_ack_settle_s > 0:
                    self._ack_quarantine_until[command] = (
                        time.monotonic() + self._command_ack_settle_s
                    )
            return CommandOutcome.timeout(attempts)

    def _wait_ack_quarantine(self, command: int) -> None:
        """Block until any post-timeout settle window for ``command`` ends."""

        while True:
            with self._ack_cond:
                until = self._ack_quarantine_until.get(command)
                if until is None:
                    return
                remaining = until - time.monotonic()
                if remaining <= 0:
                    self._ack_quarantine_until.pop(command, None)
                    return
            time.sleep(min(remaining, 0.05))

    def _await_ack(self, command: int, send_ts: float, timeout: float) -> int | None:
        deadline = send_ts + timeout
        with self._ack_cond:
            while True:
                entry = self._acks.get(command)
                if entry is not None and entry[1] >= send_ts:
                    return entry[0]
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    return None
                self._ack_cond.wait(timeout=remaining)

    def _mode_mapping(self) -> dict[str, int]:
        master = self._master
        if master is None:
            return {}
        with contextlib.suppress(Exception):
            mapping = master.mode_mapping()
            if mapping:
                return {str(name): int(num) for name, num in mapping.items()}
        return {}

    # -- mission session plumbing ------------------------------------------

    def _begin_mission_session(self) -> None:
        """Activate mission message delivery; drop any stale traffic.

        Also (re)requests the POSITION telemetry stream: mission operations
        are only allowed on the ground with fresh evidence, and operators
        inspecting the plan need current position context. On links without
        a full GCS (e.g. SITL without MAVProxy) this is what makes
        ``telemetry`` useful at all; on links where a GCS already requests
        streams this is a harmless duplicate.
        """

        with self._mission_cond:
            self._session_seq += 1
            self._mission_active = True
            self._mission_inbox.clear()
        master = self._master
        if master is not None:
            # Modern ArduPilot ignores the deprecated REQUEST_DATA_STREAM
            # message; MAV_CMD_SET_MESSAGE_INTERVAL (511) is the supported
            # way to keep GLOBAL_POSITION_INT flowing. Best effort: if the
            # vehicle rejects it the mission transaction still works — only
            # the operator's position context is degraded, and the
            # remote_mission_state_uncertain read-back hint covers ops.
            with contextlib.suppress(Exception), self._send_lock:
                master.mav.command_long_send(
                    self._target_system,
                    self._target_component,
                    mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                    0,  # confirmation
                    float(mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT),
                    500000.0,  # 2 Hz in microseconds
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                )

    def _end_mission_session(self) -> None:
        """Deactivate delivery and clear the inbox."""

        with self._mission_cond:
            self._mission_active = False
            self._mission_inbox.clear()

    def _deliver_mission(self, msg: Any) -> None:
        """Route an inbound ``MISSION_*`` message to the active transaction.

        Messages are dropped unless a transaction is running and the sender
        is the locked autopilot — stale or foreign mission traffic can never
        satisfy a later transaction. The active check, the inbox append and
        the notify all happen inside the same ``_mission_cond`` critical
        section, and the session token is captured at entry: a delivery that
        raced with a session boundary (end + begin) is recognized as stale by
        its outdated token and dropped instead of leaking into the new
        session's inbox.
        """

        session_token = self._session_seq
        if not self._is_locked_target(msg):
            return
        with self._mission_cond:
            if not self._mission_active or session_token != self._session_seq:
                return
            self._mission_inbox.append((msg.get_type(), msg))
            self._mission_cond.notify_all()

    def _wait_mission(
        self,
        accept: Callable[[str, Any], bool],
        until: float,
    ) -> tuple[str, Any] | tuple[None, None]:
        """Wait for the next mission message accepted by ``accept``.

        Accepted messages are consumed; rejected messages are dropped (the
        transaction has decided they are irrelevant — e.g. a wrong
        ``mission_type``). Returns ``(None, None)`` once ``until`` passes.
        """

        with self._mission_cond:
            while True:
                index = 0
                while index < len(self._mission_inbox):
                    msg_type, msg = self._mission_inbox[index]
                    del self._mission_inbox[index]
                    if accept(msg_type, msg):
                        return msg_type, msg
                    # rejected: dropped, index stays put
                remaining = until - time.monotonic()
                if remaining <= 0:
                    return None, None
                self._mission_cond.wait(timeout=min(remaining, 0.05))

    def _send_mission_count(self, master: Any, count: int) -> None:
        with self._send_lock:
            master.mav.mission_count_send(
                self._target_system, self._target_component, count, MISSION_TYPE_MISSION
            )

    def _send_mission_request_list(self, master: Any) -> None:
        with self._send_lock:
            master.mav.mission_request_list_send(
                self._target_system, self._target_component, MISSION_TYPE_MISSION
            )

    def _send_mission_request_int(self, master: Any, seq: int) -> None:
        with self._send_lock:
            master.mav.mission_request_int_send(
                self._target_system, self._target_component, seq, MISSION_TYPE_MISSION
            )

    def _send_mission_item(self, master: Any, item: MissionItem, seq: int) -> None:
        fields = mission_item_to_int_fields(item)
        with self._send_lock:
            master.mav.mission_item_int_send(
                self._target_system,
                self._target_component,
                seq,
                fields.frame,
                fields.command,
                0,  # current
                1,  # autocontinue
                fields.param1,
                fields.param2,
                fields.param3,
                fields.param4,
                fields.x,
                fields.y,
                fields.z,
                MISSION_TYPE_MISSION,
            )

    def _send_mission_clear_all(self, master: Any) -> None:
        with self._send_lock:
            master.mav.mission_clear_all_send(
                self._target_system, self._target_component, MISSION_TYPE_MISSION
            )

    def _send_mission_ack_accepted(self, master: Any) -> None:
        """Courtesy download-terminal ACK. Failures are non-fatal: ArduPilot
        does not consume it ([FACT], GCS_Common.cpp ``/* not used */``)."""

        with contextlib.suppress(Exception), self._send_lock:
            master.mav.mission_ack_send(
                self._target_system,
                self._target_component,
                0,  # MAV_MISSION_ACCEPTED
                MISSION_TYPE_MISSION,
            )

    # -- internals ---------------------------------------------------------

    def _heartbeat_age_locked(self) -> float | None:
        if self._last_hb_monotonic is None:
            return None
        return time.monotonic() - self._last_hb_monotonic

    def _read_loop(self) -> None:
        master = self._master
        if master is None:
            return
        while not self._stop.is_set():
            try:
                msg = master.recv_match(type=list(_SUBSCRIBED), blocking=True, timeout=1.0)
            except Exception:
                time.sleep(0.1)
                continue
            if msg is None:
                continue
            self._handle_message(msg)

    def _handle_message(self, msg: Any) -> None:
        msg_type = msg.get_type()
        if msg_type == "BAD_DATA":
            return
        if msg_type in _MISSION_TRANSACTION_TYPES:
            # MISSION_* traffic belongs to the mission transaction, never to
            # the snapshot or the COMMAND_ACK machinery.
            self._deliver_mission(msg)
            return
        handler = _HANDLERS.get(msg_type)
        if handler is not None:
            handler(self, msg)

    def _is_locked_target(self, msg: Any) -> bool:
        """True only when the autopilot target is locked and ``msg`` is from it."""

        if not self._target_locked:
            return False
        return bool(
            msg.get_srcSystem() == self._target_system
            and msg.get_srcComponent() == self._target_component
        )

    def _on_heartbeat(self, msg: Any) -> None:
        src_system = msg.get_srcSystem()
        src_component = msg.get_srcComponent()
        # Lock onto the autopilot component on first discovery; afterwards only
        # that exact system+component may update the vehicle snapshot. Heartbeats
        # from other components (gimbal, companion, GCS) must never overwrite
        # flight_mode / armed / system_status.
        if not self._target_locked:
            if src_component != mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1:
                return  # no autopilot seen yet; ignore other components
            self._target_system = src_system
            self._target_component = src_component
            self._target_locked = True
        elif not self._is_locked_target(msg):
            return

        now_mono = time.monotonic()
        now_epoch = time.time()
        flight_mode = self._flightmode_string(msg)
        armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
        system_status = _MAV_STATE_LABELS.get(msg.system_status, f"state_{msg.system_status}")
        self._request_streams_once()
        with self._lock:
            self._system_id = src_system
            self._component_id = src_component
            self._last_hb_monotonic = now_mono
            self._last_hb_epoch = now_epoch
            self._flight_mode = flight_mode
            self._armed = armed
            self._system_status = system_status

    def _request_streams_once(self) -> None:
        """Best-effort: ask for the extended-status stream so EXTENDED_SYS_STATE
        (landed_state) and HOME_POSITION populate. Runs on the reader thread."""

        if self._streams_requested or self._master is None:
            return
        self._streams_requested = True
        with contextlib.suppress(Exception), self._send_lock:
            self._master.mav.request_data_stream_send(
                self._target_system,
                self._target_component,
                mavutil.mavlink.MAV_DATA_STREAM_EXTENDED_STATUS,
                2,  # Hz
                1,  # start
            )

    def _flightmode_string(self, msg: Any) -> str | None:
        master = self._master
        if master is not None:
            with contextlib.suppress(Exception):
                mode = master.flightmode
                if isinstance(mode, str):
                    return mode
        return f"mode({msg.custom_mode})"

    def _on_sys_status(self, msg: Any) -> None:
        if not self._is_locked_target(msg):
            return
        voltage = msg.voltage_battery
        current = msg.current_battery
        remaining = msg.battery_remaining
        battery = Battery(
            voltage_v=(voltage / 1000.0) if voltage not in (0, _UINT16_MAX) else None,
            current_a=(current / 100.0) if current >= 0 else None,
            remaining_pct=remaining if remaining >= 0 else None,
        )
        with self._lock:
            self._battery = battery
            self._battery_ts_mono = time.monotonic()

    def _on_global_position(self, msg: Any) -> None:
        if not self._is_locked_target(msg):
            return
        vx = msg.vx / 100.0
        vy = msg.vy / 100.0
        vz = msg.vz / 100.0
        heading = msg.hdg / 100.0 if msg.hdg != _UINT16_MAX else None
        relative_alt = msg.relative_alt / 1000.0
        position = Position(
            lat_deg=msg.lat / 1e7,
            lon_deg=msg.lon / 1e7,
            alt_msl_m=msg.alt / 1000.0,
            relative_alt_m=relative_alt,
        )
        velocity = Velocity(
            vx_ms=vx,
            vy_ms=vy,
            vz_ms=vz,
            groundspeed_ms=math.hypot(vx, vy),
            heading_deg=heading,
        )
        with self._lock:
            self._position = position
            self._velocity = velocity
            self._relative_alt_m = relative_alt
            self._telemetry_ts = time.time()
            self._telemetry_ts_mono = time.monotonic()

    def _on_attitude(self, msg: Any) -> None:
        if not self._is_locked_target(msg):
            return
        attitude = Attitude(
            roll_deg=math.degrees(msg.roll),
            pitch_deg=math.degrees(msg.pitch),
            yaw_deg=math.degrees(msg.yaw),
        )
        with self._lock:
            self._attitude = attitude
            self._telemetry_ts = time.time()
            self._telemetry_ts_mono = time.monotonic()

    def _on_gps_raw(self, msg: Any) -> None:
        if not self._is_locked_target(msg):
            return
        gps = GpsInfo(
            fix_type=msg.fix_type,
            fix_label=_GPS_FIX_LABELS.get(msg.fix_type, "unknown"),
            satellites_visible=(
                msg.satellites_visible if msg.satellites_visible != 255 else None
            ),
        )
        with self._lock:
            self._gps = gps
            self._gps_ts_mono = time.monotonic()

    def _on_command_ack(self, msg: Any) -> None:
        # Refuse all ACKs until the autopilot target is locked. Default
        # target ids (1/1) are placeholders and must not accept traffic.
        if not self._target_locked:
            return
        # Only accept ACKs from the locked autopilot; ignore ACKs emitted by
        # other systems/components so one vehicle's ACK can never satisfy a
        # command addressed to ours.
        if not self._is_locked_target(msg):
            return
        command = int(msg.command)
        with self._ack_cond:
            until = self._ack_quarantine_until.get(command)
            if until is not None and time.monotonic() < until:
                # Late ACK during post-timeout settle window — drop.
                return
            self._acks[command] = (int(msg.result), time.monotonic())
            self._ack_cond.notify_all()

    def _on_extended_sys_state(self, msg: Any) -> None:
        if not self._is_locked_target(msg):
            return
        label = _LANDED_STATE_LABELS.get(msg.landed_state)
        with self._lock:
            self._landed_state = label
            self._landed_state_ts_mono = time.monotonic()

    def _on_home_position(self, msg: Any) -> None:
        if not self._is_locked_target(msg):
            return
        home = HomePosition(
            lat_deg=msg.latitude / 1e7,
            lon_deg=msg.longitude / 1e7,
            alt_msl_m=msg.altitude / 1000.0,
        )
        with self._lock:
            self._home = home
            self._home_ts_mono = time.monotonic()


# Dispatch table mapping MAVLink message type -> bound-method.
_HANDLERS: dict[str, Any] = {
    "HEARTBEAT": PymavlinkAdapter._on_heartbeat,
    "SYS_STATUS": PymavlinkAdapter._on_sys_status,
    "GLOBAL_POSITION_INT": PymavlinkAdapter._on_global_position,
    "ATTITUDE": PymavlinkAdapter._on_attitude,
    "GPS_RAW_INT": PymavlinkAdapter._on_gps_raw,
    "COMMAND_ACK": PymavlinkAdapter._on_command_ack,
    "EXTENDED_SYS_STATE": PymavlinkAdapter._on_extended_sys_state,
    "HOME_POSITION": PymavlinkAdapter._on_home_position,
}
