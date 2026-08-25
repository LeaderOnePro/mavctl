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
from typing import Any

from pymavlink import mavutil

from mavctl.adapter.base import ConnectionLostError
from mavctl.models import (
    Attitude,
    Battery,
    CommandOutcome,
    GpsInfo,
    HomePosition,
    Position,
    Telemetry,
    VehicleState,
    Velocity,
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
)

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

# Magic param2 value that forces (dis)arming past pre-arm checks.
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
    ) -> None:
        self._connection_string = connection_string
        self._heartbeat_timeout_s = heartbeat_timeout_s
        self._source_system = source_system
        self._command_ack_timeout_s = command_ack_timeout_s
        self._command_retries = command_retries

        self._master: Any | None = None
        self._reader: threading.Thread | None = None
        self._stop = threading.Event()
        self._lock = threading.Lock()
        self._send_lock = threading.Lock()

        # COMMAND_ACK rendezvous: command id -> (result, recv_monotonic).
        self._ack_cond = threading.Condition()
        self._acks: dict[int, tuple[int, float]] = {}

        # Target ids, learned from the first heartbeat.
        self._target_system = 1
        self._target_component = 1
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

    def arm(self, force: bool = False) -> CommandOutcome:
        return self._send_command(
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            [1.0, _FORCE_ARM_MAGIC if force else 0.0],
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
            raise ValueError(f"unknown flight mode {mode!r}; available: {sorted(mapping)}")
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
                return CommandOutcome.from_ack(result, attempt)
        return CommandOutcome.timeout(attempts)

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
        handler = _HANDLERS.get(msg_type)
        if handler is not None:
            handler(self, msg)

    def _on_heartbeat(self, msg: Any) -> None:
        now_mono = time.monotonic()
        now_epoch = time.time()
        flight_mode = self._flightmode_string(msg)
        armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
        system_status = _MAV_STATE_LABELS.get(msg.system_status, f"state_{msg.system_status}")
        src_system = msg.get_srcSystem()
        src_component = msg.get_srcComponent()
        # Learn the target ids for command addressing (autopilot component only).
        if src_component == mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1:
            self._target_system = src_system
            self._target_component = src_component
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
        with contextlib.suppress(Exception):
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

    def _on_global_position(self, msg: Any) -> None:
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

    def _on_attitude(self, msg: Any) -> None:
        attitude = Attitude(
            roll_deg=math.degrees(msg.roll),
            pitch_deg=math.degrees(msg.pitch),
            yaw_deg=math.degrees(msg.yaw),
        )
        with self._lock:
            self._attitude = attitude
            self._telemetry_ts = time.time()

    def _on_gps_raw(self, msg: Any) -> None:
        gps = GpsInfo(
            fix_type=msg.fix_type,
            fix_label=_GPS_FIX_LABELS.get(msg.fix_type, "unknown"),
            satellites_visible=(
                msg.satellites_visible if msg.satellites_visible != 255 else None
            ),
        )
        with self._lock:
            self._gps = gps

    def _on_command_ack(self, msg: Any) -> None:
        with self._ack_cond:
            self._acks[int(msg.command)] = (int(msg.result), time.monotonic())
            self._ack_cond.notify_all()

    def _on_extended_sys_state(self, msg: Any) -> None:
        label = _LANDED_STATE_LABELS.get(msg.landed_state)
        with self._lock:
            self._landed_state = label

    def _on_home_position(self, msg: Any) -> None:
        home = HomePosition(
            lat_deg=msg.latitude / 1e7,
            lon_deg=msg.longitude / 1e7,
            alt_msl_m=msg.altitude / 1000.0,
        )
        with self._lock:
            self._home = home


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
