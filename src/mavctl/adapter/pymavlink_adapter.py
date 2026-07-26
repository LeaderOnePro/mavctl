"""pymavlink-backed vehicle adapter.

This is the single module in the codebase permitted to import pymavlink.
A background thread performs the blocking MAVLink reads and updates a
lock-protected snapshot; :meth:`get_state` / :meth:`get_telemetry` are cheap
reads of that snapshot.
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
    GpsInfo,
    Position,
    Telemetry,
    VehicleState,
    Velocity,
)

# Message types we subscribe to for the Phase 1 read-only link.
_SUBSCRIBED = (
    "HEARTBEAT",
    "SYS_STATUS",
    "GLOBAL_POSITION_INT",
    "ATTITUDE",
    "GPS_RAW_INT",
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

# Sentinels used by MAVLink to mean "field not populated".
_UINT16_MAX = 65535


class PymavlinkAdapter:
    """Concrete :class:`~mavctl.adapter.base.VehicleAdapter` over pymavlink.

    Args:
        connection_string: mavutil-style connection, e.g. ``udp:127.0.0.1:14550``.
        heartbeat_timeout_s: link is considered lost if no HEARTBEAT arrives
            within this many seconds.
        source_system: MAVLink source system id for this GCS.
    """

    def __init__(
        self,
        connection_string: str,
        heartbeat_timeout_s: float = 3.0,
        source_system: int = 255,
    ) -> None:
        self._connection_string = connection_string
        self._heartbeat_timeout_s = heartbeat_timeout_s
        self._source_system = source_system

        self._master: Any | None = None
        self._reader: threading.Thread | None = None
        self._stop = threading.Event()
        self._lock = threading.Lock()

        # Snapshot fields, all guarded by ``_lock``.
        self._system_id: int | None = None
        self._component_id: int | None = None
        self._last_hb_monotonic: float | None = None
        self._last_hb_epoch: float | None = None
        self._flight_mode: str | None = None
        self._armed: bool | None = None
        self._battery = Battery()
        self._gps = GpsInfo()
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
                battery=self._battery.model_copy(),
                gps=self._gps.model_copy(),
            )

    def get_telemetry(self) -> Telemetry:
        with self._lock:
            return Telemetry(
                timestamp=self._telemetry_ts,
                position=self._position.model_copy(),
                attitude=self._attitude.model_copy(),
                velocity=self._velocity.model_copy(),
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
        handler = _HANDLERS.get(msg_type)
        if handler is not None:
            handler(self, msg)

    def _on_heartbeat(self, msg: Any) -> None:
        now_mono = time.monotonic()
        now_epoch = time.time()
        flight_mode = self._flightmode_string(msg)
        armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
        with self._lock:
            self._system_id = msg.get_srcSystem()
            self._component_id = msg.get_srcComponent()
            self._last_hb_monotonic = now_mono
            self._last_hb_epoch = now_epoch
            self._flight_mode = flight_mode
            self._armed = armed

    def _flightmode_string(self, msg: Any) -> str | None:
        master = self._master
        if master is not None:
            try:
                mode = master.flightmode
                if isinstance(mode, str):
                    return mode
            except Exception:
                pass
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
        position = Position(
            lat_deg=msg.lat / 1e7,
            lon_deg=msg.lon / 1e7,
            alt_msl_m=msg.alt / 1000.0,
            relative_alt_m=msg.relative_alt / 1000.0,
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


# Dispatch table mapping MAVLink message type -> bound-method name.
_HANDLERS: dict[str, Any] = {
    "HEARTBEAT": PymavlinkAdapter._on_heartbeat,
    "SYS_STATUS": PymavlinkAdapter._on_sys_status,
    "GLOBAL_POSITION_INT": PymavlinkAdapter._on_global_position,
    "ATTITUDE": PymavlinkAdapter._on_attitude,
    "GPS_RAW_INT": PymavlinkAdapter._on_gps_raw,
}
