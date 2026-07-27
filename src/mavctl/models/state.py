"""Vehicle status snapshot models (connection, mode, arming, power, GPS)."""

from __future__ import annotations

from pydantic import BaseModel


class Battery(BaseModel):
    """Power system summary derived from ``SYS_STATUS`` / ``BATTERY_STATUS``."""

    voltage_v: float | None = None
    current_a: float | None = None
    remaining_pct: int | None = None


class GpsInfo(BaseModel):
    """GPS summary derived from ``GPS_RAW_INT``."""

    fix_type: int | None = None
    fix_label: str | None = None
    satellites_visible: int | None = None


class HomePosition(BaseModel):
    """Home location from ``HOME_POSITION`` (degrees / metres)."""

    lat_deg: float | None = None
    lon_deg: float | None = None
    alt_msl_m: float | None = None


class VehicleState(BaseModel):
    """High-level vehicle status snapshot cached by the daemon.

    Designed to be self-describing: a single query should let an agent
    reconstruct its full picture of the vehicle. Every field is optional:
    before the first matching MAVLink message arrives its value is ``None``.
    """

    connected: bool = False
    connection_string: str | None = None
    system_id: int | None = None
    component_id: int | None = None
    last_heartbeat_ts: float | None = None
    heartbeat_age_s: float | None = None
    flight_mode: str | None = None
    armed: bool | None = None
    system_status: str | None = None
    landed_state: str | None = None
    relative_alt_m: float | None = None
    battery: Battery = Battery()
    gps: GpsInfo = GpsInfo()
    home_position: HomePosition | None = None
