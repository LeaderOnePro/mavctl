"""Vehicle status snapshot models (connection, mode, arming, power, GPS)."""

from __future__ import annotations

from pydantic import BaseModel, Field


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


class MissionExecutionState(BaseModel):
    """Mission execution observation from the locked autopilot's
    `MISSION_CURRENT` (Phase 3B-1).

    Data source is exclusively `MISSION_CURRENT` from the locked autopilot:
    `seq`/`total`/`mission_state`/`mission_mode` as populated by ArduPilot
    (total excludes the home slot; `mission_mode` is 1 while the current
    flight mode requires a mission). All fields are `None` until the first
    `MISSION_CURRENT` arrives — AUTO mode or position is never used to
    fabricate mission activity. `age_s` is monotonic-clock based like the
    other freshness ages; it keeps counting after heartbeat loss, and after
    disconnect the cached observation must not be treated as a live fact.
    """

    current_seq: int | None = None
    total: int | None = None
    state: str | None = None
    mode: str | None = None
    age_s: float | None = None


class VehicleState(BaseModel):
    """High-level vehicle status snapshot cached by the daemon.

    Designed to be self-describing: a single query should let an agent
    reconstruct its full picture of the vehicle. Every field is optional:
    before the first matching MAVLink message arrives its value is ``None``.

    Freshness metadata (``*_age_s``) reports, per cached stream, the elapsed
    seconds since the daemon last *accepted* a message of that class from the
    locked autopilot source, computed on ``time.monotonic()`` (never epoch
    wall-clock). ``None`` means the stream was never received. Ages keep
    counting after the heartbeat goes stale, so the staleness of the cached
    snapshot stays visible even on link loss. An age is a cache-staleness
    indicator, not a physical measurement of data latency. Current guard
    conditions are unchanged; future guards may use stream freshness as an
    additional input.
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
    # Freshness per stream; None = message class never received.
    telemetry_age_s: float | None = None
    gps_age_s: float | None = None
    battery_age_s: float | None = None
    home_position_age_s: float | None = None
    landed_state_age_s: float | None = None
    mission: MissionExecutionState = Field(default_factory=MissionExecutionState)
