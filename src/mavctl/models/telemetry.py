"""Telemetry snapshot models (position, attitude, velocity)."""

from __future__ import annotations

from pydantic import BaseModel


class Position(BaseModel):
    """Global position from ``GLOBAL_POSITION_INT`` (degrees / metres)."""

    lat_deg: float | None = None
    lon_deg: float | None = None
    alt_msl_m: float | None = None
    relative_alt_m: float | None = None


class Attitude(BaseModel):
    """Body attitude from ``ATTITUDE`` (degrees)."""

    roll_deg: float | None = None
    pitch_deg: float | None = None
    yaw_deg: float | None = None


class Velocity(BaseModel):
    """Velocity from ``GLOBAL_POSITION_INT`` (metres/second, degrees)."""

    vx_ms: float | None = None
    vy_ms: float | None = None
    vz_ms: float | None = None
    groundspeed_ms: float | None = None
    heading_deg: float | None = None


class Telemetry(BaseModel):
    """A coherent telemetry snapshot at a point in time."""

    timestamp: float | None = None
    position: Position = Position()
    attitude: Attitude = Attitude()
    velocity: Velocity = Velocity()
