"""Mission JSON v1 models and pure MISSION_ITEM_INT mapping helpers.

Strict pydantic v2 schemas for the Phase 3A mission format plus pure-data
mapping to ``MISSION_ITEM_INT`` field tuples. All MAVLink numbers here are
plain constants — pymavlink itself is imported only inside the adapter layer.
"""

from __future__ import annotations

from typing import Annotated, Literal

from pydantic import (
    BaseModel,
    ConfigDict,
    Field,
    ValidationError,
    field_validator,
    model_validator,
)

MISSION_SCHEMA_VERSION: Literal[1] = 1
"""The only mission schema version this build understands."""

MISSION_MAX_ITEMS = 100
"""Upper bound on mission items accepted for upload (v1 constant; the
vehicle's real capacity still surfaces as ``MAV_MISSION_NO_SPACE``)."""

# Plain MAVLink constants (verified against the installed common dialect).
MISSION_COMMAND_WAYPOINT = 16
MISSION_COMMAND_RETURN_TO_LAUNCH = 20
MISSION_COMMAND_LAND = 21
MISSION_COMMAND_TAKEOFF = 22
MISSION_FRAME_GLOBAL_RELATIVE_ALT = 3
MISSION_FRAME_GLOBAL_RELATIVE_ALT_INT = 6
MISSION_FRAME_GLOBAL = 0
MISSION_TYPE_MISSION = 0

# ArduPilot mission wire convention ([FACT] AP_Mission / MissionItemProtocol):
# storage slot 0 is the vehicle HOME (a MAV_CMD_NAV_WAYPOINT emitted in the
# GLOBAL MSL frame); it is auto-inserted on the first appended item
# (AP_Mission::add_cmd), writes to slot 0 are silently ignored
# (AP_Mission::replace_cmd), and downloads always expose home as seq 0
# (MissionItemProtocol_Waypoints::get_item "always allow HOME to be read").
# mavctl therefore transfers its v1 items in wire sequence space 1..N.
ARDUPILOT_HOME_SLOT_SEQ = 0
"""Wire sequence of the vehicle-managed home slot on ArduPilot."""

MISSION_RESULT_NAMES: dict[int, str] = {
    0: "ACCEPTED",
    1: "ERROR",
    2: "UNSUPPORTED_FRAME",
    3: "UNSUPPORTED",
    4: "NO_SPACE",
    5: "INVALID",
    6: "INVALID_PARAM1",
    7: "INVALID_PARAM2",
    8: "INVALID_PARAM3",
    9: "INVALID_PARAM4",
    10: "INVALID_PARAM5_X",
    11: "INVALID_PARAM6_Y",
    12: "INVALID_PARAM7",
    13: "INVALID_SEQUENCE",
    14: "DENIED",
    15: "OPERATION_CANCELLED",
}
"""Names for the ``MISSION_ACK.type`` values (MAV_MISSION_RESULT)."""


def mission_result_name(result: int) -> str:
    """Human-readable name for a ``MISSION_ACK.type`` value."""

    return MISSION_RESULT_NAMES.get(result, f"MISSION_RESULT_{result}")


class UnsupportedRemoteMissionItem(ValueError):
    """A remote ``MISSION_ITEM_INT`` cannot be represented in the v1 schema.

    Raised by the pure remote-conversion helper; the adapter translates it
    into :class:`~mavctl.adapter.base.MissionItemUnsupportedError`.
    """


# -- strict item models ------------------------------------------------------

_FiniteFloat = Annotated[float, Field(allow_inf_nan=False)]
_DegreesLatitude = Annotated[float, Field(ge=-90.0, le=90.0, allow_inf_nan=False)]
_DegreesLongitude = Annotated[float, Field(ge=-180.0, le=180.0, allow_inf_nan=False)]
_NonNegativeFloat = Annotated[float, Field(ge=0.0, allow_inf_nan=False)]
_PositiveAltitude = Annotated[float, Field(gt=0.0, allow_inf_nan=False)]


class _StrictItem(BaseModel):
    """Base for mission items: unknown keys are rejected, types are strict."""

    model_config = ConfigDict(extra="forbid")


class MissionTakeoff(_StrictItem):
    """Vertical takeoff to a relative altitude (x/y encode as 0 — ArduCopter
    substitutes the current position for zero coordinates)."""

    type: Literal["takeoff"] = "takeoff"
    altitude_m: _PositiveAltitude


class MissionWaypoint(_StrictItem):
    type: Literal["waypoint"] = "waypoint"
    lat_deg: _DegreesLatitude
    lon_deg: _DegreesLongitude
    altitude_m: _PositiveAltitude
    hold_s: _NonNegativeFloat = 0.0
    accept_radius_m: _NonNegativeFloat = 0.0


class MissionLand(_StrictItem):
    type: Literal["land"] = "land"
    lat_deg: _DegreesLatitude
    lon_deg: _DegreesLongitude
    altitude_m: _PositiveAltitude
    abort_alt_m: _NonNegativeFloat = 0.0


class MissionRtl(_StrictItem):
    """Return to launch; the vehicle ignores any location on RTL items."""

    type: Literal["rtl"] = "rtl"


MissionItem = Annotated[
    MissionTakeoff | MissionWaypoint | MissionLand | MissionRtl,
    Field(discriminator="type"),
]


class MissionV1(BaseModel):
    """Upload schema v1: non-empty, takeoff-first, terminal-terminated."""

    model_config = ConfigDict(extra="forbid")

    version: Annotated[int, Field(strict=True)]
    items: Annotated[
        list[MissionItem],
        Field(min_length=1, max_length=MISSION_MAX_ITEMS),
    ]

    @field_validator("version")
    @classmethod
    def _exact_version(cls, value: int) -> int:
        # Strict integer check: JSON ``1.0`` / ``true`` are rejected so the
        # schema version is deterministic for agents and Harness exports.
        if type(value) is not int or value != 1:
            raise ValueError("unsupported mission schema version; expected exactly 1")
        return value

    @model_validator(mode="after")
    def _check_sequence_rules(self) -> MissionV1:
        if not isinstance(self.items[0], MissionTakeoff):
            raise ValueError(
                "first mission item must be 'takeoff' (ArduCopter-first v1 constraint)"
            )
        for index, item in enumerate(self.items[:-1]):
            if isinstance(item, (MissionLand, MissionRtl)):
                raise ValueError(
                    f"no items may follow terminal 'land'/'rtl' item at index {index}"
                )
        return self


class DownloadedMissionV1(BaseModel):
    """Download schema v1: mirrors ``MissionV1`` but permits an empty item
    list (a remote vehicle with no mission). Empty lists remain invalid for
    upload."""

    model_config = ConfigDict(extra="forbid")

    version: Annotated[int, Field(strict=True)]
    items: Annotated[
        list[MissionItem],
        Field(max_length=MISSION_MAX_ITEMS),
    ]

    @field_validator("version")
    @classmethod
    def _exact_version(cls, value: int) -> int:
        if type(value) is not int or value != 1:
            raise ValueError("unsupported mission schema version; expected exactly 1")
        return value


# -- transaction outcomes ----------------------------------------------------


class MissionOutcome(BaseModel):
    """Terminal result of an upload or clear transaction.

    ``accepted`` mirrors the vehicle's terminal ``MISSION_ACK``; ``verified``
    and ``observed_count`` are populated by clear's read-back verification.
    """

    model_config = ConfigDict(extra="forbid")

    action: Literal["mission_upload", "mission_clear"]
    accepted: bool
    result_name: str = "ACCEPTED"
    item_count: int | None = None
    verified: bool | None = None
    observed_count: int | None = None
    sent_upto: int | None = None


# -- pure mapping helpers ----------------------------------------------------


class MissionItemIntFields(BaseModel):
    """Pure-data ``MISSION_ITEM_INT`` payload for one semantic item.

    The adapter turns this into the actual pymavlink call; nothing here
    imports pymavlink.
    """

    model_config = ConfigDict(extra="forbid")

    command: int
    param1: float
    param2: float
    param3: float
    param4: float
    x: int
    y: int
    z: float
    frame: int = MISSION_FRAME_GLOBAL_RELATIVE_ALT_INT


def mission_item_to_int_fields(item: MissionItem) -> MissionItemIntFields:
    """Map one semantic mission item to its ``MISSION_ITEM_INT`` fields.

    Pure and independently testable; encoding per the v1 design (frame 6,
    ``current=0`` / ``autocontinue=1`` are added by the adapter transaction,
    takeoff/rtl use ``x=0, y=0`` per the verified ArduCopter behavior).
    """

    if isinstance(item, MissionTakeoff):
        return MissionItemIntFields(
            command=MISSION_COMMAND_TAKEOFF,
            param1=0.0,  # min_pitch_deg intentionally not exposed in v1
            param2=0.0,
            param3=0.0,
            param4=0.0,
            x=0,
            y=0,
            z=item.altitude_m,
        )
    if isinstance(item, MissionWaypoint):
        return MissionItemIntFields(
            command=MISSION_COMMAND_WAYPOINT,
            param1=item.hold_s,
            param2=item.accept_radius_m,
            param3=0.0,  # pass radius is a v1 non-goal: pass through the WP
            param4=0.0,  # yaw is a v1 non-goal
            x=round(item.lat_deg * 1e7),
            y=round(item.lon_deg * 1e7),
            z=item.altitude_m,
        )
    if isinstance(item, MissionLand):
        return MissionItemIntFields(
            command=MISSION_COMMAND_LAND,
            param1=item.abort_alt_m,  # 0 = undefined / vehicle default
            param2=0.0,  # precision land mode is a v1 non-goal
            param3=0.0,
            param4=0.0,
            x=round(item.lat_deg * 1e7),
            y=round(item.lon_deg * 1e7),
            z=item.altitude_m,
        )
    # MissionRtl
    return MissionItemIntFields(
        command=MISSION_COMMAND_RETURN_TO_LAUNCH,
        param1=0.0,
        param2=0.0,
        param3=0.0,
        param4=0.0,
        x=0,
        y=0,
        z=0.0,
    )


def home_slot_int_fields() -> MissionItemIntFields:
    """Build the wire item mavctl sends for the ArduPilot home slot (seq 0).

    ArduPilot never stores this wire item's content: on a non-empty mission
    ``AP_Mission::replace_cmd(0)`` is a documented no-op ("writing index zero
    is not allowed, it must be home"), and on a cleared mission the first
    append auto-writes home to slot 0 and the item lands in slot 1, where the
    first real v1 item immediately replaces it. The content is therefore a
    canonical, inert waypoint; it exists purely to satisfy the strict
    in-order item transfer the vehicle drives.
    """

    return MissionItemIntFields(
        command=MISSION_COMMAND_WAYPOINT,
        param1=0.0,
        param2=0.0,
        param3=0.0,
        param4=0.0,
        x=0,
        y=0,
        z=0.0,
    )


def is_home_slot_item(
    *,
    command: int,
    frame: int,
) -> bool:
    """Recognize the vehicle-managed home entry at download seq 0.

    ArduPilot stores home as a ``MAV_CMD_NAV_WAYPOINT`` and emits it in the
    GLOBAL (MSL) frame — ``mission_cmd_to_mavlink_int`` sets frame 0 for any
    non-relative location [FACT, SITL-verified]. A v1 item can never have
    this signature (v1 emits relative frames only), so a seq-0 item that does
    not match is treated as a real first item of a non-ArduPilot-convention
    vehicle and fails the download atomically instead of being misread.
    """

    return command == MISSION_COMMAND_WAYPOINT and frame == MISSION_FRAME_GLOBAL


def mission_item_from_remote(
    *,
    seq: int,
    command: int,
    frame: int,
    param1: float,
    param2: float,
    param3: float,
    param4: float,
    x: int,
    y: int,
    z: float,
) -> MissionItem:
    """Convert a received ``MISSION_ITEM_INT`` into a semantic v1 item.

    Lossless policy: the conversion succeeds only when the remote item is
    exactly what mavctl v1 would have sent (plus schema-representable
    values for the exposed optional fields). Any parameter that v1 does not
    express, or whose default/unset semantics cannot be verified, makes the
    item unsupported — download fails atomically instead of silently
    dropping a non-default behaviour.

    Frames: both relative-altitude encodings are accepted as equivalent —
    MAV_FRAME_GLOBAL_RELATIVE_ALT_INT (6, what mavctl sends) and
    MAV_FRAME_GLOBAL_RELATIVE_ALT (3, what ArduPilot emits back in
    ``mission_cmd_to_mavlink_int`` [FACT]). They denote the same frame for
    integer messages. RTL additionally accepts frame 0 (GLOBAL MSL): its
    position is ignored by the vehicle, so the frame encodes no v1-relevant
    information ([FACT] ArduPilot stores RTL with x=0/y=0 as absolute and
    emits frame 0 back).

    Raises:
        UnsupportedRemoteMissionItem: if the frame or command is outside the
            v1 whitelist, a parameter that v1 does not express is non-default,
            or the fields fall outside the strict schema bounds. The
            conversion is deliberately conservative: download fails
            atomically instead of emitting a lossy approximation.
    """

    def _unsupported(why: str) -> UnsupportedRemoteMissionItem:
        return UnsupportedRemoteMissionItem(f"seq {seq}: {why}")

    _relative = (MISSION_FRAME_GLOBAL_RELATIVE_ALT, MISSION_FRAME_GLOBAL_RELATIVE_ALT_INT)
    _rtl_abs = command == MISSION_COMMAND_RETURN_TO_LAUNCH and frame == MISSION_FRAME_GLOBAL
    if frame not in _relative and not _rtl_abs:
        raise _unsupported(f"unsupported frame {frame}")
    try:
        if command == MISSION_COMMAND_TAKEOFF:
            # param1 = min pitch, param2/param3 empty, param4 = yaw — none are
            # expressed by the v1 schema, so all must be the canonical zeros.
            # A non-zero x/y encodes a takeoff position v1 cannot represent.
            if (param1, param2, param3, param4) != (0.0, 0.0, 0.0, 0.0):
                raise _unsupported("non-default takeoff parameters (param1..4)")
            if x != 0 or y != 0:
                raise _unsupported("non-zero takeoff coordinates (x/y)")
            return MissionTakeoff(altitude_m=z)
        if command == MISSION_COMMAND_WAYPOINT:
            # param1 = hold time and param2 = acceptance radius are v1
            # fields; param3 (pass radius) and param4 (yaw) are not, so they
            # must carry the canonical pass-through zero.
            if param3 != 0.0:
                raise _unsupported("non-default waypoint pass radius (param3)")
            if param4 != 0.0:
                raise _unsupported("non-default waypoint yaw (param4)")
            return MissionWaypoint(
                lat_deg=x / 1e7,
                lon_deg=y / 1e7,
                altitude_m=z,
                hold_s=param1,
                accept_radius_m=param2,
            )
        if command == MISSION_COMMAND_LAND:
            # param1 = abort altitude is a v1 field; param2 (precision land
            # mode), param3 and param4 (yaw) are not.
            if param2 != 0.0:
                raise _unsupported("non-default land precision mode (param2)")
            if param3 != 0.0:
                raise _unsupported("non-default land parameter (param3)")
            if param4 != 0.0:
                raise _unsupported("non-default land yaw (param4)")
            return MissionLand(
                lat_deg=x / 1e7,
                lon_deg=y / 1e7,
                altitude_m=z,
                abort_alt_m=param1,
            )
        if command == MISSION_COMMAND_RETURN_TO_LAUNCH:
            # All params are "Empty" per the XML and the location is ignored
            # by the vehicle (do_RTL takes no location), so only the params
            # must be canonical zeros for the item to round-trip. The frame
            # is wire-preserved but semantically ignored: ArduPilot stores
            # x=0/y=0 as an absolute location, so it emits frame 0 (GLOBAL
            # MSL) on download [FACT, SITL-verified] even though mavctl sent
            # frame 6 — either encoding is accepted because the position
            # plays no role in RTL execution.
            if (param1, param2, param3, param4) != (0.0, 0.0, 0.0, 0.0):
                raise _unsupported("non-default rtl parameters (param1..4)")
            return MissionRtl()
        raise _unsupported(f"unsupported command {command}")
    except ValidationError as exc:
        raise _unsupported(f"item outside v1 schema bounds ({exc.error_count()} errors)") from exc
