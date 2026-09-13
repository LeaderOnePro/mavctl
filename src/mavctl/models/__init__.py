"""Core pydantic data models shared across all layers.

This package is a leaf dependency: it must not import from ``cli``, ``daemon``,
or ``adapter``.
"""

from mavctl.models.commands import MAV_RESULT_NAMES, CommandOutcome, WaitStatus
from mavctl.models.mission import (
    MISSION_COMMAND_LAND,
    MISSION_COMMAND_RETURN_TO_LAUNCH,
    MISSION_COMMAND_TAKEOFF,
    MISSION_COMMAND_WAYPOINT,
    MISSION_FRAME_GLOBAL_RELATIVE_ALT_INT,
    MISSION_MAX_ITEMS,
    MISSION_RESULT_NAMES,
    MISSION_TYPE_MISSION,
    DownloadedMissionV1,
    MissionItem,
    MissionItemIntFields,
    MissionLand,
    MissionOutcome,
    MissionRtl,
    MissionTakeoff,
    MissionV1,
    MissionWaypoint,
    UnsupportedRemoteMissionItem,
    mission_item_from_remote,
    mission_item_to_int_fields,
    mission_result_name,
)
from mavctl.models.protocol import (
    DaemonResponse,
    ExitCode,
    RpcError,
    RpcRequest,
)
from mavctl.models.state import Battery, GpsInfo, HomePosition, VehicleState
from mavctl.models.telemetry import Attitude, Position, Telemetry, Velocity

__all__ = [
    "MAV_RESULT_NAMES",
    "MISSION_COMMAND_LAND",
    "MISSION_COMMAND_RETURN_TO_LAUNCH",
    "MISSION_COMMAND_TAKEOFF",
    "MISSION_COMMAND_WAYPOINT",
    "MISSION_FRAME_GLOBAL_RELATIVE_ALT_INT",
    "MISSION_MAX_ITEMS",
    "MISSION_RESULT_NAMES",
    "MISSION_TYPE_MISSION",
    "Attitude",
    "Battery",
    "CommandOutcome",
    "DaemonResponse",
    "DownloadedMissionV1",
    "ExitCode",
    "GpsInfo",
    "HomePosition",
    "MissionItem",
    "MissionItemIntFields",
    "MissionLand",
    "MissionOutcome",
    "MissionRtl",
    "MissionTakeoff",
    "MissionV1",
    "MissionWaypoint",
    "Position",
    "RpcError",
    "RpcRequest",
    "Telemetry",
    "UnsupportedRemoteMissionItem",
    "VehicleState",
    "Velocity",
    "WaitStatus",
    "mission_item_from_remote",
    "mission_item_to_int_fields",
    "mission_result_name",
]
