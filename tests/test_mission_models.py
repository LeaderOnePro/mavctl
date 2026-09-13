"""Unit tests for the Mission JSON v1 schema and MISSION_ITEM_INT mapping."""

from __future__ import annotations

import math

import pytest
from pydantic import ValidationError

from mavctl.models import (
    MISSION_COMMAND_LAND,
    MISSION_COMMAND_RETURN_TO_LAUNCH,
    MISSION_COMMAND_TAKEOFF,
    MISSION_COMMAND_WAYPOINT,
    MISSION_FRAME_GLOBAL_RELATIVE_ALT_INT,
    MISSION_MAX_ITEMS,
    DownloadedMissionV1,
    MissionLand,
    MissionRtl,
    MissionTakeoff,
    MissionV1,
    MissionWaypoint,
    mission_item_from_remote,
    mission_item_to_int_fields,
)


def _four_item_mission() -> dict:
    return {
        "version": 1,
        "items": [
            {"type": "takeoff", "altitude_m": 10.0},
            {
                "type": "waypoint",
                "lat_deg": -35.3632621,
                "lon_deg": 149.1652374,
                "altitude_m": 20.0,
                "hold_s": 0.0,
                "accept_radius_m": 0.0,
            },
            {"type": "waypoint", "lat_deg": -35.37, "lon_deg": 149.17, "altitude_m": 25.0},
            {"type": "rtl"},
        ],
    }


# -- valid schemas -----------------------------------------------------------


def test_valid_four_item_mission_parses() -> None:
    mission = MissionV1.model_validate(_four_item_mission())

    assert mission.version == 1
    assert len(mission.items) == 4
    assert isinstance(mission.items[0], MissionTakeoff)
    assert isinstance(mission.items[1], MissionWaypoint)
    assert isinstance(mission.items[2], MissionWaypoint)
    assert isinstance(mission.items[3], MissionRtl)


def test_waypoint_defaults_are_zero() -> None:
    item = MissionWaypoint.model_validate(
        {"type": "waypoint", "lat_deg": 1.0, "lon_deg": 2.0, "altitude_m": 5.0}
    )
    assert item.hold_s == 0.0
    assert item.accept_radius_m == 0.0


def test_land_abort_alt_defaults_to_zero() -> None:
    item = MissionLand.model_validate(
        {"type": "land", "lat_deg": 1.0, "lon_deg": 2.0, "altitude_m": 0.5}
    )
    assert item.abort_alt_m == 0.0


def test_empty_download_mission_is_representable() -> None:
    mission = DownloadedMissionV1.model_validate({"version": 1, "items": []})
    assert mission.items == []


# -- schema rejections -------------------------------------------------------


def test_empty_upload_mission_rejected() -> None:
    with pytest.raises(ValidationError):
        MissionV1.model_validate({"version": 1, "items": []})


def test_missing_version_rejected() -> None:
    payload = _four_item_mission()
    del payload["version"]
    with pytest.raises(ValidationError):
        MissionV1.model_validate(payload)


@pytest.mark.parametrize("version", [0, 2, "1", 1.0])
def test_wrong_version_rejected(version: object) -> None:
    payload = _four_item_mission()
    payload["version"] = version
    with pytest.raises(ValidationError):
        MissionV1.model_validate(payload)


def test_unknown_type_rejected() -> None:
    payload = _four_item_mission()
    payload["items"][1]["type"] = "survey"
    with pytest.raises(ValidationError):
        MissionV1.model_validate(payload)


@pytest.mark.parametrize("spelling", ["TAKEOFF", "Takeoff", "tAkEoFf"])
def test_non_lowercase_type_spelling_rejected(spelling: str) -> None:
    payload = _four_item_mission()
    payload["items"][0]["type"] = spelling
    with pytest.raises(ValidationError):
        MissionV1.model_validate(payload)


def test_extra_item_fields_rejected() -> None:
    payload = _four_item_mission()
    payload["items"][0]["min_pitch_deg"] = 5.0  # intentionally not in v1
    with pytest.raises(ValidationError):
        MissionV1.model_validate(payload)


def test_extra_top_level_fields_rejected() -> None:
    payload = _four_item_mission()
    payload["name"] = "inspection"
    with pytest.raises(ValidationError):
        MissionV1.model_validate(payload)


@pytest.mark.parametrize("bad", [float("nan"), float("inf"), -float("inf")])
def test_non_finite_numbers_rejected(bad: float) -> None:
    payload = _four_item_mission()
    payload["items"][1]["lat_deg"] = bad
    with pytest.raises(ValidationError):
        MissionV1.model_validate(payload)


@pytest.mark.parametrize("lat", [-90.0, 90.0, -89.999999])
def test_latitude_boundaries_accepted(lat: float) -> None:
    payload = _four_item_mission()
    payload["items"][1]["lat_deg"] = lat
    MissionV1.model_validate(payload)


@pytest.mark.parametrize("lat", [-90.000001, 90.000001])
def test_latitude_out_of_range_rejected(lat: float) -> None:
    payload = _four_item_mission()
    payload["items"][1]["lat_deg"] = lat
    with pytest.raises(ValidationError):
        MissionV1.model_validate(payload)


@pytest.mark.parametrize("lon", [-180.0, 180.0])
def test_longitude_boundaries_accepted(lon: float) -> None:
    payload = _four_item_mission()
    payload["items"][1]["lon_deg"] = lon
    MissionV1.model_validate(payload)


@pytest.mark.parametrize("lon", [-180.000001, 180.000001])
def test_longitude_out_of_range_rejected(lon: float) -> None:
    payload = _four_item_mission()
    payload["items"][1]["lon_deg"] = lon
    with pytest.raises(ValidationError):
        MissionV1.model_validate(payload)


@pytest.mark.parametrize("alt", [0.0, -1.0])
def test_non_positive_altitude_rejected(alt: float) -> None:
    payload = _four_item_mission()
    payload["items"][0]["altitude_m"] = alt
    with pytest.raises(ValidationError):
        MissionV1.model_validate(payload)


@pytest.mark.parametrize("field", ["hold_s", "accept_radius_m"])
def test_negative_waypoint_optionals_rejected(field: str) -> None:
    payload = _four_item_mission()
    payload["items"][1][field] = -0.1
    with pytest.raises(ValidationError):
        MissionV1.model_validate(payload)


def test_negative_land_abort_alt_rejected() -> None:
    with pytest.raises(ValidationError):
        MissionLand.model_validate(
            {"type": "land", "lat_deg": 1.0, "lon_deg": 2.0,
             "altitude_m": 1.0, "abort_alt_m": -1.0}
        )


def test_first_item_must_be_takeoff() -> None:
    payload = _four_item_mission()
    payload["items"][0] = {"type": "waypoint", "lat_deg": 1.0, "lon_deg": 2.0,
                           "altitude_m": 5.0}
    with pytest.raises(ValidationError, match="takeoff"):
        MissionV1.model_validate(payload)


def test_no_items_after_terminal_rtl() -> None:
    payload = _four_item_mission()
    payload["items"].append({"type": "waypoint", "lat_deg": 1.0, "lon_deg": 2.0,
                             "altitude_m": 5.0})
    with pytest.raises(ValidationError, match="terminal"):
        MissionV1.model_validate(payload)


def test_no_items_after_terminal_land() -> None:
    payload = _four_item_mission()
    payload["items"] = [
        *payload["items"][:3],
        {"type": "land", "lat_deg": 1.0, "lon_deg": 2.0, "altitude_m": 0.5},
        {"type": "rtl"},
    ]
    with pytest.raises(ValidationError, match="terminal"):
        MissionV1.model_validate(payload)


def test_item_count_limit_enforced() -> None:
    payload = _four_item_mission()
    items = [payload["items"][1]] * (MISSION_MAX_ITEMS + 1)
    payload["items"] = [{"type": "takeoff", "altitude_m": 10.0}, *items]
    with pytest.raises(ValidationError):
        MissionV1.model_validate(payload)


def test_rtl_rejects_any_extra_field() -> None:
    with pytest.raises(ValidationError):
        MissionRtl.model_validate({"type": "rtl", "altitude_m": 10.0})


# -- mapping -----------------------------------------------------------------


def test_mapping_takeoff() -> None:
    fields = mission_item_to_int_fields(MissionTakeoff(altitude_m=10.0))
    assert fields.command == MISSION_COMMAND_TAKEOFF
    assert (fields.param1, fields.param2, fields.param3, fields.param4) == (0.0, 0.0, 0.0, 0.0)
    assert (fields.x, fields.y, fields.z) == (0, 0, 10.0)
    assert fields.frame == MISSION_FRAME_GLOBAL_RELATIVE_ALT_INT


def test_mapping_waypoint_1e7_scaling() -> None:
    item = MissionWaypoint(
        lat_deg=-35.3632621, lon_deg=149.1652374, altitude_m=20.0, hold_s=3.0,
        accept_radius_m=2.5,
    )
    fields = mission_item_to_int_fields(item)
    assert fields.command == MISSION_COMMAND_WAYPOINT
    assert fields.param1 == pytest.approx(3.0)
    assert fields.param2 == pytest.approx(2.5)
    assert fields.param3 == 0.0
    assert fields.param4 == 0.0
    assert fields.x == round(-35.3632621 * 1e7) == -353632621
    assert fields.y == round(149.1652374 * 1e7) == 1491652374
    assert fields.z == 20.0
    assert fields.frame == MISSION_FRAME_GLOBAL_RELATIVE_ALT_INT


def test_mapping_land() -> None:
    fields = mission_item_to_int_fields(
        MissionLand(lat_deg=1.5, lon_deg=-2.5, altitude_m=0.5, abort_alt_m=12.0)
    )
    assert fields.command == MISSION_COMMAND_LAND
    assert fields.param1 == 12.0
    assert fields.x == 15000000
    assert fields.y == -25000000
    assert fields.z == 0.5


def test_mapping_rtl() -> None:
    fields = mission_item_to_int_fields(MissionRtl())
    assert fields.command == MISSION_COMMAND_RETURN_TO_LAUNCH
    assert (fields.param1, fields.param2, fields.param3, fields.param4) == (0.0, 0.0, 0.0, 0.0)
    assert (fields.x, fields.y, fields.z) == (0, 0, 0.0)


# -- remote conversion -------------------------------------------------------


def test_remote_waypoint_round_trips() -> None:
    item = mission_item_from_remote(
        seq=1,
        command=MISSION_COMMAND_WAYPOINT,
        frame=MISSION_FRAME_GLOBAL_RELATIVE_ALT_INT,
        param1=2.0,
        param2=1.5,
        param3=0.0,
        param4=0.0,
        x=-353632621,
        y=1491652374,
        z=20.0,
    )
    assert isinstance(item, MissionWaypoint)
    assert item.lat_deg == pytest.approx(-35.3632621)
    assert item.lon_deg == pytest.approx(149.1652374)
    assert item.altitude_m == 20.0
    assert item.hold_s == 2.0
    assert item.accept_radius_m == 1.5


def test_remote_rtl_ignores_location_fields() -> None:
    item = mission_item_from_remote(
        seq=3, command=MISSION_COMMAND_RETURN_TO_LAUNCH, frame=6,
        param1=0.0, param2=0.0, param3=0.0, param4=0.0, x=12345, y=6789, z=99.0,
    )
    assert isinstance(item, MissionRtl)


def test_remote_unsupported_command_rejected() -> None:
    with pytest.raises(ValueError, match="unsupported command 999"):
        mission_item_from_remote(
            seq=0, command=999, frame=6, param1=0.0, param2=0.0, param3=0.0,
            param4=0.0, x=0, y=0, z=0.0,
        )


def test_remote_unsupported_frame_rejected() -> None:
    with pytest.raises(ValueError, match="unsupported frame 3"):
        mission_item_from_remote(
            seq=0, command=MISSION_COMMAND_WAYPOINT, frame=3,
            param1=0.0, param2=0.0, param3=0.0, param4=0.0, x=1, y=2, z=3.0,
        )


def test_remote_zero_altitude_takeoff_conservatively_rejected() -> None:
    # A foreign takeoff at relative altitude 0 cannot satisfy the strict v1
    # upload schema; conversion fails atomically instead of approximating.
    with pytest.raises(ValueError, match="schema bounds"):
        mission_item_from_remote(
            seq=0, command=MISSION_COMMAND_TAKEOFF, frame=6,
            param1=0.0, param2=0.0, param3=0.0, param4=0.0, x=0, y=0, z=0.0,
        )


def test_remote_out_of_range_coordinates_rejected() -> None:
    with pytest.raises(ValueError, match="schema bounds"):
        mission_item_from_remote(
            seq=2, command=MISSION_COMMAND_WAYPOINT, frame=6,
            param1=0.0, param2=0.0, param3=0.0, param4=0.0,
            x=1000000000, y=0, z=5.0,  # 100.0 deg lat: outside [-90, 90]
        )


def test_download_empty_mission_round_trip_shape() -> None:
    mission = DownloadedMissionV1.model_validate({"version": 1, "items": []})
    assert math.isfinite(float(mission.version))  # sanity; shape covered above
    assert mission.model_dump() == {"version": 1, "items": []}
