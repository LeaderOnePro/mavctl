"""Tests for the GCS link identity defaults and validation."""

from __future__ import annotations

import pytest

from mavctl.models import (
    DEFAULT_GCS_SOURCE_COMPONENT,
    DEFAULT_GCS_SOURCE_SYSTEM,
    validate_source_system,
)


def test_default_identity_is_distinct_from_ecosystem_gcs() -> None:
    """MAVProxy (255/230) and Mission Planner/QGC (255/…) conventionally use
    system 255; mavctl must not share that identity."""

    assert DEFAULT_GCS_SOURCE_SYSTEM != 255
    assert DEFAULT_GCS_SOURCE_SYSTEM == 254
    # component 190 = MAV_COMP_ID_MISSIONPLANNER (standard GCS component)
    assert DEFAULT_GCS_SOURCE_COMPONENT == 190


@pytest.mark.parametrize("value", [1, 2, 254, 255])
def test_validate_source_system_accepts_valid_range(value: int) -> None:
    assert validate_source_system(value) == value


@pytest.mark.parametrize("value", [0, -1, 256, 1000])
def test_validate_source_system_rejects_out_of_range(value: int) -> None:
    with pytest.raises(ValueError, match=r"1\.\.255"):
        validate_source_system(value)


@pytest.mark.parametrize("value", [1.5, "3", None, True])
def test_validate_source_system_rejects_non_integers(value: object) -> None:
    with pytest.raises(ValueError, match="integer"):
        validate_source_system(value)  # type: ignore[arg-type]
