"""MAVLink link identity defaults shared by CLI, daemon, and adapter.

mavctl presents its own GCS identity on the wire, independent of the vehicle
identity it talks to. The mainstream ArduPilot ecosystem GCS convention is
source system 255 (MAVProxy, Mission Planner, QGround Control all default
there), so mavctl defaults to a distinct system id to coexist with a
conventional GCS on the same link instead of racing it.
"""

from __future__ import annotations

#: mavctl's default GCS source system id. Deliberately distinct from the
#: ecosystem-standard 255 so mavctl and a conventional GCS never share an
#: on-wire identity (ArduPilot rejects mission items whose sender identity
#: differs from the MISSION_COUNT sender — MAV_MISSION_DENIED — so shared
#: identities can cross-contaminate concurrent transfers).
DEFAULT_GCS_SOURCE_SYSTEM = 254

#: mavctl's default GCS source component id: the standard "mission planner /
#: ground station" component. Combined with the source system this keeps the
#: (system, component) pair unique next to MAVProxy's default (255, 230).
DEFAULT_GCS_SOURCE_COMPONENT = 190

_MIN_SOURCE_SYSTEM = 1
_MAX_SOURCE_SYSTEM = 255


def validate_source_system(value: int) -> int:
    """Validate a GCS source system id and return it unchanged.

    Raises:
        ValueError: if the value is not an integer in ``1..255``. System id 0
            is reserved as "unspecified" in MAVLink and is never a valid GCS
            identity.
    """

    if isinstance(value, bool) or not isinstance(value, int):
        raise ValueError(f"source system must be an integer, got {value!r}")
    if not (_MIN_SOURCE_SYSTEM <= value <= _MAX_SOURCE_SYSTEM):
        raise ValueError(
            f"source system must be in {_MIN_SOURCE_SYSTEM}..{_MAX_SOURCE_SYSTEM}, "
            f"got {value}"
        )
    return value
