"""Adapter layer — the ONLY place pymavlink may be imported.

The rest of the codebase depends solely on the :class:`VehicleAdapter`
protocol defined here plus the :func:`create_adapter` factory, keeping the
MAVLink transport swappable and testable.
"""

from mavctl.adapter.base import AdapterError, ConnectionLostError, VehicleAdapter
from mavctl.models import DEFAULT_GCS_SOURCE_COMPONENT, DEFAULT_GCS_SOURCE_SYSTEM


def create_adapter(
    connection_string: str,
    heartbeat_timeout_s: float = 3.0,
    source_system: int = DEFAULT_GCS_SOURCE_SYSTEM,
    source_component: int = DEFAULT_GCS_SOURCE_COMPONENT,
) -> VehicleAdapter:
    """Build the default (pymavlink-backed) adapter.

    Imported lazily so pymavlink stays confined to this layer and is only
    loaded when an adapter is actually constructed.
    """

    from mavctl.adapter.pymavlink_adapter import PymavlinkAdapter

    return PymavlinkAdapter(
        connection_string,
        heartbeat_timeout_s=heartbeat_timeout_s,
        source_system=source_system,
        source_component=source_component,
    )


__all__ = [
    "AdapterError",
    "ConnectionLostError",
    "VehicleAdapter",
    "create_adapter",
]
