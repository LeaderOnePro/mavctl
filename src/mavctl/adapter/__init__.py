"""Adapter layer — the ONLY place pymavlink may be imported.

The rest of the codebase depends solely on the :class:`VehicleAdapter`
protocol defined here plus the :func:`create_adapter` factory, keeping the
MAVLink transport swappable and testable.
"""

from mavctl.adapter.base import AdapterError, ConnectionLostError, VehicleAdapter


def create_adapter(connection_string: str, heartbeat_timeout_s: float = 3.0) -> VehicleAdapter:
    """Build the default (pymavlink-backed) adapter.

    Imported lazily so pymavlink stays confined to this layer and is only
    loaded when an adapter is actually constructed.
    """

    from mavctl.adapter.pymavlink_adapter import PymavlinkAdapter

    return PymavlinkAdapter(connection_string, heartbeat_timeout_s=heartbeat_timeout_s)


__all__ = [
    "AdapterError",
    "ConnectionLostError",
    "VehicleAdapter",
    "create_adapter",
]
