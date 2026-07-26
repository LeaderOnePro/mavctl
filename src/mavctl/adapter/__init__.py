"""Adapter layer — the ONLY place pymavlink may be imported.

The rest of the codebase depends solely on the :class:`VehicleAdapter`
protocol defined here, keeping the MAVLink transport swappable and testable.
"""

from mavctl.adapter.base import AdapterError, ConnectionLostError, VehicleAdapter

__all__ = ["AdapterError", "ConnectionLostError", "VehicleAdapter"]
