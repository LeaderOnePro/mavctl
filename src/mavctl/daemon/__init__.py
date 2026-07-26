"""Daemon layer — long-lived process holding the MAVLink link.

Depends on the adapter layer via its abstract interface. The CLI talks to
this layer exclusively over a Unix domain socket (see :mod:`.client`).
"""

from mavctl.daemon.client import (
    DaemonClient,
    DaemonNotRunningError,
    call_daemon,
    is_daemon_running,
)

__all__ = [
    "DaemonClient",
    "DaemonNotRunningError",
    "call_daemon",
    "is_daemon_running",
]
