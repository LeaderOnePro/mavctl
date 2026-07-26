"""Synchronous Unix-socket client used by the CLI to talk to the daemon."""

from __future__ import annotations

import socket
from typing import Any

from mavctl.daemon import wire
from mavctl.models import DaemonResponse
from mavctl.paths import socket_path


class DaemonNotRunningError(Exception):
    """Raised when no daemon is listening on the expected socket."""


def _read_frame(sock: socket.socket) -> bytes:
    chunks: list[bytes] = []
    while True:
        chunk = sock.recv(4096)
        if not chunk:
            break
        chunks.append(chunk)
        if b"\n" in chunk:
            break
    return b"".join(chunks)


def call_daemon(
    method: str,
    params: dict[str, Any] | None = None,
    timeout: float = 5.0,
) -> DaemonResponse:
    """Send one RPC request to the daemon and return its response.

    Raises:
        DaemonNotRunningError: if the socket is absent or refuses the connection.
    """

    path = socket_path()
    sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    sock.settimeout(timeout)
    try:
        try:
            sock.connect(str(path))
        except (FileNotFoundError, ConnectionRefusedError) as exc:
            raise DaemonNotRunningError(str(exc)) from exc
        sock.sendall(wire.encode({"method": method, "params": params or {}}))
        line = _read_frame(sock)
    finally:
        sock.close()

    if not line:
        raise DaemonNotRunningError("daemon closed the connection without responding")
    return DaemonResponse.model_validate(wire.decode(line))


def is_daemon_running() -> bool:
    """Return True if a daemon answers a ping on the socket."""

    try:
        return call_daemon("ping", timeout=1.0).ok
    except DaemonNotRunningError:
        return False
    except (OSError, ValueError):
        return False


class DaemonClient:
    """Thin object wrapper around :func:`call_daemon` for callers that
    prefer a handle (e.g. tests injecting a fake)."""

    def __init__(self, timeout: float = 5.0) -> None:
        self._timeout = timeout

    def call(self, method: str, params: dict[str, Any] | None = None) -> DaemonResponse:
        return call_daemon(method, params, timeout=self._timeout)

    def is_running(self) -> bool:
        return is_daemon_running()
