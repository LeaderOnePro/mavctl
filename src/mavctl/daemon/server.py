"""Asyncio Unix-socket server that owns the vehicle link and answers RPCs."""

from __future__ import annotations

import asyncio
import contextlib
import signal
from typing import TYPE_CHECKING

from mavctl.adapter.base import VehicleAdapter
from mavctl.daemon import wire
from mavctl.models import DaemonResponse, ExitCode, RpcRequest
from mavctl.paths import runtime_dir, socket_path

if TYPE_CHECKING:
    from collections.abc import Callable

# Max bytes accepted for a single request frame (defensive bound).
_MAX_FRAME = 64 * 1024


class DaemonServer:
    """Serves RPC requests over a Unix socket, backed by a vehicle adapter.

    The adapter maintains the live MAVLink snapshot on its own reader thread;
    the server's request handlers only perform cheap snapshot reads, so no
    handler blocks the event loop.
    """

    def __init__(self, adapter: VehicleAdapter, connection_string: str) -> None:
        self._adapter = adapter
        self._connection_string = connection_string
        self._stop_event = asyncio.Event()
        self._server: asyncio.AbstractServer | None = None
        self._methods: dict[str, Callable[[RpcRequest], DaemonResponse]] = {
            "ping": self._m_ping,
            "status": self._m_status,
            "telemetry": self._m_telemetry,
            "shutdown": self._m_shutdown,
        }

    async def serve(self) -> None:
        """Open the link, bind the socket, and serve until asked to stop."""

        self._adapter.connect()
        runtime_dir().mkdir(parents=True, exist_ok=True)
        sock = socket_path()
        with contextlib.suppress(FileNotFoundError):
            sock.unlink()

        self._install_signal_handlers()
        self._server = await asyncio.start_unix_server(self._handle_client, path=str(sock))
        try:
            await self._stop_event.wait()
        finally:
            await self._shutdown()

    def request_stop(self) -> None:
        """Signal the serve loop to unwind (safe to call from a signal handler)."""

        self._stop_event.set()

    # -- connection handling ----------------------------------------------

    async def _handle_client(
        self, reader: asyncio.StreamReader, writer: asyncio.StreamWriter
    ) -> None:
        try:
            line = await reader.readline()
            if not line or len(line) > _MAX_FRAME:
                return
            response = self._dispatch(line)
            writer.write(wire.encode(response.model_dump()))
            await writer.drain()
        except (ConnectionError, asyncio.IncompleteReadError):
            return
        finally:
            writer.close()
            with contextlib.suppress(ConnectionError, asyncio.CancelledError):
                await writer.wait_closed()

    def _dispatch(self, line: bytes) -> DaemonResponse:
        try:
            request = RpcRequest.model_validate(wire.decode(line))
        except (ValueError, TypeError) as exc:
            return DaemonResponse.failure(ExitCode.USAGE_ERROR, f"malformed request: {exc}")

        handler = self._methods.get(request.method)
        if handler is None:
            return DaemonResponse.failure(
                ExitCode.GENERAL_ERROR, f"unknown method: {request.method!r}"
            )
        try:
            return handler(request)
        except Exception as exc:
            return DaemonResponse.failure(ExitCode.GENERAL_ERROR, f"internal error: {exc}")

    # -- RPC methods -------------------------------------------------------

    def _m_ping(self, _request: RpcRequest) -> DaemonResponse:
        return DaemonResponse.success(
            {"pong": True, "connection_string": self._connection_string}
        )

    def _m_status(self, _request: RpcRequest) -> DaemonResponse:
        return DaemonResponse.success(self._adapter.get_state().model_dump())

    def _m_telemetry(self, _request: RpcRequest) -> DaemonResponse:
        state = self._adapter.get_state()
        if not state.connected:
            return DaemonResponse.failure(
                ExitCode.VEHICLE_NOT_CONNECTED,
                "vehicle not connected (no recent heartbeat)",
                {"connection_string": self._connection_string},
            )
        return DaemonResponse.success(self._adapter.get_telemetry().model_dump())

    def _m_shutdown(self, _request: RpcRequest) -> DaemonResponse:
        self.request_stop()
        return DaemonResponse.success({"stopping": True})

    # -- teardown ----------------------------------------------------------

    def _install_signal_handlers(self) -> None:
        loop = asyncio.get_running_loop()
        for sig in (signal.SIGTERM, signal.SIGINT):
            with contextlib.suppress(NotImplementedError):
                loop.add_signal_handler(sig, self.request_stop)

    async def _shutdown(self) -> None:
        if self._server is not None:
            self._server.close()
            with contextlib.suppress(Exception):
                await self._server.wait_closed()
            self._server = None
        self._adapter.disconnect()
        with contextlib.suppress(FileNotFoundError):
            socket_path().unlink()
