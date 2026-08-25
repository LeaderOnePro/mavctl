"""Asyncio Unix-socket server that owns the vehicle link and answers RPCs."""

from __future__ import annotations

import asyncio
import contextlib
import math
import signal
from collections.abc import Awaitable, Callable
from typing import Any

from mavctl.adapter.base import AdapterError, VehicleAdapter
from mavctl.daemon import guards, wire
from mavctl.daemon.guards import GuardConfig, GuardDecision
from mavctl.models import CommandOutcome, DaemonResponse, ExitCode, RpcRequest, WaitStatus
from mavctl.paths import runtime_dir, socket_path

# Max bytes accepted for a single request frame (defensive bound).
_MAX_FRAME = 64 * 1024

# Default --wait timeout in seconds when the client does not specify one.
_DEFAULT_WAIT_TIMEOUT = 60.0

# Fraction of target altitude that counts as "takeoff reached".
_TAKEOFF_REACHED_FRACTION = 0.95

# Poll interval while --wait is active.
_WAIT_POLL_INTERVAL = 0.25

Handler = Callable[[RpcRequest], Awaitable[DaemonResponse]]


class DaemonServer:
    """Serves RPC requests over a Unix socket, backed by a vehicle adapter.

    The adapter maintains the live MAVLink snapshot on its own reader thread;
    fast handlers (ping/status/telemetry) only read that snapshot, while
    command handlers run the blocking adapter verbs in an executor so the
    event loop stays responsive.
    """

    def __init__(
        self,
        adapter: VehicleAdapter,
        connection_string: str,
        guard_config: GuardConfig | None = None,
    ) -> None:
        self._adapter = adapter
        self._connection_string = connection_string
        self._guard_config = guard_config or GuardConfig()
        self._stop_event = asyncio.Event()
        self._server: asyncio.AbstractServer | None = None
        # Serializes state-changing commands end-to-end (state read -> guard ->
        # execute -> --wait) so they never interleave (TOCTOU) and no other
        # state-changing command runs during a takeoff/land wait. Fast handlers
        # (ping/status/telemetry) do NOT take this lock and stay concurrent.
        self._command_lock = asyncio.Lock()
        self._methods: dict[str, Handler] = {
            "ping": self._m_ping,
            "status": self._m_status,
            "telemetry": self._m_telemetry,
            "shutdown": self._m_shutdown,
            "arm": self._m_arm,
            "disarm": self._m_disarm,
            "mode": self._m_mode,
            "takeoff": self._m_takeoff,
            "land": self._m_land,
            "rtl": self._m_rtl,
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
            response = await self._dispatch(line)
            writer.write(wire.encode(response.model_dump()))
            await writer.drain()
        except (ConnectionError, asyncio.IncompleteReadError):
            return
        finally:
            writer.close()
            with contextlib.suppress(ConnectionError, asyncio.CancelledError):
                await writer.wait_closed()

    async def _dispatch(self, line: bytes) -> DaemonResponse:
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
            return await handler(request)
        except Exception as exc:
            return DaemonResponse.failure(ExitCode.GENERAL_ERROR, f"internal error: {exc}")

    # -- fast RPC methods --------------------------------------------------

    async def _m_ping(self, _request: RpcRequest) -> DaemonResponse:
        return DaemonResponse.success(
            {"pong": True, "connection_string": self._connection_string}
        )

    async def _m_status(self, _request: RpcRequest) -> DaemonResponse:
        return DaemonResponse.success(self._adapter.get_state().model_dump())

    async def _m_telemetry(self, _request: RpcRequest) -> DaemonResponse:
        state = self._adapter.get_state()
        if not state.connected:
            return self._not_connected()
        return DaemonResponse.success(self._adapter.get_telemetry().model_dump())

    async def _m_shutdown(self, _request: RpcRequest) -> DaemonResponse:
        self.request_stop()
        return DaemonResponse.success({"stopping": True})

    # -- command RPC methods ----------------------------------------------
    #
    # Every state-changing command runs its whole body under ``_command_lock``:
    # the latest-state read, the guard evaluation, the dry-run/idempotent
    # decision, the adapter call, and any --wait are one serial transaction.
    # This prevents TOCTOU races and keeps other state-changing commands out
    # during a takeoff/land wait. status/telemetry never take this lock.

    async def _m_arm(self, request: RpcRequest) -> DaemonResponse:
        p = request.params
        # Force-arm is not a supported operation at any layer: even a direct
        # RPC client cannot smuggle the 21196 magic past this boundary.
        if _flag(p, "force"):
            return DaemonResponse.failure(
                ExitCode.USAGE_ERROR,
                "force arm is not supported; pre-arm checks cannot be bypassed",
                {"reason": "unsupported_force_arm"},
            )
        async with self._command_lock:
            state = self._adapter.get_state()
            if not state.connected:
                return self._not_connected()
            decision = guards.check_arm(
                state, confirm=_flag(p, "confirm"), config=self._guard_config
            )
            pre = self._pre_execute(decision, dry_run=_flag(p, "dry_run"))
            if pre is not None:
                return pre
            outcome = await self._blocking(self._adapter.arm)
            return self._command_result("arm", outcome)

    async def _m_disarm(self, request: RpcRequest) -> DaemonResponse:
        p = request.params
        async with self._command_lock:
            state = self._adapter.get_state()
            if not state.connected:
                return self._not_connected()
            decision = guards.check_disarm(
                state,
                confirm=_flag(p, "confirm"),
                force=_flag(p, "force"),
                config=self._guard_config,
            )
            pre = self._pre_execute(decision, dry_run=_flag(p, "dry_run"))
            if pre is not None:
                return pre
            outcome = await self._blocking(self._adapter.disarm, _flag(p, "force"))
            return self._command_result("disarm", outcome)

    async def _m_mode(self, request: RpcRequest) -> DaemonResponse:
        p = request.params
        mode = str(p.get("mode", "")).upper()
        try:
            wait_timeout = _wait_timeout(p)
        except ValueError as exc:
            return self._invalid_timeout(exc)
        async with self._command_lock:
            state = self._adapter.get_state()
            if not state.connected:
                return self._not_connected()
            decision = guards.check_mode(
                state,
                mode,
                self._adapter.mode_names(),
                confirm=_flag(p, "confirm"),
                config=self._guard_config,
            )
            pre = self._pre_execute(decision, dry_run=_flag(p, "dry_run"))
            if pre is not None:
                return pre
            outcome = await self._blocking(self._adapter.set_mode, mode)
            if not outcome.accepted:
                return self._command_result("mode", outcome)
            status = await self._maybe_wait(
                p, lambda: self._adapter.get_state().flight_mode == mode, timeout=wait_timeout
            )
            return self._finish_wait(
                "mode", outcome, status, wait_timeout, f"mode did not switch to {mode}"
            )

    async def _m_takeoff(self, request: RpcRequest) -> DaemonResponse:
        p = request.params
        alt_raw = p.get("alt")
        if alt_raw is None:
            return self._invalid_altitude()
        try:
            alt = float(alt_raw)
        except (TypeError, ValueError):
            return self._invalid_altitude()
        # NaN slips every comparison and Infinity exceeds any limit; reject
        # them here so they never reach the guard or the adapter.
        if not math.isfinite(alt):
            return self._invalid_altitude()
        try:
            wait_timeout = _wait_timeout(p)
        except ValueError as exc:
            return self._invalid_timeout(exc)
        async with self._command_lock:
            state = self._adapter.get_state()
            if not state.connected:
                return self._not_connected()
            decision = guards.check_takeoff(
                state, alt, confirm=_flag(p, "confirm"), config=self._guard_config
            )
            pre = self._pre_execute(decision, dry_run=_flag(p, "dry_run"))
            if pre is not None:
                return pre
            outcome = await self._blocking(self._adapter.takeoff, alt)
            if not outcome.accepted:
                return self._command_result("takeoff", outcome)
            target = alt * _TAKEOFF_REACHED_FRACTION
            status = await self._maybe_wait(
                p, lambda: self._reached_altitude(target), timeout=wait_timeout
            )
            return self._finish_wait(
                "takeoff", outcome, status, wait_timeout, f"altitude {target:.1f}m not reached"
            )

    async def _m_land(self, request: RpcRequest) -> DaemonResponse:
        return await self._m_descent(request, "land", self._adapter.land)

    async def _m_rtl(self, request: RpcRequest) -> DaemonResponse:
        return await self._m_descent(request, "rtl", self._adapter.rtl)

    async def _m_descent(
        self, request: RpcRequest, action: str, verb: Callable[[], CommandOutcome]
    ) -> DaemonResponse:
        p = request.params
        try:
            wait_timeout = _wait_timeout(p)
        except ValueError as exc:
            return self._invalid_timeout(exc)
        async with self._command_lock:
            state = self._adapter.get_state()
            if not state.connected:
                return self._not_connected()
            decision = (
                guards.check_land(state, confirm=_flag(p, "confirm"), config=self._guard_config)
                if action == "land"
                else guards.check_rtl(state, confirm=_flag(p, "confirm"), config=self._guard_config)
            )
            pre = self._pre_execute(decision, dry_run=_flag(p, "dry_run"))
            if pre is not None:
                return pre
            outcome = await self._blocking(verb)
            if not outcome.accepted:
                return self._command_result(action, outcome)
            status = await self._maybe_wait(
                p, lambda: self._adapter.get_state().armed is False, timeout=wait_timeout
            )
            return self._finish_wait(
                action, outcome, status, wait_timeout, "vehicle did not disarm"
            )

    # -- helpers -----------------------------------------------------------

    def _not_connected(self) -> DaemonResponse:
        return DaemonResponse.failure(
            ExitCode.VEHICLE_NOT_CONNECTED,
            "vehicle not connected (no recent heartbeat)",
            {"connection_string": self._connection_string},
        )

    def _invalid_altitude(self) -> DaemonResponse:
        return DaemonResponse.failure(
            ExitCode.USAGE_ERROR,
            "takeoff requires a finite numeric --alt",
            {"reason": "invalid_altitude"},
        )

    def _invalid_timeout(self, exc: ValueError) -> DaemonResponse:
        return DaemonResponse.failure(
            ExitCode.USAGE_ERROR,
            f"invalid --timeout: {exc}",
            {"reason": "invalid_timeout"},
        )

    def _pre_execute(self, decision: GuardDecision, *, dry_run: bool) -> DaemonResponse | None:
        """Return a terminal response for dry-run/reject/idempotent, else None."""

        checks = [c.model_dump() for c in decision.checks]
        if dry_run:
            if not decision.allowed:
                return DaemonResponse.failure(
                    decision.exit_code,
                    decision.message or "would be rejected",
                    {
                        "dry_run": True,
                        "reason": decision.reason,
                        "hint": decision.hint,
                        "checks": checks,
                    },
                )
            return DaemonResponse.success(
                {
                    "dry_run": True,
                    "action": decision.action,
                    "would_execute": not decision.already_satisfied,
                    "already_satisfied": decision.already_satisfied,
                    "note": decision.note,
                    "checks": checks,
                }
            )
        if not decision.allowed:
            return DaemonResponse.failure(
                decision.exit_code,
                decision.message or "rejected by safety guard",
                {"reason": decision.reason, "hint": decision.hint, "checks": checks},
            )
        if decision.already_satisfied:
            return DaemonResponse.success(
                {
                    "action": decision.action,
                    "already_satisfied": True,
                    "note": decision.note,
                    "executed": False,
                }
            )
        return None

    async def _blocking(self, fn: Callable[..., CommandOutcome], *args: Any) -> CommandOutcome:
        loop = asyncio.get_running_loop()
        try:
            return await loop.run_in_executor(None, fn, *args)
        except AdapterError as exc:
            return CommandOutcome(accepted=False, result_name=f"ADAPTER_ERROR: {exc}")

    def _command_result(
        self,
        action: str,
        outcome: CommandOutcome,
        *,
        note: str | None = None,
        waited: bool | None = None,
    ) -> DaemonResponse:
        if not outcome.accepted:
            return DaemonResponse.failure(
                ExitCode.NACK_TIMEOUT,
                f"{action} not accepted by vehicle: {outcome.result_name}",
                {"outcome": outcome.model_dump()},
            )
        result: dict[str, Any] = {
            "action": action,
            "executed": True,
            "outcome": outcome.model_dump(),
        }
        if note is not None:
            result["note"] = note
        if waited is not None:
            result["waited"] = waited
        return DaemonResponse.success(result)

    async def _maybe_wait(
        self, params: dict[str, Any], predicate: Callable[[], bool], *, timeout: float
    ) -> WaitStatus:
        """Poll for the target state while --wait is set.

        Returns an explicit :class:`WaitStatus`. Each iteration re-checks the
        link: if the heartbeat goes stale mid-wait the command has already been
        ACKed, so we stop and report LINK_LOST (exit 4) rather than a timeout.
        """

        if not _flag(params, "wait"):
            return WaitStatus.NOT_WAITED
        loop = asyncio.get_running_loop()
        deadline = loop.time() + timeout
        while loop.time() < deadline:
            if not self._adapter.get_state().connected:
                return WaitStatus.LINK_LOST
            if predicate():
                return WaitStatus.REACHED
            await asyncio.sleep(_WAIT_POLL_INTERVAL)
        if not self._adapter.get_state().connected:
            return WaitStatus.LINK_LOST
        return WaitStatus.REACHED if predicate() else WaitStatus.TIMEOUT

    def _finish_wait(
        self,
        action: str,
        outcome: CommandOutcome,
        status: WaitStatus,
        timeout: float,
        timeout_detail: str,
    ) -> DaemonResponse:
        """Map a --wait outcome to the response / exit-code contract."""

        if status is WaitStatus.LINK_LOST:
            return DaemonResponse.failure(
                ExitCode.VEHICLE_NOT_CONNECTED,
                f"{action} was accepted by the vehicle but the link was lost during --wait",
                {
                    "outcome": outcome.model_dump(),
                    "waited": False,
                    "reason": "link_lost_during_wait",
                },
            )
        if status is WaitStatus.TIMEOUT:
            return DaemonResponse.failure(
                ExitCode.NACK_TIMEOUT,
                f"{action} accepted but did not complete within "
                f"{timeout:.0f}s: {timeout_detail}",
                {"outcome": outcome.model_dump(), "waited": False},
            )
        waited = True if status is WaitStatus.REACHED else None
        return self._command_result(action, outcome, waited=waited)

    def _reached_altitude(self, target: float) -> bool:
        rel = self._adapter.get_telemetry().position.relative_alt_m
        return rel is not None and rel >= target

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


def _flag(params: dict[str, Any], key: str) -> bool:
    return bool(params.get(key, False))


def _wait_timeout(params: dict[str, Any]) -> float:
    """Parse and validate the --wait timeout; the daemon is the final boundary.

    Returns the default when the key is absent. Raises ``ValueError`` for
    non-numeric values, NaN / Infinity, and zero or negative values — callers
    must surface that as a usage error (exit 2), never fall back silently.
    """

    value = params.get("timeout")
    if value is None:
        return _DEFAULT_WAIT_TIMEOUT
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError("must be a number of seconds")
    timeout = float(value)
    if not math.isfinite(timeout) or timeout <= 0:
        raise ValueError("must be finite and > 0")
    return timeout
