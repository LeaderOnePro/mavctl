"""Asyncio Unix-socket server that owns the vehicle link and answers RPCs."""

from __future__ import annotations

import asyncio
import contextlib
import math
import signal
import time
from collections.abc import Awaitable, Callable
from typing import Any, ParamSpec, TypeVar

from pydantic import ValidationError

from mavctl.adapter.base import (
    AdapterError,
    MissionCountUnsupportedError,
    MissionItemUnsupportedError,
    MissionProtocolError,
    MissionStateUncertainError,
    ModeMappingUnavailableError,
    VehicleAdapter,
)
from mavctl.daemon import guards, wire
from mavctl.daemon.guards import GuardConfig, GuardDecision
from mavctl.daemon.operations import OperationRegistry
from mavctl.models import (
    CommandOutcome,
    DaemonResponse,
    ExitCode,
    MissionV1,
    OperationKind,
    OperationState,
    RpcRequest,
    WaitStatus,
)
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

_P = ParamSpec("_P")
_T = TypeVar("_T")


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
        # Phase 3B-0: long-running operation foundation (Issue #21 design
        # §C.2.1) — one active passive-observation operation per vehicle,
        # epoch-fenced; in-memory only (lost on restart → uncertain).
        self.operations = OperationRegistry()
        self._observation_tasks: set[asyncio.Task[None]] = set()
        self._methods: dict[str, Handler] = {
            "ping": self._m_ping,
            "status": self._m_status,
            "telemetry": self._m_telemetry,
            "shutdown": self._m_shutdown,
            "operation_get": self._m_operation_get,
            "mission_start": self._m_mission_start,
            "mission_upload": self._m_mission_upload,
            "mission_download": self._m_mission_download,
            "mission_clear": self._m_mission_clear,
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

    # -- mission RPC methods (Phase 3A) ------------------------------------

    def _parse_mission(self, raw: Any) -> MissionV1 | None:
        """Authoritative daemon-side schema validation."""

        try:
            return MissionV1.model_validate(raw)
        except ValidationError:
            return None

    def _mission_uncertain(self, exc: MissionStateUncertainError, action: str) -> DaemonResponse:
        detail: dict[str, Any] = {
            "reason": "remote_mission_state_uncertain",
            "hint": "verify the remote mission with 'mavctl mission download'",
        }
        if exc.sent_upto is not None:
            detail["sent_upto"] = exc.sent_upto
        if exc.observed_count is not None:
            detail["observed_count"] = exc.observed_count
        expected_seq = getattr(exc, "expected_seq", None)
        requested_seq = getattr(exc, "requested_seq", None)
        if expected_seq is not None:
            detail["expected_seq"] = expected_seq
        if requested_seq is not None:
            detail["requested_seq"] = requested_seq
        if action == "mission_clear" and exc.observed_count is not None:
            message = (
                f"mission_clear uncertain; "
                f"remote mission count observed: {exc.observed_count}"
            )
        elif action == "mission_clear":
            message = (
                "mission_clear outcome uncertain; "
                "remote mission count could not be observed"
            )
        else:
            message = f"{action} outcome uncertain; remote mission state must be verified"
        return DaemonResponse.failure(ExitCode.NACK_TIMEOUT, message, detail)

    def _mission_rejected(self, exc: MissionProtocolError) -> DaemonResponse:
        return DaemonResponse.failure(
            ExitCode.NACK_TIMEOUT,
            f"mission operation failed: {exc.result_name}",
            {"reason": "mission_rejected", "result_name": exc.result_name},
        )

    async def _m_mission_upload(self, request: RpcRequest) -> DaemonResponse:
        p = request.params
        mission = self._parse_mission(p.get("mission"))
        if mission is None:
            return DaemonResponse.failure(
                ExitCode.USAGE_ERROR,
                "invalid mission: the payload does not match the v1 mission schema",
                {
                    "reason": "invalid_mission",
                    "hint": "validate the mission JSON against the mavctl v1 schema",
                },
            )
        async with self._command_lock:
            state = self._adapter.get_state()
            if not state.connected:
                return self._not_connected()
            decision = guards.check_mission_upload(
                state, mission, confirm=_flag(p, "confirm"), config=self._guard_config
            )
            pre = self._pre_execute(decision, dry_run=_flag(p, "dry_run"))
            if pre is not None:
                return pre
            try:
                outcome = await self._blocking_mission(self._adapter.upload_mission, mission)
            except MissionStateUncertainError as exc:
                return self._mission_uncertain(exc, "mission_upload")
            except MissionProtocolError as exc:
                return self._mission_rejected(exc)
        return DaemonResponse.success(outcome.model_dump())

    async def _m_mission_download(self, request: RpcRequest) -> DaemonResponse:
        # Read-only: daemon _command_lock deliberately not taken, so a
        # download may run beside a long --wait (design §H).
        state = self._adapter.get_state()
        if not state.connected:
            return self._not_connected()
        try:
            mission = await self._blocking_mission(self._adapter.download_mission)
        except MissionItemUnsupportedError as exc:
            return DaemonResponse.failure(
                ExitCode.NACK_TIMEOUT,
                "remote mission contains items outside the mavctl v1 schema",
                {
                    "reason": "mission_item_unsupported",
                    "seq": exc.seq,
                    "command": exc.command,
                    "frame": exc.frame,
                    "hint": "inspect the mission with a full GCS; mavctl v1 cannot represent it",
                },
            )
        except MissionCountUnsupportedError as exc:
            # observed_count is the wire count, which on ArduPilot includes the
            # vehicle-managed home slot (seq 0); v1 items = wire count - 1.
            v1_items = max(exc.observed_count - 1, 0)
            return DaemonResponse.failure(
                ExitCode.NACK_TIMEOUT,
                f"remote mission has {v1_items} items; mavctl v1 "
                f"supports at most {exc.max_supported_items}",
                {
                    "reason": "mission_item_unsupported",
                    "observed_count": v1_items,
                    "max_supported_items": exc.max_supported_items,
                },
            )
        except MissionStateUncertainError as exc:
            return self._mission_uncertain(exc, "mission_download")
        except MissionProtocolError as exc:
            return DaemonResponse.failure(
                ExitCode.NACK_TIMEOUT,
                f"mission download failed: {exc.result_name}",
                {"reason": "mission_protocol_timeout", "result_name": exc.result_name},
            )
        return DaemonResponse.success(
            {"action": "mission_download", "mission": mission.model_dump()}
        )

    async def _m_mission_clear(self, request: RpcRequest) -> DaemonResponse:
        p = request.params
        async with self._command_lock:
            state = self._adapter.get_state()
            if not state.connected:
                return self._not_connected()
            decision = guards.check_mission_clear(
                state, confirm=_flag(p, "confirm"), config=self._guard_config
            )
            pre = self._pre_execute(decision, dry_run=_flag(p, "dry_run"))
            if pre is not None:
                return pre
            try:
                outcome = await self._blocking_mission(self._adapter.clear_mission)
            except MissionStateUncertainError as exc:
                return self._mission_uncertain(exc, "mission_clear")
            except MissionProtocolError as exc:
                return self._mission_rejected(exc)
        return DaemonResponse.success(outcome.model_dump())

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
            try:
                outcome = await self._blocking(self._adapter.set_mode, mode)
            except ModeMappingUnavailableError:
                # TOCTOU: the mapping was valid when the guard checked
                # mode_names(), but vanished or changed before the adapter
                # could resolve the target. Structured and retryable —
                # never an internal error.
                return DaemonResponse.failure(
                    ExitCode.SAFETY_REJECTED,
                    "vehicle mode mapping is not available; cannot switch mode",
                    {
                        "reason": "mode_map_unavailable",
                        "hint": (
                            "wait for the vehicle's mode map to populate "
                            "(check: mavctl status), then retry this command"
                        ),
                    },
                )
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
            # Command-lock checkpoint (Issue #21 design §C.2.1-C): the
            # accepted ACK atomically registers the operation as the active
            # observation owner. This is the LAST thing under the lock —
            # passive observation runs outside it so RTL/land stay available.
            operation = self.operations.activate(
                OperationKind.TAKEOFF,
                effect_sent_monotonic=time.monotonic(),
                ack_monotonic=time.monotonic(),
            )

        target = alt * _TAKEOFF_REACHED_FRACTION
        if _flag(p, "wait"):
            await self._observe_operation(
                operation, lambda: self._reached_altitude(target), timeout=wait_timeout
            )
            # Client deadline hit (or milestone reached): keep observing in
            # the background so `operation get` stays truthful — the client
            # timeout did not cancel the accepted vehicle action.
            self._spawn_operation_watch(operation, lambda: self._reached_altitude(target))
            return self._finish_operation_wait(
                "takeoff", outcome, operation, wait_timeout,
                f"altitude {target:.1f}m not reached",
            )
        self._spawn_operation_watch(operation, lambda: self._reached_altitude(target))
        return self._command_result(
            "takeoff", outcome, operation_id=operation.operation_id
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
            # Command-lock checkpoint (Issue #21 design §C.2.1-C): mirror of
            # the takeoff migration — the accepted ACK activates the
            # operation as the LAST act under the lock; disarm observation
            # runs outside it so other commands stay available.
            operation = self.operations.activate(
                OperationKind(action),
                effect_sent_monotonic=time.monotonic(),
                ack_monotonic=time.monotonic(),
            )

        if _flag(p, "wait"):
            await self._observe_operation(
                operation,
                lambda: self._adapter.get_state().armed is False,
                timeout=wait_timeout,
            )
            self._spawn_operation_watch(
                operation, lambda: self._adapter.get_state().armed is False
            )
            return self._finish_operation_wait(
                action, outcome, operation, wait_timeout, "vehicle did not disarm"
            )
        self._spawn_operation_watch(
            operation, lambda: self._adapter.get_state().armed is False
        )
        return self._command_result(
            action, outcome, operation_id=operation.operation_id
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
        except (MissionProtocolError, ModeMappingUnavailableError):
            # Typed vehicle/protocol states that mission and mode commands map
            # to structured responses — never ADAPTER_ERROR outcomes and
            # never internal errors.
            raise
        except AdapterError as exc:
            return CommandOutcome(accepted=False, result_name=f"ADAPTER_ERROR: {exc}")

    async def _blocking_mission(
        self, fn: Callable[_P, _T], *args: _P.args, **kwargs: _P.kwargs
    ) -> _T:
        """Run a mission transaction in the executor; typed mission errors
        propagate to the RPC handler for structured mapping."""

        loop = asyncio.get_running_loop()
        return await loop.run_in_executor(None, fn, *args)

    def _command_result(
        self,
        action: str,
        outcome: CommandOutcome,
        *,
        note: str | None = None,
        waited: bool | None = None,
        operation_id: str | None = None,
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
        if operation_id is not None:
            result["operation_id"] = operation_id
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

    async def _observe_operation(
        self,
        operation: Any,
        predicate: Callable[[], bool],
        *,
        timeout: float,
    ) -> None:
        """Passive milestone observation OUTSIDE `_command_lock`.

        Updates the registry state (fenced by operation_id + epoch per
        Issue #21 design §C.2.1-E): link loss → LINK_LOST, predicate →
        REACHED, deadline → TIMED_OUT with the operation left WAITING
        (client timeout ≠ vehicle command cancelled; observation continues
        in the background). If the operation was superseded mid-wait the
        fenced writes are no-ops and the loop simply ends.
        """

        loop = asyncio.get_running_loop()
        deadline = loop.time() + timeout
        try:
            while loop.time() < deadline:
                if operation.state is not OperationState.WAITING:
                    # superseded by another accepted command (§C.2.1-D): stop
                    # observing immediately; the fenced writes are no-ops
                    # anyway.
                    return
                if not self._adapter.get_state().connected:
                    self.operations.record_link_lost(
                        operation.operation_id, operation.epoch
                    )
                    return
                if predicate():
                    self.operations.record_reached(
                        operation.operation_id, operation.epoch
                    )
                    return
                await asyncio.sleep(_WAIT_POLL_INTERVAL)
            if operation.state is not OperationState.WAITING:
                return
            if self._adapter.get_state().connected and predicate():
                self.operations.record_reached(operation.operation_id, operation.epoch)
                return
            if not self._adapter.get_state().connected:
                self.operations.record_link_lost(operation.operation_id, operation.epoch)
                return
            self.operations.record_timed_out(operation.operation_id, operation.epoch)
        except Exception:
            # Known observation failure (e.g. adapter snapshot exploded): the
            # operation must not stay WAITING and the failure must not leak a
            # traceback to the client — it becomes a controlled UNCERTAIN
            # terminal state. Cancellation (daemon shutdown) is a
            # BaseException and passes through untouched.
            self.operations.record_uncertain(
                operation.operation_id,
                operation.epoch,
                reason="operation_observation_failed",
            )

    def _spawn_operation_watch(
        self,
        operation: Any,
        predicate: Callable[[], bool],
    ) -> None:
        """Background continuation after a client timeout / no-wait return:
        observe until a terminal state so `operation get` stays truthful."""

        async def _watch() -> None:
            try:
                while operation.state is OperationState.WAITING:
                    if not self._adapter.get_state().connected:
                        self.operations.record_link_lost(
                            operation.operation_id, operation.epoch
                        )
                        return
                    if predicate():
                        self.operations.record_reached(
                            operation.operation_id, operation.epoch
                        )
                        return
                    await asyncio.sleep(_WAIT_POLL_INTERVAL)
            except Exception:
                # Known observation failure: contain it in the operation (see
                # _observe_operation). Cancellation (daemon shutdown) passes
                # through as a BaseException.
                self.operations.record_uncertain(
                    operation.operation_id,
                    operation.epoch,
                    reason="operation_observation_failed",
                )

        task = asyncio.create_task(_watch())
        self._observation_tasks.add(task)
        task.add_done_callback(self._observation_tasks.discard)

    def _finish_operation_wait(
        self,
        action: str,
        outcome: CommandOutcome,
        operation: Any,
        timeout: float,
        timeout_detail: str,
    ) -> DaemonResponse:
        """Map a fenced operation outcome to the response / exit-code contract.

        Honest semantics (Issue #21 design §E/§G): a superseded operation
        never claims the earlier vehicle command was cancelled; a client
        timeout never claims the vehicle action was cancelled either — the
        operation keeps running and is queryable via `operation get`.
        """

        if operation.state is OperationState.REACHED:
            return self._command_result(
                action, outcome, waited=True, operation_id=operation.operation_id
            )
        if operation.state is OperationState.SUPERSEDED:
            return DaemonResponse.failure(
                ExitCode.NACK_TIMEOUT,
                f"{action} was accepted by the vehicle but superseded by "
                f"operation {operation.superseded_by_operation_id}: "
                "vehicle state must be re-queried",
                {
                    "reason": "operation_superseded",
                    "operation_id": operation.operation_id,
                    "superseded_by_operation_id":
                        operation.superseded_by_operation_id,
                    "hint": "re-check the vehicle with 'mavctl status'; "
                    "the superseded command was not cancelled",
                    "outcome": outcome.model_dump(),
                },
            )
        if operation.state is OperationState.LINK_LOST:
            return DaemonResponse.failure(
                ExitCode.VEHICLE_NOT_CONNECTED,
                f"{action} was accepted by the vehicle but the link was lost "
                "during --wait",
                {
                    "reason": "link_lost_during_wait",
                    "operation_id": operation.operation_id,
                    "outcome": outcome.model_dump(),
                },
            )
        if operation.state is OperationState.UNCERTAIN:
            reason = (
                "operation_observation_failed"
                if operation.terminal_reason == "operation_observation_failed"
                else "operation_uncertain"
            )
            return DaemonResponse.failure(
                ExitCode.NACK_TIMEOUT,
                f"{action} accepted but the vehicle state could not be "
                "established",
                {
                    "reason": reason,
                    "operation_id": operation.operation_id,
                    "hint": "re-check the vehicle with 'mavctl status --json'",
                    "outcome": outcome.model_dump(),
                },
            )
        # TIMED_OUT / still WAITING after the client's deadline: the daemon
        # operation continues observing in the background.
        return DaemonResponse.failure(
            ExitCode.NACK_TIMEOUT,
            f"{action} accepted but did not complete within {timeout:.0f}s: "
            f"{timeout_detail}",
            {
                "reason": "operation_wait_timeout",
                "operation_still_running": True,
                "operation_id": operation.operation_id,
                "hint": "query 'mavctl operation get <id>'; the accepted "
                "vehicle action is not cancelled by this timeout",
                "outcome": outcome.model_dump(),
            },
        )

    async def _m_mission_start(self, request: RpcRequest) -> DaemonResponse:
        """Start or resume the stored mission (Phase 3B-1).

        Flow (Issue #21 design §C.2.1-C checkpoint): guard + count probe +
        effecting command + operation activation all under `_command_lock`;
        passive milestone observation (mission ACTIVE + mode AUTO) runs
        outside the lock, fenced by operation_id + epoch.
        """

        p = request.params
        try:
            wait_timeout = _wait_timeout(p)
        except ValueError as exc:
            return self._invalid_timeout(exc)
        async with self._command_lock:
            state = self._adapter.get_state()
            if not state.connected:
                return self._not_connected()

            # Vehicle-verified mission count: this is the guard's
            # "mission exists" evidence (a stale cache or inference from
            # AUTO/position would be fabricable).
            try:
                mission_count = await self._blocking_mission(
                    self._adapter.get_mission_count
                )
            except MissionProtocolError as exc:
                return DaemonResponse.failure(
                    ExitCode.SAFETY_REJECTED,
                    f"refusing to mission_start: the mission count could "
                    f"not be verified ({exc.result_name})",
                    {
                        "reason": "mission_count_unverified",
                        "result_name": exc.result_name,
                        "hint": "verify the stored mission with "
                        "'mavctl mission download', then retry",
                    },
                )

            decision = guards.check_mission_start(
                state,
                mission_count=mission_count,
                confirm=_flag(p, "confirm"),
                config=self._guard_config,
            )
            pre = self._pre_execute(decision, dry_run=_flag(p, "dry_run"))
            if pre is not None:
                return pre
            if decision.already_satisfied:
                return DaemonResponse.success(
                    {
                        "action": "mission_start",
                        "executed": False,
                        "already_running": True,
                    }
                )

            outcome = await self._blocking(self._adapter.start_mission)
            if not outcome.accepted:
                return self._command_result("mission_start", outcome)
            # Command-lock checkpoint (§C.2.1-C): the accepted ACK activates
            # the operation as the LAST act under the lock.
            operation = self.operations.activate(
                OperationKind.MISSION_START,
                effect_sent_monotonic=time.monotonic(),
                ack_monotonic=time.monotonic(),
            )

        def _milestone() -> bool:
            snapshot = self._adapter.get_state()
            execution = snapshot.mission
            mission_active = (
                execution is not None and execution.state == "active"
            )
            mode_auto = snapshot.flight_mode == "AUTO"
            return mission_active and mode_auto

        if _flag(p, "wait"):
            await self._observe_operation(
                operation, _milestone, timeout=wait_timeout
            )
            # Client deadline hit (or milestone reached): keep observing in
            # the background so `operation get` stays truthful.
            self._spawn_operation_watch(operation, _milestone)
            return self._finish_operation_wait(
                "mission_start", outcome, operation, wait_timeout,
                "mission execution (AUTO + mission_state ACTIVE) not observed",
            )
        self._spawn_operation_watch(operation, _milestone)
        return self._command_result(
            "mission_start", outcome, operation_id=operation.operation_id
        )

    async def _m_operation_get(self, request: RpcRequest) -> DaemonResponse:
        """Read-only operation observation (never takes the command lock)."""

        operation_id = request.params.get("operation_id")
        snapshot = (
            self.operations.snapshot(str(operation_id))
            if operation_id
            else None
        )
        if snapshot is None:
            return DaemonResponse.failure(
                ExitCode.USAGE_ERROR,
                f"operation not found: {operation_id}",
                {
                    "reason": "operation_not_found",
                    "operation_id": str(operation_id) if operation_id else None,
                    "hint": "the daemon may have restarted; re-query "
                    "'mavctl status' — this does not mean the vehicle "
                    "action did not happen",
                },
            )
        return DaemonResponse.success({"operation": snapshot.model_dump()})

    # -- teardown ----------------------------------------------------------

    def _install_signal_handlers(self) -> None:
        loop = asyncio.get_running_loop()
        for sig in (signal.SIGTERM, signal.SIGINT):
            with contextlib.suppress(NotImplementedError):
                loop.add_signal_handler(sig, self.request_stop)

    async def _shutdown(self) -> None:
        # 1. stop accepting new connections
        if self._server is not None:
            self._server.close()
            with contextlib.suppress(Exception):
                await self._server.wait_closed()
            self._server = None
        # 2. cancel + await local background observers (daemon-local shutdown
        #    only: this never cancels or retracts a vehicle action, and sends
        #    nothing to the vehicle). Cancellation is a BaseException, so the
        #    watchers' exception containment does not misreport it.
        watchers = [t for t in self._observation_tasks if not t.done()]
        for task in watchers:
            task.cancel()
        if watchers:
            await asyncio.gather(*watchers, return_exceptions=True)
        self._observation_tasks.clear()
        # 3. release the active owner (registry is in-memory; after shutdown
        #    a new daemon answers operation get with operation_not_found)
        self.operations.release_active()
        # 4. disconnect the link and clean the socket last
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
