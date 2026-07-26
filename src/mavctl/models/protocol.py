"""Daemon <-> CLI wire protocol and process-wide exit-code semantics."""

from __future__ import annotations

from enum import IntEnum
from typing import Any

from pydantic import BaseModel, Field


class ExitCode(IntEnum):
    """Process exit codes with fixed semantics (see AGENTS.md 铁律).

    These values are part of the tool's contract and must not change.
    """

    SUCCESS = 0
    GENERAL_ERROR = 1
    USAGE_ERROR = 2
    DAEMON_NOT_RUNNING = 3
    VEHICLE_NOT_CONNECTED = 4
    SAFETY_REJECTED = 5
    NACK_TIMEOUT = 6


class RpcRequest(BaseModel):
    """A JSON-RPC-flavoured request sent from CLI to daemon over the socket."""

    method: str
    params: dict[str, Any] = Field(default_factory=dict)


class RpcError(BaseModel):
    """Structured error returned inside a :class:`DaemonResponse`."""

    code: ExitCode
    message: str
    detail: dict[str, Any] = Field(default_factory=dict)


class DaemonResponse(BaseModel):
    """The daemon's reply to an :class:`RpcRequest`.

    Exactly one of ``result`` or ``error`` is populated, keyed by ``ok``.
    """

    ok: bool
    result: dict[str, Any] | None = None
    error: RpcError | None = None

    @classmethod
    def success(cls, result: dict[str, Any] | None = None) -> DaemonResponse:
        return cls(ok=True, result=result or {})

    @classmethod
    def failure(
        cls,
        code: ExitCode,
        message: str,
        detail: dict[str, Any] | None = None,
    ) -> DaemonResponse:
        return cls(ok=False, error=RpcError(code=code, message=message, detail=detail or {}))
