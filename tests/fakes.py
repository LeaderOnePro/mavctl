"""Test doubles for MAVLink messages and connections (no real transport)."""

from __future__ import annotations

import queue
from types import SimpleNamespace
from typing import Any


class FakeMsg:
    """A stand-in for a pymavlink message object.

    Exposes fields as attributes plus the ``get_type`` / ``get_srcSystem`` /
    ``get_srcComponent`` accessors the adapter relies on.
    """

    def __init__(
        self,
        msg_type: str,
        src_system: int = 1,
        src_component: int = 1,
        **fields: Any,
    ) -> None:
        self._type = msg_type
        self._src_system = src_system
        self._src_component = src_component
        for key, value in fields.items():
            setattr(self, key, value)

    def get_type(self) -> str:
        return self._type

    def get_srcSystem(self) -> int:
        return self._src_system

    def get_srcComponent(self) -> int:
        return self._src_component


class FakeMaster:
    """A scriptable stand-in for ``mavutil.mavlink_connection``.

    ``recv_match`` drains a preloaded queue then returns ``None`` (as the real
    call does on timeout). ``mav`` exposes the send methods the adapter uses.
    """

    def __init__(
        self,
        messages: list[FakeMsg] | None = None,
        flightmode: str = "STABILIZE",
        modes: dict[str, int] | None = None,
    ) -> None:
        self._queue: queue.Queue[FakeMsg] = queue.Queue()
        for msg in messages or []:
            self._queue.put(msg)
        self.flightmode = flightmode
        self.closed = False
        self._modes = modes or {"STABILIZE": 0, "GUIDED": 4, "LOITER": 5, "RTL": 6, "LAND": 9}
        self.sent: list[tuple[int, int, tuple[float, ...]]] = []
        self.mav = SimpleNamespace(
            command_long_send=self._command_long_send,
            request_data_stream_send=lambda *a, **k: None,
        )

    def _command_long_send(
        self, tsys: int, tcomp: int, command: int, confirmation: int, *params: float
    ) -> None:
        self.sent.append((command, confirmation, params))

    def mode_mapping(self) -> dict[str, int]:
        return dict(self._modes)

    def recv_match(
        self,
        type: list[str] | str | None = None,
        blocking: bool = True,
        timeout: float | None = None,
    ) -> FakeMsg | None:
        try:
            return self._queue.get_nowait()
        except queue.Empty:
            return None

    def close(self) -> None:
        self.closed = True
