"""Test doubles for MAVLink messages and connections (no real transport)."""

from __future__ import annotations

import queue
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
    call does on timeout). ``flightmode`` is a settable attribute.
    """

    def __init__(
        self, messages: list[FakeMsg] | None = None, flightmode: str = "STABILIZE"
    ) -> None:
        self._queue: queue.Queue[FakeMsg] = queue.Queue()
        for msg in messages or []:
            self._queue.put(msg)
        self.flightmode = flightmode
        self.closed = False

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
