"""Newline-delimited JSON framing shared by the daemon server and client.

Each request/response is a single UTF-8 JSON object terminated by ``\\n``.
"""

from __future__ import annotations

import json
from typing import Any

_ENCODING = "utf-8"


def encode(payload: dict[str, Any]) -> bytes:
    """Serialise a payload to a single newline-terminated JSON frame."""

    return (json.dumps(payload, separators=(",", ":")) + "\n").encode(_ENCODING)


def decode(line: bytes) -> dict[str, Any]:
    """Parse a single JSON frame into a dict.

    Raises:
        ValueError: if the frame is not a JSON object.
    """

    obj = json.loads(line.decode(_ENCODING))
    if not isinstance(obj, dict):
        raise ValueError("expected a JSON object frame")
    return obj
