"""mavctl — headless MAVLink ground control station CLI."""

from __future__ import annotations

import re
from importlib.metadata import PackageNotFoundError
from importlib.metadata import version as _installed_version
from pathlib import Path


def _resolve_version() -> str:
    """Package version, without a second hardcoded constant.

    Single source of truth is ``pyproject.toml``: read through
    ``importlib.metadata`` when installed, falling back to parsing the
    ``pyproject.toml`` next to this checkout for a plain source tree.
    """

    try:
        return _installed_version("mavctl")
    except PackageNotFoundError:
        pass
    pyproject = Path(__file__).resolve().parents[2] / "pyproject.toml"
    try:
        match = re.search(
            r'^version = "([^"]+)"$',
            pyproject.read_text(encoding="utf-8"),
            re.MULTILINE,
        )
    except OSError:
        return "unknown"
    return match.group(1) if match else "unknown"


__version__ = _resolve_version()
