"""Filesystem locations for daemon runtime state.

Shared by both the CLI and daemon layers; contains no business logic.
"""

from __future__ import annotations

import os
from pathlib import Path


def runtime_dir() -> Path:
    """Return the mavctl runtime directory (``~/.mavctl`` by default).

    Overridable via the ``MAVCTL_HOME`` environment variable, which keeps
    tests hermetic and lets multiple daemons coexist.
    """

    override = os.environ.get("MAVCTL_HOME")
    base = Path(override).expanduser() if override else Path.home() / ".mavctl"
    return base


def socket_path() -> Path:
    """Unix domain socket the daemon listens on."""

    return runtime_dir() / "daemon.sock"


def pid_path() -> Path:
    """Pidfile tracking the running daemon process."""

    return runtime_dir() / "daemon.pid"


def log_path() -> Path:
    """Daemon log file."""

    return runtime_dir() / "daemon.log"
