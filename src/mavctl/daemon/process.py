"""Daemon process lifecycle: pidfile management, spawning, and stopping."""

from __future__ import annotations

import os
import signal
import subprocess
import sys
import time
from contextlib import suppress

from mavctl.daemon.client import is_daemon_running
from mavctl.paths import log_path, pid_path, runtime_dir, socket_path


def read_pid() -> int | None:
    """Return the pid recorded in the pidfile, or None if absent/unreadable."""

    path = pid_path()
    try:
        text = path.read_text().strip()
    except FileNotFoundError:
        return None
    try:
        return int(text)
    except ValueError:
        return None


def write_pid(pid: int) -> None:
    """Record ``pid`` in the pidfile, creating the runtime dir if needed."""

    runtime_dir().mkdir(parents=True, exist_ok=True)
    pid_path().write_text(f"{pid}\n")


def clear_pid() -> None:
    """Remove the pidfile if present."""

    with suppress(FileNotFoundError):
        pid_path().unlink()


def pid_alive(pid: int) -> bool:
    """Return True if a process with ``pid`` exists and is signalable."""

    try:
        os.kill(pid, 0)
    except ProcessLookupError:
        return False
    except PermissionError:
        return True
    return True


def is_running() -> bool:
    """Return True if a live daemon is recorded and reachable.

    Cleans up a stale pidfile whose process is gone.
    """

    pid = read_pid()
    if pid is None:
        return False
    if not pid_alive(pid):
        clear_pid()
        return False
    return True


def spawn(connection_string: str, heartbeat_timeout: float, startup_timeout: float = 12.0) -> int:
    """Launch a detached daemon process and wait until it answers a ping.

    Returns:
        The pid of the started daemon.

    Raises:
        RuntimeError: if the daemon exits early or never becomes ready.
    """

    runtime_dir().mkdir(parents=True, exist_ok=True)
    log = log_path().open("a")
    try:
        proc = subprocess.Popen(
            [
                sys.executable,
                "-m",
                "mavctl.daemon",
                "--connect",
                connection_string,
                "--heartbeat-timeout",
                str(heartbeat_timeout),
            ],
            stdin=subprocess.DEVNULL,
            stdout=log,
            stderr=log,
            start_new_session=True,
            env=os.environ.copy(),
        )
    finally:
        log.close()

    deadline = time.monotonic() + startup_timeout
    while time.monotonic() < deadline:
        exit_code = proc.poll()
        if exit_code is not None:
            raise RuntimeError(
                f"daemon exited during startup (code {exit_code}); see {log_path()}"
            )
        if is_daemon_running():
            return proc.pid
        time.sleep(0.15)

    proc.terminate()
    raise RuntimeError(f"daemon did not become ready within {startup_timeout:.0f}s")


def stop(term_timeout: float = 8.0) -> bool:
    """Stop the running daemon via SIGTERM, escalating to SIGKILL.

    Returns:
        True if a daemon was stopped, False if none was running.
    """

    pid = read_pid()
    if pid is None or not pid_alive(pid):
        clear_pid()
        _cleanup_socket()
        return False

    try:
        os.kill(pid, signal.SIGTERM)
    except ProcessLookupError:
        clear_pid()
        _cleanup_socket()
        return False

    deadline = time.monotonic() + term_timeout
    while time.monotonic() < deadline:
        if not pid_alive(pid):
            break
        time.sleep(0.1)
    else:
        with suppress(ProcessLookupError):
            os.kill(pid, signal.SIGKILL)

    clear_pid()
    _cleanup_socket()
    return True


def _cleanup_socket() -> None:
    with suppress(FileNotFoundError):
        socket_path().unlink()
