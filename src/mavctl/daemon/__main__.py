"""Runtime entrypoint for the daemon process (``python -m mavctl.daemon``).

Launched detached by :func:`mavctl.daemon.process.spawn`. Writes its own
pidfile, runs the asyncio server, and cleans up on exit.
"""

from __future__ import annotations

import argparse
import asyncio
import math
import os

from mavctl.adapter import create_adapter
from mavctl.daemon import process
from mavctl.daemon.guards import GuardConfig
from mavctl.daemon.server import DaemonServer


def _parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(prog="mavctl.daemon", add_help=True)
    parser.add_argument("--connect", required=True, help="mavutil connection string")
    parser.add_argument("--heartbeat-timeout", type=float, default=3.0)
    args = parser.parse_args(argv)
    if not (math.isfinite(args.heartbeat_timeout) and args.heartbeat_timeout > 0):
        parser.error(
            f"--heartbeat-timeout must be finite and > 0 (got {args.heartbeat_timeout!r})"
        )
    return args


def main(argv: list[str] | None = None) -> int:
    args = _parse_args(argv)
    process.write_pid(os.getpid())
    adapter = create_adapter(
        connection_string=args.connect,
        heartbeat_timeout_s=args.heartbeat_timeout,
    )
    # Single source of truth: the same --heartbeat-timeout value drives the
    # adapter's ``connected`` computation AND the guards' heartbeat-freshness
    # gate, so they can never drift apart.
    server = DaemonServer(
        adapter,
        args.connect,
        guard_config=GuardConfig(max_heartbeat_age_s=args.heartbeat_timeout),
    )
    try:
        asyncio.run(server.serve())
    finally:
        process.clear_pid()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
