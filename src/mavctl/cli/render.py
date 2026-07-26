"""Output formatting and exit-code helpers for the CLI.

Human-readable text and structured ``--json`` share these helpers so the
exit-code contract (AGENTS.md 铁律) is enforced in exactly one place.
"""

from __future__ import annotations

import json
import sys
from typing import Any, NoReturn

import typer

from mavctl.models import ExitCode


def emit_success(
    data: dict[str, Any],
    *,
    json_mode: bool,
    human: str,
) -> None:
    """Print a successful result to stdout in the requested format."""

    if json_mode:
        typer.echo(json.dumps(data, indent=2, sort_keys=True))
    else:
        typer.echo(human)


def fail(
    code: ExitCode,
    message: str,
    *,
    json_mode: bool,
    detail: dict[str, Any] | None = None,
) -> NoReturn:
    """Print a structured error to stderr and exit with ``code``.

    In JSON mode the payload is ``{"error": {...}}``; otherwise a plain
    ``error: <message>`` line. Either way the process exits with ``code``.
    """

    if json_mode:
        payload = {
            "error": {
                "code": int(code),
                "message": message,
                "detail": detail or {},
            }
        }
        print(json.dumps(payload, indent=2, sort_keys=True), file=sys.stderr)
    else:
        print(f"error: {message}", file=sys.stderr)
    raise typer.Exit(int(code))
