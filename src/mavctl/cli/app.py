"""Typer application: ``mavctl`` command surface."""

from __future__ import annotations

from typing import Annotated, Any

import typer

from mavctl.cli.render import emit_success, fail
from mavctl.daemon import process
from mavctl.daemon.client import DaemonNotRunningError, call_daemon
from mavctl.models import ExitCode
from mavctl.paths import socket_path

app = typer.Typer(
    name="mavctl",
    help="Headless MAVLink ground control station CLI.",
    no_args_is_help=True,
    add_completion=False,
)
daemon_app = typer.Typer(help="Manage the mavctl daemon process.", no_args_is_help=True)
app.add_typer(daemon_app, name="daemon")

JsonOption = Annotated[bool, typer.Option("--json", help="Emit structured JSON to stdout.")]

_DAEMON_DOWN_HINT = "daemon is not running; start it with 'mavctl daemon start --connect <conn>'"


def _query(method: str, json_mode: bool) -> dict[str, Any]:
    """Call a daemon method, mapping failures to the exit-code contract."""

    try:
        response = call_daemon(method)
    except DaemonNotRunningError:
        fail(ExitCode.DAEMON_NOT_RUNNING, _DAEMON_DOWN_HINT, json_mode=json_mode)
    except (OSError, ValueError) as exc:
        fail(ExitCode.GENERAL_ERROR, f"daemon communication failed: {exc}", json_mode=json_mode)

    if not response.ok:
        err = response.error
        assert err is not None  # ok is False -> error is populated
        fail(err.code, err.message, json_mode=json_mode, detail=err.detail)
    return response.result or {}


# -- daemon subcommands ----------------------------------------------------


@daemon_app.command("start")
def daemon_start(
    connect: Annotated[
        str, typer.Option("--connect", help="mavutil connection string, e.g. udp:127.0.0.1:14550")
    ],
    heartbeat_timeout: Annotated[
        float, typer.Option("--heartbeat-timeout", help="Seconds before link is deemed lost.")
    ] = 3.0,
    json_mode: JsonOption = False,
) -> None:
    """Start the background daemon and connect to the vehicle."""

    if process.is_running():
        pid = process.read_pid()
        emit_success(
            {"status": "already_running", "pid": pid},
            json_mode=json_mode,
            human=f"daemon already running (pid {pid})",
        )
        return

    try:
        pid = process.spawn(connect, heartbeat_timeout)
    except RuntimeError as exc:
        fail(ExitCode.GENERAL_ERROR, str(exc), json_mode=json_mode)

    emit_success(
        {"status": "started", "pid": pid, "connection_string": connect},
        json_mode=json_mode,
        human=f"daemon started (pid {pid}), connecting to {connect}",
    )


@daemon_app.command("stop")
def daemon_stop(json_mode: JsonOption = False) -> None:
    """Stop the running daemon."""

    if not process.is_running():
        fail(ExitCode.DAEMON_NOT_RUNNING, "daemon is not running", json_mode=json_mode)

    stopped = process.stop()
    emit_success(
        {"status": "stopped" if stopped else "not_running"},
        json_mode=json_mode,
        human="daemon stopped" if stopped else "daemon was not running",
    )


@daemon_app.command("status")
def daemon_status(json_mode: JsonOption = False) -> None:
    """Report whether the daemon is running (exit 3 if not)."""

    running = process.is_running()
    pid = process.read_pid() if running else None
    data = {"running": running, "pid": pid, "socket": str(socket_path())}
    if not running:
        fail(
            ExitCode.DAEMON_NOT_RUNNING,
            "daemon is not running",
            json_mode=json_mode,
            detail=data,
        )
    emit_success(data, json_mode=json_mode, human=f"daemon running (pid {pid})")


# -- top-level commands ----------------------------------------------------


@app.command("status")
def status(json_mode: JsonOption = False) -> None:
    """Show vehicle connection status, flight mode, arming, power, GPS."""

    state = _query("status", json_mode)
    emit_success(state, json_mode=json_mode, human=_format_state(state))


@app.command("telemetry")
def telemetry(json_mode: JsonOption = False) -> None:
    """Show a position / attitude / velocity snapshot."""

    snapshot = _query("telemetry", json_mode)
    emit_success(snapshot, json_mode=json_mode, human=_format_telemetry(snapshot))


# -- human formatters ------------------------------------------------------


def _fmt(value: Any, suffix: str = "", nd: int = 2) -> str:
    if value is None:
        return "n/a"
    if isinstance(value, float):
        return f"{value:.{nd}f}{suffix}"
    return f"{value}{suffix}"


def _format_state(s: dict[str, Any]) -> str:
    battery = s.get("battery") or {}
    gps = s.get("gps") or {}
    connected = s.get("connected")
    lines = [
        f"connection : {'CONNECTED' if connected else 'DISCONNECTED'} "
        f"({s.get('connection_string') or 'n/a'})",
        f"heartbeat  : {_fmt(s.get('heartbeat_age_s'), 's')} ago"
        f"  sys={_fmt(s.get('system_id'))} comp={_fmt(s.get('component_id'))}",
        f"mode       : {s.get('flight_mode') or 'n/a'}",
        f"armed      : {'ARMED' if s.get('armed') else 'disarmed' if connected else 'n/a'}",
        f"battery    : {_fmt(battery.get('voltage_v'), ' V')}"
        f"  {_fmt(battery.get('current_a'), ' A')}"
        f"  {_fmt(battery.get('remaining_pct'), ' %', nd=0)}",
        f"gps        : {gps.get('fix_label') or 'n/a'}"
        f"  sats={_fmt(gps.get('satellites_visible'))}",
    ]
    return "\n".join(lines)


def _format_telemetry(t: dict[str, Any]) -> str:
    pos = t.get("position") or {}
    att = t.get("attitude") or {}
    vel = t.get("velocity") or {}
    lines = [
        f"position : lat={_fmt(pos.get('lat_deg'), nd=7)}  lon={_fmt(pos.get('lon_deg'), nd=7)}",
        f"           alt_msl={_fmt(pos.get('alt_msl_m'), ' m')}"
        f"  rel_alt={_fmt(pos.get('relative_alt_m'), ' m')}",
        f"attitude : roll={_fmt(att.get('roll_deg'), ' deg')}"
        f"  pitch={_fmt(att.get('pitch_deg'), ' deg')}"
        f"  yaw={_fmt(att.get('yaw_deg'), ' deg')}",
        f"velocity : gs={_fmt(vel.get('groundspeed_ms'), ' m/s')}"
        f"  hdg={_fmt(vel.get('heading_deg'), ' deg')}"
        f"  vz={_fmt(vel.get('vz_ms'), ' m/s')}",
    ]
    return "\n".join(lines)


def main() -> None:
    """Console-script entrypoint."""

    app()


if __name__ == "__main__":
    main()
