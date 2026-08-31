"""Typer application: ``mavctl`` command surface."""

from __future__ import annotations

import math
from typing import Annotated, Any

import typer

from mavctl import __version__
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


def _version_callback(value: bool) -> None:
    if value:
        # Pure client-side: no daemon, no vehicle link required.
        typer.echo(f"mavctl {__version__}")
        raise typer.Exit()


@app.callback()
def _root(
    version: Annotated[
        bool,
        typer.Option(
            "--version",
            help="Show the mavctl version and exit.",
            callback=_version_callback,
            is_eager=True,
        ),
    ] = False,
) -> None:
    """Headless MAVLink ground control station CLI."""

JsonOption = Annotated[bool, typer.Option("--json", help="Emit structured JSON to stdout.")]
ConfirmOption = Annotated[
    bool, typer.Option("--confirm", help="Confirm this state-changing command (required).")
]
DryRunOption = Annotated[
    bool, typer.Option("--dry-run", help="Run safety checks and report without executing.")
]
# Only disarm takes --force (emergency motor stop); arm has no force path.
DisarmForceOption = Annotated[
    bool,
    typer.Option(
        "--force",
        help="Emergency motor stop; using in flight may cause a crash.",
    ),
]
WaitOption = Annotated[
    bool, typer.Option("--wait", help="Block until the target state is reached.")
]
TimeoutOption = Annotated[
    float, typer.Option("--timeout", help="Seconds to wait when --wait is set (default 60).")
]

_DAEMON_DOWN_HINT = "daemon is not running; start it with 'mavctl daemon start --connect <conn>'"

# Client socket timeout for quick queries; commands compute their own.
_QUERY_TIMEOUT = 5.0
# Buffer added on top of an ACK burst (retries) / a --wait window.
_COMMAND_BASE_TIMEOUT = 25.0


def _client_timeout(wait: bool, timeout: float) -> float:
    return timeout + 30.0 if wait else _COMMAND_BASE_TIMEOUT


def _check_timeout(timeout: float, *, json_mode: bool) -> None:
    """Early CLI-side range check; the daemon re-validates as final boundary."""

    if not (math.isfinite(timeout) and timeout > 0):
        fail(ExitCode.USAGE_ERROR, "--timeout must be finite and > 0", json_mode=json_mode)


def _call(
    method: str,
    json_mode: bool,
    params: dict[str, Any] | None = None,
    timeout: float = _QUERY_TIMEOUT,
) -> dict[str, Any]:
    """Call a daemon method, mapping failures to the exit-code contract."""

    try:
        response = call_daemon(method, params, timeout=timeout)
    except DaemonNotRunningError:
        fail(ExitCode.DAEMON_NOT_RUNNING, _DAEMON_DOWN_HINT, json_mode=json_mode)
    except (OSError, ValueError) as exc:
        fail(ExitCode.GENERAL_ERROR, f"daemon communication failed: {exc}", json_mode=json_mode)

    if not response.ok:
        err = response.error
        assert err is not None  # ok is False -> error is populated
        message = err.message
        hint = err.detail.get("hint")
        if hint and not json_mode:
            message = f"{message}\nhint: {hint}"
        fail(err.code, message, json_mode=json_mode, detail=err.detail)
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

    state = _call("status", json_mode)
    emit_success(state, json_mode=json_mode, human=_format_state(state))


@app.command("telemetry")
def telemetry(json_mode: JsonOption = False) -> None:
    """Show a position / attitude / velocity snapshot."""

    snapshot = _call("telemetry", json_mode)
    emit_success(snapshot, json_mode=json_mode, human=_format_telemetry(snapshot))


# -- flight-control commands -----------------------------------------------


@app.command("arm")
def arm(
    confirm: ConfirmOption = False,
    dry_run: DryRunOption = False,
    json_mode: JsonOption = False,
) -> None:
    """Arm the vehicle (requires --confirm; pre-arm checks are never bypassed)."""

    result = _call(
        "arm",
        json_mode,
        {"confirm": confirm, "dry_run": dry_run},
        timeout=_COMMAND_BASE_TIMEOUT,
    )
    emit_success(result, json_mode=json_mode, human=_format_command(result))


@app.command("disarm")
def disarm(
    confirm: ConfirmOption = False,
    force: DisarmForceOption = False,
    dry_run: DryRunOption = False,
    json_mode: JsonOption = False,
) -> None:
    """Disarm the vehicle (requires --confirm; --force for emergency stop)."""

    result = _call(
        "disarm",
        json_mode,
        {"confirm": confirm, "force": force, "dry_run": dry_run},
        timeout=_COMMAND_BASE_TIMEOUT,
    )
    emit_success(result, json_mode=json_mode, human=_format_command(result))


@app.command("mode")
def mode(
    mode_name: Annotated[
        str, typer.Argument(metavar="MODE", help="Target flight mode, e.g. GUIDED")
    ],
    confirm: ConfirmOption = False,
    wait: WaitOption = False,
    timeout: TimeoutOption = 60.0,
    dry_run: DryRunOption = False,
    json_mode: JsonOption = False,
) -> None:
    """Switch flight mode (requires --confirm)."""

    _check_timeout(timeout, json_mode=json_mode)
    result = _call(
        "mode",
        json_mode,
        {
            "mode": mode_name,
            "confirm": confirm,
            "wait": wait,
            "timeout": timeout,
            "dry_run": dry_run,
        },
        timeout=_client_timeout(wait, timeout),
    )
    emit_success(result, json_mode=json_mode, human=_format_command(result))


@app.command("takeoff")
def takeoff(
    alt: Annotated[float, typer.Option("--alt", help="Target relative altitude in metres.")],
    confirm: ConfirmOption = False,
    wait: WaitOption = False,
    timeout: TimeoutOption = 60.0,
    dry_run: DryRunOption = False,
    json_mode: JsonOption = False,
) -> None:
    """Take off to a target altitude (requires --confirm; GUIDED + armed)."""

    _check_timeout(timeout, json_mode=json_mode)
    result = _call(
        "takeoff",
        json_mode,
        {"alt": alt, "confirm": confirm, "wait": wait, "timeout": timeout, "dry_run": dry_run},
        timeout=_client_timeout(wait, timeout),
    )
    emit_success(result, json_mode=json_mode, human=_format_command(result))


@app.command("land")
def land(
    confirm: ConfirmOption = False,
    wait: WaitOption = False,
    timeout: TimeoutOption = 60.0,
    dry_run: DryRunOption = False,
    json_mode: JsonOption = False,
) -> None:
    """Land at the current position (requires --confirm)."""

    _check_timeout(timeout, json_mode=json_mode)
    result = _call(
        "land",
        json_mode,
        {"confirm": confirm, "wait": wait, "timeout": timeout, "dry_run": dry_run},
        timeout=_client_timeout(wait, timeout),
    )
    emit_success(result, json_mode=json_mode, human=_format_command(result))


@app.command("rtl")
def rtl(
    confirm: ConfirmOption = False,
    wait: WaitOption = False,
    timeout: TimeoutOption = 60.0,
    dry_run: DryRunOption = False,
    json_mode: JsonOption = False,
) -> None:
    """Return to launch (requires --confirm)."""

    _check_timeout(timeout, json_mode=json_mode)
    result = _call(
        "rtl",
        json_mode,
        {"confirm": confirm, "wait": wait, "timeout": timeout, "dry_run": dry_run},
        timeout=_client_timeout(wait, timeout),
    )
    emit_success(result, json_mode=json_mode, human=_format_command(result))



# -- human formatters ------------------------------------------------------


def _fmt(value: Any, suffix: str = "", nd: int = 2) -> str:
    if value is None:
        return "n/a"
    if isinstance(value, float):
        return f"{value:.{nd}f}{suffix}"
    return f"{value}{suffix}"


def _fmt_age(seconds: Any) -> str:
    """Render a freshness age as ``0.2s`` / ``n/a`` (never received)."""

    if seconds is None:
        return "n/a"
    return f"{seconds:.1f}s"


def _format_state(s: dict[str, Any]) -> str:
    battery = s.get("battery") or {}
    gps = s.get("gps") or {}
    connected = s.get("connected")
    # armed is tri-state: never render an unknown (None) as "disarmed".
    armed_raw = s.get("armed")
    if armed_raw is True:
        armed_text = "ARMED"
    elif armed_raw is False:
        armed_text = "disarmed"
    else:
        armed_text = "unknown" if connected else "n/a"
    lines = [
        f"connection : {'CONNECTED' if connected else 'DISCONNECTED'} "
        f"({s.get('connection_string') or 'n/a'})",
        f"heartbeat  : {_fmt(s.get('heartbeat_age_s'), 's')} ago"
        f"  sys={_fmt(s.get('system_id'))} comp={_fmt(s.get('component_id'))}",
        f"mode       : {s.get('flight_mode') or 'n/a'}",
        f"armed      : {armed_text}",
        f"status     : {s.get('system_status') or 'n/a'}"
        f"  landed={s.get('landed_state') or 'n/a'}"
        f"  rel_alt={_fmt(s.get('relative_alt_m'), ' m')}",
        f"battery    : {_fmt(battery.get('voltage_v'), ' V')}"
        f"  {_fmt(battery.get('current_a'), ' A')}"
        f"  {_fmt(battery.get('remaining_pct'), ' %', nd=0)}"
        f"  age={_fmt_age(s.get('battery_age_s'))}",
        f"gps        : {gps.get('fix_label') or 'n/a'}"
        f"  sats={_fmt(gps.get('satellites_visible'))}"
        f"  age={_fmt_age(s.get('gps_age_s'))}",
        f"telemetry  : age={_fmt_age(s.get('telemetry_age_s'))}",
    ]
    home = s.get("home_position")
    if home:
        lines.append(
            f"home       : lat={_fmt(home.get('lat_deg'), nd=7)}"
            f"  lon={_fmt(home.get('lon_deg'), nd=7)}"
            f"  alt_msl={_fmt(home.get('alt_msl_m'), ' m')}"
            f"  age={_fmt_age(s.get('home_position_age_s'))}"
        )
    return "\n".join(lines)


def _format_command(r: dict[str, Any]) -> str:
    action = r.get("action", "command")
    if r.get("dry_run"):
        verdict = "WOULD EXECUTE" if r.get("would_execute") else "no-op (already satisfied)"
        lines = [f"[dry-run] {action}: {verdict}"]
        if r.get("note"):
            lines.append(f"  note: {r.get('note')}")
        for c in r.get("checks", []):
            mark = "PASS" if c.get("passed") else "FAIL"
            lines.append(f"  [{mark}] {c.get('name')}: {c.get('detail')}")
        return "\n".join(lines)
    if r.get("already_satisfied"):
        return f"{action}: already satisfied ({r.get('note')}) — no action taken"
    outcome = r.get("outcome") or {}
    parts = [f"{action}: {outcome.get('result_name', 'OK')}"]
    if r.get("waited") is True:
        parts.append("(target state reached)")
    if r.get("note"):
        parts.append(f"— note: {r.get('note')}")
    return " ".join(parts)



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
