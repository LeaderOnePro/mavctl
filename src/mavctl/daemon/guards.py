"""Safety-guard framework for state-changing (dangerous) commands.

Each guard is a pure function of the cached :class:`VehicleState` plus command
parameters, returning a :class:`GuardDecision`. The daemon runs the relevant
guard before touching the adapter; ``--dry-run`` runs the guard and reports
without executing. Rejections carry a machine-readable ``reason``, a human
``message``, and a ``hint`` telling an agent what to do next.

Every guard shares a preamble of two gates, in order:
  1. confirm      — the command must be explicitly confirmed (exit 5)
  2. connection   — the link must be up and the heartbeat fresh (exit 4)
Only then are command-specific preconditions evaluated.
"""

from __future__ import annotations

import math

from pydantic import BaseModel

from mavctl.models import ExitCode, VehicleState

# Defaults for precondition thresholds.
DEFAULT_MAX_TAKEOFF_ALT_M = 120.0
DEFAULT_MIN_GPS_FIX_TYPE = 3  # 3D fix
DEFAULT_AIRBORNE_ALT_THRESHOLD_M = 1.0
DEFAULT_MAX_HEARTBEAT_AGE_S = 3.0
# Conservative ceiling under which a *known* relative altitude counts as
# "on the ground" for disarm. Deliberately far below the airborne threshold:
# altitudes between the two are treated as UNKNOWN, not as ground.
DEFAULT_MAX_ON_GROUND_ALT_M = 0.5
_GUIDED = "GUIDED"


class GuardConfig(BaseModel):
    """Tunable guard thresholds."""

    max_takeoff_alt_m: float = DEFAULT_MAX_TAKEOFF_ALT_M
    min_gps_fix_type: int = DEFAULT_MIN_GPS_FIX_TYPE
    airborne_alt_threshold_m: float = DEFAULT_AIRBORNE_ALT_THRESHOLD_M
    max_on_ground_alt_m: float = DEFAULT_MAX_ON_GROUND_ALT_M
    max_heartbeat_age_s: float = DEFAULT_MAX_HEARTBEAT_AGE_S


class GuardCheck(BaseModel):
    """One named precondition check and its result."""

    name: str
    passed: bool
    detail: str


class GuardDecision(BaseModel):
    """Outcome of running a command's guards.

    ``allowed`` says whether execution may proceed. When rejected,
    ``reason``/``message``/``hint`` explain why and what to do, and
    ``exit_code`` is the process code to surface. ``already_satisfied`` marks
    the idempotent case: allowed, but the desired state already holds.
    """

    allowed: bool
    action: str
    checks: list[GuardCheck] = []
    reason: str | None = None
    message: str | None = None
    hint: str | None = None
    exit_code: ExitCode = ExitCode.SUCCESS
    already_satisfied: bool = False
    note: str | None = None


def _passed(name: str, detail: str) -> GuardCheck:
    return GuardCheck(name=name, passed=True, detail=detail)


def _reject(
    action: str,
    reason: str,
    message: str,
    hint: str,
    checks: list[GuardCheck],
    failed_check: GuardCheck,
    exit_code: ExitCode = ExitCode.SAFETY_REJECTED,
) -> GuardDecision:
    return GuardDecision(
        allowed=False,
        action=action,
        checks=[*checks, failed_check],
        reason=reason,
        message=message,
        hint=hint,
        exit_code=exit_code,
    )


def _preamble(
    action: str, state: VehicleState, confirm: bool, config: GuardConfig
) -> tuple[GuardDecision | None, list[GuardCheck]]:
    """Run the shared confirm + connection gates.

    Returns ``(terminal_decision, checks)``. If ``terminal_decision`` is not
    None the caller must return it immediately; otherwise ``checks`` holds the
    passed preamble checks to extend with command-specific ones.
    """

    # Gate 1: confirmation.
    if not confirm:
        return (
            _reject(
                action=action,
                reason="confirmation_required",
                message=f"{action} is a state-changing command and requires explicit confirmation",
                hint=(
                    f"re-run with --confirm (preview first with --dry-run): "
                    f"mavctl {action} --confirm"
                ),
                checks=[],
                failed_check=GuardCheck(
                    name="confirm", passed=False, detail="--confirm not provided"
                ),
            ),
            [],
        )
    checks = [_passed("confirm", "confirmed")]

    # Gate 2: connection + heartbeat freshness. ``connected`` already encodes
    # "heartbeat age <= adapter timeout"; we additionally reject a stale or
    # missing heartbeat defensively so a guard never runs on phantom state.
    age = state.heartbeat_age_s
    fresh = state.connected and age is not None and age <= config.max_heartbeat_age_s
    if not fresh:
        age_text = "never" if age is None else f"{age:.1f}s"
        return (
            _reject(
                action=action,
                reason="not_connected",
                message=(
                    f"vehicle not connected or heartbeat stale "
                    f"(last heartbeat: {age_text} ago)"
                ),
                hint="ensure the vehicle link is up before commanding (check: mavctl status)",
                checks=checks,
                failed_check=GuardCheck(
                    name="connected",
                    passed=False,
                    detail=f"connected={state.connected} heartbeat_age={age_text}",
                ),
                exit_code=ExitCode.VEHICLE_NOT_CONNECTED,
            ),
            checks,
        )
    checks.append(_passed("connected", f"heartbeat {age:.1f}s ago"))
    return None, checks


def check_arm(state: VehicleState, *, confirm: bool, config: GuardConfig) -> GuardDecision:
    action = "arm"
    terminal, checks = _preamble(action, state, confirm, config)
    if terminal is not None:
        return terminal

    # Idempotency: only short-circuit on a *known* armed state. armed is None
    # (unknown) falls through to the checks below — the safe direction, since
    # we then re-verify preconditions rather than assume already-armed.
    if state.armed:
        return GuardDecision(
            allowed=True,
            action=action,
            checks=[*checks, _passed("already_armed", "vehicle already armed")],
            already_satisfied=True,
            note="already armed",
        )

    fix = state.gps.fix_type or 0
    if fix < config.min_gps_fix_type:
        return _reject(
            action=action,
            reason="gps_not_ready",
            message=(
                f"GPS fix insufficient for arming "
                f"(fix_type={fix}, need >= {config.min_gps_fix_type})"
            ),
            hint="wait for a 3D GPS fix before arming (check: mavctl status)",
            checks=checks,
            failed_check=GuardCheck(
                name="gps_fix", passed=False, detail=f"fix_type={fix} ({state.gps.fix_label})"
            ),
        )
    checks.append(_passed("gps_fix", f"fix_type={fix} ({state.gps.fix_label})"))
    return GuardDecision(allowed=True, action=action, checks=checks)


def check_disarm(
    state: VehicleState, *, confirm: bool, force: bool, config: GuardConfig
) -> GuardDecision:
    action = "disarm"
    terminal, checks = _preamble(action, state, confirm, config)
    if terminal is not None:
        return terminal

    # Only treat an explicit ``False`` as already-disarmed. armed is None
    # (unknown) is NOT treated as satisfied and falls through to the
    # ground/air evidence check below — the safe direction.
    if state.armed is False:
        return GuardDecision(
            allowed=True,
            action=action,
            checks=[*checks, _passed("already_disarmed", "vehicle already disarmed")],
            already_satisfied=True,
            note="already disarmed",
        )

    # --force is an explicit emergency override of every ground/air judgement
    # below; mark it in checks so the audit trail shows it was used. It does
    # NOT skip the idempotent short-circuit above (nothing to stop anyway).
    if force:
        return GuardDecision(
            allowed=True,
            action=action,
            checks=[
                *checks,
                _passed(
                    "on_ground",
                    f"forced (landed_state={state.landed_state} "
                    f"rel_alt={state.relative_alt_m})",
                ),
            ],
        )

    # Evidence of being airborne wins over anything else: checked BEFORE
    # on-ground so contradictory telemetry rejects safely.
    airborne = state.landed_state == "in_air" or (
        state.relative_alt_m is not None
        and state.relative_alt_m > config.airborne_alt_threshold_m
    )
    if airborne:
        return _reject(
            action=action,
            reason="in_flight",
            message="refusing to disarm: vehicle appears to be airborne",
            hint="land first (mavctl land --confirm) or override with --force (DANGEROUS)",
            checks=checks,
            failed_check=GuardCheck(
                name="on_ground",
                passed=False,
                detail=f"landed_state={state.landed_state} rel_alt={state.relative_alt_m}",
            ),
        )

    # Positive ground evidence: either an explicit landed_state or a known
    # relative altitude at/below the conservative on-ground ceiling.
    rel_alt = state.relative_alt_m
    on_ground = state.landed_state == "on_ground"
    low_known_alt = rel_alt is not None and rel_alt <= config.max_on_ground_alt_m
    if not (on_ground or low_known_alt):
        # Armed (or armed-unknown) but we can neither prove air nor ground.
        # Missing telemetry must never be read as "on the ground"; refuse
        # rather than lean on the autopilot's NACK as the safety net.
        return _reject(
            action=action,
            reason="ground_state_unknown",
            message=(
                "refusing to disarm: vehicle is armed but it cannot be safely "
                "determined whether it is on the ground"
            ),
            hint=(
                "check landed_state / relative_alt first (mavctl status or "
                "mavctl telemetry); land first (mavctl land --confirm) if in "
                "doubt; --force is reserved for an intentional emergency "
                "motor stop"
            ),
            checks=checks,
            failed_check=GuardCheck(
                name="ground_known",
                passed=False,
                detail=f"landed_state={state.landed_state} rel_alt={rel_alt}",
            ),
        )

    checks.append(_passed("ground_known", f"landed_state={state.landed_state} rel_alt={rel_alt}"))
    return GuardDecision(allowed=True, action=action, checks=checks)


def check_mode(
    state: VehicleState, mode: str, available: list[str], *, confirm: bool, config: GuardConfig
) -> GuardDecision:
    action = "mode"
    terminal, checks = _preamble(action, state, confirm, config)
    if terminal is not None:
        return terminal
    target = mode.upper()

    # Idempotency first: if the vehicle is already in the target mode nothing
    # will execute, so an unavailable mode map must not block a no-op.
    if state.flight_mode == target:
        return GuardDecision(
            allowed=True,
            action=action,
            checks=[*checks, _passed("already_in_mode", f"already in {target}")],
            already_satisfied=True,
            note=f"already in {target}",
        )

    if not available:
        # Connected but the vehicle's mode map has not populated yet. Calling
        # set_mode now would raise "unknown flight mode" and surface as an
        # internal error (exit 1), so reject structurally instead.
        #
        # Exit 5 semantics on purpose: exit 4 is reserved everywhere else for
        # *missing* vehicle state (no fresh heartbeat); here the link is alive
        # and only a precondition is not yet ready, so a safety rejection with
        # a retry hint is the honest code.
        return _reject(
            action=action,
            reason="mode_map_unavailable",
            message=(
                "vehicle mode mapping is not available yet; "
                f"cannot validate target mode {target!r}"
            ),
            hint=(
                "wait for the vehicle's mode map to populate "
                "(check: mavctl status), then retry this command"
            ),
            checks=checks,
            failed_check=GuardCheck(
                name="mode_known", passed=False, detail="mode list unavailable"
            ),
        )

    if target not in available:
        return _reject(
            action=action,
            reason="unknown_mode",
            message=f"unknown flight mode {target!r}",
            hint=f"choose one of: {', '.join(available)}",
            checks=checks,
            failed_check=GuardCheck(
                name="mode_known", passed=False, detail=f"{target} not available"
            ),
            exit_code=ExitCode.USAGE_ERROR,
        )

    checks.append(_passed("mode_known", f"{target} is available"))
    return GuardDecision(allowed=True, action=action, checks=checks)


def check_takeoff(
    state: VehicleState, altitude_m: float, *, confirm: bool, config: GuardConfig
) -> GuardDecision:
    action = "takeoff"
    terminal, checks = _preamble(action, state, confirm, config)
    if terminal is not None:
        return terminal

    # NaN slips every comparison, so it must be rejected explicitly before
    # any threshold logic; Infinity likewise never reaches the adapter.
    if not math.isfinite(altitude_m) or altitude_m <= 0:
        return _reject(
            action=action,
            reason="invalid_altitude",
            message=f"takeoff altitude must be a finite positive number (got {altitude_m})",
            hint="provide a positive --alt in metres",
            checks=checks,
            failed_check=GuardCheck(name="alt_positive", passed=False, detail=f"alt={altitude_m}"),
            exit_code=ExitCode.USAGE_ERROR,
        )
    checks.append(_passed("alt_positive", f"alt={altitude_m}m"))

    if altitude_m > config.max_takeoff_alt_m:
        return _reject(
            action=action,
            reason="altitude_limit",
            message=f"requested altitude {altitude_m}m exceeds limit {config.max_takeoff_alt_m}m",
            hint=f"request <= {config.max_takeoff_alt_m}m, or raise the configured limit",
            checks=checks,
            failed_check=GuardCheck(
                name="alt_limit", passed=False, detail=f"{altitude_m} > {config.max_takeoff_alt_m}"
            ),
        )
    checks.append(_passed("alt_limit", f"{altitude_m} <= {config.max_takeoff_alt_m}"))

    if state.flight_mode != _GUIDED:
        return _reject(
            action=action,
            reason="wrong_mode",
            message=f"takeoff requires {_GUIDED} mode (current: {state.flight_mode})",
            hint=f"set mode then arm then takeoff: mavctl mode {_GUIDED} --confirm && "
            "mavctl arm --confirm && mavctl takeoff --alt <m> --confirm",
            checks=checks,
            failed_check=GuardCheck(
                name="mode_guided", passed=False, detail=f"mode={state.flight_mode}"
            ),
        )
    checks.append(_passed("mode_guided", f"mode={state.flight_mode}"))

    # Require a *known* armed=True. armed None (unknown) or False both reject:
    # we never launch unless arming is positively confirmed — the safe side.
    if state.armed is not True:
        return _reject(
            action=action,
            reason="not_armed",
            message="takeoff requires the vehicle to be armed",
            hint="arm first: mavctl arm --confirm (then mavctl takeoff --alt <m> --confirm)",
            checks=checks,
            failed_check=GuardCheck(name="armed", passed=False, detail=f"armed={state.armed}"),
        )
    checks.append(_passed("armed", "armed"))
    return GuardDecision(allowed=True, action=action, checks=checks)


def check_land(state: VehicleState, *, confirm: bool, config: GuardConfig) -> GuardDecision:
    action = "land"
    terminal, checks = _preamble(action, state, confirm, config)
    if terminal is not None:
        return terminal

    # Only an explicit disarmed/on-ground state is treated as already-landed;
    # armed None (unknown) falls through and lands, which is the safe side.
    if state.armed is False or state.landed_state == "on_ground":
        return GuardDecision(
            allowed=True,
            action=action,
            checks=[*checks, _passed("already_landed", "already on ground / disarmed")],
            already_satisfied=True,
            note="already on ground",
        )
    return GuardDecision(allowed=True, action=action, checks=checks)


def check_rtl(state: VehicleState, *, confirm: bool, config: GuardConfig) -> GuardDecision:
    action = "rtl"
    terminal, checks = _preamble(action, state, confirm, config)
    if terminal is not None:
        return terminal

    # As with land: only an explicit on-ground/disarmed state short-circuits;
    # armed None (unknown) proceeds to command RTL (safe side).
    if state.armed is False or state.landed_state == "on_ground":
        return GuardDecision(
            allowed=True,
            action=action,
            checks=[*checks, _passed("already_home", "already on ground / disarmed")],
            already_satisfied=True,
            note="already on ground",
        )
    return GuardDecision(allowed=True, action=action, checks=checks)
