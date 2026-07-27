"""Safety-guard framework for state-changing (dangerous) commands.

Each guard is a pure function of the cached :class:`VehicleState` plus command
parameters, returning a :class:`GuardDecision`. The daemon runs the relevant
guard before touching the adapter; ``--dry-run`` runs the guard and reports
without executing. Rejections carry a machine-readable ``reason``, a human
``message``, and a ``hint`` telling an agent what to do next.
"""

from __future__ import annotations

from pydantic import BaseModel

from mavctl.models import ExitCode, VehicleState

# Defaults for precondition thresholds.
DEFAULT_MAX_TAKEOFF_ALT_M = 120.0
DEFAULT_MIN_GPS_FIX_TYPE = 3  # 3D fix
_GUIDED = "GUIDED"


class GuardConfig(BaseModel):
    """Tunable guard thresholds."""

    max_takeoff_alt_m: float = DEFAULT_MAX_TAKEOFF_ALT_M
    min_gps_fix_type: int = DEFAULT_MIN_GPS_FIX_TYPE


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


def _confirm_check(action: str, confirm: bool) -> GuardDecision | None:
    """Shared first gate: every dangerous command requires ``--confirm``."""

    if confirm:
        return None
    return _reject(
        action=action,
        reason="confirmation_required",
        message=f"{action} is a state-changing command and requires explicit confirmation",
        hint=f"re-run with --confirm (and --dry-run first to preview): mavctl {action} --confirm",
        checks=[],
        failed_check=GuardCheck(name="confirm", passed=False, detail="--confirm not provided"),
    )


def check_arm(state: VehicleState, *, confirm: bool, config: GuardConfig) -> GuardDecision:
    action = "arm"
    gate = _confirm_check(action, confirm)
    if gate is not None:
        return gate
    checks = [_passed("confirm", "confirmed")]

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
    gate = _confirm_check(action, confirm)
    if gate is not None:
        return gate
    checks = [_passed("confirm", "confirmed")]

    if state.armed is False:
        return GuardDecision(
            allowed=True,
            action=action,
            checks=[*checks, _passed("already_disarmed", "vehicle already disarmed")],
            already_satisfied=True,
            note="already disarmed",
        )

    airborne = state.landed_state == "in_air" or (
        state.relative_alt_m is not None and state.relative_alt_m > 1.0
    )
    if airborne and not force:
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
    checks.append(
        _passed("on_ground", "forced" if force else f"landed_state={state.landed_state}")
    )
    return GuardDecision(allowed=True, action=action, checks=checks)


def check_mode(
    state: VehicleState, mode: str, available: list[str], *, confirm: bool
) -> GuardDecision:
    action = "mode"
    gate = _confirm_check(action, confirm)
    if gate is not None:
        return gate
    checks = [_passed("confirm", "confirmed")]
    target = mode.upper()

    if available and target not in available:
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

    if state.flight_mode == target:
        return GuardDecision(
            allowed=True,
            action=action,
            checks=[*checks, _passed("already_in_mode", f"already in {target}")],
            already_satisfied=True,
            note=f"already in {target}",
        )
    return GuardDecision(allowed=True, action=action, checks=checks)


def check_takeoff(
    state: VehicleState, altitude_m: float, *, confirm: bool, config: GuardConfig
) -> GuardDecision:
    action = "takeoff"
    gate = _confirm_check(action, confirm)
    if gate is not None:
        return gate
    checks = [_passed("confirm", "confirmed")]

    if altitude_m <= 0:
        return _reject(
            action=action,
            reason="invalid_altitude",
            message=f"takeoff altitude must be positive (got {altitude_m})",
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

    if not state.armed:
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


def check_land(state: VehicleState, *, confirm: bool) -> GuardDecision:
    action = "land"
    gate = _confirm_check(action, confirm)
    if gate is not None:
        return gate
    checks = [_passed("confirm", "confirmed")]

    if state.armed is False or state.landed_state == "on_ground":
        return GuardDecision(
            allowed=True,
            action=action,
            checks=[*checks, _passed("already_landed", "already on ground / disarmed")],
            already_satisfied=True,
            note="already on ground",
        )
    return GuardDecision(allowed=True, action=action, checks=checks)


def check_rtl(state: VehicleState, *, confirm: bool) -> GuardDecision:
    action = "rtl"
    gate = _confirm_check(action, confirm)
    if gate is not None:
        return gate
    checks = [_passed("confirm", "confirmed")]

    if state.armed is False or state.landed_state == "on_ground":
        return GuardDecision(
            allowed=True,
            action=action,
            checks=[*checks, _passed("already_home", "already on ground / disarmed")],
            already_satisfied=True,
            note="already on ground",
        )
    return GuardDecision(allowed=True, action=action, checks=checks)
