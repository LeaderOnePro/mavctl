# mavctl safety model

How mavctl's guard layer thinks, what it refuses, and why. All behaviour
described here is implemented in the daemon (`src/mavctl/daemon/guards.py`,
`server.py`) and covered by tests — nothing on this page is aspirational.

## Confirm and dry-run

- Every state-changing command (`arm`, `disarm`, `mode`, `takeoff`, `land`,
  `rtl`) **requires `--confirm`**. Without it the command is rejected before
  anything is sent: reason `confirmation_required`, exit code 5.
- `--dry-run` runs the exact same guard evaluation but never contacts the
  vehicle. Use it to preview a decision. A dry-run that would be rejected
  exits with the same code the real attempt would produce (usually 5).

## Exit-code contract

| Code | Meaning |
| ---- | ------- |
| 0 | success (including idempotent no-op and passed dry-run) |
| 1 | general error |
| 2 | usage error (bad altitude / timeout / unknown mode) |
| 3 | daemon not running |
| 4 | no live vehicle state (link lost or heartbeat expired) |
| 5 | safety guard rejection |
| 6 | vehicle NACK / ACK timeout / wait timeout |

Exit 4 means exactly "there is no alive vehicle state right now". A live link
with an undetermined precondition is exit 5, never 4.

## Positive ground evidence (disarm)

A non-force `disarm --confirm` is allowed **only** when ground contact can be
proven:

1. Airborne evidence first: `landed_state == "in_air"`, or known relative
   altitude above the airborne threshold (default 1 m) → rejected as
   `in_flight` (exit 5). Contradictory telemetry rejects safely here too.
2. Ground evidence: explicit `landed_state == "on_ground"`, or a known
   relative altitude at/below the on-ground ceiling (default 0.5 m,
   deliberately conservative) → allowed.
3. Everything else — including the ~1 s takeoff transition window and missing
   telemetry — is rejected as `ground_state_unknown` (exit 5).

Missing telemetry is never read as "on the ground", and the autopilot's own
NACK is not relied on as a design-level defence. On ArduPilot, ArduCopter
usually does not stream `EXTENDED_SYS_STATE`, so `landed_state` is often
unavailable — treat `disarm` as a tool for confirmed-ground situations only.

Thresholds are daemon-side configuration defaults (airborne 1 m, on-ground
0.5 m, max takeoff altitude 120 m); they are not CLI flags.

## Force-arm does not exist

There is no way to arm past the autopilot's pre-arm checks through mavctl:

- the CLI has no `arm --force` option;
- a direct RPC request carrying `"force": true` for arm is rejected at the
  daemon boundary with exit 2 (`unsupported_force_arm`) before reaching the
  adapter;
- the adapter's arm verb takes no arguments and always sends
  `param1=1.0, param2=0.0` — the MAVLink 21196 bypass magic cannot be sent.

Do not try to reconstruct a force-arm; pre-arm failures mean the vehicle is
telling you something you should read via `status --json`.

## disarm --force

`--force` exists on `disarm` only, and its help text says it plainly:
"Emergency motor stop; using in flight may cause a crash." It overrides every
ground/air judgement and marks the audit trail accordingly. Use it only for
an intentional emergency stop (e.g. runaway ground behaviour), never as a
landing procedure.

## Altitude limits

`takeoff` requires a finite positive altitude (exit 2 `invalid_altitude`
otherwise — NaN/Infinity included) and rejects targets above the configured
ceiling (default 120 m) as `altitude_limit` (exit 5).

## Command ACK vs actual flight state

A green COMMAND_ACK means the autopilot accepted the command — not that the
physical state has changed yet:

- after `arm --confirm`, `armed=true` can lag the ACK by roughly a heartbeat;
  poll `mavctl status --json` until `armed` is `true` before `takeoff`;
- `mode --wait`, `takeoff --wait`, `land --wait`, `rtl --wait` block until the
  *target state* is observed (e.g. altitude reached, disarmed after landing);
- `MAV_RESULT_IN_PROGRESS` counts as accepted; real refusals (DENIED/FAILED)
  surface as exit 6.

## Link-loss behaviour

If the heartbeat expires mid-`--wait` (default threshold 3 s, settable via
`daemon start --heartbeat-timeout`), the daemon stops waiting immediately and
returns exit 4 with reason `link_lost_during_wait` plus the already-received
ACK in the detail — the command side succeeded, the observation side did not.
After the link returns, the daemon reconnects on its own; re-poll `status`.

## Cached data caveats

`status` / `telemetry` read the daemon's latest snapshot:

- when `connected` is `false`, volatile fields are suppressed or unknown
  (`armed` renders `unknown`/`n/a`, never silently `disarmed`) and battery /
  GPS values may be stale cache — do not act on them as live data;
- `landed_state` may be absent entirely on ArduCopter;
- prefer fresh `heartbeat_age_s` (small, single-digit seconds) as your
  freshness signal.
