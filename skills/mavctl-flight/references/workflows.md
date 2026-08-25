# mavctl workflows

Copy-paste command sequences built only from the current mavctl CLI surface
(`status`, `telemetry`, `arm`, `disarm`, `mode`, `takeoff`, `land`, `rtl`,
`daemon`). Every flag below exists in `mavctl <command> --help`.

All state-changing commands are serialized by the daemon: issue them **one at
a time** and observe the result before the next one. `status` / `telemetry`
are safe to run at any time, including during a `--wait`.

## 1. Start the daemon and verify the connection

```bash
# Is the daemon already running?
mavctl daemon status ; echo "exit=$?"          # exit 0 = running, exit 3 = not running

# Not running: start it against your vehicle endpoint.
# Default SITL endpoint is udp:127.0.0.1:14550; pass whatever mavutil string your setup uses.
mavctl daemon start --connect udp:127.0.0.1:14550 --json

# Verify the vehicle link before any flight command:
mavctl status --json
```

A healthy pre-flight `status --json` looks like:

```json
{
  "connected": true,
  "connection_string": "udp:127.0.0.1:14550",
  "heartbeat_age_s": 0.2,
  "flight_mode": "GUIDED",
  "armed": false,
  "system_status": "standby",
  "landed_state": null,
  "relative_alt_m": -0.01,
  "battery": {"voltage_v": 12.6, "current_a": 0.0, "remaining_pct": 100},
  "gps": {"fix_type": 6, "fix_label": "rtk_fixed", "satellites_visible": 10},
  "home_position": {"lat_deg": -35.36, "lon_deg": 149.16, "alt_msl_m": 584.09}
}
```

Gate on: `"connected": true`, fresh `heartbeat_age_s` (a few seconds at most),
`gps.fix_type >= 3`. `home_position` appears only after the vehicle sets home;
`landed_state` is often `null` on ArduPilot (do not depend on it).

## 2. Safe takeoff to 10 m

```bash
# Preview the guard decision without executing (optional but recommended):
mavctl takeoff --alt 10 --confirm --dry-run

# Switch to GUIDED and wait until the vehicle reports GUIDED:
mavctl mode GUIDED --confirm --wait --timeout 10

# Arm:
mavctl arm --confirm
```

Agent gate before takeoff — do not trust the arm ACK alone: the ACK can
arrive about one second before the heartbeat reports the state.

1. Call `mavctl status --json`.
2. Parse the JSON and read the `armed` field.
3. Proceed only when `armed == true`; otherwise wait briefly and re-poll.

Human shell equivalent (illustrative only — agents should parse
`mavctl status --json` directly rather than screen-scrape; needs `jq`):

```bash
while [ "$(mavctl status --json | jq -r '.armed')" != "true" ]; do sleep 1; done
```

```bash
# Take off and block until ~95% of the target relative altitude is reached:
mavctl takeoff --alt 10 --confirm --wait --timeout 45
```

While `takeoff --wait` is in progress, do **not** send another dangerous
command (arm/disarm/mode/takeoff/land/rtl) — it would queue behind the wait.
`status` and `telemetry` remain available for observation.

## 3. Observe with status / telemetry

```bash
mavctl status            # human-readable one-screen summary
mavctl status --json     # machine-readable world state (preferred for agents)
mavctl telemetry --json  # position / attitude / velocity snapshot
```

Use `status --json` as the single source of truth between commands. If
`connected` becomes `false` (or `heartbeat_age_s` grows past a few seconds),
stop issuing control commands and treat every other field as stale cache.

## 4. Return to launch (normal ending)

```bash
mavctl rtl --confirm --wait --timeout 120
# Blocks until the vehicle lands and disarms at home.
```

## 5. Land in place

```bash
mavctl land --confirm --wait --timeout 90
# Blocks until the vehicle lands and disarms at the current position.
```

Do **not** use `disarm` as a routine way to finish a flight. Landing via
`rtl` / `land` lets the autopilot touch down and disarm itself. `disarm
--confirm` is refused unless ground contact can be proven, and `disarm
--force` is an emergency motor stop that can cause a crash in flight.

## 6. Dry-run previews

```bash
mavctl arm --confirm --dry-run                 # exit 0 + checks, nothing executed
mavctl takeoff --alt 10 --confirm --dry-run    # e.g. rejected with not_armed (exit 5)
```

Dry-run runs the exact same safety guards and reports each check, but never
reaches the vehicle. A rejected dry-run returns the same exit code (usually 5)
the real attempt would produce.

## 7. Handling exit codes 4 / 5 / 6

Run commands with `--json`; failures print an error object on stderr:

```json
{"error": {"code": 5, "message": "...", "detail": {"reason": "...", "hint": "...", "checks": []}}}
```

| Exit | Meaning | Agent action |
| ---- | ------- | ------------ |
| 4 | No live vehicle state: link lost or heartbeat expired | Stop sending control commands. Diagnose the link (is SITL/the vehicle still up? correct endpoint?), then re-poll `status --json` until `connected` is true again |
| 5 | Safety guard rejection | Read `detail.reason` and `detail.hint` from stderr JSON and follow the hint (e.g. arm first, switch mode, land first) |
| 6 | Vehicle NACK / ACK timeout / `--wait` timeout | The command was not accepted, or was accepted but did not finish. Query `status --json` to see the actual state, then decide whether an idempotent retry makes sense |

Never retry a failed dangerous command in a tight loop. Each attempt is a new
transaction; understand the failure first.

## 8. Running SITL for practice

The common local setup is ArduPilot's simulator publishing on the default
endpoint:

```bash
sim_vehicle.py -v ArduCopter --out udp:127.0.0.1:14550
```

Any tool or adapter that exposes a mavutil-compatible endpoint works too
(`tcp:`, `serial:`, forwarded UDP …) — just pass the same string to
`mavctl daemon start --connect <string>`. See `references/troubleshooting.md`
for the macOS virtualenv note (`ModuleNotFoundError: pexpect`) when launching
SITL tools.
