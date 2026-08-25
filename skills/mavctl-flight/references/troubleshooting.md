# mavctl troubleshooting

Failure modes an agent will actually hit, with the real exit codes, JSON
reasons, and fixes. All commands shown exist in the current CLI.

## Exit 3 — daemon not running

```text
$ mavctl status
error: ... exit=3
```

The daemon process is down (never started, crashed, or machine rebooted).

```bash
mavctl daemon status --json        # confirm: not running
mavctl daemon start --connect udp:127.0.0.1:14550 --json
mavctl status --json               # verify connected=true before continuing
```

If `daemon start` reports the daemon exited immediately, check that the
vehicle endpoint is reachable and that no stale daemon holds the socket.

## Exit 4 — vehicle not connected

`status --json` shows `"connected": false`, or a command fails with reason
`not_connected`. There is no fresh heartbeat.

- Is SITL / the vehicle actually still running and publishing on the endpoint?
- Does `mavctl daemon start --connect <string>` use the same string as your
  simulator output?
- After fixing the link, just re-poll `status --json` — the daemon reconnects
  by itself; do not restart it.
- During `--wait`, link loss surfaces as exit 4 with
  `detail.reason == "link_lost_during_wait"`; see `references/safety.md`.

## confirmation_required (exit 5)

You forgot `--confirm`. Re-run with it. Preview first with `--dry-run`.

## gps_not_ready (exit 5, arm)

Arming requires at least a 3D fix (`gps.fix_type >= 3`). Poll
`status --json` until `gps.fix_label` shows `3d_fix` or better. SITL usually
converges within seconds; a real vehicle may need sky visibility or a reset.

## wrong_mode (exit 5, takeoff)

Takeoff requires GUIDED. The hint tells you the sequence:

```bash
mavctl mode GUIDED --confirm --wait --timeout 10   # then retry takeoff
```

## not_armed (exit 5, takeoff)

Arm first — but remember the ACK/state gap:

```bash
mavctl arm --confirm
# poll status --json until "armed": true, then:
mavctl takeoff --alt 10 --confirm --wait --timeout 45
```

## ground_state_unknown (exit 5, disarm)

The vehicle is armed (or its arm state is unknown) and neither air nor ground
can be proven — typically because ArduCopter does not stream `landed_state`
and relative altitude is missing/stale. Correct responses:

- land instead: `mavctl land --confirm --wait --timeout 90`;
- re-check evidence with `status --json` / `telemetry --json`;
- only if you *intend* an emergency motor stop:
  `mavctl disarm --confirm --force` (may cause a crash in flight).

## mode_map_unavailable (exit 5, mode)

The link is up but the vehicle's flight-mode table has not populated yet, so
the target mode cannot be validated. Wait and re-poll:

```bash
mavctl status --json    # until "flight_mode" shows a real mode name
mavctl mode LOITER --confirm
```

Switching to the already-active mode stays an idempotent success even while
the map is empty.

## altitude_limit / invalid_altitude (exit 5 / exit 2, takeoff)

- `altitude_limit` (exit 5): target above the configured ceiling (default
  120 m). Choose a lower `--alt`.
- `invalid_altitude` (exit 2): altitude ≤ 0, non-numeric, NaN or Infinity.
  Provide a finite positive metre value.

## unknown_mode (exit 2, mode)

The mode name is not in the vehicle's table (typos, wrong frame). Use a name
from `status --json` → `flight_mode` (e.g. GUIDED, LOITER, RTL, LAND).

## NACK / timeout (exit 6)

Two shapes:

- **Vehicle refused** (`DENIED`/`FAILED`/...): e.g. arming while already in
  RTL is refused by the autopilot itself — this is the final safety net doing
  its job. Read `detail.outcome.result_name`, query `status --json`, adjust.
- **Wait timeout**: command accepted but target state not reached within
  `--timeout`. Query `status` to see how far it got; retrying the same
  command is often idempotent-safe, but understand the state first.

Do not blind-retry in a loop; each attempt is a new transaction.

## unsupported_force_arm (exit 2)

Someone sent force for arm over direct RPC. There is deliberately no force
arm anywhere in mavctl; pre-arm checks cannot be bypassed. Fix the underlying
pre-arm condition instead.

## invalid_timeout (exit 2)

`--timeout` must be a finite number > 0 (`0`, `-1`, `nan`, `inf`, `abc` all
fail). The daemon enforces this boundary; there is no silent fallback to the
60 s default.

## macOS / SITL: ModuleNotFoundError: pexpect

Launching ArduPilot SITL tools (`sim_vehicle.py`) with the system Python
fails like:

```text
ModuleNotFoundError: No module named 'pexpect'
```

Cause: the ArduPilot virtualenv was not activated. Fix:

```bash
source ~/venv-ardupilot/bin/activate
sim_vehicle.py -v ArduCopter --out udp:127.0.0.1:14550
```

This is a simulator-side environment issue — unrelated to mavctl itself;
once SITL is up, mavctl talks to it over UDP normally.
