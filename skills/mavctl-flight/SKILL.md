---
name: mavctl-flight
description: Fly and monitor a MAVLink / ArduPilot vehicle (real or SITL) through the mavctl headless GCS CLI. Use when the user asks to connect to a drone, arm or disarm, change flight mode, take off, land, RTL, check vehicle status/telemetry, or diagnose mavctl errors. Enforces --confirm guards, exit-code semantics (3 daemon down, 4 link lost, 5 guard rejection, 6 NACK/timeout), and polling-based observation.
---

# mavctl flight control

Headless ground-control CLI for MAVLink vehicles (ArduPilot-first), designed
to be driven by agents. A resident daemon owns the vehicle link; every
`mavctl` invocation is a short request to it.

## Supported commands (complete list)

`status`, `telemetry`, `arm`, `disarm`, `mode`, `takeoff`, `land`, `rtl`,
`daemon start|stop|status`. Nothing else exists — do not guess or simulate
commands for missions, parameter editing, geofences, etc. They are not
implemented yet.

Common flags: `--json` everywhere; `--confirm` required on all state-changing
commands; `--dry-run` previews guard decisions; `--wait --timeout <s>`
(default 60) blocks until a target state is reached.

## Before flying: environment and connection

1. Check the daemon: `mavctl daemon status` (exit 0 running / exit 3 down).
2. If down: `mavctl daemon start --connect udp:127.0.0.1:14550 --json`
   (default SITL endpoint; pass whatever connection string your setup uses).
3. Verify with `mavctl status --json`: gate on `"connected": true`, fresh
   `heartbeat_age_s`, `gps.fix_type >= 3`.

## Standard takeoff workflow

```bash
mavctl mode GUIDED --confirm --wait --timeout 10
mavctl arm --confirm
```

Agent gate before takeoff — do not fire on the ACK alone: call
`mavctl status --json`, parse the JSON, and continue only when the `armed`
field is `true`. The arm ACK can beat the heartbeat by roughly a second.

```bash
mavctl takeoff --alt <metres> --confirm --wait --timeout <seconds>
```

Never send a second dangerous command while one is in `--wait`;
`status` / `telemetry` are always safe.

## Standard ending

- Prefer `mavctl rtl --confirm --wait --timeout 120`.
- Land in place with `mavctl land --confirm --wait --timeout 90`.
- Do not use `disarm` as a landing tool. `disarm --force` is an emergency
  motor stop only — in flight it can cause a crash.

## Safety rules and exit codes

- All state changes need `--confirm`; run `--dry-run` first when unsure.
- Exit 4 = no live vehicle state (link lost / heartbeat expired): stop
  controlling, fix the link, re-poll status.
- Exit 5 = guard rejection: read `detail.reason` + `detail.hint` from the
  stderr JSON and follow them.
- Exit 6 = vehicle NACK / ACK or wait timeout: query `status --json`, then
  decide; no tight retry loops.
- There is **no `arm --force`** anywhere; pre-arm checks cannot be bypassed.
- `landed_state` may be missing on ArduCopter; `disarm` needs provable ground
  contact, otherwise `ground_state_unknown` (exit 5).

## Async observation principles

- `--wait` blocks until the *target state*, not just an ACK.
- An arm ACK may precede `armed=true` by ~a heartbeat — poll before takeoff.
- Treat `status --json` as the current world state between commands.
- The daemon serializes dangerous commands, but issue them one at a time
  anyway; never parallelize flight commands.

Details: [references/workflows.md](references/workflows.md),
[references/safety.md](references/safety.md),
[references/troubleshooting.md](references/troubleshooting.md).
