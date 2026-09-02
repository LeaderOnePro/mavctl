# mavctl

English | [简体中文](README_ZH.md)

> Headless, agent-first MAVLink ground-control CLI for ArduPilot vehicles.

---

**Quick start — paste this into any coding agent:**

```bash
uv tool install mavctl
npx skills add LeaderOnePro/mavctl -y -g
```

连接飞控后就可以用自然语言指挥你的 Agent 控制无人机了——起飞、悬停、降落、返航，一条指令就够了。

---

mavctl is **ArduPilot-first** and built to be driven by both humans on a
terminal and AI coding agents (Claude Code, Codex, OpenClaw, …). A resident
daemon keeps the MAVLink link alive and caches vehicle state; every CLI call
is one short, structured request to that daemon.

**Status:** developed and verified against ArduPilot SITL. It has not been
proven across the breadth of real MAVLink vehicles and is **not** presented as
ready for production flight on a real aircraft.

## Why mavctl

GUI ground stations such as Mission Planner or QGroundControl are excellent
for a human at the controls — and a poor interface for a shell script or an
LLM agent: clickable UIs, no stable exit codes, no machine-readable output.

mavctl takes the other side of that trade:

- the daemon owns the MAVLink connection and continuously caches telemetry,
  so each command is quick and stateless;
- every command prints human-readable output by default and structured JSON
  with `--json`;
- failures carry explicit exit codes (3 daemon down, 4 link lost, 5 guard
  rejection, 6 vehicle NACK / timeout) instead of stack traces;
- dangerous operations pass safety guards before anything reaches the vehicle;
- mavctl embeds no LLM — it is designed to be *called* by agents such as
  Claude Code, Codex or OpenClaw, or by plain bash.

## Current capabilities

Implemented commands — this is the complete list:

```bash
mavctl daemon start|stop|status
mavctl status
mavctl telemetry
mavctl arm
mavctl disarm
mavctl mode <MODE>
mavctl takeoff --alt <metres>
mavctl land
mavctl rtl
```

Cross-cutting behaviour:

| Flag / behaviour | Meaning |
| ---------------- | ------- |
| `--json` | structured output on stdout; errors as `{"error": {...}}` on stderr |
| `--confirm` | required on every state-changing command; without it exit code 5 |
| `--dry-run` | run the exact same guards, never reach the vehicle |
| `--wait --timeout <s>` | block until the target state is reached (default 60 s) |
| idempotent repeats | re-applying an achieved change succeeds ("already armed") |
| transaction safety | ACK/NACK handling, serialized commands, link-loss abort |

Not implemented — current scope only, not a roadmap promise:

```text
Mission upload/download/start
Parameters
Geofence
Log download / analysis
Firmware flashing
Multi-vehicle orchestration
```

Notable in 0.2.1:

- `mavctl --version` prints the installed version — no daemon or vehicle
  needed.
- `status --json` carries per-stream freshness ages: `telemetry_age_s`,
  `gps_age_s`, `battery_age_s`, `home_position_age_s`, `landed_state_age_s`
  (monotonic-clock based; they keep counting after heartbeat loss so cache
  staleness stays visible).
- Ordinary non-force `disarm` requires fresh positive ground evidence; stale
  apparent ground evidence is rejected as `ground_state_stale` (exit 5).

## Quickstart with ArduPilot SITL

Requires Python >= 3.10 and [uv](https://docs.astral.sh/uv/). Always bring up
SITL first; do not point an agent-driven workflow at a real vehicle.

Terminal 1 — start ArduPilot SITL:

```bash
sim_vehicle.py -v ArduCopter --out udp:127.0.0.1:14550
```

Terminal 2 — install from source and connect:

```bash
uv sync
uv run mavctl daemon start --connect udp:127.0.0.1:14550
uv run mavctl status --json
```

Safe takeoff to 10 m and return to launch:

```bash
uv run mavctl mode GUIDED --confirm --wait
uv run mavctl arm --confirm
# Poll status --json until armed=true (the arm ACK can beat the heartbeat)
uv run mavctl takeoff --alt 10 --confirm --wait --timeout 45
uv run mavctl rtl --confirm --wait --timeout 120
uv run mavctl daemon stop
```

Safety notes — read before pointing mavctl at anything that flies:

- Validate every workflow in SITL first; treat real-aircraft use as its own
  review process.
- After `arm`, poll `status --json` until `armed=true` before takeoff: the
  COMMAND_ACK can arrive about one heartbeat before reported state catches up.
- End flights with `rtl` / `land`, not `disarm`. Ordinary `disarm` requires
  provable ground contact (`ground_state_unknown` otherwise).
- `disarm --force` is an emergency motor stop only — in flight it can cause a
  crash.
- There is no `arm --force` anywhere in mavctl; pre-arm checks cannot be
  bypassed.

## Install from PyPI

mavctl 0.2.0 is published on production PyPI. Install with:

```bash
uv tool install mavctl
# or run once without a persistent install:
uvx mavctl --help
# or:
pipx install mavctl
```

Verify the install — `mavctl --version` needs no daemon or vehicle:

```bash
mavctl --version
```

Before pointing mavctl at anything that flies, walk through the SITL-first
quickstart above. Release history and the publishing runbook live in
[docs/PUBLISHING.md](docs/PUBLISHING.md).

## Install from source

For development from source, use `uv sync` and `uv run mavctl …`:

```bash
git clone https://github.com/LeaderOnePro/mavctl.git
cd mavctl
uv sync
uv run mavctl --help
```

## Agent Skill

mavctl bundles a portable Agent Skill under `skills/mavctl-flight/`
(entrypoint plus workflows / safety / troubleshooting references). It is a
source asset of this repo: installing the Skill is independent of installing
the mavctl CLI from PyPI, e.g. `uv tool install mavctl`.

Install the bundled Agent Skill globally:

```bash
npx skills add LeaderOnePro/mavctl -y -g
```

The community skills CLI manages installation for the agent runtimes it
supports, according to its current environment and configuration. See
[docs/SKILLS_CLI_ACCEPTANCE.md](docs/SKILLS_CLI_ACCEPTANCE.md) for tested
behaviour and compatibility notes.

## Safety model

Short version; full details in
[skills/mavctl-flight/references/safety.md](skills/mavctl-flight/references/safety.md):

- the daemon owns the vehicle link; CLI calls are short transactions;
- state-changing commands require `--confirm`; `--dry-run` previews decisions;
- exit 4 = no live vehicle state (link lost / heartbeat expired), including
  mid-`--wait` loss (`link_lost_during_wait`);
- exit 5 = guard rejection with structured `reason` + `hint`;
- exit 6 = vehicle NACK / ACK timeout / wait timeout;
- force-arm does not exist at any layer (CLI option, RPC field, adapter verb);
- ordinary `disarm` needs positive ground evidence, else `ground_state_unknown`
  (exit 5);
- after link loss, `status` reflects stale cache: `armed` renders unknown/n/a,
  never silently disarmed.

## Development

```bash
uv sync
uv run ruff check .
uv run mypy .
uv run pytest -m "not sitl"
uv run pytest -m sitl   # requires a running ArduPilot SITL
```

Further reading:

- [docs/SITL_ACCEPTANCE.md](docs/SITL_ACCEPTANCE.md)
- [docs/SITL_ACCEPTANCE_PHASE2.md](docs/SITL_ACCEPTANCE_PHASE2.md)
- [AGENTS.md](AGENTS.md) — architecture rules and contribution constraints
- [skills/mavctl-flight/SKILL.md](skills/mavctl-flight/SKILL.md) — agent-facing flight guidance

## License

[MIT](LICENSE) — Copyright (c) 2026 LeaderOnePro.
