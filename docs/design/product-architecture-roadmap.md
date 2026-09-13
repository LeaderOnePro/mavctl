# mavctl product & architecture roadmap

Status: **design/roadmap document — statements below are either recorded fact
(current state) or explicitly marked candidate/unresolved direction. Nothing
in the future sections is implemented or committed to a date.**

Companion repo idea: a standalone "drone-domain Agent Harness" is a
**candidate** (section D). No repository has been created.

---

## A. Current state (recorded facts)

| Item | State |
| --- | --- |
| PyPI latest stable release | `0.2.1` |
| main development version | `0.2.2.dev0` |
| Phase 1 (daemon + CLI + link/status/telemetry) | **done** (PR #1, tag `v0.1.0-phase1`) |
| Phase 2 (flight control + guards + ACK safety) | **done** (tags `v0.2.0-phase2`, `v0.2.0`) |
| Phase 2.1 (freshness metadata, `ground_state_stale`, `--version`) | **done** (PR #14) |
| 0.2.2 daemon consistency (heartbeat single threshold, mode TOCTOU) | **done on main**, unreleased (`0.2.2.dev0`, PR #23) |
| Mission protocol | **not started** |
| Platform support | macOS/Linux. Windows native IPC is design-only: Issue #19 |
| Agent Skill install | `npx skills add LeaderOnePro/mavctl -y -g` (verified against skills CLI 1.5.23; see `docs/SKILLS_CLI_ACCEPTANCE.md`) |
| Validation scope | ArduPilot SITL (ArduCopter, `udp:127.0.0.1:14550`): link/status/telemetry, guarded arm/disarm/mode/takeoff/land/rtl, `--wait`, freshness, link-loss exit 4, mode-map unavailability. **No real-aircraft validation** |

Current command surface (complete list): `daemon start|stop|status`,
`status`, `telemetry`, `arm`, `disarm`, `mode`, `takeoff --alt`, `land`,
`rtl`, `--version`; every command supports `--json`. All state-changing
commands require `--confirm` and support `--dry-run`; only `mode`,
`takeoff`, `land`, and `rtl` support `--wait` / `--timeout` — `arm` and
`disarm` do not.

Open issues and their true standing:

| Issue | Standing |
| --- | --- |
| #19 Windows native daemon IPC | open, **design doc merged** (`docs/design/windows-native-support.md`), unimplemented |
| #21 emergency interruption of long `--wait` | open, design+implementation tracking, deliberately not quick-fixed |
| #22 BATTERY_STATUS multi-battery policy | open, design issue (SYS_STATUS coverage is correct and wire-tested) |
| #4 EKF / pre-arm health guard | open, **not started** — candidate for Phase 4 |
| #5 freshness metadata | open but **delivered by Phase 2.1** (PR #14); candidate for closure after confirmation |
| #6 HOME_POSITION / EXTENDED_SYS_STATE source-filter tests | open but **delivered by Phase 2.1** (adapter tests); candidate for closure after confirmation |

---

## B. mavctl responsibility boundary (decided)

mavctl **is**:

- a **headless** MAVLink ground-control **execution layer** — no UI of any
  kind;
- a daemon-backed, agent-friendly CLI: stable exit codes, structured JSON,
  idempotent-friendly operations, self-describing status;
- the owner of the safety guard framework, MAVLink command transactions
  (ACK correlation, target locking, quarantine), and — in Phase 3 — the
  mission protocol;
- callable by humans, shell scripts, coding agents, and a future harness
  through the same CLI/daemon API.

mavctl **is not / must not become**:

- a map UI, conversation UI, or any interactive planning interface;
- an LLM inference/runtime or agent framework host;
- a drag/drop waypoint editor;
- an arbitrary unsafe MAVLink send escape hatch (this prohibition is
  already enforced by design and tests);
- a direct attitude/throttle flight-control AI — ArduPilot remains the
  flight controller.

---

## C. Revised mavctl roadmap (candidate phasing — no dates)

### Phase 3A — mission protocol core

- **Goal**: upload, download, and clear waypoint/mission plans on
  the vehicle via MAVLink mission protocol.
- **User-visible capabilities** (planned, not implemented):

  ```bash
  mavctl mission upload <mission.json> --confirm [--dry-run]
  mavctl mission download [--output <mission.json>] [--json]
  mavctl mission clear --confirm [--dry-run]
  ```

  `upload` sends a local mission JSON to the vehicle; `download` reads the
  vehicle's current mission and emits/writes it as JSON. Terminology follows
  common MAVLink/GCS usage; mission items are structured JSON throughout.
- **MVP input boundary** (planned, deliberately narrow):

  - `mission upload` accepts exactly **one explicit JSON file path**;
  - no stdin / pipe input;
  - no interactive mission editing;
  - no QGC WPL (`.waypoints`) import/export;
  - no partial upload, append, insert, or patch of an existing mission.

  Rationale: a mission is a high-risk, auditable object. A file input is
  previewable, reproducible, reviewable, and trivially produced by a future
  Harness export. stdin and other import formats may be discussed later,
  after a concrete need exists — they are **not** Phase 3A capabilities.
- **Safety constraints** (planned):

  - `mission upload` / `mission clear` require `--confirm` and support
    `--dry-run`;
  - both require a **fresh link state**;
  - both require the vehicle **disarmed with fresh positive ground
    evidence** (the same `ground_state_unknown` / `ground_state_stale`
    discipline as ordinary disarm);
  - `--dry-run` must not send any MAVLink traffic;
  - an upload timeout may leave the remote mission state **uncertain** —
    that outcome must be reported explicitly (never silently "probably
    fine");
  - `mission download` is a **read-only** operation and does not require
    `--confirm`;
  - no raw arbitrary MAVLink escape hatch.
- **Dependencies**: none beyond current main. **Must consider Issue #21**
  (long operations vs interruption) in design, but preemption is **not**
  required to land 3A.
- **Explicitly out of scope**: map UI, auto mission generation, geofence,
  rally points, execution/start commands (3B), stdin/pipe input, interactive
  editing, QGC WPL import/export, partial/append/insert/patch upload.
- **Release thought**: natural content of the `0.3.0` feature release
  (minor bump), after the `0.2.x` line closes.

### Phase 3B — mission execution / observation

- **Goal**: start/pause-resume/stop mission execution; observe progress.
- **User-visible capabilities** (planned):

  ```bash
  mavctl mission start --confirm [--wait] [--timeout]
  ```

  plus mission progress fields in `status --json` (current seq, total,
  state) and `--wait` for completion or waypoint milestones. Exact
  pause/resume/stop semantics belong to 3B design.
- **Start semantics**: `mission start` does **not** implicitly switch the
  vehicle to AUTO or any other mode; it requires an explicit, verifiable
  pre-flight state (mode, armed/disarmed posture, mission presence) — the
  precise precondition set is 3B design work, but it must be checkable by
  guards, not assumed.
- **Safety constraints**: start requires `--confirm`; mission running state
  and progress must be visible in `status --json` — Phase 3B must define the
  MAVLink data source and freshness semantics for those fields (the existing
  per-stream freshness ages do not automatically carry mission state);
  RTL/land must remain available — which **does** require an answer to
  Issue #21 before or with this phase.
- **Dependencies**: 3A; a design decision on #21 (preemption or explicit
  cancellation contract).
- **Out of scope**: autonomous replanning, vision/precision landing.
- **Release thought**: same `0.3.x` cycle or `0.4.0` depending on size of
  the #21 work.

### Phase 4 — configuration, parameters, diagnostics, logs, health

- **Goal**: the operational ground-control surface agents need beyond
  flight: parameter get/set (typed, range-checked), diagnostics/sensor
  health (EKF status → Issue #4), log listing/download, firmware/version
  reporting.
- **User-visible capabilities**: `param get/set/list`, `ekf status`,
  `health`, `log list/download` — exact surface decided at phase start.
- **Safety constraints**: parameter writes are state-changing (`--confirm`,
  dry-run); parameter changes that affect safety (e.g. fence/arm checks)
  must not be silently possible via a bulk path; guards unchanged.
- **Dependencies**: none hard; benefits from 3A's structured request
  patterns.
- **Out of scope**: firmware flashing, parameter *tuning automation*.
- **Release thought**: `0.4.x`/`0.5.x` range.

### Phase 5 — platform and operational hardening

- **Goal**: make the daemon deployable and operable beyond a single dev
  machine.
- **Contents**: Windows native IPC implementation (Issue #19, design
  already merged), Issue #21 interruption (if not consumed by 3B), CI
  expansion (Windows smoke per the #19 design), benchmark groundwork,
  multi-vehicle groundwork (multiple daemons / namespacing — design only).
- **Out of scope**: remote/public-network daemons (permanently non-goal
  until a separate threat model exists).
- **Release thought**: tracks implementation, not a version promise.

---

## D. Future Drone Agent Harness concept (candidate — no repo created)

A **separate, future repository**. Candidate layering:

```text
User / map / conversation UI          ← Harness (new repo)
        ↓
Agent orchestration / planning        ← Harness (new repo)
        ↓
mavctl execution and safety layer     ← this repo (existing)
        ↓
ArduPilot / MAVLink vehicle
```

Division of responsibility (candidate design):

| Layer | Responsibility |
| --- | --- |
| Harness UI | map click-to-waypoint, area/polygon selection, route preview, home/current-position display, mission JSON preview, **human approval** before upload/start, live status panel, conversation |
| Harness agent | natural-language understanding, task decomposition, tool calls into mavctl, explaining results and failures to the user |
| mavctl | constrained execution: guards, MAVLink transactions, mission protocol (3A/3B), exit-code contract |
| ArduPilot | actual flight control, EKF, failsafes |

Hard interface rules (candidate but strongly intended):

- the Harness does **not** import pymavlink;
- the Harness does **not** re-implement guards, ACK handling, heartbeat
  tracking, or the mission protocol — it consumes mavctl's CLI/daemon API;
- initially **SITL-only** target for the Harness;
- mavctl stays independent: it must remain fully usable by humans, bash,
  and coding agents without the Harness.

## E. Maps and mission planning (candidate direction)

- Mission upload itself is **map-independent**: planned mavctl 3A accepts a
  single explicit mission JSON file — a plain JSON file is sufficient as the
  narrow Phase 3A mission-upload input boundary; it is **not** a substitute
  for a user-facing planning UI.
- User-friendly inspection/survey/area missions need a map — that belongs
  to the future Harness, not mavctl.
- Candidate Harness map capabilities: click-to-waypoint, polygon/area
  selection, route preview, home/current-position display, mission JSON
  preview, human approval gate before upload/start.
- **Unresolved**: which map SDK/tile source (license, offline tiles,
  geofence data) — deliberately not chosen in this round; no map UI inside
  mavctl.

## F. Agent and RL direction (exploratory — nothing decided)

- A future Harness **may** use Pi or another agent runtime as the
  orchestration layer; **no runtime is currently integrated and none is
  decided**.
- Future small-model SFT/RL **could** optimize: constrained tool choice,
  structured mission planning, safety-aware retry/recovery, and SITL task
  completion rates.
- Non-negotiable: models must **not** emit low-level MAVLink messages,
  attitude, or throttle commands, and must not bypass mavctl guards. The
  action space stays bounded to high-level, validated tools/schemas;
  ArduPilot remains the flight controller.
- RL/SFT work requires a Harness + SITL benchmark to exist first; training
  design is a separate research effort after that. This document makes **no
  claim** about any specific framework, runtime, or model — those choices
  are open.

## G. Safety invariants across repos (non-negotiable)

1. SITL-first: every workflow is validated in SITL before any real aircraft
   is considered.
2. No force-arm anywhere, in any repo, at any layer.
3. Ordinary disarm requires **positive and fresh** ground evidence
   (`ground_state_unknown` / `ground_state_stale` otherwise).
4. Dangerous actions require explicit confirmation (`--confirm`).
5. Long-running mission plans require human preview/approval before
   upload/start (Harness responsibility, backed by mavctl's confirm gate).
6. No raw arbitrary MAVLink escape hatch for agents or the Harness.
7. Link loss stops unsafe control assumptions (exit 4 semantics).
8. Status freshness must remain visible (freshness ages never hidden on
   link loss).
9. The agent/Harness cannot bypass mavctl guards — the daemon is the
   enforcement boundary.
10. Direct RL/LLM low-level flight control is out of scope, permanently
    unless a future decision explicitly revisits it.

## H. Open questions

1. **Mission schema**: own JSON schema vs import/export compatibility with
   existing industry formats (e.g. QGC WPL) — decide at Phase 3A start;
   the Phase 3A MVP intentionally ships with file-path JSON only.
2. **Map SDK**: license, offline tile support, geofence data source (E).
3. **UI architecture**: Harness local app vs local server + browser;
   deployment model for non-developer users.
4. **Harness→mavctl integration**: subprocess CLI calls (version-pinned,
   exit codes) vs speaking the daemon RPC API directly (socket discovery,
   richer streaming) — likely CLI-first, unresolved.
5. **Approval UX & audit log**: how human approvals are recorded; whether
   mavctl gains a local audit log of guard decisions.
6. **Mission interruption / Issue #21**: cancellation contract vs priority
   preemption; must be settled before or with Phase 3B.
7. **Windows IPC scheduling** (Issue #19): which phase consumes it
   (candidate Phase 5).
8. **Multi-vehicle model**: one daemon per vehicle + namespacing vs a
   multiplexing daemon — design-only for now.
9. **RL benchmark & reward design**: task suite, success metrics, safety
   violations as hard negatives — after Harness + SITL exist.
10. **Data collection & scenario generation** for simulation training —
    unresolved, depends on 9.

---

## Related documents

- `docs/design/windows-native-support.md` — Windows IPC design (Issue #19)
- `docs/SITL_ACCEPTANCE_PHASE2.md` — current manual acceptance surface
- `docs/PUBLISHING.md` — release process and current dev state
