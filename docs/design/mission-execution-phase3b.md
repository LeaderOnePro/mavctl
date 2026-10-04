# Design: Phase 3B — Mission Execution, Progress Observation, and Safe Interruption

Status: **mission start / observation mock-first implemented** (Phase 3B-1,
0.4.0.dev0): `mission start --confirm [--wait] [--timeout] [--dry-run]`,
the `MissionExecutionState` observation (MISSION_CURRENT, locked source),
`check_mission_start` guards, and the operation integration are
implemented and mock-tested; **SITL execution conformance is pending**.
Pause/resume/stop and interruption runtime remain unimplemented. Tracking
issue: #31. This document is the input for the future Phase 3B
implementation branch; the [FACT]/[DECIDED]/[OPEN]/[NON-GOAL] discipline
matches docs/design/mission-protocol-v1.md (Phase 3A).

---

## A. Current preconditions (what Phase 3A already provides)

`mavctl 0.3.0` (main @ `96be778`) provides:

- `mavctl mission upload / download / clear` (ArduPilot-first Mission JSON
  v1, lossless-or-fail download, fresh-ground guards, clear read-back
  verification, ArduPilot home-slot wire compatibility, relay-residue settle
  quarantine) — `[FACT]`, released and documented in
  docs/PUBLISHING.md (`Production release record: 0.3.0`).
- It does **not** provide: `mission start`, AUTO transition by mavctl,
  progress observation fields, any interruption mechanism — `[FACT]`.
- Progress-adjacent messages are already subscribed at the adapter level
  (`MISSION_CURRENT`, `MISSION_ITEM_REACHED` are in `_SUBSCRIBED`), but they
  are routed into the mission transaction inbox and **unused** — `[FACT]`
  (pymavlink_adapter.py `_MISSION_TRANSACTION_TYPES`).

Known related issues:

- **#21** — long `--wait` emergency interruption (blocking constraint for
  Phase 3B `--wait`; see §E).
- **#25** — Phase 3A mission core (closed: implemented and released).
- **#4** — EKF / pre-arm health guard (adjacent: `mission start` preconditions
  would benefit from it, but it is independent work).

---

## B. MAVLink / ArduPilot execution facts (source research, read-only)

All items below were verified against the **local ArduPilot checkout**
(revision `4c98c9221a`, the same provenance as the Phase 3A validation) and
the installed pymavlink 2.4.49 common dialect. Nothing here has been
exercised on SITL yet — SITL verification is Phase 3B implementation work.

### B.1 `MAV_CMD_MISSION_START` (33) — vehicle-side semantics

- **Direction**: GCS → vehicle command (`COMMAND_LONG`/`COMMAND_INT`),
  answered with `COMMAND_ACK`.
- **ArduCopter handler** (`ArduCopter/GCS_MAVLink_Copter.cpp`,
  `handle_MAV_CMD_MISSION_START`) `[FACT]`:
  - `param1`/`param2` must be zero — first-item/last-item selection is
    **not supported** and answers `MAV_RESULT_DENIED`;
  - the handler **switches the vehicle to AUTO itself**
    (`copter.set_mode(Mode::Number::AUTO, ModeReason::GCS_COMMAND)`),
    calls `set_auto_armed(true)` (auto-throttle arming of the flight mode —
    **not** motor arming), and calls `mission.start_or_resume()` only if the
    mission is not already `MISSION_RUNNING`;
  - returns `MAV_RESULT_ACCEPTED` only if the mode switch succeeded,
    `MAV_RESULT_FAILED` otherwise.
- **Consequence for the product contract**: the AUTO transition is part of
  the vehicle's command semantics. mavctl cannot send
  `MAV_CMD_MISSION_START` and honestly claim "the vehicle stayed in its
  previous mode" — `mission start` is therefore designed as an execution
  command that may transition the vehicle to AUTO (§C.1), gated by
  `--confirm` and the guard chain.
- On a **disarmed** vehicle the command still succeeds logically (mode
  AUTO + mission started) but nothing flies until motors are armed —
  guard implications in §F. `[FACT]` for the code path; physical behavior
  SITL-unverified.

### B.2 Mission auto-start on entering AUTO

- `ArduCopter/mode_auto.cpp` `ModeAuto::run()`: on entering AUTO the mode
  sets `waiting_to_start`, and once the vehicle has an EKF origin it calls
  `mission.start_or_resume()` — **entering AUTO starts or resumes the
  stored mission by itself**; `MIS_RESTART` governs start-vs-resume
  behavior `[FACT]` (parameter value semantics SITL-unverified).
- Consequence: `mavctl mode AUTO --confirm` **alone can start the stored
  mission**. Phase 3A/3B documentation and guards must not imply that mode
  AUTO is a neutral state change. This is central to §C and §F.

### B.3 `MISSION_CURRENT` (42) — progress evidence

- **Direction**: vehicle → GCS, streamed (`MSG_CURRENT_WAYPOINT` deferred
  bucket) and also emitted on `MISSION_SET_CURRENT` `[FACT]`
  (`GCS_Common.cpp send_mission_current`).
- **Fields populated by ArduPilot** `[FACT]`:
  - `seq` = `mission.get_current_nav_index()` (current nav item index);
  - `total` = `num_commands() - 1` (home slot excluded from the count);
  - `mission_state` = mapped from `AP_Mission::state()`:
    `MISSION_STATE_NO_MISSION` (empty) / `NOT_STARTED` (STOPPED) /
    `ACTIVE` (RUNNING) / `COMPLETE`;
  - `mission_mode` = 1 while the current flight mode requires a mission.
  (`latitude/longitude/altitude` payload extensions from the MAVLink XML
  are **not** populated by this sender.)
- **Safe to use as progress evidence**: yes, with freshness metadata —
  `seq` progression plus `mission_state`/`mission_mode` is the most
  complete signal. The mapping is direct from the mission library state, so
  "ACTIVE but seq not advancing" is expressible and must be representable
  (§D).
- **SITL-unverified**: observed stream rate on the mavctl link, exact
  `total` semantics with the home slot, behavior across
  upload-while-running.

### B.4 `MISSION_ITEM_REACHED` (46)

- **Direction**: vehicle → GCS, `mavlink_msg_mission_item_reached_send(chan,
  mission_item_reached_index)` via `GCS::send_mission_item_reached_message`
  `[FACT]`.
- **Emission sites found in the local checkout**: ArduPlane
  (`commands_logic.cpp`, `navigation.cpp`) and ArduSub only — **no
  ArduCopter call site was found**. `[FACT]` for the local source; treat
  MISSION_ITEM_REACHED as **Plane/Sub-verified, Copter-unverified** —
  Phase 3B must not make it a required progress signal for Copter. `[OPEN]`
  whether any Copter path emits it (SITL sniff on the dedicated instance
  will settle it).

### B.5 Heartbeat / AUTO mode mapping

- Copter `custom_mode` for AUTO is **3** (`ArduCopter/mode.h`:
  `AUTO = 3`) `[FACT]`; mavctl already maps it
  (`_COPTER_MODE_LABELS`, `mode AUTO` command works today).
- "Flight mode == AUTO" is therefore observable from the existing heartbeat
  path and is a valid `mission start --wait` milestone component `[FACT]`.

### B.6 `MAV_CMD_DO_PAUSE_CONTINUE` ( Pause/Resume )

- **Direction**: GCS → vehicle command; **ArduCopter implements it**
  (`handle_command_pause_continue`): `param1=0` → `flightmode->pause()`,
  `param1=1` → `flightmode->resume()`, other → `MAV_RESULT_DENIED`;
  `ACCEPTED`/`FAILED` per call `[FACT]`.
- Catalogued as a **future hook only** — pause/resume mavctl commands are a
  `[NON-GOAL]` for initial Phase 3B (§H, §E.5) but the vehicle-side surface
  exists, which matters for the Issue #21 interplay (a paused mission is a
  safer pre-emption state than an aborted one).

### B.7 Vehicle-side mission-state safety facts (from Phase 3A work)

- `AP_Mission::clear()` refuses while `soft_armed && MISSION_RUNNING`
  (`[FACT]`, source-read during the 0.3.0 clear-path work) — the vehicle
  itself protects a running mission from being cleared in flight; mavctl
  must not assume `mission clear` succeeds on an armed flying vehicle.
- `MAV_CMD_MISSION_START` on an already-`MISSION_RUNNING` mission skips
  `start_or_resume()` (no restart) — idempotency signal usable by mavctl
  `[FACT]`.

---

## C. Candidate CLI contract

```bash
mavctl mission start --confirm [--wait] [--timeout <s>]
```

Decisions below are [DECIDED] where the Phase 3A safety model plus the §B
facts are sufficient; otherwise [OPEN] for the implementation round.

### C.1 Mission start / AUTO product contract

`[DECIDED]` **`mission start` is an execution command that may change the
flight mode to AUTO on ArduCopter.** This is not implicit behavior: the user
explicitly invoked `mission start` and must provide `--confirm`, and the
vehicle-side semantics of `MAV_CMD_MISSION_START` (§B.1) are documented —
the handler itself switches the vehicle to AUTO, sets auto-armed, and
starts or resumes the stored mission. The contract therefore does **not**
require `mode == AUTO` as a precondition and does **not** claim that
"mission start does not implicitly change AUTO" — that phrasing contradicts
the verified ArduCopter behavior and is removed from this design.

### C.1.1 Preconditions (guard chain, mirroring Phase 3A style)

| Precondition | Decision | Rationale |
| --- | --- | --- |
| `--confirm` required | `[DECIDED]` | execution command; exit 5 `confirmation_required` otherwise |
| fresh daemon/vehicle link (heartbeat age) | `[DECIDED]` | same preamble as every dangerous command |
| mission exists and remote wire count >= 2 (verified by read-back, not cache) | `[DECIDED]` | `mission start` with no stored mission cannot succeed; Phase 3A download gives the authoritative count. The wire count includes the vehicle-managed home slot (Phase 3A convention): an empty mission with home written reports `MISSION_COUNT == 1`, and ArduPilot re-writes home on every unlocked arming (AP_Arming_Copter → AP_AHRS::set_home → write_home_to_storage), so a flown-then-cleared vehicle reports 1 — only count >= 2 proves at least one mission item. Confirmed against ArduCopter SITL (§8i) |
| vehicle heartbeat reports `armed == true` | `[DECIDED]` candidate | ArduPilot's `set_auto_armed(true)` (§B.1) is an **internal flight-mode state, not vehicle motor arming** — mavctl must still require actual heartbeat `armed == true` (exit 5, distinct reason), otherwise a mission could "start" on a disarmed vehicle |
| execution readiness (GPS / home / telemetry freshness) | `[OPEN]` — explicit Phase 3B design decision | mission execution needs an EKF origin (B.2); freshness discipline exists since Phase 2.1; exact required set to be fixed at implementation review |
| EKF / pre-arm health | `[NON-GOAL]` for Phase 3B v1 | Issue #4 — independent; mavctl relies on the vehicle's own pre-arm checks |

`[DECIDED]` guard-side non-actions (all silent absences, explicitly):
no implicit motor arming, no implicit takeoff, no hidden mission
upload/clear, and no execution at all without `--confirm`.

### C.1.2 Already-running / AUTO-but-idle behavior

- **already AUTO + mission active** (`mission_state == ACTIVE` observed
  before sending): `[DECIDED]` candidate — mavctl treats this as an
  **idempotent success** (`already_running: true`, mirroring the Phase 2.1
  `already armed` pattern) and does **not** re-send
  `MAV_CMD_MISSION_START`, avoiding dependence on `MIS_RESTART` and other
  parameter-dependent restart semantics.
- **AUTO but mission not active** (`mission_state` NOT_STARTED/STOPPED):
  `[DECIDED]` — mavctl sends `MAV_CMD_MISSION_START`; ArduCopter's handler
  calls `start_or_resume()` when not already RUNNING (source-verified).
  SITL verification of the observable outcome is still pending; if SITL
  contradicts this, downgrade to refuse-with-hint.

### C.2 What `--wait` waits for

Candidate milestone (`[DECIDED]` as the v1 policy; each element **must be
downgraded to `[OPEN]` and re-designed if ArduPilot/SITL shows the
underlying fields unreliable** — never hardcoded as fact):

1. `COMMAND_ACK(MAV_CMD_MISSION_START)` = ACCEPTED (command-level outcome);
2. vehicle heartbeat flight mode observed as AUTO (Copter custom_mode 3);
3. `MISSION_CURRENT` observed with `mission_state == ACTIVE`
   (`MISSION_STATE_ACTIVE`, §B.3).

`--wait` = all three within `--timeout`. **`--wait` does not wait for whole
mission completion** (`mission_state == COMPLETE` is `[NON-GOAL]` for the
default `--wait` — missions are minutes-to-hours; a bounded observation
window is what an agent needs to confirm the start succeeded). If
`MISSION_CURRENT` proves unreliable in SITL, milestone elements 2–3
downgrade to `[OPEN]` and the contract falls back to ack-only + observation
guidance.

### C.3 Defaults and exit mapping

| Aspect | Value | Exit code |
| --- | --- | --- |
| default `--timeout` | 60 s (`--wait` milestone window) | 6 on timeout |
| not connected | — | 4 |
| guard refusal (confirm/link/disarmed-policy/count==0/mode policy) | — | 5 |
| `COMMAND_ACK` DENIED (e.g. param1/2 set) / FAILED (mode switch refused) | — | 6 (`mission_start_rejected`, result name in detail) |
| timeout waiting milestone | — | 6 (`remote_mission_state_uncertain`-style: remote mission may be active — read-back/observe, never claim failure of state) |
| schema/usage errors | — | 2 |
| success (ack + milestone) | outcome JSON + human line | 0 |

### C.3 Why `--wait` does not wait for the whole mission

`[DECIDED]`: missions run minutes-to-hours; a blocking CLI command holding
a transaction for the entire flight contradicts the Issue #21 analysis
(§E) and the Phase 3A daemon lock model. `--wait` confirms the *start*
milestone; ongoing progress belongs to `status --json` observation (§D) and
future pause/resume/stop work.

---

## D. Mission execution state model (future `status --json`)

Candidate shape (illustrative; not all fields may be available):

```json
{
  "mission": {
    "current_seq": 1,
    "total": 4,
    "state": "active",
    "mode": "mission"
  }
}
```

Field provenance and freshness:

| Field | Source | Freshness |
| --- | --- | --- |
| `mission.current_seq` | `MISSION_CURRENT.seq` (vehicle-populated, §B.3) | `mission_current_age_s` (monotonic, same discipline as `telemetry_age_s` — never conflated with it) |
| `mission.total` | `MISSION_CURRENT.total` (vehicle-populated) | same age as `current_seq` (same message) |
| `mission.state` | `MISSION_CURRENT.mission_state` mapped to `no_mission` / `not_started` / `active` / `complete` / `unknown` | same age |
| `mission.mode` | `MISSION_CURRENT.mission_mode` (1 = current mode requires mission) | same age |
| reached-item history | **deferred**: `MISSION_ITEM_REACHED` is Copter-unverified (§B.4) | n/a for v1 |

`[DECIDED]` candidates:

- **every mission field is a candidate**, sourced in principle from
  `MISSION_CURRENT` (`current_seq`/`total`/`state`/`mode`, §B.3); if SITL
  shows any field unreliable, that field downgrades to `[OPEN]` or is
  dropped — never hardcoded as fact.
- all mission fields must carry their **own freshness age**
  (`mission_*_age_s`, same monotonic discipline as Phase 2.1 ages);
- **mission active must never be inferred** from the AUTO mode or from
  position/progress heuristics — the only source is
  `MISSION_CURRENT.mission_state`; when no `MISSION_CURRENT` has been
  received, mission state is **unknown/null** (`null` value + `null` age);
- `mission_state == ACTIVE` is the candidate success signal for
  `mission start --wait` (§C.2, downgradable);
- `state: "unknown"` vs `no_mission` must stay distinguishable
  (`MISSION_STATE_UNKNOWN` value / never-received);
- ages live in their own namespace (`mission_*_age_s`) — never mixed into
  `telemetry_age_s` (different message, different cadence);
- `seq` progression is observation-only in v1 (a stalled seq with ACTIVE
  state is a reportable observation, not a command failure);
- daemon-side inference: the daemon may label `current_seq` as "nav index"
  but must not translate it into "waypoint N of M" prose (item indices
  include command/do-jump semantics the daemon does not model).

`[OPEN]`: whether `MISSION_CURRENT` streaming needs an explicit
`MAV_CMD_SET_MESSAGE_INTERVAL` request (like Phase 3A's position/home
streams) — depends on the observed SITL stream rate on a bare link.

---

## E. Issue #21 integration: long `--wait` and safe interruption

Current mechanics (`[FACT]`): every state-changing command runs under the
daemon `_command_lock` for its whole transaction, including `--wait`
windows (up to `--timeout`); `status`/`telemetry` never take that lock, so
observation stays live, but any *other* state-changing command queues
behind a long `--wait` (e.g. `takeoff --wait --timeout 60` blocks a
subsequent `rtl --confirm` for up to 60 s). Additionally the adapter holds
`_mission_lock` for whole mission transactions.

Phase 3B `mission start --wait` adds a new long-waiting command, so the
interruption question must be answered. Two candidate models:

### E.1 Option 1 — operation model (asymmetric: observe, don't preempt)

`mission start` registers a daemon-side operation (id, kind, deadline,
milestone state) and `--wait` becomes a *client-side* observation loop:
the daemon does not hold a worker blocked for the whole window — the RPC
handler polls the operation state, and the CLI polls or long-polls the
operation.

- Race conditions: command ordering stays strict (the actual
  `MAV_CMD_MISSION_START` still runs under `_command_lock` once); the
  wait itself moves out of the lock, so `rtl` can always run. Worst case:
  two agents start two operations — operations are single-slot per kind
  (one active mission-start observation), later starts are refused as
  already-running.
- Daemon lock semantics: `_command_lock` hold time shrinks to the ack
  exchange (~ms); milestone observation happens lock-free off the shared
  snapshot (like `status`).
- Terminal state: operation ends on milestone reached / deadline / link
  loss / explicit cancel.
- User/agent contract: `mission start --wait` still blocks client-side by
  default, but a second agent can `rtl` immediately; the first client's
  wait simply observes the resulting state change (mode leaves AUTO →
  milestone can never complete → operation reports superseded/aborted).
- Failure mode: none of the Phase 2 TOCTOU races return, because the
  *effecting* command remains serialized; only the *observation* is
  decoupled.
- Why `disarm --force` must not automatically preempt: force-disarm is a
  direct motor-stop command — it does not need the wait to be cancelled to
  execute (it runs under `_command_lock` after the current
  effecting-command finishes, ~ms), and making it a preemption control
  would couple an emergency action to observation bookkeeping. Emergency
  response today: run `disarm --force` in another terminal — it already
  works and never queues behind anything the agent can't avoid (commands
  queue, but each effecting command is ~ms long). The real blocker today
  is only the wait window itself, which Option 1 removes.
- Can RTL/land "interrupt" a running mission? The vehicle answers for
  itself: switching out of AUTO (e.g. `mode RTL --confirm`) stops mission
  execution at the flight-mode level (`[FACT]` mission runs only in AUTO);
  `mission clear` is refused by the vehicle while armed+running (§B.7).
  mavctl needs no mission-protocol preemption — mode change **is** the
  interruption. This keeps the mission protocol state machine untouched.

### E.2 Option 2 — cancellable / priority command model

`rtl`/`land`/`disarm` declare preemption priority; the daemon cancels a
running wait (event set through the transaction), rolls the waiter back to
"superseded", then runs the priority command.

- Race conditions: cancellation must be atomic with lock handover
  (cancel-flag + condvar inside `_command_lock`); the interrupted client
  must get a well-defined outcome (`wait_superseded_by_rtl`), and the
  vehicle state after preemption is *mode-dependent* (RTL aborts the
  milestone), which the waiter must report without calling it a failure of
  its own command.
- Daemon lock semantics: adds a second synchronization axis (priority) to
  a lock that Phase 2 deliberately kept single-axis — significant
  complexity and review surface.
- Adapter impact: none for the effecting command; the wait is daemon-side.
- Failure mode: priority inversion bugs (a priority command arriving mid-
  effecting-command), thundering-herd of waiters, and the temptation to
  preempt the *vehicle-side* mission (which the vehicle only supports
  through mode change or `DO_PAUSE_CONTINUE` — see E.1).

### E.3 Recommendation

`[OPEN]` → leaning `[DECIDED]` **Option 1 (operation model)**, deferring
the final call to the implementation round's review:

- it removes the actual hazard (blocked state-changing commands) with the
  smallest lock-semantics change;
- it composes with the existing `status`-style lock-free observation path;
- preemption of the *vehicle* is already available where it matters
  (`mode RTL/land`, `disarm --force`) and never needs mavctl-side wait
  cancellation;
- Issue #21's acceptance (urgent `rtl --confirm` acts immediately) is met
  because no command queues behind a wait anymore.

If Option 1 is adopted, Issue #21 can be closed *by* the Phase 3B
implementation (its scope generalizes to all `--wait` commands, not just
mission start — the implementation must decide whether to convert existing
`--wait`s in the same round or ship mission-start-first).

### E.4 Relationship to future pause/resume/stop

Vehicle surface exists (`MAV_CMD_DO_PAUSE_CONTINUE`, §B.6; `MISSION_SET_CURRENT`
repositioning also exists). A paused mission (`mission_state` stays ACTIVE,
seq frozen) is the natural pre-emption state for "stop and hold". Phase 3B
v1 ships none of these; the operation model (E.1) leaves room: operations
can later carry pause/resume actions targeting the same observation
machinery.

### E.5 Mission protocol safety under interruption

`[DECIDED]`: mavctl never preempts a running mission by touching mission
protocol messages (no CLEAR/SET_CURRENT while running). Interruption is a
*flight-mode* action (mode change out of AUTO), which the vehicle handles;
the mission stays stored and resumable (`MIS_RESTART` semantics apply,
SITL-verify).

---

## F. Safety model

- `mission start` requires `--confirm` (`[DECIDED]`); it is an **execution
  command that may transition the vehicle to AUTO on ArduCopter** — that
  transition is explicit, confirmation-gated mission execution behavior,
  not a hidden side effect (§C.1) (`[DECIDED]`).
- Mode change and mission start must not be modeled as independently
  ordered universal operations: on ArduCopter, entering AUTO alone can have
  mission start/resume semantics (§B.2), and `MAV_CMD_MISSION_START`
  includes the AUTO transition (§B.1) (`[FACT]` + `[DECIDED]`).
- No implicit arm, no implicit takeoff (`[DECIDED]`); `set_auto_armed(true)`
  is vehicle-internal flight-mode arming, not motor arming — documented to
  avoid agent confusion.
- Stale link / stale telemetry / unknown ground evidence block the start
  exactly like Phase 3A guards (`[DECIDED]`).
- Mission progress observation is **not** proof of physical safety —
  `mission_state ACTIVE` says the autopilot is executing items, nothing
  about airspace, battery, or EKF health (Issue #4 remains the vehicle-side
  authority plus its own pre-arm checks) (`[DECIDED]`).
- RTL / land safety path must remain available at all times — the operation
  model (E.1) guarantees no queueing behind waits (`[DECIDED]` if E.1
  adopted; otherwise a Phase 3B blocker).
- Real-aircraft execution remains out of scope (`[NON-GOAL]`).
- Initial Phase 3B implementation is SITL-only (`[DECIDED]`).

---

## G. Testing and SITL acceptance plan

Unit / mock layers:

- **A. models**: mission-progress state model (mapping
  `MISSION_STATE_*` → labels, `unknown`/`null` semantics, ages namespace).
- **B. mock adapter**: `MISSION_CURRENT` / `MISSION_ITEM_REACHED` routing
  into a progress snapshot (not the transaction inbox), sequence-gaps,
  stale progress, duplicate messages; `MAV_CMD_MISSION_START` ack handling
  (ACCEPTED / DENIED param1/2 / FAILED).
- **C. daemon/CLI contract**: guard chain (confirm/link/disarmed-policy/
  count>0 via read-back/mode policy), `--wait` milestone loop, exit-code
  mapping (2/4/5/6), idempotent already-running, JSON/human shapes.
- **D. operation/interruption**: operation lifecycle (registered →
  milestone/deadline/superseded), `rtl` running *during* a mission-start
  wait, two concurrent starts, link-loss mid-wait, wait outcome after mode
  left AUTO.

SITL acceptance (all on loopback; the flight steps require the operator's
explicit SITL arming — none of this runs in the design round):

1. upload mission (Phase 3A command);
2. verify disarmed / fresh state / mission count via read-back;
3. **arm and switch AUTO manually outside `mission start`** (operator
   commands; establishes the C.1 policy baseline);
4. `mission start`;
5. observe `MISSION_CURRENT` / progress in `status --json`;
6. verify `--wait` milestone completes (ack + AUTO + ACTIVE);
7. verify mission ends via its terminal item (RTL/land item) —
   `mission_state COMPLETE`;
8. RTL/land interruption of a running mission **only after the Issue #21
   solution is implemented**;
9. cleanup mission (`mission clear`, verified read-back).

---

## H. Open questions

1. `MAV_CMD_MISSION_START` exact behavior when the vehicle is **disarmed**
   (logical start vs refusal; physical inertness until arming).
2. Behavior when the vehicle is **AUTO + NOT_STARTED** — send
   `MAV_CMD_MISSION_START` (start-or-resume) or refuse with a hint
   (§C.1.2).
3. `MIS_RESTART` effect on `MAV_CMD_MISSION_START` / `start_or_resume`
   (start-vs-resume semantics) — SITL-verify.
4. Timing/availability of `MISSION_CURRENT` fields (`mission_state`,
   `total`, `mission_mode`) in ArduCopter SITL — reliability of the §C.2
   milestone elements; downgrade to `[OPEN]` if unreliable.
5. Whether `MISSION_CURRENT` requires an explicit message-interval request
   on a bare mavctl link (like Phase 3A's position/home streams).
6. Operation model vs cancellation/preemption for Issue #21 — adopt E.1?
   (and does the same round convert all existing `--wait`s, or
   mission-start only?)
7. Relationship with `MAV_CMD_DO_PAUSE_CONTINUE` — catalogue only in v1;
   pause/resume commands later (§B.6, §E.4).
8. Mission execution across daemon reconnect: progress snapshot rebuild
   (re-subscribe + fresh `MISSION_CURRENT`) and what `--wait` means if the
   mission was already running before mavctl connected.
9. `MISSION_ITEM_REACHED` availability on Copter; whether it becomes a
   future enhancement / non-Copter support consideration (§B.4).
10. Mission completion detection: `mission_state COMPLETE` alone, or
    seq == last-nav-item + landed evidence — pick in implementation.
11. Interaction between a mission RTL item and a user-issued `mavctl rtl`
    (mode change mid-mission): document the vehicle behavior, define what
    mavctl reports.
12. Whether Phase 3B ships as `0.4.0` (feature release) — version decision
    for the release round, per PUBLISHING guidance (next dev version moves
    forward from 0.3.0).

## Related documents

- docs/design/mission-protocol-v1.md (Phase 3A protocol core, §D.0.1
  settle quarantine)
- docs/design/product-architecture-roadmap.md (Phase roadmap)
- docs/PUBLISHING.md (`Production release record: 0.3.0`)
