# Design: Long-Running Operation Interruption (Issue #21)

Status: **design only** — nothing in this document is implemented. Tracking
issue: #21. Companion to `docs/design/mission-execution-phase3b.md` (§E
analysed the same problem from the Phase 3B side; this document is the
authoritative deep dive).

Classification discipline: `[FACT]` (verified against current code /
sources), `[DECIDED]` (mavctl product decision, evidence-sufficient),
`[OPEN]` (unresolved), `[NON-GOAL]` (excluded).

---

## A. Current behavior / problem

### A.1 The lock and its real scope

`[FACT]` The daemon serializes state-changing commands with a single
`asyncio.Lock` (`_command_lock`, server.py). Handlers that take it for
their **entire body**:

| Handler | Lock held through | `--wait`? |
| --- | --- | --- |
| `mission_upload` | guard + full wire transaction (up to the 15 s mission transaction timeout) | no |
| `mission_clear` | guard + wire transaction + count read-back (same bound) | no |
| `mission_download` | **no lock** (deliberately read-only) | n/a |
| `arm` / `disarm` | guard + single effecting command | **no** |
| `mode` | guard + effecting command + optional wait | **yes** |
| `takeoff` | guard + effecting command + optional wait | **yes** |
| `land` / `rtl` | guard + effecting command + optional wait (shared `_m_descent`) | **yes** |

`[FACT]` The `--wait` implementation (`_maybe_wait`) polls every 0.25 s
**inside** the lock; each iteration re-checks link liveness (heartbeat
stale mid-wait → `LINK_LOST`, exit 4) and the target predicate
(e.g. `_reached_altitude`).

`[FACT]` `status` / `telemetry` never take `_command_lock` — they read the
adapter snapshot directly and stay responsive during any transaction.

### A.2 The problem

`[FACT]` While any of the `--wait` commands (or a mission upload/clear) is
inside its lock window, every other state-changing command **queues**:

```text
takeoff --alt 50 --confirm --wait --timeout 60
→ concurrent `rtl --confirm` waits behind it (up to the remainder of the
  original timeout)
```

This is a **safety/UX design limitation**, not a reported incident: the
queued RTL still executes after the wait expires, but the delay is exactly
what an emergency action must not have. Phase 3B's `mission start --wait`
would add a new long-window command and amplify this.

### A.3 Why the wait cannot simply move outside the lock

`[FACT]` The Phase 2 design holds the lock across guard → effect → wait so
that: (1) no second command can interleave between guard validation and
the effecting command; (2) no second command acts on stale state during
another command's wait window; (3) the guard's vehicle snapshot cannot
drift before the effect lands. Releasing the lock at the `--wait` boundary
would allow a queued `mode`/`takeoff` to interleave *during* the first
command's wait — reintroducing the exact TOCTOU the serialization was
built to prevent. Any solution must preserve these invariants (§B).

---

## B. Requirements

1. Preserve command transaction serialization (guard → effect ordering;
   no interleaving of effecting commands).
2. `status` / `telemetry` remain responsive (unchanged).
3. Long operations must be **observable** — a client/agent can query what
   the daemon is doing and how far it has progressed.
4. Client waiting is **bounded** (`--timeout` semantics survive).
5. RTL/land must become available during another command's wait window
   **without unsafe concurrent effecting-command overlap**.
6. No implicit `disarm --force` preemption.
7. No raw MAVLink escape hatch.
8. Works for the future Phase 3B `mission start --wait`.
9. Compatible with the CLI-first contract today and a future Harness.
10. No real-aircraft claim; SITL-only validation when implemented.

---

## C. Candidate models

### C.1 Option 1 — current synchronous lock model (status quo)

The command + `--wait` hold `_command_lock` end-to-end.

- **Pros**: trivially correct ordering; no new state; guards always see a
  quiescent vehicle snapshot; one client per transition by construction.
- **Cons**: the wait window blocks all state-changing commands (A.2);
  unbounded growth as more long-window commands appear; RTL/land queue —
  unacceptable for Phase 3B.
- **Why it is not enough**: it cannot satisfy §B.5 while keeping
  `mission start --wait`.

### C.2 Option 2 — daemon operation model

A state-changing command creates a **daemon-owned operation**: the
effecting command runs under `_command_lock` as today; the subsequent
passive wait runs as a daemon-side operation that the client may observe
or wait on. Candidate interface (candidates only — **nothing is
implemented and nothing is promised for a specific release**;
`operation cancel` is additionally `[OPEN]` — see §G):

```bash
mavctl operation get <id>     # observe: state, kind, elapsed, last evidence
```

Analysis dimensions:

- **Client disconnect**: the operation **continues** (the vehicle command
  was already sent and accepted); a reconnecting client queries its
  outcome. Disconnect ≠ cancellation (`[DECIDED]`, §C.2.1-F).
- **Daemon restart**: in-memory operations die with the daemon. The
  vehicle-side truth survives (e.g. an accepted takeoff keeps climbing) —
  a restarted daemon re-establishes state from the vehicle snapshot;
  prior operations become `uncertain`/unknown. Persistent operation
  journals are `[NON-GOAL]` for v1 (§J).
- **Operation persistence**: none in v1 beyond the process lifetime.
- **Lock release checkpoint**: precisely defined — see §C.2.1-C.
- **Completion observation**: milestone predicate evaluated by the daemon
  loop outside the lock (same predicates as today: altitude reached,
  landed, mode confirmed, `MISSION_CURRENT` ACTIVE); clients poll
  `operation get` / `status --json`.
- **Implications for `mission start`**: the Phase 3B milestone (ack
  ACCEPTED + mode AUTO + `MISSION_CURRENT` ACTIVE, §C.2 of the Phase 3B
  design) maps directly onto an operation with three milestone predicates;
  RTL availability during the wait is then free (E.1 of that doc).

#### C.2.1 Operation ownership, epoch, and linearization (`[DECIDED]` v1 design)

**A. One active observation operation per vehicle** — **not** per kind.

Each daemon/vehicle pair has **at most one active passive-observation
operation** at any time. Examples: `takeoff --wait`, `land --wait`,
`rtl --wait`, future `mission start --wait`. When a new operation becomes
the active owner, the previous active operation loses write access to its
outcome (§C.2.1-E) and can never report `reached`/success again.

Rationale: long-operation *observation* semantics on one vehicle cannot be
split by command kind — `takeoff --wait` and `rtl --wait` observe the same
vehicle state, and per-kind ownership would allow two operations to make
contradictory success claims about that state simultaneously. v1 chooses
the simple, auditable model: a single active owner per vehicle.

**B. Operation record (candidate fields)**:

```text
operation_id                     # CLI/API-visible
epoch                            # daemon-internal generation, not exposed
kind                             # takeoff | land | rtl | mode | mission_start
state                            # §F state machine
created_monotonic
effect_sent_monotonic
ack_monotonic                    # set when COMMAND_ACK is consumed
superseded_by_operation_id       # optional, set on supersession
terminal_reason                  # optional, human-readable
```

`operation_id` is visible to CLI/API (outcome JSON + `operation get`).
`epoch` is a daemon-internal generation counter — it exists so that a
*stale passive waiter* (one whose operation has since been superseded) can
detect that it no longer owns the observation, even if operation ids were
reused or a waiter races the registry. **Rule**: an operation's status may
only be advanced by an observer holding the matching
`operation_id + epoch`.

**C. Linearization / command-lock checkpoint** — `[DECIDED]`, replacing
the earlier "bounded post-command checkpoint (e.g. first predicate
evaluation)" idea, which is **withdrawn**:

```text
under _command_lock:
  guard passed
  → effecting MAVLink command sent
  → accepted COMMAND_ACK received
  → operation record atomically registered / activated in the registry
    (this atomically supersedes any prior active operation)
→ release _command_lock
→ begin passive observation OUTSIDE the lock
```

- "First predicate evaluation" is **not** a checkpoint — predicates are
  evaluated outside the lock and are fenced by epoch (§C.2.1-E).
- guard → effect produces no TOCTOU: both stay inside the lock, exactly as
  in the Phase 2 model.
- operation registration and active-owner replacement happen **in the same
  critical section** as the ACK consumption — supersession is therefore
  linearized with command acceptance.
- `status` / `telemetry` never take the command lock (unchanged).
- Passive observation must **never** re-acquire the command lock for its
  duration; it reads the adapter snapshot only.

**D. Supersession semantics for RTL / land** — `[DECIDED]`:

When a new effecting command is **accepted and registered** as the active
observation operation, any prior active observation operation becomes
`superseded`.

- `superseded` ≠ "vehicle command cancelled". It means exactly: **mavctl
  no longer treats the old operation as the current observation target.**
- RTL/land are **not magical cancellation commands**: they go through the
  same path as every effecting command — they require `--confirm`, they
  acquire `_command_lock`, they **re-read the latest vehicle state and
  re-run their guards** (fresh link, safety evidence — nothing skipped),
  and only after their own ACK do they replace the active observation
  operation.
- The old waiter must **never** later report `reached`/success (fenced by
  §C.2.1-E). The old client sees the terminal local state `superseded` and
  is explicitly told the **vehicle state must be re-queried** (the new
  operation id is referenced in the superseded outcome's
  `superseded_by_operation_id`).
- `disarm --force` gains **no** automatic preemption rights — it is an
  ordinary serialized command (`[DECIDED]`, unchanged from §E).

**E. Passive waiter update rule** — pseudocode-level constraint that all
waiters (takeoff, land, rtl, future `mission start`) must satisfy:

```text
before a waiter records reached / timed_out / link_lost:
    if registry.active_operation_id != operation_id
       or registry.active_epoch != epoch:
        do NOT write reached / timed_out / success
        terminal state = superseded (observation invalidated)
    else:
        record normally
```

This fences the classic stale-waiter race: an old waiter that wakes up
after RTL/land replaced it can no longer write a success verdict over the
new operation's observation.

**F. Client timeout / disconnect / daemon restart** — `[DECIDED]` v1
semantics, kept deliberately distinct (never one shared "uncertain" bucket
for all three):

- **CLI `--wait` timeout**: the client exits 6; the **daemon operation
  continues**; the response states `operation_still_running = true` and
  includes the `operation_id`.
- **Client disconnect**: does **not** cancel the daemon operation.
- **Daemon restart**: the in-memory operation registry is lost; prior
  operations become **unknown/uncertain**; the new daemon requires fresh
  status/progress observation; no operation persistence in v1.

These map onto four **distinct terminal reasons** (evidence levels in
§F): `link_lost` = daemon observed heartbeat loss; `timed_out` =
observation deadline elapsed (while still the active owner); `superseded`
= daemon-local active-owner replacement; `uncertain` = the vehicle
effect/state cannot be established after daemon restart or another
evidence-loss condition.

### C.3 Option 3 — priority/cancellable wait inside the command lock

RTL/land request signals cancellation of the in-flight wait; the current
command transitions to `cancelled/uncertain`; the lock is handed over; the
priority command executes.

- **Lock handoff**: requires cancel-flag + condition-variable handoff
  *inside* `_command_lock` — a second synchronization axis (priority) on a
  lock deliberately kept single-axis in Phase 2.
- **Race conditions**: the cancel signal can arrive between guard-pass and
  effect-send; between effect-send and ACK; or after ACK during the wait.
  Each phase needs a defined outcome, and the waiter must distinguish
  "cancelled before action" (never claimable without evidence — §E) from
  "vehicle already acting".
- **If takeoff is active while RTL arrives**: the vehicle has already
  accepted the takeoff — motors are spinning. The "interruption" is really
  *queue-jumping the effecting command*, which is safe only because RTL is
  a mode change the vehicle serializes internally — but then Option 2
  achieves the same outcome with no preemption machinery.
- **Why `disarm --force` must not automatically preempt**: force-disarm is
  a direct motor-stop whose effecting command is already ~ms; coupling it
  to preemption bookkeeping adds an emergency-path failure mode for no
  latency gain (§E).
- **Complexity vs operation model**: strictly more complex (priority
  arbitration + phase-aware cancel semantics) for the same observable
  benefit.

### C.4 Comparison summary

| Dimension | O1 status quo | O2 operation model | O3 priority cancel |
| --- | --- | --- | --- |
| Blocks state-changing commands during wait | yes | **no** (lock released at ACK + registry activation) | no (after handoff) |
| New lock axes | — | none (wait leaves the lock; epoch fencing, no priority axis) | priority axis |
| Race surface | — | small (operation registry + epoch fencing) | phase-aware cancel matrix |
| Observability | none | operation id + epoch + state | partial |
| Client disconnect | ambiguous | defined (operation continues) | needs defined outcome |
| Stale-waiter protection | n/a | epoch fencing (superseded cannot write reached) | cancel-phase matrix |
| Fits `mission start --wait` | no | **yes** | yes |
| Complexity | — | moderate | high |

---

## D. Recommended direction

`[DECIDED]` **Option 2 — daemon operation model with one active
observation owner per vehicle and epoch fencing** (ownership, checkpoint,
supersession, and client semantics per §C.2.1). The previously open
checkpoint question is resolved: the command lock is released immediately
after `accepted ACK + atomic operation registry activation`, and predicate
evaluation happens outside the lock, fenced by `operation_id + epoch`.

- `--wait` means the *client* waits for a bounded milestone (or
  `operation get` polling for non-blocking agents); the daemon never holds
  the command lock for passive observation.
- State-changing transitions remain serialized; RTL/land supersede the
  active observation operation only after their own guarded, ACKed effect
  (§C.2.1-D).
- Rationale for preferring this over Option 3: same observable benefit,
  strictly smaller synchronization surface (no priority axis; fencing by
  epoch instead), and it generalizes to `mission start --wait` without
  per-command priority rules.

Remaining `[OPEN]` implementation decisions (none of them affects the
ownership/linearization model above): the `operation cancel` surface
(§G), operation id format (§J), which commands convert in the first
round, and Harness consumption (§J).

---

## E. Safety semantics (candidate rules)

- **takeoff `--wait` interrupted by RTL** (post-implementation): the
  takeoff command may already have been accepted by the vehicle — the
  operation result must **never** be reported as "cancelled before action"
  without direct evidence; the honest outcome is `superseded` (§C.2.1-D/E
  fencing: a superseded waiter cannot later write `reached`), with the
  vehicle state re-queried via the superseding operation
  (`[DECIDED]`).
- **land / rtl** are the candidates for safety-priority actions: they still
  require `--confirm`, and they must **re-check current vehicle state**
  (guards) before send — priority in scheduling never skips the guard
  chain (`[DECIDED]` candidate).
- **disarm --force**: never automatic priority; explicit user command only
  (`[DECIDED]` — matches Issue #21's original constraint).
- **mission start** (Phase 3B): integrates through the same operation
  model; its AUTO-transition semantics are per the Phase 3B design (§C.1
  there) (`[DECIDED]` candidate).

---

## F. Proposed operation state model

Candidate states (none implemented):

```text
pending → command_sent → accepted → waiting → reached
                                     ↘ timed_out
              (any state) ↘ link_lost
accepted/waiting ↘ superseded        (active-owner replacement, §C.2.1-D)
any post-send state ↘ uncertain      (restart / evidence loss)
guard/effect failure ↘ failed
```

Evidence classification:

| State | Classification |
| --- | --- |
| `pending` | daemon-local only |
| `command_sent` | daemon-local (send attempt made; no vehicle confirmation yet) |
| `accepted` | **vehicle-confirmed** (COMMAND_ACK) |
| `waiting`, `reached`, `timed_out` | daemon-local observation **of vehicle-confirmed predicates** (mode/altitude/`MISSION_CURRENT` from the vehicle snapshot) — writes fenced by `operation_id + epoch` (§C.2.1-E) |
| `link_lost` | daemon-local observation of heartbeat loss **while still the active owner** (fenced like every predicate write) |
| `superseded` | daemon-local active-owner replacement — **not** a vehicle-confirmed cancellation; vehicle state must be re-queried |
| `uncertain` | honest composite: an effect may have happened, but the final vehicle state cannot be established (daemon restart / evidence loss) |
| `failed` | vehicle-confirmed (NACK) or guard-local |

`[DECIDED]` rules: (1) every predicate write is fenced by matching
`operation_id + epoch` (§C.2.1-E); (2) any state transition that cannot
cite a vehicle-confirmed predicate must not be reported as a vehicle
outcome — it stays daemon-local or becomes `uncertain`; (3) `link_lost`,
`timed_out`, `superseded`, and `uncertain` are **distinct** terminal
reasons and are never conflated into one bucket.

---

## G. CLI/API design candidates (candidates only)

- `--wait` behavior (`[DECIDED]` candidate): client blocks on the
  operation until milestone, deadline, or supersession; without `--wait`
  the command returns immediately after the effecting command with the
  operation id attached to the outcome (candidate field).
- `--timeout` behavior (`[DECIDED]`): unchanged semantics — bounds the
  *client* wait; the daemon operation outlives a client timeout
  (§C.2.1-F).
- **Client timeout vs daemon operation** (the key honesty rule,
  `[DECIDED]`): if the client's `--timeout` expires but the operation is
  still waiting, the CLI returns exit 6 with
  `operation_still_running = true` + operation id + hint to
  `operation get` — an agent must **never** read "timeout" as "the vehicle
  action was cancelled" (the action was ACKed long before).
- Operation id exposure: in JSON outcome + human line for every
  `--wait`-capable command (candidate).
- `operation get <id>`: **candidate surface**, no implementation implied.
- `operation cancel <id>`: **`[OPEN]`** — the cancel surface is explicitly
  not designed to v1 completeness yet (request-level wait-stop per §J.2);
  it must not appear as an existing command anywhere.
- Human vs JSON: `operation get` mirrors `status --json` conventions
  (structured JSON vs table line).
- Exit-code mapping candidates: effecting-command failures keep today's
  2/4/5/6; observation-only outcomes introduce no new exit codes —
  `superseded`/`still_running` report under 6 with distinct reasons
  (`[OPEN]` whether `superseded` deserves its own code).

---

## H. Test plan (future implementation)

- **Operation lifecycle mock tests**: full state-machine walks, single
  active owner per vehicle, epoch bump on every supersession, id
  stability, JSON shapes.
- **Epoch fencing**: a superseded waiter that wakes up and observes its
  predicate satisfied must NOT write `reached` — it stays `superseded`
  (the §C.2.1-E rule exercised for takeoff/land/rtl/mission-start
  waiters).
- **takeoff wait + RTL request**: RTL executes without waiting out the
  takeoff milestone; takeoff operation ends `superseded`/`uncertain`,
  never "cancelled before action".
- **takeoff wait + status still live**: `status`/`telemetry` latencies
  unchanged during an operation.
- **Link loss during operation**: `link_lost` outcome; no phantom
  milestone after reconnect.
- **Client disconnect**: operation continues; reconnect observes outcome.
- **Daemon restart**: operation reported uncertain/lost; vehicle snapshot
  re-established; no stale operation resurrected.
- **`mission start` ACTIVE milestone**: Phase 3B milestone predicates
  through the same machinery.
- **No auto force-disarm**: force-disarm gains no preemption rights; runs
  serialized like any command.
- **Deadlock / lock-order tests**: `_command_lock` hold-time bounds per
  command kind; no path holds `_command_lock` while acquiring `_mission_lock`
  and vice versa in a circular order.
- **SITL-only execution tests** after mock acceptance (loopback; per the
  standing SITL discipline).

---

## I. Relationship to Phase 3B

- **Phase 3B runtime must not start** until this design has an explicit
  adopted decision and the chosen model's implementation plan
  (`[DECIDED]` — matches the Phase 3B design's own constraint).
- `mission start --wait` **reuses** this model — the same registry, epoch
  fencing, and supersession rules; it must not build a parallel
  observation mechanism (`[DECIDED]` candidate).
- Phase 3A `mission upload/download/clear` do **not** need refactoring
  into operations for v1 — their lock windows are wire-transaction-bounded
  (15 s) and safety-critical ordering is exactly why they serialize;
  revisit only if implementation shows agent workflows need mid-transfer
  observation (`[OPEN]`).
- `MISSION_CURRENT` progress (Phase 3B design §D) is the natural future
  observation source for mission-kind operations (`[DECIDED]` candidate).

---

## J. Open questions

1. Operation persistence across daemon restart — in-memory
   `unknown/uncertain` on restart is the v1 decision; any durable journal
   is a later question (leaning no).
2. `operation cancel <id>` surface — `[OPEN]`: request-level (stop the
   *wait*) is the leaning; vehicle-level actions remain the normal
   commands (`rtl`, `land`, `disarm --force`). Exact cancel semantics,
   authorization, and interaction with supersession are undesignated.
3. `mission start` AUTO-transition timing — the lock is released at ACK +
   registry activation; the vehicle's own AUTO transition continues
   vehicle-side afterwards (milestone ②/③ observe it). Confirmed
   consistent with §C.2.1-C; SITL will verify observable ordering.
4. RTL/land priority guarantees — with the checkpoint fixed at ACK +
   registry activation, RTL/land always acquire the lock immediately after
   the current effecting command's ACK; whether they must also preempt
   *not-yet-sent* queued commands is `[OPEN]` (queue is FIFO today).
5. Output contract when the client timeout expires first — exact JSON
   shape of `operation_still_running` and how agents should consume it.
6. How a future Harness consumes operations (polling vs subscribe; does
   the daemon need an operation-event stream?).
7. Are operation ids stable enough for logs/audit (uuid4 per operation vs
   monotonic per-vehicle counters)?
8. Scope: which existing commands become operations in the first
   implementation round — all `--wait` commands at once, or
   `mission start` first with the rest converted incrementally?

## Related documents

- docs/design/mission-execution-phase3b.md (§E analysed the same problem;
  §C.2/§D define the milestone predicates and progress model that map onto
  operations)
- docs/design/mission-protocol-v1.md (§H lock model; §D.0.1 settle
  quarantine)
- docs/design/product-architecture-roadmap.md (issue standings)
