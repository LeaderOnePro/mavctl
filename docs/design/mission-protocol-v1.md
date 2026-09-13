# Design: mavctl mission protocol v1 (Phase 3A)

Status: **implementation-grade design — no mission runtime code exists yet.**
Tracking issue: [#25 feat(mission): add MAVLink mission upload, download, and clear](https://github.com/LeaderOnePro/mavctl/issues/25).
Roadmap context: `docs/design/product-architecture-roadmap.md` (Phase 3A;
mission execution is Phase 3B and is out of scope here).

Research revision: 2026-09-12. This revision deepens the original draft with
source-verified upload/download state-machine behavior, an explicit lock
model, and evidence-backed field encodings. MAVProxy was **not available**
locally (import checked) and is therefore not used as a source; where only
SITL can settle a question the statement is marked `[OPEN]`.

Every statement below is classified as exactly one of:

```text
[FACT]      Directly verified from a local, readable source: the installed
            pymavlink common dialect (generated code + bundled common.xml),
            or the local ArduPilot checkout (libraries/GCS_MAVLink/,
            ArduCopter/). File/function cited inline.
[DECIDED]   mavctl v1 product behavior chosen by this design.
[OPEN]      Not yet reliably verifiable from local sources; must be settled
            by SITL or a later decision. Listed in §J.
[NON-GOAL]  Explicitly out of scope.
```

---

## A. Protocol facts verification

### A.1 Messages and wire IDs

Source: installed `pymavlink/dialects/v20/common.xml` (bundled with
pymavlink 2.4.49) and the generated `common.py`. `[FACT]` all rows:

| Message | Wire ID | Direction | Core fields (type) | Extension fields |
| --- | --- | --- | --- | --- |
| `MISSION_REQUEST_LIST` | 43 | GCS → vehicle | target_system u8, target_component u8, mission_type u8 | — |
| `MISSION_COUNT` | 44 | vehicle → GCS (download reply) **and** GCS → vehicle (upload start) | target_system u8, target_component u8, count u16 | `mission_type` |
| `MISSION_CLEAR_ALL` | 45 | GCS → vehicle | target_system u8, target_component u8 | `mission_type` |
| `MISSION_ITEM_REACHED` | 46 | vehicle → GCS | seq u16 | — |
| `MISSION_ACK` | 47 | vehicle → GCS (upload/clear terminal) **and** GCS → vehicle (download terminal, courtesy — see §E) | target_system u8, target_component u8, type u8 (MAV_MISSION_RESULT) | `mission_type` |
| `MISSION_CURRENT` | 42 | vehicle → GCS | seq u16 | `total`, `mission_state`, `mission_mode` |
| `MISSION_REQUEST_INT` | 51 | vehicle → GCS (upload pacing) **and** GCS → vehicle (download fetch) | target_system u8, target_component u8, seq u16 | `mission_type` |
| `MISSION_ITEM_INT` | 73 | GCS → vehicle (upload) **and** vehicle → GCS (download) | target_system u8, target_component u8, seq u16, frame u8 (MAV_FRAME), command u16 (MAV_CMD), current u8, autocontinue u8, param1..4 f32, x i32, y i32, z f32 | `mission_type` |

`[FACT]` `mission_type` is an **extension field** on every mission message
that carries it, and `total`/`mission_state`/`mission_mode` are extension
fields on `MISSION_CURRENT` (bundled `common.xml`, `<extensions/>` markers).
`[FACT]` pymavlink `MISSION_ITEM_INT` doc: `seq` "starts at zero, increases
monotonically, no gaps"; `current` 0/1; NaN (float) / INT32_MAX (int32) may
indicate optional/default values.

Practical consequence `[DECIDED]`: mavctl always speaks MAVLink v2
(pymavlink default for modern vehicles) and always populates `mission_type`;
a missing `mission_type` (v1 peer zero-fill) is treated as
`MAV_MISSION_TYPE_MISSION` (=0) for tolerance, but an explicit different
value is a mismatch (§D/E).

### A.2 Enums

`[FACT]` `MAV_MISSION_TYPE`: `MISSION=0`, `FENCE=1`, `RALLY=2`, `ALL=255`.
`[FACT]` `MAV_MISSION_RESULT`: `ACCEPTED=0`, `ERROR=1`,
`UNSUPPORTED_FRAME=2`, `UNSUPPORTED=3`, `NO_SPACE=4`, `INVALID=5`,
`INVALID_PARAM1..4=6..9`, `INVALID_PARAM5_X=10`, `INVALID_PARAM6_Y=11`,
`INVALID_PARAM7=12`, `INVALID_SEQUENCE=13`, `DENIED=14`,
`OPERATION_CANCELLED=15`. `[FACT]` Frames: `MAV_FRAME_GLOBAL=0`,
`MAV_FRAME_GLOBAL_RELATIVE_ALT=3`, `MAV_FRAME_GLOBAL_INT=5`,
`MAV_FRAME_GLOBAL_RELATIVE_ALT_INT=6`.

### A.3 Commands

`[FACT]` `MAV_CMD_NAV_WAYPOINT=16`, `MAV_CMD_NAV_RETURN_TO_LAUNCH=20`,
`MAV_CMD_NAV_LAND=21`, `MAV_CMD_NAV_TAKEOFF=22`, `MAV_CMD_MISSION_START=300`.
`[FACT]` **No `MAV_CMD_MISSION_CLEAR_ALL` / `MAV_CMD_MISSION_CLEAR` exists**
in the common dialect (zero occurrences) — clearing uses the
`MISSION_CLEAR_ALL` **message** (§F).

### A.4 Command parameter semantics

Source: bundled `common.xml`, `MAV_CMD` entries. `[FACT]` all rows:

| Command | param1 | param2 | param3 | param4 | param5/6 | param7 |
| --- | --- | --- | --- | --- | --- | --- |
| `NAV_TAKEOFF` (22) | Minimum pitch (if airspeed sensor present), desired pitch without sensor | Empty | Empty | Yaw; NaN = current heading mode | Latitude / Longitude | Altitude |
| `NAV_WAYPOINT` (16) | Hold time (s; ignored by fixed wing, time to stay for rotary wing) | Acceptance radius (m; sphere hit = reached) | 0 = pass through WP; >0 radius to pass by | Desired yaw; NaN = current heading mode | Latitude / Longitude | Altitude |
| `NAV_LAND` (21) | Minimum target altitude if landing is aborted (**0 = undefined / use system default**) | Precision land mode | Empty | Desired yaw; NaN = current heading mode | Latitude / Longitude | Landing altitude (ground level in current frame) |
| `NAV_RETURN_TO_LAUNCH` (20) | Empty | Empty | Empty | Empty | Empty | Empty |

### A.5 ArduPilot upload/download behavior (source-verified)

Source: `libraries/GCS_MAVLink/MissionItemProtocol.cpp` /
`MissionItemProtocol.h` / `GCS_Common.cpp`; `ArduCopter/mode_auto.cpp`.
`[FACT]` all bullets in this subsection unless noted:

- **Upload start** (`handle_mission_count`): gated on
  `mavlink2_requirement_met` — a MAVLink v1 sender is **silently ignored
  (no ACK)**; `mission_type` without a matching protocol handler → immediate
  `MISSION_ACK(MAV_MISSION_UNSUPPORTED)`; `count > max_items()` →
  `MISSION_ACK(MAV_MISSION_NO_SPACE)`; a repeated `MISSION_COUNT` **cancels
  any in-progress upload and restarts** (`cancel_upload` then re-init);
  `count == 0` → `transfer_is_complete` **immediately** (terminal ACCEPTED
  ACK, no item requests).
- **Item reception** (`handle_mission_item`): **strict in-order** —
  `cmd.seq != request_i` → `MISSION_ACK(MAV_MISSION_INVALID_SEQUENCE)` while
  the session **stays alive** (`request_i` unchanged, still waiting for the
  expected seq); source binding — item from any system/component other than
  the uploader (`dest_sysid`/`dest_compid`) →
  `MISSION_ACK(MAV_MISSION_DENIED)`; accepted item → `request_i++` and the
  **next `MISSION_REQUEST_INT` is sent immediately** (or queued if no buffer
  space); after the **last** item (`request_i > request_last`) →
  `transfer_is_complete` → `complete()` → terminal `MISSION_ACK` is sent
  **immediately after processing the last item**; a replace/append storage
  failure → `MISSION_ACK(result)` and the session **ends**
  (`receiving = false`).
- **Vehicle-side upload timeout**: `upload_timeout_ms = 8000` — if no item
  arrives within 8 s the vehicle cancels the session and sends
  `MISSION_ACK(MAV_MISSION_OPERATION_CANCELLED)` to the uploader.
- **Download start** (`handle_mission_request_list`): while an upload is
  being received, `MISSION_REQUEST_LIST` is answered
  `MISSION_ACK(MAV_MISSION_DENIED)` — a download cannot run concurrently
  with an upload on the vehicle.
- **Received `MISSION_ACK`**: the vehicle's dispatch contains
  `case MAVLINK_MSG_ID_MISSION_ACK: /* not used */ break;` — ArduPilot does
  **not** consume the GCS's download-terminal ACK.
- **Copter item semantics**: `ModeAuto::do_takeoff` passes
  `cmd.content.location` to `takeoff_start`; the shared
  `get_loc_from_cmd` helper documents and implements **"use current lat,
  lon if zero"** (`loc.lat == 0 && loc.lng == 0` → default/current
  position); `ModeAuto::do_RTL(void)` takes **no location at all** — RTL
  mission items ignore x/y/z entirely.

`[OPEN]` Exact request-retry behavior of the vehicle at the transport layer
(queued ap-message re-sends) and whether re-requests of the *same* seq occur
in practice — mavctl must tolerate duplicates regardless (§D).
`[OPEN]` Whether `accept_radius_m = 0` selects the vehicle's `WP_RADIUS`
parameter default on ArduCopter — the XML defines the radius meaning but not
the zero-value behavior; settle in SITL (§C/§J).

---

## B. Exact CLI contract

All three commands require a running daemon (exit 3 otherwise). **B.4**
defines the two distinct pre-execution paths; only `upload`/`clear` run the
state-changing guard path.

### B.1 `mavctl mission upload <mission.json> --confirm [--dry-run]`

| Aspect | Design |
| --- | --- |
| Required | positional `mission.json` (must be an existing file), `--confirm` |
| Optional | `--dry-run`, `--json` |
| Pre-daemon | CLI reads and **schema-validates the file locally** (pydantic schema, §C); schema failure → exit 2 before any daemon contact. The daemon re-validates authoritatively (defense in depth). |
| Human output (success) | `mission uploaded: N items accepted` (+ duration); on `--dry-run`: `[dry-run] mission upload: WOULD EXECUTE` with the guard checks listed |
| `--json` success | `{"action": "mission_upload", "executed": true, "item_count": N, "outcome": {...ack result...}, "dry_run": false}` — same envelope family as existing commands |
| Failure shape | standard `{"error": {"code", "message", "detail": {"reason", "hint", ...}}}` |
| Exit codes | 2 schema/usage · 3 daemon down · 4 link · 5 guard rejection · 6 vehicle NACK / transaction timeout / **`remote_mission_state_uncertain`** |
| Idempotency | **Not idempotent by design**: re-upload overwrites the remote mission; a repeated upload of the same file succeeds again (no "already" short-circuit). Documented; no misleading already-satisfied state. |
| File/stdout | upload never writes files; stdout carries only the response |
| reason/hint expectations | `mission_schema_invalid` (2), `confirmation_required` (5), `not_connected` (4), `mission_requires_disarmed` (5), `ground_state_stale`/`ground_state_unknown` (5), `remote_mission_state_uncertain` (6, hint: "verify with `mavctl mission download`"), `mission_rejected` (6, with `MAV_MISSION_RESULT` name in detail) |

### B.2 `mavctl mission download [--output <mission.json>] [--json]`

**Decided policy (deterministic):**

- `--json` alone: the mission JSON is printed to **stdout as the only stdout
  content** (machine payload; no human/progress text may pollute stdout).
- `--output <path>` alone: mission JSON is written to the file; stdout gets
  a short human summary (`mission downloaded: N items → <path>`).
- **`--output` and `--json` together are a usage error (exit 2)** — one
  deterministic direction per invocation, no double-emission ambiguity.
- Human output (default, no flags): compact table-like text (seq, type, lat,
  lon, alt).
- Exit codes: 2 (mutually-exclusive flags; unwritable `--output`) · 3 · 4 ·
  6 (transaction failure — **download fails atomically**, §E; no partial
  mission is ever emitted).
- Idempotency: read-only; inherently repeatable.
- reason/hint: `mission_item_unsupported` (6, lists offending seq/command),
  `mission_protocol_timeout` (6). An empty remote mission is a **success**
  (`{"version": 1, "items": []}`).

### B.3 `mavctl mission clear --confirm [--dry-run]`

| Aspect | Design |
| --- | --- |
| Required | `--confirm` |
| Human output (success) | `mission cleared (remote count verified 0)` or the uncertain-state response below |
| `--json` success | `{"action": "mission_clear", "executed": true, "verified": true}` |
| Post-ACK verification | **Decided**: after `MAV_MISSION_ACCEPTED`, run the §E download-prefix (`MISSION_REQUEST_LIST` → expect `MISSION_COUNT == 0`). Verified → success. Verification times out or returns a non-zero count → **`remote_mission_state_uncertain`** (exit 6, `detail.observed_count` when known), never a silent success. |
| Exit codes | 2 · 3 · 4 · 5 · 6 as in B.1 |
| Idempotency | Effectively idempotent: clearing an already-empty mission succeeds (`verified: true, count 0`). |

### B.4 Pre-execution paths (terminology fix)

**State-changing path — `mission upload`, `mission clear`** (same shape as
other dangerous commands):

1. `--confirm` gate (missing → exit 5 `confirmation_required`);
2. fresh link / heartbeat gate (exit 4 `not_connected`);
3. **disarmed** (`armed is False`) **and fresh positive ground evidence**
   (`ground_state_unknown` / `ground_state_stale`, exit 5);
4. `--dry-run` short-circuits here with zero MAVLink traffic.

**Read-only path — `mission download`**:

1. daemon availability (exit 3) and **fresh link / heartbeat** (exit 4)
   only;
2. **no confirmation gate**;
3. **no disarmed / ground-evidence requirement** (nothing is written to the
   vehicle);
4. the link check is *not* the full state-changing guard preamble —
   wording anywhere in this document must not claim all mission commands
   share the confirm/disarmed preamble.

---

## C. Mission JSON v1 schema

```json
{
  "version": 1,
  "items": [
    {"type": "takeoff", "altitude_m": 10.0},
    {"type": "waypoint", "lat_deg": -35.36, "lon_deg": 149.16,
     "altitude_m": 20.0, "hold_s": 0.0, "accept_radius_m": 0.0},
    {"type": "waypoint", "lat_deg": -35.37, "lon_deg": 149.17,
     "altitude_m": 25.0},
    {"type": "rtl"}
  ]
}
```

`[DECIDED]` v1 supports exactly four item types; mapping to
`MISSION_ITEM_INT` (conversions are pure arithmetic — pymavlink stays in the
adapter; models carry plain command numbers):

| type | MAV_CMD | required fields | optional fields (→ params) | x / y / z |
| --- | --- | --- | --- | --- |
| `takeoff` | `MAV_CMD_NAV_TAKEOFF` (22) | `altitude_m` | — (param1 fixed 0.0 in v1) | x=0, y=0, z=`altitude_m` |
| `waypoint` | `MAV_CMD_NAV_WAYPOINT` (16) | `lat_deg`, `lon_deg`, `altitude_m` | `hold_s` (param1, s, default 0.0), `accept_radius_m` (param2, m, default 0.0) | x=`round(lat_deg*1e7)`, y=`round(lon_deg*1e7)`, z=`altitude_m` |
| `land` | `MAV_CMD_NAV_LAND` (21) | `lat_deg`, `lon_deg`, `altitude_m` | `abort_alt_m` (param1, m, default 0.0 = vehicle default per XML) | x/y scaled, z=`altitude_m` |
| `rtl` | `MAV_CMD_NAV_RETURN_TO_LAUNCH` (20) | — (empty object) | — (all params "Empty" per XML → 0.0) | x=0, y=0, z=0 |

**x/y encoding for non-positional items** (`[DECIDED]`, now evidence-backed):

- `[FACT] for compatibility behavior`: ArduCopter explicitly supports
  zero coordinates on takeoff mission items — `get_loc_from_cmd`
  (ArduCopter/mode_auto.cpp) substitutes the current/default position when
  `lat == 0 && lng == 0` — and ignores the location of RTL items entirely
  (`ModeAuto::do_RTL(void)` takes no location).
- `[DECIDED]`: mavctl v1 encodes takeoff and rtl with `x=0, y=0` (rtl
  additionally `z=0`). The `INT32_MAX`-as-default convention documented by
  pymavlink is **not** used in v1; switching to it would need a new decision.
- `[FACT]` the XML parameter tables (§A.4) confirm takeoff param1 is a
  pitch field and RTL params are all "Empty", so fixing `param1=0.0` for
  takeoff and all params 0.0 for rtl is semantically neutral.

**`[OPEN]`** SITL validation gate: the v1 implementation must confirm on
ArduPilot SITL that uploads with `x=0/y=0` takeoff/rtl items are accepted
(`MAV_MISSION_ACCEPTED`) before shipping; if SITL rejects the zero
encoding, takeoff/rtl item construction must be deferred/fixed — not
shipped on assumption.

Rules (`[DECIDED]` unless noted):

- **Type spelling**: the four `type` values are **lowercase canonical
  only** (`"takeoff"`, `"waypoint"`, `"land"`, `"rtl"`); `"TAKEOFF"` or any
  other casing is a schema error (exit 2). No silent case normalization —
  agent and Harness output must round-trip deterministically.
- **Frame**: every item uses `MAV_FRAME_GLOBAL_RELATIVE_ALT_INT` (6) —
  altitudes relative to home, consistent with the existing `takeoff --alt`
  semantics. Absolute-MSL frames are deferred.
- **Coordinate scaling**: `x = round(lat_deg * 1e7)`, `y = round(lon_deg *
  1e7)` into int32; `lat_deg ∈ [-90, 90]`, `lon_deg ∈ [-180, 180]`; every
  number finite (NaN/Infinity → exit 2).
- **Altitude rules**: every `altitude_m` is finite and
  `0 < altitude_m <= GuardConfig.max_takeoff_alt_m` (single reused ceiling).
- **Waypoint extras**: `hold_s` ≥ 0 (seconds; ignored by fixed wing,
  meaningful for rotary — ArduCopter-first so in scope); `accept_radius_m`
  ≥ 0 (meters). `[OPEN]` whether `accept_radius_m = 0` means "vehicle
  `WP_RADIUS` default" on ArduCopter. `pass_radius_m` (param3) and waypoint
  yaw (param4) are **non-goals in v1** — fixed `param3=0.0` (pass through)
  and `param4=0.0`.
- **Land extras**: `abort_alt_m` ≥ 0 with default 0.0 — the XML states
  `0 = undefined / use system default`, so the default is semantically
  safe. Precision land mode (param2) is a **non-goal in v1** (fixed 0.0).
- **Sequence**: `seq` = list index, 0-based, gapless (`[FACT]` pymavlink
  requirement). Items are emitted in list order.
- **`current` / `autocontinue`**: all items uploaded with `current=0`,
  `autocontinue=1`.
- **Max item count**: 100 (`[DECIDED]` v1 constant; the vehicle's real
  capacity surfaces as `MAV_MISSION_NO_SPACE` — `[OPEN]` reconcile).
- **First item must be `takeoff`** — a mavctl **product constraint**
  (ArduCopter-first), **not** a MAVLink protocol rule; violation → schema
  error exit 2. (`[OPEN]` whether to relax later.)
- **Empty missions rejected on upload**: `items: []` is a valid *download*
  result but an **upload schema error** (nothing to upload is a user
  mistake, and an empty upload is a confusing no-op).
- **No airborne mission replacement**: v1 requires disarmed + fresh ground
  evidence for upload/clear (§G); replacing a mission in flight is out of
  scope, as are future Plane/Rover frames and pre-armed workflows.
- **Terminal rule**: nothing may follow `land` or `rtl` — a mavctl **v1
  product constraint** (unreachable items are a planning error), **not** a
  MAVLink protocol rule; violation → schema error exit 2.
- **Strict schema**: unknown `version`, unknown `type`, unknown extra keys,
  wrong types → exit 2. **No arbitrary `MAV_CMD`, no arbitrary param arrays,
  no arbitrary frame, no mission-type selection** are expressible in JSON.
- **Evolution policy**: a mavctl version supports an **explicit finite set
  of mission schema versions**; it rejects unsupported input versions. An
  upload of a newer file than the tool understands fails (exit 2, hint:
  upgrade mavctl). Download emits the newest schema version that can
  represent the remote mission **losslessly**; otherwise download fails
  atomically with `mission_item_unsupported`. No "always additive" or
  "always convertible" promise is made.

---

## D. Upload state machine

Transaction (ArduPilot-paced; §A.5 facts):

```text
GCS → MISSION_COUNT(count=N, mission_type=MISSION)
vehicle → MISSION_REQUEST_INT(seq=0)          (vehicle drives pacing)
GCS → MISSION_ITEM_INT(seq=0)                 (repeat for seq=1..N-1)
vehicle → MISSION_ACK(type, mission_type)     terminal, immediately after
                                              the last item is processed
```

`[FACT]` ArduPilot enforces **strict in-order** items: any item whose seq ≠
expected gets `MISSION_ACK(MAV_MISSION_INVALID_SEQUENCE)` **while the
session stays alive** waiting for the expected seq. `[FACT]` the vehicle
times an upload out after **8 s** without items and cancels with
`MISSION_ACK(MAV_MISSION_OPERATION_CANCELLED)`.

`[DECIDED]` **request classification** (mavctl tracks
`expected_next_seq`, starting at 0 after `MISSION_COUNT`): the vehicle
decides *pacing*, but mavctl must respect the verified strict sequence —
a future item sent ahead of `request_i` would be answered
`INVALID_SEQUENCE` by the vehicle, so it is never sent:

| Incoming `MISSION_REQUEST_INT` | mavctl behavior |
| --- | --- |
| `seq == expected_next_seq` | send exactly `MISSION_ITEM_INT(seq)`; advance `expected_next_seq` |
| `seq < expected_next_seq` (duplicate of an already-sent item) | re-send that already-sent item **without advancing** `expected_next_seq` (packet-loss / retry compatible) |
| `seq > expected_next_seq` (future item / gap) | **send nothing; abort** → `remote_mission_state_uncertain` (exit 6) with `detail.expected_seq`, `detail.requested_seq` and `detail.accepted_upto`; hint: verify with `mavctl mission download` |
| `seq >= N` (out of range) | same abort as the future-item row (a special case of the gap) |

`[DECIDED]` GCS-side phases and behavior:

| Phase | State | Timeout action |
| --- | --- | --- |
| **U0** | `MISSION_COUNT` not yet sent (local validation/guards) | plain failure (exit 2/5); **no uncertain state** — nothing was sent |
| **U1** | `COUNT` sent, awaiting first `MISSION_REQUEST_INT(0)` | no item has been accepted; **resend `MISSION_COUNT`** up to `mission_retry_count`; if still nothing → abort with `remote_mission_state_uncertain` (the vehicle may be mid-allocation; read-back recommended) |
| **U2** | items flowing (awaiting `REQUEST_INT(seq)` after sending item `seq`) | requests are classified per the ordering table above (expected / duplicate / future-gap); on request timeout re-send the **last expected** item; on resend exhaustion → abort `remote_mission_state_uncertain` (a partial prefix may be stored). **Never** send items ahead of requests — future items are a strict-sequence violation that aborts the transaction |
| **U3** | last item sent, awaiting terminal `MISSION_ACK` | wait `mission_ack_timeout_s` (candidate 2.0 s; the vehicle ACKs immediately per §A.5, so this window is short); on timeout → **not** an immediate error: the 8 s vehicle timer may still deliver `OPERATION_CANCELLED`/ACCEPTED — keep listening up to the overall transaction deadline, then `remote_mission_state_uncertain` |

`[DECIDED]` cross-cutting behavior:

| Event | Behavior |
| --- | --- |
| unexpected `MISSION_ACK(MAV_MISSION_OPERATION_CANCELLED)` at any phase | the **vehicle** cancelled (its 8 s timer) → abort, `remote_mission_state_uncertain` (exit 6), hint: read-back |
| terminal ACK `MAV_MISSION_ACCEPTED` | success |
| terminal ACK non-zero (`NO_SPACE`, `INVALID_SEQUENCE`, `UNSUPPORTED_FRAME`, …) | `mission_rejected` (exit 6, result name in detail). **Special case** `MAV_MISSION_INVALID_SEQUENCE` on the *GCS* side is a mavctl bug (we only send the requested seq) — surfaced as such |
| ACK with `mission_type != MISSION` | ignored (mismatch; keep waiting for the real ACK) |
| item/ACK from a non-locked source | ignored entirely (same source-filtering discipline as COMMAND_ACK) |
| overall transaction deadline (candidate 15 s) | abort → `remote_mission_state_uncertain` whenever `COUNT` was already sent (U1+); plain error in U0 |
| daemon shutdown mid-transaction | transaction state discarded; client sees connection loss; remote state uncertain by construction |
| concurrency | adapter `_mission_lock` (§H); mission RPCs that change state also take the daemon `_command_lock` |

`[DECIDED]` **uncertain-state reporting**: every `remote_mission_state_uncertain`
response carries `detail.reason`, a `hint` recommending
`mavctl mission download` read-back, and — when items were accepted before
the abort — `detail.accepted_upto` (highest seq known accepted). "Uncertain"
is **never** used for failures that occurred strictly before `MISSION_COUNT`
was sent (U0).

**Why `_send_command()` cannot be reused** (`[FACT]` by construction): the
COMMAND_ACK machinery correlates one `COMMAND_LONG` → one `COMMAND_ACK` by
command id; the mission protocol is a multi-message session where the
*vehicle* drives pacing via `MISSION_REQUEST_INT` and terminates with
`MISSION_ACK`. `[DECIDED]`: a dedicated adapter-level mission transaction
(own waiter on the reader thread, analogous to `_acks`) plus
`_mission_lock`; the reader thread remains the only MAVLink reader.

`[OPEN]` Whether ArduPilot ever re-requests the same seq at the transport
layer (ap-message queue re-sends): mavctl tolerates duplicate requests
either way; confirm in SITL.

---

## E. Download state machine

```text
GCS → MISSION_REQUEST_LIST(mission_type=MISSION)
vehicle → MISSION_COUNT(count, mission_type=MISSION)
GCS → MISSION_REQUEST_INT(seq=0..count-1)
vehicle → MISSION_ITEM_INT(seq)                (per request)
GCS → MISSION_ACK(MAV_MISSION_ACCEPTED)        (terminal, GCS ends session)
```

`[FACT]` ArduPilot answers `MISSION_REQUEST_LIST` with
`MISSION_ACK(MAV_MISSION_DENIED)` **while an upload is in progress** —
another reason `adapter _mission_lock` serializes sessions (§H).
`[FACT]` ArduPilot does **not** consume the GCS's download-terminal ACK
(`/* not used */` in its dispatch) — sending it is protocol courtesy and its
failure must be **non-fatal**.

`[DECIDED]` handling:

| Event | Behavior |
| --- | --- |
| `MISSION_COUNT.count == 0` | empty mission → success `{"version": 1, "items": []}`; terminal ACK still sent (courtesy) |
| duplicate item (seq seen) | keep the first; ignore retransmissions |
| out-of-order item | buffer by seq; completion requires all of 0..count-1 |
| missing item / request timeout | re-send `MISSION_REQUEST_INT(seq)` up to `mission_retry_count` (candidate 3, `mission_request_timeout_s` candidate 1.0 s); then **fail atomically** (exit 6, `mission_protocol_timeout`, `detail.received`) — **no partial mission is ever emitted** |
| source / mission-type filtering | count/items from other sources or `mission_type`s ignored |
| unsupported remote item (`command` outside the v1 whitelist or frame ≠ 6) | **download fails atomically** with `mission_item_unsupported` (exit 6) listing seq/command — never silently dropped, never lossy pseudo-JSON (download must round-trip) |
| conversion to JSON | inverse of §C; ints scaled `/1e7` |
| final ACK | GCS sends `MISSION_ACK(MAV_MISSION_ACCEPTED)` targeting the locked autopilot; send failure is **non-fatal** (`[FACT]`: ArduPilot ignores it) |
| target IDs | all requests and the terminal ACK use the locked autopilot's system/component (same discovery as COMMAND_ACK) |

---

## F. Clear protocol

`[FACT]` Clearing is the `MISSION_CLEAR_ALL` **message** (id 45); **no such
MAV_CMD exists** (§A.3).

`[FACT]` ArduPilot handles `MISSION_CLEAR_ALL` via
`GCS_MAVLINK::handle_mission_clear_all` → protocol `handle_mission_clear_all`,
and replies with `MISSION_ACK` (sent via `send_mission_ack` throughout the
protocol implementation). The `mission_type` extension field selects the
cleared mission store.

`[DECIDED]` flow:

1. send `MISSION_CLEAR_ALL(mission_type=MISSION)` to the locked autopilot;
2. await `MISSION_ACK(type, mission_type)`: `ACCEPTED` → verification step;
   non-zero → `mission_rejected` (exit 6); ACK timeout → **one** automatic
   resend of `MISSION_CLEAR_ALL`, then `remote_mission_state_uncertain`;
3. **post-clear verification** (`[DECIDED]`): run the §E download-prefix
   (`MISSION_REQUEST_LIST` → `MISSION_COUNT`) and require `count == 0`;
   verified → success; a non-zero count → `remote_mission_state_uncertain`
   with `detail.observed_count` (the observed number is diagnostic data, not
   a silent failure); verification timeout → `remote_mission_state_uncertain`
   without `observed_count`.

---

## G. Guard / safety model

`[DECIDED]` two distinct paths (see §B.4):

- **`mission upload` / `mission clear`** (state-changing path): standard
  `--confirm` gate → fresh heartbeat gate → **disarmed** (`armed is False`)
  **and fresh positive ground evidence** — reusing the `_is_fresh_age` +
  ground-evidence discipline of ordinary disarm (`ground_state_unknown` /
  `ground_state_stale`, exit 5). Candidate additional reason:
  `mission_requires_disarmed`.
- **`mission download`** (read-only path): daemon availability + fresh
  link/heartbeat only; no confirmation, no disarmed/ground requirement.

Limits enforced at schema/guard layer: item count ≤ 100; every altitude ≤
`GuardConfig.max_takeoff_alt_m`; command whitelist = the four semantic
types; coordinate ranges validated (§C).

Exit-code mapping: JSON/schema/usage **exit 2** (`invalid_mission` with
field-level detail) · guard rejection **exit 5** · connection loss
**exit 4** · MAVLink NACK / transaction timeout /
`remote_mission_state_uncertain` **exit 6**.

`--dry-run` performs schema validation + guards and **sends zero MAVLink
traffic** (test-asserted via `master.sent == []`).

`mission download` read-only handling of unsupported remote items: atomic
structured failure (§E) — safety is preserved because nothing is mutated,
and no misleading JSON is produced.

---

## H. Architecture proposal

Likely future modules (none created in this design round):

```text
models/mission.py            MissionV1/MissionItem pydantic schemas, item→
                             (command, param1..4, x, y, z, frame) mapping
                             (pure data + arithmetic; no pymavlink import)
adapter/mission.py           MISSION_ITEM_INT construction, upload/download/
                             clear transactions, _mission_lock, reader-thread
                             routing of MISSION_* messages
daemon/guards.py             mission guards (share ground-evidence helpers)
daemon/server.py             mission_upload / mission_download / mission_clear
                             RPC handlers
cli/app.py                   `mission` Typer sub-app
tests/test_mission_*.py      schemas, mappings, state machines, guards, CLI
```

**Lock model** (`[DECIDED]`):

| Lock | Held by | Purpose |
| --- | --- | --- |
| adapter `_mission_lock` | upload, download, clear (all three) | serializes MISSION_* protocol sessions on the one vehicle link — overlapping sessions would interleave requests/items/ACKs; ArduPilot also **DENIES** downloads during uploads (`[FACT]`), so local serialization simply respects vehicle reality |
| daemon `_command_lock` | `mission upload`, `mission clear` | these mutate vehicle state and must serialize with arm/takeoff/land/rtl — same transaction discipline as every state-changing command |
| daemon `_command_lock` | **`mission download` does NOT take it** | download is read-only: it touches no vehicle control state, so a download may run **concurrently with a long `takeoff --wait`** — giving agents a way to inspect the remote mission during long waits (positive interaction with Issue #21, not a conflict) |

Rationale and safety analysis:

- *Why download may run beside flight commands*: it only exchanges
  MISSION_REQUEST/ITEM/ACK protocol traffic for the stored mission; it does
  not read or write flight-control state, and the guards are not involved.
- *How download/upload/clear overlap is prevented*: the shared adapter
  `_mission_lock` covers all three; the vehicle's own
  `MISSION_DENIED`-during-upload behavior is thereby never triggered
  locally.
- *status/telemetry*: remain readable at all times — neither the mission
  locks nor `_command_lock` are taken by fast handlers.
- *TOCTOU / reader routing*: MISSION_* messages are consumed exclusively by
  the active mission transaction on the reader thread; the COMMAND_ACK map
  and the mission waiter are disjoint message sets, so no cross-routing race
  exists. A mission RPC that changes state takes `_command_lock` for its
  whole transaction (state read → guards → execute), preserving the
  existing TOCTOU guarantees.
- *Issue #21 relation*: unchanged — upload/clear are bounded short
  transactions under `_command_lock`; the long-`--wait` interruption
  problem is orthogonal and tracked in #21.

Hard constraints (all `[DECIDED]`, unchanged from the architecture rules):

- pymavlink imports remain **only** in the adapter layer (models use plain
  command-number constants);
- the reader thread stays the only MAVLink reader; MISSION_* messages are
  routed to the mission transaction handler from there;
- the existing COMMAND_ACK machinery stays separate and untouched;
- no raw MAVLink send path for agents;
- the Windows IPC transport design (Issue #19) remains independent — mission
  code rides the existing daemon RPC surface only.

---

## I. Test and SITL acceptance plan

Implementation testing is split into five tiers (no test code exists yet):

**A. Pure schema / mapping unit tests**

- valid minimal/typical missions; every rejection rule (version, unknown
  type/keys, non-lowercase type spelling, non-finite, lat/lon ranges,
  altitude ceiling/count, empty-items upload, first-item-takeoff,
  trailing-after-terminal).
- mapping: each item type → exact `MISSION_ITEM_INT` field tuple (command
  number, params, 1e7 scaling, frame 6, current/autocontinue); round-trip
  download-mapping equivalence.

**B. Mock MAVLink transaction tests**

- upload: happy path; duplicate `MISSION_REQUEST_INT` (same seq twice);
  duplicate request (`seq < expected` → the same item is re-sent);
  **future request** (`seq > expected` → strict-sequence violation per
  ArduPilot `request_i`: nothing is sent, the transaction aborts
  `remote_mission_state_uncertain` with `expected_seq`/`requested_seq`
  detail); invalid requested seq (`>= N`) → same abort; U1/U2 resend
  exhaustion → `remote_mission_state_uncertain`; unsolicited
  `OPERATION_CANCELLED`; ACK rejection names; mission-type and source
  filtering; U0 failures produce **no** uncertain state.
- download: count=0; happy path; duplicate/out-of-order items; missing item
  → atomic failure with `detail.received`; unsupported command → atomic
  failure; terminal ACK sent, non-fatal.
- clear: ACCEPTED + verified 0 → success; non-zero observed_count →
  uncertain with `detail.observed_count`; ACK timeout → resend → uncertain.

**C. Adapter-reader routing / concurrency tests**

- MISSION_* messages reach the mission transaction, COMMAND_ACK still
  reaches the command waiter (disjoint routing).
- status/telemetry reads succeed while a mission transaction is in flight.
- two concurrent mission RPCs serialize on `_mission_lock`; upload and arm
  serialize on `_command_lock`; download does **not** block a `takeoff
  --wait`.

**D. Daemon/CLI contract tests**

- human and `--json` outputs for all three commands; `--output` + `--json`
  mutual exclusion (exit 2); file-not-found; stdout purity in `--json`
  mode; exit-code mapping 2/3/4/5/6; reason/hint presence.

**E. SITL protocol conformance tests** (implementation phase)

1. Start clean SITL.
2. Upload the exact four-item mission:
   `TAKEOFF 10m → WAYPOINT A → WAYPOINT B → RTL`.
3. **Verify the actual requested sequence pattern** observed by the mock
   GCS is `REQUEST_INT(0), REQUEST_INT(1), …` in order.
4. **Verify terminal ACK timing/type/mission_type**: `MAV_MISSION_ACCEPTED`,
   `mission_type=MISSION`, arriving immediately after the last item.
5. Download and compare semantic round-trip equivalence.
6. Clear and read back `count == 0`.
7. Re-upload the same mission (overwrite path).
8. If the harness can inject traffic: simulate a duplicated
   `MISSION_REQUEST_INT` and confirm mavctl re-sends the item.
9. **Verify the chosen `x=0/y=0` TAKEOFF/RTL encoding is accepted**
   (`MAV_MISSION_ACCEPTED`) — the §C `[OPEN]` gate.
10. Verify timeout behavior only inside a controlled harness (e.g. a
    fake endpoint that stops responding) — never by stalling real SITL
    arbitrarily.

Safety constraints for all SITL tests: **no mission execution, no takeoff,
no AUTO switch**; upload/clear do mutate SITL state, so the test fixture
must restore/clear mission state afterwards; real aircraft are permanently
out of scope.

---

## J. Open questions

1. Does `MAV_CMD_MISSION_START` (300) belong entirely to Phase 3B?
   (Current answer: yes — verified here only to settle §F/A.3.)
2. Exact ArduPilot `MISSION_ACK` pacing nuances beyond the verified
   immediate-after-last-item behavior — confirm against SITL.
3. Unsupported remotely-downloaded items: v1 fails atomically; should a
   later version offer a raw-passthrough view for inspection?
4. Final post-clear verification policy (current: mandatory read-back with
   `observed_count` on mismatch).
5. Retry timing / transaction timeout defaults (candidates: 1.0 s
   per-request, 3 retries, 15 s overall, 2.0 s terminal-ACK window) — tune
   against SITL against the vehicle's own 8 s upload timer.
6. Relation to Issue #21: not a 3A blocker; download is deliberately
   runnable during long waits; upload/clear cancellation remains future.
7. Mission item count limit (v1 constant 100) vs `MAV_MISSION_NO_SPACE`
   runtime truth — reconcile at implementation.
8. Schema evolution details beyond the finite-version-set policy (§C).
9. Optional future QGC WPL import (post-MVP, separate decision).
10. How a future Harness maps map geometry (polygons, survey grids) into
    mission JSON — Harness-side concern; interface undefined.
11. `x`/`y` encoding is settled for ArduCopter (0 = current position /
    ignored); confirm acceptance on SITL per §C gate, and revisit only if
    other frames are ever supported.
12. Whether `rtl` items should accept an optional cruise altitude later.
13. Whether `waypoint.accept_radius_m = 0` selects the vehicle `WP_RADIUS`
    default on ArduCopter.
14. Whether `mission download` should ever be restricted while armed
    (ArduPilot only rejects it during an active upload; mavctl currently
    allows read-only download any time the link is fresh).

---

## Related documents

- `docs/design/windows-native-support.md` — Windows IPC design (Issue #19)
- `docs/design/product-architecture-roadmap.md` — Phase 3A/3B context
- `docs/SITL_ACCEPTANCE_PHASE2.md` — current manual acceptance surface
