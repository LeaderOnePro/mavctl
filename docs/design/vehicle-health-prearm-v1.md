# Design: Vehicle Health and Pre-Arm Observation (v1 candidate)

Status: **design research only** — nothing in this document is implemented.
No runtime health guard exists; `arm` / `mission start` behavior is unchanged
by this document. Tracking issue: #4. The [FACT]/[DECIDED]/[OPEN]/[NON-GOAL]
discipline matches docs/design/mission-protocol-v1.md and
docs/design/mission-execution-phase3b.md.

ArduPilot-first: every protocol fact below is verified against the pymavlink
dialect XMLs shipped in mavctl's virtualenv and a local ArduPilot checkout at
revision `4c98c9221a` (the same reviewed checkout as every mavctl SITL
validation; `AP_FWVersion.h` macOS host-build workaround only).

## A. Current mavctl safety inputs

What mavctl consumes today (adapter handlers, `src/mavctl/adapter/
pymavlink_adapter.py`):

| Input | Source message | Consumer |
| --- | --- | --- |
| heartbeat freshness | HEARTBEAT | `_on_heartbeat` (link, armed, mode, system_status) |
| GPS fix | GPS_RAW_INT | `_on_gps_raw` (fix_type → fix_label) |
| battery | SYS_STATUS | `_on_sys_status` (**battery fields only**) |
| ground / landed evidence | EXTENDED_SYS_STATE | `_on_extended_sys_state` (landed_state) |
| position / attitude | GLOBAL_POSITION_INT, ATTITUDE | telemetry + guards |
| freshness metadata | per-stream monotonic ages | `status --json` |
| mission existence | MISSION_REQUEST_LIST probe | mission-start guard |
| command outcomes | COMMAND_ACK | exit-code contract |

`[FACT]` mavctl has **no EKF/estimator observation and no pre-arm-health
guard** today: the SYS_STATUS handler stores battery fields only — the
`onboard_control_sensors_present/enabled/health` bitmasks are dropped; no
STATUSTEXT, EKF_STATUS_REPORT, VIBRATION, or GPS2_RAW handler exists.

`[FACT]` the adapter requests `MAV_DATA_STREAM_EXTENDED_STATUS` at 2 Hz on
connect (plus `MAV_CMD_SET_MESSAGE_INTERVAL` for HOME_POSITION). ArduPilot
documents EKF_STATUS_REPORT and VIBRATION as belonging to the AHRS/SR
stream class (GCS_MAVLink_Parameters.cpp:179), so EKF status may already
arrive on the wire without mavctl consuming it — to be verified at SITL
[OPEN].

`[DECIDED]` ArduPilot's own pre-arm checks remain the **final authority**
for arming safety. mavctl's goal is observation and explainability, never
re-implementing the ArduPilot check set (see Non-goals).

## B. ArduPilot / MAVLink health facts (verified, read-only)

Research method: pymavlink dialect XMLs in mavctl's venv
(`pymavlink/message_definitions/v1.0/`) + local ArduPilot checkout
`4c98c9221a`. Note: in the shipped pymavlink, common.xml holds both enums
and messages; `EKF_STATUS_REPORT` lives in **ardupilotmega.xml**.

### EKF_STATUS_REPORT (id 193)

- `[FACT]` **ardupilotmega.xml extension** (not MAVLink common): fields
  `flags (uint16, EKF_STATUS_FLAGS bitmask)` + 7 variance floats
  (velocity, pos_horiz, pos_vert, compass, terrain_alt, airspeed).
- `[FACT]` direction: vehicle → GCS. ArduPilot sends it via
  `AP::ahrs().send_ekf_status_report()` from the GCS message scheduler
  (GCS_Common.cpp:6780-6782).
- `[FACT]` the sender **early-returns — sends nothing — when
  `filter_status_valid` or `variances_valid` is false**
  (AP_AHRS.cpp:1765-1772). Message absence therefore does **not** mean
  "unhealthy": early after boot (or with an estimator that has not
  converged) the message stream is simply silent. Any consumer must model
  this as `unknown`, never as failure.
- `[FACT]` the flags are produced by the AHRS layer from the configured
  estimator backend (EKF2/EKF3/DCM) — a unified flag set across backends
  (AP_AHRS.cpp:1760-1810 maps `filter_status.flags.attitude / horiz_vel /
  vert_vel / horiz_pos_rel / horiz_pos_abs / …` → `EKF_ATTITUDE`,
  `EKF_VELOCITY_HORIZ`, …).
- Suitability: **strong candidate** for agent observation (stable bitmask,
  documented enum) and a future guard input; needs a stream-rate check at
  SITL [OPEN].

### ESTIMATOR_STATUS (id 230)

- `[FACT]` MAVLink **common.xml** message (flags bitmask
  `ESTIMATOR_STATUS_FLAGS`, accuracy floats, health ratios).
- `[FACT]` **ArduPilot does not send it** in the researched checkout: no
  `ESTIMATOR_STATUS` sender exists under `libraries/AP_AHRS/` or
  `libraries/GCS_MAVLINK/` (grep-verified, 2026-10).
- `[DECIDED]` not a viable observation source for ArduPilot-first work:
  mavctl designs against what ArduPilot actually emits. Re-evaluate only if
  a future firmware starts sending it.

### SYS_STATUS (id 1)

- `[FACT]` common.xml; three bitmask fields
  (`onboard_control_sensors_present / enabled / health`,
  `MAV_SYS_STATUS_SENSOR_*` semantics) + battery + comm error counters.
- `[FACT]` direction: vehicle → GCS, streamed; mavctl already receives it
  (battery handler) but drops the sensor bitmasks.
- Suitability: good coarse observation (which sensors the vehicle reports
  present/enabled/healthy) and a plausible future guard input. Caveat: the
  bitmask semantics for which sensors ArduPilot reports as "health" vary
  by vehicle/firmware (e.g. sensors not fitted are present=0) — mapping
  needs a documented table and SITL verification [OPEN].

### STATUSTEXT (id 253)

- `[FACT]` common.xml; `severity` + `text` (char[50], plus `id` /
  `chunk_seq` chunking fields in newer dialects — long messages are split,
  so text assembly is itself version-dependent).
- `[FACT]` direction: vehicle → GCS (events). ArduPilot's arming checks
  report failures as `PreArm: <reason>` STATUSTEXT with
  `MAV_SEVERITY_CRITICAL` (AP_Arming.cpp:349-365); a disarmed
  `MAV_CMD_RUN_PREARM_CHECKS` request runs the check suite and reports the
  same way (GCS_Common.cpp:5040-5047: ACCEPTED after running
  `AP::arming().pre_arm_checks(true)`, TEMPORARILY_REJECTED if soft-armed).
- `[FACT]` the `PreArm: %s` strings are composed per check — the reason
  text is **not a stable API**; it varies with firmware version, vehicle
  type, and check parameters.
- `[DECIDED]` STATUSTEXT must **not** be the sole safety signal, and no
  specific text string may be treated as a stable interface. Normalization
  (prefix `PreArm:`/`Arm:`/`PreArm_EKF:` extraction, severity, free-text
  kept for display) is the only defensible use. "Ready because no PreArm
  text arrived" is forbidden reasoning (absence of errors ≠ health).

### EXTENDED_SYS_STATE (id 245)

- `[FACT]` common.xml (`vtol_state`, `landed_state`); already consumed by
  mavctl (ground evidence). Not an EKF/pre-arm signal; listed for
  completeness. No new work.

### HEARTBEAT.system_status

- `[FACT]` common/minimal (`MAV_STATE`: STANDBY, ACTIVE, CRITICAL, …);
  already consumed and exposed by mavctl. Coarse (CRITICAL/FAQ flags
  signal trouble) but unstable as a guard input (varies with failsafe
  state); observe-only candidate.

### VIBRATION (id 241)

- `[FACT]` common.xml (`vibration_x/y/z`, clipping counters); ArduPilot
  sends it in the AHRS/SR stream class (parameter description,
  GCS_MAVLink_Parameters.cpp:179). Long-term health trend input only —
  no instantaneous threshold is safe [OPEN].

### GPS_RAW_INT (id 24) / GPS2_RAW (id 124)

- `[FACT]` common.xml; fix_type, satellites_visible, eph/epv, h_acc/v_acc.
  mavctl consumes GPS_RAW_INT (fix label) already; GPS2_RAW is not
  consumed. Raw-GPS accuracy is an input to the estimator, not estimator
  state itself — useful context for explaining EKF degradation, not a
  health verdict.

### MAV_CMD_RUN_PREARM_CHECKS (command 401)

- `[FACT]` **MAVLink common command, value 401** — defined in MAVLink
  common.xml both in mavctl's venv pymavlink (`common.xml:1572`) and in
  ArduPilot's dialect submodule
  (`modules/mavlink/message_definitions/v1.0/common.xml:1947`). The name
  is available to pymavlink users — no numeric-only use or dialect bump
  is needed.
- `[FACT]` **command 241 is MAV_CMD_PREFLIGHT_CALIBRATION — a different
  command** (sensor calibration; common.xml:1491). It must never be sent
  to "run pre-arm checks"; an earlier draft of this document wrongly
  paired the name with 241 — corrected here against both dialect sources
  (post-review factual fix, 2026-10).
- `[FACT]` ArduPilot handles 401 (GCS_Common.cpp:5933 →
  `handle_command_run_prearm_checks`, 5040-5047):
  `MAV_RESULT_TEMPORARILY_REJECTED` while soft-armed; otherwise it runs
  `AP::arming().pre_arm_checks(true)` and returns `MAV_RESULT_ACCEPTED`.
  `MAV_CMD_PREFLIGHT_CALIBRATION` (241) is handled by a separate path
  (GCS_Common.cpp:5901 → calibration handlers) and does not run the
  arming check suite.
- `[FACT]` the official XML description of 401 states the return value
  "does not indicate whether the vehicle is armable or not, just whether
  the system has successfully run/is currently running the checks" and
  that the spec reflects check results in SYS_STATUS — whereas ArduPilot's
  observed reporting path for check failures is `PreArm:` STATUSTEXT.
  The spec-vs-implementation reporting gap is itself a compatibility
  question [OPEN].
- Suitability: the cleanest way to ask the vehicle "run your pre-arm
  checks now and tell me" without arming — a command-level alternative to
  passive text listening. The ACCEPTED result means "checks ran", **not**
  "checks passed" — pass/fail still arrives via STATUSTEXT, and
  STATUSTEXT/text correlation remains **not** a stable hard-safety API
  [OPEN: how to correlate reliably].

## C. Candidate health model (not implemented)

`[DECIDED]` structure is a **candidate only**; everything below is design
material for a future implementation phase.

```json
{
  "health": {
    "estimator": {
      "status": "unknown",
      "last_status": null,
      "flags": [],
      "age_s": null
    },
    "prearm": {
      "ready": null,
      "last_message": null,
      "age_s": null
    }
  }
}
```

Rules — the model deliberately separates **current verdict**, **last
observed (possibly stale) verdict**, and **never observed**:

- `[DECIDED]` `status` is the **current** verdict:
  `healthy | degraded | unknown`. It is `unknown` whenever there is no
  currently usable verdict — either nothing was ever observed, or the
  last observation is older than the freshness policy allows. `unknown`
  never means healthy.
- `[DECIDED]` `last_status` (`healthy | degraded | null`) is the last
  **observed** verdict and may be a **stale cache**; `null` means never
  observed. `flags` (possibly stale, `[]` when never observed) belongs to
  the same last-observation namespace.
- `[DECIDED]` `age_s` distinguishes the two non-fresh cases:
  `null` = **never observed**; a monotonically increasing number =
  **observed, this long ago**. A consumer can therefore tell "no data
  yet" apart from "data is N seconds old" — both render as
  `status == "unknown"`, but only the latter keeps a cached
  `last_status`.
- `[DECIDED]` **stale cached data must never be presented as current
  health**: cached `last_status == "healthy"` is history, not permission.
  A future guard must never treat a stale cached healthy observation as
  an allow basis (§D/§E); any staleness threshold that downgrades
  `status` from a cached verdict to `unknown` must be a documented
  policy constant [OPEN: threshold values].
- `[DECIDED]` `unknown` is a distinct state and **never** means healthy.
  Boot-time silence of EKF_STATUS_REPORT (verified early-return behavior,
  §B) surfaces as `status == "unknown"`, `last_status == null`,
  `age_s == null`.
- `[DECIDED]` STATUSTEXT-derived `prearm` data is display/annotation
  material; `prearm.ready` may only become `false` from an observed
  failure report, never `true` from text alone (§B: text is not a stable
  API; absence of errors ≠ health). `prearm` carries the same
  never-observed (`age_s: null`) vs stale (`age_s` counting) split.
- `[DECIDED]` health freshness is **independent** of the existing
  telemetry freshness metadata: a fresh GPS fix or fresh battery does not
  freshen the estimator observation.
- `[DECIDED]` flag → `healthy | degraded` mapping must be a documented,
  versioned table (see Open questions), never an inline heuristic.

## D. Candidate guard policy

Three candidate policies for a future guard (comparison only; nothing is
implemented):

| | 1. Observe only | 2. Advisory guard | 3. Hard safety rejection |
| --- | --- | --- | --- |
| Behavior | status --json + health fields; no gating | warn in output / structured warning, exit 0 | reject arm / mission start (exit 5) on unhealthy-or-unknown |
| False positive cost | none | noise; agents may learn to ignore | **blocks a safe vehicle** (e.g. boot-time `unknown`, flags mavctl maps wrongly) |
| False negative cost | agent may miss unhealthy state | same, smaller | none at mavctl level (ArduPilot still gates) |
| Firmware compatibility | safe (unknown degrades gracefully) | needs per-version flag table | needs per-version flag table + verified stream behavior across versions |
| Agent behavior | full freedom, self-explainable | better prompts; may override | may cause retry loops against a vehicle mavctl refuses |
| Impact on arm / mission start | none | none (exit 0) | **changes exit-code contract surface** (new reasons in the exit-5 family) |
| ArduPilot final authority | intact | intact | intact (mavctl only adds a client-side gate) |
| SITL testability | direct (message capture) | direct | needs failure-scenario injection (simulated unhealthy EKF) |
| Real-aircraft risk | none | low | highest — a wrong mapping becomes a hard block |

`[DECIDED]` recommended progressive route (nothing implemented now):

1. **v1: observe only** — consume EKF_STATUS_REPORT (+ SYS_STATUS sensor
   bitmasks) into the candidate health model; STATUSTEXT annotation only.
2. **validate** — across SITL firmware versions and the shared-MAVProxy
   topology; build the flag→state mapping table from evidence.
3. **advisory** — surface warnings without changing exit codes.
4. **possible hard guard** — only for states proven safe to reject across
   the validated matrix (e.g. an EKF flag combination ArduPilot itself
   refuses to arm on); must remain subordinate to ArduPilot's own
   pre-arm authority.

## E. Agent / error contract (future)

A future implementation should follow the existing mavctl conventions:

- structured rejections/warnings carry `reason` (stable token), `message`
  (human), `hint` (next action) — e.g. `estimator_unhealthy`,
  `estimator_state_unknown`, `prearm_check_failed`;
- `status --json` exposes the candidate health block so a single query
  keeps the vehicle picture self-describing;
- when health is unknown or stale the contract must **not** invent a
  verdict: `status` reports `unknown`, while the cached `last_status`,
  `flags` and the counting `age_s` stay truthful about what was last
  observed and how long ago — and any future advisory wording must say
  "no current estimator information", never "estimator OK";
- a stale cached `healthy` observation is never an allow basis for any
  future guard — consistent with the mission-count philosophy: guards act
  on vehicle-verified **current** evidence, never on cached or absent
  evidence;
- consistent with the mission-count philosophy: guards act on
  vehicle-verified evidence, never on absence of evidence.

## F. Future test plan (designed now, not run)

- mock message-parsing tests: EKF_STATUS_REPORT flag bitmasks → candidate
  states; SYS_STATUS bitmask combinations; VIBRATION/clipping ingestion;
- source filtering: locked-target only; foreign-source EKF/STATUSTEXT
  never pollutes state (mirrors MISSION_CURRENT tests);
- freshness tests: age growth, never-observed (`age_s: null`,
  `last_status: null`) vs stale-observed (`age_s` counting,
  `last_status` retained) three-way semantics, `status` downgrade to
  `unknown` past the policy threshold, disconnect staleness not treated
  as live fact;
- STATUSTEXT normalization tests: `PreArm:`/`Arm:` prefix extraction,
  chunked text assembly, severity handling, unstable-text isolation
  (assertion that no bare string is matched);
- SITL observation tests (sitl-marked, loopback-only): EKF status arrives
  on the validated topology; `MAV_CMD_RUN_PREARM_CHECKS` ACCEPTED + text
  correlation on a healthy SITL vehicle;
- pre-arm failure scenarios: induced failure at SITL (e.g. forced
  EKF-unhealthy via parameter or sim state) — design of the injection
  method is [OPEN];
- firmware compatibility matrix: flag/table rows per tested ArduPilot
  version, recorded in this document before any advisory/hard guard.

## G. Non-goals

- no full duplication of ArduPilot's pre-arm logic (ArduPilot stays the
  final authority);
- no generic autopilot health framework (ArduPilot-first only; no
  generic-MAVLink health support claim);
- no real-aircraft validation or support claim;
- no direct sensor-calibration automation;
- no parameter auto-tuning;
- no replacement for ArduPilot failsafes;
- no runtime changes in this research phase.

## H. Open questions

1. Which estimator message is reliable across ArduPilot versions?
   (EKF_STATUS_REPORT is sent and stable in the researched checkout;
   ESTIMATOR_STATUS is not sent at all — confirm across the versions
   mavctl supports.)
2. How to map EKF flags to stable agent-friendly states
   (`healthy | degraded | unknown`) — which flag combinations are
   degraded vs fatal, and does the mapping hold across EKF2/EKF3/DCM
   backends?
3. Whether STATUSTEXT can be safely normalized (prefix + severity only),
   and how chunked text assembly behaves across dialect versions?
4. How to detect true pre-arm readiness without parsing unstable text —
   is `MAV_CMD_RUN_PREARM_CHECKS` ACCEPTED + a bounded STATUSTEXT
   correlation window a defensible "checks ran, no failure reported"
   signal, given absence-of-error caveats?
5. What is safe to hard-reject, ever? (The boot-time `unknown` case and
   failsafe-driven `system_status` transitions argue for a very small
   set.)
6. Multi-EKF / multi-GPS semantics: EKF_STATUS_REPORT covers the
   *configured* primary estimator (AP_AHRS aggregate); GPS2_RAW exists
   separately — how should blended/secondary sources surface?
7. Mission start vs arm guard integration: if a hard guard ever exists,
   does mission start inherit the same estimator policy as arm, given
   ArduCopter's mission start handler requires its own readiness?
