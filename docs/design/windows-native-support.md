# Design: native Windows daemon IPC support

Status: **design proposal — nothing in this document is implemented yet.**
Tracking issue: [#19 feat(platform): add native Windows daemon IPC support](https://github.com/LeaderOnePro/mavctl/issues/19).

## 1. Background and problem definition

mavctl runs a resident daemon that owns the MAVLink link; the CLI is a thin
client that talks to it over IPC. On macOS/Linux that IPC is a Unix domain
socket. The POSIX-specific surface is small and concentrated:

| Concern | Current implementation | Files |
| --- | --- | --- |
| IPC server | `asyncio.start_unix_server(..., path=daemon.sock)` | `daemon/server.py` |
| IPC client | `socket.socket(socket.AF_UNIX, SOCK_STREAM)` + `connect(path)` | `daemon/client.py` |
| Runtime discovery | fixed socket path under `MAVCTL_HOME` / `~/.mavctl` (`paths.socket_path()`) | `paths.py` |
| Stale-socket cleanup | `sock.unlink()` on startup/shutdown | `daemon/server.py`, `daemon/process.py` |
| Process lifecycle | `os.kill(pid, SIGTERM)` escalating to `SIGKILL`; `loop.add_signal_handler(SIGTERM/SIGINT)` | `daemon/process.py`, `daemon/server.py` |
| Liveness probe | pidfile + `os.kill(pid, 0)` signalability check | `daemon/process.py` |

The wire protocol itself (`daemon/wire.py`: one newline-delimited UTF-8 JSON
object per request/response) is transport-agnostic and needs no change. The
CLI layer (`cli/app.py`, `cli/render.py`) only knows "call a method, get a
`DaemonResponse`", so the exit-code contract, `--json` payloads, and all human
rendering are unaffected by any transport swap.

**Layers that must NOT be affected by this work:**

- `adapter/` — pymavlink is transport-agnostic to the GCS-side IPC; MAVLink
  command transactions (ACK correlation, quarantine, target locking) are
  untouched;
- `daemon/guards.py` — every safety decision (fresh-ground-evidence disarm,
  confirm gate, exit codes 4/5/6) is a pure function of `VehicleState`;
- models, exit-code contract, JSON wire payloads;
- future mission-protocol code — it should consume the same
  transport-agnostic RPC surface.

## 2. Goals

1. mavctl daemon, CLI, `status` / `telemetry`, and future mission features
   run on native Windows.
2. macOS/Linux keep the Unix domain socket with today's behaviour and
   security properties — no regression.
3. Local IPC security is preserved on every platform: other processes
   running under *different* identities must not be able to drive the daemon.
4. Windows ArduPilot SITL is explicitly **not** required by this effort.
5. No change to existing CLI commands, the exit-code contract, or JSON wire
   payloads.

## 3. Non-goals

- No remote daemon: the IPC endpoint is strictly same-machine.
- No exposure on non-loopback interfaces; no public TCP.
- No fixed, unauthenticated localhost port.
- No native Windows ArduPilot SITL in this effort (tracked separately later).
- No behavioural change to the pymavlink adapter.
- No weakening of existing safety guards, and no `type: ignore` /
  platform hacks / mypy-strict reductions to force Windows through.

## 4. Option comparison

### Option A — POSIX Unix socket + Windows named pipe

Keep the current Unix socket on macOS/Linux; on Windows serve the same
newline-JSON framing over a Windows named pipe.

| Criterion | Assessment |
| --- | --- |
| Security model | Named pipes offer OS ACL security — the closest analogue to Unix-socket semantics. |
| Python/asyncio feasibility | Python asyncio lacks a simple, stable, first-class server path suitable for this daemon protocol; the workarounds required are version-sensitive. |
| Testing & maintenance | Requires Windows-specific testing and ongoing maintenance; POSIX CI cannot exercise the pipe path at all. |

On the strength of those three rows alone — security is the only clear win,
and it is matched by Option B's token mechanism — **Option B is selected**
(see below).

### Option B — POSIX Unix socket + Windows loopback TCP + random token

Keep the Unix socket on macOS/Linux; on Windows the daemon binds
`127.0.0.1` with **port 0** (OS-assigned ephemeral port) and speaks the same
newline-JSON framing over TCP. Every daemon generates a per-instance random
token at startup; the CLI presents it on every RPC; the daemon verifies it in
constant time before dispatch.

| Criterion | Assessment |
| --- | --- |
| Python/asyncio feasibility | Excellent. `asyncio.start_server` / `asyncio.open_connection` are fully supported on Windows (Proactor loop by default). No HANDLE tricks, no version cliffs. |
| Windows-native dependencies | None beyond the stdlib. |
| CLI/daemon discovery | Runtime metadata file (see §5) replaces the fixed socket path. Slightly more moving parts than a pipe name, but versionable. |
| Same-machine security model | Loopback TCP alone is **not** sufficient — any local process could connect — which is exactly what the per-daemon token closes. Token knowledge ≈ authorization, equivalent to "can write to the socket file". |
| Token / permissions | CSPRNG token (≥ 256-bit entropy) + metadata file ACL'd to the current user. |
| Test complexity | Low: TCP transports test identically on every OS; the full server/client path can be exercised in POSIX unit tests too. |
| CI feasibility | A Windows smoke test needs only Python — no ArduPilot, no SITL. |
| Impact on client/server/process/paths | Endpoint + metadata abstraction (§6); `paths.py` gains a metadata store; `process.py` lifecycle is orthogonal and separately handled. |
| Impact on adapter/guards | None. |
| Long-term maintenance | Boring, stable APIs; token logic is ~100 lines total. |

### Decision: Option B

**Loopback TCP + per-daemon random token** is the recommended Windows
transport, with the Unix socket untouched on POSIX. Named pipes are the
conceptually purer match for Unix-socket semantics, but in Python they cost
asyncio-internals coupling and Windows-version sensitivity now, to save a
token check that Option B needs anyway for its metadata file. Option B gets
full test coverage on POSIX CI from day one.

## 5. Recommended mechanism (design sketch, not implemented)

### Daemon startup (Windows)

1. Bind `127.0.0.1`, **port 0** — the OS assigns an ephemeral port; the
   daemon never chooses one.
2. Generate the token with a CSPRNG: `secrets.token_urlsafe(32)`.
3. Write the runtime metadata file (under `runtime_dir()`, e.g.
   `daemon.json`) with a **versioned schema**:

   ```json
   {
     "schema_version": 1,
     "transport": "tcp",
     "host": "127.0.0.1",
     "port": 53124,
     "token": "<token_urlsafe>",
     "pid": 12345,
     "process_started_at": "<verifiable process start identity>",
     "protocol_version": 1
   }
   ```

   `process_started_at` records the operating system's view of when the
   daemon process started (on Windows: the process creation time; the exact
   wire format and acquisition API are implementation details). It must come
   from a verifiable process-start identity obtained from the OS — **not**
   from a wall-clock value the daemon simply wrote itself — so that a later
   `daemon stop` can confirm the PID still refers to *this* daemon before
   any forceful action (see Lifecycle below; PID values are reusable by the
   OS, and PID reuse is exactly why this field exists).

4. Restrict the metadata file to the current user. Acceptance definition
   (implementation may choose any Win32/ctypes mechanism that satisfies it):

   - file access must be limited to the **current user SID**;
   - `SYSTEM` / `Administrators` may retain access only where required by
     Windows policy;
   - `Users` / `Everyone` must **not** have read permission;
   - the Windows CI suite must include an **ACL verification test** asserting
     this.

   Threat model: token + ACL protect the daemon against *other OS-user
   identities* on the same machine. Processes running as the **same** user
   are not fully isolated — they can read the metadata file — which is the
   same trust position as Unix processes sharing a UID and therefore access
   to a Unix socket file. mavctl does not attempt to defend against a
   malicious same-user process; it defends the cross-user boundary.

### Client (Windows)

1. Read the metadata file; reject it if `schema_version` is unknown.
2. Open a TCP connection to `host:port` **only if** `host` is a loopback
   address (defence in depth against a tampered metadata file).
3. Send **every** one-shot RPC connection with the token carried in a
   dedicated, non-business envelope field. The TCP request envelope is
   pinned as:

   ```json
   {
     "method": "status",
     "params": {},
     "auth": {
       "token": "<per-daemon-secret>"
     }
   }
   ```

   `method`, `params`, every success result, and every existing business
   payload are **completely unchanged** by this field. Only the
   `LocalRpcClient` / `LocalRpcServer` transport layer reads or writes
   `auth`.

### Daemon-side verification

- Before dispatching any request, compare the presented token with
  `hmac.compare_digest` (constant time). A missing, malformed, or wrong
  token all receive the **same generic response** — no oracle about which
  check failed:

  ```json
  {
    "ok": false,
    "error": {
      "code": 1,
      "message": "daemon authentication failed",
      "detail": {}
    }
  }
  ```

  The connection is closed immediately after this response. The empty
  `detail` is deliberate: no hint, no reason, no echo of the presented
  token.

### Lifecycle, staleness, cleanup

**Graceful stop first.** `daemon stop` reads the endpoint and token from the
runtime metadata and sends an **authenticated `shutdown` RPC**. On a valid
shutdown request the daemon exits itself and removes the metadata file — no
external process termination is involved in the normal path.

**Force termination only after identity verification.** If the shutdown RPC
is unreachable, `daemon stop` must **not** blindly kill the PID from
metadata. OS PID values are *reusable*: after a daemon crash, an unrelated
process may later own the same PID, and killing it by number could terminate
an arbitrary victim. Therefore:

- the metadata is treated only as a stale-state hint — a failed
  ping/authenticated probe proves the endpoint is gone, **never** that the
  recorded PID is still the daemon or that it is safe to kill;
- before any platform-specific force termination, the PID identity must be
  re-verified: at minimum, the OS-reported **process start time** must match
  `process_started_at` in the metadata; where available, the executable
  path / command line is additionally checked;
- only a verified match may be force-terminated; if verification cannot
  confirm the identity, `daemon stop` must return a safe error with cleanup
  guidance (e.g. remove the stale metadata manually or re-run
  `daemon start`) and kill nothing;
- the next `daemon start` may safely overwrite stale metadata **after** it
  has confirmed the previous endpoint is dead (failed ping) and treated the
  recorded process as unverified — it starts a fresh daemon with a fresh
  token and fresh process identity; it never reuses or revives the old
  identity.

The PID-reuse hazard is the documented reason for every rule in this
subsection.

### Secrecy rules

- The token NEVER appears in logs, error messages, stdout/stderr,
  `status --json`, human output, or test snapshots. The metadata file is the
  only place it exists on disk.
- `status --json` continues to describe the *vehicle*; runtime transport
  metadata is internal and is not added to it.

### POSIX

macOS/Linux keep the Unix domain socket and today's code paths. The POSIX
Unix-socket transport **does not require authentication** — socket-file
permissions remain the access control. If the shared envelope carries the
optional `auth` field there, the transport layer simply ignores it; the
business payload and framing are identical on both platforms.

## 6. Minimal interface abstraction

```text
LocalIpcEndpoint        # where + how to reach/serve the daemon
    transport: "unix" | "tcp"
    path: Path | None          # unix
    host: str; port: int       # tcp (host must be loopback)
    token: str | None          # internal; never logged

LocalRpcServer          # asyncio server bound to an endpoint
    async serve(handler) -> None
    async stop() -> None

LocalRpcClient          # one-shot RPC used by the CLI
    call(method, params, timeout) -> DaemonResponse

RuntimeMetadataStore    # versioned read/write of daemon.json + pidfile
    write(metadata) / read() -> RuntimeMetadata | None
    clear()
```

Call graph and isolation boundary:

```text
CLI → LocalRpcClient → LocalIpcEndpoint → [unix socket | loopback TCP] → DaemonServer
                                                                            ↓
                                                     existing business handlers, guards,
                                                     adapter — transport-unaware
```

The daemon's business handlers (`ping`, `status`, `telemetry`, `arm`, …),
the guard framework, and the adapter never learn which transport carried a
request. `cli/app.py` keeps calling `call_daemon(method, params)`; only the
client's internals dispatch per endpoint.

## 7. Phased implementation plan

1. **Abstraction only** (no behaviour change): introduce
   `LocalIpcEndpoint` / `LocalRpcServer` / `LocalRpcClient` /
   `RuntimeMetadataStore`; rewire the existing Unix-socket server/client
   behind them; byte-identical behaviour on macOS/Linux.
2. **POSIX regression gate**: full existing test suite green with the
   abstraction in place; no Windows code paths exercised yet.
3. **Windows loopback authenticated transport**: port-0 bind, token
   generation, metadata file, constant-time verification.
4. **Windows lifecycle**: `daemon start/stop/status` on Windows —
   authenticated `shutdown` RPC as the primary stop path, PID-identity
   verification (process start time) before any forced termination, stale
   metadata handling, crash cleanup (replacing SIGTERM/SIGKILL assumptions).
5. **Cross-platform unit tests** (mock adapter; both transports on POSIX;
   auth-positive and auth-negative cases).
6. **GitHub Actions Windows smoke test**: `daemon start` with a mock/fake
   link or a skip-if-no-SITL guard, `status --json`, `daemon stop` on a
   `windows-latest` runner.
7. **(Separate, later)** Windows SITL or remote-SITL integration —
   deliberately decoupled from IPC work.

## 8. Test plan

- POSIX Unix-socket regression: every existing daemon/CLI test passes
  unchanged.
- Envelope compatibility: requests with `{"method", "params"}` and with
  `{"method", "params", "auth"}` produce identical business results; the
  `auth` field never leaks into params, results, or payloads; the POSIX
  transport ignores an optional `auth` field.
- Windows TCP endpoint: binds loopback only; refuses to start with a
  non-loopback host.
- Token secrecy: token absent from all logs, errors, `status --json`,
  exception text, and test snapshots (asserted by scanning outputs).
- Auth enforcement: missing / wrong / malformed token → the **same generic
  auth-failure response** (`{"ok": false, "error": {"code": 1, "message":
  "daemon authentication failed", "detail": {}}}`), connection closed, no
  oracle distinguishing the failure modes (asserted by comparing all three
  responses byte-for-byte).
- Constant-time comparison used (implementation asserts `compare_digest`
  path in unit tests).
- Graceful shutdown: an authenticated `shutdown` RPC makes the daemon exit
  and clean up its metadata; no process kill involved.
- Stop must not kill an unrelated process: a stale metadata file whose PID
  identity fails verification (reused PID / mismatched start time) results
  in a safe error with cleanup guidance — no signal is sent (asserted with
  a fake process table).
- Metadata ACL (Windows CI): the metadata file grants only the current user
  SID; `Users` / `Everyone` have no read access.
- Stale metadata: dead-pid / closed-port metadata → clean exit 3; `daemon
  start` recovers by overwriting after confirming the old endpoint is dead.
- `daemon start` / `daemon stop` round-trip on both transports.
- Two concurrent CLI clients against one daemon (fast handlers stay
  concurrent).
- Windows CI smoke: mock-adapter `status --json` round-trip on a
  `windows-latest` runner — no ArduPilot/SITL prerequisite for IPC
  validation.

## 9. README impact

**None in this round.** The README platform statement is only updated after
the Windows implementation lands and the Windows CI smoke test passes; until
then the documents continue to describe macOS/Linux support. This mirrors
the repo's documentation-honesty policy (see `tests/test_project_docs.py`).
