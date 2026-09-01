# Publishing mavctl to PyPI

Status: **released** — mavctl 0.2.0 is published on production PyPI
(2026-08-26). This document remains the operator runbook for every future
release.

## Package name

The PyPI project name `mavctl` was confirmed unoccupied on 2026-08-26 (the
PyPI JSON API returned 404 for `pypi.org/pypi/mavctl/json`).

> This is historical information — **re-confirm immediately before the first
> release.**

## Version scheme

- Between releases, `pyproject.toml` carries the next PEP 440 development
  version; for a release commit the `.devN` suffix is dropped before tagging.
- Git tags come in two kinds:
  - `v0.2.0-phase2` is a **Git milestone tag, not a PyPI release tag.**
  - Release tags are strictly `vX.Y.Z` (e.g. `v0.2.0`). The publish workflow
    rejects every other shape and additionally cross-checks that the tag
    equals the package version.
- The published `0.2.0` is immutable on PyPI. Future cycles move strictly
  forward from it: develop against `0.2.1.dev0` (patch) or `0.3.0.dev0`
  (feature/minor) and release later as `0.2.1` / `0.3.0`. Never re-publish an
  existing version number.

## Production release record

Production PyPI release: mavctl 0.2.0

- Released: 2026-08-26
- Publishing method: GitHub Actions OIDC Trusted Publishing
- Published artifacts: wheel and sdist
- Verification: clean-venv install, `mavctl --help`, `mavctl daemon --help`

## Release state before v0.2.1

This release branch prepares mavctl 0.2.1. Production PyPI publication has
not happened until this branch is merged to main and the v0.2.1 tag is pushed.

## First-release checklist (v0.2.0)

Done:

- Re-confirmed `mavctl` was unoccupied on production PyPI before preparation
  started.
- Registered the PyPI Trusted Publisher as a pending publisher (project
  `mavctl`, owner `LeaderOnePro`, repository `mavctl`, workflow filename
  `publish.yml`, environment empty).
- Rehearsed on TestPyPI successfully (see the record below); the rehearsal
  token was revoked afterwards.
- Landed the release commit changing `version` to `0.2.0` on the
  `release/0.2.0` branch and opened the release PR.

Completed on 2026-08-26, strictly in this order:

1. Merged the release PR into main.
2. Created and pushed annotated tag `v0.2.0`.
3. The GitHub Actions run (`Publish to PyPI`) validated tag/version and
   published over OIDC.
4. Verified installation in a clean environment with `uvx mavctl --help`.

For the next release, start from the version-scheme rules above and reuse
this checklist with the new version number.

## Trusted Publisher configuration (manual, one-time)

The GitHub Actions workflow authenticates with OIDC only — there are no PyPI
token secrets anywhere. You must register the trust relationship on the PyPI
side once:

1. Sign in at pypi.org. For a project that does not exist yet, use
   *Account → Publishing → Add a new pending publisher*; for an existing
   project, use its *Settings → Publishing*.
2. Fill in exactly:
   - **PyPI project name:** `mavctl`
   - **Owner:** `LeaderOnePro`
   - **Repository:** `mavctl`
   - **Workflow filename:** `publish.yml`
   - **Environment:** leave empty — the workflow declares no environment.
     If you ever add an `environment:` to the job, the name configured here
     must match exactly.
3. Repeat on test.pypi.org if you rehearse there with the same workflow.

A mismatch in any of these fields makes the OIDC claim invalid and the
publish step fails.

## Local build check

```bash
uv build
ls -la dist/
tar tzf dist/mavctl-*.tar.gz | head
python3 -m zipfile -l dist/mavctl-*.whl
```

Optional metadata lint:

```bash
uvx twine check dist/*
```

Smoke-test the wheel without touching your development environment:

```bash
uv venv /tmp/mavctl-smoke
VIRTUAL_ENV=/tmp/mavctl-smoke uv pip install dist/mavctl-*.whl
/tmp/mavctl-smoke/bin/mavctl --help
```

## TestPyPI rehearsal

TestPyPI is a separate registry with separate accounts; nothing published
there affects real PyPI. Recommended dry run before the first real release:

1. Create/configure a pending publisher on test.pypi.org with the same
   Owner / Repository / workflow-filename values as above.
2. Either point a throwaway branch of the publish step at TestPyPI
   (`with: repository-url: https://test.pypi.org/legacy/`) and push a
   rehearsal tag, or upload the locally built artifacts by hand using a
   personal TestPyPI token (this manual path uses a token; the production
   workflow never does):
   ```bash
   uv build
   export UV_PUBLISH_USERNAME=__token__
   export UV_PUBLISH_PASSWORD=<TEST_PYPI_TOKEN>
   uv publish --publish-url https://test.pypi.org/legacy/
   ```
3. Install the result into a scratch venv from TestPyPI's index and run
   `mavctl --help`.

## TestPyPI rehearsal record

- Rehearsal completed successfully on 2026-08-26.
- Tested package version: `0.2.0.dev0`; both wheel and sdist uploaded to
  TestPyPI.
- A clean temporary virtual environment installed `mavctl==0.2.0.dev0` from
  TestPyPI, with dependencies resolved from PyPI.
- Smoke checks passed: `mavctl --help` and `mavctl daemon --help`.
- This does **not** mean mavctl has been released on production PyPI;
  production publication happened later — see the production release record
  above.
- TestPyPI does not permit re-uploading the same distribution version, so a
  future rehearsal needs a new version such as `0.2.0.dev1`.

## Post-release verification

Only after the Actions run succeeds and the release shows on pypi.org:

```bash
uvx mavctl --help
```

Then connect the installed tool to SITL once
(`mavctl daemon start --connect udp:127.0.0.1:14550`) to confirm the wheel
ships a working entrypoint.
