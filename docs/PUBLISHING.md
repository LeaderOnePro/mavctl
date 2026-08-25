# Publishing mavctl to PyPI

Status: **prepared, not yet published**. This is the operator runbook for
cutting real releases; none of it has executed end-to-end yet.

## Package name

The PyPI project name `mavctl` was confirmed unoccupied on 2026-08-26 (the
PyPI JSON API returned 404 for `pypi.org/pypi/mavctl/json`).

> This is historical information — **re-confirm immediately before the first
> release.**

## Version scheme

- `pyproject.toml` carries `0.2.0.dev0`: a PEP 440 development version marking
  unreleased work toward 0.2.0 (main already contains the phase-2 safety
  hardening; `0.1.0` was never published to PyPI).
- Git tags come in two kinds:
  - `v0.2.0-phase2` is a **Git milestone tag, not a PyPI release tag.**
  - Release tags are strictly `vX.Y.Z` (e.g. `v0.2.0`). The publish workflow
    rejects every other shape and additionally cross-checks that the tag
    equals the package version, so the `.dev0` suffix must be dropped in a
    release commit before tagging.

## First-release checklist

1. Re-confirm `mavctl` is still free on PyPI.
2. Land a release commit changing `version = "X.Y.Z"` (no suffix).
3. Register the PyPI Trusted Publisher (below) — before pushing any tag.
4. Validate locally: `uv build`, inspect `dist/`, smoke-test the wheel.
5. Rehearse on TestPyPI (below).
6. Push the release tag:
   ```bash
   git tag vX.Y.Z
   git push origin vX.Y.Z
   ```
7. Watch the GitHub Actions run (`Publish to PyPI`), then verify post-release
   (below).

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
- This does **not** mean mavctl has been released on production PyPI; the
  status at the top of this document still applies.
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
