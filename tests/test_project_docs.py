"""Consistency tests binding README, packaging metadata and the publish workflow.

These tests keep the public entry documents honest about what mavctl can do
today: the README must document only real commands and describe the PyPI
channel conditionally (never claiming an already-published production release
or equating TestPyPI rehearsals with one), and the publish workflow must
refuse anything but formal vX.Y.Z release tags.
"""

from __future__ import annotations

import re
from pathlib import Path

_ROOT = Path(__file__).resolve().parent.parent
_README = (_ROOT / "README.md").read_text(encoding="utf-8")
_PYPROJECT = (_ROOT / "pyproject.toml").read_text(encoding="utf-8")
_PUBLISHING = (_ROOT / "docs" / "PUBLISHING.md").read_text(encoding="utf-8")
_WORKFLOW = (_ROOT / ".github" / "workflows" / "publish.yml").read_text(encoding="utf-8")

_SUPPORTED_COMMANDS = frozenset(
    {"status", "telemetry", "arm", "disarm", "mode", "takeoff", "land", "rtl", "daemon"}
)

_MAVCTL_INVOCATION = re.compile(r"mavctl\s+([A-Za-z][A-Za-z0-9_-]*)")
# Unimplemented capabilities may be *named* as bare words, never invoked.
_UNSUPPORTED_INVOCATION = re.compile(
    r"mavctl\s+(mission|geofence|fence|rally|params?)\b", re.IGNORECASE
)
_FORCE_ARM_INVOCATION = re.compile(r"mavctl\s+arm\b[^\n]*--force", re.IGNORECASE)

_PYPI_INSTALL_COMMANDS = ("uv tool install mavctl", "uvx mavctl", "pipx install mavctl")


# A claim that production PyPI already serves mavctl — never allowed, the
# release is only *prepared* until tag v0.2.0 is pushed and published.
_PUBLISHED_ON_PYPI_CLAIM = re.compile(
    r"\b(?:is|are|has\s+been)\s+(?:now\s+)?"
    r"(?:available|published|released|live)\s+on\s+PyPI\b",
    re.IGNORECASE,
)

_FENCE_LINE = re.compile(r"^\s*(`{3,})(.*)$")


def _prose_and_code(text: str) -> list[tuple[str, str]]:
    """Split a Markdown document into alternating ``("prose"|"code", chunk)`` parts."""
    segments: list[tuple[str, str]] = []
    open_len: int | None = None
    kind = "prose"
    buf: list[str] = []
    for line in text.splitlines():
        match = _FENCE_LINE.match(line)
        if match is None:
            buf.append(line)
            continue
        ticks, info = len(match.group(1)), match.group(2).strip()
        segments.append((kind, "\n".join(buf)))
        if open_len is None:
            open_len, kind, buf = ticks, "code", [line]
        elif not info and ticks >= open_len:
            open_len, kind, buf = None, "prose", [line]
        else:
            buf.append(line)
    segments.append((kind, "\n".join(buf)))
    return [(chunk_kind, body) for chunk_kind, body in segments if body.strip()]


def _code_chunks(text: str) -> list[str]:
    """Fenced block bodies plus inline code spans — the runnable-looking text."""
    chunks: list[str] = []
    for kind, body in _prose_and_code(text):
        if kind == "code":
            chunks.append(body)
        else:
            chunks.extend(re.findall(r"`([^`\n]+)`", body))
    return chunks


# -- README command surface --------------------------------------------------


def test_readme_documents_only_real_commands() -> None:
    for chunk in _code_chunks(_README):
        # Match per line: invocations are single-line, and a fenced block such
        # as "cd mavctl\nuv sync" must not read as "mavctl uv".
        for line in chunk.splitlines():
            for command in _MAVCTL_INVOCATION.findall(line):
                assert command in _SUPPORTED_COMMANDS, f"README: mavctl {command}"


def test_readme_never_invokes_unimplemented_capabilities() -> None:
    for chunk in _code_chunks(_README):
        for line in chunk.splitlines():
            match = _UNSUPPORTED_INVOCATION.search(line)
            assert match is None, f"README: invoked {match.group(0)!r}"


def test_readme_never_shows_a_force_arm_invocation() -> None:
    match = _FORCE_ARM_INVOCATION.search(_README)
    assert match is None, f"README: {match.group(0)!r}"


def test_readme_quickstart_covers_the_safe_workflow() -> None:
    assert "--confirm" in _README
    assert "sim_vehicle.py" in _README
    assert "udp:127.0.0.1:14550" in _README
    assert "armed=true" in _README  # ACK-beats-heartbeat polling gate
    assert "mavctl rtl" in _README or "mavctl land" in _README
    assert "sitl" in _README.lower()


def test_readme_documents_the_pypi_install_channels() -> None:
    for command in _PYPI_INSTALL_COMMANDS:
        assert command in _README, command


def test_readme_pypi_wording_is_release_aware() -> None:
    # Instructions are conditional on the actual release…
    assert re.search(
        r"after the\s+PyPI release is published", _README, re.IGNORECASE
    ), "install commands must be gated on the release being published"
    # …TestPyPI rehearsals must never be presented as production releases…
    assert re.search(
        r"rehearsal artifacts are not production releases", _README, re.IGNORECASE
    )
    equivalence = re.search(
        r"TestPyPI[^.\n]*(?:same as|identical to|equals)", _README, re.IGNORECASE
    )
    assert equivalence is None, equivalence.group(0) if equivalence else ""
    # …and no wording may claim mavctl is already served by production PyPI.
    claim = _PUBLISHED_ON_PYPI_CLAIM.search(_README)
    assert claim is None, claim.group(0) if claim else ""


# -- packaging metadata ------------------------------------------------------


def test_pyproject_packaging_metadata_is_release_ready_shape() -> None:
    # Regex-scanned (not tomllib) so the suite also runs under Python 3.10.
    def metadata_line(pattern: str) -> re.Match[str] | None:
        return re.search(pattern, _PYPROJECT, re.MULTILINE)

    assert metadata_line(r'^name = "mavctl"$')
    assert metadata_line(r'^readme = "README\.md"$')
    assert metadata_line(r'^license = "MIT"$')
    # The formal v0.2.0 release version — plain PEP 440, no dev suffix.
    assert metadata_line(r'^version = "0\.2\.0"$')
    assert metadata_line(r'^mavctl = "[^"]+"$')
    license_text = (_ROOT / "LICENSE").read_text(encoding="utf-8")
    assert "MIT License" in license_text
    assert "LeaderOnePro" in license_text


# -- publishing docs ---------------------------------------------------------


def test_publishing_doc_states_production_release_pending() -> None:
    assert "prepared, not yet published" in _PUBLISHING
    assert "has been released on production PyPI" in _PUBLISHING


def test_publishing_doc_tracks_pre_v020_release_state() -> None:
    assert "## Release state before v0.2.0" in _PUBLISHING
    assert (
        "tag v0.2.0 has been pushed successfully" in _PUBLISHING
    ), "the doc must gate publication on the tag actually being pushed"
    claim = _PUBLISHED_ON_PYPI_CLAIM.search(_PUBLISHING)
    assert claim is None, claim.group(0) if claim else ""


def test_testpypi_rehearsal_is_recorded() -> None:
    assert "## TestPyPI rehearsal record" in _PUBLISHING
    assert "2026-08-26" in _PUBLISHING
    assert "0.2.0.dev0" in _PUBLISHING
    assert "0.2.0.dev1" in _PUBLISHING


def test_publishing_doc_contains_no_token_material() -> None:
    # Real PyPI/TestPyPI API tokens start with "pypi-" — never even in examples.
    assert "pypi-" not in _PUBLISHING
    # Credential assignments may appear only with an obvious <PLACEHOLDER> value.
    assert re.search(r"UV_PUBLISH_PASSWORD=(?!<[A-Z_]+>)", _PUBLISHING) is None
    # Wherever the __token__ username appears, any following password assignment
    # must also be a placeholder — never a concrete credential.
    for match in re.finditer(r"UV_PUBLISH_USERNAME=__token__", _PUBLISHING):
        tail = _PUBLISHING[match.end() :]
        follow = re.search(r"UV_PUBLISH_PASSWORD=(\S*)", tail)
        if follow is not None:
            assert follow.group(1).startswith("<"), follow.group(0)


# -- publish workflow --------------------------------------------------------


def test_publish_workflow_accepts_only_formal_release_tags() -> None:
    match = re.search(r"RELEASE_TAG_PATTERN:\s*'([^']+)'", _WORKFLOW)
    assert match is not None, "workflow must define RELEASE_TAG_PATTERN"
    pattern = re.compile(match.group(1))
    assert pattern.fullmatch("v0.2.0")
    assert pattern.fullmatch("v10.20.30")
    for rejected in ("v0.2.0-phase2", "v0.2.0.dev0", "v1.2", "vX.Y.Z"):
        assert pattern.fullmatch(rejected) is None, rejected
    assert '"v*"' in _WORKFLOW


def test_publish_workflow_uses_oidc_trusted_publishing() -> None:
    assert re.search(r"id-token:\s*write", _WORKFLOW)
    assert "pypa/gh-action-pypi-publish" in _WORKFLOW
    assert "PYPI_API_TOKEN" not in _WORKFLOW
    assert "password:" not in _WORKFLOW


def test_publish_workflow_runs_gates_before_building_and_publishing() -> None:
    order = [
        _WORKFLOW.index("ruff check"),
        _WORKFLOW.index("mypy"),
        _WORKFLOW.index('pytest -m "not sitl"'),
        _WORKFLOW.index("uv build"),
        _WORKFLOW.index("gh-action-pypi-publish"),
    ]
    assert order == sorted(order)
