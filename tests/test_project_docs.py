"""Consistency tests binding README, packaging metadata and the publish workflow.

These tests keep the public entry documents honest about what mavctl can do
today: the README must document only real commands and state the PyPI channel
accurately (the published production release is 0.2.0 — no other version may
be claimed as released), and the publish workflow must refuse anything but
formal vX.Y.Z release tags.
"""

from __future__ import annotations

import re
from pathlib import Path

_ROOT = Path(__file__).resolve().parent.parent
_README = (_ROOT / "README.md").read_text(encoding="utf-8")
_ZH_README = (_ROOT / "README_ZH.md").read_text(encoding="utf-8")
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

# "mavctl X.Y.Z is published on production PyPI" — only 0.2.0 may ever match.
_VERSIONED_PUBLISHED_CLAIM = re.compile(
    r"\bmavctl\s+(\d+\.\d+\.\d+)\s+(?:is|has\s+been)\s+published\s+on\s+production",
    re.IGNORECASE,
)
# Chinese counterpart in README_ZH.md: "mavctl X.Y.Z 已发布到正式 PyPI".
_CHINESE_VERSIONED_PUBLISHED_CLAIM = re.compile(
    r"\bmavctl\s+(\d+\.\d+\.\d+)\s+已(?:经)?发布(?:到|至|于)正式"
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


def test_readme_states_pypi_install_availability() -> None:
    assert "## Install from PyPI" in _README
    assert "mavctl 0.2.0 is published on production PyPI" in _README
    # Stale pre-release wording must not survive the release.
    for stale in ("not available yet", "package is not published", "is being prepared"):
        assert stale not in _README, stale
    # The README speaks only about the production channel — rehearsal history
    # lives in docs/PUBLISHING.md.
    assert "TestPyPI" not in _README


def test_only_the_released_version_is_claimed_published() -> None:
    corpus = f"{_README}\n{_ZH_README}\n{_PUBLISHING}"
    claimed = set(_VERSIONED_PUBLISHED_CLAIM.findall(corpus))
    claimed |= set(_CHINESE_VERSIONED_PUBLISHED_CLAIM.findall(corpus))
    assert claimed == {"0.2.0"}, claimed


# -- Chinese README (README_ZH.md) -------------------------------------------
#
# The translation is bound to the same honesty invariants as the English
# README: real commands only, no unimplemented capability invocations, no
# force-arm examples, the safe SITL workflow, and the exact released version.


def test_chinese_readme_cross_links_the_english_readme() -> None:
    assert "[English](README.md)" in _ZH_README
    assert "README_ZH.md" in _README


def test_chinese_readme_documents_only_real_commands() -> None:
    for chunk in _code_chunks(_ZH_README):
        for line in chunk.splitlines():
            for command in _MAVCTL_INVOCATION.findall(line):
                assert command in _SUPPORTED_COMMANDS, f"README_ZH: mavctl {command}"


def test_chinese_readme_never_invokes_unimplemented_capabilities() -> None:
    for chunk in _code_chunks(_ZH_README):
        for line in chunk.splitlines():
            match = _UNSUPPORTED_INVOCATION.search(line)
            assert match is None, f"README_ZH: invoked {match.group(0)!r}"


def test_chinese_readme_never_shows_a_force_arm_invocation() -> None:
    match = _FORCE_ARM_INVOCATION.search(_ZH_README)
    assert match is None, f"README_ZH: {match.group(0)!r}"


def test_chinese_readme_quickstart_covers_the_safe_workflow() -> None:
    assert "--confirm" in _ZH_README
    assert "sim_vehicle.py" in _ZH_README
    assert "udp:127.0.0.1:14550" in _ZH_README
    assert "armed=true" in _ZH_README  # ACK-beats-heartbeat polling gate
    assert "mavctl rtl" in _ZH_README or "mavctl land" in _ZH_README
    assert "sitl" in _ZH_README.lower()


def test_chinese_readme_documents_the_pypi_install_channels() -> None:
    for command in _PYPI_INSTALL_COMMANDS:
        assert command in _ZH_README, command


def test_chinese_readme_states_pypi_install_availability() -> None:
    assert "## 从 PyPI 安装" in _ZH_README
    assert "mavctl 0.2.0 已发布到正式 PyPI" in _ZH_README
    # Stale pre-release wording must not survive the release.
    for stale in ("尚未发布", "暂未发布", "即将发布"):
        assert stale not in _ZH_README, stale
    # The README speaks only about the production channel — rehearsal history
    # lives in docs/PUBLISHING.md.
    assert "TestPyPI" not in _ZH_README


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


def test_testpypi_record_keeps_denying_production_equivalence() -> None:
    # Historical rehearsal record: it was a rehearsal, never a production release.
    assert "has been released on production PyPI" in _PUBLISHING


def test_publishing_doc_records_production_release() -> None:
    assert "mavctl 0.2.0 is published on production PyPI" in _PUBLISHING
    assert "Production PyPI release: mavctl 0.2.0" in _PUBLISHING
    assert "Released: 2026-08-26" in _PUBLISHING
    assert "GitHub Actions OIDC Trusted Publishing" in _PUBLISHING
    assert "wheel and sdist" in _PUBLISHING
    assert "clean-venv install" in _PUBLISHING
    # Forward-only versioning guidance for the next release cycle.
    assert "0.2.1.dev0" in _PUBLISHING
    assert "0.3.0.dev0" in _PUBLISHING
    # Whitespace-normalized: the sentence wraps across source lines.
    assert (
        "Never re-publish an existing version number"
        in " ".join(_PUBLISHING.split())
    )


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
