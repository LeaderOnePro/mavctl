"""Consistency tests binding README, packaging metadata and the publish workflow.

These tests keep the public entry documents honest about what mavctl can do
today: the README must document only real commands and state the PyPI channel
accurately (the published production releases are 0.2.0, 0.2.1, 0.3.0 and
0.4.0 — no other version may be claimed as released), and the publish
workflow must refuse anything but formal vX.Y.Z release tags.
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
_SKILLS_ACCEPTANCE = (_ROOT / "docs" / "SKILLS_CLI_ACCEPTANCE.md").read_text(encoding="utf-8")
_AGENTS_MD = (_ROOT / "AGENTS.md").read_text(encoding="utf-8")
_PHASE3B_DESIGN = (_ROOT / "docs" / "design" / "mission-execution-phase3b.md").read_text(
    encoding="utf-8"
)

_SUPPORTED_COMMANDS = frozenset(
    {"status", "telemetry", "arm", "disarm", "mode", "takeoff", "land", "rtl",
     "daemon", "mission", "operation"}
)

_MAVCTL_INVOCATION = re.compile(r"mavctl\s+([A-Za-z][A-Za-z0-9_-]*)")
# Mission execution beyond `start` is Phase 3B: `mavctl mission start`
# itself is implemented (mock- and ArduCopter-SITL-validated) and may
# appear in runnable text; pause/resume/stop/set-current never may.
_MISSION_EXECUTION_INVOCATION = re.compile(
    r"mavctl\s+mission\s+(pause|resume|stop|set-current)\b", re.IGNORECASE
)
# Unimplemented capabilities may be *named* as bare words, never invoked.
_UNSUPPORTED_INVOCATION = re.compile(
    r"mavctl\s+(geofence|fence|rally|params?)\b", re.IGNORECASE
)
_FORCE_ARM_INVOCATION = re.compile(r"mavctl\s+arm\b[^\n]*--force", re.IGNORECASE)

_PYPI_INSTALL_COMMANDS = ("uv tool install mavctl", "uvx mavctl", "pipx install mavctl")

# "mavctl X.Y.Z is published on production PyPI" — only
# 0.2.0/0.2.1/0.3.0/0.4.0 may match.
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


def test_readme_states_mission_surface_is_sitl_validated_not_flight_proven() -> None:
    # Honesty gate (updated after the Phase 3A SITL acceptance): the READMEs
    # must state that the mission transfer is ArduPilot SITL validated only
    # (no real-aircraft claim) and that there is no mission execution command.
    for readme in (_README, _ZH_README):
        assert "mission upload" in readme
        assert "SITL validated" in readme or "SITL 验证" in readme
        assert "real-aircraft" in readme or "真实飞机" in readme


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
    assert "mavctl 0.4.0 is published on production PyPI" in _README
    # Stale pre-release wording must not survive the release.
    for stale in ("not available yet", "package is not published", "is being prepared"):
        assert stale not in _README, stale
    # The README speaks only about the production channel — rehearsal history
    # lives in docs/PUBLISHING.md.
    assert "TestPyPI" not in _README


def test_only_the_released_version_is_claimed_published() -> None:
    # Production releases may be claimed as published: 0.2.0, 0.2.1,
    # 0.3.0 and 0.4.0. No future version (0.4.1 / 0.5.0 / …) may ever
    # appear as a published claim.
    corpus = f"{_README}\n{_ZH_README}\n{_PUBLISHING}"
    claimed = set(_VERSIONED_PUBLISHED_CLAIM.findall(corpus))
    claimed |= set(_CHINESE_VERSIONED_PUBLISHED_CLAIM.findall(corpus))
    assert claimed == {"0.2.0", "0.2.1", "0.3.0", "0.4.0"}, claimed
    for future in ("0.4.1", "0.5.0"):
        assert not re.search(
            rf"\bmavctl\s+{re.escape(future)}\s+(?:is|has\s+been)\s+published",
            corpus, re.IGNORECASE,
        ), future
        assert not re.search(
            rf"\bmavctl\s+{re.escape(future)}\s+已(?:经)?发布", corpus
        ), future


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
    assert "mavctl 0.4.0 已发布到正式 PyPI" in _ZH_README
    # Stale pre-release wording must not survive the release.
    for stale in ("尚未发布", "暂未发布", "即将发布"):
        assert stale not in _ZH_README, stale
    # The README speaks only about the production channel — rehearsal history
    # lives in docs/PUBLISHING.md.
    assert "TestPyPI" not in _ZH_README


# -- skills CLI install commands ---------------------------------------------

_SKILLS_INSTALL_COMMAND = "npx skills add LeaderOnePro/mavctl -y -g"
_SKILLS_FORBIDDEN_IN_READMES = (
    "skills@",
    "-s mavctl-flight",
    "-a claude-code",
    "--all",
    "--copy",
    "/path/to/mavctl",
    "mkdir -p .claude",
    "ln -s",
)


def test_readmes_document_the_single_global_skill_install() -> None:
    # Product decision: the READMEs expose exactly one generic install
    # command. Target-specific, project-local and bulk-copy variants live in
    # docs/SKILLS_CLI_ACCEPTANCE.md only.
    for readme in (_README, _ZH_README):
        assert _SKILLS_INSTALL_COMMAND in readme
        for forbidden in _SKILLS_FORBIDDEN_IN_READMES:
            assert forbidden not in readme, forbidden
        for line in readme.splitlines():
            for invocation in re.findall(r"npx skills add.*", line):
                assert invocation.strip() == _SKILLS_INSTALL_COMMAND, invocation


def test_readmes_state_the_skill_install_semantics() -> None:
    # Minimal semantics both READMEs must carry: the Skill install is
    # independent of the Python CLI install; the skills CLI picks supported
    # runtimes by environment/configuration; no promise of covering every
    # runtime; the acceptance record is linked.
    independence = ("independent of installing", "相互独立")
    runtime_choice = ("environment and configuration", "当前环境与配置")
    for readme in (_README, _ZH_README):
        assert any(phrase in readme for phrase in independence)
        assert any(phrase in readme for phrase in runtime_choice)
        assert "docs/SKILLS_CLI_ACCEPTANCE.md" in readme
        for overpromise in ("every agent", "all agents", "所有 agent", "全部 agent"):
            assert overpromise not in readme, overpromise


def test_skills_cli_acceptance_records_global_install_and_matrix() -> None:
    assert _SKILLS_INSTALL_COMMAND in _SKILLS_ACCEPTANCE
    assert "1.5.23" in _SKILLS_ACCEPTANCE
    assert "PromptScript" in _SKILLS_ACCEPTANCE
    for agent_id in ("zcode", "claude-code", "codex", "pi"):
        assert agent_id in _SKILLS_ACCEPTANCE, agent_id
    assert "--all --copy" in _SKILLS_ACCEPTANCE


def test_skills_cli_acceptance_scopes_technical_paths_to_the_record() -> None:
    # Project-local install, bulk copy and manual linking are documented as
    # technical reference only — never as README onboarding.
    assert "project-local" in _SKILLS_ACCEPTANCE
    assert "manual-linking" in _SKILLS_ACCEPTANCE
    assert "手动 symlink" in _SKILLS_ACCEPTANCE
    assert "onboarding" in _SKILLS_ACCEPTANCE
    assert "技术参考" in _SKILLS_ACCEPTANCE


# -- packaging metadata ------------------------------------------------------


def test_pyproject_packaging_metadata_is_release_ready_shape() -> None:
    # Regex-scanned (not tomllib) so the suite also runs under Python 3.10.
    def metadata_line(pattern: str) -> re.Match[str] | None:
        return re.search(pattern, _PYPROJECT, re.MULTILINE)

    assert metadata_line(r'^name = "mavctl"$')
    assert metadata_line(r'^readme = "README\.md"$')
    assert metadata_line(r'^license = "MIT"$')
    # The current development version (PEP 440 dev suffix); the release
    # version is promoted on a release branch before tagging. Published
    # production releases: 0.2.0, 0.2.1, 0.3.0 and 0.4.0 (see the claim
    # test above).
    assert metadata_line(r'^version = "0\.4\.1\.dev0"$')
    assert metadata_line(r'^mavctl = "[^"]+"$')
    license_text = (_ROOT / "LICENSE").read_text(encoding="utf-8")
    assert "MIT License" in license_text
    assert "LeaderOnePro" in license_text


# -- 0.2.1 release preparation ----------------------------------------------


def test_development_version_is_041_dev0() -> None:
    # The 0.4.1.dev0 development cycle (mission help text fix) is
    # unreleased: 0.4.0 is the newest production release with a full
    # record, no v0.4.1 tag exists, and the dev state section records the
    # branch.
    assert re.search(r'(?m)^version = "0\.4\.1\.dev0"$', _PYPROJECT) is not None
    assert re.search(r'(?m)^version = "0\.4\.0"$', _PYPROJECT) is None
    assert "## Production release record: 0.4.0" in _PUBLISHING
    assert "## Development state: 0.4.1.dev0" in _PUBLISHING
    assert "## Release state before v0.4.0" not in _PUBLISHING


def test_publishing_doc_records_030_production_release() -> None:
    # The 0.3.0 production record must capture the release facts: date,
    # OIDC method, artifacts, GitHub Release, verification steps, feature
    # scope, validation, provenance, and the next-dev-version guidance.
    normalized = " ".join(_PUBLISHING.split())
    assert "## Production release record: 0.3.0" in _PUBLISHING
    assert "Released: 2026-09-28" in normalized
    assert "Version: 0.3.0" in normalized
    assert "GitHub Actions OIDC Trusted Publishing" in normalized
    assert "wheel and sdist" in normalized
    assert "GitHub Release: v0.3.0" in normalized
    for verification in (
        "production PyPI JSON metadata", "clean virtual-environment installation",
        "`mavctl --version`", "`mavctl --help`", "`mavctl mission --help`",
        "`mavctl daemon --help`",
    ):
        assert verification in normalized, verification
    assert "`mavctl mission upload`" in normalized
    assert "`mavctl mission download`" in normalized
    assert "`mavctl mission clear`" in normalized
    assert "out of scope" in normalized  # mission execution / AUTO / start
    assert "upload → download → clear → empty read-back" in normalized
    assert "shared MAVProxy loopback" in normalized
    assert "isolated no-MAVProxy loopback" in normalized
    assert "4c98c9221a" in _PUBLISHING
    assert "AP_FWVersion.h" in _PUBLISHING
    assert "no real-aircraft validation or support claim" in normalized


def test_publishing_doc_records_040_production_release() -> None:
    # The 0.4.0 production record must capture the release facts: date,
    # OIDC method, artifacts, GitHub Release, merge commit, verification,
    # scope limits, validation, and provenance.
    normalized = " ".join(_PUBLISHING.split())
    assert "## Production release record: 0.4.0" in _PUBLISHING
    assert "Released: 2026-10-04" in normalized
    assert "Version: 0.4.0" in normalized
    assert "GitHub Actions OIDC Trusted Publishing" in normalized
    assert "GitHub Release: v0.4.0" in normalized
    assert "Merge commit: `7c523da`" in normalized
    assert "`mavctl mission start --confirm [--wait] [--timeout]`" in normalized
    assert "`mavctl operation get <id>`" in normalized
    # scope honesty: the unimplemented execution surface stays named
    assert (
        "mission pause/resume/stop/set-current and operation cancel remain"
        in normalized
    )
    assert "out of scope" in normalized
    assert "passed twice" in normalized  # SITL suite, twice consecutively
    assert "4c98c9221a" in normalized
    assert "no real-aircraft validation or support claim" in normalized


def test_publishing_doc_gives_next_dev_version_guidance() -> None:
    # The next development version must move forward from 0.3.0; existing
    # version numbers are never re-published.
    normalized = " ".join(_PUBLISHING.split())
    assert "0.3.1.dev0" in normalized
    assert "0.4.0.dev0" in normalized
    assert "Never re-publish an existing version number" in normalized
    # no token material in the release record or guidance
    assert not re.search(r"(?i)\b(pypi[_-]?api[_-]?token|gh[_-]?token|password)\b", normalized)


def test_readme_mission_surface_lists_exactly_the_implemented_commands() -> None:
    # The READMEs document exactly the four implemented mission commands
    # (upload / download / clear / start) and never show a runnable
    # pause/resume/stop/set-current command.
    for readme in (_README, _ZH_README):
        assert "mavctl mission upload" in readme
        assert "mavctl mission download" in readme
        assert "mavctl mission clear" in readme
        assert "mavctl mission start" in readme
        assert "mavctl operation get" in readme
        assert _MISSION_EXECUTION_INVOCATION.findall(readme) == []


def test_readme_zh_mission_start_paragraph_matches_english_semantics() -> None:
    """README_ZH must mirror the README.md mission-start semantics:
    implemented (mock- and ArduCopter-SITL-validated), may transition to
    AUTO, no implicit arm/takeoff, already_running idempotent, --wait
    bounded to the start milestone, SITL conformance run (automated,
    loopback) — and no residual "SITL conformance pending" claim."""

    # the old negations are gone
    assert "目前没有 mission start" not in _ZH_README
    assert "没有 mission start/execution" not in _ZH_README
    assert "尚未运行" not in _ZH_README

    # implemented + mock- and ArduCopter-SITL-validated (conformance ran)
    assert "mavctl mission start --confirm" in _ZH_README
    assert "0.4.0" in _ZH_README
    # the dev suffix is gone from the released docs
    assert "0.4.0.dev0" not in _ZH_README
    assert "ArduCopter SITL 验证" in _ZH_README
    assert "SITL execution conformance 已在隔离" in _ZH_README
    assert "tests/test_mission_execution_sitl.py" in _ZH_README

    # the command-block annotations state the same validation scope
    assert "mock- and ArduCopter SITL-validated" in _README
    assert "已完成 mock 与 ArduCopter SITL 验证" in _ZH_README

    # execution-command semantics: may transition to AUTO; no implicit
    # arm/takeoff; idempotent; milestone boundary
    assert "切换到 AUTO" in _ZH_README
    assert "不会 arm 电机" in _ZH_README
    assert "隐式起飞" in _ZH_README
    assert "already_running" in _ZH_README
    assert "不等待整趟任务完成" in _ZH_README

    # the unsupported surface stays honestly named (prose only, no commands)
    assert "pause/resume/stop/set-current" in _ZH_README
    assert "mavctl mission pause" not in _ZH_README
    assert "mavctl mission stop" not in _ZH_README


def test_phase3b_design_doc_records_completed_sitl_validation() -> None:
    """The Phase 3B design must state that mission-start execution
    conformance ran (never "pending"), keep the validation boundaries
    explicit (ArduCopter SITL only, no real-aircraft claim, locally
    modified checkout provenance), and keep the remaining Phase 3B
    surface unimplemented."""
    normalized = " ".join(_PHASE3B_DESIGN.split())
    assert "has been validated against ArduCopter SITL" in normalized
    assert "SITL execution conformance is pending" not in normalized
    assert "no real-aircraft validation/support claim" in normalized
    assert "4c98c9221a" in normalized
    assert "AP_FWVersion.h" in normalized
    assert "no mission/GCS runtime source modification" in normalized
    # remaining Phase 3B capabilities stay honestly unimplemented
    assert (
        "pause/resume/stop/set-current, operation cancel, progress UX"
        in normalized
    )
    assert "is still **unimplemented**" in normalized


def test_agents_md_records_merge_commit_policy() -> None:
    """AGENTS.md must pin the repository merge policy: merge commits by
    default with the branch's Conventional Commits preserved, squash merge
    only on explicit owner request, and no automatic feature-branch
    deletion after merge."""
    normalized = " ".join(_AGENTS_MD.split())
    assert 'use GitHub "Create a merge commit"' in normalized
    assert "Preserve the feature branch's meaningful Conventional Commits" in normalized
    assert "Do not squash merge by default" in normalized
    assert (
        "Squash merge is allowed only when the repository owner explicitly "
        "requests it" in normalized
    )
    assert (
        "Do not automatically delete feature branches after merge unless "
        "explicitly requested" in normalized
    )


def test_readme_mission_section_is_ardupilot_first_and_sitl_only() -> None:
    # Phase 3A wording invariants: ArduPilot-first JSON, SITL-validated only,
    # no real-aircraft claim, locally modified ArduPilot provenance.
    for readme, first_marker, sitl_marker, craft_marker in (
        (_README, "ArduPilot-first", "SITL validated only", "real-aircraft"),
        (_ZH_README, "ArduPilot 优先", "SITL 验证", "真实飞机"),
    ):
        assert first_marker in readme
        assert sitl_marker in readme
        assert craft_marker in readme
        assert "4c98c9221a" in readme
        assert "AP_FWVersion.h" in readme


def test_publishing_doc_records_022_dev_version_history() -> None:
    # 0.2.2.dev0 was the Phase 3A development version and was never
    # released — the section must stay a historical note, and no 0.2.2
    # production release record may exist.
    assert "## Version history note: 0.2.2.dev0 (never released)" in _PUBLISHING
    normalized = " ".join(_PUBLISHING.split())
    assert "0.2.2.dev0" in normalized
    assert "**0.2.2 was never released**" in normalized
    assert "shipped as part of 0.3.0" in normalized
    assert "## Production release record: 0.2.2" not in _PUBLISHING


def test_readmes_highlight_the_040_notable_changes() -> None:
    # The 0.4.0 highlight section: the operation foundation, mission start
    # (SITL-validated), and the home-slot guard fix — present in both
    # READMEs, newest release first; the 0.3.0 section stays as history.
    en_facts = ("Notable in 0.4.0:", "Operation foundation (Phase 3B-0)",
                "Mission start (Phase 3B-1)", "operation_superseded",
                "operation_wait_timeout", "mavctl operation get <id>",
                "requires wire count >= 2")
    zh_facts = ("0.4.0 主要变化", "Operation foundation",
                "Mission start", "operation_superseded",
                "operation_wait_timeout", "mavctl operation get <id>",
                "wire count >= 2")
    for readme, facts in ((_README, en_facts), (_ZH_README, zh_facts)):
        for fact in facts:
            assert fact in readme, fact
        # newest release first: the 0.4.0 section precedes the 0.3.0 one
        assert readme.index(facts[0]) < readme.index(
            "Notable in 0.3.0:" if readme is _README else "0.3.0 主要变化")


def test_readmes_highlight_the_021_notable_changes() -> None:
    # The release-prep READMEs must surface the 0.2.1 notable changes:
    # the version flag, the freshness fields, and the stale-ground guard.
    for readme in (_README, _ZH_README):
        assert "mavctl --version" in readme
        assert "telemetry_age_s" in readme
        assert "ground_state_stale" in readme


# -- publishing docs ---------------------------------------------------------


def test_publishing_doc_records_the_021_production_release() -> None:
    assert "## Production release record: 0.2.1" in _PUBLISHING
    assert "mavctl 0.2.1 is published on production PyPI" in _PUBLISHING
    assert "Released: 2026-08-31" in _PUBLISHING
    assert "Version: `0.2.1`" in _PUBLISHING
    assert "GitHub Release: `v0.2.1`" in _PUBLISHING
    # Recorded verification facts.
    normalized = " ".join(_PUBLISHING.split())
    assert "production PyPI JSON metadata" in normalized
    assert "clean virtual-environment installation" in normalized
    assert "mavctl --version" in normalized
    assert "mavctl --help" in normalized
    assert "mavctl daemon --help" in normalized
    # Scope note: safety and observability patch release.
    assert (
        "0.2.1 is a safety and observability patch release"
        in " ".join(_PUBLISHING.split())
    )


def test_testpypi_record_keeps_denying_production_equivalence() -> None:
    # Historical rehearsal record: it was a rehearsal, never a production release.
    assert "has been released on production PyPI" in _PUBLISHING


def test_publishing_doc_records_production_release() -> None:
    assert "mavctl 0.2.0 is published on production PyPI" in _PUBLISHING
    assert "Released: 2026-08-26" in _PUBLISHING
    assert "GitHub Actions OIDC Trusted Publishing" in _PUBLISHING
    assert "wheel and sdist" in _PUBLISHING
    assert "clean-venv install" in _PUBLISHING
    # Historical mention: 0.2.2.dev0 appears only in the never-released
    # version-history note; the current guidance names 0.3.1.dev0/0.4.0.dev0.
    assert "0.2.2.dev0" in _PUBLISHING
    assert "0.3.1.dev0" in _PUBLISHING
    assert "0.4.0.dev0" in _PUBLISHING
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
