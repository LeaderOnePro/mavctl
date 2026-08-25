"""Documentation-consistency tests for the bundled agent skill.

The skill under ``skills/mavctl-flight/`` must only document the CLI surface
that actually exists: every ``mavctl <cmd>`` appearing as runnable text
(fenced blocks or inline code) must be a real top-level command, and
dangerous unimplemented capabilities (missions, geofences, parameter
editing) may be *named* as absent in plain prose but must never show up as
runnable text. The docs must also stay structurally valid Markdown (balanced
fences, headings-first) and keep their safety semantics explicit.
"""

from __future__ import annotations

import re
from pathlib import Path

_SKILL_DIR = Path(__file__).resolve().parent.parent / "skills" / "mavctl-flight"
_DOC_FILES = (
    _SKILL_DIR / "SKILL.md",
    _SKILL_DIR / "references" / "workflows.md",
    _SKILL_DIR / "references" / "safety.md",
    _SKILL_DIR / "references" / "troubleshooting.md",
)

# Top-level commands the current CLI actually implements (mavctl --help).
_SUPPORTED_COMMANDS = frozenset(
    {"status", "telemetry", "arm", "disarm", "mode", "takeoff", "land", "rtl", "daemon"}
)

# Unimplemented dangerous capabilities: free to *name* in plain prose ("not
# implemented"); forbidden as runnable text inside code segments.
_UNSUPPORTED_DANGEROUS = re.compile(
    r"mavctl\s+(mission|geofence|fence|rally|params?)\b", re.IGNORECASE
)
_MAVCTL_INVOCATION = re.compile(r"mavctl\s+([A-Za-z][A-Za-z0-9_-]*)")

# arm has no force path anywhere; a runnable "mavctl arm ... --force" in a code
# segment must never appear (a plain-prose warning about it stays allowed).
_FORCE_ARM_INVOCATION = re.compile(r"mavctl\s+arm\b[^\n]*--force", re.IGNORECASE)

# A forced disarm must be framed as an emergency motor stop / crash risk.
_FORCED_DISARM = re.compile(r"(mavctl\s+)?disarm[^\n`]*--force")
_EMERGENCY_FRAMING = re.compile(r"emergency|motor stop|crash", re.IGNORECASE)

# The ACK-beats-heartbeat caveat: "ack" and "armed" mentioned near each other.
_ACK_BEFORE_ARMED = re.compile(
    r"ack[\s\S]{0,160}\barmed\b|\barmed\b[\s\S]{0,160}\back\b", re.IGNORECASE
)

_FENCE_LINE = re.compile(r"^\s*(`{3,})(.*)$")


def _combined_text() -> str:
    return "\n".join(path.read_text(encoding="utf-8") for path in _DOC_FILES)


def _code_segments(text: str) -> list[str]:
    """Return fenced code-block lines plus inline code spans of a document."""
    segments: list[str] = []
    in_fence = False
    for line in text.splitlines():
        if line.lstrip().startswith("```"):
            in_fence = not in_fence
            continue
        if in_fence:
            segments.append(line)
        else:
            segments.extend(re.findall(r"`([^`\n]+)`", line))
    return segments


def _find_in_code(pattern: re.Pattern[str], text: str) -> list[str]:
    """Matches of ``pattern`` inside code segments only — plain prose is ignored."""

    return [
        match.group(0)
        for segment in _code_segments(text)
        for match in pattern.finditer(segment)
    ]


def _fences_balanced(text: str) -> bool:
    """True when every fenced block is closed by a matching fence.

    A line with 3+ backticks opens a block when none is open; while a block
    is open only a bare fence of at least the opener's length closes it.
    """

    open_len: int | None = None
    for line in text.splitlines():
        match = _FENCE_LINE.match(line)
        if match is None:
            continue
        ticks, info = len(match.group(1)), match.group(2).strip()
        if open_len is None:
            open_len = ticks
        elif not info and ticks >= open_len:
            open_len = None
    return open_len is None


def _body_after_frontmatter(text: str) -> str:
    """Strip a leading YAML frontmatter block, if present."""

    if not text.startswith("---\n"):
        return text
    end = text.find("\n---\n", 3)
    return text[end + len("\n---\n"):] if end != -1 else text


def _first_nonblank_line(text: str) -> str:
    for line in text.splitlines():
        if line.strip():
            return line
    return ""


# -- structure --------------------------------------------------------------


def test_skill_files_exist() -> None:
    for path in _DOC_FILES:
        assert path.is_file(), path.relative_to(_SKILL_DIR)


def test_fenced_code_blocks_are_balanced() -> None:
    for path in _DOC_FILES:
        text = path.read_text(encoding="utf-8")
        assert _fences_balanced(text), f"{path.name}: unbalanced code fence"


def test_documents_open_with_headings_not_fences() -> None:
    for path in _DOC_FILES[1:]:
        first = _first_nonblank_line(path.read_text(encoding="utf-8"))
        assert first.startswith("# "), f"{path.name} must start with a heading"
    skill_body = _body_after_frontmatter(_DOC_FILES[0].read_text(encoding="utf-8"))
    first = _first_nonblank_line(skill_body)
    assert first.startswith("# "), "SKILL.md body must start with a heading"


def test_skill_frontmatter_names_the_directory() -> None:
    head = _DOC_FILES[0].read_text(encoding="utf-8")
    assert head.startswith("---\n"), "SKILL.md must open with YAML frontmatter"
    assert "\nname: mavctl-flight\n" in head
    assert "\ndescription: " in head


# -- command surface --------------------------------------------------------


def test_no_unsupported_dangerous_command_is_invoked() -> None:
    for path in _DOC_FILES:
        hits = _find_in_code(_UNSUPPORTED_DANGEROUS, path.read_text(encoding="utf-8"))
        assert not hits, f"{path.name}: runnable invocation(s) {hits}"


def test_every_documented_mavctl_invocation_exists() -> None:
    for path in _DOC_FILES:
        for segment in _code_segments(path.read_text(encoding="utf-8")):
            for command in _MAVCTL_INVOCATION.findall(segment):
                assert command in _SUPPORTED_COMMANDS, f"{path.name}: mavctl {command}"


def test_force_arm_is_never_an_invocation() -> None:
    for path in _DOC_FILES:
        hits = _find_in_code(_FORCE_ARM_INVOCATION, path.read_text(encoding="utf-8"))
        assert not hits, f"{path.name}: forbidden runnable form {hits}"


def test_plain_prose_may_name_unimplemented_capabilities() -> None:
    prose = (
        "Mission upload is future work: the mavctl mission, mavctl fence and "
        "mavctl param commands are not implemented yet."
    )
    assert _find_in_code(_UNSUPPORTED_DANGEROUS, prose) == []


def test_plain_prose_may_warn_against_force_arm() -> None:
    prose = "Warning: do not try mavctl arm --force; force arming is unsupported."
    assert _find_in_code(_FORCE_ARM_INVOCATION, prose) == []


def test_runnable_forms_are_rejected_in_code_segments() -> None:
    fenced = "```bash\nmavctl mission upload x\nmavctl arm --confirm --force\n```\n"
    assert _find_in_code(_UNSUPPORTED_DANGEROUS, fenced) == ["mavctl mission"]
    assert _find_in_code(_FORCE_ARM_INVOCATION, fenced) == [
        "mavctl arm --confirm --force"
    ]
    inline = "run `mavctl params set ARMING_CHECK 0`\n"
    assert _find_in_code(_UNSUPPORTED_DANGEROUS, inline) == ["mavctl params"]


# -- safety semantics -------------------------------------------------------


def test_confirm_and_dry_run_are_documented() -> None:
    combined = _combined_text()
    assert "--confirm" in combined
    assert "--dry-run" in combined


def test_structured_rejection_reasons_are_named() -> None:
    combined = _combined_text()
    assert "ground_state_unknown" in combined
    assert "link_lost_during_wait" in combined


def test_ack_beats_heartbeat_caveat_is_stated() -> None:
    assert _ACK_BEFORE_ARMED.search(_combined_text()), (
        "docs must state that the arm ACK can precede armed=true"
    )


def test_forced_disarm_is_framed_as_emergency_stop() -> None:
    combined = _combined_text()
    occurrences = list(_FORCED_DISARM.finditer(combined))
    assert occurrences, "docs must mention disarm --force and its meaning"
    for match in occurrences:
        window = combined[max(0, match.start() - 400) : match.end() + 400]
        assert _EMERGENCY_FRAMING.search(window), (
            f"{match.group(0)!r} must be framed as emergency motor stop / crash risk"
        )
