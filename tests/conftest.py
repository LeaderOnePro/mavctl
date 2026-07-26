"""Shared pytest fixtures.

Every test runs against an isolated ``MAVCTL_HOME`` so no test touches the
developer's real ``~/.mavctl`` state.
"""

from __future__ import annotations

from collections.abc import Iterator
from pathlib import Path

import pytest


@pytest.fixture(autouse=True)
def isolated_home(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> Iterator[Path]:
    home = tmp_path / "mavctl"
    monkeypatch.setenv("MAVCTL_HOME", str(home))
    yield home
