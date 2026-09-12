"""Tests for the daemon process entrypoint (``python -m mavctl.daemon``).

B2: the ``--heartbeat-timeout`` value is the single source of truth for both
the adapter's ``connected`` computation and the guards' heartbeat-freshness
gate; the entrypoint must validate it and wire it to both.
"""

from __future__ import annotations

import pytest

from mavctl.daemon.__main__ import _parse_args, main
from mavctl.daemon.guards import GuardConfig


@pytest.mark.parametrize("bad", ["0", "-1", "nan", "inf"])
def test_parse_args_rejects_non_positive_or_nonfinite_heartbeat_timeout(bad: str) -> None:
    with pytest.raises(SystemExit) as excinfo:
        _parse_args(["--connect", "udp:x", "--heartbeat-timeout", bad])
    assert excinfo.value.code == 2  # argparse usage error


def test_parse_args_accepts_positive_timeout() -> None:
    args = _parse_args(["--connect", "udp:x", "--heartbeat-timeout", "5"])
    assert args.heartbeat_timeout == 5.0
    assert args.connect == "udp:x"


def test_main_wires_one_heartbeat_timeout_to_adapter_and_guards(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    captured: dict[str, object] = {}

    def fake_create_adapter(connection_string: str, heartbeat_timeout_s: float) -> object:
        captured["adapter_timeout"] = heartbeat_timeout_s
        return object()

    def fake_server_cls(
        adapter: object, connection_string: str, guard_config: GuardConfig | None = None
    ) -> object:
        captured["guard_config"] = guard_config

        class FakeServer:
            def serve(self) -> object:
                async def run() -> None:
                    return None

                return run()

        return FakeServer()

    monkeypatch.setattr("mavctl.daemon.__main__.create_adapter", fake_create_adapter)
    monkeypatch.setattr("mavctl.daemon.__main__.DaemonServer", fake_server_cls)
    monkeypatch.setattr("mavctl.daemon.process.write_pid", lambda pid: None)
    monkeypatch.setattr("mavctl.daemon.process.clear_pid", lambda: None)

    exit_code = main(["--connect", "udp:x", "--heartbeat-timeout", "5"])

    assert exit_code == 0
    assert captured["adapter_timeout"] == 5.0
    guard_config = captured["guard_config"]
    assert isinstance(guard_config, GuardConfig)
    # Same value, both consumers: no drifting defaults possible.
    assert guard_config.max_heartbeat_age_s == 5.0
