"""Health observation tests (Issue #4, observe-only v1).

Model-level semantics (this module) and adapter-level observation tests:
raw vehicle-reported health data, never a verdict. All clock-dependent
behaviour is driven with fake monotonic clocks — no sleeps, no wall clock.

Honesty invariants pinned here:

- ``unknown`` is a distinct state and never means healthy;
- v1 emits no ``healthy`` / ``safe`` / ``ready`` / ``prearm_passed``
  verdict anywhere;
- ``last_status`` / cached raw fields are historical observations and must
  never become guard allow evidence (guards never read health — see the
  guard regression tests);
- health freshness is independent of telemetry freshness.
"""

from __future__ import annotations

import json
from typing import Any

import pytest
from pymavlink import mavutil

from mavctl.adapter.pymavlink_adapter import PymavlinkAdapter
from mavctl.daemon.guards import GuardConfig
from mavctl.models import (
    ESTIMATOR_STALE_AFTER_S,
    EstimatorObservation,
    HealthObservation,
    PrearmObservation,
    SensorObservation,
    VehicleState,
)
from tests.fakes import FakeMaster, FakeMsg

# -- models: never observed ----------------------------------------------------


def test_never_observed_health_is_all_unknown() -> None:
    health = HealthObservation()
    assert health.estimator.status == "unknown"
    assert health.estimator.last_status is None
    assert health.estimator.flags == []
    assert health.estimator.flags_mask is None
    assert health.estimator.age_s is None
    for variance in (
        "velocity_variance",
        "pos_horiz_variance",
        "pos_vert_variance",
        "compass_variance",
        "terrain_alt_variance",
        "airspeed_variance",
    ):
        assert getattr(health.estimator, variance) is None
    assert health.sensors.present is None
    assert health.sensors.enabled is None
    assert health.sensors.health is None
    assert health.sensors.age_s is None
    assert health.prearm.last_message is None
    assert health.prearm.severity is None
    assert health.prearm.age_s is None


def test_vehicle_state_defaults_to_never_observed_health() -> None:
    state = VehicleState()
    assert state.health == HealthObservation()
    assert state.health.estimator.status == "unknown"


def test_health_json_shape_matches_contract() -> None:
    health = HealthObservation()
    dumped = json.loads(health.model_dump_json())
    assert set(dumped) == {"estimator", "sensors", "prearm"}
    assert set(dumped["estimator"]) == {
        "status",
        "last_status",
        "flags",
        "flags_mask",
        "velocity_variance",
        "pos_horiz_variance",
        "pos_vert_variance",
        "compass_variance",
        "terrain_alt_variance",
        "airspeed_variance",
        "age_s",
    }
    assert set(dumped["sensors"]) == {"present", "enabled", "health", "age_s"}
    assert set(dumped["prearm"]) == {"last_message", "severity", "age_s"}
    # the full state snapshot carries the same health block
    state_dump = json.loads(VehicleState().model_dump_json())
    assert state_dump["health"] == dumped


# -- models: no health verdicts in v1 -----------------------------------------


def test_estimator_status_never_carries_health_verdicts() -> None:
    # "observed" only ever means "a report arrived" — the literal set is
    # restricted at the type level; healthy/safe/ready cannot be expressed.
    fresh = EstimatorObservation(status="observed", last_status="observed")
    assert fresh.status == "observed"
    assert "healthy" not in fresh.model_dump_json()
    assert "ready" not in fresh.model_dump_json()
    assert "safe" not in fresh.model_dump_json()
    assert "prearm_passed" not in fresh.model_dump_json()


def test_stale_after_constant_is_documented_policy() -> None:
    # The EKF stream is requested at 2 Hz; the policy threshold must be
    # comfortably above a couple of missed periods.
    assert 1.0 <= ESTIMATOR_STALE_AFTER_S <= 30.0


def test_fresh_observed_keeps_raw_fields() -> None:
    health = HealthObservation(
        estimator=EstimatorObservation(
            status="observed",
            last_status="observed",
            flags=["EKF_ATTITUDE", "EKF_POS_HORIZ_ABS"],
            flags_mask=0b01010011,
            velocity_variance=0.01,
            pos_horiz_variance=0.02,
            pos_vert_variance=0.03,
            compass_variance=0.04,
            terrain_alt_variance=0.05,
            airspeed_variance=None,
            age_s=0.2,
        ),
        sensors=SensorObservation(present=0x7F, enabled=0x3F, health=0x3F, age_s=0.1),
        prearm=PrearmObservation(
            last_message="PreArm: Barometer not healthy", severity=4, age_s=1.5
        ),
    )
    assert health.estimator.flags_mask == 0b01010011
    assert health.sensors.present == 0x7F  # raw bitmask, 0 is valid too
    assert health.prearm.severity == 4


# -- adapter: EKF_STATUS_REPORT observation ------------------------------------

_ARMED_FLAG = 0b10000000


class _FakeMonotonicClock:
    def __init__(self, start: float = 1000.0) -> None:
        self.now = start

    def advance(self, seconds: float) -> None:
        self.now += seconds

    def __call__(self) -> float:
        return self.now


def _health_adapter() -> tuple[PymavlinkAdapter, FakeMaster]:
    adapter = PymavlinkAdapter("udp:127.0.0.1:14550")
    master = FakeMaster(flightmode="GUIDED")
    adapter._master = master
    return adapter, master


def _lock(adapter: PymavlinkAdapter) -> None:
    adapter._on_heartbeat(
        FakeMsg("HEARTBEAT", base_mode=0, custom_mode=4, system_status=3)
    )


def _ekf(system: int = 1, component: int = 1, **overrides: Any) -> FakeMsg:
    fields: dict[str, Any] = {
        "flags": 0b01010011,
        "velocity_variance": 0.01,
        "pos_horiz_variance": 0.02,
        "pos_vert_variance": 0.03,
        "compass_variance": 0.04,
        "terrain_alt_variance": 0.05,
        "airspeed_variance": 0.06,
    }
    fields.update(overrides)
    return FakeMsg(
        "EKF_STATUS_REPORT",
        src_system=fields.pop("src_system", system),
        src_component=fields.pop("src_component", component),
        **fields,
    )


def test_locked_ekf_updates_all_raw_fields() -> None:
    adapter, _master = _health_adapter()
    _lock(adapter)
    adapter._on_ekf_status_report(_ekf())

    health = adapter.get_state().health
    estimator = health.estimator
    assert estimator.status == "observed"
    assert estimator.last_status == "observed"
    assert estimator.flags_mask == 0b01010011
    assert estimator.flags == [
        "EKF_ATTITUDE",
        "EKF_VELOCITY_HORIZ",
        "EKF_POS_HORIZ_ABS",
        "EKF_POS_VERT_AGL",
    ]
    assert estimator.velocity_variance == 0.01
    assert estimator.pos_horiz_variance == 0.02
    assert estimator.pos_vert_variance == 0.03
    assert estimator.compass_variance == 0.04
    assert estimator.terrain_alt_variance == 0.05
    assert estimator.airspeed_variance == 0.06
    assert estimator.age_s is not None and estimator.age_s < 1.0


def test_foreign_ekf_ignored() -> None:
    adapter, _master = _health_adapter()
    _lock(adapter)
    adapter._on_ekf_status_report(_ekf(src_system=2))
    adapter._on_ekf_status_report(_ekf(src_component=154))

    health = adapter.get_state().health
    assert health.estimator.status == "unknown"
    assert health.estimator.last_status is None
    assert health.estimator.flags_mask is None
    assert health.estimator.age_s is None


def test_unknown_ekf_bits_survive_in_raw_mask() -> None:
    adapter, _master = _health_adapter()
    _lock(adapter)
    adapter._on_ekf_status_report(_ekf(flags=0xFFFF))

    estimator = adapter.get_state().health.estimator
    # Every dialect-known single-bit flag is named; the four bits the
    # dialect does not define (0xF000) survive only in the raw mask —
    # never dropped, never guessed into names.
    assert estimator.flags_mask == 0xFFFF
    assert len(estimator.flags) == 12
    assert "EKF_ATTITUDE" in estimator.flags
    assert not any("ENUM_END" in name for name in estimator.flags)


def test_ekf_age_grows_and_degrades_to_unknown_with_fake_clock(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    clock = _FakeMonotonicClock()
    monkeypatch.setattr("mavctl.adapter.pymavlink_adapter.time.monotonic", clock)
    adapter, _master = _health_adapter()
    _lock(adapter)
    adapter._on_ekf_status_report(_ekf())

    clock.advance(1.0)
    estimator = adapter.get_state().health.estimator
    assert estimator.status == "observed"
    assert estimator.age_s == pytest.approx(1.0)

    # Past the staleness threshold: the current verdict is unknown while
    # the cached observation stays visible for explainability.
    clock.advance(ESTIMATOR_STALE_AFTER_S)
    estimator = adapter.get_state().health.estimator
    assert estimator.status == "unknown"
    assert estimator.last_status == "observed"
    assert estimator.flags_mask == 0b01010011
    assert estimator.age_s == pytest.approx(1.0 + ESTIMATOR_STALE_AFTER_S)


# -- adapter: SYS_STATUS sensor bitmasks ----------------------------------------


def _sys_status(system: int = 1, component: int = 1, **overrides: Any) -> FakeMsg:
    fields: dict[str, Any] = {
        "voltage_battery": 12600,
        "current_battery": 100,
        "battery_remaining": 80,
        "onboard_control_sensors_present": 0x7F,
        "onboard_control_sensors_enabled": 0x3F,
        "onboard_control_sensors_health": 0x3F,
    }
    fields.update(overrides)
    return FakeMsg(
        "SYS_STATUS",
        src_system=fields.pop("src_system", system),
        src_component=fields.pop("src_component", component),
        **fields,
    )


def test_locked_sys_status_updates_battery_and_sensors() -> None:
    adapter, _master = _health_adapter()
    _lock(adapter)
    adapter._on_sys_status(_sys_status())

    state = adapter.get_state()
    assert state.battery.voltage_v == 12.6
    sensors = state.health.sensors
    assert sensors.present == 0x7F
    assert sensors.enabled == 0x3F
    assert sensors.health == 0x3F
    assert sensors.age_s is not None and sensors.age_s < 1.0


def test_foreign_sys_status_ignored_for_battery_and_sensors() -> None:
    adapter, _master = _health_adapter()
    _lock(adapter)
    adapter._on_sys_status(_sys_status())
    adapter._on_sys_status(_sys_status(src_system=2, voltage_battery=5000))
    adapter._on_sys_status(_sys_status(src_component=154, onboard_control_sensors_present=0x1FF))

    state = adapter.get_state()
    assert state.battery.voltage_v == 12.6
    assert state.health.sensors.present == 0x7F
    assert state.health.sensors.enabled == 0x3F


# -- adapter: STATUSTEXT prearm diagnostics --------------------------------------


def _statustext(
    text: str,
    severity: int = 4,
    system: int = 1,
    component: int = 1,
    **overrides: Any,
) -> FakeMsg:
    fields: dict[str, Any] = {"severity": severity, "text": text}
    fields.update(overrides)
    return FakeMsg(
        "STATUSTEXT",
        src_system=fields.pop("src_system", system),
        src_component=fields.pop("src_component", component),
        **fields,
    )


def test_matching_statustext_saved() -> None:
    adapter, _master = _health_adapter()
    _lock(adapter)
    adapter._on_statustext(_statustext("PreArm: Barometer not healthy", severity=4))

    prearm = adapter.get_state().health.prearm
    assert prearm.last_message == "PreArm: Barometer not healthy"
    assert prearm.severity == 4
    assert prearm.age_s is not None and prearm.age_s < 1.0

    adapter._on_statustext(_statustext("Arm: Gyros inconsistent", severity=4))
    assert adapter.get_state().health.prearm.last_message == "Arm: Gyros inconsistent"


def test_foreign_and_nonmatching_statustext_ignored() -> None:
    adapter, _master = _health_adapter()
    _lock(adapter)
    adapter._on_statustext(_statustext("PreArm: from gimbal", src_component=154))
    adapter._on_statustext(_statustext("PreArm: from another system", src_system=2))
    adapter._on_statustext(_statustext("Hit: something unrelated"))
    adapter._on_statustext(_statustext("CriticalSection: also unrelated"))

    assert adapter.get_state().health.prearm.last_message is None


def test_chunked_statustext_conservatively_ignored() -> None:
    adapter, _master = _health_adapter()
    _lock(adapter)
    # Continuation chunk and a chunked family (nonzero id): v1 never
    # assembles partial text.
    adapter._on_statustext(_statustext("continuation", chunk_seq=1))
    adapter._on_statustext(_statustext("first of many", id=7, chunk_seq=0))

    assert adapter.get_state().health.prearm.last_message is None

    # A complete single-chunk message still lands.
    adapter._on_statustext(_statustext("PreArm: complete message"))
    assert adapter.get_state().health.prearm.last_message == "PreArm: complete message"


def test_no_statustext_does_not_imply_ready() -> None:
    adapter, _master = _health_adapter()
    _lock(adapter)
    health = adapter.get_state().health
    # No message at all: prearm stays null — never "ready"/"passed".
    assert health.prearm.last_message is None
    assert health.prearm.severity is None
    assert "ready" not in health.model_dump_json()


# -- adapter: handler isolation ----------------------------------------------------


def test_health_handlers_send_nothing_and_leave_machinery_alone() -> None:
    adapter, master = _health_adapter()
    _lock(adapter)
    sent_before = list(master.sent)
    setup_before = list(master.link_setup)

    adapter._on_ekf_status_report(_ekf())
    adapter._on_statustext(_statustext("PreArm: diagnostic"))
    adapter._on_sys_status(_sys_status())

    # Observation handlers are passive: no command, no setup churn.
    assert master.sent == sent_before
    assert master.link_setup == setup_before
    # COMMAND_ACK map and mission transaction inbox are untouched.
    assert adapter._acks == {}
    assert adapter._mission_inbox == []
    # MISSION_CURRENT still routes to its own observation.
    adapter._on_mission_current(
        FakeMsg("MISSION_CURRENT", seq=1, total=4, mission_state=3, mission_mode=1)
    )
    mission = adapter.get_state().mission
    assert mission is not None and mission.state == "active"


# -- adapter: EKF stream request ---------------------------------------------------


def test_ekf_stream_request_sent_once_with_correct_interval() -> None:
    adapter, master = _health_adapter()
    _lock(adapter)  # first heartbeat triggers _request_streams_once

    ekf_requests = [
        entry
        for entry in master.link_setup
        if entry[0] == mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL
        and entry[2][0] == float(mavutil.mavlink.MAVLINK_MSG_ID_EKF_STATUS_REPORT)
    ]
    assert len(ekf_requests) == 1
    _command, _confirmation, params = ekf_requests[0]
    assert params[1] == 500000.0  # 2 Hz in microseconds

    _lock(adapter)  # a later heartbeat must not re-request
    ekf_requests = [
        entry
        for entry in master.link_setup
        if entry[0] == mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL
        and entry[2][0] == float(mavutil.mavlink.MAVLINK_MSG_ID_EKF_STATUS_REPORT)
    ]
    assert len(ekf_requests) == 1


def test_ekf_stream_request_failure_does_not_break_lock() -> None:
    adapter, master = _health_adapter()

    def _explode(*_a: Any, **_k: Any) -> None:
        raise OSError("link hiccup")

    master.mav.command_long_send = _explode
    _lock(adapter)  # must not raise
    assert adapter.get_state().connected is True
    assert adapter.get_state().health.estimator.status == "unknown"


# -- CLI: status rendering -----------------------------------------------------------


def test_status_human_render_never_observed() -> None:
    from mavctl.cli.app import _format_state

    state = {"connected": True, "health": HealthObservation().model_dump()}
    lines = _format_state(state).splitlines()
    start = next(i for i, line in enumerate(lines) if line.startswith("health"))
    health_lines = lines[start : start + 3]
    assert health_lines == [
        "health     : estimator=unknown age=n/a",
        "             sensors=n/a/n/a/n/a  age=n/a",
        "             prearm=n/a  age=n/a",
    ]


def test_status_human_render_fresh_and_stale() -> None:
    from mavctl.cli.app import _format_state

    fresh: dict[str, Any] = {
        "connected": True,
        "health": {
            "estimator": {
                "status": "observed", "last_status": "observed",
                "flags": ["EKF_ATTITUDE"], "flags_mask": 0x53, "age_s": 0.2,
            },
            "sensors": {"present": 0x7F, "enabled": 0x3F, "health": 0x3F, "age_s": 0.1},
            "prearm": {
                "last_message": "PreArm: Barometer not healthy", "severity": 4, "age_s": 1.5,
            },
        },
    }
    lines = _format_state(fresh).splitlines()
    assert "health     : estimator=observed age=0.2s flags=0x53" in lines
    assert "             sensors=0x7f/0x3f/0x3f  age=0.1s" in lines
    assert "             prearm=PreArm: Barometer not healthy (sev=4)  age=1.5s" in lines

    stale: dict[str, Any] = {
        "connected": True,
        "health": {
            "estimator": {
                "status": "unknown", "last_status": "observed",
                "flags": ["EKF_ATTITUDE"], "flags_mask": 0x53, "age_s": 30.0,
            },
            "sensors": fresh["health"]["sensors"],
            "prearm": fresh["health"]["prearm"],
        },
    }
    assert "health     : estimator=unknown cached_age=30.0s flags=0x53" in _format_state(stale)


def test_status_json_carries_health_block() -> None:
    adapter, _master = _health_adapter()
    _lock(adapter)
    adapter._on_ekf_status_report(_ekf())
    dumped = json.loads(adapter.get_state().model_dump_json())
    assert dumped["health"]["estimator"]["status"] == "observed"
    assert dumped["health"]["estimator"]["flags_mask"] == 0b01010011


# -- guard regression: health data cannot gate anything -----------------------------


def _guard_state(**overrides: Any) -> VehicleState:
    from mavctl.models import GpsInfo

    fields: dict[str, Any] = {
        "connected": True,
        "heartbeat_age_s": 0.2,
        "flight_mode": "GUIDED",
        "armed": True,
        "gps": GpsInfo(fix_type=6, fix_label="rtk_fixed"),
    }
    fields.update(overrides)
    return VehicleState(**fields)


def test_health_data_cannot_change_guard_decisions() -> None:
    from mavctl.daemon import guards

    alarming = HealthObservation(
        estimator=EstimatorObservation(
            status="observed",
            last_status="observed",
            flags=["EKF_GPS_GLITCHING", "EKF_UNINITIALIZED"],
            flags_mask=0xFFFF,
        ),
        sensors=SensorObservation(present=0, enabled=0, health=0, age_s=99.0),
        prearm=PrearmObservation(
            last_message="PreArm: everything is wrong", severity=1, age_s=0.1
        ),
    )
    for state in (_guard_state(), _guard_state(armed=False)):
        stripped = state.model_copy(update={"health": HealthObservation()})
        alarming_state = state.model_copy(update={"health": alarming})

        # Identical inputs (health is not one) must give identical decisions.
        assert guards.check_arm(
            alarming_state, confirm=True, config=GuardConfig()
        ) == guards.check_arm(stripped, confirm=True, config=GuardConfig())
        assert guards.check_mission_start(
            alarming_state, mission_count=4, confirm=True, config=GuardConfig()
        ) == guards.check_mission_start(
            stripped, mission_count=4, confirm=True, config=GuardConfig()
        )
        # And with a disarmed vehicle the mission-start guard still refuses
        # for its own existing reason, never because of health.
        decision = guards.check_mission_start(
            _guard_state(armed=False, health=alarming),
            mission_count=4, confirm=True, config=GuardConfig(),
        )
        assert decision.reason == "mission_requires_armed"
