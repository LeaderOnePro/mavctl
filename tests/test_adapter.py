"""Unit tests for the pymavlink adapter using mocked MAVLink messages."""

from __future__ import annotations

import threading
import time

import pytest
from pymavlink import mavutil

from mavctl.adapter.pymavlink_adapter import PymavlinkAdapter
from mavctl.models import CommandOutcome
from tests.fakes import FakeMaster, FakeMsg

# MAV_MODE_FLAG_SAFETY_ARMED bit.
_ARMED_FLAG = 0b10000000


def _heartbeat(armed: bool = False, custom_mode: int = 0, system_status: int = 3) -> FakeMsg:
    return FakeMsg(
        "HEARTBEAT",
        base_mode=_ARMED_FLAG if armed else 0,
        custom_mode=custom_mode,
        system_status=system_status,
    )


def test_heartbeat_marks_connected_and_arming() -> None:
    adapter = PymavlinkAdapter("udp:127.0.0.1:14550")
    adapter._master = FakeMaster(flightmode="GUIDED")

    adapter._on_heartbeat(_heartbeat(armed=True))
    state = adapter.get_state()

    assert state.connected is True
    assert state.armed is True
    assert state.flight_mode == "GUIDED"
    assert state.system_id == 1
    assert state.heartbeat_age_s is not None and state.heartbeat_age_s < 1.0


def test_heartbeat_timeout_reports_disconnected() -> None:
    adapter = PymavlinkAdapter("udp:127.0.0.1:14550", heartbeat_timeout_s=3.0)
    adapter._master = FakeMaster()
    adapter._on_heartbeat(_heartbeat())

    # Backdate the last heartbeat beyond the timeout window.
    adapter._last_hb_monotonic = time.monotonic() - 5.0
    state = adapter.get_state()

    assert state.connected is False
    # When disconnected, volatile fields are suppressed.
    assert state.flight_mode is None
    assert state.armed is None
    assert state.heartbeat_age_s is not None and state.heartbeat_age_s >= 5.0


def _lock_autopilot(adapter: PymavlinkAdapter) -> None:
    """Lock target onto default sys=1/comp=1 autopilot (FakeMsg defaults)."""

    adapter._master = adapter._master or FakeMaster(flightmode="GUIDED")
    adapter._on_heartbeat(_heartbeat())


def test_sys_status_populates_battery() -> None:
    adapter = PymavlinkAdapter("udp:127.0.0.1:14550")
    _lock_autopilot(adapter)
    adapter._on_sys_status(
        FakeMsg("SYS_STATUS", voltage_battery=12600, current_battery=1550, battery_remaining=87)
    )
    battery = adapter.get_state().battery

    assert battery.voltage_v == pytest.approx(12.6)
    assert battery.current_a == pytest.approx(15.5)
    assert battery.remaining_pct == 87


def test_sys_status_handles_unknown_sentinels() -> None:
    adapter = PymavlinkAdapter("udp:127.0.0.1:14550")
    _lock_autopilot(adapter)
    adapter._on_sys_status(
        FakeMsg("SYS_STATUS", voltage_battery=65535, current_battery=-1, battery_remaining=-1)
    )
    battery = adapter.get_state().battery

    assert battery.voltage_v is None
    assert battery.current_a is None
    assert battery.remaining_pct is None


def test_global_position_populates_position_and_velocity() -> None:
    adapter = PymavlinkAdapter("udp:127.0.0.1:14550")
    _lock_autopilot(adapter)
    adapter._on_global_position(
        FakeMsg(
            "GLOBAL_POSITION_INT",
            lat=-353632621,
            lon=1491652374,
            alt=584080,
            relative_alt=1000,
            vx=300,
            vy=400,
            vz=-50,
            hdg=9000,
        )
    )
    telemetry = adapter.get_telemetry()

    assert telemetry.position.lat_deg == pytest.approx(-35.3632621)
    assert telemetry.position.lon_deg == pytest.approx(149.1652374)
    assert telemetry.position.alt_msl_m == pytest.approx(584.08)
    assert telemetry.position.relative_alt_m == pytest.approx(1.0)
    assert telemetry.velocity.groundspeed_ms == pytest.approx(5.0)  # hypot(3,4)
    assert telemetry.velocity.heading_deg == pytest.approx(90.0)


def test_global_position_unknown_heading() -> None:
    adapter = PymavlinkAdapter("udp:127.0.0.1:14550")
    _lock_autopilot(adapter)
    adapter._on_global_position(
        FakeMsg(
            "GLOBAL_POSITION_INT",
            lat=0,
            lon=0,
            alt=0,
            relative_alt=0,
            vx=0,
            vy=0,
            vz=0,
            hdg=65535,
        )
    )
    assert adapter.get_telemetry().velocity.heading_deg is None


def test_attitude_converts_to_degrees() -> None:
    adapter = PymavlinkAdapter("udp:127.0.0.1:14550")
    _lock_autopilot(adapter)
    adapter._on_attitude(FakeMsg("ATTITUDE", roll=0.0, pitch=0.0, yaw=1.5707963267948966))
    attitude = adapter.get_telemetry().attitude

    assert attitude.roll_deg == pytest.approx(0.0)
    assert attitude.yaw_deg == pytest.approx(90.0)


def test_gps_raw_populates_fix() -> None:
    adapter = PymavlinkAdapter("udp:127.0.0.1:14550")
    _lock_autopilot(adapter)
    adapter._on_gps_raw(FakeMsg("GPS_RAW_INT", fix_type=3, satellites_visible=11))
    gps = adapter.get_state().gps

    assert gps.fix_type == 3
    assert gps.fix_label == "3d_fix"
    assert gps.satellites_visible == 11


def test_connect_runs_reader_thread_and_ingests(monkeypatch: pytest.MonkeyPatch) -> None:
    fake = FakeMaster(messages=[_heartbeat(armed=False, custom_mode=4)], flightmode="GUIDED")
    monkeypatch.setattr(
        "mavctl.adapter.pymavlink_adapter.mavutil.mavlink_connection",
        lambda *args, **kwargs: fake,
    )

    adapter = PymavlinkAdapter("udp:127.0.0.1:14550")
    adapter.connect()
    try:
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline and not adapter.get_state().connected:
            time.sleep(0.02)
        state = adapter.get_state()
    finally:
        adapter.disconnect()

    assert state.connected is True
    assert state.flight_mode == "GUIDED"
    assert fake.closed is True


# -- command sending (ACK / retry / timeout / NACK) ------------------------


def _cmd_adapter(
    *,
    command_ack_timeout_s: float = 0.05,
    command_retries: int = 3,
    command_ack_settle_s: float = 0.0,
    lock_target: bool = True,
) -> tuple[PymavlinkAdapter, FakeMaster]:
    adapter = PymavlinkAdapter(
        "udp:127.0.0.1:14550",
        command_ack_timeout_s=command_ack_timeout_s,
        command_retries=command_retries,
        command_ack_settle_s=command_ack_settle_s,
    )
    master = FakeMaster()
    adapter._master = master
    if lock_target:
        adapter._on_heartbeat(_heartbeat())
    return adapter, master


def _wire_acks(adapter: PymavlinkAdapter, master: FakeMaster, results: list[int | None]) -> None:
    """Make each command_long_send deliver the next scripted MAV_RESULT.

    A ``None`` entry simulates no ACK arriving (drives the retry/timeout path).
    ACKs are delivered from the adapter's locked target (default 1/1) so the
    source-validation check accepts them.
    """

    state = {"i": 0}

    def responder(tsys: int, tcomp: int, command: int, confirmation: int, *params: float) -> None:
        master.sent.append((command, confirmation, params))
        i = state["i"]
        state["i"] += 1
        result = results[i] if i < len(results) else None
        if result is not None:
            adapter._on_command_ack(
                FakeMsg(
                    "COMMAND_ACK",
                    src_system=adapter._target_system,
                    src_component=adapter._target_component,
                    command=command,
                    result=result,
                )
            )

    master.mav.command_long_send = responder


def test_arm_accepted() -> None:
    adapter, master = _cmd_adapter()
    _wire_acks(adapter, master, [0])
    outcome = adapter.arm()

    assert outcome.accepted is True
    assert outcome.result_name == "ACCEPTED"
    assert outcome.attempts == 1
    command, _conf, params = master.sent[0]
    assert command == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM
    assert params[0] == 1.0


def test_arm_sends_unconditional_param2_zero() -> None:
    """arm() must never send the 21196 magic: pre-arm checks are not bypassable."""

    adapter, master = _cmd_adapter()
    _wire_acks(adapter, master, [0])
    outcome = adapter.arm()

    assert outcome.accepted is True
    command, _conf, params = master.sent[0]
    assert command == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM
    assert params[0] == 1.0
    assert params[1] == 0.0


def test_command_nack() -> None:
    adapter, master = _cmd_adapter()
    _wire_acks(adapter, master, [2])  # MAV_RESULT_DENIED
    outcome = adapter.disarm()

    assert outcome.accepted is False
    assert outcome.result_code == 2
    assert outcome.result_name == "DENIED"
    assert outcome.attempts == 1


def test_command_timeout_after_retries() -> None:
    adapter, master = _cmd_adapter()
    _wire_acks(adapter, master, [None, None, None])
    outcome = adapter.arm()

    assert outcome.timed_out is True
    assert outcome.accepted is False
    assert outcome.attempts == 3
    assert len(master.sent) == 3  # resent once per attempt


def test_command_retry_then_success() -> None:
    adapter, master = _cmd_adapter()
    _wire_acks(adapter, master, [None, 0])
    outcome = adapter.arm()

    assert outcome.accepted is True
    assert outcome.attempts == 2
    assert len(master.sent) == 2


def test_set_mode_resolves_custom_number() -> None:
    adapter, master = _cmd_adapter()
    _wire_acks(adapter, master, [0])
    outcome = adapter.set_mode("guided")

    assert outcome.accepted is True
    command, _conf, params = master.sent[0]
    assert command == mavutil.mavlink.MAV_CMD_DO_SET_MODE
    assert params[1] == 4.0  # GUIDED custom mode number from the fake mode map


def test_set_mode_unknown_raises() -> None:
    adapter, _master = _cmd_adapter()
    with pytest.raises(ValueError, match="unknown flight mode"):
        adapter.set_mode("NOPE")


def test_takeoff_sends_target_altitude() -> None:
    adapter, master = _cmd_adapter()
    _wire_acks(adapter, master, [0])
    adapter.takeoff(12.5)

    command, _conf, params = master.sent[0]
    assert command == mavutil.mavlink.MAV_CMD_NAV_TAKEOFF
    assert params[6] == 12.5


def test_mode_names_from_mapping() -> None:
    adapter, _master = _cmd_adapter()
    assert "GUIDED" in adapter.mode_names()
    assert adapter.mode_names() == sorted(adapter.mode_names())


def test_command_in_progress_counts_as_accepted() -> None:
    adapter, master = _cmd_adapter()
    _wire_acks(adapter, master, [5])  # MAV_RESULT_IN_PROGRESS
    outcome = adapter.arm()

    assert outcome.accepted is True
    assert outcome.in_progress is True
    assert outcome.result_name == "IN_PROGRESS"
    assert outcome.timed_out is False


# -- ACK source validation (P0) --------------------------------------------


def test_ack_rejected_before_target_locked() -> None:
    """Even a sys=1/comp=1 ACK must be ignored until autopilot HEARTBEAT locks."""

    adapter, master = _cmd_adapter(lock_target=False, command_retries=1, command_ack_timeout_s=0.05)
    assert adapter._target_locked is False

    def responder(tsys: int, tcomp: int, command: int, confirmation: int, *params: float) -> None:
        master.sent.append((command, confirmation, params))
        adapter._on_command_ack(
            FakeMsg("COMMAND_ACK", src_system=1, src_component=1, command=command, result=0)
        )

    master.mav.command_long_send = responder
    outcome = adapter.arm()

    assert outcome.timed_out is True
    assert outcome.accepted is False
    with adapter._ack_cond:
        assert adapter._acks == {}


def test_ack_from_wrong_source_is_ignored() -> None:
    adapter, master = _cmd_adapter()

    state = {"i": 0}

    def responder(tsys: int, tcomp: int, command: int, confirmation: int, *params: float) -> None:
        master.sent.append((command, confirmation, params))
        state["i"] += 1
        # ACK from a different system/component than the locked autopilot.
        adapter._on_command_ack(
            FakeMsg("COMMAND_ACK", src_system=7, src_component=99, command=command, result=0)
        )

    master.mav.command_long_send = responder
    outcome = adapter.arm()

    # The foreign ACK is dropped, so the transaction times out after retries.
    assert outcome.timed_out is True
    assert outcome.accepted is False


def test_ack_from_locked_target_is_accepted() -> None:
    adapter, master = _cmd_adapter()
    _wire_acks(adapter, master, [0])
    assert adapter.arm().accepted is True


# -- heartbeat component filtering (P1) ------------------------------------


def _hb(system: int, component: int, armed: bool, custom_mode: int, system_status: int) -> FakeMsg:
    return FakeMsg(
        "HEARTBEAT",
        src_system=system,
        src_component=component,
        base_mode=_ARMED_FLAG if armed else 0,
        custom_mode=custom_mode,
        system_status=system_status,
    )


def test_non_autopilot_heartbeat_does_not_update_snapshot() -> None:
    adapter = PymavlinkAdapter("udp:127.0.0.1:14550")
    adapter._master = FakeMaster(flightmode="GUIDED")

    # Autopilot (component 1) locks and populates the snapshot.
    adapter._on_heartbeat(_hb(1, 1, armed=True, custom_mode=4, system_status=4))
    first = adapter.get_state()
    assert first.armed is True
    assert first.system_id == 1

    # A gimbal on the same system (component 154) must NOT overwrite anything.
    adapter._on_heartbeat(_hb(1, 154, armed=False, custom_mode=0, system_status=3))
    after = adapter.get_state()
    assert after.armed is True  # unchanged
    assert after.component_id == 1  # still the autopilot

    # A different vehicle entirely (system 2) is ignored too.
    adapter._on_heartbeat(_hb(2, 1, armed=False, custom_mode=0, system_status=3))
    assert adapter.get_state().armed is True


def test_target_locks_onto_autopilot_even_if_seen_after_others() -> None:
    adapter = PymavlinkAdapter("udp:127.0.0.1:14550")
    adapter._master = FakeMaster()

    # A non-autopilot heartbeat arrives first and must be ignored (no lock).
    adapter._on_heartbeat(_hb(1, 154, armed=True, custom_mode=0, system_status=4))
    assert adapter.get_state().connected is False  # no autopilot yet

    adapter._on_heartbeat(_hb(1, 1, armed=True, custom_mode=4, system_status=4))
    assert adapter.get_state().connected is True
    assert adapter._target_locked is True


def test_snapshot_telemetry_ignored_before_target_lock() -> None:
    adapter = PymavlinkAdapter("udp:127.0.0.1:14550")
    adapter._on_sys_status(
        FakeMsg("SYS_STATUS", voltage_battery=12600, current_battery=100, battery_remaining=50)
    )
    adapter._on_gps_raw(FakeMsg("GPS_RAW_INT", fix_type=3, satellites_visible=10))
    adapter._on_global_position(
        FakeMsg(
            "GLOBAL_POSITION_INT",
            lat=1,
            lon=1,
            alt=1000,
            relative_alt=500,
            vx=0,
            vy=0,
            vz=0,
            hdg=0,
        )
    )
    assert adapter.get_state().battery.voltage_v is None
    assert adapter.get_state().gps.fix_type is None
    assert adapter.get_telemetry().position.relative_alt_m is None


def test_foreign_snapshot_telemetry_does_not_overwrite_locked() -> None:
    """GPS / position / battery from other system or non-autopilot must not stick."""

    adapter = PymavlinkAdapter("udp:127.0.0.1:14550")
    adapter._master = FakeMaster(flightmode="GUIDED")
    adapter._on_heartbeat(_hb(1, 1, armed=True, custom_mode=4, system_status=4))

    adapter._on_sys_status(
        FakeMsg(
            "SYS_STATUS",
            src_system=1,
            src_component=1,
            voltage_battery=12600,
            current_battery=1000,
            battery_remaining=80,
        )
    )
    adapter._on_gps_raw(
        FakeMsg("GPS_RAW_INT", src_system=1, src_component=1, fix_type=6, satellites_visible=12)
    )
    adapter._on_global_position(
        FakeMsg(
            "GLOBAL_POSITION_INT",
            src_system=1,
            src_component=1,
            lat=-353632621,
            lon=1491652374,
            alt=584080,
            relative_alt=2000,
            vx=0,
            vy=0,
            vz=0,
            hdg=0,
        )
    )

    # Foreign component (gimbal) and foreign system attempt to overwrite.
    adapter._on_sys_status(
        FakeMsg(
            "SYS_STATUS",
            src_system=1,
            src_component=154,
            voltage_battery=5000,
            current_battery=0,
            battery_remaining=1,
        )
    )
    adapter._on_gps_raw(
        FakeMsg("GPS_RAW_INT", src_system=2, src_component=1, fix_type=1, satellites_visible=0)
    )
    adapter._on_global_position(
        FakeMsg(
            "GLOBAL_POSITION_INT",
            src_system=1,
            src_component=154,
            lat=0,
            lon=0,
            alt=0,
            relative_alt=999000,
            vx=0,
            vy=0,
            vz=0,
            hdg=0,
        )
    )
    adapter._on_attitude(
        FakeMsg(
            "ATTITUDE",
            src_system=2,
            src_component=1,
            roll=1.0,
            pitch=1.0,
            yaw=1.0,
        )
    )

    state = adapter.get_state()
    tel = adapter.get_telemetry()
    assert state.battery.voltage_v == pytest.approx(12.6)
    assert state.battery.remaining_pct == 80
    assert state.gps.fix_type == 6
    assert state.gps.satellites_visible == 12
    assert tel.position.relative_alt_m == pytest.approx(2.0)
    assert tel.attitude.roll_deg is None  # foreign attitude dropped; never set


# -- freshness metadata + HOME_POSITION / EXTENDED_SYS_STATE filtering ------


class _FakeMonotonicClock:
    """Deterministic ``time.monotonic()`` replacement (no sleeps, no flakes)."""

    def __init__(self, start: float = 1000.0) -> None:
        self.now = start

    def advance(self, seconds: float) -> None:
        self.now += seconds

    def __call__(self) -> float:
        return self.now


def _freeze_clock(monkeypatch: pytest.MonkeyPatch, clock: _FakeMonotonicClock) -> None:
    monkeypatch.setattr("mavctl.adapter.pymavlink_adapter.time.monotonic", clock)


def _home_msg(system: int = 1, component: int = 1) -> FakeMsg:
    return FakeMsg(
        "HOME_POSITION",
        src_system=system,
        src_component=component,
        latitude=-353632621,
        longitude=1491652374,
        altitude=584080,
    )


def _ext_state_msg(landed: int, system: int = 1, component: int = 1) -> FakeMsg:
    return FakeMsg(
        "EXTENDED_SYS_STATE",
        src_system=system,
        src_component=component,
        landed_state=landed,
    )


def test_home_position_from_locked_autopilot_updates_state_and_age(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    clock = _FakeMonotonicClock()
    _freeze_clock(monkeypatch, clock)
    adapter = PymavlinkAdapter("udp:127.0.0.1:14550")
    adapter._master = FakeMaster(flightmode="GUIDED")
    adapter._on_heartbeat(_hb(1, 1, armed=False, custom_mode=0, system_status=3))

    adapter._on_home_position(_home_msg())
    clock.advance(0.05)

    state = adapter.get_state()
    assert state.home_position is not None
    assert state.home_position.lat_deg == pytest.approx(-35.3632621)
    assert state.home_position.alt_msl_m == pytest.approx(584.08)
    assert state.home_position_age_s == pytest.approx(0.05)


def test_foreign_home_position_never_overwrites_locked_home(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    clock = _FakeMonotonicClock()
    _freeze_clock(monkeypatch, clock)
    adapter = PymavlinkAdapter("udp:127.0.0.1:14550")
    adapter._master = FakeMaster(flightmode="GUIDED")
    adapter._on_heartbeat(_hb(1, 1, armed=False, custom_mode=0, system_status=3))
    adapter._on_home_position(_home_msg())

    clock.advance(1.0)
    # A foreign vehicle and a gimbal component both try to set home.
    adapter._on_home_position(_home_msg(system=2, component=1))
    adapter._on_home_position(_home_msg(system=1, component=154))

    state = adapter.get_state()
    assert state.home_position is not None
    assert state.home_position.lat_deg == pytest.approx(-35.3632621)  # original
    # Age still counts from the accepted message — not refreshed by foreign.
    assert state.home_position_age_s == pytest.approx(1.0)


def test_extended_sys_state_from_locked_autopilot_updates_landed_and_age(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    clock = _FakeMonotonicClock()
    _freeze_clock(monkeypatch, clock)
    adapter = PymavlinkAdapter("udp:127.0.0.1:14550")
    adapter._master = FakeMaster(flightmode="GUIDED")
    adapter._on_heartbeat(_hb(1, 1, armed=False, custom_mode=0, system_status=3))

    adapter._on_extended_sys_state(_ext_state_msg(1))
    clock.advance(0.05)

    state = adapter.get_state()
    assert state.landed_state == "on_ground"
    assert state.landed_state_age_s == pytest.approx(0.05)


def test_foreign_extended_sys_state_never_overwrites_locked_landed_state(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    clock = _FakeMonotonicClock()
    _freeze_clock(monkeypatch, clock)
    adapter = PymavlinkAdapter("udp:127.0.0.1:14550")
    adapter._master = FakeMaster(flightmode="GUIDED")
    adapter._on_heartbeat(_hb(1, 1, armed=False, custom_mode=0, system_status=3))
    adapter._on_extended_sys_state(_ext_state_msg(1))

    clock.advance(1.0)
    adapter._on_extended_sys_state(_ext_state_msg(2, system=2, component=1))
    adapter._on_extended_sys_state(_ext_state_msg(2, system=1, component=154))

    state = adapter.get_state()
    assert state.landed_state == "on_ground"  # original
    assert state.landed_state_age_s == pytest.approx(1.0)  # not refreshed


def test_home_and_landed_state_ignored_before_target_lock() -> None:
    adapter = PymavlinkAdapter("udp:127.0.0.1:14550")
    adapter._on_home_position(_home_msg())
    adapter._on_extended_sys_state(_ext_state_msg(1))

    state = adapter.get_state()
    assert state.home_position is None
    assert state.landed_state is None
    assert state.home_position_age_s is None
    assert state.landed_state_age_s is None


def test_freshness_ages_none_then_recent_then_grow_while_disconnected(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """Ages: None → near-zero on receipt → keep growing after heartbeat loss."""

    clock = _FakeMonotonicClock()
    _freeze_clock(monkeypatch, clock)
    adapter = PymavlinkAdapter("udp:127.0.0.1:14550")
    adapter._master = FakeMaster(flightmode="GUIDED")

    # Nothing received yet: every age is None.
    state = adapter.get_state()
    assert state.telemetry_age_s is None
    assert state.gps_age_s is None
    assert state.battery_age_s is None
    assert state.home_position_age_s is None
    assert state.landed_state_age_s is None

    adapter._on_heartbeat(_hb(1, 1, armed=False, custom_mode=0, system_status=3))
    adapter._on_sys_status(
        FakeMsg(
            "SYS_STATUS",
            src_system=1,
            src_component=1,
            voltage_battery=12600,
            current_battery=1000,
            battery_remaining=80,
        )
    )
    adapter._on_gps_raw(
        FakeMsg("GPS_RAW_INT", src_system=1, src_component=1, fix_type=6, satellites_visible=12)
    )
    adapter._on_global_position(
        FakeMsg(
            "GLOBAL_POSITION_INT",
            src_system=1,
            src_component=1,
            lat=1,
            lon=1,
            alt=1000,
            relative_alt=500,
            vx=0,
            vy=0,
            vz=0,
            hdg=0,
        )
    )
    adapter._on_home_position(_home_msg())
    adapter._on_extended_sys_state(_ext_state_msg(1))

    # Just received: non-None and near zero, without any sleeping.
    clock.advance(0.05)
    state = adapter.get_state()
    for field in (
        "telemetry_age_s",
        "gps_age_s",
        "battery_age_s",
        "home_position_age_s",
        "landed_state_age_s",
    ):
        assert getattr(state, field) == pytest.approx(0.05), field

    # Old cached data: ages keep growing even though the heartbeat went
    # stale and volatile flight state is suppressed to None.
    clock.advance(10.0)
    state = adapter.get_state()
    assert state.connected is False  # heartbeat age 10.05s > 3.0s timeout
    assert state.flight_mode is None
    assert state.armed is None
    assert state.battery_age_s == pytest.approx(10.05)
    assert state.gps_age_s == pytest.approx(10.05)
    assert state.telemetry_age_s == pytest.approx(10.05)
    assert state.home_position_age_s == pytest.approx(10.05)
    assert state.landed_state_age_s == pytest.approx(10.05)


def test_rejected_sources_never_set_freshness_timestamps(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """Messages that fail target locking must not create age metadata."""

    clock = _FakeMonotonicClock()
    _freeze_clock(monkeypatch, clock)
    adapter = PymavlinkAdapter("udp:127.0.0.1:14550")
    adapter._master = FakeMaster(flightmode="GUIDED")

    # Before lock: every stream message is refused, ages stay None.
    adapter._on_sys_status(
        FakeMsg("SYS_STATUS", voltage_battery=12600, current_battery=0, battery_remaining=50)
    )
    adapter._on_gps_raw(FakeMsg("GPS_RAW_INT", fix_type=6, satellites_visible=10))
    adapter._on_global_position(
        FakeMsg(
            "GLOBAL_POSITION_INT",
            lat=1,
            lon=1,
            alt=1000,
            relative_alt=0,
            vx=0,
            vy=0,
            vz=0,
            hdg=0,
        )
    )
    adapter._on_attitude(FakeMsg("ATTITUDE", roll=0.0, pitch=0.0, yaw=0.0))
    adapter._on_home_position(_home_msg())
    adapter._on_extended_sys_state(_ext_state_msg(1))
    state = adapter.get_state()
    assert state.battery.voltage_v is None
    assert state.battery_age_s is None
    assert state.gps_age_s is None
    assert state.telemetry_age_s is None
    assert state.home_position is None
    assert state.home_position_age_s is None
    assert state.landed_state_age_s is None

    # After lock: foreign system / non-autopilot component stays refused.
    adapter._on_heartbeat(_hb(1, 1, armed=False, custom_mode=0, system_status=3))
    clock.advance(2.0)
    adapter._on_sys_status(
        FakeMsg(
            "SYS_STATUS",
            src_system=2,
            src_component=1,
            voltage_battery=12600,
            current_battery=0,
            battery_remaining=50,
        )
    )
    adapter._on_gps_raw(
        FakeMsg("GPS_RAW_INT", src_system=1, src_component=154, fix_type=3, satellites_visible=5)
    )
    adapter._on_global_position(
        FakeMsg(
            "GLOBAL_POSITION_INT",
            src_system=2,
            src_component=1,
            lat=1,
            lon=1,
            alt=1000,
            relative_alt=0,
            vx=0,
            vy=0,
            vz=0,
            hdg=0,
        )
    )
    adapter._on_attitude(
        FakeMsg("ATTITUDE", src_system=1, src_component=154, roll=0.0, pitch=0.0, yaw=0.0)
    )
    adapter._on_home_position(_home_msg(system=2, component=1))
    adapter._on_extended_sys_state(_ext_state_msg(2, system=1, component=154))

    state = adapter.get_state()
    assert state.battery_age_s is None
    assert state.gps_age_s is None
    assert state.telemetry_age_s is None
    assert state.home_position_age_s is None
    assert state.landed_state_age_s is None


# -- command transaction serialization (P0) --------------------------------


def _slow_ack_responder(
    adapter: PymavlinkAdapter,
    master: FakeMaster,
    *,
    delay_s: float = 0.03,
    result: int = 0,
    overlap: dict[str, bool] | None = None,
    active: dict[str, bool] | None = None,
) -> None:
    """Install a command_long_send hook that ACKs after a short delay.

    When ``overlap`` / ``active`` are provided, concurrent entry into the
    responder is recorded (proves transactions are serialized).
    """

    if overlap is None:
        overlap = {"seen": False}
    if active is None:
        active = {"in": False}

    def responder(tsys: int, tcomp: int, command: int, confirmation: int, *params: float) -> None:
        if active["in"]:
            overlap["seen"] = True
        active["in"] = True
        master.sent.append((command, confirmation, params))
        time.sleep(delay_s)  # widen the window for an overlap to be observable
        adapter._on_command_ack(
            FakeMsg(
                "COMMAND_ACK",
                src_system=adapter._target_system,
                src_component=adapter._target_component,
                command=command,
                result=result,
            )
        )
        active["in"] = False

    master.mav.command_long_send = responder


def test_concurrent_arm_and_disarm_are_serialized_no_cross_ack() -> None:
    """arm/disarm share MAV_CMD_COMPONENT_ARM_DISARM; transactions must not overlap."""

    adapter, master = _cmd_adapter()

    overlap = {"seen": False}
    active = {"in": False}
    _slow_ack_responder(adapter, master, overlap=overlap, active=active)

    results: dict[str, CommandOutcome] = {}

    def run_arm() -> None:
        results["arm"] = adapter.arm()

    def run_disarm() -> None:
        results["disarm"] = adapter.disarm()

    t1 = threading.Thread(target=run_arm)
    t2 = threading.Thread(target=run_disarm)
    t1.start()
    t2.start()
    t1.join(5.0)
    t2.join(5.0)

    assert overlap["seen"] is False, "command transactions overlapped"
    assert len(master.sent) == 2
    assert results["arm"].accepted is True
    assert results["disarm"].accepted is True


def test_concurrent_same_command_are_serialized() -> None:
    """Two concurrent arm() calls must run one-after-another, not interleave."""

    adapter, master = _cmd_adapter()

    overlap = {"seen": False}
    active = {"in": False}
    _slow_ack_responder(adapter, master, delay_s=0.04, overlap=overlap, active=active)

    outcomes: list[CommandOutcome] = []
    barrier = threading.Barrier(2)

    def run_arm() -> None:
        barrier.wait(timeout=2.0)
        outcomes.append(adapter.arm())

    t1 = threading.Thread(target=run_arm)
    t2 = threading.Thread(target=run_arm)
    t1.start()
    t2.start()
    t1.join(5.0)
    t2.join(5.0)

    assert overlap["seen"] is False, "same-command transactions overlapped"
    assert len(master.sent) == 2
    assert len(outcomes) == 2
    assert all(o.accepted for o in outcomes)


def test_second_command_cannot_consume_first_ack() -> None:
    """A late ACK for the first transaction must not satisfy a later one.

    arm and disarm share the same MAV_CMD id. If transaction B could see
    transaction A's ACK (or a stale pre-planted ACK), B would falsely succeed
    without ever receiving its own reply. The command lock + stale-ACK clear
    prevent that.
    """

    adapter, master = _cmd_adapter()
    cmd_id = mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM

    # Plant a stale ACCEPTED ACK as if a prior/foreign path left it behind.
    with adapter._ack_cond:
        adapter._acks[cmd_id] = (0, time.monotonic() - 1.0)

    first_send = threading.Event()
    release_first_ack = threading.Event()
    sends: list[tuple[float, ...]] = []

    def responder(tsys: int, tcomp: int, command: int, confirmation: int, *params: float) -> None:
        master.sent.append((command, confirmation, params))
        sends.append(params)
        first_send.set()
        # First transaction: wait until the test injects a controlled ACK path.
        if len(master.sent) == 1:
            release_first_ack.wait(timeout=2.0)
            adapter._on_command_ack(
                FakeMsg(
                    "COMMAND_ACK",
                    src_system=1,
                    src_component=1,
                    command=command,
                    result=0,
                )
            )
            return
        # Second transaction: must get its own ACK; a leftover first ACK must
        # not count. Deliver DENIED so we can prove this send was matched.
        adapter._on_command_ack(
            FakeMsg(
                "COMMAND_ACK",
                src_system=1,
                src_component=1,
                command=command,
                result=2,  # DENIED
            )
        )

    master.mav.command_long_send = responder

    results: dict[str, CommandOutcome] = {}

    def run_arm() -> None:
        results["arm"] = adapter.arm()

    def run_disarm() -> None:
        # Ensure arm has entered its transaction (and holds the lock) first.
        assert first_send.wait(timeout=2.0)
        results["disarm"] = adapter.disarm()

    t_arm = threading.Thread(target=run_arm)
    t_disarm = threading.Thread(target=run_disarm)
    t_arm.start()
    t_disarm.start()
    # Arm is blocked inside send/await; release its ACK so it finishes, then
    # disarm may proceed under the lock.
    assert first_send.wait(timeout=2.0)
    # Give disarm a moment to block on the command lock (must not have sent yet).
    time.sleep(0.05)
    assert len(master.sent) == 1, "second command sent while first transaction open"
    release_first_ack.set()
    t_arm.join(5.0)
    t_disarm.join(5.0)

    assert results["arm"].accepted is True
    assert results["arm"].result_name == "ACCEPTED"
    # Disarm matched its own DENIED ACK, not the prior ACCEPTED.
    assert results["disarm"].accepted is False
    assert results["disarm"].result_name == "DENIED"
    assert len(master.sent) == 2
    assert sends[0][0] == 1.0  # arm param1
    assert sends[1][0] == 0.0  # disarm param1


def test_stale_ack_cleared_at_transaction_start() -> None:
    """Pre-existing ACK for the command id must not short-circuit a new send."""

    adapter, master = _cmd_adapter(command_ack_settle_s=0.0)
    cmd_id = mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM

    # Far-future timestamp: without the transaction-start clear, _await_ack
    # would treat this as a valid post-send ACK and return immediately.
    with adapter._ack_cond:
        adapter._acks[cmd_id] = (0, time.monotonic() + 3600.0)

    # Never deliver a real ACK → must time out (stale entry was discarded).
    _wire_acks(adapter, master, [None, None, None])
    outcome = adapter.arm()

    assert outcome.timed_out is True
    assert outcome.accepted is False
    assert len(master.sent) == 3


def test_late_ack_after_timeout_dropped_during_quarantine() -> None:
    """Post-timeout settle window drops late ACKs for the same command id.

    This is risk reduction only: COMMAND_ACK cannot name which send it answers
    when arm/disarm share MAV_CMD_COMPONENT_ARM_DISARM.
    """

    settle = 0.2
    adapter, master = _cmd_adapter(
        command_ack_timeout_s=0.05,
        command_retries=1,
        command_ack_settle_s=settle,
    )
    cmd_id = mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM
    _wire_acks(adapter, master, [None])  # first arm times out
    t0 = time.monotonic()
    assert adapter.arm().timed_out is True
    with adapter._ack_cond:
        until = adapter._ack_quarantine_until.get(cmd_id)
        assert until is not None and until > time.monotonic()

    # Late ACK arrives during quarantine — must not enter _acks.
    adapter._on_command_ack(
        FakeMsg("COMMAND_ACK", src_system=1, src_component=1, command=cmd_id, result=0)
    )
    with adapter._ack_cond:
        assert cmd_id not in adapter._acks

    # Next arm waits out the settle window, then gets a real ACK.
    _wire_acks(adapter, master, [0])
    outcome = adapter.arm()
    assert outcome.accepted is True
    assert time.monotonic() - t0 >= settle * 0.9


def test_get_state_not_blocked_during_command_transaction() -> None:
    """Snapshot reads must stay concurrent with an in-flight command."""

    adapter, master = _cmd_adapter()
    # Re-lock with armed=True for the snapshot assertion.
    adapter._on_heartbeat(_heartbeat(armed=True))
    entered = threading.Event()
    release = threading.Event()

    def responder(tsys: int, tcomp: int, command: int, confirmation: int, *params: float) -> None:
        master.sent.append((command, confirmation, params))
        entered.set()
        release.wait(timeout=2.0)
        adapter._on_command_ack(
            FakeMsg("COMMAND_ACK", src_system=1, src_component=1, command=command, result=0)
        )

    master.mav.command_long_send = responder

    outcome_box: dict[str, CommandOutcome] = {}

    def run_arm() -> None:
        outcome_box["arm"] = adapter.arm()

    t = threading.Thread(target=run_arm)
    t.start()
    assert entered.wait(timeout=2.0)

    # While arm holds the command lock awaiting ACK, get_state must return.
    deadline = time.monotonic() + 1.0
    state = None
    while time.monotonic() < deadline:
        state = adapter.get_state()
        break
    assert state is not None
    assert state.connected is True
    assert state.armed is True

    release.set()
    t.join(5.0)
    assert outcome_box["arm"].accepted is True
