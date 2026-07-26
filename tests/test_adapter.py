"""Unit tests for the pymavlink adapter using mocked MAVLink messages."""

from __future__ import annotations

import time

import pytest
from tests.fakes import FakeMaster, FakeMsg

from mavctl.adapter.pymavlink_adapter import PymavlinkAdapter

# MAV_MODE_FLAG_SAFETY_ARMED bit.
_ARMED_FLAG = 0b10000000


def _heartbeat(armed: bool = False, custom_mode: int = 0) -> FakeMsg:
    return FakeMsg(
        "HEARTBEAT",
        base_mode=_ARMED_FLAG if armed else 0,
        custom_mode=custom_mode,
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


def test_sys_status_populates_battery() -> None:
    adapter = PymavlinkAdapter("udp:127.0.0.1:14550")
    adapter._on_sys_status(
        FakeMsg("SYS_STATUS", voltage_battery=12600, current_battery=1550, battery_remaining=87)
    )
    battery = adapter.get_state().battery

    assert battery.voltage_v == pytest.approx(12.6)
    assert battery.current_a == pytest.approx(15.5)
    assert battery.remaining_pct == 87


def test_sys_status_handles_unknown_sentinels() -> None:
    adapter = PymavlinkAdapter("udp:127.0.0.1:14550")
    adapter._on_sys_status(
        FakeMsg("SYS_STATUS", voltage_battery=65535, current_battery=-1, battery_remaining=-1)
    )
    battery = adapter.get_state().battery

    assert battery.voltage_v is None
    assert battery.current_a is None
    assert battery.remaining_pct is None


def test_global_position_populates_position_and_velocity() -> None:
    adapter = PymavlinkAdapter("udp:127.0.0.1:14550")
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
    adapter._on_attitude(FakeMsg("ATTITUDE", roll=0.0, pitch=0.0, yaw=1.5707963267948966))
    attitude = adapter.get_telemetry().attitude

    assert attitude.roll_deg == pytest.approx(0.0)
    assert attitude.yaw_deg == pytest.approx(90.0)


def test_gps_raw_populates_fix() -> None:
    adapter = PymavlinkAdapter("udp:127.0.0.1:14550")
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
