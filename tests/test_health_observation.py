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

from mavctl.models import (
    ESTIMATOR_STALE_AFTER_S,
    EstimatorObservation,
    HealthObservation,
    PrearmObservation,
    SensorObservation,
    VehicleState,
)

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
