"""Health observation models (Issue #4, observe-only v1).

These models carry **raw vehicle-reported health observations** into
``VehicleState`` / ``status --json``. They are deliberately non-judgemental:

- v1 never derives ``healthy`` / ``safe`` / ``ready`` / ``prearm_passed``
  from any observation — the estimator ``status`` field only ever takes
  ``unknown`` or ``observed``, where ``observed`` means "an
  ``EKF_STATUS_REPORT`` was received", not "the estimator is healthy";
- ``unknown`` is a distinct state and never means healthy;
- ``last_status`` / ``flags`` / variance fields are the **last observed**
  (possibly stale) cache; ``status`` is the current usable verdict, and a
  stale cached observation must never become future guard allow evidence;
- ArduPilot's own pre-arm checks remain the final authority for arming
  safety; nothing in this module gates any command.

All freshness ages are monotonic-clock based (``time.monotonic()``), never
epoch wall-clock, and are independent of the telemetry freshness metadata.
"""

from __future__ import annotations

from typing import Literal

from pydantic import BaseModel, Field

__all__ = [
    "ESTIMATOR_STALE_AFTER_S",
    "EstimatorObservation",
    "HealthObservation",
    "PrearmObservation",
    "SensorObservation",
]

# Policy constant: an estimator observation older than this many monotonic
# seconds is no longer a *current* verdict — ``status`` degrades back to
# ``unknown`` while the cached ``last_status`` / ``flags`` / variances are
# retained for explainability. The EKF_STATUS_REPORT stream is requested at
# 2 Hz, so 5 s tolerates ~10 missed periods before degrading.
ESTIMATOR_STALE_AFTER_S = 5.0


class EstimatorObservation(BaseModel):
    """Raw observation of the vehicle's ``EKF_STATUS_REPORT`` (id 193).

    ``status`` is the current usable verdict and is restricted to
    ``unknown`` (no current usable observation — never received, or the
    last one is older than ``ESTIMATOR_STALE_AFTER_S``) and ``observed``
    (a report is fresh). It never takes a health judgement in v1.
    ``last_status`` is the historical observation marker: ``None`` means
    never received; ``"observed"`` means at least one report arrived (it
    says nothing about estimator health).
    """

    status: Literal["unknown", "observed"] = "unknown"
    last_status: Literal["observed"] | None = None
    flags: list[str] = Field(default_factory=list)
    flags_mask: int | None = None
    velocity_variance: float | None = None
    pos_horiz_variance: float | None = None
    pos_vert_variance: float | None = None
    compass_variance: float | None = None
    terrain_alt_variance: float | None = None
    airspeed_variance: float | None = None
    age_s: float | None = None


class SensorObservation(BaseModel):
    """Raw ``SYS_STATUS`` sensor bitmasks (``MAV_SYS_STATUS_SENSOR_*``).

    The integers are the raw vehicle-reported bitmasks — no health verdict
    is derived from them in v1. ``0`` is a valid raw value (the vehicle
    reports no sensors in that class); there is no sentinel mapping. Bit
    semantics vary by vehicle/firmware, so interpretation is left to the
    consumer and to ArduPilot itself.
    """

    present: int | None = None
    enabled: int | None = None
    health: int | None = None
    age_s: float | None = None


class PrearmObservation(BaseModel):
    """Last matching arming-check diagnostic from ``STATUSTEXT``.

    Only messages from the locked autopilot whose text starts with a known
    arming-check prefix (``PreArm:``, ``Arm:``, ``PreArm_EKF:``) are kept —
    the most recent complete match, never a history. ``severity`` is the
    raw ``MAV_SEVERITY`` value. The absence of a message must never be
    read as "pre-arm passed": text strings are not a stable API and
    failures are only observable when the vehicle reports them.
    """

    last_message: str | None = None
    severity: int | None = None
    age_s: float | None = None


class HealthObservation(BaseModel):
    """Vehicle health observations (observe-only v1).

    Aggregates the estimator, sensor-bitmask and prearm-diagnostic
    observations. Every sub-block starts fully ``unknown``/``None``
    ("never observed") and fills in only from messages actually received
    from the locked autopilot. No field in this model gates any command:
    health data cannot allow or deny anything in v1.
    """

    estimator: EstimatorObservation = Field(default_factory=EstimatorObservation)
    sensors: SensorObservation = Field(default_factory=SensorObservation)
    prearm: PrearmObservation = Field(default_factory=PrearmObservation)
