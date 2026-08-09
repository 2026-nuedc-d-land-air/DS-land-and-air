#!/usr/bin/env python3
"""Pure-Python display and pre-flight calibration policies for the Qt UI."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass, replace
import math

from ground_station_protocol import (
    COORDINATE_FIELD_GLOBAL,
    CarPose,
    FlightTelemetry,
)


# FIELD_GLOBAL uses the lower-right field corner as (0, 0), +X upward and
# +Y leftward in the official top-down view. H is the center of its rings:
# each outer edge is 75 cm from the field edge and the outer diameter is 75 cm.
FIELD_LANDMARKS_CM: dict[str, tuple[float, float]] = {
    "H": (112.5, 287.5),
    "A": (200.0, 250.0),
    "B": (350.0, 250.0),
    "C": (350.0, 100.0),
    "D": (200.0, 100.0),
}

# Flight telemetry is reported in the aircraft's H-centred navigation frame.
# The official field map instead uses its lower-right corner as the origin, so
# this is a UI-only translation.  Calibration and CALIBRATION_SET always use
# the unmodified protocol coordinates.
AIRCRAFT_H_DISPLAY_OFFSET_X_CM = 113
AIRCRAFT_H_DISPLAY_OFFSET_Y_CM = 288
CAR_A_DISPLAY_OFFSET_X_CM = 200
CAR_A_DISPLAY_OFFSET_Y_CM = 250


def aircraft_telemetry_to_map_display(telemetry: FlightTelemetry) -> FlightTelemetry:
    """Translate aircraft telemetry for the map/text display only."""

    return replace(
        telemetry,
        x_cm=telemetry.x_cm + AIRCRAFT_H_DISPLAY_OFFSET_X_CM,
        y_cm=telemetry.y_cm + AIRCRAFT_H_DISPLAY_OFFSET_Y_CM,
    )


def car_pose_to_map_display(pose: CarPose) -> CarPose:
    """Translate a car pose for the map/text display without changing its wire value.

    Before calibration, car coordinates are local to its A-point power-on
    position.  After calibration, the car has applied DeltaX/DeltaY and shares
    the aircraft's H-centred navigation coordinates.
    """

    if pose.calibrated:
        offset_x_cm = AIRCRAFT_H_DISPLAY_OFFSET_X_CM
        offset_y_cm = AIRCRAFT_H_DISPLAY_OFFSET_Y_CM
    else:
        offset_x_cm = CAR_A_DISPLAY_OFFSET_X_CM
        offset_y_cm = CAR_A_DISPLAY_OFFSET_Y_CM
    return replace(
        pose,
        x_cm=pose.x_cm + offset_x_cm,
        y_cm=pose.y_cm + offset_y_cm,
    )

@dataclass(frozen=True)
class TimedTelemetry:
    value: FlightTelemetry
    received_at: float


@dataclass(frozen=True)
class TimedCarPose:
    value: CarPose
    sequence: int
    received_at: float


@dataclass(frozen=True)
class Freshness:
    state: str
    age_seconds: float | None
    text: str


@dataclass(frozen=True)
class CalibrationCandidate:
    delta_x_cm: int
    delta_y_cm: int
    flight_source_time_ms: int
    car_source_time_ms: int


def freshness(
    received_at: float | None,
    now: float,
    *,
    warning_after: float,
    invalid_after: float,
) -> Freshness:
    if received_at is None:
        return Freshness("missing", None, "未收到")
    age = max(0.0, now - received_at)
    if age > invalid_after:
        return Freshness("invalid", age, f"失效 {age:.2f}s")
    if age > warning_after:
        return Freshness("warning", age, f"警告 {age:.2f}s")
    return Freshness("fresh", age, f"新鲜 {age:.2f}s")


def calibration_candidate(
    telemetry: TimedTelemetry | None,
    car_pose: TimedCarPose | None,
    now: float,
    *,
    flight_freshness_seconds: float,
    car_freshness_seconds: float = 0.5,
) -> tuple[CalibrationCandidate | None, str]:
    if telemetry is None:
        return None, "等待 FLIGHT_TELEMETRY"
    if now - telemetry.received_at > flight_freshness_seconds:
        return None, "FLIGHT_TELEMETRY 已陈旧"
    flight = telemetry.value
    if flight.coordinate_frame != COORDINATE_FIELD_GLOBAL:
        return None, "飞机坐标系不是 FIELD_GLOBAL"
    if flight.mode_code != 0:
        return None, "飞机已进入任务阶段"
    if car_pose is None:
        return None, "等待未校准 CAR_POSE"
    if now - car_pose.received_at > car_freshness_seconds:
        return None, "CAR_POSE 已失效（>500 ms）"
    car = car_pose.value
    if not car.position_valid:
        return None, "CAR_POSE 的 POSITION_VALID 未置位"
    if car.calibrated:
        return None, "小车已校准，READY 只收模式"
    if car.calibration_id != 0:
        return None, "未校准 CAR_POSE 携带非零 CalibrationId，拒绝维护"
    if car.car_running:
        return None, "小车正在运行"
    return (
        CalibrationCandidate(
            flight.x_cm - car.x_cm,
            flight.y_cm - car.y_cm,
            flight.source_time_ms,
            car.source_time_ms,
        ),
        "可采样",
    )


class CalibrationStability:
    """Track recent distinct snapshots without silently approving calibration."""

    def __init__(self, sample_count: int = 5, threshold_cm: float = 5.0):
        if sample_count < 2:
            raise ValueError("sample_count must be at least two")
        if threshold_cm <= 0:
            raise ValueError("threshold_cm must be greater than zero")
        self.sample_count = sample_count
        self.threshold_cm = threshold_cm
        self._samples: deque[CalibrationCandidate] = deque(maxlen=sample_count)
        self._last_key: tuple[int, int] | None = None

    def clear(self) -> None:
        self._samples.clear()
        self._last_key = None

    def observe(self, candidate: CalibrationCandidate) -> None:
        key = (candidate.flight_source_time_ms, candidate.car_source_time_ms)
        if key == self._last_key:
            return
        self._last_key = key
        self._samples.append(candidate)

    @property
    def count(self) -> int:
        return len(self._samples)

    @property
    def spread_cm(self) -> float | None:
        if len(self._samples) < 2:
            return None
        dx_values = [sample.delta_x_cm for sample in self._samples]
        dy_values = [sample.delta_y_cm for sample in self._samples]
        return math.hypot(max(dx_values) - min(dx_values), max(dy_values) - min(dy_values))

    @property
    def stable(self) -> bool:
        spread = self.spread_cm
        return len(self._samples) >= self.sample_count and spread is not None and spread <= self.threshold_cm


def stage_countdown(stage: int | None, stage_received_at: float | None, now: float) -> str:
    """Show only countdowns explicitly derivable from the V2.3 stage definition."""

    if stage == 9 and stage_received_at is not None:
        remaining = max(0.0, 5.0 - max(0.0, now - stage_received_at))
        return f"平台停留约 {remaining:.1f}s（按状态接收时刻估算）"
    return "—（协议未提供该阶段倒计时）"


def field_projection(
    x_cm: float, y_cm: float, width_px: float, height_px: float
) -> tuple[float, float, bool]:
    """Map FIELD_GLOBAL X=0..500, Y=0..400 to a top-down widget."""

    inside = 0.0 <= x_cm <= 500.0 and 0.0 <= y_cm <= 400.0
    clamped_x = min(500.0, max(0.0, x_cm))
    clamped_y = min(400.0, max(0.0, y_cm))
    # X points upward on screen and +Y points left, matching forward/left.
    px = width_px * (1.0 - clamped_y / 400.0)
    py = height_px * (1.0 - clamped_x / 500.0)
    return px, py, inside
