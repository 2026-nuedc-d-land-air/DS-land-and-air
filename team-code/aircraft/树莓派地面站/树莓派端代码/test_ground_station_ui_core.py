#!/usr/bin/env python3
"""Tests for dependency-free display and calibration policies."""

from __future__ import annotations

import unittest

from ground_station_protocol import CarPose, FlightTelemetry
from ground_station_ui_core import (
    AIRCRAFT_H_DISPLAY_OFFSET_X_CM,
    AIRCRAFT_H_DISPLAY_OFFSET_Y_CM,
    CAR_A_DISPLAY_OFFSET_X_CM,
    CAR_A_DISPLAY_OFFSET_Y_CM,
    CalibrationStability,
    FIELD_LANDMARKS_CM,
    TimedCarPose,
    TimedTelemetry,
    aircraft_telemetry_to_map_display,
    calibration_candidate,
    car_pose_to_map_display,
    field_projection,
    freshness,
    stage_countdown,
)


def telemetry(x: int = 100, y: int = 200, source_ms: int = 1000, mode: int = 0):
    return FlightTelemetry(0, 1, mode, x, y, 150, 0, 0, source_ms)


def car(
    x: int = 10,
    y: int = 20,
    source_ms: int = 1000,
    flags: int = 0x09,
    calibration_id: int = 0,
):
    return CarPose(0, flags, calibration_id, x, y, 0, 0x7FFF, 0x7FFF, source_ms)


class FreshnessTests(unittest.TestCase):
    def test_car_warning_and_invalid_thresholds(self) -> None:
        self.assertEqual(freshness(10.0, 10.28, warning_after=0.28, invalid_after=0.5).state, "fresh")
        self.assertEqual(freshness(10.0, 10.281, warning_after=0.28, invalid_after=0.5).state, "warning")
        self.assertEqual(freshness(10.0, 10.501, warning_after=0.28, invalid_after=0.5).state, "invalid")


class CalibrationPolicyTests(unittest.TestCase):
    def test_candidate_requires_fresh_unconfigured_stationary_car(self) -> None:
        candidate, reason = calibration_candidate(
            TimedTelemetry(telemetry(), 10.0),
            TimedCarPose(car(), 7, 10.1),
            10.2,
            flight_freshness_seconds=1.2,
        )
        self.assertEqual((candidate.delta_x_cm, candidate.delta_y_cm), (90, 180))
        self.assertEqual(reason, "可采样")

        candidate, reason = calibration_candidate(
            TimedTelemetry(telemetry(), 10.0),
            TimedCarPose(car(flags=0x0B, calibration_id=9), 7, 10.1),
            10.2,
            flight_freshness_seconds=1.2,
        )
        self.assertIsNone(candidate)
        self.assertIn("READY", reason)

        candidate, reason = calibration_candidate(
            TimedTelemetry(telemetry(mode=1), 10.0),
            TimedCarPose(car(), 7, 10.1),
            10.2,
            flight_freshness_seconds=1.2,
        )
        self.assertIsNone(candidate)
        self.assertIn("任务阶段", reason)

    def test_stability_needs_distinct_snapshots_and_bounded_spread(self) -> None:
        stability = CalibrationStability(sample_count=3, threshold_cm=5.0)
        for index, offset in enumerate((0, 1, 2)):
            candidate, _ = calibration_candidate(
                TimedTelemetry(telemetry(x=100 + offset, source_ms=1000 + index), 10.0),
                TimedCarPose(car(source_ms=2000 + index), index, 10.0),
                10.1,
                flight_freshness_seconds=1.2,
            )
            stability.observe(candidate)
        self.assertTrue(stability.stable)
        self.assertEqual(stability.count, 3)
        stability.observe(candidate)
        self.assertEqual(stability.count, 3)

    def test_on_platform_countdown_is_only_derived_countdown(self) -> None:
        self.assertIn("3.5s", stage_countdown(9, 10.0, 11.5))
        self.assertIn("协议未提供", stage_countdown(8, 10.0, 11.5))


class FieldProjectionTests(unittest.TestCase):
    def test_field_corners_and_outside_clamping(self) -> None:
        self.assertEqual(field_projection(0, 0, 400, 500), (400, 500, True))
        self.assertEqual(field_projection(500, 400, 400, 500), (0, 0, True))
        self.assertEqual(field_projection(600, -20, 400, 500), (400, 0, False))

    def test_official_landmarks_align_with_clean_map(self) -> None:
        expected_pixels = {
            "H": (112.5, 387.5),
            "A": (150.0, 300.0),
            "B": (150.0, 150.0),
            "C": (300.0, 150.0),
            "D": (300.0, 300.0),
        }
        for name, coordinate in FIELD_LANDMARKS_CM.items():
            projected_x, projected_y, inside = field_projection(*coordinate, 400, 500)
            self.assertTrue(inside, name)
            expected_x, expected_y = expected_pixels[name]
            self.assertAlmostEqual(projected_x, expected_x, msg=name)
            self.assertAlmostEqual(projected_y, expected_y, msg=name)

    def test_aircraft_h_translation_is_display_only(self) -> None:
        raw = telemetry(x=0, y=0)
        display = aircraft_telemetry_to_map_display(raw)
        self.assertEqual(
            (display.x_cm, display.y_cm),
            (AIRCRAFT_H_DISPLAY_OFFSET_X_CM, AIRCRAFT_H_DISPLAY_OFFSET_Y_CM),
        )
        self.assertEqual((raw.x_cm, raw.y_cm), (0, 0))
        self.assertEqual(display.z_cm, raw.z_cm)
        self.assertEqual(display.source_time_ms, raw.source_time_ms)

    def test_car_uses_a_before_calibration_and_h_after_calibration(self) -> None:
        uncalibrated = car(x=0, y=0)
        uncalibrated_display = car_pose_to_map_display(uncalibrated)
        self.assertEqual(
            (uncalibrated_display.x_cm, uncalibrated_display.y_cm),
            (CAR_A_DISPLAY_OFFSET_X_CM, CAR_A_DISPLAY_OFFSET_Y_CM),
        )
        self.assertEqual((uncalibrated.x_cm, uncalibrated.y_cm), (0, 0))

        calibrated = car(x=0, y=0, flags=0x0B, calibration_id=1)
        calibrated_display = car_pose_to_map_display(calibrated)
        self.assertEqual(
            (calibrated_display.x_cm, calibrated_display.y_cm),
            (AIRCRAFT_H_DISPLAY_OFFSET_X_CM, AIRCRAFT_H_DISPLAY_OFFSET_Y_CM),
        )
        self.assertEqual((calibrated.x_cm, calibrated.y_cm), (0, 0))


if __name__ == "__main__":
    unittest.main()
