#!/usr/bin/env python3
"""Protocol-level acceptance tests for D-task V2.3."""

from __future__ import annotations

import struct
import unittest

from ground_station_protocol import (
    ADDRESS_AIRCRAFT,
    ADDRESS_BROADCAST,
    ADDRESS_CAR,
    ADDRESS_GROUND_STATION,
    FLAG_ACK_REQUIRED,
    CarPose,
    MaintenanceReset,
    MessageType,
    PayloadError,
    SequenceTracker,
    StreamParser,
    V2Frame,
    crc16_ccitt_false,
    decode_ack,
    decode_car_pose,
    decode_car_task_request,
    decode_flight_telemetry,
    decode_maintenance_reset,
    encode_calibration_set,
    encode_v2,
    source_time_relation,
)


CAR_VECTOR = bytes.fromhex(
    "AA 55 02 80 30 10 42 00 16 01 0F 00 01 00 00 00 78 FF FF FF CE "
    "03 84 00 0A 00 00 00 00 04 D2 B2 2D"
)
TASK_VECTOR = bytes.fromhex(
    "AA 55 02 81 30 20 43 01 0C 01 01 00 01 00 01 00 00 00 00 04 D2 7F 78"
)
CALIBRATION_VECTOR = bytes.fromhex(
    "AA 55 02 83 40 30 16 01 0C 00 00 00 64 FF FF FF CE 00 01 01 00 F1 24"
)
RESET_VECTOR = bytes.fromhex(
    "AA 55 02 85 30 10 52 00 08 00 01 01 00 00 00 04 D2 9C B2"
)


class CrcAndVectorTests(unittest.TestCase):
    def test_crc_standard_vector(self) -> None:
        self.assertEqual(crc16_ccitt_false(b"123456789"), 0x29B1)

    def test_all_documented_vectors_recalculate(self) -> None:
        for vector in (CAR_VECTOR, TASK_VECTOR, CALIBRATION_VECTOR, RESET_VECTOR):
            expected = int.from_bytes(vector[-2:], "big")
            self.assertEqual(crc16_ccitt_false(vector[2:-2]), expected)

    def test_calibration_encoder_matches_appendix(self) -> None:
        self.assertEqual(encode_calibration_set(100, -50, 1, 0x16), CALIBRATION_VECTOR)

    def test_documented_car_and_task_decode(self) -> None:
        car_frame = StreamParser().feed(CAR_VECTOR)[0]
        pose = decode_car_pose(car_frame.payload)
        self.assertEqual(
            pose,
            CarPose(1, 0x0F, 1, 120, -50, 900, 10, 0, 1234),
        )
        task_frame = StreamParser().feed(TASK_VECTOR)[0]
        task = decode_car_task_request(task_frame.payload)
        self.assertEqual((task.task_type, task.mission_id, task.calibration_id), (1, 1, 1))

    def test_v23_maintenance_reset_vector_and_schema(self) -> None:
        frame = StreamParser().feed(RESET_VECTOR)[0]
        self.assertEqual(
            (frame.message_type, frame.source, frame.destination, frame.sequence, frame.flags),
            (MessageType.MAINTENANCE_RESET, ADDRESS_CAR, ADDRESS_BROADCAST, 0x52, 0),
        )
        self.assertEqual(
            decode_maintenance_reset(frame.payload),
            MaintenanceReset(1, 0x01, 1234),
        )
        for bad_payload in (
            struct.pack(">HBBI", 0, 1, 0, 1234),
            struct.pack(">HBBI", 1, 0, 0, 1234),
            struct.pack(">HBBI", 1, 3, 0, 1234),
            struct.pack(">HBBI", 1, 1, 1, 1234),
            bytes(7),
        ):
            with self.assertRaises(PayloadError):
                decode_maintenance_reset(bad_payload)


class StreamParserTests(unittest.TestCase):
    @staticmethod
    def telemetry_frame(sequence: int = 7) -> bytes:
        payload = struct.pack(">HBBiiihHI", 0x0017, 1, 4, -10, 20, 300, -905, 123, 9000)
        return encode_v2(
            MessageType.FLIGHT_TELEMETRY,
            ADDRESS_AIRCRAFT,
            ADDRESS_BROADCAST,
            sequence,
            0,
            payload,
        )

    def test_arbitrary_split_sticky_and_noise(self) -> None:
        parser = StreamParser()
        stream = b"noise\xaa\xbbold" + CAR_VECTOR + self.telemetry_frame()
        frames = []
        for byte in stream:
            frames.extend(parser.feed(bytes((byte,)), now=1.0))
        self.assertEqual([frame.message_type for frame in frames], [0x80, 0x02])
        self.assertEqual(parser.stats.valid_frames, 2)
        self.assertGreaterEqual(parser.stats.noise_bytes, len(b"noise\xaa\xbbold"))

    def test_crc_error_does_not_block_next_frame(self) -> None:
        parser = StreamParser()
        damaged = bytearray(self.telemetry_frame())
        damaged[12] ^= 0x80
        frames = parser.feed(bytes(damaged) + self.telemetry_frame(8))
        self.assertEqual([frame.sequence for frame in frames], [8])
        self.assertEqual(parser.stats.crc_errors, 1)

    def test_illegal_length_and_version_resynchronize(self) -> None:
        parser = StreamParser()
        bad_length = bytes.fromhex("AA 55 02 02 20 10 01 00 41")
        bad_version = bytes.fromhex("AA 55 01 02 20 10 01 00 00 00 00")
        frames = parser.feed(bad_length + bad_version + CAR_VECTOR)
        self.assertEqual(len(frames), 1)
        self.assertEqual(parser.stats.length_errors, 1)
        self.assertEqual(parser.stats.version_errors, 1)

    def test_partial_frame_expires_at_100_ms(self) -> None:
        parser = StreamParser(inter_byte_timeout=0.1)
        parser.feed(CAR_VECTOR[:7], now=10.0)
        self.assertFalse(parser.expire(10.100))
        self.assertTrue(parser.expire(10.101))
        self.assertEqual(parser.stats.timeout_errors, 1)

    def test_wrong_destination_is_rejected(self) -> None:
        parser = StreamParser()
        frame = encode_v2(MessageType.HEARTBEAT, ADDRESS_AIRCRAFT, 0x50, 1, 0, bytes(8))
        self.assertEqual(parser.feed(frame), [])
        self.assertEqual(parser.stats.address_errors, 1)

    def test_aircraft_destination_is_observable_only_for_task_lock(self) -> None:
        frame = StreamParser().feed(TASK_VECTOR)[0]
        self.assertEqual(frame.destination, ADDRESS_AIRCRAFT)

    def test_reserved_flags_are_rejected_after_valid_crc(self) -> None:
        raw = bytearray(self.telemetry_frame())
        raw[7] = 0x80
        raw[-2:] = crc16_ccitt_false(raw[2:-2]).to_bytes(2, "big")
        parser = StreamParser()
        self.assertEqual(parser.feed(raw), [])
        self.assertEqual(parser.stats.flag_errors, 1)

    def test_signed_telemetry_values(self) -> None:
        frame = StreamParser().feed(self.telemetry_frame())[0]
        telemetry = decode_flight_telemetry(frame.payload)
        self.assertEqual((telemetry.x_cm, telemetry.yaw_tenths_degree), (-10, -905))

    def test_non_field_global_telemetry_is_rejected(self) -> None:
        payload = struct.pack(">HBBiiihHI", 0, 0, 0, 1, 2, 3, 4, 5, 6)
        with self.assertRaises(PayloadError):
            decode_flight_telemetry(payload)

    def test_car_pose_requires_field_global_and_consistent_velocity(self) -> None:
        non_field = struct.pack(">BBHiihhhI", 0, 0x09, 0, 1, 2, 0, 0x7FFF, 0x7FFF, 6)
        with self.assertRaises(PayloadError):
            decode_car_pose(non_field)

        invalid_velocity = struct.pack(">BBHiihhhI", 1, 0x09, 0, 1, 2, 0, 0, 0x7FFF, 6)
        with self.assertRaises(PayloadError):
            decode_car_pose(invalid_velocity)

    def test_ack_rejects_undefined_result_or_detail(self) -> None:
        with self.assertRaises(PayloadError):
            decode_ack(bytes((MessageType.CALIBRATION_SET, 1, 7, 0)))
        with self.assertRaises(PayloadError):
            decode_ack(bytes((MessageType.CALIBRATION_SET, 1, 0, 7)))


class OrderingTests(unittest.TestCase):
    def test_source_time_wrap_duplicate_and_reset(self) -> None:
        self.assertEqual(source_time_relation(10, None), "new")
        self.assertEqual(source_time_relation(10, 10), "duplicate")
        self.assertEqual(source_time_relation(2, 0xFFFFFFFE), "new")
        self.assertEqual(source_time_relation(900, 1000), "stale")
        self.assertEqual(source_time_relation(100, 65_000), "reset")

    def test_sequence_tracking_is_per_message_type(self) -> None:
        tracker = SequenceTracker()
        car = V2Frame(0x80, ADDRESS_CAR, ADDRESS_BROADCAST, 10, 0, b"", b"")
        task = V2Frame(0x81, ADDRESS_CAR, ADDRESS_AIRCRAFT, 11, FLAG_ACK_REQUIRED, b"", b"")
        car_next = V2Frame(0x80, ADDRESS_CAR, ADDRESS_BROADCAST, 12, 0, b"", b"")
        self.assertEqual(tracker.observe(car), 0)
        self.assertEqual(tracker.observe(task), 0)
        self.assertEqual(tracker.observe(car_next), 1)


if __name__ == "__main__":
    unittest.main()
