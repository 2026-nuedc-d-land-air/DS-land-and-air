#!/usr/bin/env python3
"""Simulated-UART safety and timing tests for the V2.3 serial worker."""

from __future__ import annotations

import logging
import struct
import threading
import time
import unittest

from ground_station_protocol import (
    ADDRESS_AIRCRAFT,
    ADDRESS_BROADCAST,
    ADDRESS_CAR,
    ADDRESS_GROUND_STATION,
    FLAG_ACK_REQUIRED,
    FLAG_RETRANSMISSION,
    FLAG_URGENT,
    MessageType,
    StreamParser,
    crc16_ccitt_false,
    decode_car_pose,
    encode_v2,
)
from ground_station_serial import (
    CalibrationEvent,
    CarPoseEvent,
    DEFAULT_SERIAL_DEVICE,
    MaintenanceResetEvent,
    SerialLinkEvent,
    SerialSettings,
    SerialWorker,
    TelemetryEvent,
    TransmitLockEvent,
)


def telemetry_frame(sequence: int, source_ms: int, mode: int = 0, x: int = 100, y: int = 200) -> bytes:
    payload = struct.pack(">HBBiiihHI", 0, 1, mode, x, y, 150, 0, 0, source_ms)
    return encode_v2(MessageType.FLIGHT_TELEMETRY, ADDRESS_AIRCRAFT, ADDRESS_BROADCAST, sequence, 0, payload)


def car_frame(
    sequence: int,
    source_ms: int,
    *,
    x: int = 10,
    y: int = 20,
    flags: int = 0x09,
    calibration_id: int = 0,
    coordinate_frame: int = 1,
) -> bytes:
    payload = struct.pack(">BBHiihhhI", coordinate_frame, flags, calibration_id, x, y, 100, 0x7FFF, 0x7FFF, source_ms)
    return encode_v2(MessageType.CAR_POSE, ADDRESS_CAR, ADDRESS_BROADCAST, sequence, 0, payload)


def task_frame(sequence: int = 9) -> bytes:
    payload = struct.pack(">BBHHHI", 1, 1, 7, 3, 0, 1234)
    return encode_v2(MessageType.CAR_TASK_REQUEST, ADDRESS_CAR, ADDRESS_AIRCRAFT, sequence, FLAG_ACK_REQUIRED, payload)


def mission_abort_frame(sequence: int = 10) -> bytes:
    return encode_v2(
        MessageType.MISSION_ABORT,
        ADDRESS_CAR,
        ADDRESS_AIRCRAFT,
        sequence,
        FLAG_ACK_REQUIRED | FLAG_URGENT,
    )


def ack_frame(request_sequence: int, result: int = 0) -> bytes:
    payload = struct.pack(">BBBB", MessageType.CALIBRATION_SET, request_sequence, result, 0)
    return encode_v2(MessageType.ACK, ADDRESS_CAR, ADDRESS_GROUND_STATION, 44, 0, payload)


def maintenance_reset_frame(
    reset_id: int = 1,
    source_ms: int = 1234,
    *,
    reset_flags: int = 1,
    header_flags: int = 0,
    destination: int = ADDRESS_BROADCAST,
) -> bytes:
    payload = struct.pack(">HBBI", reset_id, reset_flags, 0, source_ms)
    return encode_v2(
        MessageType.MAINTENANCE_RESET,
        ADDRESS_CAR,
        destination,
        0x52,
        header_flags,
        payload,
    )


def one_frame(raw: bytes, *, destinations=None):
    parser = StreamParser() if destinations is None else StreamParser(accepted_destinations=destinations)
    return parser.feed(raw)[0]


class FakeSerial:
    def __init__(self, chunks: list[bytes] | None = None):
        self.chunks = list(chunks or [])
        self.writes: list[bytes] = []
        self.closed = False
        self.dtr = True
        self.rts = True

    def read(self, _size: int = 1) -> bytes:
        if self.chunks:
            return self.chunks.pop(0)
        time.sleep(0.002)
        return b""

    def write(self, data: bytes) -> int:
        self.writes.append(bytes(data))
        return len(data)

    def close(self) -> None:
        self.closed = True


class SerialWorkerTests(unittest.TestCase):
    def make_worker(self, *, send_tolerance: int = 5):
        events: list[object] = []
        worker = SerialWorker(
            SerialSettings(calibration_send_tolerance_cm=send_tolerance, initial_sequence=0x16),
            events.append,
            logging.getLogger("serial-test"),
        )
        fake = FakeSerial()
        worker.port = fake
        return worker, fake, events

    def prime_uncalibrated(self, worker: SerialWorker) -> None:
        worker._process_frame(one_frame(telemetry_frame(1, 1000)), 10.0)
        worker._process_frame(one_frame(car_frame(1, 1000)), 10.01)

    def test_default_device_and_timing_read_timeout(self) -> None:
        settings = SerialSettings()
        self.assertEqual(settings.device, DEFAULT_SERIAL_DEVICE)
        self.assertLessEqual(settings.read_timeout, 0.005)

    def test_calibration_only_sends_at_35_ms_then_closes_ack_pose_loop(self) -> None:
        worker, fake, events = self.make_worker()
        self.prime_uncalibrated(worker)
        worker._queue_calibration(90, 180, 0x1234, 1000, 1000, 10.02)
        worker._process_frame(one_frame(car_frame(7, 1100)), 10.10)
        worker._service_calibration(10.129)
        self.assertEqual(fake.writes, [])
        worker._service_calibration(10.135)
        self.assertEqual(len(fake.writes), 1)
        sent = one_frame(fake.writes[0], destinations=(ADDRESS_CAR,))
        self.assertEqual((sent.message_type, sent.source, sent.destination), (0x83, 0x40, 0x30))
        self.assertEqual(sent.sequence, 0x16)
        self.assertEqual(sent.flags, FLAG_ACK_REQUIRED)

        worker._process_frame(one_frame(ack_frame(0x16)), 10.20)
        self.assertEqual(worker.pending_calibration.state, "waiting_calibrated_pose")
        worker._process_frame(
            one_frame(car_frame(8, 1200, flags=0x0B, calibration_id=0x1234, coordinate_frame=1)),
            10.30,
        )
        self.assertTrue(worker.transmit_locked)
        states = [event.state for event in events if isinstance(event, CalibrationEvent)]
        self.assertIn("waiting_ack", states)
        self.assertIn("waiting_calibrated_pose", states)
        self.assertIn("success", states)
        self.assertTrue(any(isinstance(event, TransmitLockEvent) for event in events))

    def test_non_mod5_slot_never_arms_or_writes(self) -> None:
        worker, fake, _events = self.make_worker()
        self.prime_uncalibrated(worker)
        worker._queue_calibration(90, 180, 2, 1000, 1000, 10.02)
        worker._process_frame(one_frame(car_frame(6, 1100)), 10.10)
        worker._service_calibration(10.20)
        self.assertEqual(worker.pending_calibration.state, "waiting_window")
        self.assertEqual(fake.writes, [])

    def test_missed_window_waits_for_next_slot_without_writing(self) -> None:
        worker, fake, events = self.make_worker()
        self.prime_uncalibrated(worker)
        worker._queue_calibration(90, 180, 2, 1000, 1000, 10.02)
        worker._process_frame(one_frame(car_frame(7, 1100)), 10.10)
        worker._service_calibration(10.151)
        self.assertEqual(fake.writes, [])
        self.assertEqual(worker.pending_calibration.state, "waiting_window")
        self.assertIn("窗口已错过", [e.message for e in events if isinstance(e, CalibrationEvent)][-1])

    def test_task_request_irreversibly_cancels_pending_transmit(self) -> None:
        worker, fake, events = self.make_worker()
        self.prime_uncalibrated(worker)
        worker._queue_calibration(90, 180, 2, 1000, 1000, 10.02)
        worker._process_frame(one_frame(car_frame(7, 1100)), 10.10)
        worker._process_frame(one_frame(task_frame()), 10.11)
        worker._service_calibration(10.135)
        self.assertTrue(worker.transmit_locked)
        self.assertEqual(fake.writes, [])
        worker._queue_calibration(90, 180, 3, 1000, 1100, 10.14)
        self.assertEqual(fake.writes, [])
        self.assertTrue(any(isinstance(event, TransmitLockEvent) for event in events))

    def test_task_envelope_locks_even_when_payload_is_invalid(self) -> None:
        worker, fake, _events = self.make_worker()
        self.prime_uncalibrated(worker)
        invalid_payload = struct.pack(">BBHHHI", 0, 1, 7, 3, 0, 1234)
        raw = encode_v2(
            MessageType.CAR_TASK_REQUEST,
            ADDRESS_CAR,
            ADDRESS_AIRCRAFT,
            9,
            FLAG_ACK_REQUIRED,
            invalid_payload,
        )
        worker._process_frame(one_frame(raw), 10.1)
        self.assertTrue(worker.transmit_locked)
        self.assertEqual(worker.parser.stats.payload_errors, 1)
        self.assertEqual(fake.writes, [])

    def test_mission_abort_irreversibly_disables_transmission(self) -> None:
        worker, fake, _events = self.make_worker()
        self.prime_uncalibrated(worker)
        worker._process_frame(one_frame(mission_abort_frame()), 10.1)
        self.assertTrue(worker.transmit_locked)
        self.assertIn("MISSION_ABORT", worker.transmit_lock_reason)
        self.assertEqual(fake.writes, [])

    def test_invalid_telemetry_payload_never_updates_calibration_snapshot(self) -> None:
        worker, _fake, _events = self.make_worker()
        raw = telemetry_frame(1, 1000)
        corrupted_payload = bytearray(raw)
        corrupted_payload[11] = 0  # FLIGHT_TELEMETRY CoordinateFrame.
        corrupted_payload[-2:] = crc16_ccitt_false(corrupted_payload[2:-2]).to_bytes(2, "big")
        worker._process_frame(one_frame(bytes(corrupted_payload)), 10.0)
        self.assertIsNone(worker.latest_telemetry)
        self.assertEqual(worker.parser.stats.payload_errors, 1)

    def test_existing_calibrated_pose_enters_ready_receive_only(self) -> None:
        worker, fake, _events = self.make_worker()
        worker._process_frame(
            one_frame(car_frame(8, 1200, flags=0x0B, calibration_id=9, coordinate_frame=1)),
            10.0,
        )
        self.assertTrue(worker.transmit_locked)
        self.assertIn("READY", worker.transmit_lock_reason)
        self.assertEqual(fake.writes, [])

    def test_maintenance_reset_clears_old_state_once_and_requires_new_manual_path(self) -> None:
        worker, fake, events = self.make_worker()
        worker._process_frame(
            one_frame(car_frame(8, 1200, flags=0x0B, calibration_id=9)), 10.0
        )
        self.assertTrue(worker.transmit_locked)

        worker._process_frame(one_frame(maintenance_reset_frame()), 10.1)
        self.assertFalse(worker.transmit_locked)
        self.assertIsNone(worker.latest_telemetry)
        self.assertIsNone(worker.latest_car_pose)
        reset_events = [event for event in events if isinstance(event, MaintenanceResetEvent)]
        self.assertEqual(len(reset_events), 1)
        self.assertEqual(reset_events[0].reset.reset_id, 1)
        self.assertEqual(fake.writes, [])

        # Only a fresh post-reset snapshot can be submitted for calibration.
        self.prime_uncalibrated(worker)
        worker._queue_calibration(90, 180, 2, 1000, 1000, 10.2)
        pending = worker.pending_calibration
        self.assertIsNotNone(pending)

        # Required retransmissions carry the same ResetId and must not clear a
        # newly, manually submitted calibration or emit a second reset event.
        worker._process_frame(
            one_frame(maintenance_reset_frame(header_flags=FLAG_RETRANSMISSION)), 10.3
        )
        self.assertIs(worker.pending_calibration, pending)
        self.assertEqual(
            len([event for event in events if isinstance(event, MaintenanceResetEvent)]), 1
        )

    def test_invalid_maintenance_reset_never_releases_receive_only_lock(self) -> None:
        worker, _fake, events = self.make_worker()
        worker._process_frame(
            one_frame(car_frame(8, 1200, flags=0x0B, calibration_id=9)), 10.0
        )
        worker._process_frame(one_frame(maintenance_reset_frame(reset_flags=0)), 10.1)
        self.assertTrue(worker.transmit_locked)
        self.assertEqual([event for event in events if isinstance(event, MaintenanceResetEvent)], [])
        self.assertEqual(worker.parser.stats.payload_errors, 1)

    def test_position_change_beyond_confirmed_tolerance_blocks_send(self) -> None:
        worker, fake, events = self.make_worker(send_tolerance=2)
        self.prime_uncalibrated(worker)
        worker._queue_calibration(90, 180, 2, 1000, 1000, 10.02)
        worker._process_frame(one_frame(car_frame(7, 1100, x=20)), 10.10)
        worker._service_calibration(10.135)
        self.assertEqual(fake.writes, [])
        self.assertIsNone(worker.pending_calibration)
        self.assertIn("位置变化", [e.message for e in events if isinstance(e, CalibrationEvent)][-1])

    def test_newer_confirmed_snapshot_within_tolerance_can_queue_calibration(self) -> None:
        worker, _fake, events = self.make_worker(send_tolerance=5)
        self.prime_uncalibrated(worker)

        # A human cannot dismiss the confirmation modal before another 10 Hz
        # CAR_POSE arrives.  Its source time may advance, but the offset remains
        # within the documented confirmation-to-send tolerance.
        worker._process_frame(one_frame(car_frame(2, 1100, x=11, y=19)), 10.10)
        worker._queue_calibration(90, 180, 2, 1000, 1000, 10.11)

        self.assertIsNotNone(worker.pending_calibration)
        self.assertEqual(worker.pending_calibration.state, "waiting_window")
        self.assertNotIn(
            "failure", [event.state for event in events if isinstance(event, CalibrationEvent)]
        )

    def test_newer_confirmed_snapshot_beyond_tolerance_is_rejected(self) -> None:
        worker, _fake, events = self.make_worker(send_tolerance=5)
        self.prime_uncalibrated(worker)
        worker._process_frame(one_frame(car_frame(2, 1100, x=16)), 10.10)
        worker._queue_calibration(90, 180, 2, 1000, 1000, 10.11)

        self.assertIsNone(worker.pending_calibration)
        failure = [event for event in events if isinstance(event, CalibrationEvent)][-1]
        self.assertEqual(failure.state, "failure")
        self.assertIn("位置变化超过发送容差", failure.message)

    def test_ack_timeout_does_not_automatically_retransmit(self) -> None:
        worker, fake, events = self.make_worker()
        self.prime_uncalibrated(worker)
        worker._queue_calibration(90, 180, 2, 1000, 1000, 10.02)
        worker._process_frame(one_frame(car_frame(7, 1100)), 10.10)
        worker._service_calibration(10.135)
        worker._service_calibration(11.136)
        self.assertEqual(len(fake.writes), 1)
        self.assertIsNone(worker.pending_calibration)
        self.assertIn("未自动重发", [e.message for e in events if isinstance(e, CalibrationEvent)][-1])

    def test_open_failure_reconnects_and_split_telemetry_is_delivered(self) -> None:
        raw = telemetry_frame(9, 12345, mode=0)
        fake = FakeSerial([b"noise" + raw[:4], raw[4:17], raw[17:]])
        attempts = 0

        def factory(**_kwargs: object) -> FakeSerial:
            nonlocal attempts
            attempts += 1
            if attempts == 1:
                raise OSError("simulated missing UART")
            return fake

        events: list[object] = []
        received = threading.Event()

        def receive(event: object) -> None:
            events.append(event)
            if isinstance(event, TelemetryEvent):
                received.set()

        worker = SerialWorker(
            SerialSettings(reconnect_delay=0.01),
            receive,
            logging.getLogger("serial-thread-test"),
            serial_factory=factory,
        )
        worker.start()
        self.assertTrue(received.wait(1.0))
        worker.stop()
        self.assertFalse(fake.dtr)
        self.assertFalse(fake.rts)
        self.assertTrue(fake.closed)
        states = [event.state for event in events if isinstance(event, SerialLinkEvent)]
        self.assertIn("offline", states)
        self.assertIn("open", states)
        telemetry = next(event for event in events if isinstance(event, TelemetryEvent))
        self.assertEqual((telemetry.telemetry.x_cm, telemetry.telemetry.y_cm), (100, 200))


if __name__ == "__main__":
    unittest.main()
