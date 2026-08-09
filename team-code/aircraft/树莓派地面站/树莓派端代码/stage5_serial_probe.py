#!/usr/bin/env python3
"""Read-only UART/USB-TTL probe for the D-task V2.3 LoRa protocol."""

from __future__ import annotations

import argparse
from collections import Counter
from dataclasses import asdict
import json
import sys
import time
from typing import Sequence

try:
    import serial
except ImportError as exc:  # pragma: no cover - target dependency check.
    raise SystemExit("缺少 pyserial，请安装 python3-serial") from exc

from ground_station_protocol import (
    MessageType,
    PayloadError,
    SequenceTracker,
    StreamParser,
    V2Frame,
    decode_ack,
    decode_car_pose,
    decode_car_task_request,
    decode_flight_telemetry,
    decode_heartbeat,
    decode_mission_status,
    decode_maintenance_reset,
)
from ground_station_serial import DEFAULT_SERIAL_DEVICE


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--device", default=DEFAULT_SERIAL_DEVICE)
    parser.add_argument("--baudrate", type=int, default=115200)
    parser.add_argument("--duration", type=float, default=10.0)
    parser.add_argument("--preview-bytes", type=int, default=128)
    args = parser.parse_args(argv)
    if args.baudrate <= 0:
        parser.error("--baudrate 必须大于零")
    if args.duration <= 0:
        parser.error("--duration 必须大于零")
    if args.preview_bytes < 0:
        parser.error("--preview-bytes 不能小于零")
    return args


def decoded_payload(frame: V2Frame) -> object | None:
    if frame.message_type == MessageType.FLIGHT_TELEMETRY:
        return decode_flight_telemetry(frame.payload)
    if frame.message_type == MessageType.HEARTBEAT:
        return decode_heartbeat(frame.payload)
    if frame.message_type == MessageType.CAR_POSE:
        return decode_car_pose(frame.payload)
    if frame.message_type == MessageType.CAR_TASK_REQUEST:
        return decode_car_task_request(frame.payload)
    if frame.message_type == MessageType.MISSION_STATUS:
        return decode_mission_status(frame.payload)
    if frame.message_type == MessageType.ACK:
        return decode_ack(frame.payload)
    if frame.message_type == MessageType.MAINTENANCE_RESET:
        return decode_maintenance_reset(frame.payload)
    return None


def run_probe(
    device: str, baudrate: int, duration: float, preview_limit: int
) -> tuple[int, dict[str, object]]:
    parser = StreamParser()
    sequences = SequenceTracker()
    raw_preview = bytearray()
    raw_bytes = 0
    chunks = 0
    types: Counter[str] = Counter()
    payload_errors = 0
    last_messages: dict[str, object] = {}
    started_at = time.monotonic()

    try:
        port = serial.Serial(
            port=device,
            baudrate=baudrate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=0.05,
            write_timeout=0.5,
            xonxoff=False,
            rtscts=False,
            dsrdtr=False,
            exclusive=True,
        )
    except (OSError, serial.SerialException) as exc:
        return 2, {
            "result": "OPEN_FAILED",
            "device": device,
            "baudrate": baudrate,
            "error": str(exc),
        }

    try:
        port.dtr = False
        port.rts = False
        while time.monotonic() - started_at < duration:
            chunk = port.read(256)
            now = time.monotonic()
            if not chunk:
                parser.expire(now)
                continue
            chunks += 1
            raw_bytes += len(chunk)
            remaining = max(0, preview_limit - len(raw_preview))
            raw_preview.extend(chunk[:remaining])
            for frame in parser.feed(chunk, now):
                sequences_lost = sequences.observe(frame)
                parser.stats.estimated_dropped += sequences_lost
                try:
                    type_name = MessageType(frame.message_type).name
                except ValueError:
                    type_name = f"UNKNOWN_0x{frame.message_type:02X}"
                    parser.stats.unknown_messages += 1
                types[type_name] += 1
                try:
                    decoded = decoded_payload(frame)
                except PayloadError:
                    payload_errors += 1
                    parser.stats.payload_errors += 1
                    continue
                if decoded is not None:
                    last_messages[type_name] = asdict(decoded)
    finally:
        port.close()

    valid_frames = parser.stats.valid_frames
    if valid_frames:
        result = "PASS"
        exit_code = 0
    elif raw_bytes:
        result = "RAW_DATA_WITHOUT_VALID_FRAME"
        exit_code = 4
    else:
        result = "NO_DATA"
        exit_code = 5
    report: dict[str, object] = {
        "result": result,
        "device": device,
        "baudrate": baudrate,
        "duration_seconds": round(time.monotonic() - started_at, 3),
        "raw_bytes": raw_bytes,
        "read_chunks": chunks,
        "raw_preview_hex": bytes(raw_preview).hex(" "),
        "frame_types": dict(types),
        "protocol_stats": parser.stats.snapshot(),
        "payload_errors": payload_errors,
        "last_messages": last_messages,
    }
    return exit_code, report


def main(argv: Sequence[str] | None = None) -> int:
    args = parse_args(argv)
    exit_code, report = run_probe(
        args.device, args.baudrate, args.duration, args.preview_bytes
    )
    print(json.dumps(report, ensure_ascii=False, indent=2, sort_keys=True))
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
