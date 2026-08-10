#!/usr/bin/env python3
"""2026 D 题 V2.3 LoRa protocol primitives for the Raspberry Pi ground station.

This module intentionally has no Qt or serial dependency.  The only accepted
wire format is the ``AA 55`` V2.3 frame from the frozen communication
specification; the historical QR-code ``AA BB`` format is not parsed here.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass
from enum import IntEnum
import struct
import time
from typing import Iterable


V2_HEADER = b"\xAA\x55"
V2_VERSION = 0x02
MAX_PAYLOAD = 64

ADDRESS_BROADCAST = 0x10
ADDRESS_AIRCRAFT = 0x20
ADDRESS_CAR = 0x30
ADDRESS_GROUND_STATION = 0x40

FLAG_ACK_REQUIRED = 0x01
FLAG_RETRANSMISSION = 0x02
FLAG_URGENT = 0x04
KNOWN_FLAGS = FLAG_ACK_REQUIRED | FLAG_RETRANSMISSION | FLAG_URGENT

COORDINATE_FIELD_GLOBAL = 0x01
POSE_POSITION_VALID = 0x01
POSE_CALIBRATED = 0x02
POSE_VELOCITY_VALID = 0x04
POSE_YAW_VALID = 0x08
POSE_CAR_RUNNING = 0x10
KNOWN_POSE_FLAGS = (
    POSE_POSITION_VALID
    | POSE_CALIBRATED
    | POSE_VELOCITY_VALID
    | POSE_YAW_VALID
    | POSE_CAR_RUNNING
)


class MessageType(IntEnum):
    FLIGHT_TELEMETRY = 0x02
    HEARTBEAT = 0x03
    ACK = 0x11
    CAR_POSE = 0x80
    CAR_TASK_REQUEST = 0x81
    MISSION_STATUS = 0x82
    CALIBRATION_SET = 0x83
    MISSION_ABORT = 0x84
    MAINTENANCE_RESET = 0x85


class TaskType(IntEnum):
    DROP = 0x01
    DYNAMIC_LAND = 0x02


MODE_TEXT = {
    0: "IDLE / 等待有效校准",
    1: "PRECHECK / 预检",
    2: "TAKEOFF / 起飞",
    3: "INTERCEPT / 拦截",
    4: "FOLLOW / 伴飞",
    5: "DROP_ALIGN / 抛投对准",
    6: "DROP_ACTION / 执行抛投",
    7: "LAND_ALIGN / 着陆对准",
    8: "DESCEND / 下降",
    9: "ON_PLATFORM_5S / 平台停留",
    10: "PLATFORM_TAKEOFF / 平台起飞",
    11: "RETURN_HOME / 返航",
    12: "HOME_LAND / H 点降落",
    13: "ABORT / FAILSAFE",
}

TASK_TEXT = {TaskType.DROP: "抛投", TaskType.DYNAMIC_LAND: "动态起降"}

ACK_RESULT_TEXT = {
    0x00: "接受",
    0x01: "重复但此前已接受",
    0x02: "忙",
    0x03: "当前状态不允许",
    0x04: "不支持",
    0x05: "参数非法",
    0x06: "内部错误",
}

ACK_DETAIL_TEXT = {
    0x00: "无",
    0x01: "CH6 未允许",
    0x02: "定位/车位姿不新鲜",
    0x03: "校准无效",
    0x04: "任务已运行",
    0x05: "飞控 UART 超时",
    0x06: "任务类型非法",
}


@dataclass(frozen=True)
class V2Frame:
    message_type: int
    source: int
    destination: int
    sequence: int
    flags: int
    payload: bytes
    raw: bytes


@dataclass(frozen=True)
class FlightTelemetry:
    status_flags: int
    coordinate_frame: int
    mode_code: int
    x_cm: int
    y_cm: int
    z_cm: int
    yaw_tenths_degree: int
    ground_speed_cm_s: int
    source_time_ms: int


@dataclass(frozen=True)
class Heartbeat:
    device_status: int
    error_code: int
    uptime_ms: int


@dataclass(frozen=True)
class Acknowledgement:
    request_type: int
    request_sequence: int
    result: int
    detail: int


@dataclass(frozen=True)
class CarPose:
    coordinate_frame: int
    pose_flags: int
    calibration_id: int
    x_cm: int
    y_cm: int
    yaw_tenths_degree: int
    vx_cm_s: int
    vy_cm_s: int
    source_time_ms: int

    @property
    def position_valid(self) -> bool:
        return bool(self.pose_flags & POSE_POSITION_VALID)

    @property
    def calibrated(self) -> bool:
        return bool(self.pose_flags & POSE_CALIBRATED)

    @property
    def velocity_valid(self) -> bool:
        return bool(self.pose_flags & POSE_VELOCITY_VALID)

    @property
    def yaw_valid(self) -> bool:
        return bool(self.pose_flags & POSE_YAW_VALID)

    @property
    def car_running(self) -> bool:
        return bool(self.pose_flags & POSE_CAR_RUNNING)


@dataclass(frozen=True)
class CarTaskRequest:
    task_type: int
    request_flags: int
    mission_id: int
    calibration_id: int
    reserved: int
    source_time_ms: int


@dataclass(frozen=True)
class MissionStatus:
    task_type: int
    stage: int
    status_flags: int
    mission_id: int
    error_code: int
    source_time_ms: int


@dataclass(frozen=True)
class MaintenanceReset:
    """A physical car-side maintenance reset, broadcast without an ACK."""

    reset_id: int
    reset_flags: int
    source_time_ms: int

    @property
    def clears_calibration(self) -> bool:
        return bool(self.reset_flags & 0x01)


@dataclass
class ProtocolStats:
    valid_frames: int = 0
    v2_frames: int = 0
    crc_errors: int = 0
    length_errors: int = 0
    version_errors: int = 0
    flag_errors: int = 0
    address_errors: int = 0
    timeout_errors: int = 0
    unknown_messages: int = 0
    payload_errors: int = 0
    stale_source_frames: int = 0
    source_resets: int = 0
    estimated_dropped: int = 0
    noise_bytes: int = 0

    def snapshot(self) -> dict[str, int]:
        return asdict(self)


class PayloadError(ValueError):
    """The CRC-valid frame payload does not match the V2.3 message schema."""


def crc16_ccitt_false(data: bytes | bytearray | memoryview) -> int:
    crc = 0xFFFF
    for value in data:
        crc ^= value << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


def encode_v2(
    message_type: int,
    source: int,
    destination: int,
    sequence: int,
    flags: int,
    payload: bytes = b"",
) -> bytes:
    payload = bytes(payload)
    header_values = (message_type, source, destination, sequence, flags)
    if any(not 0 <= value <= 0xFF for value in header_values):
        raise ValueError("V2.3 header fields must fit uint8")
    if source == ADDRESS_BROADCAST:
        raise ValueError("broadcast cannot be used as a source")
    if flags & ~KNOWN_FLAGS:
        raise ValueError("reserved flag bits must be zero")
    if destination == ADDRESS_BROADCAST and flags & FLAG_ACK_REQUIRED:
        raise ValueError("broadcast frames cannot request ACK")
    if len(payload) > MAX_PAYLOAD:
        raise ValueError("V2.3 payload exceeds 64 bytes")
    body = bytes(
        (
            V2_VERSION,
            message_type,
            source,
            destination,
            sequence,
            flags,
            len(payload),
        )
    ) + payload
    return V2_HEADER + body + struct.pack(">H", crc16_ccitt_false(body))


def encode_calibration_set(
    delta_x_cm: int, delta_y_cm: int, calibration_id: int, sequence: int
) -> bytes:
    if not -0x80000000 <= delta_x_cm <= 0x7FFFFFFF:
        raise ValueError("DeltaX must fit int32")
    if not -0x80000000 <= delta_y_cm <= 0x7FFFFFFF:
        raise ValueError("DeltaY must fit int32")
    if not 1 <= calibration_id <= 0xFFFF:
        raise ValueError("CalibrationId must be a non-zero uint16")
    payload = struct.pack(">iiHBB", delta_x_cm, delta_y_cm, calibration_id, 0x01, 0)
    return encode_v2(
        MessageType.CALIBRATION_SET,
        ADDRESS_GROUND_STATION,
        ADDRESS_CAR,
        sequence,
        FLAG_ACK_REQUIRED,
        payload,
    )


def _require_length(payload: bytes, expected: int, name: str) -> None:
    if len(payload) != expected:
        raise PayloadError(f"{name} payload must be {expected} bytes, got {len(payload)}")


def decode_flight_telemetry(payload: bytes) -> FlightTelemetry:
    _require_length(payload, 24, "FLIGHT_TELEMETRY")
    telemetry = FlightTelemetry(*struct.unpack(">HBBiiihHI", payload))
    if telemetry.coordinate_frame != COORDINATE_FIELD_GLOBAL:
        raise PayloadError("FLIGHT_TELEMETRY CoordinateFrame must be FIELD_GLOBAL")
    if telemetry.mode_code not in MODE_TEXT:
        raise PayloadError("FLIGHT_TELEMETRY ModeCode is invalid")
    return telemetry


def decode_heartbeat(payload: bytes) -> Heartbeat:
    _require_length(payload, 8, "HEARTBEAT")
    device_status, error_code, reserved, uptime_ms = struct.unpack(">BBHI", payload)
    if reserved != 0:
        raise PayloadError("HEARTBEAT Reserved must be zero")
    return Heartbeat(device_status, error_code, uptime_ms)


def decode_ack(payload: bytes) -> Acknowledgement:
    _require_length(payload, 4, "ACK")
    acknowledgement = Acknowledgement(*struct.unpack(">BBBB", payload))
    if acknowledgement.result not in ACK_RESULT_TEXT:
        raise PayloadError("ACK Result is invalid")
    if acknowledgement.detail not in ACK_DETAIL_TEXT:
        raise PayloadError("ACK Detail is invalid")
    return acknowledgement


def decode_car_pose(payload: bytes) -> CarPose:
    _require_length(payload, 22, "CAR_POSE")
    pose = CarPose(*struct.unpack(">BBHiihhhI", payload))
    if pose.coordinate_frame != COORDINATE_FIELD_GLOBAL:
        raise PayloadError("CAR_POSE CoordinateFrame must be FIELD_GLOBAL")
    if pose.pose_flags & ~KNOWN_POSE_FLAGS:
        raise PayloadError("CAR_POSE reserved PoseFlags must be zero")
    if pose.calibrated and pose.calibration_id == 0:
        raise PayloadError("calibrated CAR_POSE must carry a non-zero CalibrationId")
    if not pose.velocity_valid and (pose.vx_cm_s, pose.vy_cm_s) != (0x7FFF, 0x7FFF):
        raise PayloadError("velocity-invalid CAR_POSE must use 0x7FFF for Vx/Vy")
    return pose


def decode_car_task_request(payload: bytes) -> CarTaskRequest:
    _require_length(payload, 12, "CAR_TASK_REQUEST")
    request = CarTaskRequest(*struct.unpack(">BBHHHI", payload))
    if request.task_type not in (TaskType.DROP, TaskType.DYNAMIC_LAND):
        raise PayloadError("CAR_TASK_REQUEST TaskType is invalid")
    if request.request_flags != 0x01:
        raise PayloadError("CAR_TASK_REQUEST RequestFlags must contain START only")
    if request.reserved != 0:
        raise PayloadError("CAR_TASK_REQUEST Reserved must be zero")
    if request.calibration_id == 0:
        raise PayloadError("CAR_TASK_REQUEST CalibrationId must be non-zero")
    return request


def decode_mission_status(payload: bytes) -> MissionStatus:
    _require_length(payload, 12, "MISSION_STATUS")
    task_type, stage, status_flags, mission_id, error_code, reserved, source_time_ms = struct.unpack(
        ">BBHHBBI", payload
    )
    if task_type not in (TaskType.DROP, TaskType.DYNAMIC_LAND):
        raise PayloadError("MISSION_STATUS TaskType is invalid")
    if stage not in MODE_TEXT:
        raise PayloadError("MISSION_STATUS Stage is invalid")
    if reserved != 0:
        raise PayloadError("MISSION_STATUS Reserved must be zero")
    return MissionStatus(task_type, stage, status_flags, mission_id, error_code, source_time_ms)


def decode_maintenance_reset(payload: bytes) -> MaintenanceReset:
    """Decode the Appendix C physical-maintenance reset payload.

    A zero ID, unknown flag bits, a missing CLEAR_CALIBRATION bit, or a
    non-zero reserved byte must never release the receive-only TX latch.
    """

    _require_length(payload, 8, "MAINTENANCE_RESET")
    reset_id, reset_flags, reserved, source_time_ms = struct.unpack(">HBBI", payload)
    if reset_id == 0:
        raise PayloadError("MAINTENANCE_RESET ResetId must be non-zero")
    if reset_flags != 0x01:
        raise PayloadError("MAINTENANCE_RESET ResetFlags must be CLEAR_CALIBRATION only")
    if reserved != 0:
        raise PayloadError("MAINTENANCE_RESET Reserved must be zero")
    return MaintenanceReset(reset_id, reset_flags, source_time_ms)


def source_time_relation(new_value: int, previous_value: int | None) -> str:
    """Classify a uint32 device-monotonic timestamp without comparing clocks.

    A natural uint32 wrap is newer.  A large backwards jump is treated as a
    device reboot so callers can invalidate old samples explicitly.
    """

    if previous_value is None:
        return "new"
    delta = (new_value - previous_value) & 0xFFFFFFFF
    if delta == 0:
        return "duplicate"
    if delta < 0x80000000:
        return "new"
    # A clearly restarted uptime counter is accepted as a new epoch.  Smaller
    # backwards moves are ordinary out-of-order frames and must not overwrite
    # a newer display/control-maintenance sample.
    if new_value <= 10_000 and previous_value >= 60_000:
        return "reset"
    return "stale"


class StreamParser:
    """Incrementally parse V2.3 frames from arbitrary UART byte chunks."""

    def __init__(
        self,
        *,
        accepted_destinations: Iterable[int] = (
            ADDRESS_GROUND_STATION,
            ADDRESS_BROADCAST,
            # Passive observation of the request addressed to the aircraft is
            # required so the ground station can permanently disable TX.
            ADDRESS_AIRCRAFT,
        ),
        inter_byte_timeout: float = 0.1,
    ):
        if inter_byte_timeout <= 0:
            raise ValueError("inter_byte_timeout must be greater than zero")
        self.accepted_destinations = frozenset(accepted_destinations)
        self.inter_byte_timeout = inter_byte_timeout
        self.buffer = bytearray()
        self.last_byte_at: float | None = None
        self.stats = ProtocolStats()

    def reset_incomplete(self) -> None:
        self.buffer.clear()
        self.last_byte_at = None

    def expire(self, now: float | None = None) -> bool:
        now = time.monotonic() if now is None else now
        if (
            self.buffer
            and self.last_byte_at is not None
            and now - self.last_byte_at > self.inter_byte_timeout
        ):
            self.stats.timeout_errors += 1
            self.stats.noise_bytes += len(self.buffer)
            self.reset_incomplete()
            return True
        return False

    def feed(
        self, data: bytes | bytearray | memoryview, now: float | None = None
    ) -> list[V2Frame]:
        now = time.monotonic() if now is None else now
        self.expire(now)
        if not data:
            return []
        self.buffer.extend(data)
        self.last_byte_at = now
        frames: list[V2Frame] = []

        while self.buffer:
            start = self.buffer.find(V2_HEADER)
            if start < 0:
                # Keep a trailing AA because it may be the first header byte.
                keep = 1 if self.buffer[-1] == V2_HEADER[0] else 0
                discard = len(self.buffer) - keep
                self.stats.noise_bytes += discard
                if discard:
                    del self.buffer[:discard]
                break
            if start:
                self.stats.noise_bytes += start
                del self.buffer[:start]
            frame = self._take_v2()
            if frame is False:
                continue
            if frame is None:
                break
            frames.append(frame)
        return frames

    def _take_v2(self) -> V2Frame | None | bool:
        if len(self.buffer) < 9:
            return None
        if self.buffer[2] != V2_VERSION:
            self.stats.version_errors += 1
            del self.buffer[0]
            return False
        payload_length = self.buffer[8]
        if payload_length > MAX_PAYLOAD:
            self.stats.length_errors += 1
            del self.buffer[0]
            return False
        total_length = payload_length + 11
        if len(self.buffer) < total_length:
            return None
        candidate = bytes(self.buffer[:total_length])
        expected_crc = struct.unpack(">H", candidate[-2:])[0]
        actual_crc = crc16_ccitt_false(candidate[2:-2])
        if actual_crc != expected_crc:
            self.stats.crc_errors += 1
            del self.buffer[0]
            return False
        del self.buffer[:total_length]

        source = candidate[4]
        destination = candidate[5]
        flags = candidate[7]
        if flags & ~KNOWN_FLAGS:
            self.stats.flag_errors += 1
            return False
        if (
            source == ADDRESS_BROADCAST
            or destination not in self.accepted_destinations
            or (destination == ADDRESS_BROADCAST and flags & FLAG_ACK_REQUIRED)
        ):
            self.stats.address_errors += 1
            return False
        self.stats.valid_frames += 1
        self.stats.v2_frames += 1
        return V2Frame(
            message_type=candidate[3],
            source=source,
            destination=destination,
            sequence=candidate[6],
            flags=flags,
            payload=candidate[9:-2],
            raw=candidate,
        )


class SequenceTracker:
    """Estimate gaps separately by source and message type."""

    def __init__(self):
        self._last: dict[tuple[int, int], int] = {}

    def observe(self, frame: V2Frame) -> int:
        key = (frame.source, frame.message_type)
        previous = self._last.get(key)
        if previous is None:
            self._last[key] = frame.sequence
            return 0
        delta = (frame.sequence - previous) & 0xFF
        if delta == 0 or delta > 127:
            return 0
        self._last[key] = frame.sequence
        return max(0, delta - 1)
