#!/usr/bin/env python3
"""Reconnecting receive-first USB-TTL/LoRa worker for the V2.3 ground station.

The sole transmit path is a one-shot CALIBRATION_SET in the section 7.1
maintenance slot.  A valid READY indication, CAR_TASK_REQUEST, mission status,
or running state irreversibly latches transmission off for this process.
"""

from __future__ import annotations

from dataclasses import dataclass
import logging
import queue
import threading
import time
from typing import Callable, Protocol

try:
    import serial
except ImportError:  # pragma: no cover - reported as a link event on target.
    serial = None  # type: ignore[assignment]

from ground_station_protocol import (
    ACK_DETAIL_TEXT,
    ACK_RESULT_TEXT,
    ADDRESS_AIRCRAFT,
    ADDRESS_BROADCAST,
    ADDRESS_CAR,
    ADDRESS_GROUND_STATION,
    COORDINATE_FIELD_GLOBAL,
    FLAG_ACK_REQUIRED,
    FLAG_RETRANSMISSION,
    FLAG_URGENT,
    Acknowledgement,
    CarPose,
    CarTaskRequest,
    FlightTelemetry,
    Heartbeat,
    MaintenanceReset,
    MessageType,
    MissionStatus,
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
    encode_calibration_set,
    source_time_relation,
)


DEFAULT_SERIAL_DEVICE = "/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0"


class SerialPort(Protocol):
    def read(self, size: int = 1) -> bytes: ...

    def write(self, data: bytes) -> int: ...

    def close(self) -> None: ...


@dataclass(frozen=True)
class SerialSettings:
    device: str = DEFAULT_SERIAL_DEVICE
    baudrate: int = 115200
    reconnect_delay: float = 2.0
    # Five milliseconds keeps the worker schedulable inside the 30--50 ms TX
    # window while retaining ordinary blocking serial reads.
    read_timeout: float = 0.005
    write_timeout: float = 0.05
    flight_freshness: float = 1.2
    car_freshness: float = 0.5
    calibration_ack_timeout: float = 1.0
    calibrated_pose_timeout: float = 2.0
    calibration_send_tolerance_cm: int = 5
    initial_sequence: int = 0

    def __post_init__(self) -> None:
        if not self.device:
            raise ValueError("serial device cannot be empty")
        if self.baudrate <= 0:
            raise ValueError("baudrate must be greater than zero")
        if min(
            self.reconnect_delay,
            self.read_timeout,
            self.write_timeout,
            self.flight_freshness,
            self.car_freshness,
            self.calibration_ack_timeout,
            self.calibrated_pose_timeout,
        ) <= 0:
            raise ValueError("serial timeouts must be greater than zero")
        if not 0 <= self.calibration_send_tolerance_cm <= 1000:
            raise ValueError("calibration_send_tolerance_cm is out of range")
        if not 0 <= self.initial_sequence <= 0xFF:
            raise ValueError("initial_sequence must fit uint8")


@dataclass(frozen=True)
class SerialLinkEvent:
    state: str
    device: str
    message: str
    occurred_at: float


@dataclass(frozen=True)
class TelemetryEvent:
    telemetry: FlightTelemetry
    sequence: int
    received_at: float


@dataclass(frozen=True)
class HeartbeatEvent:
    heartbeat: Heartbeat
    sequence: int
    received_at: float


@dataclass(frozen=True)
class CarPoseEvent:
    pose: CarPose
    sequence: int
    received_at: float


@dataclass(frozen=True)
class TaskRequestEvent:
    request: CarTaskRequest
    sequence: int
    received_at: float


@dataclass(frozen=True)
class MissionStatusEvent:
    status: MissionStatus
    sequence: int
    received_at: float


@dataclass(frozen=True)
class ProtocolStatsEvent:
    stats: dict[str, int]
    last_valid_at: float | None
    last_flight_at: float | None
    last_car_at: float | None


@dataclass(frozen=True)
class CalibrationEvent:
    state: str
    message: str
    calibration_id: int | None = None
    sequence: int | None = None
    delta_x_cm: int | None = None
    delta_y_cm: int | None = None


@dataclass(frozen=True)
class TransmitLockEvent:
    reason: str
    occurred_at: float


@dataclass(frozen=True)
class MaintenanceResetEvent:
    """A newly observed physical maintenance reset from the car."""

    reset: MaintenanceReset
    sequence: int
    received_at: float


@dataclass
class _PendingCalibration:
    delta_x_cm: int
    delta_y_cm: int
    calibration_id: int
    flight_source_time_ms: int
    car_source_time_ms: int
    state: str = "waiting_window"
    sequence: int | None = None
    slot_received_at: float | None = None
    send_due_at: float | None = None
    send_deadline_at: float | None = None
    response_deadline_at: float | None = None


class SerialWorker:
    """Run blocking serial I/O, parsing and the gated calibration TX path."""

    def __init__(
        self,
        settings: SerialSettings,
        event_callback: Callable[[object], None],
        logger: logging.Logger,
        *,
        serial_factory: Callable[..., SerialPort] | None = None,
    ):
        self.settings = settings
        self.event_callback = event_callback
        self.logger = logger
        self.serial_factory = serial_factory or self._default_serial_factory
        self.parser = StreamParser()
        self.sequences = SequenceTracker()
        self.actions: queue.Queue[tuple[str, tuple[object, ...]]] = queue.Queue()
        self.stop_event = threading.Event()
        self.thread: threading.Thread | None = None
        self.port: SerialPort | None = None

        self.last_valid_at: float | None = None
        self.last_flight_at: float | None = None
        self.last_car_at: float | None = None
        self.last_stats_logged_at = 0.0
        self.last_flight_source_ms: int | None = None
        self.last_car_source_ms: int | None = None
        self.latest_telemetry: FlightTelemetry | None = None
        self.latest_car_pose: CarPose | None = None

        self.next_sequence = settings.initial_sequence
        self.pending_calibration: _PendingCalibration | None = None
        self.transmit_locked = False
        self.transmit_lock_reason = ""
        # Appendix C deduplicates on Src + Type + ResetId.  The fixed source
        # and type are implicit here, so retaining IDs is sufficient.
        self.seen_maintenance_reset_ids: set[int] = set()

    @staticmethod
    def _default_serial_factory(**kwargs: object) -> SerialPort:
        if serial is None:
            raise RuntimeError("缺少 pyserial，请安装 python3-serial")
        return serial.Serial(**kwargs)

    def start(self) -> None:
        if self.thread is not None and self.thread.is_alive():
            return
        self.stop_event.clear()
        self.thread = threading.Thread(
            target=self._run, name="ground-station-uart-rx", daemon=True
        )
        self.thread.start()

    def stop(self, timeout: float = 2.0) -> None:
        self.stop_event.set()
        thread = self.thread
        if thread is not None and thread is not threading.current_thread():
            thread.join(timeout)
        self.thread = None

    def submit_calibration(
        self,
        delta_x_cm: int,
        delta_y_cm: int,
        calibration_id: int,
        flight_source_time_ms: int,
        car_source_time_ms: int,
    ) -> None:
        self.actions.put(
            (
                "calibration",
                (
                    int(delta_x_cm),
                    int(delta_y_cm),
                    int(calibration_id),
                    int(flight_source_time_ms),
                    int(car_source_time_ms),
                ),
            )
        )

    def disable_transmission(self, reason: str) -> None:
        """Request an irreversible TX latch from another thread."""

        self.actions.put(("lock", (str(reason),)))

    def _emit(self, event: object) -> None:
        try:
            self.event_callback(event)
        except Exception:
            self.logger.exception("serial event callback failed")

    def _run(self) -> None:
        self.logger.info(
            "V2.3 UART worker starting device=%s baud=%d receive_first=true",
            self.settings.device,
            self.settings.baudrate,
        )
        while not self.stop_event.is_set():
            self._emit(
                SerialLinkEvent(
                    "opening", self.settings.device, "正在打开 LoRa USB-TTL", time.monotonic()
                )
            )
            try:
                port = self.serial_factory(
                    port=self.settings.device,
                    baudrate=self.settings.baudrate,
                    bytesize=8,
                    parity="N",
                    stopbits=1,
                    timeout=self.settings.read_timeout,
                    write_timeout=self.settings.write_timeout,
                    xonxoff=False,
                    rtscts=False,
                    dsrdtr=False,
                    exclusive=True,
                )
                self.port = port
                # Transparent LoRa UARTs do not use modem-control lines.  Keep
                # them inactive so opening CH340 cannot reset attached nodes.
                if hasattr(port, "dtr"):
                    port.dtr = False  # type: ignore[attr-defined]
                if hasattr(port, "rts"):
                    port.rts = False  # type: ignore[attr-defined]
                self.parser.reset_incomplete()
                self.sequences = SequenceTracker()
                self._emit(
                    SerialLinkEvent(
                        "open",
                        self.settings.device,
                        "串口已打开，V2.2 接收中",
                        time.monotonic(),
                    )
                )
                self.logger.info("UART opened device=%s", self.settings.device)
                self._run_connected(port)
            except Exception as exc:
                if not self.stop_event.is_set():
                    self.logger.warning("UART disconnected: %s", exc)
                    self._emit(
                        SerialLinkEvent(
                            "offline", self.settings.device, str(exc), time.monotonic()
                        )
                    )
            finally:
                if self.pending_calibration is not None:
                    self._fail_calibration("串口连接中断，校准未完成")
                self.parser.reset_incomplete()
                port = self.port
                self.port = None
                if port is not None:
                    try:
                        port.close()
                    except Exception:
                        self.logger.debug("failed to close UART", exc_info=True)
            if not self.stop_event.is_set():
                self.stop_event.wait(self.settings.reconnect_delay)
        self.logger.info("V2.3 UART worker stopped")

    def _run_connected(self, port: SerialPort) -> None:
        while not self.stop_event.is_set():
            now = time.monotonic()
            self._process_actions(now)
            self._service_calibration(now)

            chunk = port.read(256)
            now = time.monotonic()
            if chunk:
                frames = self.parser.feed(chunk, now)
                for frame in frames:
                    if self.last_valid_at is not None and now - self.last_valid_at > 1.0:
                        self.logger.warning(
                            "UART valid-frame receive gap duration=%.2fs",
                            now - self.last_valid_at,
                        )
                    self.last_valid_at = now
                    self._process_frame(frame, now)
                self._emit_stats(now)
            elif self.parser.expire(now):
                self._emit_stats(now)
            self._service_calibration(time.monotonic())

    def _process_actions(self, now: float) -> None:
        while True:
            try:
                action, values = self.actions.get_nowait()
            except queue.Empty:
                return
            if action == "lock":
                self._lock_transmission(str(values[0]), now)
                continue
            if action != "calibration":
                self.logger.warning("discarding unknown UART action=%s", action)
                continue
            dx, dy, calibration_id, flight_ms, car_ms = map(int, values)
            self._queue_calibration(dx, dy, calibration_id, flight_ms, car_ms, now)

    def _queue_calibration(
        self,
        delta_x_cm: int,
        delta_y_cm: int,
        calibration_id: int,
        flight_source_time_ms: int,
        car_source_time_ms: int,
        now: float,
    ) -> None:
        if self.transmit_locked:
            self._emit(
                CalibrationEvent(
                    "failure", f"发送已永久禁用：{self.transmit_lock_reason}", calibration_id
                )
            )
            return
        if self.pending_calibration is not None:
            self._emit(CalibrationEvent("failure", "已有校准请求等待处理", calibration_id))
            return
        reason = self._maintenance_precondition_failure(now)
        if reason:
            self._emit(CalibrationEvent("failure", reason, calibration_id))
            return
        assert self.latest_telemetry is not None
        assert self.latest_car_pose is not None
        current_dx = self.latest_telemetry.x_cm - self.latest_car_pose.x_cm
        current_dy = self.latest_telemetry.y_cm - self.latest_car_pose.y_cm
        tolerance = self.settings.calibration_send_tolerance_cm
        # The modal manual confirmation naturally takes longer than one CAR_POSE
        # period.  Source timestamps therefore cannot be required to remain
        # byte-for-byte identical.  Keep the safety guarantee in physical units:
        # accept newer, still-fresh snapshots only when their requested offset is
        # within the same bounded movement tolerance enforced again at transmit.
        if (
            abs(current_dx - delta_x_cm) > tolerance
            or abs(current_dy - delta_y_cm) > tolerance
        ):
            self._emit(
                CalibrationEvent(
                    "failure",
                    "确认后位置变化超过发送容差，请重新对准并确认",
                    calibration_id,
                )
            )
            return
        try:
            # Validate widths and CalibrationId before waiting for a slot.
            encode_calibration_set(delta_x_cm, delta_y_cm, calibration_id, 0)
        except ValueError as exc:
            self._emit(CalibrationEvent("failure", str(exc), calibration_id))
            return
        self.pending_calibration = _PendingCalibration(
            delta_x_cm,
            delta_y_cm,
            calibration_id,
            flight_source_time_ms,
            car_source_time_ms,
        )
        self._emit(
            CalibrationEvent(
                "waiting_window",
                "已人工确认，等待下一帧 CAR_POSE Seq mod 5 = 2",
                calibration_id,
                delta_x_cm=delta_x_cm,
                delta_y_cm=delta_y_cm,
            )
        )

    def _maintenance_precondition_failure(self, now: float) -> str | None:
        if self.port is None:
            return "LoRa 串口未打开"
        if self.latest_telemetry is None or self.last_flight_at is None:
            return "尚未收到 FLIGHT_TELEMETRY"
        if now - self.last_flight_at > self.settings.flight_freshness:
            return "FLIGHT_TELEMETRY 已陈旧"
        if self.latest_telemetry.coordinate_frame != COORDINATE_FIELD_GLOBAL:
            return "飞机坐标系不是 FIELD_GLOBAL"
        if self.latest_telemetry.mode_code != 0:
            return "飞机已进入任务阶段"
        if self.latest_car_pose is None or self.last_car_at is None:
            return "尚未收到 CAR_POSE"
        if now - self.last_car_at > self.settings.car_freshness:
            return "CAR_POSE 已失效"
        pose = self.latest_car_pose
        if not pose.position_valid:
            return "CAR_POSE 位置无效"
        if pose.calibrated:
            return "小车已校准（READY）"
        if pose.calibration_id != 0:
            return "未校准 CAR_POSE 携带非零 CalibrationId"
        if pose.car_running:
            return "小车正在运行"
        return None

    def _arm_calibration_slot(self, frame: V2Frame, now: float) -> None:
        pending = self.pending_calibration
        if pending is None or pending.state != "waiting_window":
            return
        if frame.sequence % 5 != 2:
            return
        if self._maintenance_precondition_failure(now):
            return
        pending.slot_received_at = now
        pending.send_due_at = now + 0.035
        pending.send_deadline_at = now + 0.050
        pending.state = "window_armed"
        self._emit(
            CalibrationEvent(
                "window_armed",
                "已捕获维护槽，计划在接收完成后 35 ms 发送",
                pending.calibration_id,
                delta_x_cm=pending.delta_x_cm,
                delta_y_cm=pending.delta_y_cm,
            )
        )

    def _service_calibration(self, now: float) -> None:
        pending = self.pending_calibration
        if pending is None:
            return
        if self.transmit_locked:
            self._fail_calibration(f"发送已永久禁用：{self.transmit_lock_reason}")
            return
        if pending.state == "window_armed":
            assert pending.send_due_at is not None
            assert pending.send_deadline_at is not None
            if now < pending.send_due_at:
                return
            if now > pending.send_deadline_at:
                pending.state = "waiting_window"
                pending.slot_received_at = None
                pending.send_due_at = None
                pending.send_deadline_at = None
                self._emit(
                    CalibrationEvent(
                        "waiting_window",
                        "本轮 30–50 ms 窗口已错过，未发送；等待下一维护槽",
                        pending.calibration_id,
                    )
                )
                return
            reason = self._maintenance_precondition_failure(now)
            if reason:
                self._fail_calibration(reason)
                return
            assert self.latest_telemetry is not None
            assert self.latest_car_pose is not None
            current_dx = self.latest_telemetry.x_cm - self.latest_car_pose.x_cm
            current_dy = self.latest_telemetry.y_cm - self.latest_car_pose.y_cm
            tolerance = self.settings.calibration_send_tolerance_cm
            if (
                abs(current_dx - pending.delta_x_cm) > tolerance
                or abs(current_dy - pending.delta_y_cm) > tolerance
            ):
                self._fail_calibration("确认后位置变化超过发送容差，请重新对准并确认")
                return
            sequence = self.next_sequence
            self.next_sequence = (sequence + 1) & 0xFF
            frame = encode_calibration_set(
                pending.delta_x_cm,
                pending.delta_y_cm,
                pending.calibration_id,
                sequence,
            )
            self._write_calibration_frame(frame, now)
            pending.sequence = sequence
            pending.state = "waiting_ack"
            pending.response_deadline_at = now + self.settings.calibration_ack_timeout
            self._emit(
                CalibrationEvent(
                    "waiting_ack",
                    "CALIBRATION_SET 已发送，等待小车 ACK",
                    pending.calibration_id,
                    sequence,
                    pending.delta_x_cm,
                    pending.delta_y_cm,
                )
            )
            return
        if pending.state in {"waiting_ack", "waiting_calibrated_pose"}:
            assert pending.response_deadline_at is not None
            if now >= pending.response_deadline_at:
                if pending.state == "waiting_ack":
                    self._fail_calibration("等待小车 ACK 超时；未自动重发")
                else:
                    self._fail_calibration("ACK 已接受，但等待 CALIBRATED CAR_POSE 超时")

    def _write_calibration_frame(self, frame: bytes, now: float) -> None:
        pending = self.pending_calibration
        if self.transmit_locked or pending is None or pending.state != "window_armed":
            raise RuntimeError("calibration transmit path is disabled")
        assert pending.slot_received_at is not None
        elapsed = now - pending.slot_received_at
        if not 0.030 <= elapsed <= 0.050:
            raise RuntimeError(f"CALIBRATION_SET outside maintenance window: {elapsed:.6f}s")
        if self.port is None:
            raise OSError("串口未打开")
        written = self.port.write(frame)
        if written != len(frame):
            raise OSError(f"串口短写：{written}/{len(frame)} 字节")
        self.logger.info(
            "UART tx CALIBRATION_SET elapsed_ms=%.2f bytes=%d frame=%s",
            elapsed * 1000.0,
            len(frame),
            frame.hex(" "),
        )

    def _handle_calibration_ack(self, ack: Acknowledgement, frame: V2Frame, now: float) -> None:
        pending = self.pending_calibration
        if (
            pending is None
            or pending.state != "waiting_ack"
            or pending.sequence is None
            or ack.request_type != MessageType.CALIBRATION_SET
            or ack.request_sequence != pending.sequence
            or frame.source != ADDRESS_CAR
            or frame.destination != ADDRESS_GROUND_STATION
        ):
            return
        result_text = ACK_RESULT_TEXT.get(ack.result, f"未知结果 0x{ack.result:02X}")
        detail_text = ACK_DETAIL_TEXT.get(ack.detail, f"未知详情 0x{ack.detail:02X}")
        if ack.result not in (0x00, 0x01):
            self._fail_calibration(f"小车拒绝校准：{result_text}；{detail_text}")
            return
        pending.state = "waiting_calibrated_pose"
        pending.response_deadline_at = now + self.settings.calibrated_pose_timeout
        self._emit(
            CalibrationEvent(
                "waiting_calibrated_pose",
                f"ACK：{result_text}；等待同 ID 的 CALIBRATED CAR_POSE",
                pending.calibration_id,
                pending.sequence,
                pending.delta_x_cm,
                pending.delta_y_cm,
            )
        )

    def _fail_calibration(self, message: str) -> None:
        pending = self.pending_calibration
        if pending is None:
            return
        self.pending_calibration = None
        self._emit(
            CalibrationEvent(
                "failure",
                message,
                pending.calibration_id,
                pending.sequence,
                pending.delta_x_cm,
                pending.delta_y_cm,
            )
        )

    def _lock_transmission(self, reason: str, now: float) -> None:
        if self.transmit_locked:
            return
        self.transmit_locked = True
        self.transmit_lock_reason = reason
        if self.pending_calibration is not None:
            self._fail_calibration(f"发送锁定，取消未完成校准：{reason}")
        while True:
            try:
                self.actions.get_nowait()
            except queue.Empty:
                break
        self.logger.warning("ground-station transmission permanently locked: %s", reason)
        self._emit(TransmitLockEvent(reason, now))

    def _enter_maintenance_reset(self, reset: MaintenanceReset, frame: V2Frame, now: float) -> None:
        """Apply one valid, previously unseen Appendix C reset event.

        Only this narrow transition may clear the process-local receive-only
        latch.  Old telemetry and CAR_POSE snapshots are discarded too, so a
        later calibration can be based only on data received after the reset.
        """

        if self.pending_calibration is not None:
            self._fail_calibration("maintenance reset cancelled the pending CALIBRATION_SET")
        while True:
            try:
                self.actions.get_nowait()
            except queue.Empty:
                break

        self.latest_telemetry = None
        self.latest_car_pose = None
        self.last_flight_at = None
        self.last_car_at = None
        self.last_flight_source_ms = None
        self.last_car_source_ms = None
        self.transmit_locked = False
        self.transmit_lock_reason = "awaiting fresh IDLE and uncalibrated CAR_POSE after reset"
        self.logger.warning(
            "accepted MAINTENANCE_RESET id=%d seq=%d; awaiting new manual calibration",
            reset.reset_id,
            frame.sequence,
        )
        self._emit(MaintenanceResetEvent(reset, frame.sequence, now))

    def _process_frame(self, frame: V2Frame, now: float) -> None:
        self.parser.stats.estimated_dropped += self.sequences.observe(frame)
        try:
            if frame.message_type == MessageType.FLIGHT_TELEMETRY:
                if frame.source != ADDRESS_AIRCRAFT or frame.destination != ADDRESS_BROADCAST:
                    return
                telemetry = decode_flight_telemetry(frame.payload)
                relation = source_time_relation(
                    telemetry.source_time_ms, self.last_flight_source_ms
                )
                if relation in {"duplicate", "stale"}:
                    self.parser.stats.stale_source_frames += 1
                    return
                if relation == "reset":
                    self.parser.stats.source_resets += 1
                self.last_flight_source_ms = telemetry.source_time_ms
                self.latest_telemetry = telemetry
                self.last_flight_at = now
                if telemetry.mode_code != 0:
                    self._lock_transmission(
                        f"飞机进入 {telemetry.mode_code} 号任务阶段", now
                    )
                self._emit(TelemetryEvent(telemetry, frame.sequence, now))
                return

            if frame.message_type == MessageType.HEARTBEAT:
                if frame.destination != ADDRESS_BROADCAST:
                    return
                heartbeat = decode_heartbeat(frame.payload)
                self._emit(HeartbeatEvent(heartbeat, frame.sequence, now))
                return

            if frame.message_type == MessageType.CAR_POSE:
                if frame.source != ADDRESS_CAR or frame.destination != ADDRESS_BROADCAST:
                    return
                pose = decode_car_pose(frame.payload)
                relation = source_time_relation(pose.source_time_ms, self.last_car_source_ms)
                if relation in {"duplicate", "stale"}:
                    self.parser.stats.stale_source_frames += 1
                    return
                if relation == "reset":
                    self.parser.stats.source_resets += 1
                self.last_car_source_ms = pose.source_time_ms
                self.latest_car_pose = pose
                self.last_car_at = now
                self._emit(CarPoseEvent(pose, frame.sequence, now))

                pending = self.pending_calibration
                if (
                    pose.calibrated
                    and pose.calibration_id != 0
                    and pending is not None
                    and pending.state == "waiting_calibrated_pose"
                    and pose.calibration_id == pending.calibration_id
                    and pose.coordinate_frame == COORDINATE_FIELD_GLOBAL
                ):
                    self._emit(
                        CalibrationEvent(
                            "success",
                            "校准闭环完成：ACK 与后续 CALIBRATED CAR_POSE 均已确认",
                            pending.calibration_id,
                            pending.sequence,
                            pending.delta_x_cm,
                            pending.delta_y_cm,
                        )
                    )
                    self.pending_calibration = None
                if pose.calibrated and pose.calibration_id != 0:
                    self._lock_transmission("小车已校准，地面站进入 READY 只收模式", now)
                elif pose.car_running:
                    self._lock_transmission("CAR_POSE 显示小车已运行", now)
                else:
                    self._arm_calibration_slot(frame, now)
                return

            if frame.message_type == MessageType.CAR_TASK_REQUEST:
                if (
                    frame.source != ADDRESS_CAR
                    or frame.destination != ADDRESS_AIRCRAFT
                    or not frame.flags & FLAG_ACK_REQUIRED
                ):
                    return
                # The correctly addressed request envelope is enough to end
                # maintenance.  Payload errors may suppress its display, but
                # can never leave the ground-station transmitter enabled.
                self._lock_transmission("收到 CAR_TASK_REQUEST，任务期只接收", now)
                request = decode_car_task_request(frame.payload)
                self._emit(TaskRequestEvent(request, frame.sequence, now))
                return

            if frame.message_type == MessageType.MISSION_STATUS:
                if frame.source != ADDRESS_AIRCRAFT or frame.destination != ADDRESS_BROADCAST:
                    return
                self._lock_transmission("收到 MISSION_STATUS，任务期只接收", now)
                status = decode_mission_status(frame.payload)
                self._emit(MissionStatusEvent(status, frame.sequence, now))
                return

            if frame.message_type == MessageType.ACK:
                ack = decode_ack(frame.payload)
                self._handle_calibration_ack(ack, frame, now)
                return

            if frame.message_type == MessageType.MAINTENANCE_RESET:
                if (
                    frame.source != ADDRESS_CAR
                    or frame.destination != ADDRESS_BROADCAST
                    or frame.flags & ~FLAG_RETRANSMISSION
                ):
                    return
                reset = decode_maintenance_reset(frame.payload)
                if reset.reset_id in self.seen_maintenance_reset_ids:
                    # The two required retransmissions refresh parser/link
                    # statistics only.  They must not clear state a second
                    # time or silently reopen a calibration path.
                    return
                self.seen_maintenance_reset_ids.add(reset.reset_id)
                self._enter_maintenance_reset(reset, frame, now)
                return

            if frame.message_type == MessageType.MISSION_ABORT:
                if (
                    frame.source == ADDRESS_CAR
                    and frame.destination == ADDRESS_AIRCRAFT
                    and (frame.flags & (FLAG_ACK_REQUIRED | FLAG_URGENT))
                    == (FLAG_ACK_REQUIRED | FLAG_URGENT)
                ):
                    self._lock_transmission("收到 MISSION_ABORT，任务/安全期只接收", now)
                return

            if frame.message_type == MessageType.CALIBRATION_SET:
                return
            self.parser.stats.unknown_messages += 1
        except PayloadError as exc:
            self.parser.stats.payload_errors += 1
            self.logger.warning(
                "discarding V2.3 payload type=0x%02X: %s", frame.message_type, exc
            )

    def _emit_stats(self, now: float) -> None:
        stats = self.parser.stats.snapshot()
        if now - self.last_stats_logged_at >= 10.0:
            flight_age = -1.0 if self.last_flight_at is None else now - self.last_flight_at
            car_age = -1.0 if self.last_car_at is None else now - self.last_car_at
            self.logger.info(
                "UART metrics valid=%d crc=%d length=%d payload=%d unknown=%d "
                "dropped=%d flight_age=%.3fs car_age=%.3fs tx_locked=%s",
                stats["valid_frames"],
                stats["crc_errors"],
                stats["length_errors"],
                stats["payload_errors"],
                stats["unknown_messages"],
                stats["estimated_dropped"],
                flight_age,
                car_age,
                self.transmit_locked,
            )
            self.last_stats_logged_at = now
        self._emit(
            ProtocolStatsEvent(stats, self.last_valid_at, self.last_flight_at, self.last_car_at)
        )
