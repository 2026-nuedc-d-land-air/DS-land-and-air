#!/usr/bin/env python3
"""Offline Qt 6 receive-first ground station for the 2026 D-task V2.3 link."""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import logging
from logging.handlers import RotatingFileHandler
import os
from pathlib import Path
import secrets
import sys
import time

try:
    from PyQt6.QtCore import QObject, QPointF, QRectF, Qt, QTimer, pyqtSignal
    from PyQt6.QtGui import QColor, QFont, QKeyEvent, QPainter, QPen, QPixmap, QPolygonF
    from PyQt6.QtWidgets import (
        QApplication,
        QFrame,
        QGridLayout,
        QGroupBox,
        QHBoxLayout,
        QLabel,
        QMainWindow,
        QMessageBox,
        QPushButton,
        QScrollArea,
        QSizePolicy,
        QVBoxLayout,
        QWidget,
    )
except ImportError as exc:  # pragma: no cover - target dependency message.
    raise SystemExit("缺少 PyQt6，请安装 python3-pyqt6") from exc

from ground_station_protocol import MODE_TEXT, TASK_TEXT, CarPose, FlightTelemetry
from ground_station_serial import (
    DEFAULT_SERIAL_DEVICE,
    CalibrationEvent,
    CarPoseEvent,
    MaintenanceResetEvent,
    MissionStatusEvent,
    ProtocolStatsEvent,
    SerialLinkEvent,
    SerialSettings,
    SerialWorker,
    TaskRequestEvent,
    TelemetryEvent,
    TransmitLockEvent,
)
from ground_station_ui_core import (
    CalibrationCandidate,
    CalibrationStability,
    TimedCarPose,
    TimedTelemetry,
    aircraft_telemetry_to_map_display,
    calibration_candidate,
    car_pose_to_map_display,
    field_projection,
    freshness,
    stage_countdown,
)


@dataclass(frozen=True)
class AppSettings:
    serial_device: str
    baudrate: int
    flight_freshness: float
    calibration_samples: int
    calibration_stability_cm: float
    calibration_send_tolerance_cm: int
    windowed: bool
    verbose: bool


class SingleInstanceLock:
    """Keep one process in charge of the exclusive USB-TTL device."""

    def __init__(self, path: Path):
        self.path = path
        self.handle = None

    def acquire(self) -> bool:
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.handle = self.path.open("a+", encoding="utf-8")
        try:
            import fcntl

            fcntl.flock(self.handle.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
        except (ImportError, BlockingIOError):
            self.handle.close()
            self.handle = None
            return False
        self.handle.seek(0)
        self.handle.truncate()
        self.handle.write(f"{os.getpid()}\n")
        self.handle.flush()
        return True

    def release(self) -> None:
        if self.handle is None:
            return
        try:
            import fcntl

            fcntl.flock(self.handle.fileno(), fcntl.LOCK_UN)
        except ImportError:
            pass
        self.handle.close()
        self.handle = None


def configure_logging(verbose: bool) -> logging.Logger:
    state_dir = Path(os.environ.get("XDG_STATE_HOME", Path.home() / ".local/state"))
    log_dir = state_dir / "d-task-ground-station"
    log_dir.mkdir(parents=True, exist_ok=True)
    logger = logging.getLogger("d-task-ground-station")
    logger.setLevel(logging.DEBUG if verbose else logging.INFO)
    logger.handlers.clear()
    formatter = logging.Formatter("%(asctime)s %(levelname)s %(threadName)s %(message)s")
    file_handler = RotatingFileHandler(
        log_dir / "ground_station_ui.log", maxBytes=1_000_000, backupCount=3, encoding="utf-8"
    )
    file_handler.setFormatter(formatter)
    logger.addHandler(file_handler)
    if verbose:
        console = logging.StreamHandler()
        console.setFormatter(formatter)
        logger.addHandler(console)
    return logger


class SerialBridge(QObject):
    event_received = pyqtSignal(object)

    def publish(self, event: object) -> None:
        self.event_received.emit(event)


class FieldWidget(QWidget):
    """Top-down 400 x 500 cm field view; markers are platform/reference centers."""

    # Pixel centerlines of the 400 x 500 cm boundary in the supplied clean map.
    # Drawing the bitmap around this calibrated rectangle keeps the official
    # track and H markings aligned with FIELD_GLOBAL coordinates.
    MAP_BORDER_LEFT_PX = 4.5
    MAP_BORDER_TOP_PX = 4.5
    MAP_BORDER_RIGHT_PX = 606.5
    MAP_BORDER_BOTTOM_PX = 757.5

    def __init__(self):
        super().__init__()
        self.setMinimumSize(360, 420)
        self.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        self.aircraft: FlightTelemetry | None = None
        self.car: CarPose | None = None
        self.aircraft_fresh = False
        self.car_fresh = False
        self.map_path = Path(__file__).resolve().parent / "assets" / "field_map.png"
        self.map_pixmap = QPixmap(str(self.map_path))

    def set_state(
        self,
        aircraft: FlightTelemetry | None,
        car: CarPose | None,
        aircraft_fresh: bool,
        car_fresh: bool,
    ) -> None:
        self.aircraft = aircraft
        self.car = car
        self.aircraft_fresh = aircraft_fresh
        self.car_fresh = car_fresh
        self.update()

    def paintEvent(self, _event: object) -> None:
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)
        painter.fillRect(self.rect(), QColor("#081018"))

        margin_left, margin_top, margin_right, margin_bottom = 48.0, 34.0, 24.0, 42.0
        available_w = max(1.0, self.width() - margin_left - margin_right)
        available_h = max(1.0, self.height() - margin_top - margin_bottom)
        scale = min(available_w / 400.0, available_h / 500.0)
        field_w, field_h = 400.0 * scale, 500.0 * scale
        left = margin_left + (available_w - field_w) / 2.0
        top = margin_top + (available_h - field_h) / 2.0
        field = QRectF(left, top, field_w, field_h)

        if self.map_pixmap.isNull():
            painter.setPen(QPen(QColor("#ff6b6b"), 2))
            painter.setBrush(QColor("#101d27"))
            painter.drawRect(field)
            painter.drawText(field, Qt.AlignmentFlag.AlignCenter, "比赛场地图资源缺失")
        else:
            scale_x = field.width() / (
                self.MAP_BORDER_RIGHT_PX - self.MAP_BORDER_LEFT_PX
            )
            scale_y = field.height() / (
                self.MAP_BORDER_BOTTOM_PX - self.MAP_BORDER_TOP_PX
            )
            map_rect = QRectF(
                field.left() - self.MAP_BORDER_LEFT_PX * scale_x,
                field.top() - self.MAP_BORDER_TOP_PX * scale_y,
                self.map_pixmap.width() * scale_x,
                self.map_pixmap.height() * scale_y,
            )
            painter.drawPixmap(map_rect, self.map_pixmap, QRectF(self.map_pixmap.rect()))

        painter.setFont(QFont("Sans", 9))
        painter.setPen(QColor("#a9c1cf"))
        painter.drawText(QPointF(field.right() - 98, field.top() - 10), "↑ +X 500 cm")
        painter.drawText(QPointF(field.left(), field.bottom() + 24), "← +Y 400 cm")
        painter.drawText(QPointF(field.right() - 78, field.bottom() + 24), "原点 (0,0)")

        if self.car is not None:
            self._draw_marker(
                painter,
                field,
                self.car.x_cm,
                self.car.y_cm,
                self.car.yaw_tenths_degree,
                "车平台中心",
                QColor("#00845a") if self.car_fresh else QColor("#697980"),
                square=True,
            )
        if self.aircraft is not None:
            self._draw_marker(
                painter,
                field,
                self.aircraft.x_cm,
                self.aircraft.y_cm,
                self.aircraft.yaw_tenths_degree,
                "飞机参考点",
                QColor("#006dcc") if self.aircraft_fresh else QColor("#697980"),
                square=False,
            )

    def _draw_marker(
        self,
        painter: QPainter,
        field: QRectF,
        x_cm: int,
        y_cm: int,
        yaw_tenths: int,
        label: str,
        color: QColor,
        *,
        square: bool,
    ) -> None:
        local_x, local_y, inside = field_projection(
            x_cm, y_cm, field.width(), field.height()
        )
        center = QPointF(field.left() + local_x, field.top() + local_y)
        painter.setPen(QPen(QColor("#ff6b6b") if not inside else color, 2))
        painter.setBrush(QColor(color.red(), color.green(), color.blue(), 80))
        if square:
            painter.drawRect(QRectF(center.x() - 8, center.y() - 8, 16, 16))
        else:
            painter.drawEllipse(center, 8, 8)

        import math

        yaw = math.radians(yaw_tenths / 10.0)
        tip = QPointF(center.x() - 22 * math.sin(yaw), center.y() - 22 * math.cos(yaw))
        painter.drawLine(center, tip)
        painter.setPen(QColor("#10222d"))
        suffix = "（场外，贴边显示）" if not inside else ""
        painter.drawText(QPointF(center.x() + 10, center.y() - 10), label + suffix)


def make_value_label(text: str = "—") -> QLabel:
    label = QLabel(text)
    label.setWordWrap(True)
    label.setTextInteractionFlags(Qt.TextInteractionFlag.TextSelectableByMouse)
    label.setObjectName("valueLabel")
    return label


class GroundStationWindow(QMainWindow):
    def __init__(self, settings: AppSettings, logger: logging.Logger):
        super().__init__()
        self.settings = settings
        self.logger = logger
        self.setWindowTitle("2026 D题 · V2.3 树莓派地面站")
        self.resize(1100, 650)

        self.serial_open = False
        self.serial_state = "opening"
        self.serial_message = "初始化中"
        self.telemetry: TimedTelemetry | None = None
        self.car_pose: TimedCarPose | None = None
        self.task_request = None
        self.task_request_at: float | None = None
        self.mission_status = None
        self.mission_status_at: float | None = None
        self.protocol_stats: dict[str, int] = {}
        self.tx_locked = False
        self.tx_lock_reason = "尚未进入 READY；仅校准维护窗口可发送"
        self.calibration_state = "idle"
        self.maintenance_reset_id: int | None = None
        self.maintenance_reset_pending = False
        self.current_candidate: CalibrationCandidate | None = None
        self.stability = CalibrationStability(
            settings.calibration_samples, settings.calibration_stability_cm
        )

        self.bridge = SerialBridge(self)
        self.bridge.event_received.connect(self._handle_serial_event)
        serial_settings = SerialSettings(
            device=settings.serial_device,
            baudrate=settings.baudrate,
            flight_freshness=settings.flight_freshness,
            calibration_send_tolerance_cm=settings.calibration_send_tolerance_cm,
        )
        self.serial_worker = SerialWorker(serial_settings, self.bridge.publish, logger)

        self._build_ui()
        self.refresh_timer = QTimer(self)
        self.refresh_timer.setInterval(100)
        self.refresh_timer.timeout.connect(self._refresh)
        self.refresh_timer.start()
        self._refresh()

    def _build_ui(self) -> None:
        root = QWidget()
        self.setCentralWidget(root)
        outer = QVBoxLayout(root)
        outer.setContentsMargins(10, 8, 10, 8)
        outer.setSpacing(8)

        header = QHBoxLayout()
        title = QLabel("D题陆空协同 · V2.3 地面站")
        title.setObjectName("title")
        header.addWidget(title)
        header.addStretch(1)
        self.clock_label = QLabel()
        self.clock_label.setObjectName("clock")
        header.addWidget(self.clock_label)
        outer.addLayout(header)

        self.receive_banner = QLabel("接收优先 · 任务阶段 0 Hz 发送")
        self.receive_banner.setObjectName("safeBanner")
        self.receive_banner.setAlignment(Qt.AlignmentFlag.AlignCenter)
        outer.addWidget(self.receive_banner)

        content = QHBoxLayout()
        content.setSpacing(10)
        outer.addLayout(content, 1)
        self.field_widget = FieldWidget()
        content.addWidget(self.field_widget, 3)

        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setMinimumWidth(390)
        scroll.setFrameShape(QFrame.Shape.NoFrame)
        panel = QWidget()
        panel_layout = QVBoxLayout(panel)
        panel_layout.setContentsMargins(2, 0, 2, 0)
        panel_layout.setSpacing(8)
        scroll.setWidget(panel)
        content.addWidget(scroll, 2)

        link_group = QGroupBox("链路与新鲜度")
        link_grid = QGridLayout(link_group)
        self.link_label = make_value_label()
        self.flight_fresh_label = make_value_label()
        self.car_fresh_label = make_value_label()
        self.tx_label = make_value_label()
        self._grid_row(link_grid, 0, "USB-TTL", self.link_label)
        self._grid_row(link_grid, 1, "飞机链路", self.flight_fresh_label)
        self._grid_row(link_grid, 2, "小车链路", self.car_fresh_label)
        self._grid_row(link_grid, 3, "发送闩锁", self.tx_label)
        panel_layout.addWidget(link_group)

        pose_group = QGroupBox("平台中心与飞机")
        pose_grid = QGridLayout(pose_group)
        self.flight_pose_label = make_value_label()
        self.car_pose_label = make_value_label()
        self.calibration_id_label = make_value_label()
        self._grid_row(pose_grid, 0, "飞机", self.flight_pose_label)
        self._grid_row(pose_grid, 1, "车平台中心", self.car_pose_label)
        self._grid_row(pose_grid, 2, "校准 ID", self.calibration_id_label)
        panel_layout.addWidget(pose_group)

        mission_group = QGroupBox("任务（仅接收）")
        mission_grid = QGridLayout(mission_group)
        self.task_label = make_value_label()
        self.stage_label = make_value_label()
        self.error_label = make_value_label()
        self.countdown_label = make_value_label()
        self._grid_row(mission_grid, 0, "任务类型", self.task_label)
        self._grid_row(mission_grid, 1, "任务阶段", self.stage_label)
        self._grid_row(mission_grid, 2, "错误", self.error_label)
        self._grid_row(mission_grid, 3, "倒计时", self.countdown_label)
        panel_layout.addWidget(mission_group)

        calibration_group = QGroupBox("起飞前校准（唯一维护页面）")
        calibration_layout = QVBoxLayout(calibration_group)
        explain = QLabel(
            "仅当飞机遥测新鲜、小车未校准且未运行时采样。人工确认后，程序只在 "
            "CAR_POSE Seq mod 5=2 的接收完成后 30–50 ms 发送一次。"
        )
        explain.setWordWrap(True)
        calibration_layout.addWidget(explain)
        self.delta_label = make_value_label("Δx/Δy：等待有效快照")
        self.stability_label = make_value_label()
        self.calibration_progress_label = make_value_label("尚未确认")
        calibration_layout.addWidget(self.delta_label)
        calibration_layout.addWidget(self.stability_label)
        calibration_layout.addWidget(self.calibration_progress_label)
        self.confirm_button = QPushButton("人工确认本次 Δx/Δy")
        self.confirm_button.clicked.connect(self._confirm_calibration)
        calibration_layout.addWidget(self.confirm_button)
        panel_layout.addWidget(calibration_group)

        diagnostics_group = QGroupBox("协议诊断")
        diagnostics_layout = QVBoxLayout(diagnostics_group)
        self.diagnostics_label = make_value_label()
        diagnostics_layout.addWidget(self.diagnostics_label)
        panel_layout.addWidget(diagnostics_group)
        panel_layout.addStretch(1)

        self.setStyleSheet(
            """
            QMainWindow, QWidget { background: #0b141c; color: #dce8ef; }
            QLabel#title { font-size: 22px; font-weight: 700; color: #f0f7fb; }
            QLabel#clock { font-size: 15px; color: #9cb5c4; }
            QLabel#safeBanner { background: #173a32; color: #8ff0c6; border: 1px solid #286653; border-radius: 5px; padding: 6px; font-weight: 700; }
            QGroupBox { border: 1px solid #29404f; border-radius: 6px; margin-top: 8px; padding-top: 8px; font-weight: 700; color: #9ec6db; }
            QGroupBox::title { subcontrol-origin: margin; left: 10px; padding: 0 4px; }
            QLabel#valueLabel { color: #edf5f8; font-weight: 400; }
            QPushButton { background: #1b5d78; color: white; border: 1px solid #3584a5; border-radius: 5px; padding: 9px; font-weight: 700; }
            QPushButton:hover:enabled { background: #24799b; }
            QPushButton:disabled { background: #151f26; color: #63727b; border-color: #29353c; }
            QScrollArea { background: transparent; }
            """
        )

    @staticmethod
    def _grid_row(grid: QGridLayout, row: int, name: str, value: QLabel) -> None:
        key = QLabel(name)
        key.setAlignment(Qt.AlignmentFlag.AlignTop)
        key.setStyleSheet("color: #7f9aa9; font-weight: 400;")
        grid.addWidget(key, row, 0)
        grid.addWidget(value, row, 1)
        grid.setColumnStretch(1, 1)

    def start_services(self) -> None:
        self.serial_worker.start()

    def _handle_serial_event(self, event: object) -> None:
        if isinstance(event, SerialLinkEvent):
            self.serial_state = event.state
            self.serial_open = event.state == "open"
            self.serial_message = event.message
        elif isinstance(event, TelemetryEvent):
            self.telemetry = TimedTelemetry(event.telemetry, event.received_at)
        elif isinstance(event, CarPoseEvent):
            self.car_pose = TimedCarPose(event.pose, event.sequence, event.received_at)
        elif isinstance(event, TaskRequestEvent):
            self.task_request = event.request
            self.task_request_at = event.received_at
        elif isinstance(event, MissionStatusEvent):
            self.mission_status = event.status
            self.mission_status_at = event.received_at
        elif isinstance(event, ProtocolStatsEvent):
            self.protocol_stats = event.stats
        elif isinstance(event, CalibrationEvent):
            self.calibration_state = event.state
            prefix = f"ID {event.calibration_id} · " if event.calibration_id else ""
            self.calibration_progress_label.setText(prefix + event.message)
        elif isinstance(event, MaintenanceResetEvent):
            self.maintenance_reset_id = event.reset.reset_id
            self.maintenance_reset_pending = True
            self.tx_locked = False
            self.tx_lock_reason = "maintenance reset accepted; waiting for fresh preflight data"
            self.telemetry = None
            self.car_pose = None
            self.task_request = None
            self.task_request_at = None
            self.mission_status = None
            self.mission_status_at = None
            self.current_candidate = None
            self.stability.clear()
            self.calibration_state = "maintenance_reset_pending"
            self.calibration_progress_label.setText(
                f"收到维护复位 ID {event.reset.reset_id}；已清除旧校准与任务状态。"
                "等待新的 IDLE/未校准数据后，仍须人工再次确认。"
            )
        elif isinstance(event, TransmitLockEvent):
            self.tx_locked = True
            self.tx_lock_reason = event.reason
            self.calibration_state = "locked"
            self.maintenance_reset_pending = False
            self.stability.clear()
        self._refresh()

    def _refresh(self) -> None:
        now = time.monotonic()
        self.clock_label.setText(time.strftime("%Y-%m-%d %H:%M:%S"))
        self.link_label.setText(
            f"{self.serial_state.upper()} · {self.settings.serial_device}\n{self.serial_message}"
        )

        flight_at = None if self.telemetry is None else self.telemetry.received_at
        car_at = None if self.car_pose is None else self.car_pose.received_at
        flight_fresh = freshness(
            flight_at,
            now,
            warning_after=min(0.8, self.settings.flight_freshness),
            invalid_after=self.settings.flight_freshness,
        )
        car_fresh = freshness(car_at, now, warning_after=0.28, invalid_after=0.5)
        self.flight_fresh_label.setText(flight_fresh.text)
        self.car_fresh_label.setText(car_fresh.text + "（>280ms 警告，>500ms 失效）")

        if self.tx_locked:
            self.tx_label.setText("永久禁用 · " + self.tx_lock_reason)
            self.receive_banner.setText("READY / 任务只收模式 · 本进程发送路径已永久熔断")
            self.receive_banner.setStyleSheet(
                "background:#4a2529;color:#ffd4d7;border:1px solid #8a4048;border-radius:5px;padding:6px;font-weight:700;"
            )
        else:
            self.tx_label.setText("受限启用 · 仅 CALIBRATION_SET 维护槽")

        if self.maintenance_reset_pending and not self.tx_locked:
            reset_id = "—" if self.maintenance_reset_id is None else str(self.maintenance_reset_id)
            self.tx_label.setText(
                f"维护复位待确认（ResetId {reset_id}）· 等待新鲜 IDLE/未校准数据与人工再次确认"
            )
            self.receive_banner.setText(
                "MAINTENANCE RESET · 已清除旧校准/任务；不会自动发送，必须重新人工确认"
            )
            self.receive_banner.setStyleSheet(
                "background:#4a3a1a;color:#ffe4a3;border:1px solid #967132;border-radius:5px;padding:6px;font-weight:700;"
            )

        flight = None if self.telemetry is None else self.telemetry.value
        display_flight = (
            None if flight is None else aircraft_telemetry_to_map_display(flight)
        )
        car = None if self.car_pose is None else self.car_pose.value
        display_car = None if car is None else car_pose_to_map_display(car)
        if display_flight is None:
            self.flight_pose_label.setText("未收到")
        else:
            speed = (
                "无效"
                if display_flight.ground_speed_cm_s == 0xFFFF
                else f"{display_flight.ground_speed_cm_s} cm/s"
            )
            self.flight_pose_label.setText(
                f"地图 X {display_flight.x_cm} · Y {display_flight.y_cm} · Z {display_flight.z_cm} cm\n"
                f"yaw {display_flight.yaw_tenths_degree / 10:.1f}° · 速度 {speed} · H 显示偏移 +113/+288"
            )
        if display_car is None:
            self.car_pose_label.setText("未收到")
            self.calibration_id_label.setText("—")
        else:
            flags = []
            if display_car.position_valid:
                flags.append("位置有效")
            if display_car.yaw_valid:
                flags.append("yaw有效")
            if display_car.calibrated:
                flags.append("已校准")
            if display_car.car_running:
                flags.append("运行中")
            display_origin = "H 共用坐标" if display_car.calibrated else "A 起点坐标"
            self.car_pose_label.setText(
                f"地图 X {display_car.x_cm} · Y {display_car.y_cm} cm · "
                f"yaw {display_car.yaw_tenths_degree / 10:.1f}°\n"
                + (" / ".join(flags) if flags else "无有效标志")
                + f" · {display_origin}显示"
            )
            self.calibration_id_label.setText(
                f"{display_car.calibration_id} (0x{display_car.calibration_id:04X}) · "
                + (
                    "FIELD_GLOBAL"
                    if display_car.coordinate_frame == 1
                    else f"坐标系 0x{display_car.coordinate_frame:02X}"
                )
            )

        self.field_widget.set_state(
            display_flight,
            display_car,
            flight_fresh.state == "fresh",
            car_fresh.state == "fresh",
        )
        self._refresh_mission(now, flight)
        self._refresh_calibration(now)
        stats = self.protocol_stats
        self.diagnostics_label.setText(
            "有效 {valid} · CRC错 {crc} · 长度错 {length} · 载荷错 {payload}\n"
            "版本错 {version} · 地址错 {address} · 标志错 {flag} · 未知 {unknown}\n"
            "估计丢序 {drop} · 噪声字节 {noise} · 源复位 {reset}".format(
                valid=stats.get("valid_frames", 0),
                crc=stats.get("crc_errors", 0),
                length=stats.get("length_errors", 0),
                payload=stats.get("payload_errors", 0),
                version=stats.get("version_errors", 0),
                address=stats.get("address_errors", 0),
                flag=stats.get("flag_errors", 0),
                unknown=stats.get("unknown_messages", 0),
                drop=stats.get("estimated_dropped", 0),
                noise=stats.get("noise_bytes", 0),
                reset=stats.get("source_resets", 0),
            )
        )

    def _refresh_mission(self, now: float, flight: FlightTelemetry | None) -> None:
        if self.mission_status is not None:
            task_type = self.mission_status.task_type
            mission_id = self.mission_status.mission_id
            stage = self.mission_status.stage
            error = self.mission_status.error_code
        elif self.task_request is not None:
            task_type = self.task_request.task_type
            mission_id = self.task_request.mission_id
            stage = None if flight is None else flight.mode_code
            error = None
        else:
            task_type = None
            mission_id = None
            stage = None if flight is None else flight.mode_code
            error = None
        task_text = "—" if task_type is None else TASK_TEXT.get(task_type, f"未知 0x{task_type:02X}")
        if mission_id is not None:
            task_text += f" · MissionId {mission_id}"
        self.task_label.setText(task_text)
        self.stage_label.setText("—" if stage is None else MODE_TEXT.get(stage, f"未知阶段 {stage}"))
        if error is None:
            self.error_label.setText("—（尚无 MISSION_STATUS）")
        elif error == 0:
            self.error_label.setText("0x00 · 无错误")
        else:
            self.error_label.setText(f"0x{error:02X}（错误码语义由飞控联调确认）")
        self.countdown_label.setText(stage_countdown(stage, self.mission_status_at, now))

    def _refresh_calibration(self, now: float) -> None:
        candidate, reason = calibration_candidate(
            self.telemetry,
            self.car_pose,
            now,
            flight_freshness_seconds=self.settings.flight_freshness,
        )
        self.current_candidate = candidate
        if candidate is None:
            if not self.tx_locked:
                self.stability.clear()
            self.delta_label.setText("Δx/Δy：不可计算 · " + reason)
        else:
            self.stability.observe(candidate)
            self.delta_label.setText(
                f"Δx = {candidate.delta_x_cm} cm · Δy = {candidate.delta_y_cm} cm"
            )
        spread = self.stability.spread_cm
        spread_text = "—" if spread is None else f"{spread:.1f} cm"
        self.stability_label.setText(
            f"稳定样本 {self.stability.count}/{self.settings.calibration_samples} · "
            f"二维极差 {spread_text} · 阈值 ≤{self.settings.calibration_stability_cm:.1f} cm"
        )
        busy = self.calibration_state in {
            "waiting_window",
            "window_armed",
            "waiting_ack",
            "waiting_calibrated_pose",
        }
        self.confirm_button.setEnabled(
            self.serial_open
            and not self.tx_locked
            and not busy
            and candidate is not None
            and self.stability.stable
        )
        self.confirm_button.setText(
            "人工再次确认本次 Δx/Δy"
            if self.maintenance_reset_pending
            else "人工确认本次 Δx/Δy"
        )

    def _confirm_calibration(self) -> None:
        candidate = self.current_candidate
        if candidate is None or not self.stability.stable or self.tx_locked:
            return
        calibration_id = secrets.randbelow(0xFFFF) + 1
        answer = QMessageBox.question(
            self,
            "确认起飞前校准",
            f"确认飞机与小车平台十字中心已人工对准、飞机未解锁、小车未行驶？\n\n"
            f"Δx = {candidate.delta_x_cm} cm\nΔy = {candidate.delta_y_cm} cm\n"
            f"CalibrationId = {calibration_id} (0x{calibration_id:04X})\n\n"
            "确认后程序将等待 Seq mod 5=2 维护窗口；不会自动重发。",
            QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No,
            QMessageBox.StandardButton.No,
        )
        if answer != QMessageBox.StandardButton.Yes:
            return
        self.calibration_state = "submitting"
        self.calibration_progress_label.setText("正在把人工确认快照交给串口安全门…")
        self.serial_worker.submit_calibration(
            candidate.delta_x_cm,
            candidate.delta_y_cm,
            calibration_id,
            candidate.flight_source_time_ms,
            candidate.car_source_time_ms,
        )
        self._refresh()

    def keyPressEvent(self, event: QKeyEvent) -> None:
        if event.key() == Qt.Key.Key_F11:
            self.showNormal() if self.isFullScreen() else self.showFullScreen()
            return
        if event.key() == Qt.Key.Key_Escape and self.isFullScreen():
            self.showNormal()
            return
        super().keyPressEvent(event)

    def closeEvent(self, event: object) -> None:
        self.refresh_timer.stop()
        self.serial_worker.stop()
        event.accept()  # type: ignore[attr-defined]


def parse_args(argv: list[str] | None = None) -> AppSettings:
    parser = argparse.ArgumentParser(description="D题 V2.3 树莓派地面站（离线、任务期只收）")
    parser.add_argument("--serial-device", default=DEFAULT_SERIAL_DEVICE)
    parser.add_argument("--baudrate", type=int, default=115200)
    parser.add_argument("--flight-freshness", type=float, default=1.2)
    parser.add_argument("--calibration-samples", type=int, default=5)
    parser.add_argument("--calibration-stability-cm", type=float, default=5.0)
    parser.add_argument("--calibration-send-tolerance-cm", type=int, default=5)
    parser.add_argument("--windowed", action="store_true")
    parser.add_argument("--verbose", action="store_true")
    args = parser.parse_args(argv)
    if args.baudrate <= 0:
        parser.error("--baudrate must be greater than zero")
    if args.flight_freshness <= 0:
        parser.error("--flight-freshness must be greater than zero")
    if args.calibration_samples < 2:
        parser.error("--calibration-samples must be at least two")
    if args.calibration_stability_cm <= 0:
        parser.error("--calibration-stability-cm must be greater than zero")
    if args.calibration_send_tolerance_cm < 0:
        parser.error("--calibration-send-tolerance-cm cannot be negative")
    return AppSettings(
        args.serial_device,
        args.baudrate,
        args.flight_freshness,
        args.calibration_samples,
        args.calibration_stability_cm,
        args.calibration_send_tolerance_cm,
        args.windowed,
        args.verbose,
    )


def main(argv: list[str] | None = None) -> int:
    settings = parse_args(argv)
    logger = configure_logging(settings.verbose)
    state_dir = Path(os.environ.get("XDG_STATE_HOME", Path.home() / ".local/state"))
    lock = SingleInstanceLock(state_dir / "d-task-ground-station" / "ground_station_ui.lock")
    if not lock.acquire():
        logger.error("another ground station instance owns the serial link")
        return 2
    try:
        app = QApplication(sys.argv[:1])
        app.setApplicationName("D Task V2.3 Ground Station")
        window = GroundStationWindow(settings, logger)
        if settings.windowed:
            window.show()
        else:
            window.showFullScreen()
        QTimer.singleShot(0, window.start_services)
        return app.exec()
    except Exception:
        logger.exception("ground station UI failed")
        return 1
    finally:
        lock.release()


if __name__ == "__main__":
    raise SystemExit(main())
