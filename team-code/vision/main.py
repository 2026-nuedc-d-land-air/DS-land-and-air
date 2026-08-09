"""
MaixCam2 无人机视觉引导系统 — V2.2 协议版
===========================================
单文件自包含，可直接在 MaixVision 中运行。

功能：检测小车平台上的"大圈"标识，通过 V2.2 协议向飞控上报
     中心偏移量 (CAMERA_TARGET)，接收飞控指令 (CAMERA_MODE/ACTION)。

通信：有线 UART (115200, 8N1), V2.2 协议帧 (AA 55 02 ... CRC16)
地址：相机 0x50 ↔ 飞控 0x21
"""

from maix import camera, display, image, nn, app, time, pinmap, pwm, uart, err
import struct, os, math


# ============================================================
# MaixCAM2 UART 适配器
# ============================================================

# 台架接线（MaixCAM2 的 UART1）：
#   飞控 TX -> A31 (UART1_RX)
#   飞控 RX <- A30 (UART1_TX)
#   两端必须共地，且 UART 电平为 3.3 V。
# 注意：A21/A22 才是 UART4；当前接在 A31/A30 时必须使用 UART1。
UART_DEVICE   = '/dev/ttyS1'
UART_BAUDRATE = 115200
UART_RX_PIN   = 'A31'
UART_TX_PIN   = 'A30'
UART_RX_FUNC  = 'UART1_RX'
UART_TX_FUNC  = 'UART1_TX'

class _MaixUART:
    """
    MaixCAM2 UART1 适配器。

    A31/A30 并非系统默认串口引脚，必须先通过 pinmap 配置复用，
    再由 MaixPy UART 驱动以 115200、8N1 打开 /dev/ttyS1。不能只用
    os.open() 打开字符设备，否则引脚复用和波特率都无法得到保证。
    """

    def __init__(self):
        self._uart = None

        try:
            err.check_raise(pinmap.set_pin_function(UART_RX_PIN, UART_RX_FUNC))
            err.check_raise(pinmap.set_pin_function(UART_TX_PIN, UART_TX_FUNC))
            self._uart = uart.UART(UART_DEVICE, UART_BAUDRATE)
            print(f"[UART] UART1 就绪: {UART_DEVICE}, {UART_BAUDRATE} 8N1 "
                  f"(RX={UART_RX_PIN}, TX={UART_TX_PIN})")
        except Exception as e:
            self._uart = None
            print(f"[UART] UART1 初始化失败: {e}")

    def ok(self):
        return self._uart is not None

    def read(self):
        if self._uart is None:
            return None
        try:
            data = self._uart.read()
            return data if data else None
        except Exception:
            return None

    def write(self, data):
        if self._uart is None:
            return 0
        try:
            return self._uart.write(data)
        except Exception:
            return 0

    def deinit(self):
        if self._uart is not None:
            try:
                self._uart.close()
            except Exception:
                pass
            self._uart = None


# ============================================================
# V2.2 协议层 (内联自 protocol_v2.py，单文件部署)
# ============================================================
# 帧格式: AA 55 | 02 | Type | Src | Dst | Seq | Flags | Len | Payload | CRC16
# 字节序: 大端 | CRC: CRC16/CCITT-FALSE (poly=0x1021, init=0xFFFF)

# ---- 地址 ----
ADDR_BROADCAST  = 0x10
ADDR_FC         = 0x21
ADDR_CAMERA     = 0x50

# ---- 消息类型 ----
TYPE_ACK                = 0x11
TYPE_CAMERA_MODE        = 0x90
TYPE_CAMERA_TARGET      = 0x91
TYPE_CAMERA_ACTION      = 0x92
TYPE_CAMERA_ACTION_RESULT = 0x93

# ---- CAMERA_ACTION 动作码 ----
# The V2.2 bench simulator sends Action=0 for the one supported DROP action.
ACTION_DROP             = 0x01

# ---- 帧标志 ----
FLAG_ACK_REQUIRED   = 0x01
FLAG_RETRANSMISSION = 0x02

# ---- 帧常量 ----
FRAME_HEADER_1   = 0xAA
FRAME_HEADER_2   = 0x55
PROTOCOL_VERSION = 0x02
MAX_PAYLOAD_LEN  = 64
FRAME_MIN_LEN    = 11

# ---- ACK 码 ----
ACK_ACCEPTED          = 0x00
ACK_BUSY              = 0x02
ACK_NOT_ALLOWED       = 0x03
ACK_NOT_SUPPORTED     = 0x04
ACK_INVALID_PARAMETER = 0x05
ACK_INTERNAL_ERROR    = 0x06

# ---- 动作结果码 ----
ACTION_COMPLETED = 0
ACTION_FAILED    = 3


# ---- CRC16/CCITT-FALSE ----

def crc16_ccitt(data):
    """CRC16/CCITT-FALSE。测试向量: b'123456789' → 0x29B1"""
    crc = 0xFFFF
    for byte in data:
        crc ^= (byte << 8)
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


# ---- V2.2 帧编码 ----

def encode_frame(msg_type, src, dst, seq, flags, payload=b''):
    """编码完整 V2.2 帧: AA 55 02 ... CRC16"""
    if len(payload) > MAX_PAYLOAD_LEN:
        raise ValueError("Payload too long")

    body = struct.pack('>BBBBBBB',
                       PROTOCOL_VERSION, msg_type,
                       src, dst, seq, flags, len(payload))
    body += payload
    crc = crc16_ccitt(body)
    return b'\xAA\x55' + body + struct.pack('>H', crc)


def encode_ack_payload(request_type, request_seq, result, detail=0):
    """ACK Payload: Type(1) + RequestSeq(1) + Result(1) + Detail(1)."""
    return struct.pack('>BBBB', request_type, request_seq, result, detail)


def _send_command_ack(frame, uart, seq_mgr, result, detail=0):
    """Send and return a command ACK for deduplicated result replay."""
    ack_frame = encode_frame(
        TYPE_ACK, ADDR_CAMERA, frame['src'],
        seq_mgr.next(), 0,
        encode_ack_payload(frame['type'], frame['seq'], result, detail))
    uart.write(ack_frame)
    return result


def encode_target_payload(flags, quality, err_x, err_y,
                          outer_diameter, frame_counter, source_time_ms,
                          task_type, mission_id, mode_seq):
    """
    CAMERA_TARGET Payload (18B).
    前14B: Flags(1) Quality(1) ErrX(2) ErrY(2) Diameter(2) FrameCnt(2) Time(4)
    后4B:  TaskType(1) MissionId(2) ModeSeq(1)
    """
    return struct.pack('>BBhhHHIBHB',
                       flags, quality,
                       err_x, err_y,
                       outer_diameter, frame_counter,
                       source_time_ms,
                       task_type, mission_id, mode_seq)


def encode_action_result_payload(action, result, action_id, source_time_ms,
                                 task_type, mission_id, mode_seq):
    """
    CAMERA_ACTION_RESULT Payload (12B):
      Action(1) + Result(1) + ActionId(2) + SourceTimeMs(4)
      + TaskType(1) + MissionId(2) + ModeSeq(1).
    """
    return struct.pack('>BBHIBHB',
                       action, result, action_id, source_time_ms,
                       task_type, mission_id, mode_seq)


# ---- 流式帧解析器 ----

class V2Parser:
    """V2.2 流式帧解析器: 字节重同步 → CRC校验 → 地址匹配。"""

    def __init__(self, src_addr):
        self._buf = bytearray()
        self._src_addr = src_addr

    def feed(self, data):
        """喂入字节，返回解析出的帧列表。"""
        self._buf.extend(data)
        frames = []
        while True:
            f = self._try_parse()
            if f is None:
                break
            frames.append(f)
        return frames

    def reset(self):
        self._buf = bytearray()

    def _try_parse(self):
        buf = self._buf

        # 搜索帧头 AA 55
        while len(buf) >= 2:
            if buf[0] == FRAME_HEADER_1 and buf[1] == FRAME_HEADER_2:
                break
            del buf[0]
        else:
            return None

        if len(buf) < FRAME_MIN_LEN:
            return None

        version     = buf[2]
        msg_type    = buf[3]
        src         = buf[4]
        dst         = buf[5]
        seq         = buf[6]
        flags       = buf[7]
        payload_len = buf[8]

        if version != PROTOCOL_VERSION or payload_len > MAX_PAYLOAD_LEN:
            del buf[0]
            return None

        frame_len = 9 + payload_len + 2
        if len(buf) < frame_len:
            return None

        payload = bytes(buf[9 : 9 + payload_len])
        crc_rx  = (buf[9 + payload_len] << 8) | buf[9 + payload_len + 1]
        crc_calc = crc16_ccitt(buf[2 : 9 + payload_len])

        if crc_calc != crc_rx:
            del buf[0]
            return None

        if dst != self._src_addr and dst != ADDR_BROADCAST:
            del buf[:frame_len]
            return None

        del buf[:frame_len]
        return {
            'type':    msg_type,
            'src':     src,
            'dst':     dst,
            'seq':     seq,
            'flags':   flags,
            'payload': payload,
            'time':    time.ticks_ms(),
        }


# ---- 序列号管理器 ----

class SeqManager:
    """发送序列号: 0–255 回绕。"""
    def __init__(self):
        self._seq = 0

    def next(self):
        s = self._seq
        self._seq = (self._seq + 1) & 0xFF
        return s


# ---- 帧去重器 ----

class Deduplicator:
    """接收去重: (Src, Type, Seq) 5s 窗口，并缓存首次 ACK Result。"""

    def __init__(self, window_ms=5000):
        self._seen = {}
        self._win = window_ms

    def _key(self, src, msg_type, seq):
        return (src, msg_type, seq)

    def _purge_expired(self):
        now = int(time.ticks_ms())
        expired = [k for k, entry in self._seen.items()
                   if now - entry[0] > self._win]
        for k in expired:
            del self._seen[k]

    def is_duplicate(self, src, msg_type, seq):
        key = self._key(src, msg_type, seq)
        self._purge_expired()
        if key in self._seen:
            return True
        self._seen[key] = (int(time.ticks_ms()), None)
        return False

    def remember_result(self, src, msg_type, seq, result):
        """保存该命令第一次已经发出的 ACK Result。"""
        key = self._key(src, msg_type, seq)
        entry = self._seen.get(key)
        timestamp = entry[0] if entry is not None else int(time.ticks_ms())
        self._seen[key] = (timestamp, result)

    def result_for(self, src, msg_type, seq):
        """返回同一命令首次 ACK 的 Result；未产生 ACK 时返回 None。"""
        entry = self._seen.get(self._key(src, msg_type, seq))
        return entry[1] if entry is not None else None

    def reset(self):
        self._seen.clear()


# ============================================================
# 投放舵机控制 (MaixPy 硬件 PWM)
# ============================================================

class DropServo:
    """
    投放舵机 PWM 控制 — 使用 MaixPy 硬件 PWM 控制器。

    替代了原有的 sysfs GPIO 位翻转方式。硬件 PWM 产生干净的
    50Hz 方波信号，舵机才能正确识别。

    The mechanical positions are configured in degrees. Pulse limits keep the
    mapping explicit because a servo's real travel must be calibrated on the
    installed release mechanism.
    """

    def __init__(self, pin_name='B3', pwm_id=5,
                 freq=50, idle_angle=90.0, release_angle=135.0,
                 angle_min=0.0, angle_max=180.0,
                 pulse_min_us=500, pulse_max_us=2500,
                 duration_ms=800):
        """
        初始化舵机 PWM 控制。

        Args:
          pin_name:    硬件引脚名 (如 "B3"), 需用 pinmap 设为 PWM 功能
          pwm_id:      PWM 通道编号 (5 = PWM5)
          freq:        PWM 频率 Hz (舵机标准 50Hz)
          idle_angle:  机构锁定时的舵机角度
          release_angle: 收到投放命令时的舵机角度
          angle_min/max: 舵机允许的命令角度范围
          pulse_min/max_us: 角度范围对应的 PWM 脉宽，用于实机标定
          duration_ms: 投放动作持续时间
        """
        self._pin_name = pin_name
        self._pwm_id = pwm_id
        self._freq = freq
        self._idle_angle = idle_angle
        self._release_angle = release_angle
        self._angle_min = angle_min
        self._angle_max = angle_max
        self._pulse_min_us = pulse_min_us
        self._pulse_max_us = pulse_max_us
        self._duration_ms = duration_ms
        self._pwm = None
        self._ready = False

        self._validate_config()
        self._duty_idle = self._angle_to_duty(idle_angle)
        self._duty_release = self._angle_to_duty(release_angle)

        try:
            # 1. 设置引脚复用为 PWM 功能
            func_name = f"PWM{pwm_id}"
            ret = pinmap.set_pin_function(pin_name, func_name)
            err.check_raise(ret)
            print(f"[SERVO] pinmap: {pin_name} → {func_name}")

            # 2. 创建 PWM 对象 (初始关闭, idle 占空比)
            self._pwm = pwm.PWM(pwm_id, freq=freq,
                                duty=self._duty_idle, enable=True)
            print(f"[SERVO] PWM{pwm_id} 就绪 "
                  f"({freq}Hz, idle={idle_angle:.1f}deg/{self._duty_idle:.2f}%, "
                  f"release={release_angle:.1f}deg/{self._duty_release:.2f}%, "
                  f"{duration_ms}ms)")
            self._ready = True

        except Exception as e:
            print(f"[SERVO] 初始化失败: {e} — 模拟模式")

    def _validate_config(self):
        if self._angle_max <= self._angle_min:
            raise ValueError("SERVO_ANGLE_MAX must be greater than SERVO_ANGLE_MIN")
        if self._pulse_max_us <= self._pulse_min_us:
            raise ValueError("SERVO_PULSE_MAX_US must be greater than SERVO_PULSE_MIN_US")
        if not (self._angle_min <= self._idle_angle <= self._angle_max):
            raise ValueError("SERVO_IDLE_ANGLE is outside the configured range")
        if not (self._angle_min <= self._release_angle <= self._angle_max):
            raise ValueError("SERVO_RELEASE_ANGLE is outside the configured range")

    def _angle_to_duty(self, angle):
        ratio = ((angle - self._angle_min) /
                 (self._angle_max - self._angle_min))
        pulse_us = (self._pulse_min_us +
                    ratio * (self._pulse_max_us - self._pulse_min_us))
        return pulse_us * self._freq / 10000.0

    def drop(self):
        """
        执行投放: 转到释放角度、保持 duration_ms、再回初始角度。

        Returns:
          (success: bool, message: str)
        """
        if not self._ready or self._pwm is None:
            print("[SERVO] 模拟投放 (无硬件 PWM)")
            t0 = int(time.ticks_ms())
            while int(time.ticks_ms()) - t0 < self._duration_ms:
                pass
            return True, "SIMULATED"

        try:
            # 切换到释放位置
            self._pwm.duty(self._duty_release)
            t0 = int(time.ticks_ms())
            while int(time.ticks_ms()) - t0 < self._duration_ms:
                pass

            # 回到空闲位置
            self._pwm.duty(self._duty_idle)

            print(f"[SERVO] 投放完成 ({self._duration_ms}ms)")
            return True, "COMPLETED"

        except Exception as e:
            print(f"[SERVO] 投放失败: {e}")
            return False, f"FAILED: {e}"

    def is_ready(self):
        return self._ready


# ============================================================
# 系统配置
# ============================================================

# SG556 release servo (MaixPy hardware PWM)
SERVO_PIN_NAME       = 'B3'     # B3 must be mapped to PWM5
SERVO_PWM_ID         = 5
SERVO_FREQ_HZ        = 50
SERVO_ANGLE_MIN      = 0.0
SERVO_ANGLE_MAX      = 180.0
SERVO_PULSE_MIN_US   = 500      # 0 deg command pulse; calibrate on hardware
SERVO_PULSE_MAX_US   = 2500     # 180 deg command pulse; calibrate on hardware
SERVO_IDLE_ANGLE     = 0.0      # 0.5 ms / 2.5% output, lock position
SERVO_RELEASE_ANGLE  = 135.0    # Current 2.0 ms / 10.0% output, release position
SERVO_DURATION_MS    = 800

# 检测参数
BIG_CIRCLE_REAL_DIAMETER_CM = 30.0
CAMERA_FOCAL_LENGTH_PX      = 280.0
# Measured optical-center offsets. Keep zero until a checkerboard calibration
# provides the principal point relative to the image center.
CAMERA_CENTER_OFFSET_X_PX   = 0.0
CAMERA_CENTER_OFFSET_Y_PX   = 0.0

CONF_THRESHOLD = 0.30
IOU_THRESHOLD  = 0.45

CLASS_BIG_CIRCLE  = 0
CLASS_LANDING_PAD = 1

TARGET_FREQ_HZ   = 18
TARGET_PERIOD_MS = 1000 // TARGET_FREQ_HZ

# Detection boxes find the target. The following checks make the final position
# depend on the known concentric-circle geometry and on temporal consistency.
CANDIDATE_MIN_SCORE       = 0.30
CANDIDATE_MIN_DIAMETER_PX = 14
CANDIDATE_ASPECT_MIN      = 0.55
CANDIDATE_ASPECT_MAX      = 1.80

# MaixPy builds with Image.find_circles() use this optional local refinement.
# If that API is absent, or a fit fails, the code safely falls back to the box.
RING_REFINEMENT_ENABLED        = True
RING_REFINE_EVERY_N_FRAMES     = 1
RING_MIN_DIAMETER_PX           = 30
RING_HOUGH_THRESHOLD           = 3500
RING_HOUGH_X_MARGIN            = 10
RING_HOUGH_R_MARGIN            = 10
RING_CENTER_GATE_RATIO         = 0.35
RING_RADIUS_TOLERANCE           = 0.35
CLASSICAL_FALLBACK_ENABLED      = True
CLASSICAL_FALLBACK_EVERY_N_FRAMES = 1
CLASSICAL_HOUGH_THRESHOLD        = 3500
CLASSICAL_HOUGH_X_MARGIN         = 10
CLASSICAL_HOUGH_R_MARGIN         = 10
CLASSICAL_MIN_RADIUS_PX          = 5
CLASSICAL_MAX_RADIUS_RATIO       = 0.34
CLASSICAL_PAIR_RATIO             = 30.0 / 50.0
CLASSICAL_PAIR_RATIO_TOLERANCE   = 0.28
CLASSICAL_PAIR_CENTER_RATIO      = 0.20

TRACK_CONFIRM_HITS       = 2
TRACK_FAST_LOCK_SCORE     = 0.75
TRACK_CONFIRM_JUMP_PX    = 32.0
TRACK_MAX_JUMP_PX        = 60.0
TRACK_REACQUIRE_HITS     = 2
TRACK_REACQUIRE_JUMP_PX  = 24.0
TRACK_MAX_DT_S           = 0.20
TRACK_PREDICT_MAX_MS     = 250
TRACK_ALPHA              = 0.55
TRACK_BETA               = 0.08

TARGET_FLAG_OBSERVED  = 0x01
TARGET_FLAG_PREDICTED = 0x02
TARGET_FLAG_LOCKED    = 0x04


# ============================================================
# 检测分析函数
# ============================================================

def find_big_circle(objs):
    """从检测结果中找到置信度最高的大圈。"""
    big = []
    for obj in objs:
        if obj.class_id != CLASS_BIG_CIRCLE or obj.score < CANDIDATE_MIN_SCORE:
            continue
        if obj.w <= 0 or obj.h <= 0:
            continue
        diameter = max(obj.w, obj.h)
        aspect = obj.w / obj.h
        if (diameter < CANDIDATE_MIN_DIAMETER_PX or
                aspect < CANDIDATE_ASPECT_MIN or
                aspect > CANDIDATE_ASPECT_MAX):
            continue
        big.append(obj)
    return max(big, key=lambda o: o.score) if big else None


def calc_offset_from_center(cx, cy, img_w, img_h):
    """Convert an image center to the existing protocol offset convention."""
    reference_x = img_w / 2.0 + CAMERA_CENTER_OFFSET_X_PX
    reference_y = img_h / 2.0 + CAMERA_CENTER_OFFSET_Y_PX
    return int(reference_y - cy), int(reference_x - cx)


def calc_offset(obj, img_w, img_h):
    """目标中心相对图像中心的像素偏移。X 正值向上，Y 正值向左。"""
    cx = obj.x + obj.w // 2
    cy = obj.y + obj.h // 2
    return calc_offset_from_center(cx, cy, img_w, img_h)


def estimate_distance(big_circle):
    """基于大圈像素尺寸估算距离 (cm)。"""
    if big_circle is None or big_circle.w <= 0:
        return -1
    return (BIG_CIRCLE_REAL_DIAMETER_CM *
            CAMERA_FOCAL_LENGTH_PX) / big_circle.w


def calc_quality(score, diameter_px, refined=False, stable_hits=0,
                 predicted=False, residual_px=0.0):
    """检测质量 0–255。"""
    if predicted:
        return max(1, min(255, int(score * 90)))
    q = int(score * 170 + min(diameter_px / 3, 40))
    q += min(30, stable_hits * 6)
    q += 22 if refined else 0
    q -= min(18, int(max(0.0, residual_px) * 3))
    return max(0, min(q, 255))


def _circle_attr(circle, name):
    """Read MaixPy circle properties exposed as an attribute or a method."""
    value = getattr(circle, name, None)
    return value() if callable(value) else value


def _box_geometry(obj):
    return {
        'cx': obj.x + obj.w / 2.0,
        'cy': obj.y + obj.h / 2.0,
        'diameter': float(max(obj.w, obj.h)),
        'refined': False,
        'residual': 0.0,
    }


def refine_concentric_circle(img, obj):
    """
    Refine a YOLO box with the platform's existing 30/50 cm concentric rings.
    Missing or incompatible MaixPy circle APIs always fall back to box geometry.
    """
    fallback = _box_geometry(obj)
    if (not RING_REFINEMENT_ENABLED or
            max(obj.w, obj.h) < RING_MIN_DIAMETER_PX or
            not hasattr(img, 'crop')):
        return fallback

    try:
        # The detector box corresponds to the 30 cm inner ring, so refine it
        # locally and let the whole-image fallback validate both circles.
        padding = max(6, int(max(obj.w, obj.h) * 0.25))
        x = max(0, int(obj.x) - padding)
        y = max(0, int(obj.y) - padding)
        w = min(img.width() - x, int(obj.w) + 2 * padding)
        h = min(img.height() - y, int(obj.h) + 2 * padding)
        if w < 16 or h < 16:
            return fallback

        roi = img.crop(x, y, w, h)
        find_circles = getattr(roi, 'find_circles', None)
        if not callable(find_circles):
            return fallback
        r_min = max(4, int(min(obj.w, obj.h) * 0.25))
        r_max = min(int(max(obj.w, obj.h) * 0.62), min(w, h) // 2)
        if r_min >= r_max:
            return fallback
        circles = find_circles(
            roi=(0, 0, w, h),
            threshold=RING_HOUGH_THRESHOLD,
            x_margin=RING_HOUGH_X_MARGIN,
            r_margin=RING_HOUGH_R_MARGIN,
            r_min=r_min, r_max=r_max, r_step=2)
    except Exception:
        return fallback
    if not circles:
        return fallback

    expected_cx = obj.x + obj.w / 2.0
    expected_cy = obj.y + obj.h / 2.0
    center_gate = max(obj.w, obj.h) * RING_CENTER_GATE_RATIO
    fitted = []
    for circle in circles:
        cx = _circle_attr(circle, 'x')
        cy = _circle_attr(circle, 'y')
        radius = _circle_attr(circle, 'r')
        if cx is None or cy is None or radius is None or radius <= 0:
            continue
        cx = float(cx) + x
        cy = float(cy) + y
        if math.hypot(cx - expected_cx, cy - expected_cy) <= center_gate:
            fitted.append((cx, cy, float(radius)))

    if not fitted:
        return fallback

    expected_radius = max(obj.w, obj.h) / 2.0
    best = min(
        fitted,
        key=lambda item: (
            abs(item[2] - expected_radius) / max(expected_radius, 1.0) +
            math.hypot(item[0] - expected_cx, item[1] - expected_cy) /
            max(max(obj.w, obj.h), 1.0)))
    radius_error = abs(best[2] - expected_radius) / max(expected_radius, 1.0)
    if radius_error > RING_RADIUS_TOLERANCE:
        return fallback
    refined_cx, refined_cy = best[0], best[1]
    center_gap = math.hypot(best[0] - expected_cx, best[1] - expected_cy)
    return {
        'cx': refined_cx,
        'cy': refined_cy,
        'diameter': best[2] * 2.0,
        'refined': True,
        'residual': center_gap,
    }


class _FallbackCircleObject:
    """Object-shaped adapter so the existing overlay and tracker can be reused."""

    def __init__(self, cx, cy, radius, score, fast_lock=False):
        self.class_id = CLASS_BIG_CIRCLE
        self.score = score
        self.fast_lock = fast_lock
        self.x = int(cx - radius)
        self.y = int(cy - radius)
        self.w = max(1, int(radius * 2))
        self.h = max(1, int(radius * 2))


def find_classical_circle_fallback(img, tracker=None, frame_counter=0):
    """Recover a black target when YOLO has no usable candidate on white cloth."""
    if (not CLASSICAL_FALLBACK_ENABLED or
            frame_counter % CLASSICAL_FALLBACK_EVERY_N_FRAMES != 0 or
            not hasattr(img, 'find_circles')):
        return None

    try:
        width, height = img.width(), img.height()
        circles = img.find_circles(
            roi=(0, 0, width, height),
            threshold=CLASSICAL_HOUGH_THRESHOLD,
            x_margin=CLASSICAL_HOUGH_X_MARGIN,
            r_margin=CLASSICAL_HOUGH_R_MARGIN,
            r_min=CLASSICAL_MIN_RADIUS_PX,
            r_max=max(CLASSICAL_MIN_RADIUS_PX + 1,
                      int(min(width, height) * CLASSICAL_MAX_RADIUS_RATIO)),
            r_step=2)
    except Exception:
        return None
    if not circles:
        return None

    parsed = []
    for circle in circles:
        cx = _circle_attr(circle, 'x')
        cy = _circle_attr(circle, 'y')
        radius = _circle_attr(circle, 'r')
        if cx is None or cy is None or radius is None or radius <= 0:
            continue
        magnitude = _circle_attr(circle, 'magnitude')
        try:
            magnitude = float(magnitude) if magnitude is not None else 0.0
        except Exception:
            magnitude = 0.0
        parsed.append((float(cx), float(cy), float(radius), magnitude))
    if not parsed:
        return None

    # Prefer a concentric 50/30 cm pair; the smaller member is the 30 cm target.
    best_pair = None
    for i in range(len(parsed)):
        for j in range(i + 1, len(parsed)):
            first, second = parsed[i], parsed[j]
            outer, inner = (first, second) if first[2] >= second[2] else (second, first)
            ratio_error = abs(inner[2] / outer[2] - CLASSICAL_PAIR_RATIO)
            center_gap = math.hypot(outer[0] - inner[0], outer[1] - inner[1])
            if (ratio_error > CLASSICAL_PAIR_RATIO_TOLERANCE or
                    center_gap > outer[2] * CLASSICAL_PAIR_CENTER_RATIO):
                continue
            ratio_quality = 1.0 - ratio_error / CLASSICAL_PAIR_RATIO_TOLERANCE
            center_quality = 1.0 - (center_gap /
                                     (outer[2] * CLASSICAL_PAIR_CENTER_RATIO))
            geometry_quality = max(0.0, ratio_quality * 0.7 + center_quality * 0.3)
            pair_score = geometry_quality * 100000.0 + outer[3] + inner[3]
            if best_pair is None or pair_score > best_pair[0]:
                best_pair = (pair_score, geometry_quality, outer, inner)

    if best_pair is not None:
        _, geometry_quality, outer, inner = best_pair
        score = 0.42 + geometry_quality * 0.28
        candidate = _FallbackCircleObject(inner[0], inner[1], inner[2], score,
                                          fast_lock=geometry_quality >= 0.75)
    else:
        # A single circle may bridge a short loss only after a reliable lock.
        if tracker is None or tracker.state not in ('TRACK', 'LOST'):
            return None
        nearby = [item for item in parsed
                  if math.hypot(item[0] - tracker.x, item[1] - tracker.y)
                  <= TRACK_MAX_JUMP_PX * 2.0]
        if not nearby:
            return None
        strongest = max(nearby, key=lambda item: item[3])
        score = 0.32 + min(0.12, max(0.0, strongest[3]) / 25000.0)
        candidate = _FallbackCircleObject(strongest[0], strongest[1],
                                           strongest[2], score)

    # Once tracking has started, reject a floor artifact far from the predicted target.
    if (tracker is not None and tracker.state in ('TRACK', 'LOST') and
            math.hypot(candidate.x + candidate.w / 2.0 - tracker.x,
                       candidate.y + candidate.h / 2.0 - tracker.y) >
            TRACK_MAX_JUMP_PX * 2.0):
        return None
    return candidate


def build_target_candidate(img, obj, frame_counter):
    """Build a single measurement and bound the optional Hough workload."""
    if obj is None:
        return None
    if frame_counter % RING_REFINE_EVERY_N_FRAMES == 0:
        geometry = refine_concentric_circle(img, obj)
    else:
        geometry = _box_geometry(obj)
    geometry['score'] = float(obj.score)
    geometry['fast_lock'] = bool(getattr(obj, 'fast_lock', False))
    return geometry


class TargetTracker:
    """Confirmation, alpha-beta tracking, and bounded outage prediction."""

    def __init__(self):
        self.reset()

    def reset(self):
        self.state = 'SEARCH'
        self.x = self.y = self.vx = self.vy = 0.0
        self.diameter = self.score = self.residual = 0.0
        self.refined = False
        self.stable_hits = 0
        self.pending_hits = 0
        self.pending_x = self.pending_y = 0.0
        self.last_time_ms = None
        self.last_measurement_ms = None
        self.measurement_history = []
        self.reacquire_hits = 0
        self.reacquire_x = self.reacquire_y = 0.0

    def _median_measurement(self, candidate):
        """Propose a median measurement without accepting an outlier yet."""
        points = self.measurement_history + [(candidate['cx'], candidate['cy'])]
        xs = sorted(point[0] for point in points)
        ys = sorted(point[1] for point in points)
        middle = len(xs) // 2
        smoothed = dict(candidate)
        smoothed['cx'] = xs[middle]
        smoothed['cy'] = ys[middle]
        return smoothed

    def _remember_measurement(self, candidate):
        """Keep a short history so the filter removes jitter without lagging motion."""
        self.measurement_history.append((candidate['cx'], candidate['cy']))
        if len(self.measurement_history) > 3:
            del self.measurement_history[0]

    def _dt(self, now_ms):
        if self.last_time_ms is None:
            return 0.0
        return max(0.0, min((now_ms - self.last_time_ms) / 1000.0,
                            TRACK_MAX_DT_S))

    def _predict(self, dt):
        self.x += self.vx * dt
        self.y += self.vy * dt

    def _measured_output(self):
        return {
            'available': True,
            'cx': self.x,
            'cy': self.y,
            'diameter': int(max(0, self.diameter)),
            'flags': TARGET_FLAG_OBSERVED | TARGET_FLAG_LOCKED,
            'quality': calc_quality(self.score, self.diameter, self.refined,
                                    self.stable_hits, False, self.residual),
            'state': self.state,
        }

    def _prediction_output(self, age_ms):
        return {
            'available': True,
            'cx': self.x,
            'cy': self.y,
            'diameter': int(max(0, self.diameter)),
            'flags': TARGET_FLAG_PREDICTED,
            'quality': calc_quality(self.score, self.diameter, False, 0, True),
            'state': 'PREDICTED',
            'age_ms': age_ms,
        }

    def _confirm(self, candidate, now_ms):
        if (self.pending_hits and
                math.hypot(candidate['cx'] - self.pending_x,
                           candidate['cy'] - self.pending_y) <= TRACK_CONFIRM_JUMP_PX):
            self.pending_hits += 1
        else:
            self.pending_hits = 1
        self.pending_x = candidate['cx']
        self.pending_y = candidate['cy']

        required_hits = (1 if candidate.get('fast_lock') or
                         (candidate['refined'] and
                          candidate['score'] >= TRACK_FAST_LOCK_SCORE)
                         else TRACK_CONFIRM_HITS)
        if self.pending_hits < required_hits:
            self.state = 'CONFIRM'
            return None

        self.state = 'TRACK'
        self.x, self.y = candidate['cx'], candidate['cy']
        self.vx = self.vy = 0.0
        self.diameter = candidate['diameter']
        self.score = candidate['score']
        self.refined = candidate['refined']
        self.residual = candidate['residual']
        self.stable_hits = max(TRACK_CONFIRM_HITS, required_hits)
        self.last_measurement_ms = now_ms
        return self._measured_output()

    def _try_reacquire(self, candidate, now_ms, dt):
        """Adopt a consistent new location after a large but real image shift."""
        if (self.reacquire_hits and
                math.hypot(candidate['cx'] - self.reacquire_x,
                           candidate['cy'] - self.reacquire_y) <= TRACK_REACQUIRE_JUMP_PX):
            self.reacquire_hits += 1
        else:
            self.reacquire_hits = 1
        self.reacquire_x = candidate['cx']
        self.reacquire_y = candidate['cy']

        if self.reacquire_hits < TRACK_REACQUIRE_HITS:
            return None

        old_x, old_y = self.x, self.y
        self.x, self.y = candidate['cx'], candidate['cy']
        if dt > 0.001:
            self.vx = max(-500.0, min(500.0, (self.x - old_x) / dt))
            self.vy = max(-500.0, min(500.0, (self.y - old_y) / dt))
        else:
            self.vx = self.vy = 0.0
        self.diameter = candidate['diameter']
        self.score = candidate['score']
        self.refined = candidate['refined']
        self.residual = candidate['residual']
        self.stable_hits = TRACK_CONFIRM_HITS
        self.state = 'TRACK'
        self.last_measurement_ms = now_ms
        self.measurement_history = []
        self._remember_measurement(candidate)
        self.reacquire_hits = 0
        return self._measured_output()

    def update(self, candidate, now_ms):
        now_ms = int(now_ms)
        dt = self._dt(now_ms)
        if self.state in ('TRACK', 'LOST'):
            self._predict(dt)
        self.last_time_ms = now_ms

        if candidate is not None:
            raw_candidate = candidate
            if self.state in ('SEARCH', 'CONFIRM'):
                output = self._confirm(raw_candidate, now_ms)
                self._remember_measurement(raw_candidate)
                return output

            jump = math.hypot(raw_candidate['cx'] - self.x,
                              raw_candidate['cy'] - self.y)
            allowed_jump = TRACK_MAX_JUMP_PX + math.hypot(self.vx, self.vy) * dt
            if jump > allowed_jump:
                output = self._try_reacquire(raw_candidate, now_ms, dt)
                if output is not None:
                    return output
            else:
                candidate = self._median_measurement(raw_candidate)
                if dt > 0.001:
                    self.vx += TRACK_BETA * (candidate['cx'] - self.x) / dt
                    self.vy += TRACK_BETA * (candidate['cy'] - self.y) / dt
                self.x += TRACK_ALPHA * (candidate['cx'] - self.x)
                self.y += TRACK_ALPHA * (candidate['cy'] - self.y)
                self.diameter += TRACK_ALPHA * (candidate['diameter'] - self.diameter)
                self.score = candidate['score']
                self.refined = candidate['refined']
                self.residual = candidate['residual']
                self.stable_hits = min(255, self.stable_hits + 1)
                self.state = 'TRACK'
                self.last_measurement_ms = now_ms
                self._remember_measurement(raw_candidate)
                self.reacquire_hits = 0
                return self._measured_output()

        if self.state in ('TRACK', 'LOST') and self.last_measurement_ms is not None:
            age_ms = now_ms - self.last_measurement_ms
            if age_ms <= TRACK_PREDICT_MAX_MS:
                self.state = 'LOST'
                return self._prediction_output(age_ms)

        self.reset()
        return None


# ============================================================
# UI 绘制
# ============================================================

def draw_overlay(img, objs, detector, big_circle, session=None, track=None):
    """绘制检测结果和引导信息 (不影响通信)。"""
    w, h = img.width(), img.height()
    cx_img, cy_img = w // 2, h // 2

    # 中心十字线
    img.draw_line(cx_img - 20, cy_img, cx_img + 20, cy_img,
                  color=image.COLOR_GREEN, thickness=1)
    img.draw_line(cx_img, cy_img - 20, cx_img, cy_img + 20,
                  color=image.COLOR_GREEN, thickness=1)

    # 所有目标
    for obj in objs:
        if obj.class_id == CLASS_BIG_CIRCLE:
            continue
        color = (image.COLOR_RED if obj.class_id == CLASS_BIG_CIRCLE
                 else image.COLOR_BLUE)
        img.draw_rect(obj.x, obj.y, obj.w, obj.h,
                      color=color, thickness=2)
        label = f'{detector.labels[obj.class_id]}: {obj.score:.2f}'
        img.draw_string(obj.x, obj.y - 12, label, color=color)

    # 大圈高亮
    if track is not None and track['available']:
        bcx, bcy = int(track['cx']), int(track['cy'])
        diameter = max(1, int(track['diameter']))
        ox, oy = calc_offset_from_center(bcx, bcy, w, h)
        dist = ((BIG_CIRCLE_REAL_DIAMETER_CM * CAMERA_FOCAL_LENGTH_PX) /
                diameter) if diameter > 0 else -1
        observed = bool(track['flags'] & TARGET_FLAG_OBSERVED)
        color = image.COLOR_GREEN if observed else image.COLOR_YELLOW
        img.draw_rect(bcx - diameter // 2, bcy - diameter // 2,
                      diameter, diameter, color=color, thickness=2)
        img.draw_circle(bcx, bcy, 4, color=color, thickness=-1)
        img.draw_line(cx_img, cy_img, bcx, bcy, color=color, thickness=2)
        info = f"{track['state']} EX:{ox} EY:{oy} D:{dist:.0f}cm"
        img.draw_string(5, 5, info, color=color)
    elif big_circle is not None:
        img.draw_string(5, 5, "ACQUIRING", color=image.COLOR_YELLOW)
    else:
        img.draw_string(5, 5, "NO TARGET", color=image.COLOR_RED)

    # 会话状态条 (屏幕底部)
    if session is not None:
        if session['active']:
            mn = {1: 'TRACK', 2: 'DROP', 3: 'LAND'}.get(
                session['active_mode'], f"M{session['active_mode']}")
            bar = f"[{mn}] T{session['task_type']} M{session['mission_id']}"
            img.draw_string(5, h - 20, bar, color=image.COLOR_GREEN)
        else:
            img.draw_string(5, h - 20, "[IDLE]", color=image.COLOR_GRAY)


# ============================================================
# 主程序
# ============================================================

def main():
    print("[SYS] main() 开始...")

    # ---- 加载模型 ----
    print("[SYS] 加载模型...")
    model_path = "model_308819.mud"
    if not os.path.exists(model_path):
        model_path = "/root/models/maixhub/308819/model_308819.mud"
    detector = nn.YOLOv5(model=model_path)
    img_w = detector.input_width()
    img_h = detector.input_height()
    print(f"[SYS] 模型加载完成, 输入={img_w}x{img_h}")

    # ---- 初始化硬件 ----
    print("[SYS] 初始化摄像头...")
    cam = camera.Camera(img_w, img_h, detector.input_format())
    print("[SYS] 初始化显示屏...")
    dis = display.Display()
    print("[SYS] 硬件初始化完成")

    # ---- 初始化通信 (非阻塞) ----
    print("[SYS] 初始化 UART...")
    uart  = _MaixUART()
    servo = DropServo(
        pin_name=SERVO_PIN_NAME,
        pwm_id=SERVO_PWM_ID,
        freq=SERVO_FREQ_HZ,
        idle_angle=SERVO_IDLE_ANGLE,
        release_angle=SERVO_RELEASE_ANGLE,
        angle_min=SERVO_ANGLE_MIN,
        angle_max=SERVO_ANGLE_MAX,
        pulse_min_us=SERVO_PULSE_MIN_US,
        pulse_max_us=SERVO_PULSE_MAX_US,
        duration_ms=SERVO_DURATION_MS)

    # 舵机自测 (启动时短暂触发, 验证接线正确)
    # 注释掉下面两行可跳过自测:
    # print("[SERVO] 自测: 短暂触发舵机...")
    # servo.drop()

    parser = V2Parser(src_addr=ADDR_CAMERA)
    seq_mgr = SeqManager()
    dedup  = Deduplicator(window_ms=5000)
    tracker = TargetTracker()

    # ---- 会话状态 (dict 可在 handlers 中修改) ----
    session = {
        'active_mode': 0,
        'task_type':   0,
        'mission_id':  0,
        'mode_seq':    0,
        'active':      False,
    }

    # ---- 帧计数与时间 ----
    frame_counter = 0
    _t0 = time.ticks_ms()
    last_target_time = int(_t0) if hasattr(_t0, '__int__') else _t0
    start_time = last_target_time

    print(f"[SYS] 图像: {img_w}x{img_h}  中心: ({img_w//2}, {img_h//2})")
    print(f"[SYS] 地址: 0x{ADDR_CAMERA:02X} -> 飞控 0x{ADDR_FC:02X}")
    if not uart.ok():
        print("[SYS] *** 仅本地模式 (无串口通信) ***")
    print("[SYS] 进入主循环")

    # ---- 主循环 ----
    while not app.need_exit():
        # 1. 接收飞控指令 (仅当 UART 可用)
        if uart.ok():
            rx_data = uart.read()
            if rx_data:
                frames = parser.feed(rx_data)
                for f in frames:
                    if dedup.is_duplicate(f['src'], f['type'], f['seq']):
                        if f['flags'] & FLAG_ACK_REQUIRED:
                            first_result = dedup.result_for(
                                f['src'], f['type'], f['seq'])
                            if first_result is not None:
                                _send_command_ack(f, uart, seq_mgr,
                                                  first_result)
                        continue

                    ack_result = None
                    if f['type'] == TYPE_CAMERA_MODE:
                        ack_result = _handle_camera_mode(
                            f, uart, seq_mgr, session, parser, dedup, tracker)

                    elif f['type'] == TYPE_CAMERA_ACTION:
                        ack_result = _handle_camera_action(
                            f, uart, seq_mgr, session, servo)

                    if ack_result is not None:
                        dedup.remember_result(f['src'], f['type'], f['seq'],
                                             ack_result)

        # 2. 检测
        img = cam.read()
        objs = detector.detect(img, conf_th=CONF_THRESHOLD,
                               iou_th=IOU_THRESHOLD)
        big_circle = find_big_circle(objs)
        frame_counter = (frame_counter + 1) & 0xFFFF
        if big_circle is None:
            big_circle = find_classical_circle_fallback(
                img, tracker, frame_counter)

        # 3. 发送 CAMERA_TARGET (非 IDLE, 15–20Hz, 仅当 UART 可用)
        now = time.ticks_ms()
        now_int = int(now) if hasattr(now, '__int__') else now
        candidate = build_target_candidate(img, big_circle, frame_counter)
        track = tracker.update(candidate, now_int)
        elapsed = now_int - last_target_time
        if (uart.ok() and session['active_mode'] != 0
                and elapsed >= TARGET_PERIOD_MS):

            source_time_ms = (now_int - start_time) & 0xFFFFFFFF

            if track is not None and track['available']:
                ox, oy = calc_offset_from_center(track['cx'], track['cy'],
                                                  img_w, img_h)
                quality = track['quality']
                target_flags = track['flags']
                diameter = track['diameter']
            else:
                ox, oy = 0, 0
                quality = 0
                target_flags = 0x00
                diameter = 0

            target_frame = encode_frame(
                TYPE_CAMERA_TARGET, ADDR_CAMERA, ADDR_FC,
                seq_mgr.next(), 0,
                encode_target_payload(
                    target_flags, quality, ox, oy,
                    diameter, frame_counter, source_time_ms,
                    session['task_type'], session['mission_id'],
                    session['mode_seq']))

            uart.write(target_frame)
            last_target_time = now_int

        # 4. 显示
        draw_overlay(img, objs, detector, big_circle, session, track)
        dis.show(img)

    uart.deinit()
    print("[SYS] 已停止")


# ============================================================
# 消息处理
# ============================================================

def _handle_camera_mode(frame, uart, seq_mgr, session, parser=None, dedup=None,
                        tracker=None):
    """
    处理 CAMERA_MODE (0x90): 切换模式 + 更新会话 + 回复 ACK。

    规范:
      1. 停止旧模式输出，清空队列
      2. 写入 TaskType + MissionId + ModeSeq
      3. 完成切换后回复 ACK
      4. 同 Seq + RETRANSMISSION 只重发 ACK (去重器已处理)
    """
    payload = frame['payload']
    if len(payload) != 6:
        return _send_command_ack(frame, uart, seq_mgr,
                                 ACK_INVALID_PARAMETER)

    mode_val   = payload[0]
    task_type  = payload[1]
    mission_id = struct.unpack('>H', payload[2:4])[0]
    mode_seq   = frame['seq']

    if tracker is not None:
        tracker.reset()

    # 如果是 IDLE → 清空会话和所有状态
    if mode_val == 0:
        session['active_mode'] = 0
        session['task_type']   = 0
        session['mission_id']  = 0
        session['mode_seq']    = 0
        session['active']      = False
        if parser is not None:
            parser.reset()
        if dedup is not None:
            dedup.reset()
        print("[MODE] -> IDLE (会话+队列已清空)")
    else:
        session['active_mode'] = mode_val
        session['task_type']   = task_type
        session['mission_id']  = mission_id
        session['mode_seq']    = mode_seq
        session['active']      = True
        mode_names = {1: 'TRACK', 2: 'DROP_ALIGN', 3: 'LAND_ALIGN'}
        mn = mode_names.get(mode_val, f'UNKNOWN({mode_val})')
        print(f"[MODE] -> {mn} tt={task_type} mid={mission_id} ms={mode_seq}")

    # Reply only after the mode/session update has completed.
    return _send_command_ack(frame, uart, seq_mgr, ACK_ACCEPTED)


def _handle_camera_action(frame, uart, seq_mgr, session, servo):
    """
    处理 CAMERA_ACTION (0x92): 校验会话 → 执行投放 → 回报结果。

    规范:
      1. 校验 TaskType+MissionId+ModeSeq 匹配当前会话
      2. 不匹配 → 丢弃
      3. 匹配 → ACK → 驱动舵机投放 → 回报 RESULT
    """
    payload = frame['payload']
    if len(payload) < 8:
        return _send_command_ack(frame, uart, seq_mgr,
                                 ACK_INVALID_PARAMETER)

    action     = payload[0]
    action_id  = struct.unpack('>H', payload[2:4])[0]
    task_type  = payload[4]
    mission_id = struct.unpack('>H', payload[5:7])[0]
    mode_seq   = payload[7]

    # 会话匹配校验
    if not session['active']:
        print(f"[ACTION] 拒绝: 无活跃会话")
        return _send_command_ack(frame, uart, seq_mgr, ACK_NOT_ALLOWED)
    if (task_type != session['task_type'] or
            mission_id != session['mission_id'] or
            mode_seq != session['mode_seq']):
        print(f"[ACTION] 拒绝: 会话不匹配 "
              f"(got tt={task_type} mid={mission_id} ms={mode_seq}, "
              f"expected tt={session['task_type']} "
               f"mid={session['mission_id']} ms={session['mode_seq']})")
        return _send_command_ack(frame, uart, seq_mgr, ACK_NOT_ALLOWED)

    if action != ACTION_DROP:
        print(f"[ACTION] 拒绝: 不支持的动作 0x{action:02X}")
        return _send_command_ack(frame, uart, seq_mgr, ACK_NOT_SUPPORTED)

    print(f"[ACTION] DROP 执行 aid={action_id}")

    # 1. 回复 ACK
    ack_frame = encode_frame(
        TYPE_ACK, ADDR_CAMERA, frame['src'],
        seq_mgr.next(), 0,
        encode_ack_payload(TYPE_CAMERA_ACTION, frame['seq'], ACK_ACCEPTED))
    uart.write(ack_frame)

    # 2. 执行投放
    ok, msg = servo.drop()
    result_code = ACTION_COMPLETED if ok else ACTION_FAILED

    # 3. 回报结果
    result_frame = encode_frame(
        TYPE_CAMERA_ACTION_RESULT, ADDR_CAMERA, frame['src'],
        seq_mgr.next(), 0,
        encode_action_result_payload(
            action, result_code, action_id,
            int(time.ticks_ms()) & 0xFFFFFFFF,
            task_type, mission_id, mode_seq))
    uart.write(result_frame)
    print(f"[ACTION] RESULT: {msg}")
    return ACK_ACCEPTED


# ============================================================
# 入口
# ============================================================

if __name__ == "__main__":
    main()
