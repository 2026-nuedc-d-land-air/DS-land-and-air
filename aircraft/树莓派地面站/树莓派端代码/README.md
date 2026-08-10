# 2026 D题 V2.3 树莓派地面站

本目录只以 `说明文档/D题_通用通信与接口规范_v2.3.docx` 为通信依据。正式运行不解析旧 `AA BB` 二维码帧，不启动实时图传，不提供启动、停止、降落、投放或参数修改控件，也不依赖网络或外网。

> 当前实现遵循 `说明文档/D题_通用通信与接口规范_v2.3.docx`。它解析小车广播的 `0x85 MAINTENANCE_RESET`，仅在 Src=`0x30`、Dst=`0x10`、8 B 载荷与非零新 `ResetId` 均合法时清除旧校准/任务状态；重复的三帧重传不会再次改变状态。复位后仍需新鲜 IDLE/未校准数据和人工再次确认，程序绝不自动发送 `CALIBRATION_SET`。

## 功能边界

- 通过 LoRa USB-TTL 接收 V2.3 `AA 55` 字节流，串口固定 `115200 8N1`、无流控；
- 在组委会 400×500 cm 净版场地图上显示飞机参考点和小车平台同心圆十字中心；
- 显示两端坐标/yaw、链路年龄、校准 ID、任务类型、阶段、错误码和可从协议确定的倒计时；
- 任务阶段严格只接收；
- 唯一发送消息为起飞前 `CALIBRATION_SET`，且只能经人工确认并在第 7.1 节维护时隙发送一次；
- 收到已校准非零 ID 的 `CAR_POSE`（READY）、`CAR_TASK_REQUEST`、`MISSION_STATUS`、非 IDLE 飞机遥测或 `CAR_RUNNING=1` 后，本进程的发送路径永久熔断。

没有单独的发送线程。串口接收线程只保留一个受前置条件、帧类型和 30–50 ms 时间窗三重检查的校准写入口；熔断后排队动作和未完成校准都会取消。

## 文件

- `ground_station_protocol.py`：V2.3 帧、CRC16/CCITT-FALSE、消息编解码、100 ms 流式重同步；
- `ground_station_serial.py`：CH340 独占串口、DTR/RTS 禁用、断线重连、消息分发、校准时隙和发送熔断；
- `ground_station_ui_core.py`：新鲜度、校准候选/稳定度、场地投影和倒计时纯逻辑；
- `ground_station_ui.py`：Qt 6 离线监视界面和唯一的“起飞前校准”维护区；
- `assets/field_map.png`：组委会净版比赛场地图；标注版仅作为尺寸标定依据，不部署；
- `stage5_serial_probe.py`：只读 V2.3 串口探针；
- `run_ground_station_ui.sh`：本地图形会话启动和异常退出重启；
- `install_ground_station_ui.sh`：当前用户范围安装与离线桌面自启动；
- `test_ground_station_*.py`：固定向量、错误恢复、时隙、熔断和 UI 策略测试。

## 运行与测试

树莓派依赖仅为：

```bash
sudo apt-get install python3-pyqt6 python3-serial
```

开发回归：

```bash
cd /path/to/树莓派端代码
python3 -m unittest -v \
  test_ground_station_protocol.py \
  test_ground_station_serial.py \
  test_ground_station_ui_core.py
python3 -m py_compile \
  ground_station_protocol.py ground_station_serial.py \
  ground_station_ui_core.py ground_station_ui.py stage5_serial_probe.py
```

窗口运行：

```bash
./run_ground_station_ui.sh --windowed --verbose
```

全屏正式运行：

```bash
./run_ground_station_ui.sh
```

目标树莓派的 Debian PyQt6 未安装 Wayland 平台插件；启动脚本在本地桌面存在
`DISPLAY` 时使用已验证的 XWayland/XCB 后端。这只影响 Qt 窗口输出，不引入
VLC、RTSP 或网络依赖。

只读串口检查（必须先退出 UI，避免争用独占串口）：

```bash
python3 stage5_serial_probe.py --duration 15
```

安装自启动：

```bash
chmod +x install_ground_station_ui.sh run_ground_station_ui.sh
./install_ground_station_ui.sh
```

安装器会替换本项目已知的旧 RTSP 自启动条目和旧地面站监管程序，不卸载系统软件包，也不修改网络配置。

## 起飞前校准闭环

1. 界面同时看到新鲜 `FLIGHT_TELEMETRY` 和 `CoordinateFrame=FIELD_GLOBAL`、`POSITION_VALID=1`、`CALIBRATED=0`、`CalibrationId=0`、`CAR_RUNNING=0` 的 `CAR_POSE`。
2. 连续 5 个不同源时间快照的 Δx/Δy 二维极差不超过默认 5 cm 后，人工核对并确认显示值。
3. 接收线程等待下一帧 `CAR_POSE Seq mod 5=2`，目标在接收完成后 35 ms 写出；若调度超过 50 ms，本轮不发，等待下一维护槽。
4. 一帧 `CALIBRATION_SET` 发出后等待小车 ACK；超时不自动重发。
5. ACK 接受后继续等待相同非零 `CalibrationId`、`CALIBRATED=1`、坐标系为 `FIELD_GLOBAL` 的后续 `CAR_POSE`。
6. 闭环完成即进入 READY，只收闩锁永久生效。

## 当前默认值与实测前提

以下不是 V2.3 冻结字段，需要首轮台架/场地实测后确认：

| 参数 | 当前默认值 | 用途 |
|---|---:|---|
| 飞机遥测 UI 新鲜阈值 | 1.2 s | 兼容规范 2 Hz 遥测及少量抖动 |
| 校准稳定样本数 | 5 | 人工确认前连续快照数 |
| Δ 二维极差阈值 | 5 cm | 判断对准读数是否稳定 |
| 确认到发送的坐标变化容差 | 5 cm | 防止确认后平台/飞机移动 |
| 维护槽目标发送时刻 | 35 ms | 位于规范 30–50 ms 窗口内 |
| ACK 等待 | 1.0 s | 不触发自动重发 |
| 校准位姿等待 | 2.0 s | ACK 后等待闭环 |
| 场地轴显示 | X=0..500 cm，Y=0..400 cm | +X 向图上、+Y 向图左 |

地图以右下角为 `(0,0)`，按 `+X` 向上、`+Y` 向左显示。尺寸标注图中的
关键点坐标为：`H=(112.5,287.5)`、`A=(200,250)`、`B=(350,250)`、
`C=(350,100)`、`D=(200,100)` cm。H 的 112.5 cm 来自边界至外圆边缘
75 cm 加外圆半径 37.5 cm。

飞机上报 X/Y 的导航原点在 H 圆心，地图原点则在场地右下角。地面站仅在地图和
飞机文字位置显示前添加 `X_MAP=X_RAW+113`、`Y_MAP=Y_RAW+288`。

小车未校准时以 A 点为本地零点，地图/文字显示添加 `(+200,+250) cm`；校准成功
后，小车已应用 `Δx/Δy` 并与飞机共享 H 原点的控制坐标，此时同样添加 `(+113,+288) cm`。
校准候选和 `CALIBRATION_SET` 始终使用未平移的原始协议 X/Y；Z 和 yaw 不施加显示偏移。

其中飞机新鲜阈值、稳定度和发送容差可通过启动参数调整，但界面在任务期没有参数修改控件。

日志位于：

```text
~/.local/state/d-task-ground-station/ground_station_ui.log
```
