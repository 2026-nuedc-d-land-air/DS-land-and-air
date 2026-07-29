# D题 Agent 修改提示词 v2.1

请将同名通信规范一并发给对应 Agent；该规范是唯一接口依据。

## B.1 飞控 STM32F407 Agent

先完整阅读《D题_通用通信与接口规范_v2.1》，它是唯一通信依据；不要沿用旧二维码/图传任务假设。改动前先阅读当前工程，保留无关功能和既有安全保护。完成后给出修改文件清单、关键协议测试、未确定的实测参数。

你的身份是无人机飞控 STM32F407。仅修改飞控工程。重点文件：User_Task.c/.h、my_uart.c/.h、my_send_test.c/.h、mission_command.c/.h，以及必要的 PID/控制接口。实现第 6.2 节扩展 LX 帧：接收 CAR_POSE 与 MISSION_REQUEST，向 ESP32回 MISSION_RESPONSE / MISSION_STATUS；保留旧 8-byte私有帧兼容。把旧二维码搜索状态机替换为‘预检→150cm起飞稳定3s→拦截→伴飞→抛投 或 动态着陆→平台停留5s→返航’两条任务。飞控是唯一任务接受者：检查 CH6、MID360/激光、小车位姿新鲜度、CalibrationId；ESP写串口成功不是接受。用小车 FIELD_GLOBAL 位姿作主目标，用相机 CAMERA_TARGET 只做末段纠偏；相机偏差按无人机 yaw 旋转到世界系。无人机 yaw 跟随小车 yaw。实现平台15cm高度补偿和末端约16cm激光落地目标，下降中按第8节处理丢车/丢相机。相机改为V2.1 MODE/TARGET/ACTION/RESULT，不得再使用二维码或旧0x50/0x60语义。不要让飞控解析LoRa。

## B.2 机载 ESP32 Agent

先完整阅读《D题_通用通信与接口规范_v2.1》，它是唯一通信依据；不要沿用旧二维码/图传任务假设。改动前先阅读当前工程，保留无关功能和既有安全保护。完成后给出修改文件清单、关键协议测试、未确定的实测参数。

你的身份是机载 ESP32 协议代理。重点文件：main.cpp、main.h、uart_lora.cpp/.h、uart_lx.cpp/.h，及 protocol_v2 模块。实现V2.1的0x80 CAR_POSE、0x81 CAR_TASK_REQUEST、0x82 MISSION_STATUS、0x84 MISSION_ABORT。所有收到的合法CAR_POSE立即用第6.2节扩展LX转给STM32；任务请求转给STM32后，必须等待STM32的MISSION_RESPONSE，才在LoRa回应窗口向小车发ACK。移除自由运行的5Hz遥测：按第7节以CAR_POSE为时基，Seq mod5=0时最多发一帧2Hz广播遥测；ACK和任务状态优先。广播Dst必须是0x10。保持飞控有线位置帧新鲜度保护，超时则不发遥测也不接受任务。不能自行决定起飞/投放/降落。必须实现CRC、长度、目的地址、5s去重和发送互斥；所有本题新帧不得落入旧AA BB二维码通道。

## B.3 小车端（树莓派4 + MCU）Agent

先完整阅读《D题_通用通信与接口规范_v2.1》，它是唯一通信依据；不要沿用旧二维码/图传任务假设。改动前先阅读当前工程，保留无关功能和既有安全保护。完成后给出修改文件清单、关键协议测试、未确定的实测参数。

你的身份是小车端，需同时处理树莓派4定位程序与小车MCU无线/按键程序。树莓派以115200 UART每100ms向MCU发送V2.1 CAR_POSE；先把SLAM雷达原点换算为平台同心圆十字中心，再按本次Δx/Δy转换为FIELD_GLOBAL。姿态：前+X、左+Y、yaw逆时针为正，单位cm/0.1°；可计算并填Vx/Vy，否则清速度有效位。MCU是唯一LoRa发射者，按第7节10Hz车端时隙广播CAR_POSE。两个物理按键分别生成CAR_TASK_REQUEST(抛投/动态起降)，同Seq最多重发3次，不能因ACK重复触发或让小车停车。实现地面站起飞前CALIBRATION_SET：MCU只在车未运行时转发给Pi；Pi内存应用后回ACK，MCU再对地面站ACK。校准重启失效，任务中不得改。任务中小车继续循线一整圈，不依赖地面站控制。

## B.4 MaixCam Pro 相机 Agent

先完整阅读《D题_通用通信与接口规范_v2.1》，它是唯一通信依据；不要沿用旧二维码/图传任务假设。改动前先阅读当前工程，保留无关功能和既有安全保护。完成后给出修改文件清单、关键协议测试、未确定的实测参数。

你的身份是MaixCam Pro下视视觉与投放执行端。重点文件：main.py、main_a16_height130_gain_forward10_release_burst.py。删除本题流程中红色X、色块、二维码和AprilTag作为目标的依赖；只识别60×60cm平台中心的黑色30/50cm同心圆和十字，并排除黑色循迹线。把UART改为第4/5.5节V2.1流式帧：接收CAMERA_MODE和CAMERA_ACTION，发送CAMERA_TARGET与CAMERA_ACTION_RESULT，115200、8N1、CRC16。TARGET需15–20Hz给出质量、目标有效位和机体系ErrXcm/ErrYcm（前+X、左+Y）；利用50cm外环尺度计算cm并完成镜头到机体系符号标定。相机只在飞控MODE非IDLE时检测；投放舵机只能在收到一次性ACTION/DROP后执行，完成才回RESULT，不能自行依据画面稳定触发。保留非阻塞舵机、超时、串口重同步和本地调试显示；不做LoRa和RTSP图传。

## B.5 树莓派地面站 Agent

先完整阅读《D题_通用通信与接口规范_v2.1》，它是唯一通信依据；不要沿用旧二维码/图传任务假设。改动前先阅读当前工程，保留无关功能和既有安全保护。完成后给出修改文件清单、关键协议测试、未确定的实测参数。

你的身份是树莓派地面站。使用LoRa USB-TTL的V2.1解析器，显示400×500cm场地上的飞机和小车平台中心、坐标/yaw、链路新鲜度、校准ID、任务类型/阶段、错误与倒计时；任务阶段仅接收。移除RTSP/VLC/实时图传及启动、停止、降落控制按钮。仅提供‘起飞前校准’维护页面：同时看到新鲜FLIGHT_TELEMETRY和未校准CAR_POSE时，计算并展示Δx/Δy；人工确认后只在第7.1节Seq mod5=2窗口向小车发CALIBRATION_SET，等待ACK和后续CALIBRATED CAR_POSE。进入READY或收到CAR_TASK_REQUEST后，彻底禁用发送线程和全部控制控件。不能在任务中下发修改参数/代码/控制命令，也不依赖外网。
