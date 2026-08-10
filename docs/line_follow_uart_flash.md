# 循迹固件烧录、串口与冻结日志

> **活动模式：** `COMPETITION_LINE_FOLLOW`。默认镜像为 `LineFollowMissionDebug`；后轮台架镜像不能替代本流程。

## 1. 默认烧录：J-Link SWD

连接 SWD、共地并保证车轮安全后，在项目根目录执行：

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File .\scripts\jlink_flash.ps1 `
  -Configuration LineFollowMissionDebug -SwdSpeedKhz 50
```

烧录成功的必要条件：

1. 配置名输出为 `LineFollowMissionDebug`；
2. `.isr_vector`、`.text`、`.rodata`、`.data` 等已写入段均显示校验通过；
3. 脚本结束时已 reset/go；
4. 打开 COM13 后能看到启动信息或响应 `P`。

若 SWD 不稳定，先检查公共地、供电、SWDIO/SWCLK/NRST 和目标复位，再降低 `-SwdSpeedKhz`；不要在未验证目标芯片时反复全片擦除。

## 2. COM13 证据链

诊断串口为 `USART1 PA9/PA10`，115200 8N1。所有现场运行都按以下顺序保存：

```powershell
# 读取当前固件参数和状态（发送 P）
powershell -NoProfile -ExecutionPolicy Bypass -File .\scripts\query_line_follow_status.ps1 -PortName COM13

# 导出本轮冻结日志（发送 F）；脚本应保存原始输出到 logs
powershell -NoProfile -ExecutionPolicy Bypass -File .\scripts\dump_line_follow_frozen_log.ps1 -PortName COM13

# 对已保存原始日志做摘要分析
powershell -NoProfile -ExecutionPolicy Bypass -File .\scripts\analyze_line_follow_frozen_log.ps1 `
  -Path .\logs\<timestamp>.log
```

`P`、`F`、`H` 的含义：

| 命令 | 证明什么 | 不能证明什么 |
| --- | --- | --- |
| `P` | 当前烧录镜像的参数、任务和安全状态；重点核对 20 s、速度、丢线与 A 返回阈值。 | 一次真实赛道运行成功。 |
| `F` | 最近一轮冻结的过程/终止数据。 | 无人机侧是否也正确收到同一帧。 |
| `H` | LoRa/阶段/位姿计数和匹配状态。 | 小车已根据合法阶段实际加速。 |

**验收原则：** 小车通信成功必须有 COM13 原始串口；无人机通信成功还必须有 ESP32 原始串口或其冻结日志。地面站画面只能辅助定位。

## 3. 烧录后必须核对的 `P` 字段

至少记录以下当前参数：

```text
start_direct_gate_ms=20000
task1_speed_mm_s=150
task2_speed_mm_s=170
task2_fast_unlock=PLATFORM_TAKEOFF_OR_LOCAL_D
task2_d_speed_latched=0_or_1
task2_fast_entry_base_slew_cps_per_20ms=70
task1_d_to_a_stability_after_mm=4650
task1_d_to_a_stability_speed_mm_s=420
post_coord_speed_profile_mm_s=600/420/360/260/180/600
lost_search_differential_cps=1400
lost_reacquire_samples=5
a_return_encoder_approach_mm=8800
a_return_heading_approach_mm=8000
a_return_heading_window_tenths=300
a_return_approach_speed_mm_s=170
task2_a_return_approach_mm=8000
a_return_stop_policy=TASK2_ENCODER_8000_PREBRAKE_TASK1_HEADING_8000_8800_GRAY7
```

若任何一项缺失、不同或 `P` 没有响应，停止现场跑题：确认是否误烧录了旧构建目录/错误预设，以及 COM 端口是否被其他程序占用。

## 4. `F` 日志的最低检查项

分析冻结日志时，至少检查：

- `terminal_reason`：正常 A 返回、丢线超时、运行守护、人工急停或其他；
- `run_distance_mm`：最终 A 判定前应达到 8800 mm；
- 灰度原始/稳定掩码：B/C/D 宽点不应被作为终点；
- `a_heading_err`、`a_heading_near_start`：确认 D→A 弧线的起始航向回归；
- 丢线方向、恢复样本和速度：确认短丢线恢复不是随机转向；
- 任务、MissionId、空中阶段与速度门：判断加速是否有合法授权。
- `task2_fast_base`、`task2_fast_ramp_active`：任务二首次快包络切入时，确认基础速度的上升斜率受限，且曲线/丢线/A 接近降速未被该门阻塞。
- `task1_d_to_a_cap=1`：任务一从 4650 mm 进入 D→A 稳态段；其后的直线基础目标不得超过 420 mm/s，任务二记录应始终为 `0`。

不要覆盖旧 `.log`。每次导出以时间戳命名，并和烧录输出、场地条件、ESP32 日志放在同一轮目录。

## 5. UART ROM Bootloader（仅故障恢复）

只有 SWD 不可用且已经核对芯片 BOOT 配置时，才用 ROM 串口下载。完成后必须恢复正常 BOOT 状态，再通过 SWD 或 COM13 的 `P` 确认运行镜像。ROM 下载是恢复通道，**不是**日常验收或替代 J-Link 的证据。

## 6. 常见异常

| 现象 | 排查顺序 |
| --- | --- |
| COM13 无法打开 | 断开其他串口工具，确认 USB 枚举与波特率；重插后记录连接时间。 |
| `P` 无回应 | 确认是 USART1 而非 JY901/LoRa，查看 reset 后启动信息，再核对镜像。 |
| `F` 中止原因不明 | 不重启、不覆盖日志；先导出原文，再用分析脚本和本轮场地备注对照。 |
| 无人机端串口读不到 | 先执行其冻结日志导出/保留当前输出，再查供电、端口和飞控日志；不能以小车 H 替代。 |
| J-Link verify 失败 | 检查供电/共地/复位/SWD 速率；确认目标未被其他调试器占用。 |

运行流程见 [`D题任务一二雷达协同联调.md`](D题任务一二雷达协同联调.md)，验收矩阵见 [`D题_V23_联调与计分矩阵.md`](D题_V23_联调与计分矩阵.md)。
