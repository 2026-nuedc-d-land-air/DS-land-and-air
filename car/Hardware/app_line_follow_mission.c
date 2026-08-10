/**
 ******************************************************************************
 * @file    app_line_follow_mission.c
 * @brief   Field-ready line-following run: task keys, radar pose gate,
 *          gray/direct-JY901 observer, and four-wheel actuation.
 *
 * Front encoders close the speed loop. The rear TB6612 receives the matching
 * signed raw command only; it is a four-wheel open-loop follower until its
 * TIM4/TIM8 encoder feedback is calibrated. The directly wired JY901 yaw is
 * the bounded yaw/rate feedback source; gray remains the primary path-error
 * source and encoder path distance supplies the primary progress distance.
 * UART4 radar pose is an auxiliary coordinate/coordination signal only: it
 * never supplies steering and never stops the vehicle by itself.
 ******************************************************************************
 */

#include "app_line_follow_mission.h"

#include "app_line_observer.h"
#include "bsp_aux_tb6612.h"
#include "bsp_car_pose_link.h"
#include "bsp_diag_uart.h"
#include "bsp_encoder.h"
#include "bsp_four_wheel_direction.h"
#include "bsp_motor.h"
#include "bsp_gyro_wit.h"
#include "bsp_robot_uart.h"
#include "bsp_v22_protocol.h"
#include "speed_ladrc.h"

/* Field wiring for the mission indicators.  All three lamps are active-low,
 * matching the existing board LED convention.  PG14 is initialized and kept
 * off as the spare lamp; PG15 and PE1 carry the task/reset prompts below. */
#define LINE_LED_GREEN_PORT                 GPIOG
#define LINE_LED_GREEN_PIN                  GPIO_Pin_15
#define LINE_LED_SPARE_PORT                 GPIOG
#define LINE_LED_SPARE_PIN                  GPIO_Pin_14
#define LINE_LED_YELLOW_PORT                GPIOE
#define LINE_LED_YELLOW_PIN                 GPIO_Pin_1

#define LINE_CONTROL_PERIOD_MS               20U
#define LINE_LOG_PERIOD_MS                  100U
#define LINE_LIVE_RECORD_LOG_ENABLE            0U
#define LINE_BUTTON_DEBOUNCE_MS              50U
#define LINE_START_GYRO_WAIT_MS            5000U
#define LINE_START_COORDINATION_WAIT_MS    5000U
#define LINE_TASK_START_DIRECT_GATE_MS    20000U
#define LINE_START_CENTER_MASK              0x18U
#define LINE_BUTTON_GREEN_PULSE_MS           300U
#define LINE_RESET_YELLOW_PULSE_MS           500U
#define LINE_LEAVE_A_TIMEOUT_MS            3000U
#define LINE_LOST_TIMEOUT_MS              12000U
/* The all-black marker is already debounced for two 20 ms observer samples.
 * Do not add another long dwell here or the car can cross A at 600 mm/s. */
#define LINE_COMPLETE_MARK_MS                20U
#define LINE_RADIO_HEARTBEAT_PERIOD_MS      500U
#define LINE_RADIO_POSE_PERIOD_MS            100U
#define LINE_RADIO_POSE_FRESH_MS             250U
#define LINE_RADIO_CAR_SLOT_MS                100U
#define LINE_RADIO_CAR_SLOT_WINDOW_MS          25U
#define LINE_RADIO_RX_POLL_LIMIT              128U
#define LINE_TASK_REQUEST_RETRY_COUNT           3U
#define LINE_TASK_POSE_PREAMBLE_COUNT           3U
#define LINE_TASK_ACK_WINDOW_MS               350U
#define LINE_MISSION_ABORT_RETRY_COUNT          3U
#define LINE_ABORT_ACK_WINDOW_MS              350U
#define LINE_TASK_REQUEST_FLAGS              0x01U
#define LINE_FLIGHT_TELEMETRY_PAYLOAD_LENGTH   24U
#define LINE_MISSION_STATUS_PAYLOAD_LENGTH     12U
#define LINE_CALIBRATION_PAYLOAD_LENGTH         12U
#define LINE_MAINTENANCE_PAYLOAD_LENGTH          8U
#define LINE_ACK_PAYLOAD_LENGTH                  4U
#define LINE_MAINTENANCE_BROADCAST_COUNT       3U
#define LINE_CALIBRATION_DEDUP_MS            5000U
#define LINE_CALIBRATION_RECORD_COUNT           4U
#define LINE_RADIO_ACK_QUEUE_COUNT              4U
#define LINE_RUN_WATCHDOG_MS              90000U
#define LINE_NO_PROGRESS_WATCHDOG_MS      20000U
#define LINE_TASK2_B_DEADLINE_MS          15000U
#define LINE_GYRO_RUN_STALE_MS              250U
#define LINE_ENCODER_LIVE_AFTER_MS         1000U
#define LINE_ENCODER_LIVE_CMD_PERCENT        35
#define LINE_ENCODER_LIVE_MIN_CPS            200
#define LINE_MOTION_START_MIN_CPS            200
/* B is about 151.5 cm from A.  The progress gate is intentionally a small
 * distance beyond that point: task 1 must never unlock its post-B envelope
 * while the platform is still in the A-B companion-flight segment. */
#define LINE_RADAR_B_REACH_DISTANCE_CM       155L
#define LINE_RADAR_A_PREPARE_RADIUS_CM        45L
#define LINE_RADAR_A_STOP_RADIUS_CM           20L
#define LINE_RADAR_MAX_STEP_CM                 40L
#define LINE_RADAR_B_CONFIRM_SAMPLES            2U
#define LINE_FLIGHT_STATE_STALE_MS           1500U
#define LINE_B_ODOMETRY_DISTANCE_MM          1550L
#define LINE_D_ODOMETRY_DISTANCE_MM         4650L
#define LINE_D_YAW_TRAVEL_TENTHS            1500L
/* The two accepted A completions were at 9380 mm and 10017 mm.  On the D-A
 * arc, accumulated yaw can be only -2542 at the first seven-head A pattern,
 * so yaw is diagnostic rather than a final-stop gate.  Encoder distance keeps
 * the start-A marker out of this window. */
#define LINE_A_RETURN_MIN_DISTANCE_MM        8800L
/* Begin encoder-only braking before the shortest observed completion; the
 * gray A marker remains the final location authority. */
#define LINE_A_RETURN_APPROACH_DISTANCE_MM   8800L
#define LINE_A_RETURN_APPROACH_SPEED_MM_S     170L
/* D-to-A is a shallow arc.  Once the car has travelled enough to be on that
 * return leg and yaw has come back near its launch heading, reduce speed
 * before the narrow A stripe instead of waiting for a raw 0xFF sample. */
#define LINE_A_RETURN_HEADING_APPROACH_DISTANCE_MM 8000L
#define LINE_A_RETURN_HEADING_WINDOW_TENTHS    300
/* Task two's 2026-08-01 return trace began to alternate large gray/yaw
 * corrections from 8189 mm while its heading had not yet entered the common
 * start-heading window.  Keep task one's heading-qualified policy unchanged,
 * but make task two enter the same 170 mm/s A-approach cap by encoder path at
 * 8000 mm.  This is speed limiting only; all final A stop guards remain
 * shared and unchanged. */
#define LINE_TASK2_A_RETURN_APPROACH_DISTANCE_MM 8000L
/* The task-one frozen trace lost the tape at 5337 mm, immediately after the
 * encoder-defined D-to-A arc began.  It had not reached either A-braking
 * gate, so cap only task one's post-D straight request here; curve/lost/A
 * reductions remain lower-priority caps and task two is excluded. */
#define LINE_TASK1_D_TO_A_STABILITY_DISTANCE_MM LINE_D_ODOMETRY_DISTANCE_MM
#define LINE_TASK1_D_TO_A_STABILITY_SPEED_MM    420L

#ifndef LINE_TASK1_COOP_SPEED_MM_S
#define LINE_TASK1_COOP_SPEED_MM_S           150L
#endif
#ifndef LINE_TASK2_COOP_SPEED_MM_S
#define LINE_TASK2_COOP_SPEED_MM_S           170L
#endif
#ifndef LINE_REQUIRE_GYRO_FOR_START
#define LINE_REQUIRE_GYRO_FOR_START              1
#endif
#ifndef LINE_REQUIRE_CALIBRATED_POSE_FOR_TASK_START
#define LINE_REQUIRE_CALIBRATED_POSE_FOR_TASK_START 0
#endif
#ifndef LINE_ENABLE_UART_MANUAL_STOP
#define LINE_ENABLE_UART_MANUAL_STOP             0
#endif
#ifndef LINE_CAR_POSE_PLATFORM_FORWARD_OFFSET_CM
/* Conservative default: the air-side/Pi chain owns its own mounting offset.
 * Keep the car egress unshifted unless a measured, single-owner correction is
 * explicitly configured for a later field build. */
#define LINE_CAR_POSE_PLATFORM_FORWARD_OFFSET_CM 0L
#endif

/* Each task leaves A at its own cooperative straight-line speed. The fast
 * envelope is unlocked only by the verified aircraft stage, never by an ACK
 * or by the local D diagnostic gate. */
#define LINE_COOP_CURVE_SPEED_MM_S           115L
#define LINE_COOP_HARD_CURVE_SPEED_MM_S      110L
#define LINE_COOP_TIGHT_CURVE_SPEED_MM_S     100L
#define LINE_COOP_LOST_HOLD_SPEED_MM_S        90L
#define LINE_COOP_WIDE_SPEED_MM_S            110L
#define LINE_POST_COORD_BASE_SPEED_MM_S      600L
/* The 2026-08-01 frozen run lost the tape after the fast profile entered a
 * mild curve (|error|=200) at 500 mm/s. Preserve the 600 mm/s straight-line
 * envelope, but give the gray controller more path margin off-centre. */
#define LINE_POST_COORD_CURVE_SPEED_MM_S     420L
#define LINE_POST_COORD_HARD_CURVE_SPEED_MM_S 360L
#define LINE_POST_COORD_TIGHT_CURVE_SPEED_MM_S 260L
/* A search is recovery, not a racing segment. Keep it slow enough that the
 * selected turning direction can be cancelled by a centre capture. */
#define LINE_POST_COORD_LOST_HOLD_SPEED_MM_S 180L
#define LINE_POST_COORD_WIDE_SPEED_MM_S      600L
/* Task two can receive PLATFORM_TAKEOFF while it is still in the stable
 * cooperative envelope.  Limit only this first upward base-speed transfer;
 * the normal 160 cps control slew remains responsible for all other phases. */
#define LINE_TASK2_FAST_ENTRY_BASE_SLEW_CPS_PER_SAMPLE 70

#define LINE_SPEED_B0                       800.0f
#define LINE_SPEED_WC                         8.0f
#define LINE_SPEED_WO                        40.0f
#define LINE_SPEED_OUT_REVERSE_MIN          -35.0f
#define LINE_SPEED_OUT_MAX                   45.0f
/* The latest task-2 run reaches B in about 13 s and the straight sections
 * are output-limited at 45%.  Give only dynamic landing a 10-point reserve;
 * LADRC still closes the wheel-speed loop and may command less. */
#define LINE_TASK2_SPEED_OUT_MAX             55.0f
#define LINE_SPEED_OUT_STEP                   6.0f
#define LINE_REAR_FOLLOW_GAIN_PERCENT          120L
#define LINE_REAR_FOLLOW_FORWARD_MIN_PERCENT     20
#define LINE_REAR_LEFT_FORWARD_TRIM_PERCENT      (-5L)
#define LINE_REAR_RIGHT_FORWARD_TRIM_PERCENT     30L
#define LINE_REAR_LEFT_STARTUP_TRIM_PERCENT      20L
#define LINE_REAR_RIGHT_STARTUP_TRIM_PERCENT     20L
#define LINE_TARGET_REVERSE_MIN_CPS         -800
#define LINE_TARGET_MAX_CPS                 4200
#define LINE_TARGET_BASE_SLEW_CPS_PER_SAMPLE 160
#define LINE_TARGET_DIFFERENTIAL_SLEW_CPS_PER_SAMPLE 300
#define LINE_LOST_DIRECTION_HISTORY_COUNT      4U
#define LINE_LOST_DIRECTION_ERROR_X100        200
#define LINE_LOST_DIRECTION_MIN_CPS           500
#define LINE_LOST_SEARCH_DIFFERENTIAL_CPS    1400
/* LOST_HOLD keeps the last reliable gray steering sign, while this bounded
 * JY901 term resists an external yaw kick and prevents that fixed search turn
 * from winding the vehicle away from its last valid line tangent. */
#define LINE_LOST_YAW_RECOVERY_GAIN             2L
#define LINE_LOST_YAW_RECOVERY_LIMIT_CPS       600
#define LINE_LOST_COMBINED_DIFF_LIMIT_CPS     1600
#define LINE_LOST_SOURCE_NONE                    0U
#define LINE_LOST_SOURCE_LAST_EDGE               1U
#define LINE_LOST_SOURCE_HISTORY                 2U
#define LINE_LOST_REACQUIRE_SAMPLES             5U
#define LINE_LOST_REACQUIRE_ERROR_X100        300
#define LINE_A_RAW_FULL_MASK                  0xFFU
#define LINE_A_RETURN_MIN_STABLE_BITS            7U
#define LINE_VELOCITY_WINDOW_COUNT             4U
#define LINE_FROZEN_RECORD_COUNT             256U
#define LINE_FLIGHT_TRACE_RECORD_COUNT         16U

#ifndef GYRO_BAUDRATE
#define GYRO_BAUDRATE 9600U
#endif
#ifndef LORA_UART_BAUDRATE
#define LORA_UART_BAUDRATE 115200U
#endif

typedef enum
{
    LINE_MISSION_IDLE = 0,
    LINE_MISSION_START_GATE,
    LINE_MISSION_LEAVE_A,
    LINE_MISSION_TRACK,
    LINE_MISSION_LOST_HOLD,
    LINE_MISSION_COMPLETE,
    LINE_MISSION_SAFE_STOP,
    LINE_MISSION_FAULT
} LineMissionState_t;

typedef enum
{
    LINE_TASK_NONE = 0,
    LINE_TASK_DROP = 1,
    LINE_TASK_DYNAMIC_LANDING = 2
} LineMissionTaskType_t;

typedef enum
{
    LINE_TASK_STATE_IDLE = 0,
    LINE_TASK_STATE_PENDING,
    LINE_TASK_STATE_WAIT_ACK,
    LINE_TASK_STATE_ACCEPTED,
    LINE_TASK_STATE_REJECTED,
    LINE_TASK_STATE_TIMEOUT,
    LINE_TASK_STATE_CANCELLED
} LineMissionTaskState_t;

typedef struct
{
    uint8_t stablePressed;
    uint8_t candidatePressed;
    uint32_t candidateSinceMs;
} LineMissionButton_t;

typedef struct
{
    uint8_t valid;
    uint8_t stage;
    uint8_t telemetryModeCode;
    uint8_t missionStatusSeen;
    uint8_t followSeen;
    uint8_t dropActionSeen;
    uint8_t abortSeen;
    uint16_t statusFlags;
    uint16_t missionId;
    uint8_t errorCode;
    uint32_t sourceTimeMs;
    uint32_t lastUpdateMs;
    uint8_t staleReported;
    uint32_t lastMissionStatusMs;
    uint32_t lastTelemetryMs;
    uint32_t missionStatusCount;
    uint32_t telemetryCount;
    uint32_t livenessRefreshCount;
    uint32_t invalidFrameCount;
} LineMissionFlightState_t;

/* Keep transport evidence independent from a task session.  A physical stop
 * or a new task must not erase the UART5 evidence needed to explain why a
 * flight-stage speed gate did or did not open. */
typedef struct
{
    uint32_t rawByteCount;
    uint32_t frameCount;
    uint32_t parseErrorCount;
    uint32_t timeoutCount;
    uint32_t missionStatusSeenCount;
    uint32_t telemetrySeenCount;
    uint32_t missionStatusRejectedCount;
    uint32_t telemetryRejectedCount;
    uint32_t directStageFrameCount;
    uint32_t unsupportedFrameCount;
    uint8_t lastStageType;
    uint8_t lastStageSource;
    uint8_t lastStageDestination;
    uint8_t lastStageSequence;
    uint8_t lastStageFlags;
    uint8_t lastStageLength;
    uint8_t lastStagePayloadValid;
    uint8_t lastStageCode;
    uint32_t lastStageSourceTimeMs;
} LineMissionRadioRxDiagnostics_t;

typedef struct
{
    uint8_t valid;
    int32_t deltaXCm;
    int32_t deltaYCm;
    uint16_t calibrationId;
} LineMissionLocalCalibration_t;

typedef struct
{
    uint8_t valid;
    uint8_t pending;
    uint8_t source;
    uint8_t sequence;
    uint8_t result;
    uint8_t detail;
    uint32_t lastUpdateMs;
} LineMissionCalibrationRecord_t;

typedef struct
{
    uint8_t destination;
    uint8_t requestType;
    uint8_t requestSequence;
    uint8_t result;
    uint8_t detail;
} LineMissionRadioAck_t;

typedef struct
{
    uint8_t armed;
    uint8_t bReached;
    uint8_t dReached;
    uint8_t aReturnPrepared;
    uint8_t aStopPrepared;
    uint8_t invalidReported;
    uint8_t bReachCandidateCount;
    uint8_t lastPoseValid;
    uint16_t calibrationId;
    int32_t aXCm;
    int32_t aYCm;
    int32_t lastPoseXCm;
    int32_t lastPoseYCm;
    uint32_t aDistanceSquaredCm;
    uint32_t lastPoseFrameMs;
    uint32_t bReachedMs;
    uint32_t dReachedMs;
    uint32_t aPreparedMs;
} LineMissionRadarAssist_t;

typedef struct
{
    int16_t delta[LINE_VELOCITY_WINDOW_COUNT];
    uint16_t periodMs[LINE_VELOCITY_WINDOW_COUNT];
    int32_t deltaSum;
    uint32_t periodSumMs;
    uint8_t writeIndex;
    uint8_t usedCount;
} LineVelocityWindow_t;

typedef struct
{
    uint32_t timeMs;
    int32_t lapYawTravelTenths;
    int16_t returnHeadingErrorTenths;
    int16_t errorX100;
    int16_t grayDiffCps;
    int16_t yawRefTenthsPerSec;
    int16_t yawTenthsDeg;
    int16_t yawRateTenthsPerSec;
    int16_t headingErrorTenthsDeg;
    int16_t gyroDiffCps;
    int16_t totalDiffCps;
    int16_t holdDifferentialCps;
    int16_t lostYawRecoveryCps;
    int16_t lastEdgeGrayDifferentialCps;
    int16_t targetLeftCps;
    int16_t targetRightCps;
    int16_t task2FastEntryBaseCps;
    uint8_t task2FastEntryRampActive;
    uint8_t task1DToAStabilityCapActive;
    int16_t measuredLeftCps;
    int16_t measuredRightCps;
    int16_t commandLeftPercent;
    int16_t commandRightPercent;
    uint8_t rawMask;
    uint8_t activeMask;
    uint8_t stableMask;
    uint8_t lostConfirmCount;
    uint8_t lostSearchSource;
    uint8_t centerCaptureActive;
    uint8_t lineClass;
    uint8_t missionState;
    uint8_t gyroFresh;
    uint8_t returnHeadingNearStart;
    uint8_t lostReacquireCount;
    uint8_t completeMarkCount;
    uint32_t runDistanceMm;
} LineFrozenRecord_t;

/* Preserve the sparse, high-value phase transitions for the whole run.  The
 * regular frozen records only cover the last 25.6 s of line control. */
typedef struct
{
    uint32_t localTimeMs;
    uint32_t sourceTimeMs;
    uint16_t missionId;
    uint8_t originType;
    uint8_t stage;
    uint8_t speedUnlocked;
    uint8_t abortSeen;
} LineMissionFlightTraceRecord_t;

/* Radio traffic can continue after the motors stop.  Preserve the state at
 * the instant of the freeze so post-stop frames cannot rewrite the evidence. */
typedef struct
{
    uint32_t freezeTimeMs;
    uint32_t flightSourceTimeMs;
    uint32_t missionStatusCount;
    uint32_t telemetryCount;
    uint32_t invalidFrameCount;
    uint32_t abortAckCount;
    uint32_t radarBReachedMs;
    uint32_t dReachedMs;
    uint32_t radarADistanceSquaredCm;
    uint16_t missionId;
    uint8_t valid;
    uint8_t taskType;
    uint8_t taskState;
    uint8_t flightStage;
    uint8_t telemetryModeCode;
    uint8_t flightValid;
    uint8_t flightFollowSeen;
    uint8_t flightAbortSeen;
    uint8_t speedUnlocked;
    uint8_t task2LocalDSpeedLatched;
    uint8_t radarBReached;
    uint8_t dReached;
    uint8_t radarAReturnPrepared;
    uint8_t radarAStopPrepared;
    uint8_t aReturnEncoderApproach;
    uint8_t abortSequence;
    uint8_t abortTxRemaining;
} LineMissionFreezeSnapshot_t;

static LineObserver_t s_observer;
static SpeedLadrc_t s_leftSpeed;
static SpeedLadrc_t s_rightSpeed;
static LineVelocityWindow_t s_leftVelocityWindow;
static LineVelocityWindow_t s_rightVelocityWindow;
static LineFrozenRecord_t s_frozenRecord[LINE_FROZEN_RECORD_COUNT];
static LineMissionFlightTraceRecord_t
    s_flightTrace[LINE_FLIGHT_TRACE_RECORD_COUNT];
static LineMissionFreezeSnapshot_t s_freezeSnapshot;

static LineMissionState_t s_state;
static uint32_t s_lastControlMs;
static uint32_t s_lastLogMs;
static uint32_t s_stateStartMs;
static uint32_t s_runStartMs;
static uint32_t s_motionStartMs;
static uint32_t s_lostStartMs;
static LineMissionButton_t s_taskOneButton;
static LineMissionButton_t s_taskTwoButton;
static LineMissionButton_t s_calibrationButton;
static uint8_t s_normalLineCount;
static uint8_t s_completeMarkCount;
static uint8_t s_gyroStaleReported;
static uint8_t s_encoderNotLiveReported;
static uint8_t s_lapYawTravelValid;
static int16_t s_lastLapYawTenths;
static int32_t s_lapYawTravelTenths;
static uint8_t s_startYawValid;
static int16_t s_startYawTenths;
static uint32_t s_runDistanceMm;
static uint32_t s_lastEncoderProgressMs;
static uint8_t s_motionStarted;
static uint8_t s_taskTwoBDeadlineReported;
static uint8_t s_taskTwoBReachedReported;
static uint8_t s_aReturnEncoderApproachReported;
static uint8_t s_radioSequence;
static uint8_t s_radioHeartbeatStarted;
static uint32_t s_lastRadioHeartbeatMs;
static uint32_t s_radioHeartbeatTxCount;
static uint32_t s_lastRadioSlot;
static uint32_t s_radioPoseTxCount;
static uint32_t s_radioPoseDropCount;
static V22StreamParser_t s_radioRxParser;
static LineMissionRadioRxDiagnostics_t s_radioRxDiagnostics;
static uint32_t s_radioAckFrameCount;
static uint32_t s_radioAckDropCount;
static uint16_t s_radioUartErrorFlags;
static LineMissionTaskType_t s_startTaskType;
static uint8_t s_startPrepared;
static LineMissionTaskType_t s_taskType;
static LineMissionTaskState_t s_taskState;
static uint16_t s_nextMissionId;
static uint16_t s_taskMissionId;
static uint16_t s_taskCalibrationId;
static uint8_t s_taskSequence;
static uint8_t s_taskTxRemaining;
static uint8_t s_taskPosePreambleCount;
static uint16_t s_taskPosePreambleCalibrationId;
static uint8_t s_taskAckResult;
static uint8_t s_taskAckDetail;
static uint32_t s_taskSourceTimeMs;
static uint32_t s_taskAckDeadlineMs;
static uint8_t s_abortSequence;
static uint8_t s_abortTxRemaining;
static uint8_t s_abortAckResult;
static uint8_t s_abortAckDetail;
static uint32_t s_abortAckDeadlineMs;
static uint32_t s_abortAckFrameCount;
static uint32_t s_abortAckDropCount;
static LineMissionFlightState_t s_flight;
static LineMissionRadarAssist_t s_radarAssist;
static uint8_t s_coordinationSpeedUnlocked;
static uint8_t s_task2LocalDSpeedLatched;
static uint16_t s_maintenanceResetId;
static uint16_t s_maintenanceNextResetId;
static uint8_t s_maintenanceBroadcastRemaining;
static uint8_t s_maintenanceBroadcastSequence;
static uint32_t s_maintenanceBroadcastSourceTimeMs;
static uint32_t s_maintenanceResetAttemptCount;
static uint32_t s_maintenanceResetSuccessCount;
static uint32_t s_maintenanceResetRunningRejectCount;
static uint32_t s_maintenanceResetBusyRejectCount;
static const char *s_maintenanceLastResult;
static LineMissionLocalCalibration_t s_localCalibration;
static LineMissionCalibrationRecord_t
    s_calibrationRecord[LINE_CALIBRATION_RECORD_COUNT];
static LineMissionRadioAck_t s_radioAckQueue[LINE_RADIO_ACK_QUEUE_COUNT];
static uint8_t s_radioAckQueueHead;
static uint8_t s_radioAckQueueTail;
static uint8_t s_radioAckQueueCount;
static uint16_t s_frozenWriteIndex;
static uint16_t s_frozenCount;
static uint8_t s_frozen;
static uint8_t s_flightTraceWriteIndex;
static uint8_t s_flightTraceCount;
static uint32_t s_lastButtonPressMs;
static uint32_t s_greenLedUntilMs;
static uint32_t s_yellowLedUntilMs;
static const char *s_lastReason;
static int16_t s_holdDifferentialCps;
static int16_t s_lostYawRecoveryCps;
static int16_t s_lastTrackingDifferentialCps;
static int16_t s_lastEdgeGrayDifferentialCps;
static int16_t s_turnHistory[LINE_LOST_DIRECTION_HISTORY_COUNT];
static uint8_t s_turnHistoryWriteIndex;
static uint8_t s_turnHistoryCount;
static uint8_t s_lostReacquireCount;
static uint8_t s_lostSearchSource;
static int16_t s_targetLeftCps;
static int16_t s_targetRightCps;
static int16_t s_smoothedBaseCps;
static int16_t s_smoothedDifferentialCps;
static int16_t s_task2FastEntryBaseCps;
static uint8_t s_task2FastEntryRampActive;
static int16_t s_measuredLeftCps;
static int16_t s_measuredRightCps;
static int16_t s_commandLeftPercent;
static int16_t s_commandRightPercent;
static int16_t s_rearLeftRawPercent;
static int16_t s_rearRightRawPercent;

static void LineMission_ResetTaskSession(LineMissionTaskType_t taskType);
static void LineMission_ResetRadarAssist(void);
static void LineMission_ClearLocalCalibration(void);
static uint8_t LineMission_IsLocalCalibrationPoseReady(uint32_t nowMs);
static int16_t LineMission_ReturnHeadingErrorTenths(void);
static uint8_t LineMission_IsReturnAHeadingNearStart(void);
static int32_t LineMission_CarPoseForwardOffsetXCm(int16_t yawTenthsDeg);
static int32_t LineMission_CarPoseForwardOffsetYCm(int16_t yawTenthsDeg);
static int32_t LineMission_CarPoseTransmitXCm(int32_t radarXCm,
                                              int16_t yawTenthsDeg);
static int32_t LineMission_CarPoseTransmitYCm(int32_t radarYCm,
                                              int16_t yawTenthsDeg);
static uint32_t LineMission_RunElapsedMs(uint32_t nowMs);
static uint32_t LineMission_EncoderProgressAgeMs(uint32_t nowMs);
static void LineMission_EnterState(LineMissionState_t state,
                                   uint32_t nowMs,
                                   const char *reason);

static void LineMission_CancelPendingTask(const char *reason, uint32_t nowMs);
static void LineMission_QueueMissionAbort(const char *reason, uint32_t nowMs);

static int16_t LineMission_Clamp(int32_t value, int16_t lower, int16_t upper)
{
    if (value > upper)
    {
        return upper;
    }
    if (value < lower)
    {
        return lower;
    }
    return (int16_t)value;
}

static int16_t LineMission_SlewToward(int16_t current,
                                      int16_t requested,
                                      int16_t maximumStep)
{
    int32_t delta = (int32_t)requested - current;

    if (delta > maximumStep)
    {
        return (int16_t)((int32_t)current + maximumStep);
    }
    if (delta < -maximumStep)
    {
        return (int16_t)((int32_t)current - maximumStep);
    }
    return requested;
}

static int32_t LineMission_Abs(int32_t value)
{
    return (value < 0) ? -value : value;
}

/* A nonzero car-owned radar-to-platform correction is expressed in the car
 * body frame and rotated by the reported yaw before publishing it. The
 * conservative release default is zero because the air-side/Pi chain already
 * owns its mounting correction. A small fixed-point sine table avoids a libm dependency in the STM32 image.
 * The table is sin(0..90 deg) in 5 degree steps, scaled by 1000. */
static int32_t LineMission_SinTenthsDegQ1000(int16_t angleTenths)
{
    static const uint16_t sineQuarterQ1000[] =
    {
        0U, 87U, 174U, 259U, 342U, 423U, 500U, 574U, 643U, 707U,
        766U, 819U, 866U, 906U, 940U, 966U, 985U, 996U, 1000U
    };
    int32_t normalizedTenths = (int32_t)angleTenths % 3600L;
    int32_t quarterTenths;
    int32_t sineQ1000;
    uint8_t negative = 0U;
    uint8_t index;
    uint8_t remainder;

    if (normalizedTenths < 0L)
    {
        normalizedTenths += 3600L;
    }

    if (normalizedTenths <= 900L)
    {
        quarterTenths = normalizedTenths;
    }
    else if (normalizedTenths <= 1800L)
    {
        quarterTenths = 1800L - normalizedTenths;
    }
    else if (normalizedTenths <= 2700L)
    {
        quarterTenths = normalizedTenths - 1800L;
        negative = 1U;
    }
    else
    {
        quarterTenths = 3600L - normalizedTenths;
        negative = 1U;
    }

    index = (uint8_t)(quarterTenths / 50L);
    remainder = (uint8_t)(quarterTenths % 50L);
    if (index >= 18U)
    {
        sineQ1000 = 1000L;
    }
    else
    {
        sineQ1000 = (int32_t)sineQuarterQ1000[index] +
                     (((int32_t)sineQuarterQ1000[index + 1U] -
                       (int32_t)sineQuarterQ1000[index]) * remainder + 25L) /
                     50L;
    }

    return (negative != 0U) ? -sineQ1000 : sineQ1000;
}

static int32_t LineMission_CarPoseForwardOffsetXCm(int16_t yawTenthsDeg)
{
    int64_t scaled = (int64_t)LINE_CAR_POSE_PLATFORM_FORWARD_OFFSET_CM *
                     (int64_t)LineMission_SinTenthsDegQ1000(
                         (int16_t)(yawTenthsDeg + 900));

    return (int32_t)((scaled >= 0L) ? ((scaled + 500L) / 1000L) :
                                       ((scaled - 500L) / 1000L));
}

static int32_t LineMission_CarPoseForwardOffsetYCm(int16_t yawTenthsDeg)
{
    int64_t scaled = (int64_t)LINE_CAR_POSE_PLATFORM_FORWARD_OFFSET_CM *
                     (int64_t)LineMission_SinTenthsDegQ1000(yawTenthsDeg);

    return (int32_t)((scaled >= 0L) ? ((scaled + 500L) / 1000L) :
                                       ((scaled - 500L) / 1000L));
}

static void LineMission_AccumulateRunDistance(uint32_t nowMs)
{
    uint32_t leftCount = (uint32_t)LineMission_Abs(g_encoderL.count);
    uint32_t rightCount = (uint32_t)LineMission_Abs(g_encoderR.count);
    uint32_t meanCount = (leftCount + rightCount) / 2U;
    uint32_t deltaMm = (meanCount * WHEEL_CIRCUMFERENCE_MM) /
                       ENCODER_4X_PPR;

    /* Raw count motion is enough to prove the vehicle is not jammed. The
     * integer mm conversion can legitimately round a very short sample to
     * zero at the cooperative low speed. */
    if (meanCount != 0U)
    {
        s_lastEncoderProgressMs = nowMs;
    }

    /* Use absolute wheel increments so an inside wheel reversing in a tight
     * turn does not erase the travelled-distance gate. */
    if (UINT32_MAX - s_runDistanceMm < deltaMm)
    {
        s_runDistanceMm = UINT32_MAX;
    }
    else
    {
        s_runDistanceMm += deltaMm;
    }
}

static uint8_t LineMission_CountBits(uint8_t mask)
{
    uint8_t count = 0U;

    while (mask != 0U)
    {
        count += (uint8_t)(mask & 1U);
        mask >>= 1U;
    }
    return count;
}

static uint8_t LineMission_IsStartMarker(void)
{
    /* The full-black A marker remains the preferred start indication.  Under
     * uneven lighting, however, an edge sensor (or X3/X7) can miss while the
     * centre of the marker is still stable.  Either X4 or X5 is sufficient
     * for the manual start gate; return-A stopping keeps its stricter marker. */
    return ((s_observer.lineClass == LINE_OBSERVER_A_MARK) ||
            ((s_observer.stableMask & LINE_START_CENTER_MASK) != 0U)) ?
           1U : 0U;
}

static const char *LineMission_ReturnAMarkerReason(void)
{
    /* The final stop needs the encoder window plus a broad gray signature.
     * A curved D-to-A approach may miss one raw head, so a stable seven-head
     * pattern is accepted instead of requiring raw 0xFF. */
    if (s_runDistanceMm < LINE_A_RETURN_MIN_DISTANCE_MM)
    {
        return 0;
    }
    if ((s_observer.lineClass == LINE_OBSERVER_A_MARK) ||
        ((s_observer.rawMask == LINE_A_RAW_FULL_MASK) &&
         (LineMission_CountBits(s_observer.stableMask) >=
          LINE_A_RETURN_MIN_STABLE_BITS)))
    {
        return "A_MARK_RETURN";
    }
    if ((LineMission_CountBits(s_observer.stableMask) >=
         LINE_A_RETURN_MIN_STABLE_BITS) &&
        (LineMission_IsReturnAHeadingNearStart() != 0U))
    {
        return "A_RETURN_GRAY_7BIT";
    }
    /* Radar cannot terminate the run by itself: its tight window still needs
     * a current centre-gray capture, return-to-start heading and the same
     * encoder distance gate. */
    if ((s_radarAssist.aStopPrepared != 0U) &&
        (s_observer.centerCaptureActive != 0U) &&
        (LineMission_IsReturnAHeadingNearStart() != 0U))
    {
        return "A_RETURN_RADAR_CENTER";
    }
    return 0;
}

static uint8_t LineMission_IsReturnAEncoderApproach(void)
{
    /* D-to-A contains a shallow arc, so cumulative yaw is not reliable as a
     * braking gate.  Use encoder distance as the primary authority, then use
     * return-to-start heading to begin braking earlier on the arc.  The 8800
     * mm arm remains a heading-independent fallback. */
    if ((s_taskType == LINE_TASK_DYNAMIC_LANDING) &&
        (s_runDistanceMm >= LINE_TASK2_A_RETURN_APPROACH_DISTANCE_MM))
    {
        return 1U;
    }
    return ((s_runDistanceMm >= LINE_A_RETURN_APPROACH_DISTANCE_MM) ||
            ((s_runDistanceMm >=
              LINE_A_RETURN_HEADING_APPROACH_DISTANCE_MM) &&
             (LineMission_IsReturnAHeadingNearStart() != 0U))) ? 1U : 0U;
}

static uint8_t LineMission_IsTask1DToAStabilityCapActive(void)
{
    /* This uses encoder route progress only.  The task-one loss appeared
     * before the yaw-qualified A approach, so a late yaw/radar decision could
     * not protect the first shallow D-to-A arc. */
    return ((s_taskType == LINE_TASK_DROP) &&
            (s_coordinationSpeedUnlocked != 0U) &&
            (s_runDistanceMm >= LINE_TASK1_D_TO_A_STABILITY_DISTANCE_MM)) ?
           1U : 0U;
}

static int16_t LineMission_WrapTenthsDeg(int32_t value)
{
    while (value > 1800L)
    {
        value -= 3600L;
    }
    while (value < -1800L)
    {
        value += 3600L;
    }
    return (int16_t)value;
}

static void LineMission_ResetLapTurnProgress(void)
{
    s_lapYawTravelTenths = 0L;
    s_lastLapYawTenths = s_observer.yawTenthsDeg;
    s_lapYawTravelValid = (s_observer.gyroFresh != 0U) ? 1U : 0U;
    s_startYawTenths = s_observer.yawTenthsDeg;
    s_startYawValid = (s_observer.gyroFresh != 0U) ? 1U : 0U;
}

static int16_t LineMission_ReturnHeadingErrorTenths(void)
{
    if ((s_startYawValid == 0U) || (s_observer.gyroFresh == 0U))
    {
        return 1800;
    }
    return LineMission_WrapTenthsDeg((int32_t)s_observer.yawTenthsDeg -
                                     s_startYawTenths);
}

static uint8_t LineMission_IsReturnAHeadingNearStart(void)
{
    return (LineMission_Abs(LineMission_ReturnHeadingErrorTenths()) <=
            LINE_A_RETURN_HEADING_WINDOW_TENTHS) ? 1U : 0U;
}

static void LineMission_UpdateLapTurnProgress(void)
{
    int16_t yawStepTenths;

    if (s_observer.gyroFresh == 0U)
    {
        return;
    }
    if (s_lapYawTravelValid == 0U)
    {
        s_lastLapYawTenths = s_observer.yawTenthsDeg;
        s_lapYawTravelValid = 1U;
        s_startYawTenths = s_observer.yawTenthsDeg;
        s_startYawValid = 1U;
        return;
    }

    yawStepTenths = LineMission_WrapTenthsDeg(
        (int32_t)s_observer.yawTenthsDeg - s_lastLapYawTenths);
    s_lastLapYawTenths = s_observer.yawTenthsDeg;
    s_lapYawTravelTenths += yawStepTenths;
    if (s_lapYawTravelTenths > 7200L)
    {
        s_lapYawTravelTenths = 7200L;
    }
    else if (s_lapYawTravelTenths < -7200L)
    {
        s_lapYawTravelTenths = -7200L;
    }
}

static uint8_t LineMission_IsActive(void)
{
    return ((s_state == LINE_MISSION_LEAVE_A) ||
            (s_state == LINE_MISSION_TRACK) ||
            (s_state == LINE_MISSION_LOST_HOLD)) ? 1U : 0U;
}

static uint8_t LineMission_IsMissionBusy(void)
{
    return ((s_state == LINE_MISSION_START_GATE) ||
            (LineMission_IsActive() != 0U)) ? 1U : 0U;
}

static uint8_t LineMission_IsFlightStageValid(uint8_t stage)
{
    return (stage <= V22_MISSION_STAGE_ABORT) ? 1U : 0U;
}

static const char *LineMission_StateName(void)
{
    switch (s_state)
    {
        case LINE_MISSION_START_GATE: return "START_GATE";
        case LINE_MISSION_LEAVE_A:    return "LEAVE_A";
        case LINE_MISSION_TRACK:      return "TRACK";
        case LINE_MISSION_LOST_HOLD:  return "LOST_HOLD";
        case LINE_MISSION_COMPLETE:   return "COMPLETE";
        case LINE_MISSION_SAFE_STOP:  return "SAFE_STOP";
        case LINE_MISSION_FAULT:      return "FAULT";
        case LINE_MISSION_IDLE:
        default:                      return "IDLE";
    }
}

static const char *LineMission_TaskTypeName(LineMissionTaskType_t taskType)
{
    switch (taskType)
    {
        case LINE_TASK_DROP:            return "DROP";
        case LINE_TASK_DYNAMIC_LANDING: return "DYNAMIC_LANDING";
        case LINE_TASK_NONE:
        default:                        return "NONE";
    }
}

static const char *LineMission_TaskStateName(LineMissionTaskState_t taskState)
{
    switch (taskState)
    {
        case LINE_TASK_STATE_PENDING:  return "PENDING";
        case LINE_TASK_STATE_WAIT_ACK: return "WAIT_ACK";
        case LINE_TASK_STATE_ACCEPTED: return "ACCEPTED";
        case LINE_TASK_STATE_REJECTED: return "REJECTED";
        case LINE_TASK_STATE_TIMEOUT:  return "TIMEOUT";
        case LINE_TASK_STATE_CANCELLED:return "CANCELLED";
        case LINE_TASK_STATE_IDLE:
        default:                       return "IDLE";
    }
}

static const char *LineMission_FlightStageName(uint8_t stage)
{
    switch (stage)
    {
        case V22_MISSION_STAGE_PRECHECK:         return "PRECHECK";
        case V22_MISSION_STAGE_TAKEOFF:          return "TAKEOFF";
        case V22_MISSION_STAGE_INTERCEPT:        return "INTERCEPT";
        case V22_MISSION_STAGE_FOLLOW:           return "FOLLOW";
        case V22_MISSION_STAGE_DROP_ALIGN:       return "DROP_ALIGN";
        case V22_MISSION_STAGE_DROP_ACTION:      return "DROP_ACTION";
        case V22_MISSION_STAGE_LAND_ALIGN:       return "LAND_ALIGN";
        case V22_MISSION_STAGE_DESCEND:          return "DESCEND";
        case V22_MISSION_STAGE_ON_PLATFORM_5S:   return "ON_PLATFORM_5S";
        case V22_MISSION_STAGE_PLATFORM_TAKEOFF: return "PLATFORM_TAKEOFF";
        case V22_MISSION_STAGE_RETURN_HOME:      return "RETURN_HOME";
        case V22_MISSION_STAGE_HOME_LAND:        return "HOME_LAND";
        case V22_MISSION_STAGE_ABORT:            return "ABORT";
        case V22_MISSION_STAGE_IDLE:
        default:                                  return "IDLE";
    }
}

static uint8_t LineMission_IsFollowableLineClass(LineObserverClass_t lineClass)
{
    return ((lineClass == LINE_OBSERVER_TRACK) ||
            (lineClass == LINE_OBSERVER_WIDE)) ? 1U : 0U;
}

static void LineMission_ResetTurnHistory(void)
{
    uint8_t index;

    s_turnHistoryWriteIndex = 0U;
    s_turnHistoryCount = 0U;
    s_lostReacquireCount = 0U;
    for (index = 0U; index < LINE_LOST_DIRECTION_HISTORY_COUNT; ++index)
    {
        s_turnHistory[index] = 0;
    }
}

static void LineMission_RecordTrackingTurn(void)
{
    int16_t differential = s_observer.totalDifferentialCps;
    int16_t grayDifferential = s_observer.grayDifferentialCps;

    s_lastTrackingDifferentialCps = differential;
    if ((LineMission_Abs(s_observer.grayErrorX100) <
         LINE_LOST_DIRECTION_ERROR_X100) ||
        (LineMission_Abs(differential) < LINE_LOST_DIRECTION_MIN_CPS))
    {
        return;
    }

    /* This is the last verified sensor-side escape direction.  It has passed
     * the gray stable-mask filter and is more local than the four-frame
     * history, so a confirmed loss searches toward the head that last saw
     * the tape.  The history remains the fallback when no edge was seen. */
    s_lastEdgeGrayDifferentialCps = grayDifferential;
    s_turnHistory[s_turnHistoryWriteIndex] = differential;
    ++s_turnHistoryWriteIndex;
    if (s_turnHistoryWriteIndex >= LINE_LOST_DIRECTION_HISTORY_COUNT)
    {
        s_turnHistoryWriteIndex = 0U;
    }
    if (s_turnHistoryCount < LINE_LOST_DIRECTION_HISTORY_COUNT)
    {
        ++s_turnHistoryCount;
    }
}

static int16_t LineMission_SelectLostHoldDifferential(void)
{
    int32_t weightedSum = 0L;
    int32_t weightSum = 0L;
    int16_t selected;
    uint8_t offset;
    uint8_t index;
    uint8_t weight;

    if (LineMission_Abs(s_lastEdgeGrayDifferentialCps) >=
        LINE_LOST_DIRECTION_MIN_CPS)
    {
        s_lostSearchSource = LINE_LOST_SOURCE_LAST_EDGE;
        return (s_lastEdgeGrayDifferentialCps < 0) ?
               -LINE_LOST_SEARCH_DIFFERENTIAL_CPS :
               LINE_LOST_SEARCH_DIFFERENTIAL_CPS;
    }

    s_lostSearchSource = LINE_LOST_SOURCE_HISTORY;
    for (offset = 0U; offset < s_turnHistoryCount; ++offset)
    {
        index = (uint8_t)(s_turnHistoryWriteIndex +
                          LINE_LOST_DIRECTION_HISTORY_COUNT - 1U - offset);
        if (index >= LINE_LOST_DIRECTION_HISTORY_COUNT)
        {
            index = (uint8_t)(index - LINE_LOST_DIRECTION_HISTORY_COUNT);
        }
        weight = (uint8_t)(s_turnHistoryCount - offset);
        weightedSum += (int32_t)s_turnHistory[index] * weight;
        weightSum += weight;
    }

    selected = (weightSum != 0L) ?
               (int16_t)(weightedSum / weightSum) :
               s_lastTrackingDifferentialCps;
    if (LineMission_Abs(selected) < LINE_LOST_DIRECTION_MIN_CPS)
    {
        s_lostSearchSource = LINE_LOST_SOURCE_NONE;
        return 0;
    }
    return (selected < 0) ? -LINE_LOST_SEARCH_DIFFERENTIAL_CPS :
                            LINE_LOST_SEARCH_DIFFERENTIAL_CPS;
}

static void LineMission_EnterLostHold(uint32_t nowMs)
{
    /* Start the timeout at the first stable all-white observation.  This is
     * shared by the A-exit and normal tracking paths so a task cannot spend
     * an extra leave-A window driving after the line is already gone. */
    s_holdDifferentialCps = LineMission_SelectLostHoldDifferential();
    s_lostReacquireCount = 0U;
    s_lostStartMs = nowMs;
    LineMission_EnterState(LINE_MISSION_LOST_HOLD, nowMs, "LINE_LOST");
}

static void LineMission_VelocityWindowReset(LineVelocityWindow_t *window)
{
    uint8_t index;

    window->deltaSum = 0;
    window->periodSumMs = 0U;
    window->writeIndex = 0U;
    window->usedCount = 0U;
    for (index = 0U; index < LINE_VELOCITY_WINDOW_COUNT; ++index)
    {
        window->delta[index] = 0;
        window->periodMs[index] = 0U;
    }
}

static int16_t LineMission_VelocityWindowPush(LineVelocityWindow_t *window,
                                              int16_t delta,
                                              uint32_t periodMs)
{
    uint16_t storedPeriodMs;
    int32_t cps;

    if (periodMs == 0U)
    {
        return 0;
    }

    storedPeriodMs = (periodMs > 65535U) ? 65535U : (uint16_t)periodMs;
    if (window->usedCount >= LINE_VELOCITY_WINDOW_COUNT)
    {
        window->deltaSum -= window->delta[window->writeIndex];
        window->periodSumMs -= window->periodMs[window->writeIndex];
    }
    else
    {
        ++window->usedCount;
    }

    window->delta[window->writeIndex] = delta;
    window->periodMs[window->writeIndex] = storedPeriodMs;
    window->deltaSum += delta;
    window->periodSumMs += storedPeriodMs;
    ++window->writeIndex;
    if (window->writeIndex >= LINE_VELOCITY_WINDOW_COUNT)
    {
        window->writeIndex = 0U;
    }

    if (window->periodSumMs == 0U)
    {
        return 0;
    }

    cps = (window->deltaSum * 1000L) / (int32_t)window->periodSumMs;
    return LineMission_Clamp(cps, -32768, 32767);
}

static int16_t LineMission_MmToCps(int32_t speedMmPerSec)
{
    if (WHEEL_CIRCUMFERENCE_MM <= 0)
    {
        return 0;
    }
    return LineMission_Clamp((speedMmPerSec * ENCODER_4X_PPR) /
                             WHEEL_CIRCUMFERENCE_MM,
                             -32768, 32767);
}

static int16_t LineMission_RoundPercent(float value)
{
    if (value >= 0.0f)
    {
        return (int16_t)(value + 0.5f);
    }
    return (int16_t)(value - 0.5f);
}

static int16_t LineMission_RearFollowerPercent(int16_t logicalPercent,
                                                int16_t forwardSign,
                                                int16_t forwardTrimPercent)
{
    int32_t scaledPercent = ((int32_t)logicalPercent *
                             LINE_REAR_FOLLOW_GAIN_PERCENT) / 100L;

    /* Rear wheel stiction is higher under the airframe load. Apply a small
     * forward-only floor and per-wheel mechanical trim, but leave deliberate
     * reverse steering commands proportional for sharp turns. */
    if ((logicalPercent > 0) &&
        (scaledPercent > 0) &&
        (scaledPercent < LINE_REAR_FOLLOW_FORWARD_MIN_PERCENT))
    {
        scaledPercent = LINE_REAR_FOLLOW_FORWARD_MIN_PERCENT;
    }
    if (logicalPercent > 0)
    {
        scaledPercent += forwardTrimPercent;
        if (scaledPercent < 0)
        {
            scaledPercent = 0;
        }
    }

    return LineMission_Clamp(scaledPercent * forwardSign, -100, 100);
}

static void LineMission_MotorOff(void)
{
    s_commandLeftPercent = 0;
    s_commandRightPercent = 0;
    s_rearLeftRawPercent = 0;
    s_rearRightRawPercent = 0;
    Motor_SetSpeedBoth(0, 0);
    Motor_Enable(0U);
    AuxTb6612_StopAll();
}

static void LineMission_WriteEvent(const char *event,
                                   const char *reason,
                                   uint32_t nowMs)
{
    DiagUart_WriteString("LF,event=");
    DiagUart_WriteString(event);
    DiagUart_WriteString(",t_ms=");
    DiagUart_WriteUInt32(nowMs);
    DiagUart_WriteString(",state=");
    DiagUart_WriteString(LineMission_StateName());
    DiagUart_WriteString(",reason=");
    DiagUart_WriteString(reason);
    DiagUart_WriteString(",mask=");
    DiagUart_WriteUInt32(s_observer.stableMask);
    DiagUart_WriteString(",gyro_fresh=");
    DiagUart_WriteUInt32(s_observer.gyroFresh);
    DiagUart_WriteString(",gyro_required=");
    DiagUart_WriteUInt32(LINE_REQUIRE_GYRO_FOR_START);
    DiagUart_WriteString(",yaw_tenths=");
    DiagUart_WriteInt32(s_observer.yawTenthsDeg);
    DiagUart_WriteString(",a_start_yaw_tenths=");
    DiagUart_WriteInt32(s_startYawTenths);
    DiagUart_WriteString(",a_heading_err_tenths=");
    DiagUart_WriteInt32(LineMission_ReturnHeadingErrorTenths());
    DiagUart_WriteString(",a_heading_near_start=");
    DiagUart_WriteUInt32(LineMission_IsReturnAHeadingNearStart());
    DiagUart_WriteString(",lap_yaw_travel_tenths=");
    DiagUart_WriteInt32(s_lapYawTravelTenths);
    DiagUart_WriteString(",run_distance_mm=");
    DiagUart_WriteUInt32(s_runDistanceMm);
    DiagUart_WriteString(",motion_started=");
    DiagUart_WriteUInt32(s_motionStarted);
    DiagUart_WriteString(",motion_elapsed_ms=");
    DiagUart_WriteUInt32((s_motionStarted != 0U) ?
                         (nowMs - s_motionStartMs) :
                         (nowMs - s_runStartMs));
    DiagUart_WriteString(",radio_hb_tx_count=");
    DiagUart_WriteUInt32(s_radioHeartbeatTxCount);
    DiagUart_WriteString(",radio_hb_last_ms=");
    DiagUart_WriteUInt32(s_lastRadioHeartbeatMs);
    DiagUart_WriteString(",gyro_age_ms=");
    DiagUart_WriteUInt32(s_observer.gyroAgeMs);
    DiagUart_WriteString(",motors_enabled=");
    DiagUart_WriteUInt32(Motor_IsEnabled());
    DiagUart_WriteString(",rear_enabled=");
    DiagUart_WriteUInt32(AuxTb6612_IsEnabled());
    DiagUart_WriteString("\r\n");
}

static void LineMission_WriteHexByte(uint8_t value)
{
    static const char hex[] = "0123456789ABCDEF";

    DiagUart_WriteChar(hex[value >> 4]);
    DiagUart_WriteChar(hex[value & 0x0FU]);
}

static void LineMission_ResetFlightTrace(void)
{
    s_flightTraceWriteIndex = 0U;
    s_flightTraceCount = 0U;
}

static void LineMission_ResetFreezeSnapshot(void)
{
    s_freezeSnapshot.valid = 0U;
    s_freezeSnapshot.freezeTimeMs = 0U;
    s_freezeSnapshot.flightSourceTimeMs = 0U;
    s_freezeSnapshot.missionStatusCount = 0U;
    s_freezeSnapshot.telemetryCount = 0U;
    s_freezeSnapshot.invalidFrameCount = 0U;
    s_freezeSnapshot.radarBReachedMs = 0U;
    s_freezeSnapshot.dReachedMs = 0U;
    s_freezeSnapshot.radarADistanceSquaredCm = 0U;
    s_freezeSnapshot.missionId = 0U;
    s_freezeSnapshot.taskType = LINE_TASK_NONE;
    s_freezeSnapshot.taskState = LINE_TASK_STATE_IDLE;
    s_freezeSnapshot.flightStage = V22_MISSION_STAGE_IDLE;
    s_freezeSnapshot.telemetryModeCode = V22_MISSION_STAGE_IDLE;
    s_freezeSnapshot.flightValid = 0U;
    s_freezeSnapshot.flightFollowSeen = 0U;
    s_freezeSnapshot.flightAbortSeen = 0U;
    s_freezeSnapshot.speedUnlocked = 0U;
    s_freezeSnapshot.task2LocalDSpeedLatched = 0U;
    s_freezeSnapshot.radarBReached = 0U;
    s_freezeSnapshot.dReached = 0U;
    s_freezeSnapshot.radarAReturnPrepared = 0U;
    s_freezeSnapshot.radarAStopPrepared = 0U;
    s_freezeSnapshot.aReturnEncoderApproach = 0U;
    s_freezeSnapshot.abortSequence = 0U;
    s_freezeSnapshot.abortTxRemaining = 0U;
    s_freezeSnapshot.abortAckCount = 0U;
}

static void LineMission_RecordFlightTrace(uint8_t originType,
                                          uint8_t stage,
                                          uint32_t sourceTimeMs,
                                          uint32_t nowMs)
{
    LineMissionFlightTraceRecord_t *record =
        &s_flightTrace[s_flightTraceWriteIndex];

    record->localTimeMs = nowMs;
    record->sourceTimeMs = sourceTimeMs;
    record->missionId = s_taskMissionId;
    record->originType = originType;
    record->stage = stage;
    record->speedUnlocked = s_coordinationSpeedUnlocked;
    record->abortSeen = s_flight.abortSeen;

    ++s_flightTraceWriteIndex;
    if (s_flightTraceWriteIndex >= LINE_FLIGHT_TRACE_RECORD_COUNT)
    {
        s_flightTraceWriteIndex = 0U;
    }
    if (s_flightTraceCount < LINE_FLIGHT_TRACE_RECORD_COUNT)
    {
        ++s_flightTraceCount;
    }
}

static void LineMission_RecordRawStageFrame(const V22Frame_t *frame,
                                            uint8_t payloadValid,
                                            uint8_t stage,
                                            uint32_t sourceTimeMs)
{
    s_radioRxDiagnostics.lastStageType = frame->type;
    s_radioRxDiagnostics.lastStageSource = frame->source;
    s_radioRxDiagnostics.lastStageDestination = frame->destination;
    s_radioRxDiagnostics.lastStageSequence = frame->sequence;
    s_radioRxDiagnostics.lastStageFlags = frame->flags;
    s_radioRxDiagnostics.lastStageLength = frame->length;
    s_radioRxDiagnostics.lastStagePayloadValid = payloadValid;
    s_radioRxDiagnostics.lastStageCode = stage;
    s_radioRxDiagnostics.lastStageSourceTimeMs = sourceTimeMs;
}

static void LineMission_WriteFreezeSnapshot(void)
{
    DiagUart_WriteString("LF,event=freeze_snapshot,valid=");
    DiagUart_WriteUInt32(s_freezeSnapshot.valid);
    if (s_freezeSnapshot.valid == 0U)
    {
        DiagUart_WriteString("\r\n");
        return;
    }

    DiagUart_WriteString(",t_ms=");
    DiagUart_WriteUInt32(s_freezeSnapshot.freezeTimeMs);
    DiagUart_WriteString(",task_type=");
    DiagUart_WriteString(
        LineMission_TaskTypeName((LineMissionTaskType_t)s_freezeSnapshot.taskType));
    DiagUart_WriteString(",task_state=");
    DiagUart_WriteString(
        LineMission_TaskStateName((LineMissionTaskState_t)s_freezeSnapshot.taskState));
    DiagUart_WriteString(",mission_id=");
    DiagUart_WriteUInt32(s_freezeSnapshot.missionId);
    DiagUart_WriteString(",flight_stage=");
    DiagUart_WriteString(LineMission_FlightStageName(s_freezeSnapshot.flightStage));
    DiagUart_WriteString(",flight_stage_code=");
    DiagUart_WriteUInt32(s_freezeSnapshot.flightStage);
    DiagUart_WriteString(",telem_mode=");
    DiagUart_WriteUInt32(s_freezeSnapshot.telemetryModeCode);
    DiagUart_WriteString(",flight_valid=");
    DiagUart_WriteUInt32(s_freezeSnapshot.flightValid);
    DiagUart_WriteString(",flight_follow_seen=");
    DiagUart_WriteUInt32(s_freezeSnapshot.flightFollowSeen);
    DiagUart_WriteString(",flight_abort_seen=");
    DiagUart_WriteUInt32(s_freezeSnapshot.flightAbortSeen);
    DiagUart_WriteString(",speed_unlocked=");
    DiagUart_WriteUInt32(s_freezeSnapshot.speedUnlocked);
    DiagUart_WriteString(",task2_d_speed_latched=");
    DiagUart_WriteUInt32(s_freezeSnapshot.task2LocalDSpeedLatched);
    DiagUart_WriteString(",radar_b_reached=");
    DiagUart_WriteUInt32(s_freezeSnapshot.radarBReached);
    DiagUart_WriteString(",radar_b_reached_ms=");
    DiagUart_WriteUInt32(s_freezeSnapshot.radarBReachedMs);
    DiagUart_WriteString(",d_reached=");
    DiagUart_WriteUInt32(s_freezeSnapshot.dReached);
    DiagUart_WriteString(",d_reached_ms=");
    DiagUart_WriteUInt32(s_freezeSnapshot.dReachedMs);
    DiagUart_WriteString(",radar_a_return_prepared=");
    DiagUart_WriteUInt32(s_freezeSnapshot.radarAReturnPrepared);
    DiagUart_WriteString(",radar_a_stop_prepared=");
    DiagUart_WriteUInt32(s_freezeSnapshot.radarAStopPrepared);
    DiagUart_WriteString(",a_return_encoder_approach=");
    DiagUart_WriteUInt32(s_freezeSnapshot.aReturnEncoderApproach);
    DiagUart_WriteString(",radar_a_dist_sq_cm=");
    DiagUart_WriteUInt32(s_freezeSnapshot.radarADistanceSquaredCm);
    DiagUart_WriteString(",flight_source_ms=");
    DiagUart_WriteUInt32(s_freezeSnapshot.flightSourceTimeMs);
    DiagUart_WriteString(",status_count=");
    DiagUart_WriteUInt32(s_freezeSnapshot.missionStatusCount);
    DiagUart_WriteString(",telem_count=");
    DiagUart_WriteUInt32(s_freezeSnapshot.telemetryCount);
    DiagUart_WriteString(",flight_invalid=");
    DiagUart_WriteUInt32(s_freezeSnapshot.invalidFrameCount);
    DiagUart_WriteString(",abort_seq=0x");
    LineMission_WriteHexByte(s_freezeSnapshot.abortSequence);
    DiagUart_WriteString(",abort_tx_left=");
    DiagUart_WriteUInt32(s_freezeSnapshot.abortTxRemaining);
    DiagUart_WriteString(",abort_ack_count=");
    DiagUart_WriteUInt32(s_freezeSnapshot.abortAckCount);
    DiagUart_WriteString("\r\n");
}

static void LineMission_DumpFlightTrace(void)
{
    uint8_t index;
    uint8_t cursor;
    const LineMissionFlightTraceRecord_t *record;

    DiagUart_WriteString("LF,event=flight_trace_begin,records=");
    DiagUart_WriteUInt32(s_flightTraceCount);
    DiagUart_WriteString("\r\n");

    cursor = (s_flightTraceCount < LINE_FLIGHT_TRACE_RECORD_COUNT) ?
             0U : s_flightTraceWriteIndex;
    for (index = 0U; index < s_flightTraceCount; ++index)
    {
        record = &s_flightTrace[cursor];
        DiagUart_WriteString("LF,event=flight_trace,t_ms=");
        DiagUart_WriteUInt32(record->localTimeMs);
        DiagUart_WriteString(",origin=0x");
        LineMission_WriteHexByte(record->originType);
        DiagUart_WriteString(",stage=");
        DiagUart_WriteUInt32(record->stage);
        DiagUart_WriteString(",source_ms=");
        DiagUart_WriteUInt32(record->sourceTimeMs);
        DiagUart_WriteString(",mission_id=");
        DiagUart_WriteUInt32(record->missionId);
        DiagUart_WriteString(",speed_unlocked=");
        DiagUart_WriteUInt32(record->speedUnlocked);
        DiagUart_WriteString(",abort_seen=");
        DiagUart_WriteUInt32(record->abortSeen);
        DiagUart_WriteString("\r\n");

        ++cursor;
        if (cursor >= LINE_FLIGHT_TRACE_RECORD_COUNT)
        {
            cursor = 0U;
        }
    }
    DiagUart_WriteString("LF,event=flight_trace_end\r\n");
}

static void LineMission_WriteRadioRxSummary(void)
{
    DiagUart_WriteString("LF,event=radio_rx_summary,raw_bytes=");
    DiagUart_WriteUInt32(s_radioRxDiagnostics.rawByteCount);
    DiagUart_WriteString(",frames=");
    DiagUart_WriteUInt32(s_radioRxDiagnostics.frameCount);
    DiagUart_WriteString(",parse_errors=");
    DiagUart_WriteUInt32(s_radioRxDiagnostics.parseErrorCount);
    DiagUart_WriteString(",timeouts=");
    DiagUart_WriteUInt32(s_radioRxDiagnostics.timeoutCount);
    DiagUart_WriteString(",status_seen=");
    DiagUart_WriteUInt32(s_radioRxDiagnostics.missionStatusSeenCount);
    DiagUart_WriteString(",status_rejected=");
    DiagUart_WriteUInt32(s_radioRxDiagnostics.missionStatusRejectedCount);
    DiagUart_WriteString(",telem_seen=");
    DiagUart_WriteUInt32(s_radioRxDiagnostics.telemetrySeenCount);
    DiagUart_WriteString(",telem_rejected=");
    DiagUart_WriteUInt32(s_radioRxDiagnostics.telemetryRejectedCount);
    DiagUart_WriteString(",direct_stage_frames=");
    DiagUart_WriteUInt32(s_radioRxDiagnostics.directStageFrameCount);
    DiagUart_WriteString(",other_frames=");
    DiagUart_WriteUInt32(s_radioRxDiagnostics.unsupportedFrameCount);
    DiagUart_WriteString(",last_stage_header=0x");
    LineMission_WriteHexByte(s_radioRxDiagnostics.lastStageType);
    DiagUart_WriteString(":0x");
    LineMission_WriteHexByte(s_radioRxDiagnostics.lastStageSource);
    DiagUart_WriteString(":0x");
    LineMission_WriteHexByte(s_radioRxDiagnostics.lastStageDestination);
    DiagUart_WriteString(":0x");
    LineMission_WriteHexByte(s_radioRxDiagnostics.lastStageSequence);
    DiagUart_WriteString(":0x");
    LineMission_WriteHexByte(s_radioRxDiagnostics.lastStageFlags);
    DiagUart_WriteString(":");
    DiagUart_WriteUInt32(s_radioRxDiagnostics.lastStageLength);
    DiagUart_WriteString(",last_stage_payload_valid=");
    DiagUart_WriteUInt32(s_radioRxDiagnostics.lastStagePayloadValid);
    DiagUart_WriteString(",last_stage_code=");
    DiagUart_WriteUInt32(s_radioRxDiagnostics.lastStageCode);
    DiagUart_WriteString(",last_stage_source_ms=");
    DiagUart_WriteUInt32(s_radioRxDiagnostics.lastStageSourceTimeMs);
    DiagUart_WriteString("\r\n");
}

static void LineMission_WriteStatus(uint32_t nowMs)
{
    const volatile GyroWitState_t *gyro = GyroWit_GetState();
    const CarPoseLinkState_t *pose = CarPoseLink_GetState();
    uint8_t index;
    uint8_t rawTaskTwoPressed =
        (GPIO_ReadInputDataBit(GPIOG, GPIO_Pin_9) == Bit_RESET) ? 1U : 0U;
    uint8_t rawCalibrationPressed =
        (GPIO_ReadInputDataBit(GPIOG, GPIO_Pin_12) == Bit_RESET) ? 1U : 0U;
    uint8_t rawTaskOnePressed =
        (GPIO_ReadInputDataBit(GPIOG, GPIO_Pin_13) == Bit_RESET) ? 1U : 0U;

    DiagUart_WriteString("LF,event=status,t_ms=");
    DiagUart_WriteUInt32(nowMs);
    DiagUart_WriteString(",state=");
    DiagUart_WriteString(LineMission_StateName());
    DiagUart_WriteString(",last_reason=");
    DiagUart_WriteString((s_lastReason != 0) ? s_lastReason : "NONE");
    DiagUart_WriteString(",run_watchdog_ms=");
    DiagUart_WriteUInt32(LINE_RUN_WATCHDOG_MS);
    DiagUart_WriteString(",no_progress_watchdog_ms=");
    DiagUart_WriteUInt32(LINE_NO_PROGRESS_WATCHDOG_MS);
    DiagUart_WriteString(",encoder_progress_age_ms=");
    DiagUart_WriteUInt32(LineMission_EncoderProgressAgeMs(nowMs));
    DiagUart_WriteString(",task2_b_deadline_ms=");
    DiagUart_WriteUInt32(LINE_TASK2_B_DEADLINE_MS);
    DiagUart_WriteString(",task1_pwm_max_pct=");
    DiagUart_WriteUInt32((uint32_t)LINE_SPEED_OUT_MAX);
    DiagUart_WriteString(",task2_pwm_max_pct=");
    DiagUart_WriteUInt32((uint32_t)LINE_TASK2_SPEED_OUT_MAX);
    DiagUart_WriteString(",rear_follower_gain_pct=");
    DiagUart_WriteUInt32(LINE_REAR_FOLLOW_GAIN_PERCENT);
    DiagUart_WriteString(",rear_follower_forward_min_pct=");
    DiagUart_WriteUInt32(LINE_REAR_FOLLOW_FORWARD_MIN_PERCENT);
    DiagUart_WriteString(",rear_left_forward_trim_pct=");
    DiagUart_WriteInt32(LINE_REAR_LEFT_FORWARD_TRIM_PERCENT);
    DiagUart_WriteString(",rear_left_startup_trim_pct=");
    DiagUart_WriteInt32(LINE_REAR_LEFT_STARTUP_TRIM_PERCENT);
    DiagUart_WriteString(",rear_right_forward_trim_pct=");
    DiagUart_WriteInt32(LINE_REAR_RIGHT_FORWARD_TRIM_PERCENT);
    DiagUart_WriteString(",rear_right_startup_trim_pct=");
    DiagUart_WriteInt32(LINE_REAR_RIGHT_STARTUP_TRIM_PERCENT);
    DiagUart_WriteString(",motion_started=");
    DiagUart_WriteUInt32(s_motionStarted);
    DiagUart_WriteString(",motion_elapsed_ms=");
    DiagUart_WriteUInt32(LineMission_RunElapsedMs(nowMs));
    DiagUart_WriteString(",pose_start_gate=");
    DiagUart_WriteUInt32(LINE_REQUIRE_CALIBRATED_POSE_FOR_TASK_START);
    DiagUart_WriteString(",task_start_direct_gate_ms=");
    DiagUart_WriteUInt32(LINE_TASK_START_DIRECT_GATE_MS);
    DiagUart_WriteString(",task_start_policy=DIRECT_NO_FOLLOW_GATE");
    DiagUart_WriteString(",fast_profile_mm_s=600/420/360/260/180/600,fast_target_max_cps=");
    DiagUart_WriteUInt32(LINE_TARGET_MAX_CPS);
    DiagUart_WriteString(",task2_fast_entry_base_slew_cps_per_20ms=");
    DiagUart_WriteUInt32(LINE_TASK2_FAST_ENTRY_BASE_SLEW_CPS_PER_SAMPLE);
    DiagUart_WriteString(",task1_d_to_a_stability_after_mm=");
    DiagUart_WriteUInt32(LINE_TASK1_D_TO_A_STABILITY_DISTANCE_MM);
    DiagUart_WriteString(",task1_d_to_a_stability_speed_mm_s=");
    DiagUart_WriteUInt32(LINE_TASK1_D_TO_A_STABILITY_SPEED_MM);
    DiagUart_WriteString(",car_pose_yaw_sign=1,leds=pg15_green_pg14_spare_pe1_yellow_active_low");
    DiagUart_WriteString(",uart_manual_stop_enabled=");
    DiagUart_WriteUInt32(LINE_ENABLE_UART_MANUAL_STOP);
    DiagUart_WriteString(",car_pose_forward_offset_cm=");
    DiagUart_WriteInt32(LINE_CAR_POSE_PLATFORM_FORWARD_OFFSET_CM);
    DiagUart_WriteString(",car_pose_tx=x:-pi_x+offset*cos(yaw)+delta_x,y:-pi_y+offset*sin(yaw)+delta_y,yaw:1");
    DiagUart_WriteString(",lost_timeout_ms=");
    DiagUart_WriteUInt32(LINE_LOST_TIMEOUT_MS);
    DiagUart_WriteString(",lost_reacquire_samples=");
    DiagUart_WriteUInt32(LINE_LOST_REACQUIRE_SAMPLES);
    DiagUart_WriteString(",lost_search_differential_cps=");
    DiagUart_WriteInt32(LINE_LOST_SEARCH_DIFFERENTIAL_CPS);
    DiagUart_WriteString(",post_coord_lost_hold_speed_mm_s=");
    DiagUart_WriteUInt32(LINE_POST_COORD_LOST_HOLD_SPEED_MM_S);
    DiagUart_WriteString(",lost_center_capture=");
    DiagUart_WriteUInt32(s_observer.centerCaptureActive);
    DiagUart_WriteString(",lost_elapsed_ms=");
    DiagUart_WriteUInt32((s_state == LINE_MISSION_LOST_HOLD) ?
                         (nowMs - s_lostStartMs) : 0U);
    DiagUart_WriteString(",live_record_log=");
    DiagUart_WriteUInt32(LINE_LIVE_RECORD_LOG_ENABLE);
    DiagUart_WriteString(",pg13_task1_raw_pressed=");
    DiagUart_WriteUInt32(rawTaskOnePressed);
    DiagUart_WriteString(",pg13_task1_stable_pressed=");
    DiagUart_WriteUInt32(s_taskOneButton.stablePressed);
    DiagUart_WriteString(",pg9_task2_raw_pressed=");
    DiagUart_WriteUInt32(rawTaskTwoPressed);
    DiagUart_WriteString(",pg9_task2_stable_pressed=");
    DiagUart_WriteUInt32(s_taskTwoButton.stablePressed);
    DiagUart_WriteString(",pg12_cal_raw_pressed=");
    DiagUart_WriteUInt32(rawCalibrationPressed);
    DiagUart_WriteString(",pg12_cal_stable_pressed=");
    DiagUart_WriteUInt32(s_calibrationButton.stablePressed);
    DiagUart_WriteString(",pg12_short_press_only=1");
    DiagUart_WriteString(",maint_reset_id=");
    DiagUart_WriteUInt32(s_maintenanceResetId);
    DiagUart_WriteString(",maint_broadcast_left=");
    DiagUart_WriteUInt32(s_maintenanceBroadcastRemaining);
    DiagUart_WriteString(",maint_reset_attempts=");
    DiagUart_WriteUInt32(s_maintenanceResetAttemptCount);
    DiagUart_WriteString(",maint_reset_successes=");
    DiagUart_WriteUInt32(s_maintenanceResetSuccessCount);
    DiagUart_WriteString(",maint_reset_reject_running=");
    DiagUart_WriteUInt32(s_maintenanceResetRunningRejectCount);
    DiagUart_WriteString(",maint_reset_reject_busy=");
    DiagUart_WriteUInt32(s_maintenanceResetBusyRejectCount);
    DiagUart_WriteString(",maint_last_result=");
    DiagUart_WriteString(s_maintenanceLastResult);
    DiagUart_WriteString(",last_button_ms=");
    DiagUart_WriteUInt32(s_lastButtonPressMs);
    DiagUart_WriteString(",stable_mask=");
    DiagUart_WriteUInt32(s_observer.stableMask);
    DiagUart_WriteString(",line_class=");
    DiagUart_WriteString(LineObserver_ClassName(s_observer.lineClass));
    DiagUart_WriteString(",gyro_fresh=");
    DiagUart_WriteUInt32(s_observer.gyroFresh);
    DiagUart_WriteString(",gyro_age_ms=");
    DiagUart_WriteUInt32(s_observer.gyroAgeMs);
    DiagUart_WriteString(",yaw_tenths=");
    DiagUart_WriteInt32(s_observer.yawTenthsDeg);
    DiagUart_WriteString(",yaw_rate_tenths_s=");
    DiagUart_WriteInt32(s_observer.yawRateTenthsPerSec);
    DiagUart_WriteString(",heading_err_tenths=");
    DiagUart_WriteInt32(s_observer.headingErrorTenthsDeg);
    DiagUart_WriteString(",lost_hold_diff_cps=");
    DiagUart_WriteInt32(s_holdDifferentialCps);
    DiagUart_WriteString(",lost_reacquire_count=");
    DiagUart_WriteUInt32(s_lostReacquireCount);
    DiagUart_WriteString(",gyro_stale_continue=");
    DiagUart_WriteUInt32(s_gyroStaleReported);
    DiagUart_WriteString(",encoder_not_live_continue=");
    DiagUart_WriteUInt32(s_encoderNotLiveReported);
    DiagUart_WriteString(",lap_yaw_travel_tenths=");
    DiagUart_WriteInt32(s_lapYawTravelTenths);
    DiagUart_WriteString(",run_distance_mm=");
    DiagUart_WriteUInt32(s_runDistanceMm);
    DiagUart_WriteString(",radio_hb_tx_count=");
    DiagUart_WriteUInt32(s_radioHeartbeatTxCount);
    DiagUart_WriteString(",radio_hb_last_ms=");
    DiagUart_WriteUInt32(s_lastRadioHeartbeatMs);
    DiagUart_WriteString(",radio_pose_tx_count=");
    DiagUart_WriteUInt32(s_radioPoseTxCount);
    DiagUart_WriteString(",radio_pose_drop_count=");
    DiagUart_WriteUInt32(s_radioPoseDropCount);
    DiagUart_WriteString(",radio_ack_count=");
    DiagUart_WriteUInt32(s_radioAckFrameCount);
    DiagUart_WriteString(",radio_ack_drop_count=");
    DiagUart_WriteUInt32(s_radioAckDropCount);
    DiagUart_WriteString(",radio_uart_err=");
    DiagUart_WriteUInt32(s_radioUartErrorFlags);
    DiagUart_WriteString(",pi_pose_fresh=");
    DiagUart_WriteUInt32(CarPoseLink_IsFresh(nowMs, LINE_RADIO_POSE_FRESH_MS));
    DiagUart_WriteString(",mcu_pose_ready=");
    DiagUart_WriteUInt32(LineMission_IsLocalCalibrationPoseReady(nowMs));
    DiagUart_WriteString(",pi_pose_rx_ok=");
    DiagUart_WriteUInt32(pose->validFrameCount);
    DiagUart_WriteString(",pi_pose_v22_ok=");
    DiagUart_WriteUInt32(pose->v22FrameCount);
    DiagUart_WriteString(",pi_pose_legacy_ok=");
    DiagUart_WriteUInt32(pose->legacyFrameCount);
    DiagUart_WriteString(",pi_pose_source=");
    DiagUart_WriteUInt32(pose->sourceFormat);
    DiagUart_WriteString(",pi_pose_raw_bytes=");
    DiagUart_WriteUInt32(pose->rawByteCount);
    DiagUart_WriteString(",pi_pose_rx_bad=");
    DiagUart_WriteUInt32(pose->invalidFrameCount);
    DiagUart_WriteString(",pi_pose_legacy_bad=");
    DiagUart_WriteUInt32(pose->legacyInvalidFrameCount);
    DiagUart_WriteString(",pi_pose_crc_bad=");
    DiagUart_WriteUInt32(pose->crcErrorCount);
    DiagUart_WriteString(",pi_pose_out_of_order=");
    DiagUart_WriteUInt32(pose->outOfOrderFrameCount);
    DiagUart_WriteString(",pi_pose_src_time_rollback=");
    DiagUart_WriteUInt32(pose->sourceTimeRollbackCount);
    DiagUart_WriteString(",pi_pose_invalid_velocity=");
    DiagUart_WriteUInt32(pose->invalidVelocityCount);
    DiagUart_WriteString(",pi_pose_uart_err=");
    DiagUart_WriteUInt32(pose->uartErrorFlags);
    DiagUart_WriteString(",pi_pose_uart_ring_ovf=");
    DiagUart_WriteUInt32(pose->uartRingOverflowCount);
    DiagUart_WriteString(",pi_pose_cal_id=");
    DiagUart_WriteUInt32(pose->calibrationId);
    DiagUart_WriteString(",pi_pose_calibrated_consecutive=");
    DiagUart_WriteUInt32(pose->calibratedConsecutiveFrameCount);
    DiagUart_WriteString(",mcu_calibrated=");
    DiagUart_WriteUInt32(s_localCalibration.valid);
    DiagUart_WriteString(",mcu_cal_id=");
    DiagUart_WriteUInt32(s_localCalibration.calibrationId);
    DiagUart_WriteString(",mcu_delta_x_cm=");
    DiagUart_WriteInt32(s_localCalibration.deltaXCm);
    DiagUart_WriteString(",mcu_delta_y_cm=");
    DiagUart_WriteInt32(s_localCalibration.deltaYCm);
    DiagUart_WriteString(",pi_pose_flags=");
    DiagUart_WriteUInt32(pose->poseFlags);
    DiagUart_WriteString(",pi_pose_x_cm=");
    DiagUart_WriteInt32(pose->xCm);
    DiagUart_WriteString(",pi_pose_y_cm=");
    DiagUart_WriteInt32(pose->yCm);
    DiagUart_WriteString(",pi_pose_yaw_tenths=");
    DiagUart_WriteInt32(pose->yawTenthsDeg);
    DiagUart_WriteString(",car_pose_forward_offset_x_cm=");
    DiagUart_WriteInt32(
        LineMission_CarPoseForwardOffsetXCm(pose->yawTenthsDeg));
    DiagUart_WriteString(",car_pose_forward_offset_y_cm=");
    DiagUart_WriteInt32(
        LineMission_CarPoseForwardOffsetYCm(pose->yawTenthsDeg));
    DiagUart_WriteString(",car_pose_tx_x_cm=");
    DiagUart_WriteInt32(
        LineMission_CarPoseTransmitXCm(pose->xCm, pose->yawTenthsDeg));
    DiagUart_WriteString(",car_pose_tx_y_cm=");
    DiagUart_WriteInt32(
        LineMission_CarPoseTransmitYCm(pose->yCm, pose->yawTenthsDeg));
    DiagUart_WriteString(",pi_pose_src_ms=");
    DiagUart_WriteUInt32(pose->sourceTimeMs);
    DiagUart_WriteString(",pi_pose_age_ms=");
    if (pose->valid != 0U)
    {
        DiagUart_WriteUInt32(nowMs - pose->lastFrameMs);
    }
    else
    {
        DiagUart_WriteString("NA");
    }
    DiagUart_WriteString(",pi_pose_legacy_hex=");
    if (pose->lastLegacyFrameAvailable != 0U)
    {
        for (index = 0U; index < 14U; ++index)
        {
            LineMission_WriteHexByte(pose->lastLegacyFrame[index]);
        }
    }
    else
    {
        DiagUart_WriteString("NA");
    }
    DiagUart_WriteString(",gyro_angle_frames=");
    DiagUart_WriteUInt32(gyro->angleFrameCount);
    DiagUart_WriteString(",gyro_rate_frames=");
    DiagUart_WriteUInt32(gyro->angularVelocityFrameCount);
    DiagUart_WriteString(",gyro_raw_bytes=");
    DiagUart_WriteUInt32(gyro->rawByteCount);
    DiagUart_WriteString(",gyro_frame_heads=");
    DiagUart_WriteUInt32(gyro->frameHeadCount);
    DiagUart_WriteString(",gyro_checksum_errors=");
    DiagUart_WriteUInt32(gyro->checksumErrorCount);
    DiagUart_WriteString(",task_type=");
    DiagUart_WriteString(LineMission_TaskTypeName(s_taskType));
    DiagUart_WriteString(",task_state=");
    DiagUart_WriteString(LineMission_TaskStateName(s_taskState));
    DiagUart_WriteString(",task_mission_id=");
    DiagUart_WriteUInt32(s_taskMissionId);
    DiagUart_WriteString(",task_cal_id=");
    DiagUart_WriteUInt32(s_taskCalibrationId);
    DiagUart_WriteString(",task_pose_preamble_tx=");
    DiagUart_WriteUInt32(s_taskPosePreambleCount);
    DiagUart_WriteString(",task_pose_preamble_required=");
    DiagUart_WriteUInt32(LINE_TASK_POSE_PREAMBLE_COUNT);
    DiagUart_WriteString(",task_pose_preamble_cal_id=");
    DiagUart_WriteUInt32(s_taskPosePreambleCalibrationId);
    DiagUart_WriteString(",task_seq=");
    DiagUart_WriteUInt32(s_taskSequence);
    DiagUart_WriteString(",task_tx_left=");
    DiagUart_WriteUInt32(s_taskTxRemaining);
    DiagUart_WriteString(",task_ack_result=");
    DiagUart_WriteUInt32(s_taskAckResult);
    DiagUart_WriteString(",task_ack_detail=");
    DiagUart_WriteUInt32(s_taskAckDetail);
    DiagUart_WriteString(",abort_seq=");
    DiagUart_WriteUInt32(s_abortSequence);
    DiagUart_WriteString(",abort_tx_left=");
    DiagUart_WriteUInt32(s_abortTxRemaining);
    DiagUart_WriteString(",abort_ack_result=");
    DiagUart_WriteUInt32(s_abortAckResult);
    DiagUart_WriteString(",abort_ack_detail=");
    DiagUart_WriteUInt32(s_abortAckDetail);
    DiagUart_WriteString(",abort_ack_deadline_ms=");
    DiagUart_WriteUInt32(s_abortAckDeadlineMs);
    DiagUart_WriteString(",abort_ack_count=");
    DiagUart_WriteUInt32(s_abortAckFrameCount);
    DiagUart_WriteString(",abort_ack_drop_count=");
    DiagUart_WriteUInt32(s_abortAckDropCount);
    DiagUart_WriteString(",flight_stage=");
    DiagUart_WriteString(LineMission_FlightStageName(s_flight.stage));
    DiagUart_WriteString(",flight_stage_code=");
    DiagUart_WriteUInt32(s_flight.stage);
    DiagUart_WriteString(",flight_stage_valid=");
    DiagUart_WriteUInt32(s_flight.valid);
    DiagUart_WriteString(",flight_follow_seen=");
    DiagUart_WriteUInt32(s_flight.followSeen);
    DiagUart_WriteString(",flight_status_seen=");
    DiagUart_WriteUInt32(s_flight.missionStatusSeen);
    DiagUart_WriteString(",flight_status_count=");
    DiagUart_WriteUInt32(s_flight.missionStatusCount);
    DiagUart_WriteString(",flight_telem_mode=");
    DiagUart_WriteUInt32(s_flight.telemetryModeCode);
    DiagUart_WriteString(",flight_telem_count=");
    DiagUart_WriteUInt32(s_flight.telemetryCount);
    DiagUart_WriteString(",flight_drop_action_seen=");
    DiagUart_WriteUInt32(s_flight.dropActionSeen);
    DiagUart_WriteString(",flight_abort_seen=");
    DiagUart_WriteUInt32(s_flight.abortSeen);
    DiagUart_WriteString(",flight_last_update_ms=");
    DiagUart_WriteUInt32(s_flight.lastUpdateMs);
    DiagUart_WriteString(",flight_source_ms=");
    DiagUart_WriteUInt32(s_flight.sourceTimeMs);
    DiagUart_WriteString(",flight_liveness_refresh_count=");
    DiagUart_WriteUInt32(s_flight.livenessRefreshCount);
    DiagUart_WriteString(",flight_rx_invalid=");
    DiagUart_WriteUInt32(s_flight.invalidFrameCount);
    DiagUart_WriteString(",coord_speed_unlocked=");
    DiagUart_WriteUInt32(s_coordinationSpeedUnlocked);
    DiagUart_WriteString(",task2_d_speed_latched=");
    DiagUart_WriteUInt32(s_task2LocalDSpeedLatched);
    DiagUart_WriteString(",radar_assist_armed=");
    DiagUart_WriteUInt32(s_radarAssist.armed);
    DiagUart_WriteString(",radar_b_reached=");
    DiagUart_WriteUInt32(s_radarAssist.bReached);
    DiagUart_WriteString(",d_reached=");
    DiagUart_WriteUInt32(s_radarAssist.dReached);
    DiagUart_WriteString(",d_reached_ms=");
    DiagUart_WriteUInt32(s_radarAssist.dReachedMs);
    DiagUart_WriteString(",d_odometry_threshold_mm=");
    DiagUart_WriteUInt32(LINE_D_ODOMETRY_DISTANCE_MM);
    DiagUart_WriteString(",d_yaw_threshold_tenths=");
    DiagUart_WriteUInt32(LINE_D_YAW_TRAVEL_TENTHS);
    DiagUart_WriteString(",radar_a_prepared=");
    DiagUart_WriteUInt32(s_radarAssist.aReturnPrepared);
    DiagUart_WriteString(",radar_a_stop_prepared=");
    DiagUart_WriteUInt32(s_radarAssist.aStopPrepared);
    DiagUart_WriteString(",radar_a_dist_sq_cm=");
    DiagUart_WriteUInt32(s_radarAssist.aDistanceSquaredCm);
    DiagUart_WriteString(",a_return_encoder_approach=");
    DiagUart_WriteUInt32(LineMission_IsReturnAEncoderApproach());
    DiagUart_WriteString(",a_return_encoder_approach_mm=");
    DiagUart_WriteUInt32(LINE_A_RETURN_APPROACH_DISTANCE_MM);
    DiagUart_WriteString(",a_return_heading_approach_mm=");
    DiagUart_WriteUInt32(LINE_A_RETURN_HEADING_APPROACH_DISTANCE_MM);
    DiagUart_WriteString(",a_return_heading_window_tenths=");
    DiagUart_WriteUInt32(LINE_A_RETURN_HEADING_WINDOW_TENTHS);
    DiagUart_WriteString(",task2_a_return_approach_mm=");
    DiagUart_WriteUInt32(LINE_TASK2_A_RETURN_APPROACH_DISTANCE_MM);
    DiagUart_WriteString(",a_return_stop_policy=TASK2_ENCODER_8000_PREBRAKE_TASK1_HEADING_8000_8800_GRAY7");
    DiagUart_WriteString(",a_return_approach_speed_mm_s=");
    DiagUart_WriteUInt32(LINE_A_RETURN_APPROACH_SPEED_MM_S);
    DiagUart_WriteString(",motors_enabled=");
    DiagUart_WriteUInt32(Motor_IsEnabled());
    DiagUart_WriteString(",rear_enabled=");
    DiagUart_WriteUInt32(AuxTb6612_IsEnabled());
    DiagUart_WriteString(",gyro_required=");
    DiagUart_WriteUInt32(LINE_REQUIRE_GYRO_FOR_START);
    DiagUart_WriteString("\r\n");
    LineMission_WriteRadioRxSummary();
}

static void LineMission_Freeze(uint32_t nowMs, const char *reason)
{
    s_frozen = 1U;
    s_freezeSnapshot.valid = 1U;
    s_freezeSnapshot.freezeTimeMs = nowMs;
    s_freezeSnapshot.flightSourceTimeMs = s_flight.sourceTimeMs;
    s_freezeSnapshot.missionStatusCount = s_flight.missionStatusCount;
    s_freezeSnapshot.telemetryCount = s_flight.telemetryCount;
    s_freezeSnapshot.invalidFrameCount = s_flight.invalidFrameCount;
    s_freezeSnapshot.missionId = s_taskMissionId;
    s_freezeSnapshot.taskType = (uint8_t)s_taskType;
    s_freezeSnapshot.taskState = (uint8_t)s_taskState;
    s_freezeSnapshot.flightStage = s_flight.stage;
    s_freezeSnapshot.telemetryModeCode = s_flight.telemetryModeCode;
    s_freezeSnapshot.flightValid = s_flight.valid;
    s_freezeSnapshot.flightFollowSeen = s_flight.followSeen;
    s_freezeSnapshot.flightAbortSeen = s_flight.abortSeen;
    s_freezeSnapshot.speedUnlocked = s_coordinationSpeedUnlocked;
    s_freezeSnapshot.task2LocalDSpeedLatched = s_task2LocalDSpeedLatched;
    s_freezeSnapshot.radarBReached = s_radarAssist.bReached;
    s_freezeSnapshot.radarBReachedMs = s_radarAssist.bReachedMs;
    s_freezeSnapshot.dReached = s_radarAssist.dReached;
    s_freezeSnapshot.dReachedMs = s_radarAssist.dReachedMs;
    s_freezeSnapshot.radarAReturnPrepared = s_radarAssist.aReturnPrepared;
    s_freezeSnapshot.radarAStopPrepared = s_radarAssist.aStopPrepared;
    s_freezeSnapshot.aReturnEncoderApproach =
        LineMission_IsReturnAEncoderApproach();
    s_freezeSnapshot.radarADistanceSquaredCm =
        s_radarAssist.aDistanceSquaredCm;
    s_freezeSnapshot.abortSequence = s_abortSequence;
    s_freezeSnapshot.abortTxRemaining = s_abortTxRemaining;
    s_freezeSnapshot.abortAckCount = s_abortAckFrameCount;
    DiagUart_WriteString("LF,event=freeze,t_ms=");
    DiagUart_WriteUInt32(nowMs);
    DiagUart_WriteString(",reason=");
    DiagUart_WriteString(reason);
    DiagUart_WriteString(",records=");
    DiagUart_WriteUInt32(s_frozenCount);
    DiagUart_WriteString(",motors_enabled=");
    DiagUart_WriteUInt32(Motor_IsEnabled());
    DiagUart_WriteString("\r\n");
}

static void LineMission_EnterState(LineMissionState_t state,
                                   uint32_t nowMs,
                                   const char *reason)
{
    s_state = state;
    s_stateStartMs = nowMs;
    s_lastReason = reason;
    LineMission_WriteEvent("state", reason, nowMs);
}

static void LineMission_Stop(LineMissionState_t state,
                             uint32_t nowMs,
                             const char *reason)
{
    if (((state == LINE_MISSION_SAFE_STOP) || (state == LINE_MISSION_FAULT)) &&
        ((LineMission_IsActive() != 0U) ||
         ((s_state == LINE_MISSION_START_GATE) &&
          (s_taskType != LINE_TASK_NONE) &&
          (s_taskMissionId != 0U) &&
          (s_taskState != LINE_TASK_STATE_REJECTED) &&
          (s_taskState != LINE_TASK_STATE_CANCELLED))))
    {
        LineMission_QueueMissionAbort(reason, nowMs);
    }
    LineMission_CancelPendingTask(reason, nowMs);
    LineMission_MotorOff();
    LineMission_EnterState(state, nowMs, reason);
    LineMission_Freeze(nowMs, reason);
}

static void LineMission_Record(uint32_t nowMs)
{
    LineFrozenRecord_t *record = &s_frozenRecord[s_frozenWriteIndex];

    record->timeMs = nowMs;
    record->lapYawTravelTenths = s_lapYawTravelTenths;
    record->returnHeadingErrorTenths =
        LineMission_ReturnHeadingErrorTenths();
    record->runDistanceMm = s_runDistanceMm;
    record->errorX100 = s_observer.grayErrorX100;
    record->grayDiffCps = s_observer.grayDifferentialCps;
    record->yawRefTenthsPerSec = s_observer.yawRateReferenceTenthsPerSec;
    record->yawTenthsDeg = s_observer.yawTenthsDeg;
    record->yawRateTenthsPerSec = s_observer.yawRateTenthsPerSec;
    record->headingErrorTenthsDeg = s_observer.headingErrorTenthsDeg;
    record->gyroDiffCps = s_observer.gyroDifferentialCps;
    record->totalDiffCps = s_observer.totalDifferentialCps;
    record->holdDifferentialCps = s_holdDifferentialCps;
    record->lostYawRecoveryCps = s_lostYawRecoveryCps;
    record->lastEdgeGrayDifferentialCps = s_lastEdgeGrayDifferentialCps;
    record->targetLeftCps = s_targetLeftCps;
    record->targetRightCps = s_targetRightCps;
    record->task2FastEntryBaseCps = s_task2FastEntryBaseCps;
    record->task2FastEntryRampActive = s_task2FastEntryRampActive;
    record->task1DToAStabilityCapActive =
        LineMission_IsTask1DToAStabilityCapActive();
    record->measuredLeftCps = s_measuredLeftCps;
    record->measuredRightCps = s_measuredRightCps;
    record->commandLeftPercent = s_commandLeftPercent;
    record->commandRightPercent = s_commandRightPercent;
    record->rawMask = s_observer.rawMask;
    record->activeMask = s_observer.activeMask;
    record->stableMask = s_observer.stableMask;
    record->lostConfirmCount = s_observer.lostConfirmCount;
    record->lostSearchSource = s_lostSearchSource;
    record->centerCaptureActive = s_observer.centerCaptureActive;
    record->lineClass = (uint8_t)s_observer.lineClass;
    record->missionState = (uint8_t)s_state;
    record->gyroFresh = s_observer.gyroFresh;
    record->returnHeadingNearStart = LineMission_IsReturnAHeadingNearStart();
    record->lostReacquireCount = s_lostReacquireCount;
    record->completeMarkCount = s_completeMarkCount;

    ++s_frozenWriteIndex;
    if (s_frozenWriteIndex >= LINE_FROZEN_RECORD_COUNT)
    {
        s_frozenWriteIndex = 0U;
    }
    if (s_frozenCount < LINE_FROZEN_RECORD_COUNT)
    {
        ++s_frozenCount;
    }
}

#if (LINE_LIVE_RECORD_LOG_ENABLE != 0U)
static void LineMission_WriteRecord(const char *event, uint32_t nowMs)
{
    DiagUart_WriteString("LF,event=");
    DiagUart_WriteString(event);
    DiagUart_WriteString(",t_ms=");
    DiagUart_WriteUInt32(nowMs);
    DiagUart_WriteString(",state=");
    DiagUart_WriteString(LineMission_StateName());
    DiagUart_WriteString(",line_class=");
    DiagUart_WriteString(LineObserver_ClassName(s_observer.lineClass));
    DiagUart_WriteString(",raw_mask=");
    DiagUart_WriteUInt32(s_observer.rawMask);
    DiagUart_WriteString(",active_mask=");
    DiagUart_WriteUInt32(s_observer.activeMask);
    DiagUart_WriteString(",stable_mask=");
    DiagUart_WriteUInt32(s_observer.stableMask);
    DiagUart_WriteString(",center_capture=");
    DiagUart_WriteUInt32(s_observer.centerCaptureActive);
    DiagUart_WriteString(",err_x100=");
    DiagUart_WriteInt32(s_observer.grayErrorX100);
    DiagUart_WriteString(",gray_diff_cps=");
    DiagUart_WriteInt32(s_observer.grayDifferentialCps);
    DiagUart_WriteString(",yaw_ref_tenths_s=");
    DiagUart_WriteInt32(s_observer.yawRateReferenceTenthsPerSec);
    DiagUart_WriteString(",yaw_tenths=");
    DiagUart_WriteInt32(s_observer.yawTenthsDeg);
    DiagUart_WriteString(",yaw_rate_tenths_s=");
    DiagUart_WriteInt32(s_observer.yawRateTenthsPerSec);
    DiagUart_WriteString(",heading_err_tenths=");
    DiagUart_WriteInt32(s_observer.headingErrorTenthsDeg);
    DiagUart_WriteString(",gyro_diff_cps=");
    DiagUart_WriteInt32(s_observer.gyroDifferentialCps);
    DiagUart_WriteString(",total_diff_cps=");
    DiagUart_WriteInt32(s_observer.totalDifferentialCps);
    DiagUart_WriteString(",lost_hold_diff_cps=");
    DiagUart_WriteInt32(s_holdDifferentialCps);
    DiagUart_WriteString(",lost_reacquire_count=");
    DiagUart_WriteUInt32(s_lostReacquireCount);
    DiagUart_WriteString(",target_l_cps=");
    DiagUart_WriteInt32(s_targetLeftCps);
    DiagUart_WriteString(",target_r_cps=");
    DiagUart_WriteInt32(s_targetRightCps);
    DiagUart_WriteString(",meas_l_cps=");
    DiagUart_WriteInt32(s_measuredLeftCps);
    DiagUart_WriteString(",meas_r_cps=");
    DiagUart_WriteInt32(s_measuredRightCps);
    DiagUart_WriteString(",cmd_l_pct=");
    DiagUart_WriteInt32(s_commandLeftPercent);
    DiagUart_WriteString(",cmd_r_pct=");
    DiagUart_WriteInt32(s_commandRightPercent);
    DiagUart_WriteString(",rear_raw_l_pct=");
    DiagUart_WriteInt32(s_rearLeftRawPercent);
    DiagUart_WriteString(",rear_raw_r_pct=");
    DiagUart_WriteInt32(s_rearRightRawPercent);
    DiagUart_WriteString(",gyro_age_ms=");
    DiagUart_WriteUInt32(s_observer.gyroAgeMs);
    DiagUart_WriteString(",gyro_fresh=");
    DiagUart_WriteUInt32(s_observer.gyroFresh);
    DiagUart_WriteString("\r\n");
}
#endif

static void LineMission_DumpFrozenLog(void)
{
    uint16_t index;
    uint16_t count;
    uint16_t cursor;
    const LineFrozenRecord_t *record;

    if (LineMission_IsActive() != 0U)
    {
        DiagUart_WriteString("LF,event=dump_denied,reason=RUNNING\r\n");
        return;
    }

    DiagUart_WriteString("LF,event=dump_begin,frozen=");
    DiagUart_WriteUInt32(s_frozen);
    DiagUart_WriteString(",reason=");
    DiagUart_WriteString((s_lastReason != 0) ? s_lastReason : "NONE");
    DiagUart_WriteString(",records=");
    DiagUart_WriteUInt32(s_frozenCount);
    DiagUart_WriteString("\r\n");
    LineMission_WriteFreezeSnapshot();

    count = s_frozenCount;
    cursor = (count < LINE_FROZEN_RECORD_COUNT) ? 0U : s_frozenWriteIndex;
    for (index = 0U; index < count; ++index)
    {
        record = &s_frozenRecord[cursor];
        DiagUart_WriteString("LF,rec,t_ms,");
        DiagUart_WriteUInt32(record->timeMs);
        DiagUart_WriteString(",lap_yaw_travel,");
        DiagUart_WriteInt32(record->lapYawTravelTenths);
        DiagUart_WriteString(",a_heading_err,");
        DiagUart_WriteInt32(record->returnHeadingErrorTenths);
        DiagUart_WriteString(",a_heading_near_start,");
        DiagUart_WriteUInt32(record->returnHeadingNearStart);
        DiagUart_WriteString(",run_distance_mm,");
        DiagUart_WriteUInt32(record->runDistanceMm);
        DiagUart_WriteString(",state,");
        DiagUart_WriteUInt32(record->missionState);
        DiagUart_WriteString(",class,");
        DiagUart_WriteUInt32(record->lineClass);
        DiagUart_WriteString(",raw,");
        DiagUart_WriteUInt32(record->rawMask);
        DiagUart_WriteString(",active,");
        DiagUart_WriteUInt32(record->activeMask);
        DiagUart_WriteString(",stable,");
        DiagUart_WriteUInt32(record->stableMask);
        DiagUart_WriteString(",lost_confirm,");
        DiagUart_WriteUInt32(record->lostConfirmCount);
        DiagUart_WriteString(",center_capture,");
        DiagUart_WriteUInt32(record->centerCaptureActive);
        DiagUart_WriteString(",err_x100,");
        DiagUart_WriteInt32(record->errorX100);
        DiagUart_WriteString(",gray_diff,");
        DiagUart_WriteInt32(record->grayDiffCps);
        DiagUart_WriteString(",yaw_ref,");
        DiagUart_WriteInt32(record->yawRefTenthsPerSec);
        DiagUart_WriteString(",yaw,");
        DiagUart_WriteInt32(record->yawTenthsDeg);
        DiagUart_WriteString(",yaw_rate,");
        DiagUart_WriteInt32(record->yawRateTenthsPerSec);
        DiagUart_WriteString(",heading_err,");
        DiagUart_WriteInt32(record->headingErrorTenthsDeg);
        DiagUart_WriteString(",gyro_diff,");
        DiagUart_WriteInt32(record->gyroDiffCps);
        DiagUart_WriteString(",total_diff,");
        DiagUart_WriteInt32(record->totalDiffCps);
        DiagUart_WriteString(",lost_hold_diff,");
        DiagUart_WriteInt32(record->holdDifferentialCps);
        DiagUart_WriteString(",lost_yaw_recovery,");
        DiagUart_WriteInt32(record->lostYawRecoveryCps);
        DiagUart_WriteString(",last_edge_gray_diff,");
        DiagUart_WriteInt32(record->lastEdgeGrayDifferentialCps);
        DiagUart_WriteString(",lost_source,");
        DiagUart_WriteUInt32(record->lostSearchSource);
        DiagUart_WriteString(",lost_reacquire_count,");
        DiagUart_WriteUInt32(record->lostReacquireCount);
        DiagUart_WriteString(",a_mark_count,");
        DiagUart_WriteUInt32(record->completeMarkCount);
        DiagUart_WriteString(",target_l,");
        DiagUart_WriteInt32(record->targetLeftCps);
        DiagUart_WriteString(",target_r,");
        DiagUart_WriteInt32(record->targetRightCps);
        DiagUart_WriteString(",task2_fast_base,");
        DiagUart_WriteInt32(record->task2FastEntryBaseCps);
        DiagUart_WriteString(",task2_fast_ramp_active,");
        DiagUart_WriteUInt32(record->task2FastEntryRampActive);
        DiagUart_WriteString(",task1_d_to_a_cap,");
        DiagUart_WriteUInt32(record->task1DToAStabilityCapActive);
        DiagUart_WriteString(",meas_l,");
        DiagUart_WriteInt32(record->measuredLeftCps);
        DiagUart_WriteString(",meas_r,");
        DiagUart_WriteInt32(record->measuredRightCps);
        DiagUart_WriteString(",cmd_l,");
        DiagUart_WriteInt32(record->commandLeftPercent);
        DiagUart_WriteString(",cmd_r,");
        DiagUart_WriteInt32(record->commandRightPercent);
        DiagUart_WriteString(",gyro_fresh,");
        DiagUart_WriteUInt32(record->gyroFresh);
        DiagUart_WriteString("\r\n");

        ++cursor;
        if (cursor >= LINE_FROZEN_RECORD_COUNT)
        {
            cursor = 0U;
        }
    }
    LineMission_WriteRadioRxSummary();
    LineMission_DumpFlightTrace();
    DiagUart_WriteString("LF,event=dump_end\r\n");
}

static uint8_t LineMission_SendRadioHeartbeat(uint32_t nowMs)
{
    V22Frame_t frame;
    uint8_t encoded[V22_MAX_FRAME_BYTES];
    uint16_t length;

    /* V2.2 maintenance heartbeat, payload is fixed at 8 bytes:
     * DeviceStatus, ErrorCode, Reserved(u16=0), UptimeMs(u32, big-endian).
     * It is not CAR_POSE and cannot request an aircraft task. */
    frame.version = V22_VERSION;
    frame.type = V22_TYPE_HEARTBEAT;
    frame.source = V22_ADDR_CAR_RADIO;
    frame.destination = V22_ADDR_BROADCAST;
    frame.sequence = s_radioSequence++;
    frame.flags = 0U;
    frame.length = LINE_MAINTENANCE_PAYLOAD_LENGTH;
    frame.payload[0] = (uint8_t)s_state;
    frame.payload[1] = (s_state == LINE_MISSION_FAULT) ? 1U : 0U;
    frame.payload[2] = 0U;
    frame.payload[3] = 0U;
    frame.payload[4] = (uint8_t)(nowMs >> 24);
    frame.payload[5] = (uint8_t)(nowMs >> 16);
    frame.payload[6] = (uint8_t)(nowMs >> 8);
    frame.payload[7] = (uint8_t)nowMs;
    length = V22Protocol_Encode(encoded, sizeof(encoded), &frame);
    if (length != 0U)
    {
        RobotUart_NanoWriteBuffer(encoded, length);
        ++s_radioHeartbeatTxCount;
        return 1U;
    }
    return 0U;
}

static void LineMission_UpdateRadioHeartbeat(uint32_t nowMs)
{
    /* No-pose maintenance fallback while stopped.  It is never CAR_POSE,
     * MISSION_STATUS, or a task request; active tasks use the formal slot
     * scheduler below. */
    if ((s_radioHeartbeatStarted == 0U) ||
        ((uint32_t)(nowMs - s_lastRadioHeartbeatMs) >=
         LINE_RADIO_HEARTBEAT_PERIOD_MS))
    {
        (void)LineMission_SendRadioHeartbeat(nowMs);
        s_lastRadioHeartbeatMs = nowMs;
        s_radioHeartbeatStarted = 1U;
    }
}

static void LineMission_WriteU16Be(uint8_t *destination, uint16_t value)
{
    destination[0] = (uint8_t)(value >> 8);
    destination[1] = (uint8_t)value;
}

static void LineMission_WriteU32Be(uint8_t *destination, uint32_t value)
{
    destination[0] = (uint8_t)(value >> 24);
    destination[1] = (uint8_t)(value >> 16);
    destination[2] = (uint8_t)(value >> 8);
    destination[3] = (uint8_t)value;
}

static uint16_t LineMission_ReadU16Be(const uint8_t *source)
{
    return (uint16_t)(((uint16_t)source[0] << 8) | source[1]);
}

static uint32_t LineMission_ReadU32Be(const uint8_t *source)
{
    return ((uint32_t)source[0] << 24) |
           ((uint32_t)source[1] << 16) |
           ((uint32_t)source[2] << 8) |
           (uint32_t)source[3];
}

static uint8_t LineMission_NextRadioAckQueueIndex(uint8_t index)
{
    ++index;
    return (index >= LINE_RADIO_ACK_QUEUE_COUNT) ? 0U : index;
}

static void LineMission_WriteCalibrationEvent(const char *event,
                                              const char *reason,
                                              uint32_t nowMs)
{
    DiagUart_WriteString("LF,event=");
    DiagUart_WriteString(event);
    DiagUart_WriteString(",reason=");
    DiagUart_WriteString(reason);
    DiagUart_WriteString(",t_ms=");
    DiagUart_WriteUInt32(nowMs);
    DiagUart_WriteString(",mcu_calibrated=");
    DiagUart_WriteUInt32(s_localCalibration.valid);
    DiagUart_WriteString(",mcu_cal_id=");
    DiagUart_WriteUInt32(s_localCalibration.calibrationId);
    DiagUart_WriteString(",mcu_delta_x_cm=");
    DiagUart_WriteInt32(s_localCalibration.deltaXCm);
    DiagUart_WriteString(",mcu_delta_y_cm=");
    DiagUart_WriteInt32(s_localCalibration.deltaYCm);
    DiagUart_WriteString(",radio_ack_queue=");
    DiagUart_WriteUInt32(s_radioAckQueueCount);
    DiagUart_WriteString("\r\n");
}

static uint32_t LineMission_RunElapsedMs(uint32_t nowMs)
{
    if (LineMission_IsActive() == 0U)
    {
        return 0U;
    }
    return (s_motionStarted != 0U) ? (nowMs - s_motionStartMs) :
           (nowMs - s_runStartMs);
}

static uint32_t LineMission_EncoderProgressAgeMs(uint32_t nowMs)
{
    uint32_t referenceMs;

    if (LineMission_IsActive() == 0U)
    {
        return 0U;
    }

    referenceMs = (s_lastEncoderProgressMs != 0U) ?
                  s_lastEncoderProgressMs : s_runStartMs;
    return nowMs - referenceMs;
}

static void LineMission_WriteTimingEvent(const char *event,
                                         const char *reason,
                                         uint32_t nowMs)
{
    DiagUart_WriteString("LF,event=");
    DiagUart_WriteString(event);
    DiagUart_WriteString(",reason=");
    DiagUart_WriteString(reason);
    DiagUart_WriteString(",t_ms=");
    DiagUart_WriteUInt32(nowMs);
    DiagUart_WriteString(",elapsed_ms=");
    DiagUart_WriteUInt32(LineMission_RunElapsedMs(nowMs));
    DiagUart_WriteString(",time_origin=");
    DiagUart_WriteString((s_motionStarted != 0U) ? "ENCODER_MOTION" :
                                                   "PROPULSION_ARMED");
    DiagUart_WriteString(",task_type=");
    DiagUart_WriteString(LineMission_TaskTypeName(s_taskType));
    DiagUart_WriteString(",state=");
    DiagUart_WriteString(LineMission_StateName());
    DiagUart_WriteString(",run_distance_mm=");
    DiagUart_WriteUInt32(s_runDistanceMm);
    DiagUart_WriteString(",radar_b_reached=");
    DiagUart_WriteUInt32(s_radarAssist.bReached);
    DiagUart_WriteString("\r\n");
}

static uint8_t LineMission_QueueRadioAck(uint8_t destination,
                                         uint8_t requestType,
                                         uint8_t requestSequence,
                                         uint8_t result,
                                         uint8_t detail)
{
    LineMissionRadioAck_t *ack;

    if (s_radioAckQueueCount >= LINE_RADIO_ACK_QUEUE_COUNT)
    {
        ++s_radioAckDropCount;
        return 0U;
    }

    ack = &s_radioAckQueue[s_radioAckQueueHead];
    ack->destination = destination;
    ack->requestType = requestType;
    ack->requestSequence = requestSequence;
    ack->result = result;
    ack->detail = detail;
    s_radioAckQueueHead =
        LineMission_NextRadioAckQueueIndex(s_radioAckQueueHead);
    ++s_radioAckQueueCount;
    return 1U;
}

static uint8_t LineMission_SendQueuedRadioAck(uint32_t nowMs)
{
    const LineMissionRadioAck_t *ack;
    V22Frame_t frame;
    uint8_t encoded[V22_MAX_FRAME_BYTES];
    uint16_t length;

    if (s_radioAckQueueCount == 0U)
    {
        return 0U;
    }

    ack = &s_radioAckQueue[s_radioAckQueueTail];
    frame.version = V22_VERSION;
    frame.type = V22_TYPE_ACK;
    frame.source = V22_ADDR_CAR_RADIO;
    frame.destination = ack->destination;
    frame.sequence = s_radioSequence++;
    frame.flags = 0U;
    frame.length = LINE_ACK_PAYLOAD_LENGTH;
    frame.payload[0] = ack->requestType;
    frame.payload[1] = ack->requestSequence;
    frame.payload[2] = ack->result;
    frame.payload[3] = ack->detail;
    length = V22Protocol_Encode(encoded, sizeof(encoded), &frame);
    if (length == 0U)
    {
        return 0U;
    }

    RobotUart_NanoWriteBuffer(encoded, length);
    s_radioAckQueueTail =
        LineMission_NextRadioAckQueueIndex(s_radioAckQueueTail);
    --s_radioAckQueueCount;
    if (ack->requestType == V22_TYPE_CALIBRATION_SET)
    {
        LineMission_WriteCalibrationEvent("calibration_ack_radio_tx",
                                          "V23_0X83", nowMs);
    }
    return 1U;
}

static int16_t LineMission_FindCalibrationRecord(uint8_t source,
                                                  uint8_t sequence,
                                                  uint32_t nowMs)
{
    uint8_t index;

    for (index = 0U; index < LINE_CALIBRATION_RECORD_COUNT; ++index)
    {
        if ((s_calibrationRecord[index].valid != 0U) &&
            ((uint32_t)(nowMs - s_calibrationRecord[index].lastUpdateMs) >
             LINE_CALIBRATION_DEDUP_MS))
        {
            s_calibrationRecord[index].valid = 0U;
        }
        if ((s_calibrationRecord[index].valid != 0U) &&
            (s_calibrationRecord[index].source == source) &&
            (s_calibrationRecord[index].sequence == sequence))
        {
            return (int16_t)index;
        }
    }
    return -1;
}

static uint8_t LineMission_ReserveCalibrationRecord(uint8_t source,
                                                     uint8_t sequence,
                                                     uint32_t nowMs,
                                                     uint8_t *recordIndex)
{
    uint8_t index;

    if (recordIndex == 0)
    {
        return 0U;
    }

    for (index = 0U; index < LINE_CALIBRATION_RECORD_COUNT; ++index)
    {
        if (s_calibrationRecord[index].valid == 0U)
        {
            s_calibrationRecord[index].valid = 1U;
            s_calibrationRecord[index].pending = 1U;
            s_calibrationRecord[index].source = source;
            s_calibrationRecord[index].sequence = sequence;
            s_calibrationRecord[index].result = V22_ACK_RESULT_INTERNAL;
            s_calibrationRecord[index].detail = 0U;
            s_calibrationRecord[index].lastUpdateMs = nowMs;
            *recordIndex = index;
            return 1U;
        }
    }
    return 0U;
}

static void LineMission_FinishCalibrationRecord(uint8_t recordIndex,
                                                 uint8_t result,
                                                 uint8_t detail,
                                                 uint32_t nowMs)
{
    LineMissionCalibrationRecord_t *record;

    if (recordIndex >= LINE_CALIBRATION_RECORD_COUNT)
    {
        return;
    }

    record = &s_calibrationRecord[recordIndex];
    if (record->valid == 0U)
    {
        return;
    }

    record->pending = 0U;
    record->result = result;
    record->detail = detail;
    record->lastUpdateMs = nowMs;
    if (LineMission_QueueRadioAck(record->source, V22_TYPE_CALIBRATION_SET,
                                  record->sequence, result, detail) == 0U)
    {
        LineMission_WriteCalibrationEvent("calibration_ack_drop",
                                          "RADIO_ACK_QUEUE_FULL", nowMs);
    }
}

static void LineMission_ClearLocalCalibration(void)
{
    s_localCalibration.valid = 0U;
    s_localCalibration.deltaXCm = 0L;
    s_localCalibration.deltaYCm = 0L;
    s_localCalibration.calibrationId = 0U;
}

static uint8_t LineMission_IsLocalCalibrationPoseReady(uint32_t nowMs)
{
    const CarPoseLinkState_t *pose = CarPoseLink_GetState();

    if ((s_localCalibration.valid == 0U) ||
        (s_localCalibration.calibrationId == 0U) ||
        (CarPoseLink_IsFresh(nowMs, LINE_RADIO_POSE_FRESH_MS) == 0U) ||
        (pose->coordinateFrame != V22_COORDINATE_FIELD_GLOBAL) ||
        ((pose->poseFlags &
          (V22_POSE_FLAG_POSITION_VALID | V22_POSE_FLAG_YAW_VALID)) !=
         (V22_POSE_FLAG_POSITION_VALID | V22_POSE_FLAG_YAW_VALID)))
    {
        return 0U;
    }

    return 1U;
}

static int32_t LineMission_SaturateCoordinateCm(int64_t value)
{
    if (value > 2147483647LL)
    {
        return 2147483647L;
    }
    if (value < (-2147483647LL - 1LL))
    {
        return (-2147483647L - 1L);
    }
    return (int32_t)value;
}

static int32_t LineMission_CarPoseTransmitXCm(int32_t radarXCm,
                                              int16_t yawTenthsDeg)
{
    /* The historical Pi axes are mirrored on LoRa. An optional car-owned
     * body-forward correction is yaw-aligned: X_lora = -X_pi +
     * offset*cos(yaw_pi) + DeltaX. The release preset sets offset to zero. */
    int64_t vehicleCenterXCm =
        (int64_t)LineMission_CarPoseForwardOffsetXCm(yawTenthsDeg) -
        (int64_t)radarXCm;

    if (s_localCalibration.valid != 0U)
    {
        vehicleCenterXCm += s_localCalibration.deltaXCm;
    }
    return LineMission_SaturateCoordinateCm(vehicleCenterXCm);
}

static int32_t LineMission_CarPoseTransmitYCm(int32_t radarYCm,
                                              int16_t yawTenthsDeg)
{
    /* Y_lora = -Y_pi + offset*sin(yaw_pi) + DeltaY. */
    int64_t vehicleCenterYCm =
        (int64_t)LineMission_CarPoseForwardOffsetYCm(yawTenthsDeg) -
        (int64_t)radarYCm;

    if (s_localCalibration.valid != 0U)
    {
        vehicleCenterYCm += s_localCalibration.deltaYCm;
    }
    return LineMission_SaturateCoordinateCm(vehicleCenterYCm);
}

static uint8_t LineMission_SendCarPose(const CarPoseLinkState_t *pose)
{
    V22Frame_t frame;
    uint8_t encoded[V22_MAX_FRAME_BYTES];
    uint8_t poseFlags;
    uint16_t length;

    if (pose == 0)
    {
        return 0U;
    }

    /* Pi coordinates stay raw for local motion and radar assistance.  The
     * MCU owns the calibration visible on LoRa, so a Pi-provided calibrated
     * flag or CalibrationId is never relayed into the external session. */
    poseFlags = pose->poseFlags & (uint8_t)~V22_POSE_FLAG_CALIBRATED;
    if (s_localCalibration.valid != 0U)
    {
        poseFlags |= V22_POSE_FLAG_CALIBRATED;
    }
    if (LineMission_IsActive() != 0U)
    {
        poseFlags |= V22_POSE_FLAG_CAR_RUNNING;
    }
    else
    {
        poseFlags &= (uint8_t)~V22_POSE_FLAG_CAR_RUNNING;
    }

    frame.version = V22_VERSION;
    frame.type = V22_TYPE_CAR_POSE;
    frame.source = V22_ADDR_CAR_RADIO;
    frame.destination = V22_ADDR_BROADCAST;
    frame.sequence = s_radioSequence++;
    frame.flags = 0U;
    frame.length = 22U;
    frame.payload[0] = pose->coordinateFrame;
    frame.payload[1] = poseFlags;
    LineMission_WriteU16Be(&frame.payload[2],
                           (s_localCalibration.valid != 0U) ?
                           s_localCalibration.calibrationId : 0U);
    /* Outgoing X/Y use the yaw-rotated platform-centre offset above and add
     * calibration exactly once.  Local control retains raw Pi pose. */
    LineMission_WriteU32Be(&frame.payload[4],
                            (uint32_t)LineMission_CarPoseTransmitXCm(
                                pose->xCm, pose->yawTenthsDeg));
    LineMission_WriteU32Be(&frame.payload[8],
                            (uint32_t)LineMission_CarPoseTransmitYCm(
                                pose->yCm, pose->yawTenthsDeg));
    LineMission_WriteU16Be(&frame.payload[12], (uint16_t)pose->yawTenthsDeg);
    LineMission_WriteU16Be(&frame.payload[14], (uint16_t)pose->vxCmPerSec);
    LineMission_WriteU16Be(&frame.payload[16], (uint16_t)pose->vyCmPerSec);
    LineMission_WriteU32Be(&frame.payload[18], pose->sourceTimeMs);

    length = V22Protocol_Encode(encoded, sizeof(encoded), &frame);
    if (length == 0U)
    {
        return 0U;
    }
    RobotUart_NanoWriteBuffer(encoded, length);
    ++s_radioPoseTxCount;
    return 1U;
}

static uint8_t LineMission_CarSlotDue(uint32_t nowMs)
{
    uint32_t slot = nowMs / LINE_RADIO_CAR_SLOT_MS;

    if ((nowMs % LINE_RADIO_CAR_SLOT_MS) >= LINE_RADIO_CAR_SLOT_WINDOW_MS)
    {
        return 0U;
    }
    return (s_lastRadioSlot != slot) ? 1U : 0U;
}

static void LineMission_MarkRadioSlot(uint32_t nowMs)
{
    s_lastRadioSlot = nowMs / LINE_RADIO_CAR_SLOT_MS;
}

static void LineMission_WriteTaskEvent(const char *event,
                                       const char *reason,
                                       uint32_t nowMs)
{
    DiagUart_WriteString("LF,event=");
    DiagUart_WriteString(event);
    DiagUart_WriteString(",reason=");
    DiagUart_WriteString(reason);
    DiagUart_WriteString(",t_ms=");
    DiagUart_WriteUInt32(nowMs);
    DiagUart_WriteString(",task_type=");
    DiagUart_WriteString(LineMission_TaskTypeName(s_taskType));
    DiagUart_WriteString(",task_state=");
    DiagUart_WriteString(LineMission_TaskStateName(s_taskState));
    DiagUart_WriteString(",mission_id=");
    DiagUart_WriteUInt32(s_taskMissionId);
    DiagUart_WriteString(",calibration_id=");
    DiagUart_WriteUInt32(s_taskCalibrationId);
    DiagUart_WriteString(",pose_preamble_tx=");
    DiagUart_WriteUInt32(s_taskPosePreambleCount);
    DiagUart_WriteString(",pose_preamble_cal_id=");
    DiagUart_WriteUInt32(s_taskPosePreambleCalibrationId);
    DiagUart_WriteString(",seq=");
    DiagUart_WriteUInt32(s_taskSequence);
    DiagUart_WriteString(",tx_left=");
    DiagUart_WriteUInt32(s_taskTxRemaining);
    DiagUart_WriteString(",ack_result=");
    DiagUart_WriteUInt32(s_taskAckResult);
    DiagUart_WriteString(",ack_detail=");
    DiagUart_WriteUInt32(s_taskAckDetail);
    DiagUart_WriteString(",abort_seq=");
    DiagUart_WriteUInt32(s_abortSequence);
    DiagUart_WriteString(",abort_tx_left=");
    DiagUart_WriteUInt32(s_abortTxRemaining);
    DiagUart_WriteString(",abort_ack_result=");
    DiagUart_WriteUInt32(s_abortAckResult);
    DiagUart_WriteString(",abort_ack_detail=");
    DiagUart_WriteUInt32(s_abortAckDetail);
    DiagUart_WriteString(",abort_ack_deadline_ms=");
    DiagUart_WriteUInt32(s_abortAckDeadlineMs);
    DiagUart_WriteString(",abort_ack_count=");
    DiagUart_WriteUInt32(s_abortAckFrameCount);
    DiagUart_WriteString(",abort_ack_drop_count=");
    DiagUart_WriteUInt32(s_abortAckDropCount);
    DiagUart_WriteString(",line_state=");
    DiagUart_WriteString(LineMission_StateName());
    DiagUart_WriteString("\r\n");
}

static void LineMission_ResetTaskPosePreamble(void)
{
    s_taskPosePreambleCount = 0U;
    s_taskPosePreambleCalibrationId = 0U;
}

static void LineMission_ObserveTaskPosePreamble(uint32_t nowMs)
{
    uint16_t calibrationId;

    /* A task request is meaningful to the aircraft only after it has seen
     * current calibrated CAR_POSE traffic for this run.  Count only frames
     * that were encoded and handed to UART5 successfully. */
    if ((s_taskType == LINE_TASK_NONE) ||
        (s_taskState != LINE_TASK_STATE_IDLE) ||
        (LineMission_IsLocalCalibrationPoseReady(nowMs) == 0U))
    {
        LineMission_ResetTaskPosePreamble();
        return;
    }

    calibrationId = s_localCalibration.calibrationId;
    if (s_taskPosePreambleCalibrationId != calibrationId)
    {
        s_taskPosePreambleCount = 0U;
        s_taskPosePreambleCalibrationId = calibrationId;
    }

    if (s_taskPosePreambleCount < LINE_TASK_POSE_PREAMBLE_COUNT)
    {
        ++s_taskPosePreambleCount;
        LineMission_WriteTaskEvent("task_pose_preamble_tx",
                                   "CALIBRATED_CAR_POSE", nowMs);
        if (s_taskPosePreambleCount == LINE_TASK_POSE_PREAMBLE_COUNT)
        {
            LineMission_WriteTaskEvent("task_pose_preamble_ready",
                                       "THREE_CALIBRATED_POSES", nowMs);
        }
    }
}

static uint8_t LineMission_TaskPosePreambleReady(uint32_t nowMs)
{
    return ((s_taskPosePreambleCount >= LINE_TASK_POSE_PREAMBLE_COUNT) &&
            (s_taskPosePreambleCalibrationId != 0U) &&
            (s_taskPosePreambleCalibrationId ==
             s_localCalibration.calibrationId) &&
            (LineMission_IsLocalCalibrationPoseReady(nowMs) != 0U)) ?
           1U : 0U;
}

static void LineMission_CancelPendingTask(const char *reason, uint32_t nowMs)
{
    if ((s_taskState != LINE_TASK_STATE_PENDING) &&
        (s_taskState != LINE_TASK_STATE_WAIT_ACK))
    {
        return;
    }

    s_taskTxRemaining = 0U;
    s_taskAckDeadlineMs = 0U;
    s_taskState = LINE_TASK_STATE_CANCELLED;
    LineMission_WriteTaskEvent("task_request_cancelled", reason, nowMs);
}

static void LineMission_QueueMissionAbort(const char *reason, uint32_t nowMs)
{
    if ((s_taskType == LINE_TASK_NONE) || (s_taskMissionId == 0U) ||
        (s_abortTxRemaining != 0U) ||
        (s_taskState == LINE_TASK_STATE_IDLE) ||
        (s_taskState == LINE_TASK_STATE_REJECTED) ||
        (s_taskState == LINE_TASK_STATE_CANCELLED))
    {
        return;
    }

    s_abortSequence = s_radioSequence++;
    s_abortTxRemaining = LINE_MISSION_ABORT_RETRY_COUNT;
    s_abortAckResult = 0xFFU;
    s_abortAckDetail = 0xFFU;
    s_abortAckDeadlineMs = 0U;
    LineMission_WriteTaskEvent("mission_abort_queued", reason, nowMs);
}

static void LineMission_WriteFlightEvent(const char *event,
                                         const char *reason,
                                         uint32_t nowMs)
{
    DiagUart_WriteString("LF,event=");
    DiagUart_WriteString(event);
    DiagUart_WriteString(",reason=");
    DiagUart_WriteString(reason);
    DiagUart_WriteString(",t_ms=");
    DiagUart_WriteUInt32(nowMs);
    DiagUart_WriteString(",task_type=");
    DiagUart_WriteString(LineMission_TaskTypeName(s_taskType));
    DiagUart_WriteString(",mission_id=");
    DiagUart_WriteUInt32(s_taskMissionId);
    DiagUart_WriteString(",flight_stage=");
    DiagUart_WriteString(LineMission_FlightStageName(s_flight.stage));
    DiagUart_WriteString(",flight_stage_code=");
    DiagUart_WriteUInt32(s_flight.stage);
    DiagUart_WriteString(",speed_unlocked=");
    DiagUart_WriteUInt32(s_coordinationSpeedUnlocked);
    DiagUart_WriteString("\r\n");
}

static void LineMission_WriteRadarEvent(const char *event,
                                        const char *reason,
                                        uint32_t nowMs)
{
    DiagUart_WriteString("LF,event=");
    DiagUart_WriteString(event);
    DiagUart_WriteString(",reason=");
    DiagUart_WriteString(reason);
    DiagUart_WriteString(",t_ms=");
    DiagUart_WriteUInt32(nowMs);
    DiagUart_WriteString(",radar_b_reached=");
    DiagUart_WriteUInt32(s_radarAssist.bReached);
    DiagUart_WriteString(",radar_a_prepared=");
    DiagUart_WriteUInt32(s_radarAssist.aReturnPrepared);
    DiagUart_WriteString(",radar_a_stop_prepared=");
    DiagUart_WriteUInt32(s_radarAssist.aStopPrepared);
    DiagUart_WriteString(",radar_a_dist_sq_cm=");
    DiagUart_WriteUInt32(s_radarAssist.aDistanceSquaredCm);
    DiagUart_WriteString(",run_distance_mm=");
    DiagUart_WriteUInt32(s_runDistanceMm);
    DiagUart_WriteString(",motion_started=");
    DiagUart_WriteUInt32(s_motionStarted);
    DiagUart_WriteString(",motion_elapsed_ms=");
    DiagUart_WriteUInt32(LineMission_RunElapsedMs(nowMs));
    DiagUart_WriteString("\r\n");
}

static uint8_t LineMission_TaskSessionCanConsumeFlight(void)
{
    if ((s_taskType == LINE_TASK_NONE) || (s_taskMissionId == 0U))
    {
        return 0U;
    }
    return ((s_taskState != LINE_TASK_STATE_REJECTED) &&
            (s_taskState != LINE_TASK_STATE_CANCELLED) &&
            (s_taskState != LINE_TASK_STATE_IDLE)) ? 1U : 0U;
}

static void LineMission_TryUnlockCoordinationSpeed(uint32_t nowMs)
{
    uint8_t unlock = 0U;
    const char *unlockReason = "NONE";

    if ((s_flight.abortSeen != 0U) ||
        (s_flight.stage == V22_MISSION_STAGE_ABORT))
    {
        /* A task-two D authorization is a local route-progress latch.  The
         * air side may legitimately report ABORT after its landing action,
         * but that report must not turn a one-lap local acceleration into a
         * momentary pulse. Physical task-key stop, encoder/no-progress,
         * lost-line and final-A safety paths still remove motor authority. */
        if ((s_taskType == LINE_TASK_DYNAMIC_LANDING) &&
            (s_task2LocalDSpeedLatched != 0U))
        {
            s_coordinationSpeedUnlocked = 1U;
            return;
        }
        s_coordinationSpeedUnlocked = 0U;
        return;
    }
    if (s_coordinationSpeedUnlocked != 0U)
    {
        return;
    }

    /* Task two is allowed to leave the cooperative envelope at the locally
     * verified D gate even if the air-side stage has not arrived.  D needs the
     * encoder path plus cumulative-turn confirmation, so this is not a raw
     * wide-black-marker shortcut.  A matching PLATFORM_TAKEOFF (or later)
     * remains the alternate, earlier air-side authorization. */
    if ((s_taskType == LINE_TASK_DYNAMIC_LANDING) &&
        (s_radarAssist.dReached != 0U))
    {
        unlock = 1U;
        unlockReason = "TASK2_LOCAL_D_GATE";
        s_task2LocalDSpeedLatched = 1U;
    }
    else if (s_flight.valid == 0U)
    {
        return;
    }
    else if (s_taskType == LINE_TASK_DROP)
    {
        /* RETURN_HOME is entered only after the DROP_ACTION has completed.
         * Use the matched air transition itself as the speed authority. B/D
         * are retained as local progress diagnostics; a missing radar B frame
         * must not defer a confirmed drop-and-return transition until D. */
        if ((s_flight.dropActionSeen != 0U) &&
            (s_flight.stage == V22_MISSION_STAGE_RETURN_HOME))
        {
            unlock = 1U;
            unlockReason = "TASK1_FLIGHT_STAGE_GATE";
        }
    }
    else if (s_taskType == LINE_TASK_DYNAMIC_LANDING)
    {
        if ((s_flight.stage >= V22_MISSION_STAGE_PLATFORM_TAKEOFF) &&
            (s_flight.stage < V22_MISSION_STAGE_ABORT))
        {
            unlock = 1U;
            unlockReason = "TASK2_PLATFORM_TAKEOFF";
        }
    }

    if (unlock != 0U)
    {
        s_coordinationSpeedUnlocked = 1U;
        if (s_taskType == LINE_TASK_DYNAMIC_LANDING)
        {
            /* Seed from the live cooperative request, not from zero, so the
             * PLATFORM_TAKEOFF/local-D handoff adds no artificial restart. */
            s_task2FastEntryBaseCps = s_smoothedBaseCps;
            if (s_task2FastEntryBaseCps <= 0)
            {
                s_task2FastEntryBaseCps =
                    LineMission_MmToCps(LINE_TASK2_COOP_SPEED_MM_S);
            }
            s_task2FastEntryRampActive = 1U;
        }
        LineMission_WriteFlightEvent("coord_speed_unlocked",
                                     unlockReason,
                                     nowMs);
    }
}

static void LineMission_UpdateFlightStage(uint8_t stage,
                                           uint32_t sourceTimeMs,
                                           uint8_t originType,
                                           const char *reason,
                                           uint32_t nowMs)
{
    uint8_t changed = ((s_flight.valid == 0U) ||
                       (s_flight.stage != stage)) ? 1U : 0U;

    /* SourceTimeMs is the aircraft's monotonic authority.  The air link may
     * retransmit a 2 Hz envelope with the same stage timestamp, which is a
     * liveness refresh only.  It must never permit a stage transition. */
    if ((s_flight.valid != 0U) &&
        (sourceTimeMs == s_flight.sourceTimeMs) &&
        (stage == s_flight.stage))
    {
        s_flight.lastUpdateMs = nowMs;
        s_flight.staleReported = 0U;
        ++s_flight.livenessRefreshCount;
        return;
    }

    if ((s_flight.valid != 0U) &&
        ((int32_t)(sourceTimeMs - s_flight.sourceTimeMs) <= 0))
    {
        ++s_flight.invalidFrameCount;
        ++s_radioAckDropCount;
        return;
    }

    s_flight.valid = 1U;
    s_flight.stage = stage;
    s_flight.sourceTimeMs = sourceTimeMs;
    s_flight.lastUpdateMs = nowMs;
    s_flight.staleReported = 0U;
    if (stage == V22_MISSION_STAGE_FOLLOW)
    {
        /* Preserve FOLLOW in the frozen evidence even if the aircraft
         * advances to DROP_ALIGN first.  It is deliberately not a motor
         * release condition: the car releases locally at 20 s. */
        s_flight.followSeen = 1U;
    }
    if ((s_taskType == LINE_TASK_DROP) &&
        (stage == V22_MISSION_STAGE_DROP_ACTION))
    {
        s_flight.dropActionSeen = 1U;
    }
    if (stage == V22_MISSION_STAGE_ABORT)
    {
        s_flight.abortSeen = 1U;
        if (!((s_taskType == LINE_TASK_DYNAMIC_LANDING) &&
              (s_task2LocalDSpeedLatched != 0U)))
        {
            s_coordinationSpeedUnlocked = 0U;
        }
    }
    if (changed != 0U)
    {
        LineMission_WriteFlightEvent("flight_stage", reason, nowMs);
    }
    LineMission_TryUnlockCoordinationSpeed(nowMs);
    if (changed != 0U)
    {
        LineMission_RecordFlightTrace(originType, stage, sourceTimeMs, nowMs);
    }
}

static uint32_t LineMission_RadarDistanceSquaredCm(const CarPoseLinkState_t *pose)
{
    int64_t dx;
    int64_t dy;
    uint64_t distanceSquared;

    if (pose == 0)
    {
        return 0xFFFFFFFFUL;
    }

    dx = (int64_t)pose->xCm - s_radarAssist.aXCm;
    dy = (int64_t)pose->yCm - s_radarAssist.aYCm;
    distanceSquared = (uint64_t)(dx * dx) + (uint64_t)(dy * dy);
    return (distanceSquared > 0xFFFFFFFFULL) ?
           0xFFFFFFFFUL : (uint32_t)distanceSquared;
}

static void LineMission_ResetRadarAssist(void)
{
    s_radarAssist.armed = 0U;
    s_radarAssist.bReached = 0U;
    s_radarAssist.dReached = 0U;
    s_radarAssist.aReturnPrepared = 0U;
    s_radarAssist.aStopPrepared = 0U;
    s_radarAssist.invalidReported = 0U;
    s_radarAssist.bReachCandidateCount = 0U;
    s_radarAssist.lastPoseValid = 0U;
    s_radarAssist.calibrationId = 0U;
    s_radarAssist.aXCm = 0L;
    s_radarAssist.aYCm = 0L;
    s_radarAssist.aDistanceSquaredCm = 0U;
    s_radarAssist.lastPoseXCm = 0L;
    s_radarAssist.lastPoseYCm = 0L;
    s_radarAssist.lastPoseFrameMs = 0U;
    s_radarAssist.bReachedMs = 0U;
    s_radarAssist.dReachedMs = 0U;
    s_radarAssist.aPreparedMs = 0U;
}

static void LineMission_ResetFlightState(void)
{
    s_flight.valid = 0U;
    s_flight.stage = V22_MISSION_STAGE_IDLE;
    s_flight.telemetryModeCode = V22_MISSION_STAGE_IDLE;
    s_flight.missionStatusSeen = 0U;
    s_flight.followSeen = 0U;
    s_flight.dropActionSeen = 0U;
    s_flight.abortSeen = 0U;
    s_flight.statusFlags = 0U;
    s_flight.missionId = 0U;
    s_flight.errorCode = 0U;
    s_flight.sourceTimeMs = 0U;
    s_flight.lastUpdateMs = 0U;
    s_flight.staleReported = 0U;
    s_flight.lastMissionStatusMs = 0U;
    s_flight.lastTelemetryMs = 0U;
    s_flight.missionStatusCount = 0U;
    s_flight.telemetryCount = 0U;
    s_flight.livenessRefreshCount = 0U;
    s_flight.invalidFrameCount = 0U;
}

static void LineMission_ResetRadioRxDiagnostics(void)
{
    s_radioRxDiagnostics.rawByteCount = 0U;
    s_radioRxDiagnostics.frameCount = 0U;
    s_radioRxDiagnostics.parseErrorCount = 0U;
    s_radioRxDiagnostics.timeoutCount = 0U;
    s_radioRxDiagnostics.missionStatusSeenCount = 0U;
    s_radioRxDiagnostics.telemetrySeenCount = 0U;
    s_radioRxDiagnostics.missionStatusRejectedCount = 0U;
    s_radioRxDiagnostics.telemetryRejectedCount = 0U;
    s_radioRxDiagnostics.directStageFrameCount = 0U;
    s_radioRxDiagnostics.unsupportedFrameCount = 0U;
    s_radioRxDiagnostics.lastStageType = 0U;
    s_radioRxDiagnostics.lastStageSource = 0U;
    s_radioRxDiagnostics.lastStageDestination = 0U;
    s_radioRxDiagnostics.lastStageSequence = 0U;
    s_radioRxDiagnostics.lastStageFlags = 0U;
    s_radioRxDiagnostics.lastStageLength = 0U;
    s_radioRxDiagnostics.lastStagePayloadValid = 0U;
    s_radioRxDiagnostics.lastStageCode = 0U;
    s_radioRxDiagnostics.lastStageSourceTimeMs = 0U;
}

static void LineMission_ResetTaskSession(LineMissionTaskType_t taskType)
{
    /* The physical run owns its task selection even when the Pi/radar link
     * is unavailable at launch.  A V2.3 task request is queued later, only
     * after raw pose and MCU-local calibration supply a CalibrationId. */
    s_taskType = taskType;
    s_taskState = LINE_TASK_STATE_IDLE;
    s_taskMissionId = 0U;
    s_taskCalibrationId = 0U;
    s_taskSequence = 0U;
    s_taskTxRemaining = 0U;
    s_taskPosePreambleCount = 0U;
    s_taskPosePreambleCalibrationId = 0U;
    s_taskAckResult = 0xFFU;
    s_taskAckDetail = 0xFFU;
    s_taskSourceTimeMs = 0U;
    s_taskAckDeadlineMs = 0U;
    s_coordinationSpeedUnlocked = 0U;
    s_task2LocalDSpeedLatched = 0U;
    s_task2FastEntryBaseCps = 0;
    s_task2FastEntryRampActive = 0U;
    LineMission_ResetFlightState();
    LineMission_ResetFlightTrace();
    LineMission_ResetRadarAssist();
}

static uint8_t LineMission_ArmRadarAssist(uint32_t nowMs)
{
    const CarPoseLinkState_t *pose = CarPoseLink_GetState();

    LineMission_ResetRadarAssist();
    if ((LineMission_IsLocalCalibrationPoseReady(nowMs) == 0U) ||
        (s_localCalibration.calibrationId != s_taskCalibrationId))
    {
        return 0U;
    }

    s_radarAssist.armed = 1U;
    s_radarAssist.calibrationId = s_localCalibration.calibrationId;
    s_radarAssist.aXCm = pose->xCm;
    s_radarAssist.aYCm = pose->yCm;
    s_radarAssist.lastPoseXCm = pose->xCm;
    s_radarAssist.lastPoseYCm = pose->yCm;
    s_radarAssist.lastPoseFrameMs = pose->lastFrameMs;
    s_radarAssist.lastPoseValid = 1U;
    LineMission_WriteRadarEvent("radar_a_captured", "START_GATE", nowMs);
    return 1U;
}

static void LineMission_UpdateRadarAssist(uint32_t nowMs)
{
    const CarPoseLinkState_t *pose;
    int64_t dx;
    int64_t dy;
    uint64_t stepDistanceSquared;
    const uint32_t bDistanceSquared =
        (uint32_t)(LINE_RADAR_B_REACH_DISTANCE_CM *
                   LINE_RADAR_B_REACH_DISTANCE_CM);
    const uint32_t maxStepDistanceSquared =
        (uint32_t)(LINE_RADAR_MAX_STEP_CM * LINE_RADAR_MAX_STEP_CM);
    const uint32_t aPrepareDistanceSquared =
        (uint32_t)(LINE_RADAR_A_PREPARE_RADIUS_CM *
                   LINE_RADAR_A_PREPARE_RADIUS_CM);
    const uint32_t aStopDistanceSquared =
        (uint32_t)(LINE_RADAR_A_STOP_RADIUS_CM *
                   LINE_RADAR_A_STOP_RADIUS_CM);

    if ((s_radarAssist.armed == 0U) || (LineMission_IsActive() == 0U))
    {
        return;
    }
    if (LineMission_IsLocalCalibrationPoseReady(nowMs) == 0U)
    {
        return;
    }

    pose = CarPoseLink_GetState();
    if (s_localCalibration.calibrationId != s_radarAssist.calibrationId)
    {
        if (s_radarAssist.invalidReported == 0U)
        {
            s_radarAssist.invalidReported = 1U;
            LineMission_WriteRadarEvent("radar_assist_disabled",
                                        "CALIBRATION_CHANGED", nowMs);
        }
        s_radarAssist.armed = 0U;
        return;
    }

    /* CarPoseLink is sampled by the control loop more quickly than the Pi
     * sends pose frames.  Only evaluate a new frame once; otherwise the same
     * B-point sample would incorrectly count as multiple confirmations. */
    if ((s_radarAssist.lastPoseValid != 0U) &&
        (pose->lastFrameMs == s_radarAssist.lastPoseFrameMs))
    {
        return;
    }
    dx = (int64_t)pose->xCm - s_radarAssist.lastPoseXCm;
    dy = (int64_t)pose->yCm - s_radarAssist.lastPoseYCm;
    stepDistanceSquared = (uint64_t)(dx * dx) + (uint64_t)(dy * dy);
    if (stepDistanceSquared > maxStepDistanceSquared)
    {
        /* A single implausible jump must not latch B or arm the return-A
         * window.  Keep the last trusted sample as the prediction anchor. */
        if (s_radarAssist.invalidReported == 0U)
        {
            s_radarAssist.invalidReported = 1U;
            LineMission_WriteRadarEvent("radar_pose_jump_rejected",
                                        "STEP_OVER_40CM", nowMs);
        }
        return;
    }
    s_radarAssist.lastPoseXCm = pose->xCm;
    s_radarAssist.lastPoseYCm = pose->yCm;
    s_radarAssist.lastPoseFrameMs = pose->lastFrameMs;
    s_radarAssist.lastPoseValid = 1U;
    s_radarAssist.invalidReported = 0U;

    s_radarAssist.aDistanceSquaredCm = LineMission_RadarDistanceSquaredCm(pose);
    if ((s_radarAssist.bReached == 0U) &&
        (s_radarAssist.aDistanceSquaredCm >= bDistanceSquared))
    {
        if (s_radarAssist.bReachCandidateCount <
            LINE_RADAR_B_CONFIRM_SAMPLES)
        {
            ++s_radarAssist.bReachCandidateCount;
        }
        if (s_radarAssist.bReachCandidateCount >=
            LINE_RADAR_B_CONFIRM_SAMPLES)
        {
            s_radarAssist.bReached = 1U;
            s_radarAssist.bReachedMs = nowMs;
            LineMission_WriteRadarEvent("radar_b_reached",
                                        "A_TO_B_2_FRAMES", nowMs);
            LineMission_TryUnlockCoordinationSpeed(nowMs);
        }
    }
    else if (s_radarAssist.bReached == 0U)
    {
        s_radarAssist.bReachCandidateCount = 0U;
    }

    if ((s_radarAssist.bReached != 0U) &&
        (s_radarAssist.aReturnPrepared == 0U) &&
        (s_radarAssist.aDistanceSquaredCm <= aPrepareDistanceSquared))
    {
        s_radarAssist.aReturnPrepared = 1U;
        s_radarAssist.aPreparedMs = nowMs;
        LineMission_WriteRadarEvent("radar_a_prepare", "RETURN_WINDOW", nowMs);
    }
    if ((s_radarAssist.aReturnPrepared != 0U) &&
        (s_radarAssist.aStopPrepared == 0U) &&
        (s_radarAssist.aDistanceSquaredCm <= aStopDistanceSquared))
    {
        s_radarAssist.aStopPrepared = 1U;
        /* This is deliberately only a pre-arm.  A current centre-gray
         * capture and the existing yaw/path guards remain necessary, so a
         * radar sample cannot stop the car by itself. */
        LineMission_WriteRadarEvent("radar_a_stop_prepare", "A_RADIUS", nowMs);
    }
}

static void LineMission_UpdateDistanceAssist(uint32_t nowMs)
{
    if (((s_taskType != LINE_TASK_DROP) &&
         (s_taskType != LINE_TASK_DYNAMIC_LANDING)) ||
        (LineMission_IsActive() == 0U) ||
        (s_radarAssist.bReached != 0U) ||
        (s_runDistanceMm < LINE_B_ODOMETRY_DISTANCE_MM))
    {
        return;
    }

    /* Encoder path distance is the primary B-progress signal.  Radar B is a
     * corroborating coordinate assist and may arrive earlier or later. */
    s_radarAssist.bReached = 1U;
    s_radarAssist.bReachedMs = nowMs;
    LineMission_WriteRadarEvent("odometry_b_reached", "ENCODER_DISTANCE",
                                nowMs);
    LineMission_TryUnlockCoordinationSpeed(nowMs);
}

static void LineMission_UpdateDProgress(uint32_t nowMs)
{
    if (((s_taskType != LINE_TASK_DROP) &&
         (s_taskType != LINE_TASK_DYNAMIC_LANDING)) ||
        (LineMission_IsActive() == 0U) ||
        (s_radarAssist.dReached != 0U) ||
        (s_flight.abortSeen != 0U) ||
        (s_runDistanceMm < LINE_D_ODOMETRY_DISTANCE_MM) ||
        (s_lapYawTravelValid == 0U) ||
        (LineMission_Abs(s_lapYawTravelTenths) <
         LINE_D_YAW_TRAVEL_TENTHS))
    {
        return;
    }

    /* D follows the A-B-C legs.  The encoder threshold sits beyond the
     * 4608 mm map distance, while two right-angle turns confirm that a long
     * straight/encoder anomaly cannot unlock either task return early. */
    s_radarAssist.dReached = 1U;
    s_radarAssist.dReachedMs = nowMs;
    LineMission_WriteFlightEvent("d_reached", "ENCODER_YAW_FALLBACK", nowMs);
    LineMission_TryUnlockCoordinationSpeed(nowMs);
}

static void LineMission_ReportTaskTwoBTiming(uint32_t nowMs)
{
    uint32_t elapsedMs;

    if ((s_taskType != LINE_TASK_DYNAMIC_LANDING) ||
        (LineMission_IsActive() == 0U) ||
        (s_motionStarted == 0U))
    {
        return;
    }

    elapsedMs = LineMission_RunElapsedMs(nowMs);
    if (s_radarAssist.bReached != 0U)
    {
        if (s_taskTwoBReachedReported == 0U)
        {
            s_taskTwoBReachedReported = 1U;
            LineMission_WriteTimingEvent(
                (elapsedMs <= LINE_TASK2_B_DEADLINE_MS) ?
                    "task2_b_within_15s" : "task2_b_late",
                "RADAR_OR_ODOMETRY", nowMs);
        }
        return;
    }

    if ((elapsedMs >= LINE_TASK2_B_DEADLINE_MS) &&
        (s_taskTwoBDeadlineReported == 0U))
    {
        s_taskTwoBDeadlineReported = 1U;
        /* A missed scoring milestone must not create an extra vehicle stop.
         * Keep completing the lap while preserving an unambiguous record. */
        LineMission_WriteTimingEvent("task2_b_deadline_missed",
                                     "B_NOT_REACHED", nowMs);
    }
}

static uint8_t LineMission_QueueTaskRequest(LineMissionTaskType_t taskType,
                                            uint32_t nowMs)
{
    if ((taskType == LINE_TASK_NONE) ||
        (taskType != s_taskType) ||
        (LineMission_IsLocalCalibrationPoseReady(nowMs) == 0U) ||
        (LineMission_TaskPosePreambleReady(nowMs) == 0U) ||
        (s_taskState != LINE_TASK_STATE_IDLE) ||
        (s_abortTxRemaining != 0U) ||
        (s_abortAckDeadlineMs != 0U))
    {
        return 0U;
    }

    if (s_nextMissionId == 0U)
    {
        s_nextMissionId = 1U;
    }
    s_taskState = LINE_TASK_STATE_PENDING;
    s_taskMissionId = s_nextMissionId++;
    if (s_nextMissionId == 0U)
    {
        s_nextMissionId = 1U;
    }
    s_taskCalibrationId = s_localCalibration.calibrationId;
    s_taskSequence = s_radioSequence++;
    s_taskTxRemaining = LINE_TASK_REQUEST_RETRY_COUNT;
    s_taskAckResult = 0xFFU;
    s_taskAckDetail = 0xFFU;
    s_taskSourceTimeMs = nowMs;
    s_taskAckDeadlineMs = 0U;
    s_coordinationSpeedUnlocked = 0U;
    LineMission_ResetFlightState();
    LineMission_ResetFlightTrace();
    LineMission_WriteTaskEvent("task_request_queued", "RADAR_READY", nowMs);
    return 1U;
}

static void LineMission_TryStartRemoteCoordination(uint32_t nowMs)
{
    if (((LineMission_IsActive() == 0U) &&
         (s_state != LINE_MISSION_START_GATE)) ||
        (s_taskType == LINE_TASK_NONE) ||
        (s_taskState != LINE_TASK_STATE_IDLE))
    {
        return;
    }

    if (LineMission_IsLocalCalibrationPoseReady(nowMs) == 0U)
    {
        /* A gap or changed calibration makes previously sent pose frames
         * ineligible as a task preamble. */
        LineMission_ResetTaskPosePreamble();
        return;
    }

    /* The vehicle may be held at A before release or already following the
     * line. The Pi pose is used here solely to build a valid 0x81 packet. */
    (void)LineMission_QueueTaskRequest(s_taskType, nowMs);
}

static uint8_t LineMission_SendTaskRequestRadio(uint32_t nowMs)
{
    V22Frame_t frame;
    uint8_t encoded[V22_MAX_FRAME_BYTES];
    uint16_t length;

    if ((s_taskState != LINE_TASK_STATE_PENDING) ||
        (s_taskTxRemaining == 0U))
    {
        return 0U;
    }

    frame.version = V22_VERSION;
    frame.type = V22_TYPE_TASK_REQUEST;
    frame.source = V22_ADDR_CAR_RADIO;
    frame.destination = V22_ADDR_AIR_RADIO;
    frame.sequence = s_taskSequence;
    frame.flags = V22_FRAME_FLAG_ACK_REQUIRED;
    if (s_taskTxRemaining < LINE_TASK_REQUEST_RETRY_COUNT)
    {
        frame.flags |= V22_FRAME_FLAG_RETRANSMISSION;
    }
    frame.length = 12U;
    frame.payload[0] = (uint8_t)s_taskType;
    frame.payload[1] = LINE_TASK_REQUEST_FLAGS;
    LineMission_WriteU16Be(&frame.payload[2], s_taskMissionId);
    LineMission_WriteU16Be(&frame.payload[4], s_taskCalibrationId);
    LineMission_WriteU16Be(&frame.payload[6], 0U);
    LineMission_WriteU32Be(&frame.payload[8], s_taskSourceTimeMs);

    length = V22Protocol_Encode(encoded, sizeof(encoded), &frame);
    if (length == 0U)
    {
        return 0U;
    }
    RobotUart_NanoWriteBuffer(encoded, length);
    --s_taskTxRemaining;
    if (s_taskTxRemaining == 0U)
    {
        s_taskState = LINE_TASK_STATE_WAIT_ACK;
        s_taskAckDeadlineMs = nowMs + LINE_TASK_ACK_WINDOW_MS;
    }
    LineMission_WriteTaskEvent("task_request_tx", "V23_0X81", nowMs);
    return 1U;
}

static uint8_t LineMission_SendMissionAbortRadio(uint32_t nowMs)
{
    V22Frame_t frame;
    uint8_t encoded[V22_MAX_FRAME_BYTES];
    uint16_t length;

    if (s_abortTxRemaining == 0U)
    {
        return 0U;
    }

    /* V2.3 reserves type 0x84 for the physical safety key and specifies no
     * payload.  The active TaskType/MissionId remains in the flight-side
     * session; this urgent frame only invalidates that session. */
    frame.version = V22_VERSION;
    frame.type = V22_TYPE_MISSION_ABORT;
    frame.source = V22_ADDR_CAR_RADIO;
    frame.destination = V22_ADDR_AIR_RADIO;
    frame.sequence = s_abortSequence;
    frame.flags = V22_FRAME_FLAG_URGENT | V22_FRAME_FLAG_ACK_REQUIRED;
    if (s_abortTxRemaining < LINE_MISSION_ABORT_RETRY_COUNT)
    {
        frame.flags |= V22_FRAME_FLAG_RETRANSMISSION;
    }
    frame.length = 0U;

    length = V22Protocol_Encode(encoded, sizeof(encoded), &frame);
    if (length == 0U)
    {
        return 0U;
    }
    RobotUart_NanoWriteBuffer(encoded, length);
    --s_abortTxRemaining;
    if (s_abortTxRemaining == 0U)
    {
        s_abortAckDeadlineMs = nowMs + LINE_ABORT_ACK_WINDOW_MS;
    }
    LineMission_WriteTaskEvent("mission_abort_tx", "V23_0X84", nowMs);
    return 1U;
}

static void LineMission_ConsumeRadioAck(const V22Frame_t *frame,
                                        uint32_t nowMs)
{
    uint8_t result;

    if ((frame->type != V22_TYPE_ACK) ||
        (frame->source != V22_ADDR_AIR_RADIO) ||
        (frame->destination != V22_ADDR_CAR_RADIO) ||
        (frame->flags != 0U) || (frame->length != 4U) ||
        (frame->payload[2] > V22_ACK_RESULT_INTERNAL))
    {
        if ((frame->type == V22_TYPE_ACK) &&
            (frame->length >= 1U) &&
            (frame->payload[0] == V22_TYPE_MISSION_ABORT))
        {
            ++s_abortAckDropCount;
        }
        ++s_radioAckDropCount;
        return;
    }

    result = frame->payload[2];
    if ((frame->payload[0] == V22_TYPE_TASK_REQUEST) &&
        (frame->payload[1] == s_taskSequence) &&
        ((s_taskState == LINE_TASK_STATE_PENDING) ||
         (s_taskState == LINE_TASK_STATE_WAIT_ACK)))
    {
        s_taskAckResult = result;
        s_taskAckDetail = frame->payload[3];
        ++s_radioAckFrameCount;
        s_taskState = ((result == V22_ACK_RESULT_ACCEPTED) ||
                       (result == V22_ACK_RESULT_DUPLICATE)) ?
                      LINE_TASK_STATE_ACCEPTED : LINE_TASK_STATE_REJECTED;
        LineMission_WriteTaskEvent("task_ack", "AIR_RADIO", nowMs);
        return;
    }

    if ((frame->payload[0] == V22_TYPE_MISSION_ABORT) &&
        (frame->payload[1] == s_abortSequence) &&
        ((s_abortTxRemaining != 0U) || (s_abortAckDeadlineMs != 0U)))
    {
        s_abortAckResult = result;
        s_abortAckDetail = frame->payload[3];
        s_abortTxRemaining = 0U;
        s_abortAckDeadlineMs = 0U;
        ++s_abortAckFrameCount;
        LineMission_WriteTaskEvent("mission_abort_ack", "AIR_RADIO", nowMs);
        return;
    }

    if (frame->payload[0] == V22_TYPE_MISSION_ABORT)
    {
        ++s_abortAckDropCount;
    }
    ++s_radioAckDropCount;
}

static void LineMission_ConsumeMissionStatus(const V22Frame_t *frame,
                                             uint32_t nowMs)
{
    uint8_t stage;
    uint16_t missionId;

    if ((frame->source != V22_ADDR_AIR_RADIO) ||
        (V22Protocol_DestinationAccepted(frame, V22_ADDR_CAR_RADIO) == 0U) ||
        (frame->flags != 0U) ||
        (frame->length != LINE_MISSION_STATUS_PAYLOAD_LENGTH))
    {
        ++s_flight.invalidFrameCount;
        ++s_radioRxDiagnostics.missionStatusRejectedCount;
        ++s_radioAckDropCount;
        return;
    }

    stage = frame->payload[1];
    missionId = LineMission_ReadU16Be(&frame->payload[4]);
    if ((LineMission_TaskSessionCanConsumeFlight() == 0U) ||
        (frame->payload[0] != (uint8_t)s_taskType) ||
        (missionId != s_taskMissionId) ||
        (frame->payload[7] != 0U) ||
        (LineMission_IsFlightStageValid(stage) == 0U))
    {
        ++s_flight.invalidFrameCount;
        ++s_radioRxDiagnostics.missionStatusRejectedCount;
        ++s_radioAckDropCount;
        return;
    }

    s_flight.missionStatusSeen = 1U;
    s_flight.statusFlags = LineMission_ReadU16Be(&frame->payload[2]);
    s_flight.missionId = missionId;
    s_flight.errorCode = frame->payload[6];
    s_flight.lastMissionStatusMs = nowMs;
    ++s_flight.missionStatusCount;
    LineMission_UpdateFlightStage(stage, LineMission_ReadU32Be(&frame->payload[8]),
                                  V22_TYPE_MISSION_STATUS, "MISSION_STATUS",
                                  nowMs);
}

static void LineMission_ConsumeFlightTelemetry(const V22Frame_t *frame,
                                                uint32_t nowMs)
{
    uint8_t modeCode;

    if ((frame->source != V22_ADDR_AIR_RADIO) ||
        (V22Protocol_DestinationAccepted(frame, V22_ADDR_CAR_RADIO) == 0U) ||
        (frame->flags != 0U) ||
        (frame->length != LINE_FLIGHT_TELEMETRY_PAYLOAD_LENGTH))
    {
        ++s_flight.invalidFrameCount;
        ++s_radioRxDiagnostics.telemetryRejectedCount;
        ++s_radioAckDropCount;
        return;
    }

    modeCode = frame->payload[3];
    if ((frame->payload[2] != V22_COORDINATE_FIELD_GLOBAL) ||
        (LineMission_IsFlightStageValid(modeCode) == 0U))
    {
        ++s_flight.invalidFrameCount;
        ++s_radioRxDiagnostics.telemetryRejectedCount;
        ++s_radioAckDropCount;
        return;
    }

    s_flight.telemetryModeCode = modeCode;
    s_flight.lastTelemetryMs = nowMs;
    ++s_flight.telemetryCount;

    /* A telemetry broadcast carries no TaskType/MissionId.  The matching
     * 0x81 ACK binds it to this local task; a matching MISSION_STATUS adds
     * stronger confirmation but must not be required, otherwise a dropped
     * first 0x82 packet would prevent the specified 2 Hz recovery path. */
    if ((LineMission_TaskSessionCanConsumeFlight() != 0U) &&
        ((s_taskState == LINE_TASK_STATE_ACCEPTED) ||
         (s_flight.missionStatusSeen != 0U)))
    {
        LineMission_UpdateFlightStage(modeCode,
                                      LineMission_ReadU32Be(&frame->payload[20]),
                                      V22_TYPE_FLIGHT_TELEMETRY,
                                      "FLIGHT_TELEMETRY", nowMs);
    }
}

static void LineMission_ConsumeCalibrationSet(const V22Frame_t *frame,
                                               uint32_t nowMs)
{
    int16_t foundIndex;
    uint8_t recordIndex;
    uint8_t replacingLocalCalibration;
    LineMissionCalibrationRecord_t *record;

    if ((frame->type != V22_TYPE_CALIBRATION_SET) ||
        (frame->source != V22_ADDR_GROUND) ||
        (frame->destination != V22_ADDR_CAR_RADIO) ||
        ((frame->flags & V22_FRAME_FLAG_ACK_REQUIRED) == 0U))
    {
        ++s_radioAckDropCount;
        return;
    }

    foundIndex = LineMission_FindCalibrationRecord(frame->source,
                                                    frame->sequence, nowMs);
    if (foundIndex >= 0)
    {
        record = &s_calibrationRecord[(uint8_t)foundIndex];
        record->lastUpdateMs = nowMs;
        if (record->pending == 0U)
        {
            (void)LineMission_QueueRadioAck(record->source,
                                             V22_TYPE_CALIBRATION_SET,
                                             record->sequence,
                                             record->result, record->detail);
            LineMission_WriteCalibrationEvent("calibration_duplicate_ack",
                                              "DEDUP_5S", nowMs);
        }
        return;
    }

    if (LineMission_ReserveCalibrationRecord(frame->source, frame->sequence,
                                             nowMs, &recordIndex) == 0U)
    {
        (void)LineMission_QueueRadioAck(frame->source,
                                         V22_TYPE_CALIBRATION_SET,
                                         frame->sequence,
                                         V22_ACK_RESULT_BUSY, 0U);
        LineMission_WriteCalibrationEvent("calibration_rejected",
                                          "DEDUP_CACHE_FULL", nowMs);
        return;
    }

    record = &s_calibrationRecord[recordIndex];
    if (((frame->flags &
          (uint8_t)~(V22_FRAME_FLAG_ACK_REQUIRED |
                     V22_FRAME_FLAG_RETRANSMISSION)) != 0U) ||
        (frame->length != LINE_CALIBRATION_PAYLOAD_LENGTH) ||
        (LineMission_ReadU16Be(&frame->payload[8]) == 0U) ||
        (frame->payload[10] != V22_CALIBRATION_FLAG_APPLY) ||
        (frame->payload[11] != 0U))
    {
        LineMission_FinishCalibrationRecord(recordIndex,
                                            V22_ACK_RESULT_PARAMETER, 0U,
                                            nowMs);
        LineMission_WriteCalibrationEvent("calibration_rejected",
                                          "INVALID_0X83", nowMs);
        return;
    }

    if (LineMission_IsMissionBusy() != 0U)
    {
        LineMission_FinishCalibrationRecord(recordIndex,
                                            V22_ACK_RESULT_STATE_DENIED, 0U,
                                            nowMs);
        LineMission_WriteCalibrationEvent("calibration_rejected",
                                          "CAR_RUNNING", nowMs);
        return;
    }

    if (s_maintenanceBroadcastRemaining != 0U)
    {
        LineMission_FinishCalibrationRecord(recordIndex, V22_ACK_RESULT_BUSY,
                                            0U, nowMs);
        LineMission_WriteCalibrationEvent("calibration_rejected",
                                          "MAINTENANCE_BUSY", nowMs);
        return;
    }

    /* UART4 is Pi -> MCU raw-pose ingress only.  A new request sequence may
     * deliberately replace an earlier stopped-state calibration for bench
     * debugging.  DeltaX/Y are complete offsets from the raw Pi pose, never
     * increments to the previous calibration. */
    replacingLocalCalibration = s_localCalibration.valid;
    s_localCalibration.valid = 1U;
    s_localCalibration.deltaXCm =
        (int32_t)LineMission_ReadU32Be(&frame->payload[0]);
    s_localCalibration.deltaYCm =
        (int32_t)LineMission_ReadU32Be(&frame->payload[4]);
    s_localCalibration.calibrationId = LineMission_ReadU16Be(&frame->payload[8]);
    LineMission_ResetTaskSession(LINE_TASK_NONE);
    LineMission_FinishCalibrationRecord(recordIndex, V22_ACK_RESULT_ACCEPTED,
                                        0U, nowMs);
    LineMission_WriteCalibrationEvent(
        (replacingLocalCalibration != 0U) ? "calibration_replaced_mcu" :
                                             "calibration_applied_mcu",
        (replacingLocalCalibration != 0U) ? "REPLACE_RAW_OFFSET" :
                                             "GROUND_TO_MCU_LOCAL",
        nowMs);
}

static void LineMission_PollRadioRx(uint32_t nowMs)
{
    V22Frame_t frame;
    V22ParseResult_t result;
    uint8_t byte;
    uint16_t count = 0U;

    result = V22Protocol_ParserCheckTimeout(&s_radioRxParser, nowMs);
    if (result != V22_PARSE_NONE)
    {
        ++s_radioRxDiagnostics.timeoutCount;
        ++s_radioRxDiagnostics.parseErrorCount;
        ++s_radioAckDropCount;
    }
    while ((count < LINE_RADIO_RX_POLL_LIMIT) &&
           (RobotUart_NanoTryReadByte(&byte) != 0U))
    {
        ++s_radioRxDiagnostics.rawByteCount;
        result = V22Protocol_ParserPush(&s_radioRxParser, byte, nowMs, &frame);
        if (result == V22_PARSE_FRAME)
        {
            ++s_radioRxDiagnostics.frameCount;
            if (frame.type == V22_TYPE_ACK)
            {
                LineMission_ConsumeRadioAck(&frame, nowMs);
            }
            else if (frame.type == V22_TYPE_MISSION_STATUS)
            {
                ++s_radioRxDiagnostics.missionStatusSeenCount;
                if (frame.destination == V22_ADDR_CAR_RADIO)
                {
                    ++s_radioRxDiagnostics.directStageFrameCount;
                }
                if (frame.length == LINE_MISSION_STATUS_PAYLOAD_LENGTH)
                {
                    LineMission_RecordRawStageFrame(
                        &frame, 1U, frame.payload[1],
                        LineMission_ReadU32Be(&frame.payload[8]));
                }
                else
                {
                    LineMission_RecordRawStageFrame(&frame, 0U, 0xFFU, 0U);
                }
                LineMission_ConsumeMissionStatus(&frame, nowMs);
            }
            else if (frame.type == V22_TYPE_FLIGHT_TELEMETRY)
            {
                ++s_radioRxDiagnostics.telemetrySeenCount;
                if (frame.destination == V22_ADDR_CAR_RADIO)
                {
                    ++s_radioRxDiagnostics.directStageFrameCount;
                }
                if (frame.length == LINE_FLIGHT_TELEMETRY_PAYLOAD_LENGTH)
                {
                    LineMission_RecordRawStageFrame(
                        &frame, 1U, frame.payload[3],
                        LineMission_ReadU32Be(&frame.payload[20]));
                }
                else
                {
                    LineMission_RecordRawStageFrame(&frame, 0U, 0xFFU, 0U);
                }
                LineMission_ConsumeFlightTelemetry(&frame, nowMs);
            }
            else if (frame.type == V22_TYPE_CALIBRATION_SET)
            {
                LineMission_ConsumeCalibrationSet(&frame, nowMs);
            }
            else
            {
                ++s_radioRxDiagnostics.unsupportedFrameCount;
                ++s_radioAckDropCount;
            }
        }
        else if (result != V22_PARSE_NONE)
        {
            ++s_radioRxDiagnostics.parseErrorCount;
            ++s_radioAckDropCount;
        }
        ++count;
    }
    s_radioUartErrorFlags |= RobotUart_NanoConsumeErrorFlags();

    if ((s_taskState == LINE_TASK_STATE_WAIT_ACK) &&
        ((int32_t)(nowMs - s_taskAckDeadlineMs) >= 0))
    {
        s_taskState = LINE_TASK_STATE_TIMEOUT;
        LineMission_WriteTaskEvent("task_ack_timeout", "NO_AIR_ACK", nowMs);
    }

    if ((s_abortTxRemaining == 0U) &&
        (s_abortAckDeadlineMs != 0U) &&
        ((int32_t)(nowMs - s_abortAckDeadlineMs) >= 0))
    {
        s_abortAckDeadlineMs = 0U;
        ++s_abortAckDropCount;
        LineMission_WriteTaskEvent("mission_abort_ack_timeout",
                                   "NO_AIR_ACK", nowMs);
    }
}

static void LineMission_WriteMaintenanceEvent(const char *event,
                                              const char *reason,
                                              uint32_t nowMs)
{
    DiagUart_WriteString("LF,event=");
    DiagUart_WriteString(event);
    DiagUart_WriteString(",reason=");
    DiagUart_WriteString(reason);
    DiagUart_WriteString(",t_ms=");
    DiagUart_WriteUInt32(nowMs);
    DiagUart_WriteString(",reset_id=");
    DiagUart_WriteUInt32(s_maintenanceResetId);
    DiagUart_WriteString(",state=");
    DiagUart_WriteString(LineMission_StateName());
    DiagUart_WriteString("\r\n");
}

static uint8_t LineMission_RequestMaintenanceReset(uint32_t nowMs)
{
    ++s_maintenanceResetAttemptCount;
    if (LineMission_IsMissionBusy() != 0U)
    {
        ++s_maintenanceResetRunningRejectCount;
        s_maintenanceLastResult = "CAR_RUNNING";
        LineMission_WriteMaintenanceEvent("maintenance_reset_rejected",
                                          "CAR_RUNNING", nowMs);
        return 0U;
    }
    if (s_maintenanceBroadcastRemaining != 0U)
    {
        ++s_maintenanceResetBusyRejectCount;
        s_maintenanceLastResult = "BROADCAST_BUSY";
        LineMission_WriteMaintenanceEvent("maintenance_reset_rejected",
                                          "BUSY", nowMs);
        return 0U;
    }
    if (s_maintenanceNextResetId == 0U)
    {
        s_maintenanceNextResetId = 1U;
    }
    s_maintenanceResetId = s_maintenanceNextResetId;
    s_maintenanceBroadcastSourceTimeMs = nowMs;
    LineMission_ClearLocalCalibration();
    LineMission_ResetTaskSession(LINE_TASK_NONE);
    s_maintenanceBroadcastRemaining = LINE_MAINTENANCE_BROADCAST_COUNT;
    s_maintenanceBroadcastSequence = s_radioSequence++;
    ++s_maintenanceNextResetId;
    if (s_maintenanceNextResetId == 0U)
    {
        s_maintenanceNextResetId = 1U;
    }
    ++s_maintenanceResetSuccessCount;
    s_maintenanceLastResult = "PG12_SHORT_PRESS";
    LineMission_WriteMaintenanceEvent("maintenance_reset_local_applied",
                                      "PG12_SHORT_PRESS", nowMs);
    return 1U;
}

static uint8_t LineMission_SendMaintenanceResetRadio(uint32_t nowMs)
{
    V22Frame_t frame;
    uint8_t encoded[V22_MAX_FRAME_BYTES];
    uint16_t length;

    (void)nowMs;

    /* This is a reset event, so only the successful local-maintenance
     * transaction may arm the three-frame scheduler. */
    if (s_maintenanceBroadcastRemaining == 0U)
    {
        return 0U;
    }

    frame.version = V22_VERSION;
    frame.type = V22_TYPE_MAINTENANCE_RESET;
    frame.source = V22_ADDR_CAR_RADIO;
    frame.destination = V22_ADDR_BROADCAST;
    frame.sequence = s_maintenanceBroadcastSequence;
    frame.flags = (s_maintenanceBroadcastRemaining <
                   LINE_MAINTENANCE_BROADCAST_COUNT) ?
                  V22_FRAME_FLAG_RETRANSMISSION : 0U;
    frame.length = 8U;
    LineMission_WriteU16Be(&frame.payload[0], s_maintenanceResetId);
    frame.payload[2] = V22_MAINT_RESET_FLAG_CLEAR_CALIBRATION;
    frame.payload[3] = 0U;
    /* V2.3 requires the three copies to retain identical payload, sequence
     * and ResetId.  Only the retransmission header flag changes. */
    LineMission_WriteU32Be(&frame.payload[4], s_maintenanceBroadcastSourceTimeMs);
    length = V22Protocol_Encode(encoded, sizeof(encoded), &frame);
    if (length == 0U)
    {
        return 0U;
    }
    RobotUart_NanoWriteBuffer(encoded, length);
    return 1U;
}

static void LineMission_UpdateRadioLink(uint32_t nowMs)
{
    const CarPoseLinkState_t *pose;

    CarPoseLink_Poll(nowMs);
    LineMission_PollRadioRx(nowMs);
    pose = CarPoseLink_GetState();

    if (LineMission_CarSlotDue(nowMs) == 0U)
    {
        return;
    }

    /* A physical emergency stop has priority over every other outbound frame.
     * Send its three copies in three consecutive car slots, then resume the
     * normal pose/task schedule while the ACK window is open. */
    if (s_abortTxRemaining != 0U)
    {
        if (LineMission_SendMissionAbortRadio(nowMs) != 0U)
        {
            LineMission_MarkRadioSlot(nowMs);
        }
        return;
    }

    /* A completed maintenance reset owns three consecutive car slots.  Keep
     * the copies contiguous even if an unrelated directed ACK is queued. */
    if (s_maintenanceBroadcastRemaining != 0U)
    {
        if (LineMission_SendMaintenanceResetRadio(nowMs) != 0U)
        {
            --s_maintenanceBroadcastRemaining;
            LineMission_MarkRadioSlot(nowMs);
            LineMission_WriteMaintenanceEvent("maintenance_reset_radio_tx",
                                              "V23_0X85", nowMs);
        }
        return;
    }

    /* A directed calibration reply takes the next available car slot.  It
     * replaces one pose frame but never transmits in the air response window. */
    if (s_radioAckQueueCount != 0U)
    {
        if ((s_taskType != LINE_TASK_NONE) &&
            (s_taskState == LINE_TASK_STATE_IDLE))
        {
            /* The air side requires consecutive CAR_POSE frames. A directed
             * maintenance reply between them starts the preamble again. */
            LineMission_ResetTaskPosePreamble();
        }
        if (LineMission_SendQueuedRadioAck(nowMs) != 0U)
        {
            LineMission_MarkRadioSlot(nowMs);
        }
        return;
    }

    /* A task request owns the complete slot.  This preserves the protocol's
     * three-consecutive-slot retry rule; CAR_POSE resumes on the next slot. */
    if (s_taskState == LINE_TASK_STATE_PENDING)
    {
        if (LineMission_SendTaskRequestRadio(nowMs) != 0U)
        {
            LineMission_MarkRadioSlot(nowMs);
        }
        return;
    }

    if (CarPoseLink_IsFresh(nowMs, LINE_RADIO_POSE_FRESH_MS) != 0U)
    {
        if (LineMission_SendCarPose(pose) == 0U)
        {
            ++s_radioPoseDropCount;
        }
        else
        {
            LineMission_ObserveTaskPosePreamble(nowMs);
        }
        LineMission_MarkRadioSlot(nowMs);
        return;
    }

    /* A maintenance heartbeat is kept only while no formal pose source is
     * available and the vehicle is stopped. The 55-100 ms guard interval is
     * not polluted during an active run with stale Pi data. */
    if (LineMission_IsActive() == 0U)
    {
        LineMission_UpdateRadioHeartbeat(nowMs);
        LineMission_MarkRadioSlot(nowMs);
    }
}

static void LineMission_SetIndicator(GPIO_TypeDef *port,
                                     uint16_t pin,
                                     uint8_t on)
{
    GPIO_WriteBit(port, pin, (on != 0U) ? Bit_RESET : Bit_SET);
}

static void LineMission_UpdateIndicators(uint32_t nowMs)
{
    LineMission_SetIndicator(
        LINE_LED_GREEN_PORT, LINE_LED_GREEN_PIN,
        ((int32_t)(s_greenLedUntilMs - nowMs) > 0) ? 1U : 0U);
    LineMission_SetIndicator(
        LINE_LED_YELLOW_PORT, LINE_LED_YELLOW_PIN,
        ((int32_t)(s_yellowLedUntilMs - nowMs) > 0) ? 1U : 0U);
    LineMission_SetIndicator(LINE_LED_SPARE_PORT, LINE_LED_SPARE_PIN, 0U);
}

static void LineMission_PulseGreenIndicator(uint32_t nowMs)
{
    s_greenLedUntilMs = nowMs + LINE_BUTTON_GREEN_PULSE_MS;
}

static void LineMission_PulseYellowIndicator(uint32_t nowMs)
{
    s_yellowLedUntilMs = nowMs + LINE_RESET_YELLOW_PULSE_MS;
}

static void LineMission_IndicatorInit(void)
{
    GPIO_InitTypeDef gpio;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOG |
                           RCC_APB2Periph_GPIOE, ENABLE);

    GPIO_StructInit(&gpio);
    gpio.GPIO_Pin = LINE_LED_GREEN_PIN | LINE_LED_SPARE_PIN;
    gpio.GPIO_Mode = GPIO_Mode_Out_PP;
    gpio.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(LINE_LED_GREEN_PORT, &gpio);

    gpio.GPIO_Pin = LINE_LED_YELLOW_PIN;
    GPIO_Init(LINE_LED_YELLOW_PORT, &gpio);

    s_greenLedUntilMs = 0U;
    s_yellowLedUntilMs = 0U;
    LineMission_UpdateIndicators(0U);
}

static uint8_t LineMission_ButtonReadPressed(uint16_t pin)
{
    return (GPIO_ReadInputDataBit(GPIOG, pin) == Bit_RESET) ? 1U : 0U;
}

static void LineMission_ButtonInitOne(LineMissionButton_t *button,
                                      uint16_t pin,
                                      uint32_t nowMs)
{
    uint8_t pressed = LineMission_ButtonReadPressed(pin);

    button->stablePressed = pressed;
    button->candidatePressed = pressed;
    button->candidateSinceMs = nowMs;
}

static uint8_t LineMission_ButtonUpdate(LineMissionButton_t *button,
                                        uint16_t pin,
                                        uint32_t nowMs)
{
    uint8_t rawPressed = LineMission_ButtonReadPressed(pin);

    if (rawPressed != button->candidatePressed)
    {
        button->candidatePressed = rawPressed;
        button->candidateSinceMs = nowMs;
    }

    if ((button->candidatePressed != button->stablePressed) &&
        ((uint32_t)(nowMs - button->candidateSinceMs) >=
         LINE_BUTTON_DEBOUNCE_MS))
    {
        button->stablePressed = button->candidatePressed;
        if (button->stablePressed != 0U)
        {
            s_lastButtonPressMs = nowMs;
            return 1U;
        }
    }
    return 0U;
}

static void LineMission_ButtonInit(uint32_t nowMs)
{
    GPIO_InitTypeDef gpio;

    LineMission_IndicatorInit();
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOG, ENABLE);
    GPIO_StructInit(&gpio);
    /* All task keys are active-low.  PG13 starts task 1, PG9 starts task 2
     * and a subsequent press of either key safely stops an active run.  PG12
     * is the short-press maintenance-reset key.  PG10 is intentionally unused. */
    gpio.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_12 | GPIO_Pin_13;
    gpio.GPIO_Mode = GPIO_Mode_IPU;
    gpio.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(GPIOG, &gpio);

    LineMission_ButtonInitOne(&s_taskOneButton, GPIO_Pin_13, nowMs);
    LineMission_ButtonInitOne(&s_taskTwoButton, GPIO_Pin_9, nowMs);
    LineMission_ButtonInitOne(&s_calibrationButton, GPIO_Pin_12, nowMs);
}

static void LineMission_HandleMaintenanceButton(uint8_t pressedEvent,
                                                uint32_t nowMs)
{
    /* ButtonUpdate supplies a 50 ms debounced press edge.  PG12 is intentionally
     * short-press only so a stopped vehicle can repeat a local reset during
     * calibration work without a second timing gate. */
    if (pressedEvent != 0U)
    {
        LineMission_PulseYellowIndicator(nowMs);
        (void)LineMission_RequestMaintenanceReset(nowMs);
    }
}

static void LineMission_ResetRunControllers(void)
{
    Encoder_Reset();
    s_runDistanceMm = 0U;
    s_lastEncoderProgressMs = 0U;
    s_motionStartMs = 0U;
    s_motionStarted = 0U;
    s_taskTwoBDeadlineReported = 0U;
    s_taskTwoBReachedReported = 0U;
    LineMission_VelocityWindowReset(&s_leftVelocityWindow);
    LineMission_VelocityWindowReset(&s_rightVelocityWindow);
    s_measuredLeftCps = 0;
    s_measuredRightCps = 0;
    s_targetLeftCps = 0;
    s_targetRightCps = 0;
    s_smoothedBaseCps = 0;
    s_smoothedDifferentialCps = 0;
    s_task2FastEntryBaseCps = 0;
    s_task2FastEntryRampActive = 0U;
    s_commandLeftPercent = 0;
    s_commandRightPercent = 0;
    SpeedLadrc_Reset(&s_leftSpeed, 0.0f);
    SpeedLadrc_Reset(&s_rightSpeed, 0.0f);
    s_holdDifferentialCps = 0;
    s_lostYawRecoveryCps = 0;
    s_lastTrackingDifferentialCps = 0;
    s_lastEdgeGrayDifferentialCps = 0;
    s_lostSearchSource = LINE_LOST_SOURCE_NONE;
    LineMission_ResetTurnHistory();
    s_normalLineCount = 0U;
    s_completeMarkCount = 0U;
    s_gyroStaleReported = 0U;
    s_encoderNotLiveReported = 0U;
    s_lapYawTravelValid = 0U;
    s_lastLapYawTenths = 0;
    s_lapYawTravelTenths = 0L;
    s_startYawValid = 0U;
    s_startYawTenths = 0;
    s_motionStarted = 0U;
    s_taskTwoBDeadlineReported = 0U;
    s_taskTwoBReachedReported = 0U;
    s_aReturnEncoderApproachReported = 0U;
    s_runDistanceMm = 0U;
    s_frozenWriteIndex = 0U;
    s_frozenCount = 0U;
    s_frozen = 0U;
    LineMission_ResetFreezeSnapshot();
}

static void LineMission_UpdateMotionStart(uint32_t nowMs)
{
    if ((LineMission_IsActive() == 0U) || (s_motionStarted != 0U))
    {
        return;
    }

    if ((LineMission_Abs(s_measuredLeftCps) < LINE_MOTION_START_MIN_CPS) &&
        (LineMission_Abs(s_measuredRightCps) < LINE_MOTION_START_MIN_CPS))
    {
        return;
    }

    s_motionStarted = 1U;
    s_motionStartMs = nowMs;
    s_lastEncoderProgressMs = nowMs;
    LineMission_WriteTimingEvent("motion_start", "FRONT_ENCODER_CPS", nowMs);
}

static int32_t LineMission_CooperativeBaseSpeedMmPerSec(void)
{
    return (s_taskType == LINE_TASK_DYNAMIC_LANDING) ?
           LINE_TASK2_COOP_SPEED_MM_S : LINE_TASK1_COOP_SPEED_MM_S;
}

static float LineMission_ActiveSpeedOutputMaxPercent(void)
{
    /* The task-2 reserve is for the cooperative A-B segment.  Once the
     * aircraft stage or local D gate opens, return to the verified one-lap
     * envelope and its 45% output ceiling rather than carrying the B-point
     * reserve into the fast return. */
    if ((s_taskType == LINE_TASK_DYNAMIC_LANDING) &&
        (s_coordinationSpeedUnlocked == 0U))
    {
        return LINE_TASK2_SPEED_OUT_MAX;
    }
    return LINE_SPEED_OUT_MAX;
}

static int16_t LineMission_SelectBaseCps(void)
{
    int32_t absError = LineMission_Abs(s_observer.grayErrorX100);
    int32_t cooperativeBaseSpeedMmPerSec =
        LineMission_CooperativeBaseSpeedMmPerSec();
    uint8_t fast = s_coordinationSpeedUnlocked;

    if (s_state == LINE_MISSION_LEAVE_A)
    {
        return LineMission_MmToCps(cooperativeBaseSpeedMmPerSec);
    }
    if (LineMission_IsReturnAEncoderApproach() != 0U)
    {
        /* This cap is encoder/yaw based, so an unavailable or biased radar
         * cannot prevent the vehicle from decelerating before the A marker. */
        return LineMission_MmToCps(LINE_A_RETURN_APPROACH_SPEED_MM_S);
    }
    if (s_state == LINE_MISSION_LOST_HOLD)
    {
        return LineMission_MmToCps((fast != 0U) ?
                                   LINE_POST_COORD_LOST_HOLD_SPEED_MM_S :
                                   LINE_COOP_LOST_HOLD_SPEED_MM_S);
    }
    if (s_observer.lineClass == LINE_OBSERVER_WIDE)
    {
        return LineMission_MmToCps((fast != 0U) ?
                                   LINE_POST_COORD_WIDE_SPEED_MM_S :
                                   LINE_COOP_WIDE_SPEED_MM_S);
    }
    if (absError >= 500L)
    {
        return LineMission_MmToCps((fast != 0U) ?
                                   LINE_POST_COORD_TIGHT_CURVE_SPEED_MM_S :
                                   LINE_COOP_TIGHT_CURVE_SPEED_MM_S);
    }
    if (absError > 300L)
    {
        return LineMission_MmToCps((fast != 0U) ?
                                   LINE_POST_COORD_HARD_CURVE_SPEED_MM_S :
                                   LINE_COOP_HARD_CURVE_SPEED_MM_S);
    }
    if (absError > 100L)
    {
        return LineMission_MmToCps((fast != 0U) ?
                                   LINE_POST_COORD_CURVE_SPEED_MM_S :
                                   LINE_COOP_CURVE_SPEED_MM_S);
    }
    return LineMission_MmToCps((fast != 0U) ?
                               LINE_POST_COORD_BASE_SPEED_MM_S :
                               cooperativeBaseSpeedMmPerSec);
}

static int16_t LineMission_ApplyTask2FastEntryBaseRamp(
    int16_t requestedBaseCps)
{
    int16_t fastBaseCps =
        LineMission_MmToCps(LINE_POST_COORD_BASE_SPEED_MM_S);

    if ((s_taskType != LINE_TASK_DYNAMIC_LANDING) ||
        (s_coordinationSpeedUnlocked == 0U) ||
        (s_task2FastEntryRampActive == 0U))
    {
        return requestedBaseCps;
    }

    if (s_task2FastEntryBaseCps <= 0)
    {
        s_task2FastEntryBaseCps =
            LineMission_MmToCps(LINE_TASK2_COOP_SPEED_MM_S);
    }

    /* Braking requests are never held by the entry ramp.  They still use the
     * pre-existing base-speed slew, so A approach, lost-line and curve safety
     * keep exactly their verified deceleration behaviour. */
    if (requestedBaseCps <= s_task2FastEntryBaseCps)
    {
        s_task2FastEntryBaseCps = requestedBaseCps;
        return requestedBaseCps;
    }

    s_task2FastEntryBaseCps = LineMission_SlewToward(
        s_task2FastEntryBaseCps, requestedBaseCps,
        LINE_TASK2_FAST_ENTRY_BASE_SLEW_CPS_PER_SAMPLE);

    /* The gate operates only for the handoff into the known fast envelope.
     * Once that envelope is reached, subsequent normal curve exits keep the
     * established responsiveness instead of acquiring another slow ramp. */
    if (s_task2FastEntryBaseCps >= fastBaseCps)
    {
        s_task2FastEntryRampActive = 0U;
    }
    return s_task2FastEntryBaseCps;
}

static int16_t LineMission_ApplyTask1DToAStabilityCap(
    int16_t requestedBaseCps)
{
    int16_t capCps =
        LineMission_MmToCps(LINE_TASK1_D_TO_A_STABILITY_SPEED_MM);

    if ((LineMission_IsTask1DToAStabilityCapActive() != 0U) &&
        (requestedBaseCps > capCps))
    {
        return capCps;
    }
    return requestedBaseCps;
}

static int16_t LineMission_SelectLostYawRecoveryCps(void)
{
    /* LineObserver retains the last line tangent through an all-white
     * interval.  Its gyro term therefore has the correct sign to oppose a
     * rotor-vibration/impact yaw disturbance while the gray array recovers. */
    return LineMission_Clamp(
        (int32_t)s_observer.gyroDifferentialCps *
        LINE_LOST_YAW_RECOVERY_GAIN,
        -LINE_LOST_YAW_RECOVERY_LIMIT_CPS,
        LINE_LOST_YAW_RECOVERY_LIMIT_CPS);
}

static void LineMission_ApplySpeedControl(uint32_t samplePeriodMs)
{
    int16_t baseCps;
    int16_t differentialCps;
    float outputMaxPercent = LineMission_ActiveSpeedOutputMaxPercent();
    float samplePeriodSeconds = (float)samplePeriodMs / 1000.0f;

    /* Task 2 has a verified 10-point forward-duty reserve for its B-point
     * time margin. This is an output ceiling, not a fixed PWM offset, so
     * LADRC keeps reducing drive when measured wheel speed reaches target. */
    s_leftSpeed.outputMaxPercent = outputMaxPercent;
    s_rightSpeed.outputMaxPercent = outputMaxPercent;

    baseCps = LineMission_SelectBaseCps();
    baseCps = LineMission_ApplyTask1DToAStabilityCap(baseCps);
    baseCps = LineMission_ApplyTask2FastEntryBaseRamp(baseCps);
    s_lostYawRecoveryCps = 0;
    if (s_state == LINE_MISSION_LOST_HOLD)
    {
        s_lostYawRecoveryCps = LineMission_SelectLostYawRecoveryCps();
        /* Keep history as the recovery-side decision, but blend the bounded
         * yaw term so a long blind turn is damped once yaw has already moved
         * beyond the last valid line tangent. */
        differentialCps = LineMission_Clamp(
            (int32_t)s_holdDifferentialCps + s_lostYawRecoveryCps,
            -LINE_LOST_COMBINED_DIFF_LIMIT_CPS,
            LINE_LOST_COMBINED_DIFF_LIMIT_CPS);
    }
    else
    {
        differentialCps = s_observer.totalDifferentialCps;
    }
    /* Sensor masks can change the requested curve speed/differential in one
     * 20 ms sample. Smooth those requests before the wheel LADRC loops so a
     * line-edge transition does not become a steering jerk. */
    s_smoothedBaseCps = LineMission_SlewToward(
        s_smoothedBaseCps, baseCps, LINE_TARGET_BASE_SLEW_CPS_PER_SAMPLE);
    s_smoothedDifferentialCps = LineMission_SlewToward(
        s_smoothedDifferentialCps, differentialCps,
        LINE_TARGET_DIFFERENTIAL_SLEW_CPS_PER_SAMPLE);
    s_targetLeftCps = LineMission_Clamp((int32_t)s_smoothedBaseCps -
                                        ((int32_t)s_smoothedDifferentialCps / 2L),
                                        LINE_TARGET_REVERSE_MIN_CPS,
                                        LINE_TARGET_MAX_CPS);
    s_targetRightCps = LineMission_Clamp((int32_t)s_smoothedBaseCps +
                                         ((int32_t)s_smoothedDifferentialCps / 2L),
                                         LINE_TARGET_REVERSE_MIN_CPS,
                                         LINE_TARGET_MAX_CPS);

    s_commandLeftPercent = LineMission_RoundPercent(SpeedLadrc_Update(
        &s_leftSpeed, (float)s_targetLeftCps, (float)s_measuredLeftCps,
        samplePeriodSeconds));
    s_commandRightPercent = LineMission_RoundPercent(SpeedLadrc_Update(
        &s_rightSpeed, (float)s_targetRightCps, (float)s_measuredRightCps,
        samplePeriodSeconds));

    /*
     * Apply the measured physical signs to signed logical wheel commands.
     * Extreme edge errors may pull only the inside wheel backward; rear motors
     * remain open-loop followers and are not included in speed feedback.
     */
    s_rearLeftRawPercent = LineMission_RearFollowerPercent(
        s_commandLeftPercent, REAR_LEFT_FORWARD_SIGN,
        (s_motionStarted == 0U) ? LINE_REAR_LEFT_STARTUP_TRIM_PERCENT :
                                  LINE_REAR_LEFT_FORWARD_TRIM_PERCENT);
    s_rearRightRawPercent = LineMission_RearFollowerPercent(
        s_commandRightPercent, REAR_RIGHT_FORWARD_SIGN,
        (s_motionStarted == 0U) ? LINE_REAR_RIGHT_STARTUP_TRIM_PERCENT :
                                  LINE_REAR_RIGHT_FORWARD_TRIM_PERCENT);

    Motor_SetSpeed(MOTOR_LEFT, (int16_t)(s_commandLeftPercent *
                                         FRONT_LEFT_FORWARD_SIGN));
    Motor_SetSpeed(MOTOR_RIGHT, (int16_t)(s_commandRightPercent *
                                          FRONT_RIGHT_FORWARD_SIGN));
    AuxTb6612_SetRawSpeed(AUX_TB6612_MOTOR_A, s_rearLeftRawPercent);
    AuxTb6612_SetRawSpeed(AUX_TB6612_MOTOR_B, s_rearRightRawPercent);
    Motor_Enable(1U);
    AuxTb6612_Enable(1U);
}

static void LineMission_RequestTaskStart(LineMissionTaskType_t taskType,
                                         const char *reason,
                                         uint32_t nowMs)
{
    LineMission_WriteEvent("button_press", reason, nowMs);

    if (LineMission_IsStartMarker() == 0U)
    {
        LineMission_WriteEvent("start_rejected",
                               "REQUIRE_A_FULL_BLACK_OR_CENTER", nowMs);
        return;
    }
    if ((s_taskState == LINE_TASK_STATE_PENDING) ||
        (s_taskState == LINE_TASK_STATE_WAIT_ACK) ||
        (s_abortTxRemaining != 0U) ||
        (s_abortAckDeadlineMs != 0U))
    {
        LineMission_WriteEvent("start_rejected", "RADIO_SESSION_BUSY", nowMs);
        return;
    }

    s_startTaskType = taskType;
    s_startPrepared = 0U;
    LineMission_EnterState(LINE_MISSION_START_GATE, nowMs, reason);
}

static void LineMission_HandleButtons(uint32_t nowMs)
{
    uint8_t taskOnePressed = LineMission_ButtonUpdate(&s_taskOneButton,
                                                       GPIO_Pin_13, nowMs);
    uint8_t taskTwoPressed = LineMission_ButtonUpdate(&s_taskTwoButton,
                                                       GPIO_Pin_9, nowMs);
    uint8_t calibrationPressed = LineMission_ButtonUpdate(&s_calibrationButton,
                                                           GPIO_Pin_12, nowMs);

    LineMission_HandleMaintenanceButton(calibrationPressed, nowMs);

    if ((taskOnePressed == 0U) && (taskTwoPressed == 0U))
    {
        return;
    }

    LineMission_PulseGreenIndicator(nowMs);

    if (LineMission_IsMissionBusy() != 0U)
    {
        /* A second press of either physical task key is the field emergency
         * stop.  Treat either key as safe-stop while running so a mistaken
         * task-key press never starts a different mission mid-run. */
        LineMission_Stop(LINE_MISSION_SAFE_STOP, nowMs,
                         (taskOnePressed != 0U) ? "PG13_MANUAL_STOP" :
                                                   "PG9_MANUAL_STOP");
        return;
    }

    if (taskOnePressed != 0U)
    {
        LineMission_RequestTaskStart(LINE_TASK_DROP, "PG13_TASK1_REQUEST",
                                     nowMs);
    }
    else
    {
        LineMission_RequestTaskStart(LINE_TASK_DYNAMIC_LANDING,
                                     "PG9_TASK2_REQUEST", nowMs);
    }
}

static void LineMission_UpdateRunState(uint32_t nowMs)
{
    const char *aReturnStopReason;

    if (s_state == LINE_MISSION_START_GATE)
    {
        /* A task key arms the air-task session immediately, but propulsion
         * stays disabled for the pre-start safety hold. */
        LineMission_MotorOff();

        if ((LINE_REQUIRE_GYRO_FOR_START != 0) &&
            (s_observer.gyroFresh == 0U))
        {
            if ((uint32_t)(nowMs - s_stateStartMs) >=
                LINE_START_GYRO_WAIT_MS)
            {
                s_startTaskType = LINE_TASK_NONE;
                s_startPrepared = 0U;
                LineMission_ResetTaskSession(LINE_TASK_NONE);
                LineMission_EnterState(LINE_MISSION_IDLE, nowMs,
                                       "GYRO_NOT_FRESH");
            }
            return;
        }

        if (LINE_REQUIRE_CALIBRATED_POSE_FOR_TASK_START == 0)
        {
            /* Local line-follow diagnostics deliberately retain the ability
             * to run from gray/JY901/encoder feedback without a Pi. Radar
             * remains an optional coordination aid in this image. */
            if (s_startPrepared == 0U)
            {
                LineMission_ResetRunControllers();
                LineMission_ResetTaskSession(s_startTaskType);
                if (LineMission_IsLocalCalibrationPoseReady(nowMs) != 0U)
                {
                    s_taskCalibrationId = s_localCalibration.calibrationId;
                    if (LineMission_ArmRadarAssist(nowMs) == 0U)
                    {
                        LineMission_WriteRadarEvent("radar_assist_deferred",
                                                    "A_CAPTURE_FAILED", nowMs);
                    }
                }
                else
                {
                    LineMission_WriteTaskEvent("task_coordination_deferred",
                                               "RADAR_NOT_READY", nowMs);
                }
                LineObserver_ResetHeadingReference(&s_observer);
                LineMission_ResetLapTurnProgress();
                s_startPrepared = 1U;
                LineMission_WriteTaskEvent("task_start_hold",
                                            "WAIT_DIRECT_START_20S", nowMs);
            }

            /* Stay stopped while the three CAR_POSE preamble frames and the
             * optional 0x81 task request are sent to the aircraft. */
            LineMission_TryStartRemoteCoordination(nowMs);
            if ((s_flight.abortSeen != 0U) ||
                (s_taskState == LINE_TASK_STATE_REJECTED))
            {
                LineMission_WriteTaskEvent("task_start_rejected",
                                           (s_flight.abortSeen != 0U) ?
                                               "AIR_ABORT_BEFORE_START" :
                                               "AIR_REJECTED_BEFORE_START",
                                           nowMs);
                s_startTaskType = LINE_TASK_NONE;
                s_startPrepared = 0U;
                LineMission_Stop(LINE_MISSION_SAFE_STOP, nowMs,
                                 "AIR_REJECTED_OR_ABORT_BEFORE_START");
                return;
            }

            if ((uint32_t)(nowMs - s_stateStartMs) <
                LINE_TASK_START_DIRECT_GATE_MS)
            {
                return;
            }

            s_runStartMs = nowMs;
            s_startTaskType = LINE_TASK_NONE;
            s_startPrepared = 0U;
            LineMission_EnterState(LINE_MISSION_LEAVE_A, nowMs,
                                   "DIRECT_START_AFTER_20S");
            return;
        }

        if (s_startPrepared == 0U)
        {
            if (LineMission_IsLocalCalibrationPoseReady(nowMs) == 0U)
            {
                if ((uint32_t)(nowMs - s_stateStartMs) >=
                    LINE_START_COORDINATION_WAIT_MS)
                {
                    LineMission_WriteEvent("start_rejected",
                                           "CALIBRATED_POSE_NOT_READY", nowMs);
                    s_startTaskType = LINE_TASK_NONE;
                    LineMission_ResetTaskSession(LINE_TASK_NONE);
                    LineMission_EnterState(LINE_MISSION_IDLE, nowMs,
                                           "CALIBRATED_POSE_NOT_READY");
                }
                return;
            }

            LineMission_ResetRunControllers();
            LineMission_ResetTaskSession(s_startTaskType);
            s_taskCalibrationId = s_localCalibration.calibrationId;
            if (LineMission_ArmRadarAssist(nowMs) == 0U)
            {
                LineMission_WriteEvent("start_rejected", "A_CAPTURE_FAILED",
                                       nowMs);
                s_startTaskType = LINE_TASK_NONE;
                LineMission_ResetTaskSession(LINE_TASK_NONE);
                LineMission_EnterState(LINE_MISSION_IDLE, nowMs,
                                       "A_CAPTURE_FAILED");
                return;
            }

            LineObserver_ResetHeadingReference(&s_observer);
            LineMission_ResetLapTurnProgress();
            s_startPrepared = 1U;
            LineMission_WriteTaskEvent("task_start_wait",
                                       "CAR_POSE_PREAMBLE", nowMs);
            LineMission_WriteTaskEvent("task_start_hold",
                                        "WAIT_DIRECT_START_20S", nowMs);
            return;
        }

        if (s_taskState == LINE_TASK_STATE_IDLE)
        {
            if (LineMission_IsLocalCalibrationPoseReady(nowMs) == 0U)
            {
                LineMission_ResetTaskPosePreamble();
                return;
            }
            if (LineMission_QueueTaskRequest(s_taskType, nowMs) == 0U)
            {
                return;
            }
            LineMission_WriteTaskEvent("task_start_wait", "WAIT_AIR_ACK",
                                       nowMs);
            return;
        }

        if (s_flight.abortSeen != 0U)
        {
            LineMission_WriteTaskEvent("task_start_rejected",
                                       "AIR_ABORT_BEFORE_START", nowMs);
            s_startTaskType = LINE_TASK_NONE;
            s_startPrepared = 0U;
            LineMission_Stop(LINE_MISSION_SAFE_STOP, nowMs,
                             "AIR_ABORT_BEFORE_START");
            return;
        }

        if (s_taskState == LINE_TASK_STATE_ACCEPTED)
        {
            if ((uint32_t)(nowMs - s_stateStartMs) <
                LINE_TASK_START_DIRECT_GATE_MS)
            {
                return;
            }
            s_runStartMs = nowMs;
            s_startTaskType = LINE_TASK_NONE;
            s_startPrepared = 0U;
            LineMission_EnterState(LINE_MISSION_LEAVE_A, nowMs,
                                   "DIRECT_START_AFTER_20S");
            return;
        }

        if (s_taskState == LINE_TASK_STATE_REJECTED)
        {
            LineMission_WriteTaskEvent("task_start_rejected", "AIR_REJECTED",
                                       nowMs);
            s_startTaskType = LINE_TASK_NONE;
            s_startPrepared = 0U;
            LineMission_ResetTaskSession(LINE_TASK_NONE);
            LineMission_EnterState(LINE_MISSION_IDLE, nowMs,
                                   "AIR_TASK_REJECTED");
            return;
        }

        if ((s_taskState == LINE_TASK_STATE_TIMEOUT) ||
            ((uint32_t)(nowMs - s_stateStartMs) >=
             LINE_START_COORDINATION_WAIT_MS))
        {
            LineMission_WriteTaskEvent("task_start_aborted",
                                       "AIR_TASK_ACK_TIMEOUT", nowMs);
            s_startTaskType = LINE_TASK_NONE;
            s_startPrepared = 0U;
            LineMission_Stop(LINE_MISSION_SAFE_STOP, nowMs,
                             "AIR_TASK_ACK_TIMEOUT");
        }
        return;
    }

    if (LineMission_IsActive() == 0U)
    {
        return;
    }

    /* Once a valid Pi pose arrives, send the 0x81 task request with its
     * CalibrationId.  A delayed link can therefore restore aircraft
     * coordination without ever interrupting local line following. */
    LineMission_TryStartRemoteCoordination(nowMs);

    if ((LINE_REQUIRE_GYRO_FOR_START != 0) &&
        ((s_observer.gyroFresh == 0U) ||
         (s_observer.gyroAgeMs > LINE_GYRO_RUN_STALE_MS)))
    {
        if (s_gyroStaleReported == 0U)
        {
            LineMission_WriteEvent("fault_continue", "GYRO_STALE", nowMs);
            s_gyroStaleReported = 1U;
        }
    }
    else
    {
        s_gyroStaleReported = 0U;
    }
    if (LineMission_RunElapsedMs(nowMs) >= LINE_RUN_WATCHDOG_MS)
    {
        LineMission_WriteTimingEvent("lap_timeout", "RUN_WATCHDOG_90S",
                                     nowMs);
        LineMission_Stop(LINE_MISSION_SAFE_STOP, nowMs, "RUN_WATCHDOG_90S");
        return;
    }
    if (LineMission_EncoderProgressAgeMs(nowMs) >=
        LINE_NO_PROGRESS_WATCHDOG_MS)
    {
        /* Preserve the 90 s scoring window for a moving lap, but do not keep
         * driving a jammed vehicle for that entire interval. */
        LineMission_WriteTimingEvent("progress_timeout",
                                     "NO_ENCODER_PROGRESS_20S", nowMs);
        LineMission_Stop(LINE_MISSION_SAFE_STOP, nowMs,
                         "NO_ENCODER_PROGRESS_20S");
        return;
    }

    if ((s_flight.valid != 0U) &&
        ((uint32_t)(nowMs - s_flight.lastUpdateMs) >=
         LINE_FLIGHT_STATE_STALE_MS))
    {
        if (s_flight.staleReported == 0U)
        {
            s_flight.staleReported = 1U;
            if (s_coordinationSpeedUnlocked != 0U)
            {
                /* The stage gate has already been positively matched.  Do
                 * not erase that one-way return-speed authorization merely
                 * because the later status broadcast is sparse or rejected.
                 * Abort, physical safe-stop, normal completion and reset
                 * still remove motor authority through their own paths. */
                LineMission_WriteFlightEvent("coord_speed_latched",
                                             "FLIGHT_STATE_STALE_KEEP_FAST",
                                             nowMs);
            }
        }
    }

    LineMission_UpdateLapTurnProgress();
    LineMission_UpdateRadarAssist(nowMs);
    LineMission_UpdateDistanceAssist(nowMs);
    LineMission_UpdateDProgress(nowMs);
    LineMission_ReportTaskTwoBTiming(nowMs);

    if ((LineMission_IsReturnAEncoderApproach() != 0U) &&
        (s_aReturnEncoderApproachReported == 0U))
    {
        s_aReturnEncoderApproachReported = 1U;
        LineMission_WriteTimingEvent("a_return_encoder_approach",
                                     "ENCODER_YAW_PREBRAKE", nowMs);
    }

    if ((s_state == LINE_MISSION_TRACK) &&
        (LineMission_IsFollowableLineClass(s_observer.lineClass) != 0U))
    {
        LineMission_RecordTrackingTurn();
    }

    if (s_state == LINE_MISSION_LEAVE_A)
    {
        if (s_observer.lineClass == LINE_OBSERVER_LOST)
        {
            LineMission_EnterLostHold(nowMs);
            return;
        }
        if (s_observer.lineClass == LINE_OBSERVER_TRACK)
        {
            ++s_normalLineCount;
            if (s_normalLineCount >= 3U)
            {
                LineMission_EnterState(LINE_MISSION_TRACK, nowMs, "LEFT_A_MARK");
            }
        }
        else
        {
            s_normalLineCount = 0U;
        }
        if ((uint32_t)(nowMs - s_stateStartMs) >= LINE_LEAVE_A_TIMEOUT_MS)
        {
            /* A noisy/partial A marker must not terminate the mission. Move
             * into normal tracking; an actual all-white loss still enters the
             * 12 s search path on the next control sample. */
            LineMission_EnterState(LINE_MISSION_TRACK, nowMs,
                                   "A_EXIT_TIMEOUT_CONTINUE");
        }
        return;
    }

    if (s_state == LINE_MISSION_LOST_HOLD)
    {
        /* Do not let a brief edge/width reflection cancel the lost-line
         * deadline. The previous history-based turn remains active until a
         * centre sensor sees the tape. At that moment cancel the turn before
         * qualifying the capture: the former 2400 cps hold continued to turn
         * across a recovered centre line and made the 200 ms window fail.
         * Five 20 ms samples still reject a one-frame reflection, while the
         * neutral command lets a real centre capture persist. */
        if ((s_observer.centerCaptureActive != 0U) &&
            (LineMission_IsFollowableLineClass(s_observer.lineClass) != 0U) &&
            (LineMission_Abs(s_observer.grayErrorX100) <=
             LINE_LOST_REACQUIRE_ERROR_X100))
        {
            s_holdDifferentialCps = 0;
            if (s_lostReacquireCount < LINE_LOST_REACQUIRE_SAMPLES)
            {
                ++s_lostReacquireCount;
            }
            if (s_lostReacquireCount >= LINE_LOST_REACQUIRE_SAMPLES)
            {
                s_holdDifferentialCps = 0;
                s_lostReacquireCount = 0U;
                LineMission_EnterState(LINE_MISSION_TRACK, nowMs,
                                       "LINE_REACQUIRED_STABLE");
                return;
            }
        }
        else
        {
            /* A centre glimpse that did not persist must not make the car
             * drive straight into all-white. Restore the last verified turn
             * direction and keep the original 12 s timeout running. */
            if (s_lostReacquireCount != 0U)
            {
                s_holdDifferentialCps = LineMission_SelectLostHoldDifferential();
            }
            s_lostReacquireCount = 0U;
        }
        if ((uint32_t)(nowMs - s_lostStartMs) >= LINE_LOST_TIMEOUT_MS)
        {
            LineMission_Stop(LINE_MISSION_FAULT, nowMs, "LINE_LOST_TIMEOUT");
        }
        return;
    }

    if (s_observer.lineClass == LINE_OBSERVER_LOST)
    {
        /* Keep the last four meaningful steering samples through all-white
         * loss so the search turns toward the side where the line escaped. */
        LineMission_EnterLostHold(nowMs);
        return;
    }

    /* A partial curved-entry mask can also occur elsewhere on the course.
     * The real A label is narrower than two 20 ms samples at field speed, so
     * accept its raw full-black frame only after seven stable black sensors
     * have already formed the approaching A signature. */
    aReturnStopReason = LineMission_ReturnAMarkerReason();
    if (aReturnStopReason != 0)
    {
        ++s_completeMarkCount;
        if ((uint32_t)s_completeMarkCount * LINE_CONTROL_PERIOD_MS >=
            LINE_COMPLETE_MARK_MS)
        {
            LineMission_WriteTimingEvent("lap_complete", aReturnStopReason,
                                         nowMs);
            LineMission_Stop(LINE_MISSION_COMPLETE, nowMs, aReturnStopReason);
        }
    }
    else
    {
        s_completeMarkCount = 0U;
    }
}

void LineFollowMission_Init(uint32_t nowMs)
{
    uint8_t index;

    Motor_Init();
    AuxTb6612_Init();
    Encoder_Init();
    GyroWit_Init(GYRO_BAUDRATE);
    CarPoseLink_Init(RADAR_POSE_UART_BAUDRATE);
    RobotUart_NanoInit(LORA_UART_BAUDRATE);
    LineObserver_Init(&s_observer, nowMs);
    LineMission_ButtonInit(nowMs);
    SpeedLadrc_Init(&s_leftSpeed, LINE_SPEED_B0, LINE_SPEED_WC, LINE_SPEED_WO,
                    LINE_SPEED_OUT_REVERSE_MIN, LINE_SPEED_OUT_MAX,
                    LINE_SPEED_OUT_STEP);
    SpeedLadrc_Init(&s_rightSpeed, LINE_SPEED_B0, LINE_SPEED_WC, LINE_SPEED_WO,
                    LINE_SPEED_OUT_REVERSE_MIN, LINE_SPEED_OUT_MAX,
                    LINE_SPEED_OUT_STEP);
    LineMission_VelocityWindowReset(&s_leftVelocityWindow);
    LineMission_VelocityWindowReset(&s_rightVelocityWindow);
    s_state = LINE_MISSION_IDLE;
    s_lastControlMs = nowMs;
    s_lastLogMs = nowMs;
    s_stateStartMs = nowMs;
    s_runStartMs = nowMs;
    s_motionStartMs = 0U;
    s_lostStartMs = nowMs;
    s_normalLineCount = 0U;
    s_completeMarkCount = 0U;
    s_gyroStaleReported = 0U;
    s_encoderNotLiveReported = 0U;
    s_lapYawTravelValid = 0U;
    s_lastLapYawTenths = 0;
    s_lapYawTravelTenths = 0L;
    s_aReturnEncoderApproachReported = 0U;
    s_radioSequence = 0U;
    s_radioHeartbeatStarted = 0U;
    s_lastRadioHeartbeatMs = 0U;
    s_radioHeartbeatTxCount = 0U;
    s_lastRadioSlot = 0xFFFFFFFFUL;
    s_radioPoseTxCount = 0U;
    s_radioPoseDropCount = 0U;
    V22Protocol_ParserInit(&s_radioRxParser);
    LineMission_ResetRadioRxDiagnostics();
    LineMission_ResetFlightTrace();
    LineMission_ResetFreezeSnapshot();
    s_radioAckFrameCount = 0U;
    s_radioAckDropCount = 0U;
    s_radioUartErrorFlags = 0U;
    s_startTaskType = LINE_TASK_NONE;
    s_startPrepared = 0U;
    s_taskType = LINE_TASK_NONE;
    s_taskState = LINE_TASK_STATE_IDLE;
    s_nextMissionId = 1U;
    s_taskMissionId = 0U;
    s_taskCalibrationId = 0U;
    s_taskSequence = 0U;
    s_taskTxRemaining = 0U;
    s_taskPosePreambleCount = 0U;
    s_taskPosePreambleCalibrationId = 0U;
    s_taskAckResult = 0xFFU;
    s_taskAckDetail = 0xFFU;
    s_taskSourceTimeMs = 0U;
    s_taskAckDeadlineMs = 0U;
    s_abortSequence = 0U;
    s_abortTxRemaining = 0U;
    s_abortAckResult = 0xFFU;
    s_abortAckDetail = 0xFFU;
    s_abortAckDeadlineMs = 0U;
    s_abortAckFrameCount = 0U;
    s_abortAckDropCount = 0U;
    s_coordinationSpeedUnlocked = 0U;
    s_task2LocalDSpeedLatched = 0U;
    s_task2FastEntryBaseCps = 0;
    s_task2FastEntryRampActive = 0U;
    LineMission_ResetFlightState();
    LineMission_ResetRadarAssist();
    s_maintenanceResetId = 0U;
    s_maintenanceNextResetId = 1U;
    s_maintenanceBroadcastRemaining = 0U;
    s_maintenanceBroadcastSequence = 0U;
    s_maintenanceBroadcastSourceTimeMs = 0U;
    s_maintenanceResetAttemptCount = 0U;
    s_maintenanceResetSuccessCount = 0U;
    s_maintenanceResetRunningRejectCount = 0U;
    s_maintenanceResetBusyRejectCount = 0U;
    s_maintenanceLastResult = "NONE";
    LineMission_ClearLocalCalibration();
    for (index = 0U; index < LINE_CALIBRATION_RECORD_COUNT; ++index)
    {
        s_calibrationRecord[index].valid = 0U;
    }
    s_radioAckQueueHead = 0U;
    s_radioAckQueueTail = 0U;
    s_radioAckQueueCount = 0U;
    s_frozenWriteIndex = 0U;
    s_frozenCount = 0U;
    s_frozen = 0U;
    s_lastButtonPressMs = 0U;
    s_lastReason = "BOOT";
    s_holdDifferentialCps = 0;
    s_lostYawRecoveryCps = 0;
    s_lastTrackingDifferentialCps = 0;
    s_lastEdgeGrayDifferentialCps = 0;
    s_lostSearchSource = LINE_LOST_SOURCE_NONE;
    s_targetLeftCps = 0;
    s_targetRightCps = 0;
    s_task2FastEntryBaseCps = 0;
    s_task2FastEntryRampActive = 0U;
    s_measuredLeftCps = 0;
    s_measuredRightCps = 0;
    s_commandLeftPercent = 0;
    s_commandRightPercent = 0;
    s_rearLeftRawPercent = 0;
    s_rearRightRawPercent = 0;
    LineMission_MotorOff();

    DiagUart_WriteString("LF,boot,mode=COMPETITION_LINE_FOLLOW,task1_key=PG13_active_low,task2_key=PG9_active_low,repeat_task_key=manual_stop,maint_key=PG12_short_press_after_stop_mcu_local_reset,");
    DiagUart_WriteString("gray=pc0_pc1_pc2_pg0,white_raw=0,center_mask=24,");
    DiagUart_WriteString("front=pa2_pa3_pe2_pe6,rear=pe13_pe14_pf1_pf4_pb9,enc_front=tim5_tim3,rear=open_loop_follower,");
    DiagUart_WriteString("forward_sign=fl-1_fr-1_rl+1_rr-1,gyro=usart2_remap_pd5_tx_pd6_rx_9600,pi_pose=uart4_raw_one_way_31_to_32,");
    DiagUart_WriteString("radio=uart5_pc12_pd2,task1_base_mm_s=");
    DiagUart_WriteInt32(LINE_TASK1_COOP_SPEED_MM_S);
    DiagUart_WriteString(",task2_base_mm_s=");
    DiagUart_WriteInt32(LINE_TASK2_COOP_SPEED_MM_S);
    DiagUart_WriteString(",rear_follower_gain_pct=");
    DiagUart_WriteUInt32(LINE_REAR_FOLLOW_GAIN_PERCENT);
    DiagUart_WriteString(",rear_follower_forward_min_pct=");
    DiagUart_WriteUInt32(LINE_REAR_FOLLOW_FORWARD_MIN_PERCENT);
    DiagUart_WriteString(",rear_left_forward_trim_pct=");
    DiagUart_WriteInt32(LINE_REAR_LEFT_FORWARD_TRIM_PERCENT);
    DiagUart_WriteString(",rear_left_startup_trim_pct=");
    DiagUart_WriteInt32(LINE_REAR_LEFT_STARTUP_TRIM_PERCENT);
    DiagUart_WriteString(",rear_right_forward_trim_pct=");
    DiagUart_WriteInt32(LINE_REAR_RIGHT_FORWARD_TRIM_PERCENT);
    DiagUart_WriteString(",rear_right_startup_trim_pct=");
    DiagUart_WriteInt32(LINE_REAR_RIGHT_STARTUP_TRIM_PERCENT);
    DiagUart_WriteString(",task_start_direct_gate_ms=");
    DiagUart_WriteUInt32(LINE_TASK_START_DIRECT_GATE_MS);
    DiagUart_WriteString(",task_start_policy=DIRECT_NO_FOLLOW_GATE");
    DiagUart_WriteString(",car_pose_forward_offset_cm=");
    DiagUart_WriteInt32(LINE_CAR_POSE_PLATFORM_FORWARD_OFFSET_CM);
    DiagUart_WriteString(",task1_fast_unlock=DROP_ACTION_THEN_RETURN_HOME_ONLY");
    DiagUart_WriteString(",task2_fast_unlock=PLATFORM_TAKEOFF_OR_LOCAL_D");
    DiagUart_WriteString(",task2_fast_entry_base_slew_cps_per_20ms=");
    DiagUart_WriteUInt32(LINE_TASK2_FAST_ENTRY_BASE_SLEW_CPS_PER_SAMPLE);
    DiagUart_WriteString(",task1_d_to_a_stability_after_mm=");
    DiagUart_WriteUInt32(LINE_TASK1_D_TO_A_STABILITY_DISTANCE_MM);
    DiagUart_WriteString(",task1_d_to_a_stability_speed_mm_s=");
    DiagUart_WriteUInt32(LINE_TASK1_D_TO_A_STABILITY_SPEED_MM);
    DiagUart_WriteString(",task1_base_cps=");
    DiagUart_WriteInt32(LineMission_MmToCps(LINE_TASK1_COOP_SPEED_MM_S));
    DiagUart_WriteString(",task2_base_cps=");
    DiagUart_WriteInt32(LineMission_MmToCps(LINE_TASK2_COOP_SPEED_MM_S));
    DiagUart_WriteString(",gyro_required=");
    DiagUart_WriteUInt32(LINE_REQUIRE_GYRO_FOR_START);
    DiagUart_WriteString(",distance_source=ENCODER_PATH_MM,jy901_yaw_primary=1,b_radar_after_cm=");
    DiagUart_WriteUInt32(LINE_RADAR_B_REACH_DISTANCE_CM);
    DiagUart_WriteString(",b_odometry_after_mm=");
    DiagUart_WriteUInt32(LINE_B_ODOMETRY_DISTANCE_MM);
    DiagUart_WriteString(",a_return_min_distance_mm=");
    DiagUart_WriteUInt32(LINE_A_RETURN_MIN_DISTANCE_MM);
    DiagUart_WriteString(",a_return_encoder_approach_mm=");
    DiagUart_WriteUInt32(LINE_A_RETURN_APPROACH_DISTANCE_MM);
    DiagUart_WriteString(",a_return_approach_speed_mm_s=");
    DiagUart_WriteUInt32(LINE_A_RETURN_APPROACH_SPEED_MM_S);
    DiagUart_WriteString(",radio=pose80_10hz,task81_three_slots,cal83_mcu_local_ack,reset85_mcu_local_three_slots,abort84_urgent,flight02+mission82_rx\r\n");
#if (LINE_ENABLE_UART_MANUAL_STOP != 0)
    DiagUart_WriteString("LF,commands,P=status,F=dump_frozen,S=manual_stop,H=help; task1=PG13,task2=PG9,repeat_task_key=stop,maint_event=PG12_short_press\r\n");
#else
    DiagUart_WriteString("LF,commands,P=status,F=dump_frozen,H=help; task1=PG13,task2=PG9,repeat_task_key=physical_stop,maint_event=PG12_short_press\r\n");
#endif
}

void LineFollowMission_Update(uint32_t nowMs)
{
    uint32_t samplePeriodMs;

    LineMission_UpdateIndicators(nowMs);
    LineMission_UpdateRadioLink(nowMs);
    LineMission_HandleButtons(nowMs);

    samplePeriodMs = nowMs - s_lastControlMs;
    if (samplePeriodMs < LINE_CONTROL_PERIOD_MS)
    {
        return;
    }
    s_lastControlMs = nowMs;

    Encoder_Update(samplePeriodMs);
    s_measuredLeftCps = LineMission_VelocityWindowPush(&s_leftVelocityWindow,
                                                        g_encoderL.count,
                                                        samplePeriodMs);
    s_measuredRightCps = LineMission_VelocityWindowPush(&s_rightVelocityWindow,
                                                         g_encoderR.count,
                                                         samplePeriodMs);
    LineObserver_Update(&s_observer, nowMs);
    LineMission_UpdateRunState(nowMs);
    if (LineMission_IsActive() != 0U)
    {
        LineMission_AccumulateRunDistance(nowMs);
        LineMission_UpdateMotionStart(nowMs);
    }

    if (LineMission_IsActive() != 0U)
    {
        LineMission_ApplySpeedControl(samplePeriodMs);
        if (((uint32_t)(nowMs - s_runStartMs) >= LINE_ENCODER_LIVE_AFTER_MS) &&
            (((LineMission_Abs(s_commandLeftPercent) >=
               LINE_ENCODER_LIVE_CMD_PERCENT) &&
              (LineMission_Abs(s_measuredLeftCps) < LINE_ENCODER_LIVE_MIN_CPS)) ||
             ((LineMission_Abs(s_commandRightPercent) >=
               LINE_ENCODER_LIVE_CMD_PERCENT) &&
              (LineMission_Abs(s_measuredRightCps) < LINE_ENCODER_LIVE_MIN_CPS))))
        {
            if (s_encoderNotLiveReported == 0U)
            {
                LineMission_WriteEvent("fault_continue", "ENCODER_NOT_LIVE",
                                       nowMs);
                s_encoderNotLiveReported = 1U;
            }
        }
        else
        {
            s_encoderNotLiveReported = 0U;
        }
    }
    else
    {
        LineMission_MotorOff();
    }

    if (((uint32_t)(nowMs - s_lastLogMs) >= LINE_LOG_PERIOD_MS) &&
        (LineMission_IsActive() != 0U))
    {
        s_lastLogMs = nowMs;
        LineMission_Record(nowMs);
#if (LINE_LIVE_RECORD_LOG_ENABLE != 0U)
        LineMission_WriteRecord("sample", nowMs);
#endif
    }
}

void LineFollowMission_HandleCommand(char command, uint32_t nowMs)
{
    switch (command)
    {
        case 'F':
        case 'f':
            LineMission_DumpFrozenLog();
            break;
#if (LINE_ENABLE_UART_MANUAL_STOP != 0)
        case 'S':
        case 's':
            if (LineMission_IsActive() != 0U)
            {
                LineMission_Stop(LINE_MISSION_SAFE_STOP, nowMs,
                                 "UART_MANUAL_STOP");
            }
            else
            {
                LineMission_WriteEvent("stop_ignored", "NOT_RUNNING", nowMs);
            }
            break;
#endif
        case 'P':
        case 'p':
            LineMission_WriteStatus(nowMs);
            break;
        case 'H':
        case 'h':
        case '?':
#if (LINE_ENABLE_UART_MANUAL_STOP != 0)
            DiagUart_WriteString("LF,commands,P=status,F=dump_frozen,S=manual_stop,H=help; task1=PG13,task2=PG9,repeat_task_key=stop,maint_event=PG12_short_press\r\n");
#else
            DiagUart_WriteString("LF,commands,P=status,F=dump_frozen,H=help; task1=PG13,task2=PG9,repeat_task_key=physical_stop,maint_event=PG12_short_press\r\n");
#endif
            break;
        default:
            break;
    }
}
