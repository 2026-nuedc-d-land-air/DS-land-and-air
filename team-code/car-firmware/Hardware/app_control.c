#include "app_control.h"
#include <string.h>

#define CONTROL_SAMPLE_MS 10U   /* 控制周期 10ms */
#define CONTROL_ARM_COUNT 3U

/* 速度/转向输出 - 保守设置，先跑稳 */
#define CONTROL_VX_BASE 250     /* 基础速度 - 先用低速 */
#define CONTROL_VX_FAST 280
#define CONTROL_VX_MIN  120

#define CONTROL_VZ_LIMIT 400    /* 转向限幅 - 降低避免过冲 */

/* gap/恢复策略（只影响丢线->重捕线瞬态，不拖慢正常循迹） */
#define CONTROL_RECOVER_CONFIRM_MS 25U  /* 重捕线确认时间 */
#define CONTROL_RECOVER_HOLD_MS    70U  /* 恢复窗口：限制回中斜率 */
#define CONTROL_RECOVER_MAX_VZ_STEP 35  /* 每周期允许的最大vz变化量 */

/* 特征处理参数 */
#define CONTROL_CROSS_CONFIRM_MS 40U
#define CONTROL_CROSS_HOLD_MS    180U

#define CONTROL_GAP_CONFIRM_MS   100U    /* 丢线确认时间加长 */
#define CONTROL_GAP_HOLD_MS      150U
#define CONTROL_TURN_HOLD_MS     300U
#define CONTROL_LOST_STOP_MS     900U   /* 丢线停止时间加长 */

#define CONTROL_ACUTE_EDGE_MS    80U

/* 传感器误差映射 (基于60mm传感器宽度) */
#define LINE_ERROR_MAX 30

/* PID 参数 - 大幅降低，细细补偿 */
/* 
 * 误差范围 -30~+30
 * 目标：误差=30 时，输出约 150-200 (轻微转向)
 * Kp = 150/30 ≈ 5
 */
#define PID_LINE_KP 3.0f        /* 比例系数 - 大幅降低 */
#define PID_LINE_KI 0.01f        /* 积分系数 - 关闭 */
#define PID_LINE_KD 2.0f        /* 微分系数 - 降低 */
#define PID_LINE_I_LIMIT 50.0f  /* 积分限幅 */
#define PID_LINE_OUT_LIMIT (float)CONTROL_VZ_LIMIT

/* 转向策略 - 降低转向力度 */
#define TURN_INPLACE_VX 0
#define TURN_STRONG_VZ  300     /* 强转向降低 */
#define TURN_MID_VZ     200

typedef struct {
    float kp;
    float ki;
    float kd;
    float integral;
    float prevError;
} LinePid_t;

static LinePid_t s_linePid;
static uint32_t s_lastUpdateMs = 0;

static uint8_t s_obstacleMode = 0;
static uint8_t s_obstacleActive = 0;

static uint8_t s_motionArmed = 0;
static uint8_t s_armedCount = 0;

static ControlState_t s_state = CONTROL_STATE_IDLE;

static int8_t s_lastError = 0;
static int8_t s_lastDir = 1;

static uint32_t s_crossCandidateMs = 0;
static uint32_t s_crossHoldUntilMs = 0;

static uint32_t s_gapCandidateMs = 0;

static uint32_t s_recoverCandidateMs = 0;
static uint32_t s_recoverHoldUntilMs = 0;

static uint32_t s_turnStartMs = 0;
static uint32_t s_turnHoldUntilMs = 0;
static int8_t s_turnDir = 1;

static uint32_t s_edgeCandidateMs = 0;
static int8_t s_edgeDir = 1;

static ControlOutput_t s_lastOut;

static float clamp_f(float value, float min, float max)
{
    if (value > max) return max;
    if (value < min) return min;
    return value;
}

static int16_t clamp_i16(int16_t value, int16_t min, int16_t max)
{
    if (value > max) return max;
    if (value < min) return min;
    return value;
}

static int16_t abs_i16(int16_t v)
{
    return (v < 0) ? (int16_t)-v : v;
}

static int8_t sign_i16(int16_t v)
{
    if (v > 0) return 1;
    if (v < 0) return -1;
    return 0;
}

static void LinePid_Init(LinePid_t *pid)
{
    pid->kp = PID_LINE_KP;
    pid->ki = PID_LINE_KI;
    pid->kd = PID_LINE_KD;
    pid->integral = 0.0f;
    pid->prevError = 0.0f;
}

static void LinePid_Reset(LinePid_t *pid)
{
    pid->integral = 0.0f;
    pid->prevError = 0.0f;
}

static float LinePid_Update(LinePid_t *pid, float error, float dt)
{
    float derivative;
    float output;

    if (dt < 0.0005f) dt = 0.0005f;

    pid->integral += error * dt;
    pid->integral = clamp_f(pid->integral, -PID_LINE_I_LIMIT, PID_LINE_I_LIMIT);

    derivative = (error - pid->prevError) / dt;
    output = pid->kp * error + pid->ki * pid->integral + pid->kd * derivative;
    pid->prevError = error;

    return clamp_f(output, -PID_LINE_OUT_LIMIT, PID_LINE_OUT_LIMIT);
}

static uint8_t Track_CountBits(uint8_t data)
{
    uint8_t count = 0;
    for (uint8_t i = 0; i < 8; i++)
    {
        if (data & (1U << i)) count++;
    }
    return count;
}

static int8_t Track_CalcError(uint8_t activeBits)
{
    /*
     * 传感器物理布局 (从图片):
     * - 8路传感器 X1~X8，总宽度 60mm
     * - 间距约 8.6mm (60mm / 7个间隔)
     * - 权重基于物理位置: X1=-30mm, X2=-21mm, ..., X8=+30mm
     * - 归一化到 -30 ~ +30 范围
     */
    static const int8_t weights[8] = {-30, -21, -13, -4, 4, 13, 21, 30};
    static int8_t lastValidError = 0;  /* 保存上次有效误差 */
    int16_t sum = 0;
    uint8_t count = 0;

    for (uint8_t i = 0; i < 8; i++)
    {
        if (activeBits & (1U << (7 - i)))
        {
            sum += weights[i];
            count++;
        }
    }

    /* 全白(count=0)或全黑(count=8)时保持上次误差方向 */
    if (count == 0 || count == 8)
    {
        return lastValidError;
    }

    int8_t error = (int8_t)clamp_i16((int16_t)(sum / (int16_t)count), -LINE_ERROR_MAX, LINE_ERROR_MAX);
    lastValidError = error;
    return error;
}

static uint8_t Control_ConfirmPattern(uint8_t matched, uint32_t *startMs, uint32_t confirmMs, uint32_t nowMs)
{
    if (matched)
    {
        if (*startMs == 0)
        {
            *startMs = nowMs;
        }
        if ((nowMs - *startMs) >= confirmMs)
        {
            return 1;
        }
    }
    else
    {
        *startMs = 0;
    }
    return 0;
}

static int16_t Control_LimitStepI16(int16_t target, int16_t current, int16_t maxStep)
{
    int16_t diff = (int16_t)(target - current);

    if (diff > maxStep) return (int16_t)(current + maxStep);
    if (diff < -maxStep) return (int16_t)(current - maxStep);
    return target;
}

static int8_t Control_DetectHardTurnDir(uint8_t raw, uint8_t activeBits, uint8_t count, int8_t error)
{
    if (raw == 0x0F)
    {
        return -1;
    }
    if (raw == 0xF0)
    {
        return 1;
    }

    if (count > 0)
    {
        uint8_t leftCount = Track_CountBits((uint8_t)(activeBits & 0xF0));
        uint8_t rightCount = Track_CountBits((uint8_t)(activeBits & 0x0F));

        if (leftCount >= 3 && rightCount == 0)
        {
            return -1;
        }
        if (rightCount >= 3 && leftCount == 0)
        {
            return 1;
        }

        if (count <= 2 && abs_i16(error) >= 10)
        {
            return (error >= 0) ? 1 : -1;
        }
    }

    return 0;
}

void Control_Init(void)
{
    memset(&s_lastOut, 0, sizeof(s_lastOut));

    s_lastUpdateMs = 0;

    s_obstacleMode = 0;
    s_obstacleActive = 0;

    s_motionArmed = 0;
    s_armedCount = 0;

    s_state = CONTROL_STATE_IDLE;

    s_lastError = 0;
    s_lastDir = 1;

    s_crossCandidateMs = 0;
    s_crossHoldUntilMs = 0;

    s_gapCandidateMs = 0;

    s_recoverCandidateMs = 0;
    s_recoverHoldUntilMs = 0;

    s_turnStartMs = 0;
    s_turnHoldUntilMs = 0;
    s_turnDir = 1;

    s_edgeCandidateMs = 0;
    s_edgeDir = 1;

    LinePid_Init(&s_linePid);
}

void Control_Reset(void)
{
    Control_Init();
}

void Control_SetObstacleMode(uint8_t enable)
{
    s_obstacleMode = enable ? 1U : 0U;
    s_obstacleActive = 0;
}

uint8_t Control_GetObstacleMode(void)
{
    return s_obstacleMode;
}

uint8_t Control_IsObstacleActive(void)
{
    return s_obstacleActive;
}

ControlOutput_t Control_Update(uint8_t trackData, float gyroZ, float anglePitch, float distanceCm, float speedLeftMms, float speedRightMms, uint8_t gyroValid, uint32_t nowMs)
{
    ControlOutput_t out;
    uint32_t deltaMs;
    float dt;

    uint8_t raw = trackData;
    uint8_t activeBits = (uint8_t)(~raw);
    uint8_t count = Track_CountBits(activeBits);
    int8_t error = Track_CalcError(activeBits);

    uint8_t crossConfirmed;
    uint8_t gapConfirmed;
    int8_t hardDir;

    int16_t vx = 0;
    int16_t vz = 0;

    (void)gyroZ;
    (void)anglePitch;
    (void)distanceCm;
    (void)speedLeftMms;
    (void)speedRightMms;
    (void)gyroValid;

    if (s_lastUpdateMs == 0)
    {
        deltaMs = CONTROL_SAMPLE_MS;
    }
    else
    {
        deltaMs = nowMs - s_lastUpdateMs;
        if (deltaMs == 0)
        {
            deltaMs = CONTROL_SAMPLE_MS;
        }
        if (deltaMs > 200U)
        {
            deltaMs = 200U;
        }
    }
    s_lastUpdateMs = nowMs;

    dt = (float)deltaMs / 1000.0f;

    /* 方向与急转角候选（用于 gap 后的锐角/直角处理） */
    if (raw != 0xFF && raw != 0x00 && count > 0)
    {
        if (error > 0) s_lastDir = 1;
        else if (error < 0) s_lastDir = -1;

        if (abs_i16(error) >= 10)
        {
            s_edgeCandidateMs = nowMs;
            s_edgeDir = (error >= 0) ? 1 : -1;
        }
    }

    /* 轻量 arming：看到非 gap 的循迹数据就算启动 */
    if (!s_motionArmed)
    {
        if (raw != 0xFF)
        {
            if (s_armedCount < 255) s_armedCount++;
        }
        else
        {
            s_armedCount = 0;
        }

        if (s_armedCount >= CONTROL_ARM_COUNT)
        {
            s_motionArmed = 1;
        }
    }

    if (!s_motionArmed)
    {
        memset(&out, 0, sizeof(out));
        out.state = CONTROL_STATE_IDLE;
        s_state = CONTROL_STATE_IDLE;
        s_lastOut = out;
        return out;
    }

    crossConfirmed = Control_ConfirmPattern((raw == 0x00) ? 1U : 0U, &s_crossCandidateMs, CONTROL_CROSS_CONFIRM_MS, nowMs);
    gapConfirmed = Control_ConfirmPattern((raw == 0xFF) ? 1U : 0U, &s_gapCandidateMs, CONTROL_GAP_CONFIRM_MS, nowMs);

    /* 丢线后的重捕线确认：只要不是gap且count>0即算捕线候选 */
    if (raw != 0xFF && raw != 0x00 && count > 0)
    {
        if (s_recoverCandidateMs == 0)
        {
            s_recoverCandidateMs = nowMs;
        }
        if ((nowMs - s_recoverCandidateMs) >= CONTROL_RECOVER_CONFIRM_MS)
        {
            if (nowMs >= s_recoverHoldUntilMs)
            {
                s_recoverHoldUntilMs = nowMs + CONTROL_RECOVER_HOLD_MS;
            }
        }
    }
    else
    {
        s_recoverCandidateMs = 0;
    }

    hardDir = Control_DetectHardTurnDir(raw, activeBits, count, error);
    if (hardDir != 0)
    {
        s_turnDir = hardDir;
        s_turnStartMs = nowMs;
        s_turnHoldUntilMs = nowMs + CONTROL_TURN_HOLD_MS;
        LinePid_Reset(&s_linePid);
    }

    if (crossConfirmed)
    {
        if (nowMs >= s_crossHoldUntilMs)
        {
            s_crossHoldUntilMs = nowMs + CONTROL_CROSS_HOLD_MS;
        }
    }

    /* gap：短暂保持方向；锐角/长丢线进入强转向 */
    if (gapConfirmed)
    {
        uint32_t gapMs = nowMs - s_gapCandidateMs;
        uint8_t acuteEdge = 0;

        if (s_edgeCandidateMs != 0 && (nowMs - s_edgeCandidateMs) <= CONTROL_ACUTE_EDGE_MS)
        {
            acuteEdge = 1;
        }

        if (acuteEdge || gapMs >= CONTROL_GAP_HOLD_MS)
        {
            int8_t dir = s_lastDir;
            if (acuteEdge)
            {
                dir = s_edgeDir;
            }

            if (nowMs >= s_turnHoldUntilMs)
            {
                s_turnDir = (dir == 0) ? 1 : dir;
                s_turnStartMs = nowMs;
                s_turnHoldUntilMs = nowMs + CONTROL_TURN_HOLD_MS;
                LinePid_Reset(&s_linePid);
            }
        }
        else
        {
            s_state = CONTROL_STATE_GAP;
            vx = s_lastOut.vx;
            vz = s_lastOut.vz;
            if (vx <= 0) vx = CONTROL_VX_BASE;
            if (vz == 0) vz = (int16_t)(s_lastDir * TURN_MID_VZ);
        }

        if (gapMs >= CONTROL_LOST_STOP_MS)
        {
            s_state = CONTROL_STATE_STOP;
            vx = 0;
            vz = 0;
        }
    }

    if (nowMs < s_crossHoldUntilMs)
    {
        s_state = CONTROL_STATE_CROSS;
        vx = CONTROL_VX_FAST;
        vz = 0;
        LinePid_Reset(&s_linePid);
    }
    else if (nowMs < s_turnHoldUntilMs)
    {
        s_state = CONTROL_STATE_TURN;
        vx = TURN_INPLACE_VX;
        vz = (int16_t)(s_turnDir * TURN_STRONG_VZ);

        /* 转向中如果重新捕获到线，并且已过最小保持时间，则提前退出 */
        if (raw != 0xFF && raw != 0x00 && count > 0)
        {
            if ((nowMs - s_turnStartMs) >= 80U && abs_i16(error) <= 3)
            {
                s_turnHoldUntilMs = nowMs;
            }
        }
    }
    else if (s_state != CONTROL_STATE_GAP && s_state != CONTROL_STATE_STOP)
    {
        int16_t absError = abs_i16(error);
        float pidOut;

        s_state = CONTROL_STATE_LINE_FOLLOW;

        vx = CONTROL_VX_BASE;
        if (absError <= 1 && count >= 2)
        {
            vx = CONTROL_VX_FAST;
        }
        else if (absError >= 8)
        {
            vx = CONTROL_VX_MIN;
        }

        pidOut = LinePid_Update(&s_linePid, (float)error, dt);
        vz = (int16_t)pidOut;

        s_lastError = error;
    }

    /* 重捕线恢复窗口：限制“回中速度”，但不限制“加大转向速度” */
    if (nowMs < s_recoverHoldUntilMs)
    {
        int16_t prevVz = s_lastOut.vz;
        int16_t targetVz = vz;

        if (abs_i16(targetVz) < abs_i16(prevVz))
        {
            vz = Control_LimitStepI16(targetVz, prevVz, CONTROL_RECOVER_MAX_VZ_STEP);
        }
    }

    vx = clamp_i16(vx, 0, 1000);
    vz = clamp_i16(vz, -1000, 1000);

    memset(&out, 0, sizeof(out));
    out.vx = vx;
    out.vz = vz;
    out.trackError = error;
    out.trackCount = count;
    out.trackLost = (raw == 0xFF) ? 1U : 0U;
    out.isStraight = (raw != 0xFF && raw != 0x00 && abs_i16(error) <= 1 && count >= 2) ? 1U : 0U;
    out.isSlope = 0;
    out.isRepeat = 0;
    out.isLifted = 0;
    out.state = s_state;

    s_lastOut = out;
    return out;
}
