#include "app_protocol.h"
#include "app_control.h"
#include "app_line.h"
#include "app_state.h"
#include "Motor.h"
#include "PWM.h"
#include "Serial.h"
#include <stdint.h>
#include <stdlib.h>

#define PROTO_RESULT_ERROR      0U
#define PROTO_RESULT_OK         1U
#define PROTO_RESULT_IGNORED    2U
#define PROTO_RESULT_NO_TICK    3U

#define WEB_PID_TARGET_MIN      (-200.0f)
#define WEB_PID_TARGET_MAX      (200.0f)

typedef enum
{
    PROTO_ERR_FIELD = 0,
    PROTO_ERR_UNKNOWN,
    PROTO_ERR_BAD_INT,
    PROTO_ERR_BAD_FLOAT,
    PROTO_ERR_RANGE,
    PROTO_ERR_TOO_LONG,
    PROTO_ERR_LOCKED
} ProtocolError_t;

extern volatile float g_lineBlackLevelF;
extern volatile float g_lineReverseOrderF;
extern volatile float g_lineTurnSign;
extern volatile float g_traceBaseSpeed;
extern volatile float g_lineKp;
extern volatile float g_lineKd;
extern volatile float g_lineTurnLimit;
extern volatile float g_lineLostTurn;
extern volatile float g_lineFilterAlpha;
extern volatile float g_lineSlowGain;
extern volatile float g_lineEdgeTurnExtra;
extern volatile float g_lineEdgeSpeedRatio;
extern volatile float g_lineMinTurn;
extern volatile float g_forwardSlewStep;
extern volatile float g_turnSlewStep;

extern volatile float g_forwardKp;
extern volatile float g_forwardKi;
extern volatile float g_forwardKd;
extern volatile float g_turnKp;
extern volatile float g_turnKi;
extern volatile float g_turnKd;
extern volatile float g_maxForwardCmd;
extern volatile float g_maxTurnCmd;

extern volatile WorkMode_t g_workMode;
extern volatile LocalMode_t g_localMode;
extern volatile float g_targetForwardSpeed;
extern volatile float g_targetTurnSpeed;
extern volatile uint8_t g_carEnable;
extern volatile uint32_t g_lastCmdTickMs;
extern volatile uint8_t g_safetyLocked;
extern volatile float g_btSpeedLimitPercent;
extern volatile float g_speedScale;
extern volatile float g_pwmLimit;
extern volatile uint8_t g_plotMode;
extern volatile float g_speedPwm;
extern volatile float g_diffPwm;
extern volatile int16_t g_leftPwm;
extern volatile int16_t g_rightPwm;

extern volatile uint32_t g_protoPacketOkCount;
extern volatile uint32_t g_protoErrFieldCount;
extern volatile uint32_t g_protoErrUnknownCount;
extern volatile uint32_t g_protoErrBadIntCount;
extern volatile uint32_t g_protoErrBadFloatCount;
extern volatile uint32_t g_protoErrRangeCount;
extern volatile uint32_t g_protoErrTooLongCount;
extern volatile uint32_t g_protoErrLockedCount;

extern volatile float g_mpuYawSign;
extern volatile float g_yawKp;
extern volatile float g_yawKd;
extern volatile float g_straightSpeed;

extern volatile float g_task2StraightToSearchPulse;
extern volatile float g_task2ArcWheelDiffTarget;
extern volatile float g_task2ArcMinForwardPulse;
extern volatile uint16_t g_task2LineFoundConfirmMs;
extern volatile uint16_t g_task2LineLostConfirmMs;
extern volatile float g_task2SearchSpeed;
extern volatile float g_task2AlignTurnSpeed;
extern volatile float g_task2BcTurnSign;
extern volatile float g_task2DaTurnSign;
extern volatile uint16_t g_task2AlignWaitTargetMs;
extern volatile float g_task3DiagToSearchPulse;
extern volatile float g_task3DiagTurnDiffTarget;
extern volatile float g_task3CbTurnSign;
extern volatile float g_task3BdTurnSign;
extern volatile float g_task3DaTurnSign;

extern volatile float g_entrySpeed;
extern volatile float g_entryYawKp;
extern volatile float g_entryYawKd;
extern volatile float g_entryTurnLimit;

extern volatile float g_task2BcEntryYaw;
extern volatile float g_task2DaEntryYaw;
extern volatile float g_task3CbEntryYaw;
extern volatile float g_task3DaEntryYaw;

extern volatile float g_arcExitYawWindowDeg;

extern volatile float g_task2BcExitYaw;
extern volatile float g_task2DaExitYaw;
extern volatile float g_task3CbExitYaw;
extern volatile float g_task3DaExitYaw;

extern volatile float g_yawAlignToleranceDeg;
extern volatile float g_yawAlignTurnSpeed;
extern volatile uint16_t g_yawAlignMaxMs;
extern volatile float g_yawAlignMaxWheelDiff;

extern volatile float g_task3AcYawDeg;
extern volatile float g_task3BdYawDeg;

extern volatile float g_task1MinLinePulse;
extern volatile uint8_t g_task1UseLineStop;

static float App_Protocol_LimitFloat(float value, float minVal, float maxVal)
{
    if (value < minVal) return minVal;
    if (value > maxVal) return maxVal;
    return value;
}

static char App_Protocol_LowerChar(char c)
{
    if (c >= 'A' && c <= 'Z') return (char)(c + ('a' - 'A'));
    return c;
}

static int App_Protocol_StrEqualIgnoreCase(const char *a, const char *b)
{
    while (*a != '\0' && *b != '\0')
    {
        if (App_Protocol_LowerChar(*a) != App_Protocol_LowerChar(*b)) return 0;
        a++;
        b++;
    }
    return (*a == '\0' && *b == '\0');
}

static int App_Protocol_IsDown(const char *s)
{
    return App_Protocol_StrEqualIgnoreCase(s, "down") || App_Protocol_StrEqualIgnoreCase(s, "d");
}

static int App_Protocol_IsName(const char *s, const char *a, const char *b, const char *c)
{
    if (App_Protocol_StrEqualIgnoreCase(s, a)) return 1;
    if (b && App_Protocol_StrEqualIgnoreCase(s, b)) return 1;
    if (c && App_Protocol_StrEqualIgnoreCase(s, c)) return 1;
    return 0;
}

static const char *App_Protocol_SkipSpace(const char *s)
{
    while (*s == ' ' || *s == '\t')
    {
        s++;
    }
    return s;
}

static void App_Protocol_RecordError(ProtocolError_t err, const char *reason, uint8_t report)
{
    if (err == PROTO_ERR_FIELD) g_protoErrFieldCount++;
    else if (err == PROTO_ERR_UNKNOWN) g_protoErrUnknownCount++;
    else if (err == PROTO_ERR_BAD_INT) g_protoErrBadIntCount++;
    else if (err == PROTO_ERR_BAD_FLOAT) g_protoErrBadFloatCount++;
    else if (err == PROTO_ERR_RANGE) g_protoErrRangeCount++;
    else if (err == PROTO_ERR_TOO_LONG) g_protoErrTooLongCount++;
    else if (err == PROTO_ERR_LOCKED) g_protoErrLockedCount++;

    if (report)
    {
        Serial_Printf("[status,err,%s]\r\n", reason);
    }
}

static uint8_t App_Protocol_ResultOk(uint8_t report)
{
    g_protoPacketOkCount++;
    if (report)
    {
        Serial_Printf("[status,ok]\r\n");
    }
    return PROTO_RESULT_OK;
}

static uint8_t App_Protocol_ResultNoTick(void)
{
    g_protoPacketOkCount++;
    return PROTO_RESULT_NO_TICK;
}

static uint8_t App_Protocol_ResultIgnored(const char *reason, uint8_t report)
{
    if (report)
    {
        Serial_Printf("[status,ignored,%s]\r\n", reason);
    }
    return PROTO_RESULT_IGNORED;
}

static uint8_t App_Protocol_ParseFloat(const char *text, float *out)
{
    const char *p;
    char *endPtr;
    double value;

    if (text == 0 || out == 0) return 0;

    p = App_Protocol_SkipSpace(text);
    if (*p == '\0') return 0;

    value = strtod(p, &endPtr);
    if (endPtr == p) return 0;

    endPtr = (char *)App_Protocol_SkipSpace(endPtr);
    if (*endPtr != '\0') return 0;
    if (value != value) return 0;
    if (value > 1.0e30 || value < -1.0e30) return 0;

    *out = (float)value;
    return 1;
}

static uint8_t App_Protocol_SetFloatRange(volatile float *dst, float value, float minVal, float maxVal)
{
    if (value < minVal || value > maxVal)
    {
        App_Protocol_RecordError(PROTO_ERR_RANGE, "range", 1U);
        return 0;
    }
    *dst = value;
    return 1;
}

static uint8_t App_Protocol_SetUint16Range(volatile uint16_t *dst, float value, float minVal, float maxVal)
{
    if (value < minVal || value > maxVal)
    {
        App_Protocol_RecordError(PROTO_ERR_RANGE, "range", 1U);
        return 0;
    }
    *dst = (uint16_t)value;
    return 1;
}

static uint8_t App_Protocol_NormalizeRatio(float value, float minVal, float maxVal, float *out)
{
    if (value > 1.0f)
    {
        value = value / 100.0f;
    }

    if (value < minVal || value > maxVal)
    {
        App_Protocol_RecordError(PROTO_ERR_RANGE, "range", 1U);
        return 0;
    }

    *out = value;
    return 1;
}

void App_Protocol_ApplySpeedLimitPercent(float percent)
{
    float ratio;
    percent = App_Protocol_LimitFloat(percent, 0.0f, 100.0f);
    ratio = percent / 100.0f;
    g_btSpeedLimitPercent = percent;
    g_speedScale = ratio;
    g_pwmLimit = (float)PWM_MAX_DUTY * ratio;
}

static void App_Protocol_ApplyFastPreset(void)
{
    App_Protocol_ApplySpeedLimitPercent(55.0f);

    g_traceBaseSpeed = 60.0f;
    g_lineKp = 0.350f;
    g_lineKd = 0.600f;
    g_lineTurnLimit = 180.0f;
    g_lineMinTurn = 34.0f;
    g_lineFilterAlpha = 0.58f;
    g_lineSlowGain = 0.88f;
    g_lineEdgeTurnExtra = 82.0f;
    g_lineEdgeSpeedRatio = 0.24f;
    g_forwardSlewStep = 14.0f;
    g_turnSlewStep = 60.0f;
    g_lineLostTurn = 130.0f;

    g_lineTurnSign = 1.0f;
    g_lineBlackLevelF = 1.0f;
    g_lineReverseOrderF = 0.0f;

    App_Line_ResetState();
    App_Control_ResetPID();
    App_ProtocolPromptStart(220);
}

static uint16_t App_Protocol_FloatToCent(float value)
{
    if (value < 0.0f) value = 0.0f;
    if (value > 600.0f) value = 600.0f;
    return (uint16_t)(value * 100.0f + 0.5f);
}

static int App_Protocol_FloatToIntRound(float value)
{
    if (value >= 0.0f) return (int)(value + 0.5f);
    return (int)(value - 0.5f);
}

static void App_Protocol_WebPidSendValue(void)
{
    uint16_t kpCent;
    uint16_t kiCent;
    uint16_t kdCent;
    int target;

    kpCent = App_Protocol_FloatToCent(g_forwardKp);
    kiCent = App_Protocol_FloatToCent(g_forwardKi);
    kdCent = App_Protocol_FloatToCent(g_forwardKd);
    target = App_Protocol_FloatToIntRound(g_targetForwardSpeed);

    Serial_Printf("[pid,val,kp,%u.%02u,ki,%u.%02u,kd,%u.%02u,target,%d]\r\n",
                  (unsigned int)(kpCent / 100U),
                  (unsigned int)(kpCent % 100U),
                  (unsigned int)(kiCent / 100U),
                  (unsigned int)(kiCent % 100U),
                  (unsigned int)(kdCent / 100U),
                  (unsigned int)(kdCent % 100U),
                  target);
}

static void App_Protocol_WebPidStopControl(void)
{
    App_ProtocolTaskReset();
    if (!g_safetyLocked)
    {
        g_workMode = WORK_BT;
        g_localMode = LOCAL_STANDBY;
    }
    App_ProtocolForcePWMZero();
    App_ProtocolPromptStart(180);
}

static uint8_t App_Protocol_WebPidStartControl(void)
{
    if (g_safetyLocked)
    {
        App_Protocol_RecordError(PROTO_ERR_LOCKED, "locked", 1U);
        return PROTO_RESULT_ERROR;
    }

    App_ProtocolTaskReset();
    g_workMode = WORK_BT;
    g_localMode = LOCAL_STANDBY;
    g_plotMode = PLOT_MODE_WEB_PID;
    g_targetForwardSpeed = App_Protocol_LimitFloat(g_targetForwardSpeed, WEB_PID_TARGET_MIN, WEB_PID_TARGET_MAX);
    g_targetTurnSpeed = 0.0f;
    g_speedPwm = 0.0f;
    g_diffPwm = 0.0f;
    g_leftPwm = 0;
    g_rightPwm = 0;
    g_carEnable = 1;
    Motor_StopAll();
    App_Control_ResetPID();
    App_ProtocolPromptStart(160);
    return App_Protocol_ResultOk(1U);
}

static uint8_t App_Protocol_ApplySliderPacket(const char *name, float value)
{
    float ratio;

    if (App_Protocol_IsName(name, "RP", "rp", "speedLimit"))
    {
        if (value < 0.0f || value > 100.0f)
        {
            App_Protocol_RecordError(PROTO_ERR_RANGE, "range", 1U);
            return PROTO_RESULT_ERROR;
        }
        App_Protocol_ApplySpeedLimitPercent(value);
        return App_Protocol_ResultOk(1U);
    }

    if (App_Protocol_IsName(name, "Kp", "speedKp", "forwardKp") || App_Protocol_StrEqualIgnoreCase(name, "fKp")) { if (!App_Protocol_SetFloatRange(&g_forwardKp, value, 0.0f, 80.0f)) return PROTO_RESULT_ERROR; return App_Protocol_ResultOk(1U); }
    if (App_Protocol_IsName(name, "Ki", "speedKi", "forwardKi") || App_Protocol_StrEqualIgnoreCase(name, "fKi")) { if (!App_Protocol_SetFloatRange(&g_forwardKi, value, 0.0f, 20.0f)) return PROTO_RESULT_ERROR; return App_Protocol_ResultOk(1U); }
    if (App_Protocol_IsName(name, "Kd", "speedKd", "forwardKd") || App_Protocol_StrEqualIgnoreCase(name, "fKd")) { if (!App_Protocol_SetFloatRange(&g_forwardKd, value, 0.0f, 30.0f)) return PROTO_RESULT_ERROR; return App_Protocol_ResultOk(1U); }
    if (App_Protocol_IsName(name, "target", "targetSpeed", "speedTarget")) { if (!App_Protocol_SetFloatRange(&g_targetForwardSpeed, value, WEB_PID_TARGET_MIN, WEB_PID_TARGET_MAX)) return PROTO_RESULT_ERROR; return App_Protocol_ResultOk(1U); }
    if (App_Protocol_IsName(name, "turnKp", "diffKp", "tKp")) { if (!App_Protocol_SetFloatRange(&g_turnKp, value, 0.0f, 80.0f)) return PROTO_RESULT_ERROR; return App_Protocol_ResultOk(1U); }
    if (App_Protocol_IsName(name, "turnKi", "diffKi", "tKi")) { if (!App_Protocol_SetFloatRange(&g_turnKi, value, 0.0f, 20.0f)) return PROTO_RESULT_ERROR; return App_Protocol_ResultOk(1U); }
    if (App_Protocol_IsName(name, "turnKd", "diffKd", "tKd")) { if (!App_Protocol_SetFloatRange(&g_turnKd, value, 0.0f, 30.0f)) return PROTO_RESULT_ERROR; return App_Protocol_ResultOk(1U); }
    if (App_Protocol_IsName(name, "maxForward", "maxSpeed", "btSpeed")) { if (!App_Protocol_SetFloatRange(&g_maxForwardCmd, value, 0.0f, 200.0f)) return PROTO_RESULT_ERROR; return App_Protocol_ResultOk(1U); }
    if (App_Protocol_IsName(name, "maxTurn", "btTurn", "remoteTurn")) { if (!App_Protocol_SetFloatRange(&g_maxTurnCmd, value, 0.0f, 200.0f)) return PROTO_RESULT_ERROR; return App_Protocol_ResultOk(1U); }

    if (App_Protocol_IsName(name, "traceKp", "lineKp", "lineP")) { if (!App_Protocol_SetFloatRange(&g_lineKp, value, 0.0f, 1.0f)) return PROTO_RESULT_ERROR; return App_Protocol_ResultOk(1U); }
    if (App_Protocol_IsName(name, "traceKd", "lineKd", "lineD")) { if (!App_Protocol_SetFloatRange(&g_lineKd, value, 0.0f, 1.0f)) return PROTO_RESULT_ERROR; return App_Protocol_ResultOk(1U); }
    if (App_Protocol_IsName(name, "traceSpeed", "lineSpeed", "baseSpeed")) { if (!App_Protocol_SetFloatRange(&g_traceBaseSpeed, value, 0.0f, 120.0f)) return PROTO_RESULT_ERROR; return App_Protocol_ResultOk(1U); }
    if (App_Protocol_IsName(name, "turnLimit", "lineTurnLimit", "traceTurnLimit")) { if (!App_Protocol_SetFloatRange(&g_lineTurnLimit, value, 0.0f, 180.0f)) return PROTO_RESULT_ERROR; return App_Protocol_ResultOk(1U); }
    if (App_Protocol_IsName(name, "lostTurn", "lineLostTurn", "findTurn")) { if (!App_Protocol_SetFloatRange(&g_lineLostTurn, value, 0.0f, 180.0f)) return PROTO_RESULT_ERROR; return App_Protocol_ResultOk(1U); }

    if (App_Protocol_IsName(name, "filter", "lineFilter", "alpha"))
    {
        if (!App_Protocol_NormalizeRatio(value, 0.0f, 1.0f, &ratio)) return PROTO_RESULT_ERROR;
        g_lineFilterAlpha = ratio;
        return App_Protocol_ResultOk(1U);
    }
    if (App_Protocol_IsName(name, "slowGain", "lineSlow", "curveSlow"))
    {
        if (!App_Protocol_NormalizeRatio(value, 0.0f, 0.95f, &ratio)) return PROTO_RESULT_ERROR;
        g_lineSlowGain = ratio;
        return App_Protocol_ResultOk(1U);
    }
    if (App_Protocol_IsName(name, "edgeBoost", "edgeTurn", "edgeExtra")) { if (!App_Protocol_SetFloatRange(&g_lineEdgeTurnExtra, value, 0.0f, 100.0f)) return PROTO_RESULT_ERROR; return App_Protocol_ResultOk(1U); }
    if (App_Protocol_IsName(name, "edgeSlow", "edgeSpeed", "edgeRatio"))
    {
        if (!App_Protocol_NormalizeRatio(value, 0.05f, 1.0f, &ratio)) return PROTO_RESULT_ERROR;
        g_lineEdgeSpeedRatio = ratio;
        return App_Protocol_ResultOk(1U);
    }
    if (App_Protocol_IsName(name, "minTurn", "lineMinTurn", "traceMinTurn")) { if (!App_Protocol_SetFloatRange(&g_lineMinTurn, value, 0.0f, 80.0f)) return PROTO_RESULT_ERROR; return App_Protocol_ResultOk(1U); }
    if (App_Protocol_IsName(name, "forwardSlew", "speedSlew", "fSlew")) { if (!App_Protocol_SetFloatRange(&g_forwardSlewStep, value, 0.0f, 30.0f)) return PROTO_RESULT_ERROR; return App_Protocol_ResultOk(1U); }
    if (App_Protocol_IsName(name, "turnSlew", "diffSlew", "tSlew")) { if (!App_Protocol_SetFloatRange(&g_turnSlewStep, value, 0.0f, 60.0f)) return PROTO_RESULT_ERROR; return App_Protocol_ResultOk(1U); }
    if (App_Protocol_IsName(name, "lineSign", "traceSign", "turnSign")) { if (value < -1.0f || value > 1.0f) { App_Protocol_RecordError(PROTO_ERR_RANGE, "range", 1U); return PROTO_RESULT_ERROR; } g_lineTurnSign = (value < 0.0f) ? -1.0f : 1.0f; return App_Protocol_ResultOk(1U); }
    if (App_Protocol_IsName(name, "blackLevel", "lineLevel", "black")) { if (value < 0.0f || value > 1.0f) { App_Protocol_RecordError(PROTO_ERR_RANGE, "range", 1U); return PROTO_RESULT_ERROR; } g_lineBlackLevelF = (value <= 0.0f) ? 0.0f : 1.0f; return App_Protocol_ResultOk(1U); }
    if (App_Protocol_IsName(name, "lineReverse", "reverseLine", "sensorReverse")) { if (value < 0.0f || value > 1.0f) { App_Protocol_RecordError(PROTO_ERR_RANGE, "range", 1U); return PROTO_RESULT_ERROR; } g_lineReverseOrderF = (value <= 0.0f) ? 0.0f : 1.0f; return App_Protocol_ResultOk(1U); }

    if (App_Protocol_IsName(name, "yawSign", "gyroSign", "mpuSign")) { if (value < -1.0f || value > 1.0f) { App_Protocol_RecordError(PROTO_ERR_RANGE, "range", 1U); return PROTO_RESULT_ERROR; } g_mpuYawSign = (value < 0.0f) ? -1.0f : 1.0f; return App_Protocol_ResultOk(1U); }
    if (App_Protocol_IsName(name, "yawKp", "headingKp", "straightKp")) { if (!App_Protocol_SetFloatRange(&g_yawKp, value, 0.0f, 20.0f)) return PROTO_RESULT_ERROR; return App_Protocol_ResultOk(1U); }
    if (App_Protocol_IsName(name, "yawKd", "headingKd", "straightKd")) { if (!App_Protocol_SetFloatRange(&g_yawKd, value, 0.0f, 10.0f)) return PROTO_RESULT_ERROR; return App_Protocol_ResultOk(1U); }
    if (App_Protocol_IsName(name, "straightSpeed", "straightV", "lineV")) { if (!App_Protocol_SetFloatRange(&g_straightSpeed, value, 0.0f, 80.0f)) return PROTO_RESULT_ERROR; return App_Protocol_ResultOk(1U); }

    if (App_Protocol_IsName(name, "entrySpeed", "entryV", "arcEntrySpeed")) { if (!App_Protocol_SetFloatRange(&g_entrySpeed, value, 5.0f, 30.0f)) return PROTO_RESULT_ERROR; return App_Protocol_ResultOk(1U); }
    if (App_Protocol_IsName(name, "entryYawKp", "entryKp", "arcEntryKp")) { if (!App_Protocol_SetFloatRange(&g_entryYawKp, value, 0.0f, 8.0f)) return PROTO_RESULT_ERROR; return App_Protocol_ResultOk(1U); }
    if (App_Protocol_IsName(name, "entryYawKd", "entryKd", "arcEntryKd")) { if (!App_Protocol_SetFloatRange(&g_entryYawKd, value, 0.0f, 3.0f)) return PROTO_RESULT_ERROR; return App_Protocol_ResultOk(1U); }
    if (App_Protocol_IsName(name, "entryTurnLimit", "entryTurnMax", "arcEntryTurnLimit")) { if (!App_Protocol_SetFloatRange(&g_entryTurnLimit, value, 20.0f, 300.0f)) return PROTO_RESULT_ERROR; return App_Protocol_ResultOk(1U); }

    if (App_Protocol_IsName(name, "task2BcEntryYaw", "bcEntryYaw", "task2EntryBc")) { if (!App_Protocol_SetFloatRange(&g_task2BcEntryYaw, value, -80.0f, 80.0f)) return PROTO_RESULT_ERROR; return App_Protocol_ResultOk(1U); }
    if (App_Protocol_IsName(name, "task2DaEntryYaw", "daEntryYaw", "task2EntryDa")) { if (!App_Protocol_SetFloatRange(&g_task2DaEntryYaw, value, -80.0f, 80.0f)) return PROTO_RESULT_ERROR; return App_Protocol_ResultOk(1U); }
    if (App_Protocol_IsName(name, "task3CbEntryYaw", "cbEntryYaw", "task3EntryCb")) { if (!App_Protocol_SetFloatRange(&g_task3CbEntryYaw, value, -100.0f, 100.0f)) return PROTO_RESULT_ERROR; return App_Protocol_ResultOk(1U); }
    if (App_Protocol_IsName(name, "task3DaEntryYaw", "task3DaEntry", "task3EntryDa")) { if (!App_Protocol_SetFloatRange(&g_task3DaEntryYaw, value, -100.0f, 100.0f)) return PROTO_RESULT_ERROR; return App_Protocol_ResultOk(1U); }

    if (App_Protocol_IsName(name, "arcExitYawWindow", "exitYawWindow", "arcYawWindow")) { if (!App_Protocol_SetFloatRange(&g_arcExitYawWindowDeg, value, 10.0f, 90.0f)) return PROTO_RESULT_ERROR; return App_Protocol_ResultOk(1U); }

    if (App_Protocol_IsName(name, "task2BcExitYaw", "bcExitYaw", "task2ExitBc")) { if (!App_Protocol_SetFloatRange(&g_task2BcExitYaw, value, -180.0f, 180.0f)) return PROTO_RESULT_ERROR; return App_Protocol_ResultOk(1U); }
    if (App_Protocol_IsName(name, "task2DaExitYaw", "daExitYaw", "task2ExitDa")) { if (!App_Protocol_SetFloatRange(&g_task2DaExitYaw, value, -180.0f, 180.0f)) return PROTO_RESULT_ERROR; return App_Protocol_ResultOk(1U); }
    if (App_Protocol_IsName(name, "task3CbExitYaw", "cbExitYaw", "task3ExitCb")) { if (!App_Protocol_SetFloatRange(&g_task3CbExitYaw, value, -180.0f, 180.0f)) return PROTO_RESULT_ERROR; return App_Protocol_ResultOk(1U); }
    if (App_Protocol_IsName(name, "task3DaExitYaw", "task3DaExit", "task3ExitDa")) { if (!App_Protocol_SetFloatRange(&g_task3DaExitYaw, value, -180.0f, 180.0f)) return PROTO_RESULT_ERROR; return App_Protocol_ResultOk(1U); }

    if (App_Protocol_IsName(name, "yawAlignTol", "yawAlignTolerance", "alignYawTol")) { if (!App_Protocol_SetFloatRange(&g_yawAlignToleranceDeg, value, 1.0f, 15.0f)) return PROTO_RESULT_ERROR; return App_Protocol_ResultOk(1U); }
    if (App_Protocol_IsName(name, "yawAlignTurn", "yawAlignTurnSpeed", "alignYawTurn")) { if (!App_Protocol_SetFloatRange(&g_yawAlignTurnSpeed, value, 2.0f, 30.0f)) return PROTO_RESULT_ERROR; return App_Protocol_ResultOk(1U); }
    if (App_Protocol_IsName(name, "yawAlignMaxMs", "alignYawMaxMs", "yawAlignTime")) { if (!App_Protocol_SetUint16Range(&g_yawAlignMaxMs, value, 500.0f, 8000.0f)) return PROTO_RESULT_ERROR; return App_Protocol_ResultOk(1U); }
    if (App_Protocol_IsName(name, "yawAlignMaxDiff", "yawAlignMaxWheelDiff", "alignYawMaxDiff")) { if (!App_Protocol_SetFloatRange(&g_yawAlignMaxWheelDiff, value, 500.0f, 8000.0f)) return PROTO_RESULT_ERROR; return App_Protocol_ResultOk(1U); }

    if (App_Protocol_IsName(name, "task3AcYaw", "task3AcYawDeg", "acYaw")) { if (!App_Protocol_SetFloatRange(&g_task3AcYawDeg, value, -90.0f, 90.0f)) return PROTO_RESULT_ERROR; return App_Protocol_ResultOk(1U); }
    if (App_Protocol_IsName(name, "task3BdYaw", "task3BdYawDeg", "bdYaw")) { if (!App_Protocol_SetFloatRange(&g_task3BdYawDeg, value, 60.0f, 180.0f)) return PROTO_RESULT_ERROR; return App_Protocol_ResultOk(1U); }

    if (App_Protocol_IsName(name, "task1MinLinePulse", "task1LinePulse", "lineStopPulse")) { if (!App_Protocol_SetFloatRange(&g_task1MinLinePulse, value, 0.0f, 12000.0f)) return PROTO_RESULT_ERROR; return App_Protocol_ResultOk(1U); }
    if (App_Protocol_IsName(name, "task1UseLineStop", "task1LineStop", "lineStopEnable"))
    {
        if (value < 0.0f || value > 1.0f) { App_Protocol_RecordError(PROTO_ERR_RANGE, "range", 1U); return PROTO_RESULT_ERROR; }
        g_task1UseLineStop = (value < 0.5f) ? 0U : 1U;
        return App_Protocol_ResultOk(1U);
    }

    if (App_Protocol_IsName(name, "task2SearchPulse", "searchPulse", "toArcPulse"))
    {
        if (!App_Protocol_SetFloatRange(&g_task2StraightToSearchPulse, value, 1000.0f, 15000.0f)) return PROTO_RESULT_ERROR;
        return App_Protocol_ResultOk(1U);
    }
    if (App_Protocol_IsName(name, "arcWheelDiff", "wheelDiff", "arcDiff"))
    {
        if (!App_Protocol_SetFloatRange(&g_task2ArcWheelDiffTarget, value, 500.0f, 8000.0f)) return PROTO_RESULT_ERROR;
        return App_Protocol_ResultOk(1U);
    }
    if (App_Protocol_IsName(name, "arcMinPulse", "arcForwardPulse", "arcFwdPulse"))
    {
        if (!App_Protocol_SetFloatRange(&g_task2ArcMinForwardPulse, value, 1000.0f, 15000.0f)) return PROTO_RESULT_ERROR;
        return App_Protocol_ResultOk(1U);
    }
    if (App_Protocol_IsName(name, "arcFoundMs", "lineFoundMs", "foundMs"))
    {
        if (!App_Protocol_SetUint16Range(&g_task2LineFoundConfirmMs, value, 10.0f, 2000.0f)) return PROTO_RESULT_ERROR;
        return App_Protocol_ResultOk(1U);
    }
    if (App_Protocol_IsName(name, "arcLostMs", "lineLostMs", "lostMs"))
    {
        if (!App_Protocol_SetUint16Range(&g_task2LineLostConfirmMs, value, 0.0f, 3000.0f)) return PROTO_RESULT_ERROR;
        return App_Protocol_ResultOk(1U);
    }
    if (App_Protocol_IsName(name, "alignWaitMs", "arcWaitMs", "waitAlign"))
    {
        if (!App_Protocol_SetUint16Range(&g_task2AlignWaitTargetMs, value, 0.0f, 3000.0f)) return PROTO_RESULT_ERROR;
        return App_Protocol_ResultOk(1U);
    }
    if (App_Protocol_IsName(name, "toArcSpeed", "arcSearchSpeed", "searchArcSpeed"))
    {
        if (!App_Protocol_SetFloatRange(&g_task2SearchSpeed, value, 0.0f, 60.0f)) return PROTO_RESULT_ERROR;
        return App_Protocol_ResultOk(1U);
    }
    if (App_Protocol_IsName(name, "alignTurn", "alignTurnSpeed", "arcAlignTurn"))
    {
        if (!App_Protocol_SetFloatRange(&g_task2AlignTurnSpeed, value, 0.0f, 80.0f)) return PROTO_RESULT_ERROR;
        return App_Protocol_ResultOk(1U);
    }
    if (App_Protocol_IsName(name, "bcTurnSign", "bcSign", "arcBcSign"))
    {
        if (value < -1.0f || value > 1.0f) { App_Protocol_RecordError(PROTO_ERR_RANGE, "range", 1U); return PROTO_RESULT_ERROR; }
        g_task2BcTurnSign = (value < 0.0f) ? -1.0f : 1.0f;
        return App_Protocol_ResultOk(1U);
    }
    if (App_Protocol_IsName(name, "daTurnSign", "daSign", "arcDaSign"))
    {
        if (value < -1.0f || value > 1.0f) { App_Protocol_RecordError(PROTO_ERR_RANGE, "range", 1U); return PROTO_RESULT_ERROR; }
        g_task2DaTurnSign = (value < 0.0f) ? -1.0f : 1.0f;
        return App_Protocol_ResultOk(1U);
    }
    if (App_Protocol_IsName(name, "task3SearchPulse", "task3DiagPulse", "diagSearchPulse"))
    {
        if (!App_Protocol_SetFloatRange(&g_task3DiagToSearchPulse, value, 3000.0f, 15000.0f)) return PROTO_RESULT_ERROR;
        return App_Protocol_ResultOk(1U);
    }
    if (App_Protocol_IsName(name, "task3TurnDiff", "task3DiagTurnDiff", "bdTurnDiff"))
    {
        if (!App_Protocol_SetFloatRange(&g_task3DiagTurnDiffTarget, value, 100.0f, 3000.0f)) return PROTO_RESULT_ERROR;
        return App_Protocol_ResultOk(1U);
    }
    if (App_Protocol_IsName(name, "task3CbTurnSign", "task3CbSign", "cbTurnSign"))
    {
        if (value < -1.0f || value > 1.0f) { App_Protocol_RecordError(PROTO_ERR_RANGE, "range", 1U); return PROTO_RESULT_ERROR; }
        g_task3CbTurnSign = (value < 0.0f) ? -1.0f : 1.0f;
        return App_Protocol_ResultOk(1U);
    }
    if (App_Protocol_IsName(name, "task3BdTurnSign", "task3BdSign", "bdTurnSign"))
    {
        if (value < -1.0f || value > 1.0f) { App_Protocol_RecordError(PROTO_ERR_RANGE, "range", 1U); return PROTO_RESULT_ERROR; }
        g_task3BdTurnSign = (value < 0.0f) ? -1.0f : 1.0f;
        return App_Protocol_ResultOk(1U);
    }
    if (App_Protocol_IsName(name, "task3DaTurnSign", "task3DaSign", "task3ArcDaSign"))
    {
        if (value < -1.0f || value > 1.0f) { App_Protocol_RecordError(PROTO_ERR_RANGE, "range", 1U); return PROTO_RESULT_ERROR; }
        g_task3DaTurnSign = (value < 0.0f) ? -1.0f : 1.0f;
        return App_Protocol_ResultOk(1U);
    }

    if (App_Protocol_IsName(name, "plotMode", "debugMode", "dbg"))
    {
        if (value < 0.0f || value > (float)PLOT_MODE_WEB_PID)
        {
            App_Protocol_RecordError(PROTO_ERR_RANGE, "range", 1U);
            return PROTO_RESULT_ERROR;
        }
        if (value < 0.5f) g_plotMode = 0U;
        else if (value < 1.5f) g_plotMode = 1U;
        else if (value < 2.5f) g_plotMode = 2U;
        else if (value < 3.5f) g_plotMode = 3U;
        else if (value < 4.5f) g_plotMode = 4U;
        else g_plotMode = PLOT_MODE_WEB_PID;
        return App_Protocol_ResultOk(1U);
    }

    App_Protocol_RecordError(PROTO_ERR_UNKNOWN, "unknown", 1U);
    return PROTO_RESULT_ERROR;
}

static uint8_t App_Protocol_ApplyJoystickPacket(char **tok, int n)
{
    (void)tok;
    (void)n;
    return App_Protocol_ResultIgnored("joystick_remote_disabled", 1U);
}

static uint8_t App_Protocol_WebPidApplySet(const char *name, float value)
{
    if (App_Protocol_IsName(name, "kp", "Kp", "speedKp"))
    {
        if (!App_Protocol_SetFloatRange(&g_forwardKp, value, 0.0f, 80.0f)) return PROTO_RESULT_ERROR;
        return App_Protocol_ResultOk(1U);
    }
    if (App_Protocol_IsName(name, "ki", "Ki", "speedKi"))
    {
        if (!App_Protocol_SetFloatRange(&g_forwardKi, value, 0.0f, 20.0f)) return PROTO_RESULT_ERROR;
        return App_Protocol_ResultOk(1U);
    }
    if (App_Protocol_IsName(name, "kd", "Kd", "speedKd"))
    {
        if (!App_Protocol_SetFloatRange(&g_forwardKd, value, 0.0f, 30.0f)) return PROTO_RESULT_ERROR;
        return App_Protocol_ResultOk(1U);
    }
    if (App_Protocol_IsName(name, "target", "targetSpeed", "speedTarget"))
    {
        if (!App_Protocol_SetFloatRange(&g_targetForwardSpeed, value, WEB_PID_TARGET_MIN, WEB_PID_TARGET_MAX)) return PROTO_RESULT_ERROR;
        return App_Protocol_ResultOk(1U);
    }

    App_Protocol_RecordError(PROTO_ERR_UNKNOWN, "unknown", 1U);
    return PROTO_RESULT_ERROR;
}

static uint8_t App_Protocol_ApplyPidPacket(char **tok, int n)
{
    float value;

    if (n < 2)
    {
        App_Protocol_RecordError(PROTO_ERR_FIELD, "field", 1U);
        return PROTO_RESULT_ERROR;
    }

    if (App_Protocol_IsName(tok[1], "set", "s", 0))
    {
        if (n < 4)
        {
            App_Protocol_RecordError(PROTO_ERR_FIELD, "field", 1U);
            return PROTO_RESULT_ERROR;
        }
        if (!App_Protocol_ParseFloat(tok[3], &value))
        {
            App_Protocol_RecordError(PROTO_ERR_BAD_FLOAT, "bad-number", 1U);
            return PROTO_RESULT_ERROR;
        }
        return App_Protocol_WebPidApplySet(tok[2], value);
    }

    if (App_Protocol_IsName(tok[1], "start", "run", 0))
    {
        return App_Protocol_WebPidStartControl();
    }

    if (App_Protocol_IsName(tok[1], "stop", "halt", 0))
    {
        App_Protocol_WebPidStopControl();
        return App_Protocol_ResultOk(1U);
    }

    if (App_Protocol_IsName(tok[1], "get", "val", "read"))
    {
        App_Protocol_WebPidSendValue();
        return App_Protocol_ResultNoTick();
    }

    App_Protocol_RecordError(PROTO_ERR_UNKNOWN, "unknown", 1U);
    return PROTO_RESULT_ERROR;
}

static uint8_t App_Protocol_ApplyPacket(char *payload)
{
    char *tok[12];
    int n;
    char *p;
    float value;

    n = 0;
    p = payload;
    tok[n++] = p;
    while (*p != '\0' && n < 12)
    {
        if (*p == ',')
        {
            *p = '\0';
            tok[n++] = p + 1;
        }
        p++;
    }

    if (n <= 0 || tok[0][0] == '\0')
    {
        App_Protocol_RecordError(PROTO_ERR_FIELD, "field", 1U);
        return PROTO_RESULT_ERROR;
    }

    if (App_Protocol_IsName(tok[0], "key", "k", 0))
    {
        if (n < 3)
        {
            App_Protocol_RecordError(PROTO_ERR_FIELD, "field", 1U);
            return PROTO_RESULT_ERROR;
        }

        if (!App_Protocol_IsDown(tok[2]))
        {
            return PROTO_RESULT_IGNORED;
        }

        if (App_Protocol_IsName(tok[1], "emergency", "emg", 0)) { App_EmergencyStop(); return App_Protocol_ResultOk(1U); }
        if (App_Protocol_IsName(tok[1], "unlock", "release", "resume")) { App_UnlockControl(); return App_Protocol_ResultOk(1U); }

        if (g_safetyLocked)
        {
            App_Protocol_RecordError(PROTO_ERR_LOCKED, "locked", 1U);
            return PROTO_RESULT_ERROR;
        }

        if (App_Protocol_IsName(tok[1], "presetFast", "fast", "fastPreset")) { App_Protocol_ApplyFastPreset(); return App_Protocol_ResultOk(1U); }
        if (App_Protocol_IsName(tok[1], "encClear", "encoderClear", "clearEnc")) { App_ProtocolEncoderDebugClearTotals(); return App_Protocol_ResultOk(1U); }
        if (App_Protocol_IsName(tok[1], "encDebug", "encoderDebug", "encDbg")) { App_ProtocolEnterEncoderDebug(); return App_Protocol_ResultOk(1U); }
        if (App_Protocol_IsName(tok[1], "mpuDebug", "gyroDebug", "yawDebug")) { App_ProtocolEnterMpuDebug(); return App_Protocol_ResultOk(1U); }
        if (App_Protocol_IsName(tok[1], "mpuCalib", "gyroCalib", "calib")) { g_plotMode = 2U; App_ProtocolMpuCalibrateGyroZ(); return App_Protocol_ResultOk(1U); }
        if (App_Protocol_IsName(tok[1], "yawZero", "mpuZero", "zeroYaw")) { App_ProtocolMpuResetYaw(); App_ProtocolPromptStart(180); return App_Protocol_ResultOk(1U); }
        if (App_Protocol_IsName(tok[1], "task1", "selectTask1", "t1")) { App_ProtocolSelectTask1(); return App_Protocol_ResultOk(1U); }
        if (App_Protocol_IsName(tok[1], "task2", "selectTask2", "t2")) { App_ProtocolSelectTask2(); return App_Protocol_ResultOk(1U); }
        if (App_Protocol_IsName(tok[1], "task3", "selectTask3", "t3")) { App_ProtocolSelectOnly(3U); App_ProtocolPromptStart(330); return App_Protocol_ResultOk(1U); }
        if (App_Protocol_IsName(tok[1], "task4", "selectTask4", "t4")) { App_ProtocolSelectOnly(4U); App_ProtocolPromptStart(400); return App_Protocol_ResultOk(1U); }
        if (App_Protocol_IsName(tok[1], "start", "run", "taskStart")) { App_ProtocolStartSelectedTask(); return App_Protocol_ResultOk(1U); }
        if (App_Protocol_IsName(tok[1], "stop", "halt", "brake")) { return App_Protocol_ResultIgnored("remote_motion_key_disabled", 1U); }
        if (App_Protocol_IsName(tok[1], "webStop", "pidStop", 0)) { App_Protocol_WebPidStopControl(); return App_Protocol_ResultOk(1U); }
        if (App_Protocol_IsName(tok[1], "taskStop", "taskReset", "taskIdle")) { App_ProtocolTaskStop(); return App_Protocol_ResultOk(1U); }
        if (App_Protocol_IsName(tok[1], "arcTest", "arc", "arcRun")) { App_ProtocolTaskReset(); App_ProtocolArcStart(); return App_Protocol_ResultOk(1U); }
        if (App_Protocol_IsName(tok[1], "tracing", "trace", "line")) { App_StartTracingMode(); return App_Protocol_ResultOk(1U); }
        if (App_Protocol_IsName(tok[1], "Bluetooth", "BT", "remote")) { return App_Protocol_ResultIgnored("bluetooth_remote_disabled", 1U); }
        if (App_Protocol_IsName(tok[1], "up", "forward", "fwd") ||
            App_Protocol_IsName(tok[1], "down", "backward", "back") ||
            App_Protocol_IsName(tok[1], "left", "right", 0) ||
            App_Protocol_IsName(tok[1], "speedUp", "speedDown", 0))
        {
            return App_Protocol_ResultIgnored("remote_motion_key_disabled", 1U);
        }

        App_Protocol_RecordError(PROTO_ERR_UNKNOWN, "unknown", 1U);
        return PROTO_RESULT_ERROR;
    }

    if (App_Protocol_IsName(tok[0], "slider", "s", 0))
    {
        if (n < 3)
        {
            App_Protocol_RecordError(PROTO_ERR_FIELD, "field", 1U);
            return PROTO_RESULT_ERROR;
        }
        if (!App_Protocol_ParseFloat(tok[2], &value))
        {
            App_Protocol_RecordError(PROTO_ERR_BAD_FLOAT, "bad-number", 1U);
            return PROTO_RESULT_ERROR;
        }
        return App_Protocol_ApplySliderPacket(tok[1], value);
    }

    if (App_Protocol_IsName(tok[0], "joystick", "j", 0))
    {
        return App_Protocol_ApplyJoystickPacket(tok, n);
    }

    if (App_Protocol_IsName(tok[0], "pid", 0, 0))
    {
        return App_Protocol_ApplyPidPacket(tok, n);
    }

    App_Protocol_RecordError(PROTO_ERR_UNKNOWN, "unknown", 1U);
    return PROTO_RESULT_ERROR;
}

void App_Protocol_Process(void)
{
    static uint8_t receiving = 0;
    static uint8_t index = 0;
    static char packet[128];
    uint8_t byte;
    uint8_t result;

    while (Serial_ReadByte(&byte))
    {
        char c = (char)byte;
        if (c == '[')
        {
            receiving = 1;
            index = 0;
            continue;
        }
        if (!receiving) continue;
        if (c == ']')
        {
            packet[index] = '\0';
            receiving = 0;
            result = App_Protocol_ApplyPacket(packet);
            if (result == PROTO_RESULT_OK)
            {
                g_lastCmdTickMs = 0;
            }
            continue;
        }
        if (c == '\r' || c == '\n') continue;
        if (index < sizeof(packet) - 1U) packet[index++] = c;
        else
        {
            receiving = 0;
            index = 0;
            App_Protocol_RecordError(PROTO_ERR_TOO_LONG, "too-long", 1U);
        }
    }
}
