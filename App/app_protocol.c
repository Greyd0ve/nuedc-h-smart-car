#include "app_protocol.h"
#include "app_state.h"
#include "PWM.h"
#include "Serial.h"
#include <stdint.h>

#define PROTO_RESULT_ERROR      0U
#define PROTO_RESULT_OK         1U
#define PROTO_RESULT_IGNORED    2U

typedef enum
{
    PROTO_ERR_FIELD = 0,
    PROTO_ERR_UNKNOWN,
    PROTO_ERR_BAD_FLOAT,
    PROTO_ERR_RANGE,
    PROTO_ERR_TOO_LONG,
    PROTO_ERR_LOCKED
} ProtocolError_t;

extern volatile uint32_t g_lastCmdTickMs;
extern volatile uint8_t g_safetyLocked;
extern volatile float g_btSpeedLimitPercent;
extern volatile float g_speedScale;
extern volatile float g_pwmLimit;

extern volatile uint32_t g_protoPacketOkCount;
extern volatile uint32_t g_protoErrFieldCount;
extern volatile uint32_t g_protoErrUnknownCount;
extern volatile uint32_t g_protoErrBadFloatCount;
extern volatile uint32_t g_protoErrRangeCount;
extern volatile uint32_t g_protoErrTooLongCount;
extern volatile uint32_t g_protoErrLockedCount;

extern volatile float g_gyroZRawDps;
extern volatile float g_gyroZScale;
extern volatile uint8_t g_gyroZKalmanEnable;
extern volatile float g_gyroZKalmanQ;
extern volatile float g_gyroZKalmanR;
extern volatile float g_gyroZKalmanP;
extern volatile float g_gyroZKalmanX;
extern volatile uint8_t g_staticBiasTrackEnable;
extern volatile float g_staticBiasAlpha;
extern volatile float g_gyroZDeadbandDps;
extern volatile float g_yawKp;
extern volatile float g_yawKd;
extern volatile float g_straightSpeed;

extern volatile float g_task2ArcMinForwardPulse;
extern volatile uint16_t g_task2LineLostConfirmMs;
extern volatile uint16_t g_task2AlignWaitTargetMs;
extern volatile float g_task3CbEntryYaw;
extern volatile float g_task3DaEntryYaw;

extern volatile float g_arcExitYawWindowDeg;

extern volatile float g_task2CAlignBiasDeg;
extern volatile float g_task2AAlignBiasDeg;
extern volatile float g_task3CbExitYaw;
extern volatile float g_task3DaExitYaw;

extern volatile float g_yawAlignToleranceDeg;
extern volatile float g_yawAlignTurnSpeed;
extern volatile uint16_t g_yawAlignMaxMs;

extern volatile float g_task3AcYawDeg;
extern volatile float g_task3CYawReset;
extern volatile float g_task3CAlignYaw;
extern volatile float g_task3CAlignBias;
extern volatile float g_task3DiagToSearchPulse;
extern volatile float g_task3BdYawDeg;
extern volatile uint8_t g_task4LapTarget;
extern volatile uint16_t g_task4LapYawZeroWaitMs;

static float App_Protocol_LimitFloat(float value, float minVal, float maxVal)
{
    if (value < minVal) return minVal;
    if (value > maxVal) return maxVal;
    return value;
}

static int App_Protocol_StrEqual(const char *a, const char *b)
{
    while (*a != '\0' && *b != '\0')
    {
        if (*a != *b) return 0;
        a++;
        b++;
    }
    return (*a == '\0' && *b == '\0');
}

static int App_Protocol_IsDown(const char *s)
{
    return App_Protocol_StrEqual(s, "down");
}

static int App_Protocol_IsName(const char *s, const char *name)
{
    return App_Protocol_StrEqual(s, name);
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
    else if (err == PROTO_ERR_BAD_FLOAT) g_protoErrBadFloatCount++;
    else if (err == PROTO_ERR_RANGE) g_protoErrRangeCount++;
    else if (err == PROTO_ERR_TOO_LONG) g_protoErrTooLongCount++;
    else if (err == PROTO_ERR_LOCKED) g_protoErrLockedCount++;

    if (report)
    {
#if ENABLE_VERBOSE_STATUS
        Serial_Printf("[status,err,%s]\r\n", reason);
#else
        Serial_Printf("[err]\r\n");
#endif
    }
#if !ENABLE_VERBOSE_STATUS
    (void)reason;
#endif
}

static uint8_t App_Protocol_ResultOk(uint8_t report)
{
    g_protoPacketOkCount++;
    if (report)
    {
#if ENABLE_VERBOSE_STATUS
        Serial_Printf("[status,ok]\r\n");
#else
        Serial_Printf("[ok]\r\n");
#endif
    }
    return PROTO_RESULT_OK;
}

static uint8_t App_Protocol_ParseFloat(const char *text, float *out)
{
    const char *p;
    float value;
    float fracScale;
    int sign;
    uint8_t gotDigit;

    if (text == 0 || out == 0) return 0;

    p = App_Protocol_SkipSpace(text);
    if (*p == '\0') return 0;

    sign = 1;
    if (*p == '-')
    {
        sign = -1;
        p++;
    }
    else if (*p == '+')
    {
        p++;
    }

    value = 0.0f;
    gotDigit = 0U;
    while (*p >= '0' && *p <= '9')
    {
        value = value * 10.0f + (float)(*p - '0');
        p++;
        gotDigit = 1U;
        if (value > 1000000.0f) return 0;
    }

    if (*p == '.')
    {
        p++;
        fracScale = 0.1f;
        while (*p >= '0' && *p <= '9')
        {
            value += (float)(*p - '0') * fracScale;
            fracScale *= 0.1f;
            p++;
            gotDigit = 1U;
        }
    }

    p = App_Protocol_SkipSpace(p);
    if (*p != '\0') return 0;
    if (!gotDigit) return 0;

    *out = (sign < 0) ? -value : value;
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

void App_Protocol_ApplySpeedLimitPercent(float percent)
{
    float ratio;
    percent = App_Protocol_LimitFloat(percent, 0.0f, 100.0f);
    ratio = percent / 100.0f;
    g_btSpeedLimitPercent = percent;
    g_speedScale = ratio;
    g_pwmLimit = (float)PWM_MAX_DUTY * ratio;
}

static uint8_t App_Protocol_ApplySliderPacket(const char *name, float value)
{
    uint8_t enable;

    if (App_Protocol_IsName(name, "plotMode"))
    {
        return App_Protocol_ResultOk(1U);
    }

    if (App_Protocol_IsName(name, "RP"))
    {
        if (value < 0.0f || value > 100.0f)
        {
            App_Protocol_RecordError(PROTO_ERR_RANGE, "range", 1U);
            return PROTO_RESULT_ERROR;
        }
        App_Protocol_ApplySpeedLimitPercent(value);
        return App_Protocol_ResultOk(1U);
    }

    if (App_Protocol_IsName(name, "straightSpeed")) { if (!App_Protocol_SetFloatRange(&g_straightSpeed, value, 0.0f, 80.0f)) return PROTO_RESULT_ERROR; return App_Protocol_ResultOk(1U); }
    if (App_Protocol_IsName(name, "yawKp")) { if (!App_Protocol_SetFloatRange(&g_yawKp, value, 0.0f, 20.0f)) return PROTO_RESULT_ERROR; return App_Protocol_ResultOk(1U); }
    if (App_Protocol_IsName(name, "yawKd")) { if (!App_Protocol_SetFloatRange(&g_yawKd, value, 0.0f, 10.0f)) return PROTO_RESULT_ERROR; return App_Protocol_ResultOk(1U); }

    if (App_Protocol_IsName(name, "arcMinPulse")) { if (!App_Protocol_SetFloatRange(&g_task2ArcMinForwardPulse, value, 1000.0f, 15000.0f)) return PROTO_RESULT_ERROR; return App_Protocol_ResultOk(1U); }
    if (App_Protocol_IsName(name, "arcLostMs")) { if (!App_Protocol_SetUint16Range(&g_task2LineLostConfirmMs, value, 0.0f, 3000.0f)) return PROTO_RESULT_ERROR; return App_Protocol_ResultOk(1U); }
    if (App_Protocol_IsName(name, "arcExitYawWindow")) { if (!App_Protocol_SetFloatRange(&g_arcExitYawWindowDeg, value, 10.0f, 90.0f)) return PROTO_RESULT_ERROR; return App_Protocol_ResultOk(1U); }
    if (App_Protocol_IsName(name, "alignWaitMs")) { if (!App_Protocol_SetUint16Range(&g_task2AlignWaitTargetMs, value, 0.0f, 3000.0f)) return PROTO_RESULT_ERROR; return App_Protocol_ResultOk(1U); }

    if (App_Protocol_IsName(name, "yawAlignTol")) { if (!App_Protocol_SetFloatRange(&g_yawAlignToleranceDeg, value, 1.0f, 15.0f)) return PROTO_RESULT_ERROR; return App_Protocol_ResultOk(1U); }
    if (App_Protocol_IsName(name, "yawAlignTurn")) { if (!App_Protocol_SetFloatRange(&g_yawAlignTurnSpeed, value, 2.0f, 30.0f)) return PROTO_RESULT_ERROR; return App_Protocol_ResultOk(1U); }
    if (App_Protocol_IsName(name, "yawAlignMaxMs")) { if (!App_Protocol_SetUint16Range(&g_yawAlignMaxMs, value, 500.0f, 8000.0f)) return PROTO_RESULT_ERROR; return App_Protocol_ResultOk(1U); }

    if (App_Protocol_IsName(name, "task2CAlignBias")) { if (!App_Protocol_SetFloatRange(&g_task2CAlignBiasDeg, value, -10.0f, 10.0f)) return PROTO_RESULT_ERROR; return App_Protocol_ResultOk(1U); }
    if (App_Protocol_IsName(name, "task2AAlignBias")) { if (!App_Protocol_SetFloatRange(&g_task2AAlignBiasDeg, value, -10.0f, 10.0f)) return PROTO_RESULT_ERROR; return App_Protocol_ResultOk(1U); }

    if (App_Protocol_IsName(name, "task3AcYaw")) { if (!App_Protocol_SetFloatRange(&g_task3AcYawDeg, value, -90.0f, 90.0f)) return PROTO_RESULT_ERROR; return App_Protocol_ResultOk(1U); }
    if (App_Protocol_IsName(name, "task3CYawReset")) { if (!App_Protocol_SetFloatRange(&g_task3CYawReset, value, -180.0f, 180.0f)) return PROTO_RESULT_ERROR; return App_Protocol_ResultOk(1U); }
    if (App_Protocol_IsName(name, "task3CAlignYaw")) { if (!App_Protocol_SetFloatRange(&g_task3CAlignYaw, value, -180.0f, 180.0f)) return PROTO_RESULT_ERROR; return App_Protocol_ResultOk(1U); }
    if (App_Protocol_IsName(name, "task3CAlignBias")) { if (!App_Protocol_SetFloatRange(&g_task3CAlignBias, value, -10.0f, 10.0f)) return PROTO_RESULT_ERROR; return App_Protocol_ResultOk(1U); }
    if (App_Protocol_IsName(name, "task3SearchPulse")) { if (!App_Protocol_SetFloatRange(&g_task3DiagToSearchPulse, value, 3000.0f, 15000.0f)) return PROTO_RESULT_ERROR; return App_Protocol_ResultOk(1U); }
    if (App_Protocol_IsName(name, "task3CbEntryYaw")) { if (!App_Protocol_SetFloatRange(&g_task3CbEntryYaw, value, -100.0f, 100.0f)) return PROTO_RESULT_ERROR; return App_Protocol_ResultOk(1U); }
    if (App_Protocol_IsName(name, "task3CbExitYaw")) { if (!App_Protocol_SetFloatRange(&g_task3CbExitYaw, value, -180.0f, 180.0f)) return PROTO_RESULT_ERROR; return App_Protocol_ResultOk(1U); }
    if (App_Protocol_IsName(name, "task3BdYaw")) { if (!App_Protocol_SetFloatRange(&g_task3BdYawDeg, value, 60.0f, 180.0f)) return PROTO_RESULT_ERROR; return App_Protocol_ResultOk(1U); }
    if (App_Protocol_IsName(name, "task3DaEntryYaw")) { if (!App_Protocol_SetFloatRange(&g_task3DaEntryYaw, value, -100.0f, 100.0f)) return PROTO_RESULT_ERROR; return App_Protocol_ResultOk(1U); }
    if (App_Protocol_IsName(name, "task3DaExitYaw")) { if (!App_Protocol_SetFloatRange(&g_task3DaExitYaw, value, -180.0f, 180.0f)) return PROTO_RESULT_ERROR; return App_Protocol_ResultOk(1U); }

    if (App_Protocol_IsName(name, "task4Laps"))
    {
        if (value < 1.0f || value > 10.0f) { App_Protocol_RecordError(PROTO_ERR_RANGE, "range", 1U); return PROTO_RESULT_ERROR; }
        g_task4LapTarget = (uint8_t)(value + 0.5f);
        return App_Protocol_ResultOk(1U);
    }
    if (App_Protocol_IsName(name, "task4LapYawZeroWaitMs")) { if (!App_Protocol_SetUint16Range(&g_task4LapYawZeroWaitMs, value, 0.0f, 2000.0f)) return PROTO_RESULT_ERROR; return App_Protocol_ResultOk(1U); }

    if (App_Protocol_IsName(name, "gyroZScale")) { if (!App_Protocol_SetFloatRange(&g_gyroZScale, value, 0.80f, 1.20f)) return PROTO_RESULT_ERROR; return App_Protocol_ResultOk(1U); }
    if (App_Protocol_IsName(name, "gyroZKalmanEnable"))
    {
        if (value < 0.0f || value > 1.0f) { App_Protocol_RecordError(PROTO_ERR_RANGE, "range", 1U); return PROTO_RESULT_ERROR; }
        enable = (value < 0.5f) ? 0U : 1U;
        if (enable && !g_gyroZKalmanEnable)
        {
            g_gyroZKalmanX = g_gyroZRawDps;
            g_gyroZKalmanP = 1.0f;
        }
        g_gyroZKalmanEnable = enable;
        return App_Protocol_ResultOk(1U);
    }
    if (App_Protocol_IsName(name, "gyroZKalmanQ")) { if (!App_Protocol_SetFloatRange(&g_gyroZKalmanQ, value, 0.001f, 0.2f)) return PROTO_RESULT_ERROR; return App_Protocol_ResultOk(1U); }
    if (App_Protocol_IsName(name, "gyroZKalmanR")) { if (!App_Protocol_SetFloatRange(&g_gyroZKalmanR, value, 0.1f, 10.0f)) return PROTO_RESULT_ERROR; return App_Protocol_ResultOk(1U); }
    if (App_Protocol_IsName(name, "staticBiasTrack"))
    {
        if (value < 0.0f || value > 1.0f) { App_Protocol_RecordError(PROTO_ERR_RANGE, "range", 1U); return PROTO_RESULT_ERROR; }
        g_staticBiasTrackEnable = (value < 0.5f) ? 0U : 1U;
        return App_Protocol_ResultOk(1U);
    }
    if (App_Protocol_IsName(name, "staticBiasAlpha")) { if (!App_Protocol_SetFloatRange(&g_staticBiasAlpha, value, 0.990f, 0.9999f)) return PROTO_RESULT_ERROR; return App_Protocol_ResultOk(1U); }
    if (App_Protocol_IsName(name, "gyroZDeadband")) { if (!App_Protocol_SetFloatRange(&g_gyroZDeadbandDps, value, 0.0f, 1.0f)) return PROTO_RESULT_ERROR; return App_Protocol_ResultOk(1U); }

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

    if (App_Protocol_IsName(tok[0], "key"))
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

        if (App_Protocol_IsName(tok[1], "emergency")) { App_EmergencyStop(); return App_Protocol_ResultOk(1U); }
        if (App_Protocol_IsName(tok[1], "unlock")) { App_UnlockControl(); return App_Protocol_ResultOk(1U); }

        if (g_safetyLocked)
        {
            App_Protocol_RecordError(PROTO_ERR_LOCKED, "locked", 1U);
            return PROTO_RESULT_ERROR;
        }

        if (App_Protocol_IsName(tok[1], "mpuDebug")) { App_ProtocolEnterMpuDebug(); return App_Protocol_ResultOk(1U); }
        if (App_Protocol_IsName(tok[1], "mpuCalib")) { App_ProtocolMpuCalibrateGyroZ(); return App_Protocol_ResultOk(1U); }
        if (App_Protocol_IsName(tok[1], "yawZero")) { App_ProtocolMpuResetYaw(); App_ProtocolPromptStart(180); return App_Protocol_ResultOk(1U); }
        if (App_Protocol_IsName(tok[1], "task1")) { App_ProtocolSelectTask1(); return App_Protocol_ResultOk(1U); }
        if (App_Protocol_IsName(tok[1], "task2")) { App_ProtocolSelectTask2(); return App_Protocol_ResultOk(1U); }
        if (App_Protocol_IsName(tok[1], "task3")) { App_ProtocolSelectOnly(3U); App_ProtocolPromptStart(330); return App_Protocol_ResultOk(1U); }
        if (App_Protocol_IsName(tok[1], "task4")) { App_ProtocolSelectOnly(4U); App_ProtocolPromptStart(400); return App_Protocol_ResultOk(1U); }
        if (App_Protocol_IsName(tok[1], "start")) { App_ProtocolStartSelectedTask(); return App_Protocol_ResultOk(1U); }
        if (App_Protocol_IsName(tok[1], "taskStop")) { App_ProtocolTaskStop(); return App_Protocol_ResultOk(1U); }

        App_Protocol_RecordError(PROTO_ERR_UNKNOWN, "unknown", 1U);
        return PROTO_RESULT_ERROR;
    }

    if (App_Protocol_IsName(tok[0], "slider"))
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

    if (App_Protocol_IsName(tok[0], "joystick"))
    {
        return PROTO_RESULT_IGNORED;
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
