#include "BT_proto.h"

/*
 * Legacy Bluetooth remote parser is intentionally compiled out.
 * The active serial protocol lives in App/app_protocol.c.
 */
void BT_Process(void)
{
}

#if 0
/*
 * 文件：Hardware/BT_proto.c
 * 作用：解析蓝牙串口小程序发送的数据包。
 *
 * 支持的数据包来自蓝牙串口协议：
 *   [joystick,lx,ly,rx,ry] 或短格式 [j,lx,ly,rx,ry]  已禁用运动输出
 *   [slider,id,value]      或短格式 [s,id,value]
 *   [slider,RP,0~100]     使用名为 RP 的滑杆控制最高 PWM 输出百分比
 *   [key,id,down/up]       或短格式 [k,id,d/u]
 *   [key,Bluetooth,down]   已禁用：不进入蓝牙遥控模式
 *   [key,emergency,down]   立即急停并进入安全锁定状态
 *   [key,unlock,down]      解除安全锁，回到待机状态
 *
 * 本版修改重点：
 *   原先最高速度由板载 RP4 电位器控制；现在改为由蓝牙滑杆 RP 控制。
 *   例如：[slider,RP,0] 表示 PWM 最大输出限制为 0%；
 *         [slider,RP,100] 表示 PWM 最大输出限制为 100%。
 */
#include "BT_proto.h"
#include "Serial.h"
#include "PWM.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

extern volatile uint32_t g_lastCmdTickMs;

extern volatile float g_forwardKp;
extern volatile float g_forwardKi;
extern volatile float g_forwardKd;
extern volatile float g_turnKp;
extern volatile float g_turnKi;
extern volatile float g_turnKd;
extern volatile float g_maxForwardCmd;
extern volatile float g_maxTurnCmd;
extern volatile float g_pwmLimit;
extern volatile float g_speedScale;
extern volatile float g_btSpeedLimitPercent;
extern volatile uint8_t g_safetyLocked;

extern volatile uint32_t g_protoPacketOkCount;
extern volatile uint32_t g_protoErrFieldCount;
extern volatile uint32_t g_protoErrUnknownCount;
extern volatile uint32_t g_protoErrBadIntCount;
extern volatile uint32_t g_protoErrBadFloatCount;
extern volatile uint32_t g_protoErrRangeCount;
extern volatile uint32_t g_protoErrTooLongCount;

extern void App_EmergencyStop(void);
extern void App_UnlockControl(void);

#define BT_PROTO_RESULT_ERROR      0U
#define BT_PROTO_RESULT_OK         1U
#define BT_PROTO_RESULT_IGNORED    2U

typedef enum
{
    BT_PROTO_ERR_FIELD = 0,
    BT_PROTO_ERR_UNKNOWN,
    BT_PROTO_ERR_BAD_INT,
    BT_PROTO_ERR_BAD_FLOAT,
    BT_PROTO_ERR_RANGE,
    BT_PROTO_ERR_TOO_LONG
} BT_ProtocolError_t;

static float absf_local(float x)
{
    return (x >= 0.0f) ? x : -x;
}

static int str_equal(const char *a, const char *b)
{
    return strcmp(a, b) == 0;
}

static char ascii_lower_char(char c)
{
    if (c >= 'A' && c <= 'Z')
    {
        return (char)(c + ('a' - 'A'));
    }
    return c;
}

static int str_equal_ignore_case(const char *a, const char *b)
{
    while (*a != '\0' && *b != '\0')
    {
        if (ascii_lower_char(*a) != ascii_lower_char(*b))
        {
            return 0;
        }
        a++;
        b++;
    }
    return (*a == '\0' && *b == '\0');
}

static int str_is_bluetooth_name(const char *s)
{
    return str_equal_ignore_case(s, "Bluetooth") || str_equal_ignore_case(s, "BT") || str_equal_ignore_case(s, "remote");
}

static int str_is_emergency_name(const char *s)
{
    return str_equal_ignore_case(s, "emergency") || str_equal_ignore_case(s, "emg");
}

static int str_is_unlock_name(const char *s)
{
    return str_equal_ignore_case(s, "unlock") || str_equal_ignore_case(s, "release") || str_equal_ignore_case(s, "resume");
}

static int str_is_remote_motion_key(const char *s)
{
    return str_equal_ignore_case(s, "up") ||
           str_equal_ignore_case(s, "down") ||
           str_equal_ignore_case(s, "left") ||
           str_equal_ignore_case(s, "right") ||
           str_equal_ignore_case(s, "forward") ||
           str_equal_ignore_case(s, "fwd") ||
           str_equal_ignore_case(s, "backward") ||
           str_equal_ignore_case(s, "back") ||
           str_equal_ignore_case(s, "stop") ||
           str_equal_ignore_case(s, "halt") ||
           str_equal_ignore_case(s, "brake") ||
           str_equal_ignore_case(s, "speedUp") ||
           str_equal_ignore_case(s, "speedDown");
}

static int str_is_rp_name(const char *s)
{
    return str_equal(s, "RP") || str_equal(s, "rp") || str_equal(s, "Rp") || str_equal(s, "rP");
}

static float clip_float(float x, float minVal, float maxVal)
{
    if (x < minVal)
    {
        return minVal;
    }
    if (x > maxVal)
    {
        return maxVal;
    }
    return x;
}

static const char *bt_proto_skip_space(const char *s)
{
    while (*s == ' ' || *s == '\t')
    {
        s++;
    }
    return s;
}

static void BT_RecordError(BT_ProtocolError_t err, const char *reason, uint8_t report)
{
    if (err == BT_PROTO_ERR_FIELD) g_protoErrFieldCount++;
    else if (err == BT_PROTO_ERR_UNKNOWN) g_protoErrUnknownCount++;
    else if (err == BT_PROTO_ERR_BAD_INT) g_protoErrBadIntCount++;
    else if (err == BT_PROTO_ERR_BAD_FLOAT) g_protoErrBadFloatCount++;
    else if (err == BT_PROTO_ERR_RANGE) g_protoErrRangeCount++;
    else if (err == BT_PROTO_ERR_TOO_LONG) g_protoErrTooLongCount++;

    if (report)
    {
        Serial_Printf("[status,err,%s]\r\n", reason);
    }
}

static uint8_t BT_ResultOk(uint8_t report)
{
    g_protoPacketOkCount++;
    if (report)
    {
        Serial_Printf("[status,ok]\r\n");
    }
    return BT_PROTO_RESULT_OK;
}

static uint8_t BT_ResultIgnored(const char *reason, uint8_t report)
{
    if (report)
    {
        Serial_Printf("[status,ignored,%s]\r\n", reason);
    }
    return BT_PROTO_RESULT_IGNORED;
}

static uint8_t BT_ParseInt(const char *text, int *out)
{
    const char *p;
    char *endPtr;
    long value;

    if (text == 0 || out == 0) return 0;

    p = bt_proto_skip_space(text);
    if (*p == '\0') return 0;

    value = strtol(p, &endPtr, 10);
    if (endPtr == p) return 0;

    endPtr = (char *)bt_proto_skip_space(endPtr);
    if (*endPtr != '\0') return 0;
    if (value > 32767L || value < -32768L) return 0;

    *out = (int)value;
    return 1;
}

static uint8_t BT_ReadIntRange(const char *text, int minVal, int maxVal, int *out)
{
    int value;

    if (!BT_ParseInt(text, &value))
    {
        BT_RecordError(BT_PROTO_ERR_BAD_INT, "bad-number", 1U);
        return 0;
    }

    if (value < minVal || value > maxVal)
    {
        BT_RecordError(BT_PROTO_ERR_RANGE, "range", 1U);
        return 0;
    }

    *out = value;
    return 1;
}

/*
 * 通过蓝牙串口虚拟滑杆 RP 设置最高速度/PWM 限幅。
 * 协议示例：
 *   [slider,RP,0]   -> PWM 0% 输出，g_pwmLimit = 0
 *   [slider,RP,100] -> PWM 100% 输出，g_pwmLimit = PWM_MAX_DUTY
 *
 * 同时同步 g_speedScale，使 H 题自动路径速度也随 RP 百分比缩放。
 */
static void BT_ApplySpeedLimitPercent(int value)
{
    float percent = clip_float((float)value, 0.0f, 100.0f);
    float ratio = percent / 100.0f;

    g_btSpeedLimitPercent = percent;
    g_pwmLimit = (float)PWM_MAX_DUTY * ratio;
    g_speedScale = ratio;

    g_maxForwardCmd = 80.0f * g_speedScale;
    g_maxTurnCmd = 85.0f * g_speedScale;
}

static uint8_t apply_packet(char *payload)
{
    char *tok[10] = {0};
    int n = 0;
    char *p = payload;

    tok[n++] = p;
    while (*p != '\0' && n < 10)
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
        BT_RecordError(BT_PROTO_ERR_FIELD, "field", 1U);
        return BT_PROTO_RESULT_ERROR;
    }

    /*
     * 1. 安全控制最高优先级。
     *
     * 网页急停按钮会发送：
     *   [key,emergency,down]
     *   [joystick,0,0,0,0]
     *
     * 所以必须先识别 emergency，让主程序立即关 PWM 并进入安全锁。
     * 解锁命令 [key,unlock,down] 也必须在安全锁状态下仍然可用。
     */
    if (str_equal(tok[0], "k") || str_equal(tok[0], "key"))
    {
        if (n >= 3)
        {
            int isDown = str_equal(tok[2], "d") || str_equal(tok[2], "down");

            if (isDown && str_is_emergency_name(tok[1]))
            {
                App_EmergencyStop();
                return BT_ResultOk(1U);
            }

            if (isDown && str_is_unlock_name(tok[1]))
            {
                App_UnlockControl();
                return BT_ResultOk(1U);
            }
        }
    }

    /*
     * 2. 摇杆包。
     *
     * 协议：
     *   [joystick,左X,左Y,右X,右Y]
     *
     * 蓝牙遥控运动已禁用；收到后只回 ignored，不写目标速度。
     */
    if (str_equal(tok[0], "j") || str_equal(tok[0], "joystick"))
    {
        return BT_ResultIgnored("joystick_remote_disabled", 1U);
    }

    /*
     * 3. 滑杆控制。
     *
     * 最重要的滑杆是 RP：
     *   [slider,RP,0]   -> PWM 最大输出 0%
     *   [slider,RP,100] -> PWM 最大输出 100%
     *
     * 急停锁定时仍允许 RP 更新，因为这不直接产生运动输出；
     * 其他滑杆在急停锁定时全部忽略。
     */
    if (str_equal(tok[0], "s") || str_equal(tok[0], "slider"))
    {
        if (n >= 3)
        {
            int value;
            int id;

            if (str_is_rp_name(tok[1]))
            {
                if (!BT_ReadIntRange(tok[2], 0, 100, &value)) return BT_PROTO_RESULT_ERROR;
                BT_ApplySpeedLimitPercent(value);
                return BT_ResultOk(1U);
            }

            if (g_safetyLocked)
            {
                return BT_PROTO_RESULT_IGNORED;
            }

            if (!BT_ReadIntRange(tok[2], -300, 300, &value)) return BT_PROTO_RESULT_ERROR;
            if (!BT_ReadIntRange(tok[1], 1, 10, &id)) return BT_PROTO_RESULT_ERROR;

            if (id == 1)
            {
                return BT_ResultIgnored("slider_forward_remote_disabled", 1U);
            }
            else if (id == 2)
            {
                return BT_ResultIgnored("slider_turn_remote_disabled", 1U);
            }
            else if (id == 3)
            {
                g_forwardKp = absf_local((float)value) / 10.0f;
            }
            else if (id == 4)
            {
                g_forwardKi = absf_local((float)value) / 100.0f;
            }
            else if (id == 5)
            {
                if (absf_local((float)value) > 200.0f)
                {
                    BT_RecordError(BT_PROTO_ERR_RANGE, "range", 1U);
                    return BT_PROTO_RESULT_ERROR;
                }
                g_forwardKd = absf_local((float)value) / 10.0f;
            }
            else if (id == 6)
            {
                g_turnKp = absf_local((float)value) / 10.0f;
            }
            else if (id == 7)
            {
                g_turnKi = absf_local((float)value) / 100.0f;
            }
            else if (id == 8)
            {
                if (absf_local((float)value) > 200.0f)
                {
                    BT_RecordError(BT_PROTO_ERR_RANGE, "range", 1U);
                    return BT_PROTO_RESULT_ERROR;
                }
                g_turnKd = absf_local((float)value) / 10.0f;
            }
            else if (id == 9)
            {
                if (absf_local((float)value) < 10.0f)
                {
                    BT_RecordError(BT_PROTO_ERR_RANGE, "range", 1U);
                    return BT_PROTO_RESULT_ERROR;
                }
                g_maxForwardCmd = absf_local((float)value);
            }
            else if (id == 10)
            {
                if (absf_local((float)value) < 10.0f)
                {
                    BT_RecordError(BT_PROTO_ERR_RANGE, "range", 1U);
                    return BT_PROTO_RESULT_ERROR;
                }
                g_maxTurnCmd = absf_local((float)value);
            }
            else
            {
                BT_RecordError(BT_PROTO_ERR_UNKNOWN, "unknown", 1U);
                return BT_PROTO_RESULT_ERROR;
            }
            return BT_ResultOk(1U);
        }
        BT_RecordError(BT_PROTO_ERR_FIELD, "field", 1U);
        return BT_PROTO_RESULT_ERROR;
    }

    /*
     * 4. 直接速度命令。
     *
     * 蓝牙直接速度调试已禁用：
     *   [car,forward,turn]
     *   [vel,forward,turn]
     *   [cmd,forward,turn]
     */
    if (str_equal(tok[0], "car") || str_equal(tok[0], "vel") || str_equal(tok[0], "cmd"))
    {
        return BT_ResultIgnored("direct_speed_remote_disabled", 1U);
    }

    /*
     * 5. 普通按键命令。
     *
     * 名称按键：
     *   [key,Bluetooth,down] -> 已禁用，不进入遥控模式
     *
     * 数字按键兼容旧代码，但运动输出已禁用。
     */
    if (str_equal(tok[0], "k") || str_equal(tok[0], "key"))
    {
        if (n >= 3)
        {
            int id;
            int isDown = str_equal(tok[2], "d") || str_equal(tok[2], "down");

            if (!isDown)
            {
                return BT_PROTO_RESULT_IGNORED;
            }

            if (g_safetyLocked)
            {
                return BT_PROTO_RESULT_IGNORED;
            }

            if (isDown && str_is_bluetooth_name(tok[1]))
            {
                return BT_ResultIgnored("bluetooth_remote_disabled", 1U);
            }

            if (isDown && str_is_remote_motion_key(tok[1]))
            {
                return BT_ResultIgnored("remote_motion_key_disabled", 1U);
            }

            if (!BT_ReadIntRange(tok[1], 1, 4, &id)) return BT_PROTO_RESULT_ERROR;
            (void)id;
            return BT_ResultIgnored("numeric_remote_key_disabled", 1U);
        }
        BT_RecordError(BT_PROTO_ERR_FIELD, "field", 1U);
        return BT_PROTO_RESULT_ERROR;
    }

    BT_RecordError(BT_PROTO_ERR_UNKNOWN, "unknown", 1U);
    return BT_PROTO_RESULT_ERROR;
}

/* 从串口环形缓冲区取字节，提取 [ ... ] 包后交给 BT_ParsePacket。 */
void BT_Process(void)
{
    static uint8_t receiving = 0;
    static uint8_t index = 0;
    static char packet[96];
    uint8_t byte;
    uint8_t gotPacket = 0;
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

        if (!receiving)
        {
            continue;
        }

        if (c == ']')
        {
            packet[index] = '\0';
            receiving = 0;
            gotPacket = 1;
            break;
        }

        if (c == '\r' || c == '\n')
        {
            continue;
        }

        if (index < sizeof(packet) - 1)
        {
            packet[index++] = c;
        }
        else
        {
            receiving = 0;
            index = 0;
            BT_RecordError(BT_PROTO_ERR_TOO_LONG, "too-long", 1U);
        }
    }

    if (gotPacket)
    {
        result = apply_packet(packet);
        if (result == BT_PROTO_RESULT_OK)
        {
            g_lastCmdTickMs = 0;
        }
    }
}
#endif
