/*
 * 文件：User/main.c
 * 版本：Bluetooth + 8路灰度循迹精简调参版 + main.c 灰度直读排查版 + AD1改PB3/AD2改PB4版
 *
 *
 * 本版新增：
 *   1. 在 main.c 内部强制重新初始化八路灰度 GPIO：AD0=PA8、AD1=PB3、AD2=PB4、OUT=PB0。
 *   2. 在 main.c 内部直接切换 AD0/AD1/AD2 并读取 OUT，绕过 Grayscale_ReadAll()。
 *   3. OLED 最后一行显示 R/M/E：
 *        R = 原始直读掩码，不受 lineReverse 影响，用于排查 AD1 是否生效；
 *        M = 实际参与循迹计算的掩码；
 *        E = 缩小后的循迹误差。
 *
 * 功能总览：
 *   1. 上电默认进入蓝牙遥控模式。
 *   2. 网页发送 [key,tracing,down] 后进入八路灰度黑线循迹模式。
 *   3. 网页发送 [key,Bluetooth,down] 后回到蓝牙遥控模式。
 *   4. 网页急停 [key,emergency,down] 后立即关闭 PWM，并进入安全锁定。
 *   5. 只有收到 [key,unlock,down] 后才解除锁定，避免误触后小车继续运动。
 *   6. 网页滑杆 [slider,RP,0~100] 用作最大 PWM 百分比限幅。
 *   7. 支持网页滑杆在线调 PID、循迹、速度、滤波、斜坡等重要参数。
 *   8. SW1/K1：把 RP 设置为 0%，PWM 立即清零，但不进入急停锁定。
 *   9. SW2/K2：在蓝牙遥控模式和循迹模式之间切换；急停锁定时无效。
 */

#include "stm32f10x.h"
#include "OLED.h"
#include "LED.h"
#include "Timer.h"
#include "Key.h"
#include "Motor.h"
#include "Encoder.h"
#include "Serial.h"
#include "Grayscale.h"
#include "PWM.h"
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

/* ================================================================
 * 1. 基础控制周期和安全参数
 * ================================================================ */

#define CONTROL_PERIOD_MS              10U
#define BT_TIMEOUT_MS                  600U
#define PLOT_REPORT_PERIOD_MS          100U
#define OLED_REFRESH_PERIOD_MS         100U

#define PWM_LIMIT_MIN                  0.0f
#define PWM_LIMIT_MAX                  ((float)PWM_MAX_DUTY)

/* ================================================================
 * 2. 八路灰度循迹默认参数
 * ================================================================ */

volatile float g_lineBlackLevelF = 1.0f;
volatile float g_lineReverseOrderF = 0.0f;
volatile float g_lineTurnSign = 1.0f;

volatile float g_traceBaseSpeed = 24.0f;
volatile float g_traceSearchSpeed = 8.0f;

volatile float g_lineKp = 0.135f;
volatile float g_lineKd = 0.055f;
volatile float g_lineTurnLimit = 90.0f;
volatile float g_lineLostTurn = 72.0f;
volatile float g_lineFilterAlpha = 0.65f;
volatile float g_lineSlowGain = 0.55f;
volatile float g_lineEdgeTurnExtra = 22.0f;
volatile float g_lineEdgeSpeedRatio = 0.55f;
volatile float g_lineMinTurn = 18.0f;

volatile float g_forwardSlewStep = 3.0f;
volatile float g_turnSlewStep = 14.0f;

/* ================================================================
 * 3. 速度环/差速环 PID 参数
 * ================================================================ */

typedef struct
{
    float Kp;
    float Ki;
    float Kd;
    float Integral;
    float LastError;
    float OutputLimit;
    float IntegralLimit;
} PID_TypeDef;

static PID_TypeDef ForwardPID;
static PID_TypeDef TurnPID;

volatile float g_forwardKp = 14.0f;
volatile float g_forwardKi = 0.55f;
volatile float g_forwardKd = 1.8f;

volatile float g_turnKp = 10.0f;
volatile float g_turnKi = 0.04f;
volatile float g_turnKd = 1.0f;

volatile float g_maxForwardCmd = 70.0f;
volatile float g_maxTurnCmd = 75.0f;

/* ================================================================
 * 4. 运行状态变量
 * ================================================================ */

typedef enum
{
    WORK_BT = 0,
    WORK_TRACING = 1
} WorkMode_t;

volatile WorkMode_t g_workMode = WORK_BT;

volatile float g_targetForwardSpeed = 0.0f;
volatile float g_targetTurnSpeed = 0.0f;

volatile uint8_t g_carEnable = 0;
volatile uint32_t g_lastCmdTickMs = 1000;
volatile uint8_t g_safetyLocked = 0;

volatile float g_btSpeedLimitPercent = 30.0f;
volatile float g_speedScale = 0.30f;
volatile float g_pwmLimit = 300.0f;

volatile float g_leftSpeed = 0.0f;
volatile float g_rightSpeed = 0.0f;
volatile float g_forwardSpeed = 0.0f;
volatile float g_turnSpeed = 0.0f;

volatile float g_speedPwm = 0.0f;
volatile float g_diffPwm = 0.0f;

volatile int16_t g_leftPwm = 0;
volatile int16_t g_rightPwm = 0;
volatile float g_forwardSpeedError = 0.0f;

/* 保留给旧版 BT_proto.c 链接使用。 */
volatile uint8_t g_sendPlot = 0;
volatile uint16_t g_sendDisplay = 0;

volatile uint16_t g_oledRefreshMs = 0;
volatile uint16_t g_plotReportMs = 0;

volatile int16_t g_lineError = 0;
volatile uint8_t g_lineValid = 0;
volatile uint8_t g_lineMask = 0;
volatile uint8_t g_lineRawMask = 0;
volatile int8_t g_lastLineDir = 1;
volatile uint16_t g_lineLostMs = 0;

static volatile float g_lineErrorFiltered = 0.0f;
static volatile float g_lineLastCtrlError = 0.0f;
static volatile uint16_t g_promptMs = 0;

/* ================================================================
 * 5. 小工具函数
 * ================================================================ */

static float absf_local(float x)
{
    return (x >= 0.0f) ? x : -x;
}

static float limit_float(float value, float minVal, float maxVal)
{
    if (value < minVal)
    {
        return minVal;
    }
    if (value > maxVal)
    {
        return maxVal;
    }
    return value;
}

static int16_t limit_i16(int32_t value, int16_t minVal, int16_t maxVal)
{
    if (value < minVal)
    {
        return minVal;
    }
    if (value > maxVal)
    {
        return maxVal;
    }
    return (int16_t)value;
}

static float slew_float(float current, float target, float maxStep)
{
    if (maxStep <= 0.0f)
    {
        return target;
    }
    if (target > current + maxStep)
    {
        return current + maxStep;
    }
    if (target < current - maxStep)
    {
        return current - maxStep;
    }
    return target;
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

static int str_is_down(const char *s)
{
    return str_equal_ignore_case(s, "down") || str_equal_ignore_case(s, "d");
}

static int str_is_name(const char *s, const char *a, const char *b, const char *c)
{
    if (str_equal_ignore_case(s, a))
    {
        return 1;
    }
    if (b && str_equal_ignore_case(s, b))
    {
        return 1;
    }
    if (c && str_equal_ignore_case(s, c))
    {
        return 1;
    }
    return 0;
}

/* ================================================================
 * 6. PID 和停车控制
 * ================================================================ */

static void PID_Reset(PID_TypeDef *pid)
{
    pid->Integral = 0.0f;
    pid->LastError = 0.0f;
}

static float PID_Calc(PID_TypeDef *pid, float target, float measure)
{
    float error;
    float derivative;
    float integralCandidate;
    float output;

    error = target - measure;
    derivative = error - pid->LastError;
    integralCandidate = pid->Integral + error;
    integralCandidate = limit_float(integralCandidate, -pid->IntegralLimit, pid->IntegralLimit);

    output = pid->Kp * error + pid->Ki * integralCandidate + pid->Kd * derivative;

    if (output > pid->OutputLimit)
    {
        output = pid->OutputLimit;
        if (error < 0.0f)
        {
            pid->Integral = integralCandidate;
        }
    }
    else if (output < -pid->OutputLimit)
    {
        output = -pid->OutputLimit;
        if (error > 0.0f)
        {
            pid->Integral = integralCandidate;
        }
    }
    else
    {
        pid->Integral = integralCandidate;
    }

    pid->LastError = error;
    return output;
}

static void Control_Init(void)
{
    ForwardPID.Kp = g_forwardKp;
    ForwardPID.Ki = g_forwardKi;
    ForwardPID.Kd = g_forwardKd;
    ForwardPID.Integral = 0.0f;
    ForwardPID.LastError = 0.0f;
    ForwardPID.OutputLimit = (float)PWM_MAX_DUTY;
    ForwardPID.IntegralLimit = 260.0f;

    TurnPID.Kp = g_turnKp;
    TurnPID.Ki = g_turnKi;
    TurnPID.Kd = g_turnKd;
    TurnPID.Integral = 0.0f;
    TurnPID.LastError = 0.0f;
    TurnPID.OutputLimit = (float)PWM_MAX_DUTY * 0.85f;
    TurnPID.IntegralLimit = 220.0f;
}

static void Control_UpdatePIDParam(void)
{
    ForwardPID.Kp = g_forwardKp;
    ForwardPID.Ki = g_forwardKi;
    ForwardPID.Kd = g_forwardKd;

    TurnPID.Kp = g_turnKp;
    TurnPID.Ki = g_turnKi;
    TurnPID.Kd = g_turnKd;
}

static void Control_ForcePWMZero(void)
{
    g_targetForwardSpeed = 0.0f;
    g_targetTurnSpeed = 0.0f;
    g_speedPwm = 0.0f;
    g_diffPwm = 0.0f;
    g_leftPwm = 0;
    g_rightPwm = 0;
    g_carEnable = 0;
    Motor_StopAll();
    PID_Reset(&ForwardPID);
    PID_Reset(&TurnPID);
}

static void Prompt_Start(uint16_t ms)
{
    g_promptMs = ms;
    LED_ON();
}

static void Prompt_Tick1ms(void)
{
    if (g_promptMs > 0)
    {
        g_promptMs--;
        if ((g_promptMs % 80U) == 0U)
        {
            LED_Turn();
        }
        if (g_promptMs == 0U)
        {
            LED_OFF();
        }
    }
}

/* ================================================================
 * 7. 模式切换和急停接口
 * ================================================================ */

void App_EmergencyStop(void)
{
    g_safetyLocked = 1;
    g_workMode = WORK_BT;
    Control_ForcePWMZero();
    Prompt_Start(500);
}

void App_UnlockControl(void)
{
    g_safetyLocked = 0;
    g_workMode = WORK_BT;
    g_lastCmdTickMs = 0;
    Control_ForcePWMZero();
    Prompt_Start(180);
}

void App_StartBluetoothMode(void)
{
    if (g_safetyLocked)
    {
        return;
    }

    g_workMode = WORK_BT;
    g_lastCmdTickMs = 0;
    Control_ForcePWMZero();
    Prompt_Start(160);
}

void App_StartTracingMode(void)
{
    if (g_safetyLocked)
    {
        return;
    }

    g_workMode = WORK_TRACING;
    g_lineLostMs = 0;
    g_lineErrorFiltered = 0.0f;
    g_lineLastCtrlError = 0.0f;
    g_targetForwardSpeed = 0.0f;
    g_targetTurnSpeed = 0.0f;
    g_carEnable = 1;
    g_lastCmdTickMs = 0;
    Encoder_ClearAll();
    PID_Reset(&ForwardPID);
    PID_Reset(&TurnPID);
    Prompt_Start(160);
}

/* ================================================================
 * 8. 蓝牙/网页协议解析
 * ================================================================ */

static void ApplySpeedLimitPercent(float percent)
{
    float ratio;

    percent = limit_float(percent, 0.0f, 100.0f);
    ratio = percent / 100.0f;

    g_btSpeedLimitPercent = percent;
    g_speedScale = ratio;
    g_pwmLimit = PWM_LIMIT_MAX * ratio;
}

static void Main_ApplySliderPacket(const char *name, float value)
{
    if (str_is_name(name, "RP", "rp", "speedLimit"))
    {
        ApplySpeedLimitPercent(value);
        return;
    }

    if (str_is_name(name, "speedKp", "forwardKp", "fKp"))
    {
        g_forwardKp = limit_float(value, 0.0f, 80.0f);
        return;
    }
    if (str_is_name(name, "speedKi", "forwardKi", "fKi"))
    {
        g_forwardKi = limit_float(value, 0.0f, 20.0f);
        return;
    }
    if (str_is_name(name, "speedKd", "forwardKd", "fKd"))
    {
        g_forwardKd = limit_float(value, 0.0f, 30.0f);
        return;
    }

    if (str_is_name(name, "turnKp", "diffKp", "tKp"))
    {
        g_turnKp = limit_float(value, 0.0f, 80.0f);
        return;
    }
    if (str_is_name(name, "turnKi", "diffKi", "tKi"))
    {
        g_turnKi = limit_float(value, 0.0f, 20.0f);
        return;
    }
    if (str_is_name(name, "turnKd", "diffKd", "tKd"))
    {
        g_turnKd = limit_float(value, 0.0f, 30.0f);
        return;
    }

    if (str_is_name(name, "maxForward", "maxSpeed", "btSpeed"))
    {
        g_maxForwardCmd = limit_float(value, 0.0f, 200.0f);
        return;
    }
    if (str_is_name(name, "maxTurn", "btTurn", "remoteTurn"))
    {
        g_maxTurnCmd = limit_float(value, 0.0f, 200.0f);
        return;
    }

    if (str_is_name(name, "traceKp", "lineKp", "lineP"))
    {
        g_lineKp = limit_float(value, 0.0f, 1.0f);
        return;
    }
    if (str_is_name(name, "traceKd", "lineKd", "lineD"))
    {
        g_lineKd = limit_float(value, 0.0f, 1.0f);
        return;
    }
    if (str_is_name(name, "traceSpeed", "lineSpeed", "baseSpeed"))
    {
        g_traceBaseSpeed = limit_float(value, 0.0f, 120.0f);
        return;
    }
    if (str_is_name(name, "searchSpeed", "lostSpeed", "findSpeed"))
    {
        g_traceSearchSpeed = limit_float(value, 0.0f, 80.0f);
        return;
    }
    if (str_is_name(name, "turnLimit", "lineTurnLimit", "traceTurnLimit"))
    {
        g_lineTurnLimit = limit_float(value, 0.0f, 180.0f);
        return;
    }
    if (str_is_name(name, "lostTurn", "lineLostTurn", "findTurn"))
    {
        g_lineLostTurn = limit_float(value, 0.0f, 180.0f);
        return;
    }
    if (str_is_name(name, "filter", "lineFilter", "alpha"))
    {
        if (value > 1.0f)
        {
            value = value / 100.0f;
        }
        g_lineFilterAlpha = limit_float(value, 0.0f, 1.0f);
        return;
    }
    if (str_is_name(name, "slowGain", "lineSlow", "curveSlow"))
    {
        if (value > 1.0f)
        {
            value = value / 100.0f;
        }
        g_lineSlowGain = limit_float(value, 0.0f, 0.95f);
        return;
    }
    if (str_is_name(name, "edgeBoost", "edgeTurn", "edgeExtra"))
    {
        g_lineEdgeTurnExtra = limit_float(value, 0.0f, 100.0f);
        return;
    }
    if (str_is_name(name, "edgeSlow", "edgeSpeed", "edgeRatio"))
    {
        if (value > 1.0f)
        {
            value = value / 100.0f;
        }
        g_lineEdgeSpeedRatio = limit_float(value, 0.05f, 1.0f);
        return;
    }
    if (str_is_name(name, "minTurn", "lineMinTurn", "traceMinTurn"))
    {
        g_lineMinTurn = limit_float(value, 0.0f, 80.0f);
        return;
    }
    if (str_is_name(name, "forwardSlew", "speedSlew", "fSlew"))
    {
        g_forwardSlewStep = limit_float(value, 0.0f, 30.0f);
        return;
    }
    if (str_is_name(name, "turnSlew", "diffSlew", "tSlew"))
    {
        g_turnSlewStep = limit_float(value, 0.0f, 60.0f);
        return;
    }
    if (str_is_name(name, "lineSign", "traceSign", "turnSign"))
    {
        g_lineTurnSign = (value < 0.0f) ? -1.0f : 1.0f;
        return;
    }
    if (str_is_name(name, "blackLevel", "lineLevel", "black"))
    {
        g_lineBlackLevelF = (value <= 0.0f) ? 0.0f : 1.0f;
        return;
    }
    if (str_is_name(name, "lineReverse", "reverseLine", "sensorReverse"))
    {
        g_lineReverseOrderF = (value <= 0.0f) ? 0.0f : 1.0f;
        return;
    }
}

static void Main_ApplyJoystickPacket(char **tok, int n)
{
    int turnRaw;
    int forwardRaw;
    float maxForward;
    float maxTurn;

    if (n < 3)
    {
        return;
    }
    if (g_safetyLocked || g_workMode != WORK_BT)
    {
        return;
    }

    turnRaw = atoi(tok[1]);
    forwardRaw = atoi(tok[2]);

    maxForward = g_maxForwardCmd * g_speedScale;
    maxTurn = g_maxTurnCmd * g_speedScale;

    g_targetForwardSpeed = limit_float((float)forwardRaw * maxForward / 100.0f, -maxForward, maxForward);
    g_targetTurnSpeed = limit_float((float)(-turnRaw) * maxTurn / 100.0f, -maxTurn, maxTurn);
    g_carEnable = 1;
}

static void Main_ApplyPacket(char *payload)
{
    char *tok[12];
    int n;
    char *p;

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
        return;
    }

    if (str_is_name(tok[0], "key", "k", 0))
    {
        if (n >= 3 && str_is_down(tok[2]))
        {
            if (str_is_name(tok[1], "emergency", "emg", "stop"))
            {
                App_EmergencyStop();
                return;
            }
            if (str_is_name(tok[1], "unlock", "release", "resume"))
            {
                App_UnlockControl();
                return;
            }
            if (str_is_name(tok[1], "tracing", "trace", "line"))
            {
                App_StartTracingMode();
                return;
            }
            if (str_is_name(tok[1], "Bluetooth", "BT", "remote"))
            {
                App_StartBluetoothMode();
                return;
            }
        }
        return;
    }

    if (str_is_name(tok[0], "slider", "s", 0))
    {
        if (n >= 3)
        {
            Main_ApplySliderPacket(tok[1], (float)atof(tok[2]));
        }
        return;
    }

    if (str_is_name(tok[0], "joystick", "j", 0))
    {
        Main_ApplyJoystickPacket(tok, n);
        return;
    }

    if (str_is_name(tok[0], "cmd", "car", "vel"))
    {
        float maxForward;
        float maxTurn;

        if (n >= 3 && !g_safetyLocked && g_workMode == WORK_BT)
        {
            maxForward = g_maxForwardCmd * g_speedScale;
            maxTurn = g_maxTurnCmd * g_speedScale;
            g_targetForwardSpeed = limit_float((float)atof(tok[1]), -maxForward, maxForward);
            g_targetTurnSpeed = limit_float((float)atof(tok[2]), -maxTurn, maxTurn);
            g_carEnable = 1;
        }
        return;
    }
}

static void Main_BTProcess(void)
{
    static uint8_t receiving = 0;
    static uint8_t index = 0;
    static char packet[128];
    uint8_t byte;

    while (Serial_ReadByte(&byte))
    {
        char c;
        c = (char)byte;

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
            Main_ApplyPacket(packet);
            g_lastCmdTickMs = 0;
            continue;
        }

        if (c == '\r' || c == '\n')
        {
            continue;
        }

        if (index < sizeof(packet) - 1U)
        {
            packet[index++] = c;
        }
        else
        {
            receiving = 0;
            index = 0;
        }
    }
}

/* ================================================================
 * 9. 八路灰度循迹算法 + main.c GPIO 直读
 * ================================================================ */

#define LINE_AD0_PORT      GPIOA
#define LINE_AD0_PIN       GPIO_Pin_8

#define LINE_AD1_PORT      GPIOB
#define LINE_AD1_PIN       GPIO_Pin_3

#define LINE_AD2_PORT      GPIOB
#define LINE_AD2_PIN       GPIO_Pin_4

#define LINE_OUT_PORT      GPIOB
#define LINE_OUT_PIN       GPIO_Pin_0

static void Line_GPIOForceInit(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB, ENABLE);

    /* PB3/PB4 默认属于 JTAG。这里强制关闭 JTAG，只保留 SWD。 */
    GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);

    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

    GPIO_InitStructure.GPIO_Pin = LINE_AD0_PIN;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = LINE_AD1_PIN | LINE_AD2_PIN;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_InitStructure.GPIO_Pin = LINE_OUT_PIN;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    GPIO_ResetBits(LINE_AD0_PORT, LINE_AD0_PIN);
    GPIO_ResetBits(LINE_AD1_PORT, LINE_AD1_PIN);
    GPIO_ResetBits(LINE_AD2_PORT, LINE_AD2_PIN);
}

static void Line_SelectChannelDirect(uint8_t channel)
{
    if (channel & 0x01U)
    {
        GPIO_SetBits(LINE_AD0_PORT, LINE_AD0_PIN);
    }
    else
    {
        GPIO_ResetBits(LINE_AD0_PORT, LINE_AD0_PIN);
    }

    if (channel & 0x02U)
    {
        GPIO_SetBits(LINE_AD1_PORT, LINE_AD1_PIN);
    }
    else
    {
        GPIO_ResetBits(LINE_AD1_PORT, LINE_AD1_PIN);
    }

    if (channel & 0x04U)
    {
        GPIO_SetBits(LINE_AD2_PORT, LINE_AD2_PIN);
    }
    else
    {
        GPIO_ResetBits(LINE_AD2_PORT, LINE_AD2_PIN);
    }

    for (volatile uint16_t d = 0; d < 2000; d++)
    {
        __NOP();
    }
}

static uint8_t Line_ReadOneDirect(uint8_t channel)
{
    uint8_t a;
    uint8_t b;
    uint8_t c;

    Line_SelectChannelDirect(channel);

    a = (uint8_t)GPIO_ReadInputDataBit(LINE_OUT_PORT, LINE_OUT_PIN);

    for (volatile uint16_t d = 0; d < 300; d++)
    {
        __NOP();
    }

    b = (uint8_t)GPIO_ReadInputDataBit(LINE_OUT_PORT, LINE_OUT_PIN);

    for (volatile uint16_t d = 0; d < 300; d++)
    {
        __NOP();
    }

    c = (uint8_t)GPIO_ReadInputDataBit(LINE_OUT_PORT, LINE_OUT_PIN);

    return (uint8_t)(((uint16_t)a + (uint16_t)b + (uint16_t)c) >= 2U);
}

static void Line_ReadAllDirect(uint8_t raw[GRAYSCALE_CHANNELS])
{
    uint8_t i;

    for (i = 0; i < GRAYSCALE_CHANNELS; i++)
    {
        raw[i] = Line_ReadOneDirect(i);
    }
}

static void Line_Update(void)
{
    uint8_t raw[GRAYSCALE_CHANNELS];
    static const int16_t weight[GRAYSCALE_CHANNELS] = {-350, -250, -150, -50, 50, 150, 250, 350};
    int32_t sum;
    int16_t count;
    uint8_t mask;
    uint8_t i;
    uint8_t blackLevel;
    uint8_t reverseOrder;

    sum = 0;
    count = 0;
    mask = 0;
    blackLevel = (g_lineBlackLevelF <= 0.5f) ? 0U : 1U;
    reverseOrder = (g_lineReverseOrderF <= 0.5f) ? 0U : 1U;

    Line_ReadAllDirect(raw);

    g_lineRawMask = 0;
    for (i = 0; i < GRAYSCALE_CHANNELS; i++)
    {
        if (raw[i] == blackLevel)
        {
            g_lineRawMask |= (uint8_t)(1U << i);
        }
    }

    for (i = 0; i < GRAYSCALE_CHANNELS; i++)
    {
        uint8_t physicalIndex;
        uint8_t isBlack;

        physicalIndex = reverseOrder ? (uint8_t)(GRAYSCALE_CHANNELS - 1U - i) : i;
        isBlack = (raw[physicalIndex] == blackLevel) ? 1U : 0U;

        if (isBlack)
        {
            mask |= (uint8_t)(1U << i);
            sum += weight[i];
            count++;
        }
    }

    g_lineMask = mask;

    if (count > 0)
    {
        float rawError;
        float alpha;

        g_lineValid = 1;
        rawError = (float)(sum / count);
        alpha = limit_float(g_lineFilterAlpha, 0.0f, 1.0f);

        g_lineErrorFiltered = g_lineErrorFiltered * (1.0f - alpha) + rawError * alpha;
        g_lineError = (int16_t)g_lineErrorFiltered;
        g_lastLineDir = (g_lineError >= 0) ? 1 : -1;
        g_lineLostMs = 0;
    }
    else
    {
        g_lineValid = 0;
        if (g_lineLostMs < 60000U)
        {
            g_lineLostMs += CONTROL_PERIOD_MS;
        }
    }
}

static float Line_CalcTurnCmd(void)
{
    float error;
    float dError;
    float desiredSign;
    float turn;

    error = (float)g_lineError;
    dError = error - g_lineLastCtrlError;
    g_lineLastCtrlError = error;

    desiredSign = (((-g_lineTurnSign) * error) >= 0.0f) ? 1.0f : -1.0f;
    turn = (-g_lineTurnSign) * (error * g_lineKp + dError * g_lineKd);

    if (absf_local(error) > 70.0f && absf_local(turn) < g_lineMinTurn)
    {
        turn = desiredSign * g_lineMinTurn;
    }

    if ((g_lineMask & 0xC3U) != 0U)
    {
        turn += desiredSign * g_lineEdgeTurnExtra;
    }

    return limit_float(turn, -g_lineTurnLimit, g_lineTurnLimit);
}

static void Tracing_Control10ms(void)
{
    float forward;
    float turn;

    Line_Update();

    if (g_speedScale <= 0.01f || g_pwmLimit <= 0.5f)
    {
        g_targetForwardSpeed = 0.0f;
        g_targetTurnSpeed = 0.0f;
        g_carEnable = 0;
        return;
    }

    if (g_lineValid)
    {
        float e;
        float slowRatio;
        float minForward;

        e = absf_local((float)g_lineError) / 350.0f;
        if (e > 1.0f)
        {
            e = 1.0f;
        }

        slowRatio = 1.0f - limit_float(g_lineSlowGain, 0.0f, 0.95f) * e;

        if ((g_lineMask & 0xC3U) != 0U)
        {
            slowRatio *= limit_float(g_lineEdgeSpeedRatio, 0.05f, 1.0f);
        }

        forward = g_traceBaseSpeed * g_speedScale * slowRatio;
        minForward = g_traceSearchSpeed * g_speedScale;
        if (forward < minForward)
        {
            forward = minForward;
        }

        turn = Line_CalcTurnCmd() * g_speedScale;
    }
    else
    {
        forward = g_traceSearchSpeed * g_speedScale;
        turn = (-g_lineTurnSign) * (float)g_lastLineDir * g_lineLostTurn * g_speedScale;
    }

    g_targetForwardSpeed = slew_float(g_targetForwardSpeed, forward, g_forwardSlewStep);
    g_targetTurnSpeed = slew_float(g_targetTurnSpeed, turn, g_turnSlewStep);
    g_carEnable = 1;
}

/* ================================================================
 * 10. 速度闭环、电机输出、显示和按键
 * ================================================================ */

static void Control_Run10ms(void)
{
    int16_t leftDelta;
    int16_t rightDelta;
    float pwmLimit;
    int32_t leftPwmTemp;
    int32_t rightPwmTemp;

    leftDelta = Encoder_GetLeftDelta();
    rightDelta = Encoder_GetRightDelta();

    g_leftSpeed = (float)leftDelta;
    g_rightSpeed = (float)rightDelta;
    g_forwardSpeed = (g_leftSpeed + g_rightSpeed) * 0.5f;
    g_turnSpeed = (g_rightSpeed - g_leftSpeed) * 0.5f;

    if (g_safetyLocked)
    {
        Control_ForcePWMZero();
        return;
    }

    Control_UpdatePIDParam();

    if (g_workMode == WORK_TRACING)
    {
        Tracing_Control10ms();
    }
    else
    {
        if (g_lastCmdTickMs > BT_TIMEOUT_MS)
        {
            g_targetForwardSpeed = 0.0f;
            g_targetTurnSpeed = 0.0f;
            g_carEnable = 0;
        }
    }

    if (!g_carEnable || g_pwmLimit <= 0.5f)
    {
        g_forwardSpeedError = g_targetForwardSpeed - g_forwardSpeed;
        g_speedPwm = 0.0f;
        g_diffPwm = 0.0f;
        g_leftPwm = 0;
        g_rightPwm = 0;
        Motor_StopAll();
        PID_Reset(&ForwardPID);
        PID_Reset(&TurnPID);
        return;
    }

    pwmLimit = limit_float(g_pwmLimit, PWM_LIMIT_MIN, PWM_LIMIT_MAX);
    ForwardPID.OutputLimit = pwmLimit;
    TurnPID.OutputLimit = pwmLimit * 0.85f;

    g_forwardSpeedError = g_targetForwardSpeed - g_forwardSpeed;
    g_speedPwm = PID_Calc(&ForwardPID, g_targetForwardSpeed, g_forwardSpeed);
    g_diffPwm = PID_Calc(&TurnPID, g_targetTurnSpeed, g_turnSpeed);

    leftPwmTemp = (int32_t)(g_speedPwm - g_diffPwm);
    rightPwmTemp = (int32_t)(g_speedPwm + g_diffPwm);

    g_leftPwm = limit_i16(leftPwmTemp, (int16_t)(-pwmLimit), (int16_t)pwmLimit);
    g_rightPwm = limit_i16(rightPwmTemp, (int16_t)(-pwmLimit), (int16_t)pwmLimit);

    Motor_SetPWM(g_leftPwm, g_rightPwm);
}

static char *ModeString(void)
{
    if (g_safetyLocked)
    {
        return "LOCK";
    }
    if (g_workMode == WORK_TRACING)
    {
        return "TRACE";
    }
    return "BT";
}


static void Serial_SendPlotStatus(void)
{
    int modeCode;

    /*
     * 调参网页绘图协议：短格式 [p,数值1,数值2,...]
     * 依次回传：
     * CH1  modeCode：0=蓝牙遥控，1=循迹，9=急停锁定
     * CH2  灰度误差：g_lineError
     * CH3  灰度 Mask：g_lineMask
     * CH4  目标前进速度：g_targetForwardSpeed
     * CH5  实际前进速度：g_forwardSpeed
     * CH6  目标转向速度：g_targetTurnSpeed
     * CH7  实际转向速度：g_turnSpeed
     * CH8  左轮 PWM：g_leftPwm
     * CH9  右轮 PWM：g_rightPwm
     * CH10 灰度有效标志：g_lineValid
     */

    modeCode = g_safetyLocked ? 9 : (int)g_workMode;

    Serial_Printf("[p,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d]\r\n",
                  modeCode,
                  (int)g_lineError,
                  (int)g_lineMask,
                  (int)g_targetForwardSpeed,
                  (int)g_forwardSpeed,
                  (int)g_targetTurnSpeed,
                  (int)g_turnSpeed,
                  (int)g_leftPwm,
                  (int)g_rightPwm,
                  (int)g_lineValid);
}


static void OLED_ShowStatus(void)
{
    OLED_Printf(0, 0, OLED_8X16, "M:%s LK:%d RP:%03d", ModeString(), (int)g_safetyLocked, (int)g_btSpeedLimitPercent);
    OLED_Printf(0, 16, OLED_8X16, "T:%+04d V:%+04d", (int)g_targetForwardSpeed, (int)g_forwardSpeed);
    OLED_Printf(0, 32, OLED_8X16, "L:%+04d R:%+04d", (int)g_leftPwm, (int)g_rightPwm);
    OLED_Printf(0, 48, OLED_8X16, "R:%02X M:%02X E:%+03d", (int)g_lineRawMask, (int)g_lineMask, (int)(g_lineError / 10));
    OLED_Update();
}

static void Main_KeyProcess(void)
{
    uint8_t key;

    key = Key_GetNum();
    if (key == 0U)
    {
        return;
    }

    if (key == 1U)
    {
        ApplySpeedLimitPercent(0.0f);
        Control_ForcePWMZero();
        Prompt_Start(160);
        return;
    }

    if (key == 2U)
    {
        if (g_safetyLocked)
        {
            Prompt_Start(80);
            return;
        }

        if (g_workMode == WORK_BT)
        {
            App_StartTracingMode();
        }
        else
        {
            App_StartBluetoothMode();
        }
        return;
    }
}

/* ================================================================
 * 11. 主函数和 1ms 定时中断
 * ================================================================ */

int main(void)
{
    OLED_Init();
    LED_Init();
    Key_Init();
    Grayscale_Init();
    Motor_Init();
    Encoder_Init();

    /* 放在可能碰到 AFIO/JTAG/GPIO 的初始化后面，再强制配置一次灰度接口。 */
    Line_GPIOForceInit();

    Serial_Init();
    Timer_Init();
    Control_Init();

    ApplySpeedLimitPercent(g_btSpeedLimitPercent);
    LED_OFF();

    OLED_Printf(0, 0, OLED_8X16, "BT/Trace Car");
    OLED_Printf(0, 16, OLED_8X16, "Default: BT");
    OLED_Printf(0, 32, OLED_8X16, "RP:%d%%", (int)g_btSpeedLimitPercent);
    OLED_Printf(0, 48, OLED_8X16, "Ready");
    OLED_Update();

    while (1)
    {
        Main_BTProcess();
        Main_KeyProcess();

        if (g_plotReportMs >= PLOT_REPORT_PERIOD_MS)
        {
            g_plotReportMs = 0;
            Serial_SendPlotStatus();
        }

        if (g_oledRefreshMs >= OLED_REFRESH_PERIOD_MS)
        {
            g_oledRefreshMs = 0;
            OLED_ShowStatus();
        }
    }
}

void TIM1_UP_IRQHandler(void)
{
    static uint8_t controlDiv = 0;

    if (TIM_GetITStatus(TIM1, TIM_IT_Update) == SET)
    {
        TIM_ClearITPendingBit(TIM1, TIM_IT_Update);

        Key_Tick();
        Prompt_Tick1ms();

        if (g_lastCmdTickMs < 60000U)
        {
            g_lastCmdTickMs++;
        }

        if (g_oledRefreshMs < 60000U)
        {
            g_oledRefreshMs++;
        }
        if (g_plotReportMs < 60000U)
        {
            g_plotReportMs++;
        }

        controlDiv++;
        if (controlDiv >= CONTROL_PERIOD_MS)
        {
            controlDiv = 0;
            Control_Run10ms();
        }
    }
}
