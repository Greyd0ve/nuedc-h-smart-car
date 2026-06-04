/*
 * 文件：User/main.c
 * 版本：蓝牙遥控 + 八路灰度循迹 + presetFast + PB1/PB5 声光提示
 *       + 编码器累计调试 + MPU6050 yaw 调试 + straight 直线航向保持测试
 *       + task1 最小状态机版 + arcTest 半圆弧结束判断测试版。
 *
 * 本版说明：
 *   1. 不再使用 PC13 板载 LED。
 *   2. PB1 用作 BEEP 蜂鸣器输出，蜂鸣器为低电平触发。
 *   3. PB5 用作 LED_EXT 外接提示灯输出，LED 为高电平点亮。
 *   4. Prompt_Start()/Prompt_Tick1ms() 只控制 PB1 和 PB5。
 *   5. 保留编码器累计调试模式，用于 100 cm 距离标定。
 *   6. 新增 MPU6050 yaw 调试模式，用于 GyroZ 零偏校准、yaw 积分和网页/OLED 回传。
 *   7. 保留 MPU6050 诊断信息：错误码、WHO_AM_I 读数、初始化尝试次数。
 *   8. 保留 [key,straight,down] 直线航向保持测试。
 *   9. 新增 [key,task1,down] 选择 H 题任务 1，[key,start,down] 开始执行。
 *   10. task1 第一版只做 A -> B 直线 100 cm，到 B 停车并声光提示。
 *   11. 新增 [key,arcTest,down] 半圆弧结束判断测试。
 *   12. 开机默认普通 BT 模式，按下 K2/SW2 后切换到编码器调试模式；
 *      再按一次 K2/SW2 返回普通 BT 显示模式。
 *
 * 保留功能：
 *   1. 上电默认进入蓝牙遥控模式。
 *   2. 网页发送 [key,tracing,down] 进入八路灰度循迹模式。
 *   3. 网页发送 [key,Bluetooth,down] 返回蓝牙遥控模式。
 *   4. 网页发送 [key,emergency,down] 立即清零 PWM 并进入急停锁定。
 *   5. 网页发送 [key,unlock,down] 解除急停锁定。
 *   6. 网页发送 [slider,RP,0~100] 控制最大 PWM 百分比。
 *   7. K1/SW1 将 RP 设为 0%，PWM 清零，但不进入急停锁定。
 *   8. K2/SW2 在普通 BT 显示模式和编码器调试显示模式之间切换。
 *   9. 网页发送 [key,presetFast,down] 加载高速稳定参数 v1。
 *   10. K3/SW3 加载高速稳定参数 v1。
 *   11. K4/SW4 清零编码器累计脉冲。
 *   12. [key,straight,down] 使用编码器距离 + MPU6050 yaw 保持直线并自动停车。
 *   13. [key,task1,down] + [key,start,down] 执行 H 题 task1：A -> B 停车。
 *   14. [key,arcTest,down] 从半圆弧起点开始循迹，并用 yaw 角度判断半圆结束。
 */

#include "stm32f10x.h"
#include "OLED.h"
#include "Timer.h"
#include "Key.h"
#include "Motor.h"
#include "Encoder.h"
#include "Serial.h"
#include "Grayscale.h"
#include "PWM.h"
#include "MPU6050.h"
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
#define MPU_UPDATE_PERIOD_MS           10U

#define PWM_LIMIT_MIN                  0.0f
#define PWM_LIMIT_MAX                  ((float)PWM_MAX_DUTY)

/* MPU6050 静止校准参数：300 次，每次约 3ms，总时长约 0.9s。 */
#define MPU_CALIB_SAMPLES              300U
#define MPU_CALIB_MIN_OK               240U

/* 直线距离标定结果：100cm 约 7030 个左右平均编码器脉冲。 */
#define ENCODER_PULSE_PER_CM           70.30f
#define STRAIGHT_DISTANCE_DEFAULT      7030.0f

/* straightSpeed = 15 时实测停车惯性补偿约 170 脉冲。
 * task1 中使用 7030 - 170 = 6860 作为停止判断阈值，
 * 实车测试实际停车约 99.7 cm。
 */
#define STRAIGHT_STOP_OFFSET_DEFAULT   170.0f

/* 半圆弧结束判断默认参数。
 * arcMinTime：最短运行时间，防止刚起步时误判。
 * arcYawTarget：半圆弧目标 yaw 变化角度，先用 165°，后续根据停车点微调。
 * arcMaxTime：安全超时，防止因丢线或 yaw 判断失败而一直跑。
 */
#define ARC_MIN_TIME_DEFAULT_MS        1500U
#define ARC_MAX_TIME_DEFAULT_MS        6000U
#define ARC_YAW_TARGET_DEFAULT         165.0f

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
 * 3. 速度环和转向环 PID 参数
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

typedef enum
{
    TASK_IDLE = 0,
    TASK_READY_TASK1,
    TASK_STRAIGHT_AB,
    TASK_STOP_AT_B,
    TASK_FINISH
} TaskState_t;

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

/* 编码器距离调试累计值，在 Control_Run10ms() 中每 10ms 更新一次。 */
volatile int32_t g_leftEncoderTotal = 0;
volatile int32_t g_rightEncoderTotal = 0;
volatile int32_t g_forwardEncoderTotal = 0;
volatile int32_t g_turnEncoderTotal = 0;

/* 0 = 普通网页回传，1 = 编码器调试回传，2 = MPU6050 yaw 调试回传，
 * 3 = straight 直线测试回传，4 = arcTest 半圆弧测试回传。
 */
volatile uint8_t g_plotMode = 0;

volatile float g_speedPwm = 0.0f;
volatile float g_diffPwm = 0.0f;

volatile int16_t g_leftPwm = 0;
volatile int16_t g_rightPwm = 0;
volatile float g_forwardSpeedError = 0.0f;

/* 保留该变量，用于兼容旧版 BT_proto.c 的链接引用。 */
volatile uint8_t g_sendPlot = 0;
volatile uint16_t g_sendDisplay = 0;

volatile uint16_t g_oledRefreshMs = 0;
volatile uint16_t g_plotReportMs = 0;
volatile uint16_t g_mpuUpdateMs = 0;

volatile int16_t g_lineError = 0;
volatile uint8_t g_lineValid = 0;
volatile uint8_t g_lineMask = 0;
volatile uint8_t g_lineRawMask = 0;
volatile int8_t g_lastLineDir = 1;
volatile uint16_t g_lineLostMs = 0;

static volatile float g_lineErrorFiltered = 0.0f;
static volatile float g_lineLastCtrlError = 0.0f;
static volatile uint16_t g_promptMs = 0;

/* MPU6050 yaw 调试状态。 */
volatile uint8_t g_mpuReady = 0;
volatile uint8_t g_mpuErr = 0;
volatile uint8_t g_mpuCalibrated = 0;
volatile uint8_t g_mpuCalibrating = 0;

/* MPU6050 诊断信息：
 *   g_mpuWhoAmI = WHO_AM_I 寄存器读数，正常应为 0x68。
 *   g_mpuInitTryCount = 调用 MPU_AppInit() 的次数，方便确认命令是否真的触发。
 */
volatile uint8_t g_mpuWhoAmI = 0x00U;
volatile uint8_t g_mpuInitTryCount = 0U;

/* 如果实测逆时针旋转时 yaw 方向相反，可通过 [slider,yawSign,-1] 取反。 */
volatile float g_mpuYawSign = 1.0f;
volatile float g_gyroZRawDps = 0.0f;
volatile float g_gyroZBiasDps = 0.0f;
volatile float g_gyroZDps = 0.0f;
volatile float g_yawDeg = 0.0f;
volatile float g_yawTotalDeg = 0.0f;

/* straight 直线航向保持测试参数。
 * g_straightDistancePulse 使用编码器平均累计脉冲作为距离单位。
 */
volatile uint8_t g_straightActive = 0;
volatile uint8_t g_straightDone = 0;
volatile float g_straightSpeed = 18.0f;
volatile float g_straightDistancePulse = STRAIGHT_DISTANCE_DEFAULT;
volatile float g_straightTargetYaw = 0.0f;
volatile float g_straightYawError = 0.0f;
volatile float g_yawKp = 1.8f;
volatile float g_yawKd = 0.15f;

/* arcTest 半圆弧结束判断测试参数。
 * 从半圆弧起点放车，车头沿弧线切线方向，发送 [key,arcTest,down] 后开始灰度循迹。
 * 程序记录起始 yaw，并在运行时间超过 arcMinTime 后，
 * 当 abs(deltaYaw) 超过 arcYawTarget 时自动停车。
 */
volatile uint8_t g_arcActive = 0;
volatile uint8_t g_arcDone = 0;
volatile uint16_t g_arcRunMs = 0;
volatile uint16_t g_arcMinTimeMs = ARC_MIN_TIME_DEFAULT_MS;
volatile uint16_t g_arcMaxTimeMs = ARC_MAX_TIME_DEFAULT_MS;
volatile float g_arcYawTarget = ARC_YAW_TARGET_DEFAULT;
volatile float g_arcStartYaw = 0.0f;
volatile float g_arcDeltaYaw = 0.0f;

/* H 题 task1 最小状态机参数。
 * task1 目标是 A -> B 直线 100 cm，到 B 停车并声光提示。
 * g_task1DistancePulse 表示真实目标距离脉冲，默认 7030。
 * g_taskStopOffsetPulse 表示提前停车补偿，默认 170。
 * 实际给 straight 控制使用的停止阈值为：7030 - 170 = 6860。
 */
volatile TaskState_t g_taskState = TASK_IDLE;
volatile uint8_t g_taskSelected = 0;
volatile uint8_t g_taskRunning = 0;
volatile float g_task1DistancePulse = STRAIGHT_DISTANCE_DEFAULT;
volatile float g_taskStopOffsetPulse = STRAIGHT_STOP_OFFSET_DEFAULT;

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

static float wrap_180(float angle)
{
    while (angle > 180.0f)
    {
        angle -= 360.0f;
    }
    while (angle < -180.0f)
    {
        angle += 360.0f;
    }
    return angle;
}

static void Main_DelayMs(uint16_t ms)
{
    uint16_t i;
    volatile uint16_t j;

    for (i = 0; i < ms; i++)
    {
        for (j = 0; j < 7200U; j++)
        {
            __NOP();
        }
    }
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
 * 6. PB1 蜂鸣器与 PB5 外接 LED 声光提示输出
 * ================================================================ */

#define PROMPT_BEEP_PORT       GPIOB
#define PROMPT_BEEP_PIN        GPIO_Pin_1
#define PROMPT_LED_PORT        GPIOB
#define PROMPT_LED_PIN         GPIO_Pin_5

static void PromptIO_BeepOn(void)
{
    /* 蜂鸣器为低电平触发：PB1 = 0 时响。 */
    GPIO_ResetBits(PROMPT_BEEP_PORT, PROMPT_BEEP_PIN);
}

static void PromptIO_BeepOff(void)
{
    GPIO_SetBits(PROMPT_BEEP_PORT, PROMPT_BEEP_PIN);
}

static void PromptIO_BeepTurn(void)
{
    if (GPIO_ReadOutputDataBit(PROMPT_BEEP_PORT, PROMPT_BEEP_PIN))
    {
        PromptIO_BeepOn();
    }
    else
    {
        PromptIO_BeepOff();
    }
}

static void PromptIO_LedOn(void)
{
    /* 外接 LED 为高电平点亮：PB5 = 1 时亮。 */
    GPIO_SetBits(PROMPT_LED_PORT, PROMPT_LED_PIN);
}

static void PromptIO_LedOff(void)
{
    GPIO_ResetBits(PROMPT_LED_PORT, PROMPT_LED_PIN);
}

static void PromptIO_LedTurn(void)
{
    if (GPIO_ReadOutputDataBit(PROMPT_LED_PORT, PROMPT_LED_PIN))
    {
        PromptIO_LedOff();
    }
    else
    {
        PromptIO_LedOn();
    }
}

static void PromptIO_AllOn(void)
{
    PromptIO_BeepOn();
    PromptIO_LedOn();
}

static void PromptIO_AllOff(void)
{
    PromptIO_BeepOff();
    PromptIO_LedOff();
}

static void PromptIO_AllTurn(void)
{
    PromptIO_BeepTurn();
    PromptIO_LedTurn();
}

static void PromptIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Pin = PROMPT_BEEP_PIN | PROMPT_LED_PIN;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    PromptIO_AllOff();
}

/* ================================================================
 * 7. PID、停车控制、提示和编码器清零
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
    g_straightActive = 0;
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
    PromptIO_AllOn();
}

static void Prompt_Tick1ms(void)
{
    if (g_promptMs > 0)
    {
        g_promptMs--;
        if ((g_promptMs % 80U) == 0U)
        {
            PromptIO_AllTurn();
        }
        if (g_promptMs == 0U)
        {
            PromptIO_AllOff();
        }
    }
}

static void Encoder_DebugClearTotals(void)
{
    __disable_irq();

    g_leftEncoderTotal = 0;
    g_rightEncoderTotal = 0;
    g_forwardEncoderTotal = 0;
    g_turnEncoderTotal = 0;

    Encoder_ClearAll();

    __enable_irq();

    Prompt_Start(180);
}

/* ================================================================
 * 8. MPU6050 yaw 读取、积分和零偏校准
 * ================================================================ */

static void MPU_ResetYaw(void)
{
    g_yawDeg = 0.0f;
    g_yawTotalDeg = 0.0f;
}

static void MPU_AppInit(void)
{
    /*
     * 诊断流程：
     *   1. 先初始化 PB8/PB9 软件 I2C 总线；
     *   2. 直接读取 WHO_AM_I，正常 MPU6050 应返回 0x68；
     *   3. 再执行完整 MPU6050_Init()，配置采样率、滤波和量程；
     *   4. 将错误码、WHO_AM_I 和 ready 标志回传到网页/OLED。
     */
    if (g_mpuInitTryCount < 255U)
    {
        g_mpuInitTryCount++;
    }

    MPU6050_InitBus();
    Main_DelayMs(20);
    g_mpuWhoAmI = MPU6050_GetID();

    g_mpuErr = MPU6050_Init();

    if (g_mpuErr == MPU6050_OK)
    {
        g_mpuReady = 1;
        g_mpuCalibrated = 0;
        g_gyroZRawDps = 0.0f;
        g_gyroZBiasDps = 0.0f;
        g_gyroZDps = 0.0f;
        MPU_ResetYaw();
    }
    else
    {
        g_mpuReady = 0;
        g_mpuCalibrated = 0;
    }
}

static void MPU_UpdateYaw(uint16_t elapsedMs)
{
    MPU6050_Data_t data;
    float dt;

    if (!g_mpuReady || g_mpuCalibrating)
    {
        return;
    }

    if (elapsedMs == 0U)
    {
        return;
    }

    if (elapsedMs > 50U)
    {
        elapsedMs = 50U;
    }

    if (MPU6050_ReadData(&data) != MPU6050_OK)
    {
        g_mpuReady = 0;
        g_mpuErr = 0xE1U;
        return;
    }

    dt = (float)elapsedMs * 0.001f;

    g_gyroZRawDps = data.GyroZ_dps;
    g_gyroZDps = (data.GyroZ_dps - g_gyroZBiasDps) * g_mpuYawSign;

    g_yawTotalDeg += g_gyroZDps * dt;
    g_yawDeg = wrap_180(g_yawDeg + g_gyroZDps * dt);
}

static void MPU_CalibrateGyroZ(void)
{
    MPU6050_Data_t data;
    uint16_t i;
    uint16_t okCount;
    float sum;

    /* 校准必须保持静止，并强制回到蓝牙模式停车，避免车在校准时运动。 */
    g_workMode = WORK_BT;
    Control_ForcePWMZero();

    g_mpuCalibrating = 1;
    g_mpuCalibrated = 0;
    Prompt_Start(600);

    if (!g_mpuReady)
    {
        MPU_AppInit();
    }

    if (!g_mpuReady)
    {
        g_mpuCalibrating = 0;
        return;
    }

    sum = 0.0f;
    okCount = 0;

    for (i = 0; i < MPU_CALIB_SAMPLES; i++)
    {
        if (MPU6050_ReadData(&data) == MPU6050_OK)
        {
            sum += data.GyroZ_dps;
            okCount++;
        }

        Main_DelayMs(3);
    }

    if (okCount >= MPU_CALIB_MIN_OK)
    {
        g_gyroZBiasDps = sum / (float)okCount;
        g_gyroZRawDps = g_gyroZBiasDps;
        g_gyroZDps = 0.0f;
        MPU_ResetYaw();
        g_mpuCalibrated = 1;
        g_mpuErr = MPU6050_OK;
        Prompt_Start(300);
    }
    else
    {
        g_mpuErr = 0xE2U;
        Prompt_Start(900);
    }

    g_mpuUpdateMs = 0;
    g_mpuCalibrating = 0;
}

/* ================================================================
 * 9. straight 直线航向保持测试
 * ================================================================ */

static void Straight_Start(void)
{
    if (g_safetyLocked)
    {
        Prompt_Start(80);
        return;
    }

    if (!g_mpuReady)
    {
        MPU_AppInit();
    }

    /* 直线航向保持依赖 yaw 零偏校准。未校准时不启动，避免小车跑偏。 */
    if (!g_mpuReady || !g_mpuCalibrated)
    {
        g_plotMode = 2U;
        Control_ForcePWMZero();
        Prompt_Start(900);
        return;
    }

    g_workMode = WORK_BT;
    g_plotMode = 3U;
    g_straightDone = 0;

    Control_ForcePWMZero();
    Encoder_DebugClearTotals();

    g_straightTargetYaw = g_yawDeg;
    g_straightYawError = 0.0f;
    g_targetForwardSpeed = 0.0f;
    g_targetTurnSpeed = 0.0f;
    g_carEnable = 1;
    g_straightActive = 1;
    g_lastCmdTickMs = 0;

    PID_Reset(&ForwardPID);
    PID_Reset(&TurnPID);
    Prompt_Start(180);
}

static void Straight_Finish(void)
{
    g_straightActive = 0;
    g_straightDone = 1;
    Control_ForcePWMZero();
    g_plotMode = 3U;

    if (g_taskState == TASK_STRAIGHT_AB)
    {
        g_taskState = TASK_STOP_AT_B;
        g_taskRunning = 0;
        /* task1 到达 B 点，停车声光提示时间稍长一点，方便观察。 */
        Prompt_Start(800);
        g_taskState = TASK_FINISH;
    }
    else
    {
        Prompt_Start(500);
    }
}

static void Straight_Control10ms(void)
{
    float targetDistance;
    float turnCmd;

    if (!g_straightActive)
    {
        return;
    }

    targetDistance = limit_float(g_straightDistancePulse, 0.0f, 30000.0f);

    if ((float)g_forwardEncoderTotal >= targetDistance)
    {
        Straight_Finish();
        return;
    }

    g_straightYawError = wrap_180(g_straightTargetYaw - g_yawDeg);

    /* 航向保持：偏航角误差负责拉回方向，GyroZ 项用于抑制摆动。 */
    turnCmd = g_yawKp * g_straightYawError - g_yawKd * g_gyroZDps;
    turnCmd = limit_float(turnCmd, -g_maxTurnCmd, g_maxTurnCmd);

    g_targetForwardSpeed = slew_float(g_targetForwardSpeed, g_straightSpeed, g_forwardSlewStep);
    g_targetTurnSpeed = slew_float(g_targetTurnSpeed, turnCmd, g_turnSlewStep);
    g_carEnable = 1;
    g_lastCmdTickMs = 0;
}

/* ================================================================
 * 10. arcTest 半圆弧结束判断测试
 * ================================================================ */

static void Arc_Finish(void)
{
    g_arcActive = 0;
    g_arcDone = 1;
    g_workMode = WORK_BT;
    Control_ForcePWMZero();
    g_plotMode = 4U;
    Prompt_Start(700);
}

static void Arc_Start(void)
{
    if (g_safetyLocked)
    {
        Prompt_Start(80);
        return;
    }

    if (!g_mpuReady)
    {
        MPU_AppInit();
    }

    /* 半圆弧结束判断依赖 yaw。未校准时不启动，先切到 MPU 页面提示。 */
    if (!g_mpuReady || !g_mpuCalibrated)
    {
        g_plotMode = 2U;
        Control_ForcePWMZero();
        Prompt_Start(900);
        return;
    }

    g_straightActive = 0;
    g_arcDone = 0;
    g_arcRunMs = 0;
    g_arcStartYaw = g_yawDeg;
    g_arcDeltaYaw = 0.0f;

    g_workMode = WORK_TRACING;
    g_plotMode = 4U;

    g_lineLostMs = 0;
    g_lineErrorFiltered = 0.0f;
    g_lineLastCtrlError = 0.0f;

    Control_ForcePWMZero();
    Encoder_DebugClearTotals();

    g_targetForwardSpeed = 0.0f;
    g_targetTurnSpeed = 0.0f;
    g_carEnable = 1;
    g_lastCmdTickMs = 0;
    g_arcActive = 1;

    PID_Reset(&ForwardPID);
    PID_Reset(&TurnPID);
    Prompt_Start(180);
}

/* 前向声明：Arc_Control10ms() 位于循迹函数定义之前，先声明避免 ARMCC 隐式声明错误。 */
static void Tracing_Control10ms(void);

static void Arc_Control10ms(void)
{
    float absDelta;

    if (!g_arcActive)
    {
        return;
    }

    if (g_arcRunMs < 60000U - CONTROL_PERIOD_MS)
    {
        g_arcRunMs += CONTROL_PERIOD_MS;
    }

    g_arcDeltaYaw = wrap_180(g_yawDeg - g_arcStartYaw);
    absDelta = absf_local(g_arcDeltaYaw);

    /* 结束条件：
     * 1. 至少运行 arcMinTime，避免刚起步误判；
     * 2. yaw 变化角度达到 arcYawTarget，认为半圆弧结束；
     * 3. arcMaxTime 是安全超时，防止一直跑。
     */
    if (((g_arcRunMs >= g_arcMinTimeMs) && (absDelta >= g_arcYawTarget)) ||
        (g_arcRunMs >= g_arcMaxTimeMs))
    {
        Arc_Finish();
        return;
    }

    Tracing_Control10ms();
    g_lastCmdTickMs = 0;
}


/* ================================================================
 * 11. H 题 task1 最小状态机
 * ================================================================ */

static void Task_Reset(void)
{
    g_taskState = TASK_IDLE;
    g_taskSelected = 0;
    g_taskRunning = 0;
    g_arcActive = 0;
}

static void Task_SelectTask1(void)
{
    if (g_safetyLocked)
    {
        Prompt_Start(80);
        return;
    }

    g_taskSelected = 1;
    g_taskRunning = 0;
    g_taskState = TASK_READY_TASK1;
    g_workMode = WORK_BT;
    g_plotMode = 3U;
    Control_ForcePWMZero();
    Prompt_Start(220);
}

static void Task_StartSelected(void)
{
    float stopThreshold;

    if (g_safetyLocked)
    {
        Prompt_Start(80);
        return;
    }

    if (g_taskState != TASK_READY_TASK1 || g_taskSelected != 1U)
    {
        Prompt_Start(120);
        return;
    }

    /* task1 复用已经验证过的 straight 直线控制。网页上保留真实距离 7030，
     * 程序内部减去停车补偿 170，实际停止点接近 100 cm。
     */
    stopThreshold = g_task1DistancePulse - g_taskStopOffsetPulse;
    if (stopThreshold < 0.0f)
    {
        stopThreshold = 0.0f;
    }

    g_straightDistancePulse = stopThreshold;
    g_taskRunning = 1;
    g_taskState = TASK_STRAIGHT_AB;
    Straight_Start();

    if (!g_straightActive)
    {
        g_taskRunning = 0;
        g_taskState = TASK_READY_TASK1;
    }
}

static void Task_Stop(void)
{
    g_straightActive = 0;
    g_arcActive = 0;
    g_taskRunning = 0;
    g_taskState = TASK_IDLE;
    g_workMode = WORK_BT;
    Control_ForcePWMZero();
    Prompt_Start(300);
}

/* ================================================================
 * 10. 模式切换和安全锁定
 * ================================================================ */

void App_EmergencyStop(void)
{
    g_safetyLocked = 1;
    g_workMode = WORK_BT;
    g_straightActive = 0;
    g_arcActive = 0;
    Task_Reset();
    Control_ForcePWMZero();
    Prompt_Start(500);
}

void App_UnlockControl(void)
{
    g_safetyLocked = 0;
    g_workMode = WORK_BT;
    g_straightActive = 0;
    g_arcActive = 0;
    Task_Reset();
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
    g_straightActive = 0;
    g_arcActive = 0;
    Task_Reset();
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

    g_straightActive = 0;
    g_arcActive = 0;
    Task_Reset();
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
 * 10. 蓝牙/网页数据包解析
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

static void ApplyFastPreset(void)
{
    ApplySpeedLimitPercent(55.0f);

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

    g_lineLostMs = 0;
    g_lineErrorFiltered = 0.0f;
    g_lineLastCtrlError = 0.0f;
    PID_Reset(&ForwardPID);
    PID_Reset(&TurnPID);
    Prompt_Start(220);
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
    if (str_is_name(name, "plotMode", "debugMode", "dbg"))
    {
        if (value < 0.5f)
        {
            g_plotMode = 0U;
        }
        else if (value < 1.5f)
        {
            g_plotMode = 1U;
        }
        else if (value < 2.5f)
        {
            g_plotMode = 2U;
        }
        else if (value < 3.5f)
        {
            g_plotMode = 3U;
        }
        else
        {
            g_plotMode = 4U;
        }
        return;
    }
    if (str_is_name(name, "yawSign", "gyroSign", "mpuSign"))
    {
        g_mpuYawSign = (value < 0.0f) ? -1.0f : 1.0f;
        return;
    }
    if (str_is_name(name, "yawKp", "headingKp", "straightKp"))
    {
        g_yawKp = limit_float(value, 0.0f, 20.0f);
        return;
    }
    if (str_is_name(name, "yawKd", "headingKd", "straightKd"))
    {
        g_yawKd = limit_float(value, 0.0f, 10.0f);
        return;
    }
    if (str_is_name(name, "straightSpeed", "straightV", "lineV"))
    {
        g_straightSpeed = limit_float(value, 0.0f, 80.0f);
        return;
    }
    if (str_is_name(name, "straightDistance", "straightPulse", "straightDist"))
    {
        g_straightDistancePulse = limit_float(value, 0.0f, 30000.0f);
        return;
    }
    if (str_is_name(name, "straightCm", "distanceCm", "distCm"))
    {
        g_straightDistancePulse = limit_float(value, 0.0f, 300.0f) * ENCODER_PULSE_PER_CM;
        return;
    }
    if (str_is_name(name, "task1Distance", "task1Pulse", "abDistance"))
    {
        g_task1DistancePulse = limit_float(value, 0.0f, 30000.0f);
        return;
    }
    if (str_is_name(name, "task1Cm", "abCm", "taskCm"))
    {
        g_task1DistancePulse = limit_float(value, 0.0f, 300.0f) * ENCODER_PULSE_PER_CM;
        return;
    }
    if (str_is_name(name, "stopOffset", "straightOffset", "brakeOffset"))
    {
        g_taskStopOffsetPulse = limit_float(value, 0.0f, 2000.0f);
        return;
    }
    if (str_is_name(name, "arcMinTime", "arcMin", "arcTime"))
    {
        g_arcMinTimeMs = (uint16_t)limit_float(value, 0.0f, 20000.0f);
        return;
    }
    if (str_is_name(name, "arcMaxTime", "arcTimeout", "arcLimitTime"))
    {
        g_arcMaxTimeMs = (uint16_t)limit_float(value, 500.0f, 60000.0f);
        return;
    }
    if (str_is_name(name, "arcYawTarget", "arcYaw", "arcAngle"))
    {
        g_arcYawTarget = limit_float(value, 30.0f, 180.0f);
        return;
    }
    if (str_is_name(name, "arcSpeed", "arcTraceSpeed", "arcV"))
    {
        g_traceBaseSpeed = limit_float(value, 0.0f, 120.0f);
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
    if (g_safetyLocked || g_straightActive || g_arcActive || g_workMode != WORK_BT)
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
            if (str_is_name(tok[1], "presetFast", "fast", "fastPreset"))
            {
                ApplyFastPreset();
                return;
            }
            if (str_is_name(tok[1], "encClear", "encoderClear", "clearEnc"))
            {
                Encoder_DebugClearTotals();
                return;
            }
            if (str_is_name(tok[1], "encDebug", "encoderDebug", "encDbg"))
            {
                g_plotMode = 1U;
                Encoder_DebugClearTotals();
                return;
            }
            if (str_is_name(tok[1], "mpuDebug", "gyroDebug", "yawDebug"))
            {
                g_plotMode = 2U;
                if (!g_mpuReady)
                {
                    MPU_AppInit();
                }
                Prompt_Start(180);
                return;
            }
            if (str_is_name(tok[1], "mpuCalib", "gyroCalib", "calib"))
            {
                g_plotMode = 2U;
                MPU_CalibrateGyroZ();
                return;
            }
            if (str_is_name(tok[1], "yawZero", "mpuZero", "zeroYaw"))
            {
                MPU_ResetYaw();
                Prompt_Start(180);
                return;
            }
            if (str_is_name(tok[1], "task1", "selectTask1", "t1"))
            {
                Task_SelectTask1();
                return;
            }
            if (str_is_name(tok[1], "start", "run", "taskStart"))
            {
                Task_StartSelected();
                return;
            }
            if (str_is_name(tok[1], "taskStop", "taskReset", "taskIdle"))
            {
                Task_Stop();
                return;
            }
            if (str_is_name(tok[1], "straight", "straightTest", "goStraight"))
            {
                Task_Reset();
                Straight_Start();
                return;
            }
            if (str_is_name(tok[1], "arcTest", "arc", "arcRun"))
            {
                Task_Reset();
                Arc_Start();
                return;
            }
            if (str_is_name(tok[1], "plotNormal", "normalPlot", "plot0"))
            {
                g_plotMode = 0U;
                Prompt_Start(160);
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
 * 11. 八路灰度循迹算法与 GPIO 直读
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
    volatile uint16_t d;

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

    for (d = 0; d < 2000; d++)
    {
        __NOP();
    }
}

static uint8_t Line_ReadOneDirect(uint8_t channel)
{
    uint8_t a;
    uint8_t b;
    uint8_t c;
    volatile uint16_t d;

    Line_SelectChannelDirect(channel);

    a = (uint8_t)GPIO_ReadInputDataBit(LINE_OUT_PORT, LINE_OUT_PIN);

    for (d = 0; d < 300; d++)
    {
        __NOP();
    }

    b = (uint8_t)GPIO_ReadInputDataBit(LINE_OUT_PORT, LINE_OUT_PIN);

    for (d = 0; d < 300; d++)
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
 * 12. 速度闭环、电机输出、显示和按键
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

    g_leftEncoderTotal += leftDelta;
    g_rightEncoderTotal += rightDelta;
    g_forwardEncoderTotal = (g_leftEncoderTotal + g_rightEncoderTotal) / 2;
    g_turnEncoderTotal = (g_rightEncoderTotal - g_leftEncoderTotal) / 2;

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

    if (g_straightActive)
    {
        Straight_Control10ms();
    }
    else if (g_arcActive)
    {
        Arc_Control10ms();
    }
    else if (g_workMode == WORK_TRACING)
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
    if (g_straightActive)
    {
        return "STR";
    }
    if (g_arcActive)
    {
        return "ARC";
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

    modeCode = g_safetyLocked ? 9 : (g_straightActive ? 3 : (g_arcActive ? 4 : ((g_taskState == TASK_READY_TASK1) ? 11 : ((g_taskState == TASK_FINISH) ? 12 : (int)g_workMode))));

    if (g_plotMode == 1U)
    {
        /*
         * 编码器调试模式下的网页绘图回传格式：
         * CH1  modeCode：0=蓝牙遥控，1=循迹，9=急停锁定
         * CH2  左轮最近 10ms 编码器增量
         * CH3  右轮最近 10ms 编码器增量
         * CH4  实际前进速度
         * CH5  实际转向速度
         * CH6  目标前进速度
         * CH7  目标转向速度
         * CH8  左轮累计脉冲
         * CH9  右轮累计脉冲
         * CH10 左右平均累计脉冲
         */
        Serial_Printf("[p,%d,%d,%d,%d,%d,%d,%d,%ld,%ld,%ld]\r\n",
                      modeCode,
                      (int)g_leftSpeed,
                      (int)g_rightSpeed,
                      (int)g_forwardSpeed,
                      (int)g_turnSpeed,
                      (int)g_targetForwardSpeed,
                      (int)g_targetTurnSpeed,
                      (long)g_leftEncoderTotal,
                      (long)g_rightEncoderTotal,
                      (long)g_forwardEncoderTotal);
        return;
    }

    if (g_plotMode == 2U)
    {
        /*
         * MPU6050 诊断/调试模式下的网页绘图回传格式：
         * CH1  modeCode：0=蓝牙遥控，1=循迹，9=急停锁定
         * CH2  yaw * 10，单位 0.1 度
         * CH3  扣零偏后的 GyroZ * 10，单位 0.1 度/秒
         * CH4  原始 GyroZ * 10，单位 0.1 度/秒
         * CH5  GyroZ 零偏 * 10，单位 0.1 度/秒
         * CH6  MPU ready，1=初始化成功
         * CH7  MPU 错误码：0=正常，1=NACK，2=ID错误，3=参数错误，0xE1=运行中读失败，0xE2=校准样本不足
         * CH8  WHO_AM_I 十进制读数：正常应为 104，也就是 0x68；255 常见于无通信；0 常见于总线被拉低
         * CH9  MPU calibrated，1=已经完成零偏校准
         * CH10 MPU 初始化尝试次数
         */
        Serial_Printf("[p,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d]\r\n",
                      modeCode,
                      (int)(g_yawDeg * 10.0f),
                      (int)(g_gyroZDps * 10.0f),
                      (int)(g_gyroZRawDps * 10.0f),
                      (int)(g_gyroZBiasDps * 10.0f),
                      (int)g_mpuReady,
                      (int)g_mpuErr,
                      (int)g_mpuWhoAmI,
                      (int)g_mpuCalibrated,
                      (int)g_mpuInitTryCount);
        return;
    }

    if (g_plotMode == 3U)
    {
        /*
         * straight 直线测试回传格式：
         * CH1  modeCode：3=straight 正在执行，9=急停
         * CH2  yaw * 10
         * CH3  yawError * 10
         * CH4  GyroZ * 10
         * CH5  目标前进速度
         * CH6  目标转向速度
         * CH7  当前平均累计脉冲
         * CH8  停止判断阈值脉冲
         * CH9  左 PWM
         * CH10 右 PWM
         */
        Serial_Printf("[p,%d,%d,%d,%d,%d,%d,%ld,%ld,%d,%d]\r\n",
                      modeCode,
                      (int)(g_yawDeg * 10.0f),
                      (int)(g_straightYawError * 10.0f),
                      (int)(g_gyroZDps * 10.0f),
                      (int)g_targetForwardSpeed,
                      (int)g_targetTurnSpeed,
                      (long)g_forwardEncoderTotal,
                      (long)g_straightDistancePulse,
                      (int)g_leftPwm,
                      (int)g_rightPwm);
        return;
    }

    if (g_plotMode == 4U)
    {
        /*
         * arcTest 半圆弧测试回传格式：
         * CH1  modeCode：4=arcTest 正在执行，9=急停
         * CH2  yaw * 10
         * CH3  deltaYaw * 10
         * CH4  arcRunMs / 10
         * CH5  lineError
         * CH6  lineMask
         * CH7  目标前进速度
         * CH8  目标转向速度
         * CH9  当前平均累计脉冲
         * CH10 lineValid
         */
        Serial_Printf("[p,%d,%d,%d,%d,%d,%d,%d,%d,%ld,%d]\r\n",
                      modeCode,
                      (int)(g_yawDeg * 10.0f),
                      (int)(g_arcDeltaYaw * 10.0f),
                      (int)(g_arcRunMs / 10U),
                      (int)g_lineError,
                      (int)g_lineMask,
                      (int)g_targetForwardSpeed,
                      (int)g_targetTurnSpeed,
                      (long)g_forwardEncoderTotal,
                      (int)g_lineValid);
        return;
    }

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
    if (g_plotMode == 1U)
    {
        OLED_Printf(0, 0, OLED_8X16, "ENC RP:%03d", (int)g_btSpeedLimitPercent);
        OLED_Printf(0, 16, OLED_8X16, "Ld:%+04d Rd:%+04d", (int)g_leftSpeed, (int)g_rightSpeed);
        OLED_Printf(0, 32, OLED_8X16, "L:%+06ld", (long)g_leftEncoderTotal);
        OLED_Printf(0, 48, OLED_8X16, "R:%+06ld A:%+06ld", (long)g_rightEncoderTotal, (long)g_forwardEncoderTotal);
        OLED_Update();
        return;
    }

    if (g_plotMode == 2U)
    {
        OLED_Printf(0, 0, OLED_8X16, "R:%d E:%02X ID:%02X", (int)g_mpuReady, (int)g_mpuErr, (int)g_mpuWhoAmI);
        OLED_Printf(0, 16, OLED_8X16, "C:%d N:%03d", (int)g_mpuCalibrated, (int)g_mpuInitTryCount);
        OLED_Printf(0, 32, OLED_8X16, "Y:%+04d.%d", (int)g_yawDeg, (int)(absf_local(g_yawDeg * 10.0f)) % 10);
        OLED_Printf(0, 48, OLED_8X16, "G:%+04d.%d B:%+03d", (int)g_gyroZDps, (int)(absf_local(g_gyroZDps * 10.0f)) % 10, (int)g_gyroZBiasDps);
        OLED_Update();
        return;
    }

    if (g_plotMode == 3U)
    {
        OLED_Printf(0, 0, OLED_8X16, "STR T:%d RP:%03d", (int)g_taskState, (int)g_btSpeedLimitPercent);
        OLED_Printf(0, 16, OLED_8X16, "D:%+05ld/%05ld", (long)g_forwardEncoderTotal, (long)g_straightDistancePulse);
        OLED_Printf(0, 32, OLED_8X16, "Y:%+03d E:%+03d", (int)g_yawDeg, (int)g_straightYawError);
        OLED_Printf(0, 48, OLED_8X16, "T:%+03d P:%+04d", (int)g_targetTurnSpeed, (int)g_forwardSpeed);
        OLED_Update();
        return;
    }

    if (g_plotMode == 4U)
    {
        OLED_Printf(0, 0, OLED_8X16, "ARC R:%04d T:%03d", (int)g_arcRunMs, (int)g_arcYawTarget);
        OLED_Printf(0, 16, OLED_8X16, "Y:%+04d D:%+04d", (int)g_yawDeg, (int)g_arcDeltaYaw);
        OLED_Printf(0, 32, OLED_8X16, "E:%+04d M:%02X", (int)g_lineError, (int)g_lineMask);
        OLED_Printf(0, 48, OLED_8X16, "F:%+03d T:%+03d", (int)g_targetForwardSpeed, (int)g_targetTurnSpeed);
        OLED_Update();
        return;
    }

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
        g_straightActive = 0;
        g_arcActive = 0;
        Task_Reset();
        Control_ForcePWMZero();
        Prompt_Start(160);
        return;
    }

    if (key == 2U)
    {
        if (g_straightActive || g_arcActive || g_taskRunning)
        {
            Task_Stop();
            return;
        }

        if (g_safetyLocked)
        {
            Prompt_Start(80);
            return;
        }

        /*
         * K2/SW2 为编码器调试模式切换键：
         *   开机默认 g_plotMode=0，即普通 BT 显示/回传模式；
         *   第一次按 K2：切换到编码器调试显示/回传模式，并清零累计脉冲；
         *   再按一次 K2：返回普通 BT 显示/回传模式。
         * 注意：编码器调试模式本质上仍保持 WORK_BT，便于用网页摇杆低速行驶做标定。
         */
        if (g_plotMode == 0U)
        {
            g_workMode = WORK_BT;
            g_plotMode = 1U;
            g_lastCmdTickMs = 0;
            Control_ForcePWMZero();
            Encoder_DebugClearTotals();
        }
        else
        {
            g_plotMode = 0U;
            App_StartBluetoothMode();
        }
        return;
    }

    if (key == 3U)
    {
        ApplyFastPreset();
        return;
    }

    if (key == 4U)
    {
        Encoder_DebugClearTotals();
        return;
    }
}

/* ================================================================
 * 13. 主函数和 1ms 定时中断
 * ================================================================ */

int main(void)
{
    OLED_Init();
    Key_Init();
    Grayscale_Init();
    Motor_Init();
    Encoder_Init();

    /* 先强制配置灰度接口，再初始化 PB1/PB5 声光提示输出。 */
    Line_GPIOForceInit();
    PromptIO_Init();

    /* OLED 已经初始化，PB8/PB9 总线可用，此时初始化 MPU6050。 */
    MPU_AppInit();

    Serial_Init();
    Timer_Init();
    Control_Init();

    ApplySpeedLimitPercent(g_btSpeedLimitPercent);
    PromptIO_AllOff();

    OLED_Printf(0, 0, OLED_8X16, "BT/Trace Car");
    OLED_Printf(0, 16, OLED_8X16, "Default: BT");
    OLED_Printf(0, 32, OLED_8X16, "RP:%d%%", (int)g_btSpeedLimitPercent);
    OLED_Printf(0, 48, OLED_8X16, "Ready");
    OLED_Update();

    while (1)
    {
        Main_BTProcess();
        Main_KeyProcess();

        if (g_mpuUpdateMs >= MPU_UPDATE_PERIOD_MS)
        {
            uint16_t elapsedMs;

            elapsedMs = g_mpuUpdateMs;
            g_mpuUpdateMs = 0;

            MPU_UpdateYaw(elapsedMs);
        }

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
        if (g_mpuUpdateMs < 60000U)
        {
            g_mpuUpdateMs++;
        }

        controlDiv++;
        if (controlDiv >= CONTROL_PERIOD_MS)
        {
            controlDiv = 0;
            Control_Run10ms();
        }
    }
}
