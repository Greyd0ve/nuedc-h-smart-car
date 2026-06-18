/*
 * 文件：User/main.c
 * 版本：h_auto_local_keys
 *
 * 适用硬件：
 *   STM32F103C8T6 小车训练板，标准外设库风格代码。
 *
 *
 * task2 当前策略：
 *   A->B / C->D：MPU yaw 航向保持 + 编码器距离判断。
 *   SEARCH_ARC：MPU yaw 目标角硬切入，同时读取八路灰度找黑线。
 *   TRACE_ARC：八路灰度循迹。
 *   出弯判断：最小弧线脉冲 + 连续丢线确认 + yaw 窗口。
 *   WAIT_ALIGN：短暂停车等待。
 *   ALIGN：MPU yaw 补角到下一段目标方向。
 *
 * task3 当前策略：
 *   A->C：MPU yaw 几何目标角 task3AcYaw + 编码器距离。
 *   C->B：入弯找线 + 灰度循迹 + yaw 窗口出弯。
 *   B 点：yaw 补角后进入 B->D。
 *   B->D：先 yaw 转向 task3BdYaw，再直线闭环。
 *   D->A：入弯找线 + 灰度循迹 + yaw 窗口出弯。
 *   A 点：yaw 补角并结束，task4 模式下则计圈进入下一圈。
 *
 * task2 状态顺序：
 *   TASK2_STRAIGHT_AB     A -> B 直线，平均脉冲到 task2SearchPulse 后提前找线
 *   TASK2_SEARCH_ARC_BC   继续直行，允许丢线，直到检测到 B->C 半圆弧黑线
 *   TASK2_TRACE_ARC_BC    灰度循迹跑 B->C 半圆弧
 *   TASK2_WAIT_ALIGN_C    C 出弯丢线后立即停车等待 0.5s
 *   TASK2_ALIGN_C         等待结束后用 MPU yaw 补角到 C->D 方向
 *   TASK2_STRAIGHT_CD     C -> D 直线，平均脉冲到 task2SearchPulse 后提前找线
 *   TASK2_SEARCH_ARC_DA   继续直行，允许丢线，直到检测到 D->A 半圆弧黑线
 *   TASK2_TRACE_ARC_DA    灰度循迹跑 D->A 半圆弧
 *   TASK2_WAIT_ALIGN_A    A 出弯丢线后立即停车等待 0.5s
 *   TASK2_ALIGN_A         等待结束后用 MPU yaw 补角到 A 点终止方向
 *   TASK2_FINISH          停车并声光提示
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
#include "../App/app_control.h"
#include "../App/app_line.h"
#include "../App/app_protocol.h"
#include "../App/app_state.h"
#include <stdint.h>

#ifndef GRAYSCALE_CHANNELS
#define GRAYSCALE_CHANNELS 8U
#endif

/* ================================================================
 * 1. 基础控制周期和安全参数
 * ================================================================ */

#define CONTROL_PERIOD_MS              10U
#define BT_TIMEOUT_MS                  600U
#define PLOT_REPORT_PERIOD_MS          500U
#define OLED_REFRESH_PERIOD_MS         200U
#define MPU_UPDATE_PERIOD_MS           10U
#define ENABLE_OLED_TEXT_DEBUG         1U

#define PWM_LIMIT_MAX                  ((float)PWM_MAX_DUTY)

/* MPU6050 静止校准参数。当前先保持原方案，后续可再加长采样。 */
#define MPU_CALIB_SAMPLES              300U
#define MPU_CALIB_MIN_OK               240U

/* 编码器距离标定结果：100 cm 约 7030 平均脉冲。 */
#define ENCODER_PULSE_PER_CM           70.30f
#define STRAIGHT_DISTANCE_DEFAULT      7030.0f
#define STRAIGHT_STOP_OFFSET_DEFAULT   170.0f

/* arcTest 单独调试用的 yaw 结束参数，task2 新架构不再主要依赖该 yaw 值。 */
#define ARC_MIN_TIME_DEFAULT_MS        1500U
#define ARC_MAX_TIME_DEFAULT_MS        6000U
#define ARC_YAW_TARGET_DEFAULT         165.0f

/* ================================================================
 * 2. 八路灰度循迹默认参数
 * ================================================================ */

volatile float g_lineBlackLevelF = 1.0f;
volatile float g_lineReverseOrderF = 0.0f;
volatile float g_lineTurnSign = 1.0f;

volatile float g_traceBaseSpeed = 60.0f;
volatile float g_traceSearchSpeed = 8.0f;

volatile float g_lineKp = 0.350f;
volatile float g_lineKd = 0.600f;
volatile float g_lineTurnLimit = 180.0f;
volatile float g_lineLostTurn = 130.0f;
volatile float g_lineFilterAlpha = 0.58f;
volatile float g_lineSlowGain = 0.88f;
volatile float g_lineEdgeTurnExtra = 82.0f;
volatile float g_lineEdgeSpeedRatio = 0.24f;
volatile float g_lineMinTurn = 34.0f;

volatile float g_forwardSlewStep = 14.0f;
volatile float g_turnSlewStep = 60.0f;

/* ================================================================
 * 3. 速度环和转向环 PID 参数
 * ================================================================ */

volatile float g_forwardKp = 14.0f;
volatile float g_forwardKi = 0.55f;
volatile float g_forwardKd = 1.8f;

volatile float g_turnKp = 10.0f;
volatile float g_turnKi = 0.04f;
volatile float g_turnKd = 1.0f;

volatile float g_maxForwardCmd = 70.0f;
volatile float g_maxTurnCmd = 75.0f;


typedef enum
{
    TASK_IDLE = 0,
    TASK_READY_TASK1,
    TASK_READY_TASK2,
    TASK_STRAIGHT_AB,
    TASK_STOP_AT_B,
    TASK_FINISH,

    /* task2 自动路径状态 */
    TASK2_STRAIGHT_AB,
    TASK2_SEARCH_ARC_BC,
    TASK2_TRACE_ARC_BC,
    TASK2_WAIT_ALIGN_C,
    TASK2_ALIGN_C,
    TASK2_STRAIGHT_CD,
    TASK2_SEARCH_ARC_DA,
    TASK2_TRACE_ARC_DA,
    TASK2_WAIT_ALIGN_A,
    TASK2_ALIGN_A,
    TASK2_FINISH,

    TASK_READY_TASK3,
    TASK3_STRAIGHT_AC,
    TASK3_WAIT_ALIGN_C,
    TASK3_ALIGN_C,
    TASK3_SEARCH_ARC_CB,
    TASK3_TRACE_ARC_CB,
    TASK3_WAIT_ALIGN_B,
    TASK3_ALIGN_B,
    TASK3_TURN_BD,
    TASK3_STRAIGHT_BD,
    TASK3_SEARCH_ARC_DA,
    TASK3_TRACE_ARC_DA,
    TASK3_WAIT_ALIGN_A,
    TASK3_ALIGN_A,
    TASK3_FINISH,

    TASK_READY_TASK4,
    TASK4_WAIT_YAW_ZERO_NEXT_LAP,
    TASK4_FINISH
} TaskState_t;

volatile WorkMode_t g_workMode = WORK_STANDBY;
volatile LocalMode_t g_localMode = LOCAL_STANDBY;

volatile float g_targetForwardSpeed = 0.0f;
volatile float g_targetTurnSpeed = 0.0f;

volatile uint8_t g_carEnable = 0;
volatile uint32_t g_lastCmdTickMs = 1000;
volatile uint8_t g_safetyLocked = 0;

volatile float g_btSpeedLimitPercent = 55.0f;
volatile float g_speedScale = 0.55f;
volatile float g_pwmLimit = (float)PWM_MAX_DUTY * 0.55f;

volatile float g_leftSpeed = 0.0f;
volatile float g_rightSpeed = 0.0f;
volatile float g_forwardSpeed = 0.0f;
volatile float g_turnSpeed = 0.0f;

volatile int32_t g_leftEncoderTotal = 0;
volatile int32_t g_rightEncoderTotal = 0;
volatile int32_t g_forwardEncoderTotal = 0;
volatile int32_t g_turnEncoderTotal = 0;

/* 0=普通回传，1=已废弃，2=MPU调试，3=直线调试，4=弧线/task调试 */
/* Plot modes: 0=legacy [p] normal, 1=deprecated, 2=MPU, 3=straight, 4=arc/task, 5=web PID [plot]. */
volatile uint8_t g_plotMode = 0;

volatile float g_speedPwm = 0.0f;
volatile float g_diffPwm = 0.0f;
volatile int16_t g_leftPwm = 0;
volatile int16_t g_rightPwm = 0;
volatile float g_forwardSpeedError = 0.0f;

/* 保留该变量，用于兼容旧版 BT_proto.c 的链接引用。 */
volatile uint8_t g_sendPlot = 0;
volatile uint16_t g_sendDisplay = 0;

volatile uint32_t g_protoPacketOkCount = 0;
volatile uint32_t g_protoErrFieldCount = 0;
volatile uint32_t g_protoErrUnknownCount = 0;
volatile uint32_t g_protoErrBadIntCount = 0;
volatile uint32_t g_protoErrBadFloatCount = 0;
volatile uint32_t g_protoErrRangeCount = 0;
volatile uint32_t g_protoErrTooLongCount = 0;
volatile uint32_t g_protoErrLockedCount = 0;

volatile uint16_t g_oledRefreshMs = 0;
volatile uint16_t g_plotReportMs = 0;
volatile uint16_t g_mpuUpdateMs = 0;

volatile int16_t g_lineError = 0;
volatile uint8_t g_lineValid = 0;
volatile uint8_t g_lineMask = 0;
volatile uint8_t g_lineRawMask = 0;
volatile int8_t g_lastLineDir = 1;
volatile uint16_t g_lineLostMs = 0;
static volatile uint16_t g_promptMs = 0;

/* MPU6050 yaw 调试状态。 */
volatile uint8_t g_mpuReady = 0;
volatile uint8_t g_mpuErr = 0;
volatile uint8_t g_mpuCalibrated = 0;
volatile uint8_t g_mpuCalibrating = 0;
volatile uint8_t g_mpuWhoAmI = 0x00U;
volatile uint8_t g_mpuInitTryCount = 0U;

volatile float g_mpuYawSign = 1.0f;
volatile float g_gyroZRawDps = 0.0f;
volatile float g_gyroZBiasDps = 0.0f;
volatile float g_gyroZDps = 0.0f;
volatile float g_gyroZScale = 1.0f;
volatile uint8_t g_gyroZKalmanEnable = 1U;
volatile float g_gyroZKalmanQ = 0.02f;
volatile float g_gyroZKalmanR = 1.5f;
volatile float g_gyroZKalmanP = 1.0f;
volatile float g_gyroZKalmanX = 0.0f;
volatile uint8_t g_staticBiasTrackEnable = 1U;
volatile float g_staticBiasAlpha = 0.999f;
volatile float g_gyroZDeadbandDps = 0.03f;
volatile float g_staticBiasGyroGateDps = 0.15f;
volatile float g_yawDeg = 0.0f;
volatile float g_yawTotalDeg = 0.0f;

/* straight 直线航向保持参数。 */
volatile uint8_t g_straightActive = 0;
volatile uint8_t g_straightDone = 0;
volatile float g_straightSpeed = 32.0f;
volatile float g_straightDistancePulse = STRAIGHT_DISTANCE_DEFAULT;
volatile float g_straightTargetYaw = 0.0f;
volatile float g_straightYawError = 0.0f;
volatile float g_yawKp = 1.8f;
volatile float g_yawKd = 0.15f;

/* arcTest 单独测试参数。 */
volatile uint8_t g_arcActive = 0;
volatile uint8_t g_arcDone = 0;
volatile uint16_t g_arcRunMs = 0;
volatile uint16_t g_arcMinTimeMs = ARC_MIN_TIME_DEFAULT_MS;
volatile uint16_t g_arcMaxTimeMs = ARC_MAX_TIME_DEFAULT_MS;
volatile float g_arcYawTarget = ARC_YAW_TARGET_DEFAULT;
volatile float g_arcStartYaw = 0.0f;
volatile float g_arcDeltaYaw = 0.0f;

/* task1/task2 通用参数。 */
volatile TaskState_t g_taskState = TASK_IDLE;
volatile uint8_t g_taskSelected = 1;
volatile uint8_t g_taskRunning = 0;
volatile uint8_t g_task4LapCount = 0U;
volatile uint8_t g_task4LapTarget = 4U;
volatile uint16_t g_task4LapYawZeroWaitMs = 200U;
volatile float g_task1DistancePulse = STRAIGHT_DISTANCE_DEFAULT;
volatile float g_task2DistancePulse = STRAIGHT_DISTANCE_DEFAULT;
volatile float g_taskStopOffsetPulse = STRAIGHT_STOP_OFFSET_DEFAULT;
volatile float g_taskStartYaw = 0.0f;
volatile float g_task1MinLinePulse = 4000.0f;
volatile uint8_t g_task1UseLineStop = 1U;

/* ================================================================
 * 5. task2/task3 自动路径参数
 * ================================================================ */

/* A->B、C->D 提前进入 SEARCH_ARC 的平均脉冲阈值。
 * 6500 pulse 约等于 92.5 cm，给相切接入半圆弧预留距离。
 */
volatile float g_task2StraightToSearchPulse = 6500.0f;

/* 兼容保留的半圆弧理论左右轮脉冲差参数。
 * 当前主线出弯后使用 MPU yaw 补角，轮差主要用于保护和调试观察。
 */
volatile float g_task2ArcWheelDiffTarget = 2800.0f;

/* 弧线阶段平均脉冲下限。只有走过足够长的弧线后，丢线才允许判定为出弯。 */
volatile float g_task2ArcMinForwardPulse = 6500.0f;

/* 进入弧线前，lineValid 连续有效的确认时间。 */
volatile uint16_t g_task2LineFoundConfirmMs = 80U;

/* 离开弧线时，lineValid 连续无效的确认时间。 */
volatile uint16_t g_task2LineLostConfirmMs = 0U;

/* SEARCH_ARC 阶段：继续按直线 yaw 低速前进找半圆弧黑线。 */
volatile float g_task2SearchSpeed = 20.0f;

/* 兼容保留参数；当前主线 ALIGN 转速使用 yawAlignTurn。 */
volatile float g_task2AlignTurnSpeed = 12.0f;

/* SEARCH_ARC 安全保护：提前找线后，如果又走了这么多脉冲还没找到线，则停车。 */
volatile float g_task2SearchMaxPulse = 1800.0f;

/* B->C 通常是右转，按当前编码器符号预计“右轮-左轮”为负。
 * D->A 通常是左转，预计“右轮-左轮”为正。
 * 如果实测补角方向反了，调 bcTurnSign/daTurnSign 即可。
 */
volatile float g_task2BcTurnSign = -1.0f;
volatile float g_task2DaTurnSign = -1.0f;

volatile uint16_t g_task2LineValidMs = 0;
volatile uint16_t g_task2LineLostMs = 0;
volatile float g_task2SearchStartPulse = 0.0f;
volatile float g_task2CurrentTurnSign = -1.0f;

/* task2 出弯后暂停参数：
 * C/A 点丢线判定出弯后，先立即停车保持 0.5s，
 * 再进入 ALIGN 补角，避免丢线瞬间速度惯性造成过度转向。
 */
volatile uint16_t g_task2AlignWaitMs = 0;
volatile uint16_t g_task2AlignWaitTargetMs = 500U;

volatile float g_task3DiagToSearchPulse = 9000.0f;
volatile float g_task3DiagTurnDiffTarget = 600.0f;
volatile float g_task3CbTurnSign = 1.0f;
volatile float g_task3BdTurnSign = 1.0f;
volatile float g_task3DaTurnSign = 1.0f;

volatile float g_entrySpeed = 12.0f;
volatile float g_entryYawKp = 1.8f;
volatile float g_entryYawKd = 0.15f;
volatile float g_entryTurnLimit = 120.0f;

volatile float g_task2BcEntryYaw = -5.0f;
volatile float g_task2DaEntryYaw = -5.0f;
volatile float g_task3CbEntryYaw = 30.0f;
volatile float g_task3DaEntryYaw = 30.0f;

volatile float g_arcEntryTargetYaw = 0.0f;
volatile uint8_t g_arcEntryTargetValid = 0U;

volatile float g_arcExitYawWindowDeg = 45.0f;

volatile float g_task2BcExitYaw = 180.0f;
volatile float g_task2DaExitYaw = 0.0f;
volatile float g_task2CAlignBiasDeg = 0.0f;
volatile float g_task2AAlignBiasDeg = 0.0f;
volatile float g_task3CbExitYaw = 150.0f;
volatile float g_task3DaExitYaw = 0.0f;

volatile float g_yawAlignToleranceDeg = 4.0f;
volatile float g_yawAlignTurnSpeed = 8.0f;
volatile uint16_t g_yawAlignMaxMs = 2500U;
volatile float g_yawAlignMaxWheelDiff = 3600.0f;

volatile float g_task3AcYawDeg = -39.0f;
volatile float g_task3CYawReset = -39.0f;
volatile float g_task3CAlignYaw = -60.0f;
volatile float g_task3CAlignBias = 0.0f;
volatile float g_task3BdYawDeg = 135.0f;

/* ================================================================
 * 6. 前向声明
 * ================================================================ */

static void Task2_SearchArcStart(float turnSign);
static uint8_t Task2_IsSpecialState(void);
static void Task2_Control10ms(void);
static void Task3_SearchArcStart(float turnSign);
static uint8_t Task3_IsSpecialState(void);
static void Task3_Control10ms(void);
static uint8_t TaskAuto_IsSpecialState(void);
static void Task_Stop(void);
static void Local_EnterStandby(void);
static void Local_CycleMode(void);
static void Local_SelectNextTask(void);
static void Local_ExecuteSelectedTask(void);

/* ================================================================
 * 7. 小工具函数
 * ================================================================ */

static float absf_local(float x)
{
    return (x >= 0.0f) ? x : -x;
}

static int32_t abs_i32_local(int32_t x)
{
    return (x >= 0) ? x : -x;
}

static float limit_float(float value, float minVal, float maxVal)
{
    if (value < minVal) return minVal;
    if (value > maxVal) return maxVal;
    return value;
}

static float slew_float(float current, float target, float maxStep)
{
    if (maxStep <= 0.0f) return target;
    if (target > current + maxStep) return current + maxStep;
    if (target < current - maxStep) return current - maxStep;
    return target;
}

static float wrap_180(float angle)
{
    while (angle > 180.0f) angle -= 360.0f;
    while (angle < -180.0f) angle += 360.0f;
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

/* ================================================================
 * 8. PB1 蜂鸣器与 PB5 外接 LED 声光提示
 * ================================================================ */

#define PROMPT_BEEP_PORT       GPIOB
#define PROMPT_BEEP_PIN        GPIO_Pin_1
#define PROMPT_LED_PORT        GPIOB
#define PROMPT_LED_PIN         GPIO_Pin_5

#define PROMPT_BEEP_ENABLE      0

static void PromptIO_BeepOn(void)
{
#if PROMPT_BEEP_ENABLE
    GPIO_ResetBits(PROMPT_BEEP_PORT, PROMPT_BEEP_PIN);
#else
    GPIO_SetBits(PROMPT_BEEP_PORT, PROMPT_BEEP_PIN);
#endif
}

static void PromptIO_BeepOff(void)
{
    GPIO_SetBits(PROMPT_BEEP_PORT, PROMPT_BEEP_PIN);
}
static void PromptIO_LedOn(void)   { GPIO_SetBits(PROMPT_LED_PORT, PROMPT_LED_PIN); }
static void PromptIO_LedOff(void)  { GPIO_ResetBits(PROMPT_LED_PORT, PROMPT_LED_PIN); }

static void PromptIO_BeepTurn(void)
{
#if PROMPT_BEEP_ENABLE
    if (GPIO_ReadOutputDataBit(PROMPT_BEEP_PORT, PROMPT_BEEP_PIN)) PromptIO_BeepOn();
    else PromptIO_BeepOff();
#else
    PromptIO_BeepOff();
#endif
}

static void PromptIO_LedTurn(void)
{
    if (GPIO_ReadOutputDataBit(PROMPT_LED_PORT, PROMPT_LED_PIN)) PromptIO_LedOff();
    else PromptIO_LedOn();
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
 * 9. PID、停车控制、编码器清零
 * ================================================================ */

static void Control_ForcePWMZero(void)
{
    App_Control_ForcePWMZero();
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
        if ((g_promptMs % 80U) == 0U) PromptIO_AllTurn();
        if (g_promptMs == 0U) PromptIO_AllOff();
    }
}

static void Encoder_ClearTotalsOnly(void)
{
    __disable_irq();
    g_leftEncoderTotal = 0;
    g_rightEncoderTotal = 0;
    g_forwardEncoderTotal = 0;
    g_turnEncoderTotal = 0;
    Encoder_ClearAll();
    __enable_irq();
}

static int32_t Task2_GetWheelDiffPulse(void)
{
    return g_rightEncoderTotal - g_leftEncoderTotal;
}

static float Nav_YawError(float targetYaw)
{
    return wrap_180(targetYaw - g_yawDeg);
}

static uint8_t Nav_YawInWindow(float targetYaw, float windowDeg)
{
    return (absf_local(Nav_YawError(targetYaw)) <= windowDeg) ? 1U : 0U;
}

static void Task_ApplyYawEntryHold(float targetYaw, float speed)
{
    float yawErr;
    float turnCmd;

    yawErr = Nav_YawError(targetYaw);
    turnCmd = g_entryYawKp * yawErr - g_entryYawKd * g_gyroZDps;
    turnCmd = limit_float(turnCmd, -g_entryTurnLimit, g_entryTurnLimit);

    /*
     * If the real car turns opposite to this yaw command, keep motor wiring/macros
     * unchanged and invert entry yaw signs or this command direction as one policy.
     */
    g_arcDeltaYaw = yawErr;
    g_targetForwardSpeed = slew_float(g_targetForwardSpeed, speed, g_forwardSlewStep);
    g_targetTurnSpeed = slew_float(g_targetTurnSpeed, turnCmd, g_turnSlewStep);
    g_carEnable = 1;
    g_lastCmdTickMs = 0;
}

static uint8_t Nav_YawAlignControl10ms(float targetYaw)
{
    float err;
    float absErr;
    int32_t wheelDiffAbs;

    if (g_arcRunMs < 60000U - CONTROL_PERIOD_MS)
    {
        g_arcRunMs += CONTROL_PERIOD_MS;
    }

    err = Nav_YawError(targetYaw);
    absErr = absf_local(err);
    wheelDiffAbs = abs_i32_local(Task2_GetWheelDiffPulse());

    g_arcDeltaYaw = err;

    if (absErr <= g_yawAlignToleranceDeg)
    {
        return 1U;
    }

    if (g_arcRunMs >= g_yawAlignMaxMs)
    {
        return 1U;
    }

    if ((float)wheelDiffAbs >= g_yawAlignMaxWheelDiff)
    {
        return 1U;
    }

    g_targetForwardSpeed = slew_float(g_targetForwardSpeed, 0.0f, g_forwardSlewStep);
    g_targetTurnSpeed = slew_float(
        g_targetTurnSpeed,
        (err > 0.0f) ? g_yawAlignTurnSpeed : -g_yawAlignTurnSpeed,
        g_turnSlewStep
    );

    g_carEnable = 1;
    g_lastCmdTickMs = 0;

    return 0U;
}

/* ================================================================
 * 10. MPU6050 yaw 读取、积分和零偏校准
 * ================================================================ */

static void MPU_ResetYaw(void)
{
    g_yawDeg = 0.0f;
    g_yawTotalDeg = 0.0f;
}

static void MPU_ForceYaw(float yaw)
{
    yaw = wrap_180(yaw);

    g_yawDeg = yaw;
    g_yawTotalDeg = yaw;
    g_gyroZDps = 0.0f;
    g_gyroZKalmanX = g_gyroZRawDps;
    g_gyroZKalmanP = 1.0f;
}

static float Kalman1D_GyroZUpdate(float z)
{
    float k;

    if (g_gyroZKalmanQ < 0.0f) g_gyroZKalmanQ = 0.0f;
    if (g_gyroZKalmanR < 0.0001f) g_gyroZKalmanR = 0.0001f;
    if (g_gyroZKalmanP < 0.0001f) g_gyroZKalmanP = 0.0001f;

    g_gyroZKalmanP += g_gyroZKalmanQ;
    if (g_gyroZKalmanP < 0.0001f) g_gyroZKalmanP = 0.0001f;

    k = g_gyroZKalmanP / (g_gyroZKalmanP + g_gyroZKalmanR);
    g_gyroZKalmanX = g_gyroZKalmanX + k * (z - g_gyroZKalmanX);
    g_gyroZKalmanP = (1.0f - k) * g_gyroZKalmanP;

    return g_gyroZKalmanX;
}

static uint8_t MPU_IsCarStaticForBiasUpdate(void)
{
    if (g_taskRunning) return 0U;
    if (g_straightActive) return 0U;
    if (g_arcActive) return 0U;
    if (TaskAuto_IsSpecialState()) return 0U;

    if (absf_local(g_targetForwardSpeed) > 0.5f) return 0U;
    if (absf_local(g_targetTurnSpeed) > 0.5f) return 0U;
    if (absf_local(g_forwardSpeed) > 0.5f) return 0U;
    if (absf_local(g_turnSpeed) > 0.5f) return 0U;
    if (absf_local(g_gyroZRawDps - g_gyroZBiasDps) > g_staticBiasGyroGateDps) return 0U;

    return 1U;
}



static void MPU_AppInit(void)
{
    if (g_mpuInitTryCount < 255U) g_mpuInitTryCount++;

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
        g_gyroZKalmanX = 0.0f;
        g_gyroZKalmanP = 1.0f;
        MPU_ResetYaw();
    }
    else
    {
        g_mpuReady = 0;
        g_mpuCalibrated = 0;
    }
}

static void MPU_ManualYawZero(void)
{
    if (!g_mpuReady)
    {
        MPU_AppInit();
    }

    if (!g_mpuReady)
    {
        Prompt_Start(900);
        return;
    }

    g_gyroZDps = 0.0f;
    g_yawDeg = 0.0f;
    g_yawTotalDeg = 0.0f;
    g_gyroZKalmanX = g_gyroZRawDps;
    g_gyroZKalmanP = 1.0f;

    /* 手动确认：认为当前 yaw 已经可用于直线航向保持 */
    g_mpuCalibrated = 1;
    g_mpuErr = MPU6050_OK;

    Prompt_Start(180);
}

static void MPU_UpdateYaw(uint16_t elapsedMs)
{
    MPU6050_Data_t data;
    float dt;
    float gyroRaw;
    float gyroFiltered;
    float gyroCorrected;

    if (!g_mpuReady || g_mpuCalibrating) return;
    if (elapsedMs == 0U) return;
    if (elapsedMs > 50U) elapsedMs = 50U;

    if (MPU6050_ReadData(&data) != MPU6050_OK)
    {
        g_mpuReady = 0;
        g_mpuErr = 0xE1U;
        return;
    }

    dt = (float)elapsedMs * 0.001f;
    gyroRaw = data.GyroZ_dps;
    g_gyroZRawDps = gyroRaw;

    if (g_gyroZKalmanEnable)
    {
        gyroFiltered = Kalman1D_GyroZUpdate(gyroRaw);
    }
    else
    {
        gyroFiltered = gyroRaw;
    }

    if (g_staticBiasTrackEnable && MPU_IsCarStaticForBiasUpdate())
    {
        g_gyroZBiasDps =
            g_staticBiasAlpha * g_gyroZBiasDps +
            (1.0f - g_staticBiasAlpha) * gyroFiltered;
    }

    gyroCorrected =
        (gyroFiltered - g_gyroZBiasDps) *
        g_gyroZScale *
        g_mpuYawSign;

    if (absf_local(gyroCorrected) < g_gyroZDeadbandDps)
    {
        gyroCorrected = 0.0f;
    }

    g_gyroZDps = gyroCorrected;

    g_yawTotalDeg += g_gyroZDps * dt;
    g_yawDeg = wrap_180(g_yawDeg + g_gyroZDps * dt);
}

static void MPU_CalibrateGyroZ(void)
{
    MPU6050_Data_t data;
    uint16_t i;
    uint16_t okCount;
    float sum;

    g_workMode = WORK_BT;
    Control_ForcePWMZero();
    g_taskRunning = 0;
    g_taskState = TASK_IDLE;

    g_mpuCalibrating = 1;
    g_mpuCalibrated = 0;
    Prompt_Start(600);

    if (!g_mpuReady) MPU_AppInit();
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
        g_gyroZKalmanX = g_gyroZBiasDps;
        g_gyroZKalmanP = 1.0f;
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
 * 11. straight 直线航向保持
 * ================================================================ */

static void Straight_Start(void)
{
    if (g_safetyLocked)
    {
        Prompt_Start(80);
        return;
    }

    if (!g_mpuReady) MPU_AppInit();

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
    Encoder_ClearTotalsOnly();

    g_straightTargetYaw = g_yawDeg;
    g_straightYawError = 0.0f;
    g_targetForwardSpeed = 0.0f;
    g_targetTurnSpeed = 0.0f;
    g_carEnable = 1;
    g_straightActive = 1;
    g_lastCmdTickMs = 0;

    App_Control_ResetPID();
    Prompt_Start(120);
}

static void Straight_StartToYaw(float targetYaw, float distancePulse)
{
    g_straightDistancePulse = distancePulse;
    Straight_Start();

    if (g_straightActive)
    {
        g_straightTargetYaw = wrap_180(targetYaw);
        g_straightYawError = 0.0f;
    }
}

static float Task_GetStraightStopThreshold(float realDistancePulse)
{
    float stopThreshold;

    stopThreshold = realDistancePulse - g_taskStopOffsetPulse;
    if (stopThreshold < 0.0f)
    {
        stopThreshold = 0.0f;
    }

    return stopThreshold;
}

static void Task2_ApplyYawStraightHold(float speed)
{
    float turnCmd;

    g_straightYawError = wrap_180(g_straightTargetYaw - g_yawDeg);
    turnCmd = g_yawKp * g_straightYawError - g_yawKd * g_gyroZDps;
    turnCmd = limit_float(turnCmd, -g_maxTurnCmd, g_maxTurnCmd);

    g_targetForwardSpeed = slew_float(g_targetForwardSpeed, speed, g_forwardSlewStep);
    g_targetTurnSpeed = slew_float(g_targetTurnSpeed, turnCmd, g_turnSlewStep);
    g_carEnable = 1;
    g_lastCmdTickMs = 0;
}

static void Straight_Finish(void)
{
    g_straightActive = 0;
    g_straightDone = 1;

    if (g_taskState == TASK_STRAIGHT_AB)
    {
        Control_ForcePWMZero();
        g_taskRunning = 0;
        g_taskState = TASK_FINISH;
        g_plotMode = 3U;
        Prompt_Start(800);
        return;
    }

    if (g_taskState == TASK2_STRAIGHT_AB)
    {
        /* A->B 提前到 6500 pulse 后，不停车，进入 B->C 找线阶段。 */
        g_taskState = TASK2_SEARCH_ARC_BC;
        Task2_SearchArcStart(g_task2BcTurnSign);
        Prompt_Start(120);
        return;
    }

    if (g_taskState == TASK2_STRAIGHT_CD)
    {
        /* C->D 提前到 6500 pulse 后，不停车，进入 D->A 找线阶段。 */
        g_taskState = TASK2_SEARCH_ARC_DA;
        Task2_SearchArcStart(g_task2DaTurnSign);
        Prompt_Start(120);
        return;
    }

    if (g_taskState == TASK3_STRAIGHT_AC)
    {
        Control_ForcePWMZero();
        MPU_ForceYaw(g_task3CYawReset);
        g_task2AlignWaitMs = 0;
        g_arcRunMs = 0;
        g_arcStartYaw = g_yawDeg;
        g_arcDeltaYaw = 0.0f;
        g_arcEntryTargetValid = 0U;
        g_taskState = TASK3_WAIT_ALIGN_C;
        g_workMode = WORK_BT;
        g_plotMode = 4U;
        Prompt_Start(180);
        return;
    }

    if (g_taskState == TASK3_STRAIGHT_BD)
    {
        g_taskState = TASK3_SEARCH_ARC_DA;
        Task3_SearchArcStart(g_task3DaTurnSign);
        Prompt_Start(120);
        return;
    }

    Control_ForcePWMZero();
    g_plotMode = 3U;
    Prompt_Start(500);
}

static void Straight_Control10ms(void)
{
    float forwardAbs;

    if (!g_straightActive) return;

    forwardAbs = absf_local((float)g_forwardEncoderTotal);

    if ((g_taskState == TASK_STRAIGHT_AB) && g_task1UseLineStop)
    {
        App_Line_Update();
        if (g_lineValid && (forwardAbs >= g_task1MinLinePulse))
        {
            Straight_Finish();
            return;
        }
    }

    if (forwardAbs >= limit_float(g_straightDistancePulse, 0.0f, 30000.0f))
    {
        Straight_Finish();
        return;
    }

    Task2_ApplyYawStraightHold(g_straightSpeed);
}

/* ================================================================
 * 12. 八路灰度循迹算法与 GPIO 直读
 * ================================================================ */

/* ================================================================
 * 13. arcTest 独立测试：保留旧 yaw 结束判断
 * ================================================================ */

#if ENABLE_LEGACY_ARCTEST
static void Arc_Start(void)
{
    if (g_safetyLocked)
    {
        Prompt_Start(80);
        return;
    }

    if (!g_mpuReady) MPU_AppInit();
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

    App_Line_ResetState();

    Control_ForcePWMZero();
    Encoder_ClearTotalsOnly();

    g_targetForwardSpeed = 0.0f;
    g_targetTurnSpeed = 0.0f;
    g_carEnable = 1;
    g_lastCmdTickMs = 0;
    g_arcActive = 1;

    App_Control_ResetPID();
    Prompt_Start(180);
}

static void Arc_Finish(void)
{
    g_arcActive = 0;
    g_arcDone = 1;
    g_workMode = WORK_BT;
    Control_ForcePWMZero();
    g_plotMode = 4U;
    Prompt_Start(700);
}

static void Arc_Control10ms(void)
{
    float absDelta;

    if (!g_arcActive) return;

    if (g_arcRunMs < 60000U - CONTROL_PERIOD_MS) g_arcRunMs += CONTROL_PERIOD_MS;

    g_arcDeltaYaw = wrap_180(g_yawDeg - g_arcStartYaw);
    absDelta = absf_local(g_arcDeltaYaw);

    if (((g_arcRunMs >= g_arcMinTimeMs) && (absDelta >= g_arcYawTarget)) ||
        (g_arcRunMs >= g_arcMaxTimeMs))
    {
        Arc_Finish();
        return;
    }

    App_Line_TracingControl10ms();
}
#endif

/* ================================================================
 * 14. task2/task3 自动路径状态机
 * ================================================================ */

static uint8_t Task2_IsSearchState(void)
{
    return (g_taskState == TASK2_SEARCH_ARC_BC || g_taskState == TASK2_SEARCH_ARC_DA);
}

static uint8_t Task2_IsTraceState(void)
{
    return (g_taskState == TASK2_TRACE_ARC_BC || g_taskState == TASK2_TRACE_ARC_DA);
}

static uint8_t Task2_IsWaitAlignState(void)
{
    return (g_taskState == TASK2_WAIT_ALIGN_C || g_taskState == TASK2_WAIT_ALIGN_A);
}

static uint8_t Task2_IsAlignState(void)
{
    return (g_taskState == TASK2_ALIGN_C || g_taskState == TASK2_ALIGN_A);
}

static uint8_t Task2_IsSpecialState(void)
{
    return (uint8_t)(Task2_IsSearchState() || Task2_IsTraceState() ||
                     Task2_IsWaitAlignState() || Task2_IsAlignState());
}

static void Task2_SearchArcStart(float turnSign)
{
    g_straightActive = 0;
    g_arcActive = 0;
    g_workMode = WORK_BT;
    g_plotMode = 4U;

    g_task2CurrentTurnSign = (turnSign < 0.0f) ? -1.0f : 1.0f;
    g_task2LineValidMs = 0;
    g_task2LineLostMs = 0;
    g_task2AlignWaitMs = 0;
    g_arcRunMs = 0;
    g_arcStartYaw = g_yawDeg;
    g_arcDeltaYaw = 0.0f;
    g_arcEntryTargetValid = 0U;

    if (g_taskState == TASK2_SEARCH_ARC_BC)
    {
        g_arcEntryTargetYaw = wrap_180(g_yawDeg + g_task2BcEntryYaw);
        g_arcEntryTargetValid = 1U;
    }
    else if (g_taskState == TASK2_SEARCH_ARC_DA)
    {
        g_arcEntryTargetYaw = wrap_180(g_yawDeg + g_task2DaEntryYaw);
        g_arcEntryTargetValid = 1U;
    }

    g_task2SearchStartPulse = (float)g_forwardEncoderTotal;

    App_Line_ResetState();

    g_carEnable = 1;
    g_lastCmdTickMs = 0;
}

static void Task2_StartTraceArc(void)
{
    /* 真正看到黑线后，才清零编码器，开始统计半圆弧段脉冲差。 */
    Encoder_ClearTotalsOnly();

    g_arcEntryTargetValid = 0U;
    g_arcRunMs = 0;
    g_arcStartYaw = g_yawDeg;
    g_arcDeltaYaw = 0.0f;
    g_task2LineLostMs = 0;
    g_task2LineValidMs = 0;

    App_Line_ResetState();

    if (g_taskState == TASK2_SEARCH_ARC_BC) g_taskState = TASK2_TRACE_ARC_BC;
    else if (g_taskState == TASK2_SEARCH_ARC_DA) g_taskState = TASK2_TRACE_ARC_DA;

    App_Control_ResetPID();
    Prompt_Start(120);
}

static void Task2_FinishTraceArc(void)
{
    /*
     * C/A 点出弯处理：
     * 一旦半圆弧 TRACE 阶段检测到有效出弯丢线，立即停车，
     * 先等待 0.5s，再进入 ALIGN 补角。
     * 这样可以避免出弯瞬间惯性和残余转向导致左右轮差继续过冲。
     */
    Control_ForcePWMZero();
    g_task2AlignWaitMs = 0;

    if (g_taskState == TASK2_TRACE_ARC_BC)
    {
        g_taskState = TASK2_WAIT_ALIGN_C;
        Prompt_Start(180);
        return;
    }

    if (g_taskState == TASK2_TRACE_ARC_DA)
    {
        g_taskState = TASK2_WAIT_ALIGN_A;
        Prompt_Start(180);
        return;
    }
}

static void Task2_FinishAlign(void)
{
    Control_ForcePWMZero();

    if (g_taskState == TASK2_ALIGN_C)
    {
        /* C 点补角完成后，进入 C->D 直线。Straight_Start 会清零编码器并锁当前 yaw。 */
        g_taskState = TASK2_STRAIGHT_CD;
        Straight_StartToYaw(wrap_180(g_taskStartYaw + 180.0f), g_task2StraightToSearchPulse);

        if (!g_straightActive)
        {
            g_taskRunning = 0;
            g_taskState = TASK_READY_TASK2;
        }
        Prompt_Start(160);
        return;
    }

    if (g_taskState == TASK2_ALIGN_A)
    {
        /* task2 完成一圈，到 A 点停车并长提示。 */
        g_taskRunning = 0;
        g_taskState = TASK2_FINISH;
        g_workMode = WORK_BT;
        g_plotMode = 3U;
        Control_ForcePWMZero();
        Prompt_Start(1000);
        return;
    }
}

static void Task2_ControlSearch10ms(void)
{
    float searchDeltaPulse;

    if (g_arcRunMs < 60000U - CONTROL_PERIOD_MS) g_arcRunMs += CONTROL_PERIOD_MS;
    if (g_arcEntryTargetValid)
    {
        g_arcDeltaYaw = Nav_YawError(g_arcEntryTargetYaw);
    }
    else
    {
        g_arcDeltaYaw = wrap_180(g_yawDeg - g_arcStartYaw);
    }

    /* SEARCH 阶段只读灰度，不调用普通循迹。
     * 因为刚从直线提前切入时还没接触黑线，丢线是正常状态。
     */
    App_Line_Update();

    if (g_lineValid)
    {
        if (g_task2LineValidMs < 60000U - CONTROL_PERIOD_MS)
        {
            g_task2LineValidMs += CONTROL_PERIOD_MS;
        }
    }
    else
    {
        g_task2LineValidMs = 0;
    }

    if (g_task2LineValidMs >= g_task2LineFoundConfirmMs)
    {
        Task2_StartTraceArc();
        return;
    }

    searchDeltaPulse = absf_local((float)g_forwardEncoderTotal - g_task2SearchStartPulse);
    if (searchDeltaPulse >= g_task2SearchMaxPulse)
    {
        /* 保护：提前找线后仍然很久找不到黑线，停车等待人工处理。 */
        Task_Stop();
        return;
    }

    if (g_arcEntryTargetValid)
    {
        Task_ApplyYawEntryHold(g_arcEntryTargetYaw, g_entrySpeed);
    }
    else
    {
        Task2_ApplyYawStraightHold(g_task2SearchSpeed);
    }
}

static void Task_TraceLineNoLost10ms(void)
{
    float forward;
    float turn;
    float e;
    float slowRatio;
    float minForward;

    if (g_speedScale <= 0.01f || g_pwmLimit <= 0.5f)
    {
        g_targetForwardSpeed = 0.0f;
        g_targetTurnSpeed = 0.0f;
        g_carEnable = 0;
        return;
    }

    /*
     * Used by task2/task3 arc trace states after App_Line_Update().
     * When lineValid=0, do not run the normal lostTurn search logic.
     */
    if (!g_lineValid)
    {
        g_targetForwardSpeed = 0.0f;
        g_targetTurnSpeed = 0.0f;
        g_carEnable = 1;
        return;
    }

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

    turn = App_Line_CalcTurnCmd() * g_speedScale;

    g_targetForwardSpeed = slew_float(g_targetForwardSpeed, forward, g_forwardSlewStep);
    g_targetTurnSpeed = slew_float(g_targetTurnSpeed, turn, g_turnSlewStep);
    g_carEnable = 1;
}

static void Task2_ControlTrace10ms(void)
{
    float forwardAbs;
    float targetYaw;

    if (g_arcRunMs < 60000U - CONTROL_PERIOD_MS)
    {
        g_arcRunMs += CONTROL_PERIOD_MS;
    }

    if (g_taskState == TASK2_TRACE_ARC_BC)
    {
        targetYaw = wrap_180(g_taskStartYaw + g_task2BcExitYaw);
    }
    else
    {
        targetYaw = wrap_180(g_taskStartYaw + g_task2DaExitYaw);
    }
    g_arcDeltaYaw = Nav_YawError(targetYaw);

    /*
     * task2 半圆弧 TRACE 阶段只读取一次灰度。
     * 在线时使用 task2 专用循迹控制；
     * 丢线时不允许进入普通 App_Line_TracingControl10ms() 的 lostTurn 找线逻辑。
     */
    App_Line_Update();
    forwardAbs = absf_local((float)g_forwardEncoderTotal);

    if (g_lineValid)
    {
        g_task2LineLostMs = 0;
        Task_TraceLineNoLost10ms();
        g_lastCmdTickMs = 0;
        return;
    }

    /* lineValid=0：先立即停车，避免继续转向。 */
    g_targetForwardSpeed = 0.0f;
    g_targetTurnSpeed = 0.0f;
    g_carEnable = 1;
    g_lastCmdTickMs = 0;

    if (g_task2LineLostMs < 60000U - CONTROL_PERIOD_MS)
    {
        g_task2LineLostMs += CONTROL_PERIOD_MS;
    }

    /*
     * 已经跑过足够弧线长度后丢线，认为到达 C/A 出弯点：
     * 立即切入 WAIT_ALIGN，先停车 0.5s，再开始补角。
     */
    if ((g_arcRunMs >= g_arcMinTimeMs) &&
        (forwardAbs >= g_task2ArcMinForwardPulse) &&
        (g_task2LineLostMs >= g_task2LineLostConfirmMs) &&
        Nav_YawInWindow(targetYaw, g_arcExitYawWindowDeg))
    {
        Task2_FinishTraceArc();
        return;
    }

    /* 弧线前半段异常丢线：保持停车，不找线，等待人工观察/急停。 */
}

static void Task2_ControlWaitAlign10ms(void)
{
    float targetYaw;

    /* 出弯后强制停车等待 0.5s。 */
    g_targetForwardSpeed = 0.0f;
    g_targetTurnSpeed = 0.0f;
    g_carEnable = 0;
    Motor_StopAll();
    App_Control_ResetPID();
    g_lastCmdTickMs = 0;

    if (g_task2AlignWaitMs < 60000U - CONTROL_PERIOD_MS)
    {
        g_task2AlignWaitMs += CONTROL_PERIOD_MS;
    }

    if (g_task2AlignWaitMs < g_task2AlignWaitTargetMs)
    {
        return;
    }

    g_task2AlignWaitMs = 0;

    if (g_taskState == TASK2_WAIT_ALIGN_C)
    {
        targetYaw = wrap_180(g_taskStartYaw + 180.0f);
        MPU_ForceYaw(wrap_180(targetYaw - g_task2CAlignBiasDeg));

        Encoder_ClearTotalsOnly();
        g_arcRunMs = 0;
        g_arcStartYaw = g_yawDeg;
        g_arcDeltaYaw = 0.0f;
        App_Control_ResetPID();
        g_taskState = TASK2_ALIGN_C;
        g_targetForwardSpeed = 0.0f;
        g_targetTurnSpeed = 0.0f;
        g_carEnable = 1;
        Prompt_Start(120);
        return;
    }

    if (g_taskState == TASK2_WAIT_ALIGN_A)
    {
        targetYaw = wrap_180(g_taskStartYaw + 0.0f);
        MPU_ForceYaw(wrap_180(targetYaw - g_task2AAlignBiasDeg));

        Encoder_ClearTotalsOnly();
        g_arcRunMs = 0;
        g_arcStartYaw = g_yawDeg;
        g_arcDeltaYaw = 0.0f;
        App_Control_ResetPID();
        g_taskState = TASK2_ALIGN_A;
        g_targetForwardSpeed = 0.0f;
        g_targetTurnSpeed = 0.0f;
        g_carEnable = 1;
        Prompt_Start(120);
        return;
    }
}

static void Task2_ControlAlign10ms(void)
{
    float targetYaw;

    if (g_taskState == TASK2_ALIGN_C)
    {
        targetYaw = wrap_180(g_taskStartYaw + 180.0f);
    }
    else
    {
        targetYaw = g_taskStartYaw;
    }

    if (Nav_YawAlignControl10ms(targetYaw))
    {
        Task2_FinishAlign();
        return;
    }
}

static void Task2_Control10ms(void)
{
    if (Task2_IsSearchState())
    {
        Task2_ControlSearch10ms();
        return;
    }

    if (Task2_IsTraceState())
    {
        Task2_ControlTrace10ms();
        return;
    }

    if (Task2_IsWaitAlignState())
    {
        Task2_ControlWaitAlign10ms();
        return;
    }

    if (Task2_IsAlignState())
    {
        Task2_ControlAlign10ms();
        return;
    }
}

static uint8_t Task3_IsSearchState(void)
{
    return (g_taskState == TASK3_SEARCH_ARC_CB || g_taskState == TASK3_SEARCH_ARC_DA);
}

static uint8_t Task3_IsTraceState(void)
{
    return (g_taskState == TASK3_TRACE_ARC_CB || g_taskState == TASK3_TRACE_ARC_DA);
}

static uint8_t Task3_IsWaitAlignState(void)
{
    return (g_taskState == TASK3_WAIT_ALIGN_C ||
            g_taskState == TASK3_WAIT_ALIGN_B ||
            g_taskState == TASK3_WAIT_ALIGN_A);
}

static uint8_t Task3_IsAlignState(void)
{
    return (g_taskState == TASK3_ALIGN_C ||
            g_taskState == TASK3_ALIGN_B ||
            g_taskState == TASK3_ALIGN_A);
}

static uint8_t Task3_IsTurnBdState(void)
{
    return (g_taskState == TASK3_TURN_BD);
}

static uint8_t Task3_IsSpecialState(void)
{
    return (uint8_t)(Task3_IsSearchState() || Task3_IsTraceState() ||
                     Task3_IsWaitAlignState() || Task3_IsAlignState() ||
                     Task3_IsTurnBdState() ||
                     (g_taskState == TASK4_WAIT_YAW_ZERO_NEXT_LAP));
}

static uint8_t TaskAuto_IsSpecialState(void)
{
    return (uint8_t)(Task2_IsSpecialState() || Task3_IsSpecialState());
}

static void Task3_StartLap(void)
{
    g_taskState = TASK3_STRAIGHT_AC;
    g_task2LineValidMs = 0;
    g_task2LineLostMs = 0;
    g_task2AlignWaitMs = 0;
    g_task2SearchStartPulse = 0.0f;
    g_arcRunMs = 0;
    g_arcDeltaYaw = 0.0f;
    g_arcEntryTargetValid = 0U;
    App_Line_ResetState();
    App_Control_ResetPID();

    Straight_StartToYaw(wrap_180(g_taskStartYaw + g_task3AcYawDeg), g_task3DiagToSearchPulse);
}

static void Task3_SearchArcStart(float turnSign)
{
    g_straightActive = 0;
    g_arcActive = 0;
    g_workMode = WORK_BT;
    g_plotMode = 4U;

    g_task2CurrentTurnSign = (turnSign < 0.0f) ? -1.0f : 1.0f;
    g_task2LineValidMs = 0;
    g_task2LineLostMs = 0;
    g_task2AlignWaitMs = 0;
    g_arcRunMs = 0;
    g_arcStartYaw = g_yawDeg;
    g_arcDeltaYaw = 0.0f;
    g_arcEntryTargetValid = 0U;

    if (g_taskState == TASK3_SEARCH_ARC_CB)
    {
        g_arcEntryTargetYaw = wrap_180(g_yawDeg + g_task3CbEntryYaw);
        g_arcEntryTargetValid = 1U;
    }
    else if (g_taskState == TASK3_SEARCH_ARC_DA)
    {
        g_arcEntryTargetYaw = wrap_180(g_yawDeg + g_task3DaEntryYaw);
        g_arcEntryTargetValid = 1U;
    }

    g_task2SearchStartPulse = (float)g_forwardEncoderTotal;

    App_Line_ResetState();

    g_carEnable = 1;
    g_lastCmdTickMs = 0;
}

static void Task3_StartTraceArc(void)
{
    Encoder_ClearTotalsOnly();

    g_arcEntryTargetValid = 0U;
    g_arcRunMs = 0;
    g_arcStartYaw = g_yawDeg;
    g_arcDeltaYaw = 0.0f;
    g_task2LineLostMs = 0;
    g_task2LineValidMs = 0;

    App_Line_ResetState();

    if (g_taskState == TASK3_SEARCH_ARC_CB) g_taskState = TASK3_TRACE_ARC_CB;
    else if (g_taskState == TASK3_SEARCH_ARC_DA) g_taskState = TASK3_TRACE_ARC_DA;

    App_Control_ResetPID();
    Prompt_Start(120);
}

static void Task3_FinishTraceArc(void)
{
    Control_ForcePWMZero();
    g_task2AlignWaitMs = 0;

    if (g_taskState == TASK3_TRACE_ARC_CB)
    {
        g_taskState = TASK3_WAIT_ALIGN_B;
        Prompt_Start(180);
        return;
    }

    if (g_taskState == TASK3_TRACE_ARC_DA)
    {
        g_taskState = TASK3_WAIT_ALIGN_A;
        Prompt_Start(180);
        return;
    }
}

static void Task3_StartTurnBD(void)
{
    Control_ForcePWMZero();
    Encoder_ClearTotalsOnly();
    App_Control_ResetPID();

    g_taskState = TASK3_TURN_BD;
    g_workMode = WORK_BT;
    g_plotMode = 4U;
    g_arcRunMs = 0;
    g_arcStartYaw = g_yawDeg;
    g_arcDeltaYaw = 0.0f;
    g_arcEntryTargetValid = 0U;
    g_task2LineValidMs = 0;
    g_task2LineLostMs = 0;
    g_task2AlignWaitMs = 0;

    Prompt_Start(180);
}

static void Task3_FinishTurnBD(void)
{
    Control_ForcePWMZero();
    g_taskState = TASK3_STRAIGHT_BD;
    Straight_StartToYaw(wrap_180(g_taskStartYaw + g_task3BdYawDeg), g_task3DiagToSearchPulse);

    if (!g_straightActive)
    {
        g_taskRunning = 0;
        g_taskState = (g_taskSelected == 4U) ? TASK_READY_TASK4 : TASK_READY_TASK3;
    }

    Prompt_Start(160);
}

static void Task3_FinishAlign(void)
{
    if (g_taskState == TASK3_ALIGN_C)
    {
        Control_ForcePWMZero();
        Encoder_ClearTotalsOnly();
        g_task2LineValidMs = 0;
        g_task2LineLostMs = 0;
        g_task2AlignWaitMs = 0;
        g_arcRunMs = 0;
        g_arcDeltaYaw = 0.0f;
        g_arcEntryTargetValid = 0U;
        App_Line_ResetState();
        App_Control_ResetPID();
        g_taskState = TASK3_SEARCH_ARC_CB;
        Task3_SearchArcStart(g_task3CbTurnSign);
        Prompt_Start(120);
        return;
    }

    if (g_taskState == TASK3_ALIGN_B)
    {
        Task3_StartTurnBD();
        return;
    }

    if (g_taskState == TASK3_ALIGN_A)
    {
        if (g_taskSelected == 4U)
        {
            if (g_task4LapTarget < 1U) g_task4LapTarget = 4U;
            if (g_task4LapTarget > 10U) g_task4LapTarget = 10U;
            if (g_task4LapCount < 250U) g_task4LapCount++;

            if (g_task4LapCount < g_task4LapTarget)
            {
                Control_ForcePWMZero();
                g_task2AlignWaitMs = 0;
                g_taskState = TASK4_WAIT_YAW_ZERO_NEXT_LAP;
                g_workMode = WORK_BT;
                g_plotMode = 4U;
                Prompt_Start(220);
                return;
            }

            g_taskRunning = 0;
            g_taskState = TASK4_FINISH;
            g_workMode = WORK_BT;
            g_plotMode = 3U;
            Control_ForcePWMZero();
            Prompt_Start(1200);
            return;
        }

        g_taskRunning = 0;
        g_taskState = TASK3_FINISH;
        g_workMode = WORK_BT;
        g_plotMode = 3U;
        Control_ForcePWMZero();
        Prompt_Start(1000);
        return;
    }
}

static void Task4_StartNextLapAfterYawZero(void)
{
    Control_ForcePWMZero();
    MPU_ResetYaw();
    g_taskStartYaw = 0.0f;
    Encoder_ClearTotalsOnly();
    App_Control_ResetPID();

    g_straightYawError = 0.0f;
    g_arcRunMs = 0;
    g_arcStartYaw = g_yawDeg;
    g_arcDeltaYaw = 0.0f;
    g_arcEntryTargetValid = 0U;
    g_task2LineValidMs = 0;
    g_task2LineLostMs = 0;
    g_task2AlignWaitMs = 0;
    g_task2SearchStartPulse = 0.0f;
    App_Line_ResetState();

    Task3_StartLap();
    if (!g_straightActive)
    {
        g_taskRunning = 0;
        g_taskState = TASK_READY_TASK4;
        Prompt_Start(900);
        return;
    }

    Prompt_Start(220);
}

static void Task4_ControlWaitYawZeroNextLap10ms(void)
{
    Control_ForcePWMZero();
    g_targetForwardSpeed = 0.0f;
    g_targetTurnSpeed = 0.0f;
    g_lastCmdTickMs = 0;

    if (g_task4LapYawZeroWaitMs > 2000U)
    {
        g_task4LapYawZeroWaitMs = 2000U;
    }

    if (g_task2AlignWaitMs < 60000U - CONTROL_PERIOD_MS)
    {
        g_task2AlignWaitMs += CONTROL_PERIOD_MS;
    }

    if (g_task2AlignWaitMs < g_task4LapYawZeroWaitMs)
    {
        return;
    }

    Task4_StartNextLapAfterYawZero();
}

static void Task3_ControlSearch10ms(void)
{
    float searchDeltaPulse;

    if (g_arcRunMs < 60000U - CONTROL_PERIOD_MS) g_arcRunMs += CONTROL_PERIOD_MS;
    if (g_arcEntryTargetValid)
    {
        g_arcDeltaYaw = Nav_YawError(g_arcEntryTargetYaw);
    }
    else
    {
        g_arcDeltaYaw = wrap_180(g_yawDeg - g_arcStartYaw);
    }

    App_Line_Update();

    if (g_lineValid)
    {
        if (g_task2LineValidMs < 60000U - CONTROL_PERIOD_MS)
        {
            g_task2LineValidMs += CONTROL_PERIOD_MS;
        }
    }
    else
    {
        g_task2LineValidMs = 0;
    }

    if (g_task2LineValidMs >= g_task2LineFoundConfirmMs)
    {
        Task3_StartTraceArc();
        return;
    }

    searchDeltaPulse = absf_local((float)g_forwardEncoderTotal - g_task2SearchStartPulse);
    if (searchDeltaPulse >= g_task2SearchMaxPulse)
    {
        Task_Stop();
        return;
    }

    if (g_arcEntryTargetValid)
    {
        Task_ApplyYawEntryHold(g_arcEntryTargetYaw, g_entrySpeed);
    }
    else
    {
        Task2_ApplyYawStraightHold(g_task2SearchSpeed);
    }
}

static void Task3_TraceLineNoLost10ms(void)
{
    Task_TraceLineNoLost10ms();
}

static void Task3_ControlTrace10ms(void)
{
    float forwardAbs;
    float targetYaw;

    if (g_arcRunMs < 60000U - CONTROL_PERIOD_MS)
    {
        g_arcRunMs += CONTROL_PERIOD_MS;
    }

    if (g_taskState == TASK3_TRACE_ARC_CB)
    {
        targetYaw = wrap_180(g_taskStartYaw + g_task3CbExitYaw);
    }
    else
    {
        targetYaw = wrap_180(g_taskStartYaw + g_task3DaExitYaw);
    }
    g_arcDeltaYaw = Nav_YawError(targetYaw);

    App_Line_Update();
    forwardAbs = absf_local((float)g_forwardEncoderTotal);

    if (g_lineValid)
    {
        g_task2LineLostMs = 0;
        Task3_TraceLineNoLost10ms();
        g_lastCmdTickMs = 0;
        return;
    }

    g_targetForwardSpeed = 0.0f;
    g_targetTurnSpeed = 0.0f;
    g_carEnable = 1;
    g_lastCmdTickMs = 0;

    if (g_task2LineLostMs < 60000U - CONTROL_PERIOD_MS)
    {
        g_task2LineLostMs += CONTROL_PERIOD_MS;
    }

    if ((g_arcRunMs >= g_arcMinTimeMs) &&
        (forwardAbs >= g_task2ArcMinForwardPulse) &&
        (g_task2LineLostMs >= g_task2LineLostConfirmMs) &&
        Nav_YawInWindow(targetYaw, g_arcExitYawWindowDeg))
    {
        Task3_FinishTraceArc();
        return;
    }
}

static void Task3_ControlWaitAlign10ms(void)
{
    g_targetForwardSpeed = 0.0f;
    g_targetTurnSpeed = 0.0f;
    g_carEnable = 0;
    Motor_StopAll();
    App_Control_ResetPID();
    g_lastCmdTickMs = 0;

    if (g_task2AlignWaitMs < 60000U - CONTROL_PERIOD_MS)
    {
        g_task2AlignWaitMs += CONTROL_PERIOD_MS;
    }

    if (g_task2AlignWaitMs < g_task2AlignWaitTargetMs)
    {
        return;
    }

    g_task2AlignWaitMs = 0;

    if (g_taskState == TASK3_WAIT_ALIGN_C)
    {
        Encoder_ClearTotalsOnly();
        g_arcRunMs = 0;
        g_arcStartYaw = g_yawDeg;
        g_arcDeltaYaw = 0.0f;
        App_Control_ResetPID();
        g_taskState = TASK3_ALIGN_C;
        g_targetForwardSpeed = 0.0f;
        g_targetTurnSpeed = 0.0f;
        g_carEnable = 1;
        Prompt_Start(120);
        return;
    }

    if (g_taskState == TASK3_WAIT_ALIGN_B)
    {
        Encoder_ClearTotalsOnly();
        g_arcRunMs = 0;
        g_arcStartYaw = g_yawDeg;
        g_arcDeltaYaw = 0.0f;
        App_Control_ResetPID();
        g_taskState = TASK3_ALIGN_B;
        g_targetForwardSpeed = 0.0f;
        g_targetTurnSpeed = 0.0f;
        g_carEnable = 1;
        Prompt_Start(120);
        return;
    }

    if (g_taskState == TASK3_WAIT_ALIGN_A)
    {
        Encoder_ClearTotalsOnly();
        g_arcRunMs = 0;
        g_arcStartYaw = g_yawDeg;
        g_arcDeltaYaw = 0.0f;
        App_Control_ResetPID();
        g_taskState = TASK3_ALIGN_A;
        g_targetForwardSpeed = 0.0f;
        g_targetTurnSpeed = 0.0f;
        g_carEnable = 1;
        Prompt_Start(120);
        return;
    }
}

static void Task3_ControlAlign10ms(void)
{
    float targetYaw;

    if (g_taskState == TASK3_ALIGN_C)
    {
        targetYaw = wrap_180(g_task3CAlignYaw + g_task3CAlignBias);
    }
    else if (g_taskState == TASK3_ALIGN_B)
    {
        targetYaw = wrap_180(g_taskStartYaw + g_task3CbExitYaw);
    }
    else
    {
        targetYaw = wrap_180(g_taskStartYaw + g_task3DaExitYaw);
    }

    if (Nav_YawAlignControl10ms(targetYaw))
    {
        Task3_FinishAlign();
        return;
    }
}

static void Task3_ControlTurnBD10ms(void)
{
    if (Nav_YawAlignControl10ms(wrap_180(g_taskStartYaw + g_task3BdYawDeg)))
    {
        Task3_FinishTurnBD();
        return;
    }
}

static void Task3_Control10ms(void)
{
    if (g_taskState == TASK4_WAIT_YAW_ZERO_NEXT_LAP)
    {
        Task4_ControlWaitYawZeroNextLap10ms();
        return;
    }

    if (Task3_IsSearchState())
    {
        Task3_ControlSearch10ms();
        return;
    }

    if (Task3_IsTraceState())
    {
        Task3_ControlTrace10ms();
        return;
    }

    if (Task3_IsWaitAlignState())
    {
        Task3_ControlWaitAlign10ms();
        return;
    }

    if (Task3_IsAlignState())
    {
        Task3_ControlAlign10ms();
        return;
    }

    if (Task3_IsTurnBdState())
    {
        Task3_ControlTurnBD10ms();
        return;
    }
}

static void Task_Reset(void)
{
    g_taskState = TASK_IDLE;
    /* 不清空 g_taskSelected：本地面板 K2 选中的任务需要在待机中保持。 */
    g_taskRunning = 0;
    g_task4LapCount = 0U;
    g_straightActive = 0;
    g_arcActive = 0;
    g_task2LineValidMs = 0;
    g_task2LineLostMs = 0;
    g_task2AlignWaitMs = 0;
    g_task2SearchStartPulse = 0.0f;
    g_task2CurrentTurnSign = g_task2BcTurnSign;
    g_arcRunMs = 0;
    g_arcDeltaYaw = 0.0f;
    g_arcEntryTargetValid = 0U;
}

static void LoadPreset_SetSpeedLimitPercent(float percent)
{
    float ratio;

    if (percent < 0.0f) percent = 0.0f;
    if (percent > 100.0f) percent = 100.0f;

    ratio = percent / 100.0f;
    g_btSpeedLimitPercent = percent;
    g_speedScale = ratio;
    g_pwmLimit = (float)PWM_MAX_DUTY * ratio;
}

static void LoadPreset_Task2Stable(void)
{
    LoadPreset_SetSpeedLimitPercent(65.0f);
    g_straightSpeed = 40.0f;
    g_yawKp = 1.8f;
    g_yawKd = 0.15f;

    g_task2ArcMinForwardPulse = 5800.0f;
    g_task2LineLostConfirmMs = 100U;
    g_arcExitYawWindowDeg = 80.0f;
    g_task2AlignWaitTargetMs = 300U;

    g_yawAlignToleranceDeg = 3.0f;
    g_yawAlignTurnSpeed = 8.0f;
    g_yawAlignMaxMs = 2500U;

    g_task2CAlignBiasDeg = -5.0f;
    g_task2AAlignBiasDeg = 0.0f;
}

static void LoadPreset_Task3Stable(void)
{
    LoadPreset_SetSpeedLimitPercent(75.0f);
    g_straightSpeed = 50.0f;
    g_yawKp = 1.5f;
    g_yawKd = 0.12f;

    g_task2ArcMinForwardPulse = 5800.0f;
    g_task2LineLostConfirmMs = 100U;
    g_arcExitYawWindowDeg = 80.0f;
    g_task2AlignWaitTargetMs = 300U;

    g_yawAlignToleranceDeg = 4.0f;
    g_yawAlignTurnSpeed = 8.0f;
    g_yawAlignMaxMs = 3000U;

    g_task3AcYawDeg = -39.0f;
    g_task3CYawReset = -39.0f;
    g_task3CAlignYaw = 5.0f;
    g_task3CAlignBias = 15.0f;

    g_task3DiagToSearchPulse = 8900.0f;
    g_task3CbEntryYaw = 5.0f;
    g_task3CbExitYaw = 150.0f;

    g_task3BdYawDeg = 175.0f;
    g_task3DaEntryYaw = -7.0f;
    g_task3DaExitYaw = 13.0f;
}

static void LoadPreset_Task4Stable(void)
{
    LoadPreset_SetSpeedLimitPercent(75.0f);
    g_straightSpeed = 50.0f;
    g_yawKp = 1.5f;
    g_yawKd = 0.12f;

    g_task2ArcMinForwardPulse = 5800.0f;
    g_task2LineLostConfirmMs = 100U;
    g_arcExitYawWindowDeg = 80.0f;
    g_task2AlignWaitTargetMs = 300U;

    g_yawAlignToleranceDeg = 8.0f;
    g_yawAlignTurnSpeed = 9.0f;
    g_yawAlignMaxMs = 3000U;

    g_task3AcYawDeg = -38.0f;
    g_task3CYawReset = -38.0f;
    g_task3CAlignYaw = 5.0f;
    g_task3CAlignBias = 15.0f;

    g_task3DiagToSearchPulse = 8900.0f;
    g_task3CbEntryYaw = 5.0f;
    g_task3CbExitYaw = 150.0f;

    g_task3BdYawDeg = 175.0f;
    g_task3DaEntryYaw = -14.0f;
    g_task3DaExitYaw = 7.0f;

    g_task4LapTarget = 4U;
    g_task4LapYawZeroWaitMs = 200U;
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
    g_task4LapCount = 0U;
    g_taskState = TASK_READY_TASK1;
    g_workMode = WORK_STANDBY;
    g_localMode = LOCAL_STANDBY;
    g_plotMode = 3U;
    Control_ForcePWMZero();
    Prompt_Start(220);
}

static void Task_SelectTask2(void)
{
    if (g_safetyLocked)
    {
        Prompt_Start(80);
        return;
    }

    LoadPreset_Task2Stable();

    g_taskSelected = 2;
    g_taskRunning = 0;
    g_task4LapCount = 0U;
    g_taskState = TASK_READY_TASK2;
    g_workMode = WORK_STANDBY;
    g_localMode = LOCAL_STANDBY;
    g_plotMode = 3U;
    Control_ForcePWMZero();
    Prompt_Start(260);
}

static void Task_SelectOnly(uint8_t task)
{
    if (task < 1U || task > 4U)
    {
        task = 1U;
    }

    g_taskSelected = task;
    g_taskRunning = 0;
    g_task4LapCount = 0U;

    if (task == 1U)
    {
        g_taskState = TASK_READY_TASK1;
    }
    else if (task == 2U)
    {
        LoadPreset_Task2Stable();
        g_taskState = TASK_READY_TASK2;
    }
    else if (task == 3U)
    {
        LoadPreset_Task3Stable();
        g_taskState = TASK_READY_TASK3;
    }
    else
    {
        LoadPreset_Task4Stable();
        g_taskState = TASK_READY_TASK4;
    }
}

static void Task_StartSelected(void)
{
    if (g_safetyLocked)
    {
        Prompt_Start(80);
        return;
    }

    if (g_taskSelected == 1U)
    {
        /* task1：A->B 直线 100 cm，到 B 停车提示。
         * 仍使用已验证的 7030-170=6860 pulse 停止阈值。
         */
        g_taskStartYaw = g_yawDeg;
        g_taskRunning = 1;
        g_taskState = TASK_STRAIGHT_AB;
        Straight_StartToYaw(g_taskStartYaw, Task_GetStraightStopThreshold(g_task1DistancePulse));

        if (!g_straightActive)
        {
            g_taskRunning = 0;
            g_taskState = TASK_READY_TASK1;
        }
        return;
    }

    if (g_taskSelected == 2U)
    {
        /* task2：使用编码器脉冲差架构。 */
        g_taskStartYaw = g_yawDeg;
        g_taskRunning = 1;
        g_taskState = TASK2_STRAIGHT_AB;
        Straight_StartToYaw(g_taskStartYaw, g_task2StraightToSearchPulse);

        if (!g_straightActive)
        {
            g_taskRunning = 0;
            g_taskState = TASK_READY_TASK2;
        }
        return;
    }

    if (g_taskSelected == 3U || g_taskSelected == 4U)
    {
        if (g_taskSelected == 4U)
        {
            g_task4LapCount = 0U;
            if (g_task4LapTarget < 1U) g_task4LapTarget = 4U;
            if (g_task4LapTarget > 10U) g_task4LapTarget = 10U;
        }

        g_taskStartYaw = g_yawDeg;
        g_taskRunning = 1;
        Task3_StartLap();

        if (!g_straightActive)
        {
            g_taskRunning = 0;
            g_taskState = (g_taskSelected == 4U) ? TASK_READY_TASK4 : TASK_READY_TASK3;
        }
        return;
    }

    Control_ForcePWMZero();
    g_taskRunning = 0;
    g_taskState = TASK_IDLE;
    Prompt_Start(700);
}

static void Task_Stop(void)
{
    g_straightActive = 0;
    g_arcActive = 0;
    g_arcEntryTargetValid = 0U;
    g_taskRunning = 0;
    g_task4LapCount = 0U;
    g_taskState = TASK_IDLE;
    g_workMode = WORK_STANDBY;
    g_localMode = LOCAL_STANDBY;
    g_plotMode = 0U;
    Control_ForcePWMZero();
    Prompt_Start(300);
}

/* ================================================================
 * 15. 本地按键面板逻辑
 * ================================================================ */

static void Local_EnterStandby(void)
{
    g_localMode = LOCAL_STANDBY;
    g_workMode = WORK_STANDBY;
    g_plotMode = 0U;
    Control_ForcePWMZero();
}

static void Local_EnterMpuDebug(void)
{
    g_localMode = LOCAL_MPU_DEBUG;
    g_workMode = WORK_STANDBY;
    g_plotMode = 2U;
    Control_ForcePWMZero();
    if (!g_mpuReady)
    {
        MPU_AppInit();
    }
    Prompt_Start(180);
}

static void Local_CycleMode(void)
{
    if (g_taskRunning || g_straightActive || g_arcActive || TaskAuto_IsSpecialState())
    {
        Prompt_Start(80);
        return;
    }

    if (g_localMode == LOCAL_STANDBY)
    {
        Local_EnterMpuDebug();
    }
    else
    {
        Local_EnterStandby();
        Prompt_Start(160);
    }
}

static void Local_SelectNextTask(void)
{
    if (g_taskRunning || g_straightActive || g_arcActive || TaskAuto_IsSpecialState())
    {
        Prompt_Start(80);
        return;
    }

    if (g_taskSelected < 1U || g_taskSelected >= 4U)
    {
        g_taskSelected = 1U;
    }
    else
    {
        g_taskSelected++;
    }

    Task_SelectOnly(g_taskSelected);
    g_localMode = LOCAL_STANDBY;
    g_workMode = WORK_STANDBY;
    g_plotMode = 0U;
    Control_ForcePWMZero();
    Prompt_Start((uint16_t)(120U + 70U * g_taskSelected));
}

static void Local_ExecuteSelectedTask(void)
{
    if (g_safetyLocked)
    {
        Prompt_Start(80);
        return;
    }

    if (g_taskRunning || g_straightActive || g_arcActive || TaskAuto_IsSpecialState())
    {
        Prompt_Start(80);
        return;
    }

    if (g_localMode == LOCAL_MPU_DEBUG)
    {
        MPU_ManualYawZero();
        return;
    }

    g_localMode = LOCAL_STANDBY;
    g_workMode = WORK_STANDBY;
    Task_StartSelected();
}

static uint8_t Local_IsCommandWaitState(void)
{
    return (uint8_t)(
        !g_safetyLocked &&
        !g_taskRunning &&
        !g_straightActive &&
        !g_arcActive &&
        !TaskAuto_IsSpecialState() &&
        g_workMode == WORK_STANDBY
    );
}

static void Local_Sw4MpuDebugAndCalib(void)
{
    /*
     * SW4/K4 在等待命令阶段：
     *   进入 MPU 调试、停车、等待车体稳定、恢复推荐 MPU 参数、
     *   执行 GyroZ 零偏校准、yaw 清零，并保持 staticBiasTrack 开启。
     *
     * 非等待阶段仍然保留原来的 K4 解锁/停车回待机功能。
     */
    if (!Local_IsCommandWaitState())
    {
        App_UnlockControl();
        return;
    }

    Control_ForcePWMZero();
    Motor_StopAll();
    App_Control_ResetPID();

    Local_EnterMpuDebug();
    g_plotMode = 2U;

    g_gyroZScale = 1.0f;
    g_gyroZKalmanEnable = 1U;
    g_gyroZKalmanQ = 0.02f;
    g_gyroZKalmanR = 1.5f;
    g_gyroZKalmanP = 1.0f;
    g_gyroZKalmanX = g_gyroZRawDps;
    g_gyroZDeadbandDps = 0.03f;
    g_staticBiasAlpha = 0.999f;
    g_staticBiasGyroGateDps = 0.15f;

    Main_DelayMs(800);

    g_staticBiasTrackEnable = 1U;
    MPU_CalibrateGyroZ();
    if (g_mpuCalibrated)
    {
        MPU_ResetYaw();
    }

    /*
     * 比赛/任务运行默认保持静止零偏跟踪开启。
     * 如需手动转圈标定 gyroZScale，可通过网页临时发送：
     * [slider,staticBiasTrack,0]
     */
    g_staticBiasTrackEnable = 1U;
    g_plotMode = 2U;
    g_localMode = LOCAL_MPU_DEBUG;
    g_workMode = WORK_STANDBY;

    Prompt_Start(g_mpuCalibrated ? 300U : 900U);
}

/* ================================================================
 * 15. 模式切换和安全锁定
 * ================================================================ */


void App_StateTaskReset(void)
{
    Task_Reset();
}

void App_StateForcePWMZero(void)
{
    Control_ForcePWMZero();
}

void App_StatePromptStart(uint16_t ms)
{
    Prompt_Start(ms);
}

void App_ProtocolTaskReset(void)
{
    Task_Reset();
}

void App_ProtocolForcePWMZero(void)
{
    Control_ForcePWMZero();
}

void App_ProtocolPromptStart(uint16_t ms)
{
    Prompt_Start(ms);
}

void App_ProtocolEnterMpuDebug(void)
{
    Local_EnterMpuDebug();
}

void App_ProtocolMpuCalibrateGyroZ(void)
{
    MPU_CalibrateGyroZ();
}

void App_ProtocolMpuResetYaw(void)
{
    MPU_ResetYaw();
}

void App_ProtocolSelectTask1(void)
{
    Task_SelectTask1();
}

void App_ProtocolSelectTask2(void)
{
    Task_SelectTask2();
}

void App_ProtocolSelectOnly(uint8_t task)
{
    Task_SelectOnly(task);
}

void App_ProtocolStartSelectedTask(void)
{
    Task_StartSelected();
}

void App_ProtocolTaskStop(void)
{
    Task_Stop();
}

#if ENABLE_LEGACY_ARCTEST
void App_ProtocolArcStart(void)
{
    Arc_Start();
}
#endif

uint8_t App_ProtocolTask2IsSpecialState(void)
{
    return TaskAuto_IsSpecialState();
}

/* ================================================================
 * 17. 速度闭环、电机输出、显示和按键
 * ================================================================ */

static void Control_Run10ms(void)
{
    App_Control_UpdateEncoderSpeed();

    if (g_safetyLocked)
    {
        Control_ForcePWMZero();
        return;
    }

    App_Control_UpdatePIDParam();

    if (TaskAuto_IsSpecialState())
    {
        if (Task2_IsSpecialState()) Task2_Control10ms();
        else Task3_Control10ms();
    }
    else if (g_straightActive)
    {
        Straight_Control10ms();
    }
#if ENABLE_LEGACY_ARCTEST
    else if (g_arcActive)
    {
        Arc_Control10ms();
    }
#endif
    else if (g_workMode == WORK_TRACING)
    {
        App_Line_TracingControl10ms();
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

    App_Control_ApplyMotorOutput();
}

static int ModeCode(void)
{
    if (g_safetyLocked) return 9;
    if (g_straightActive) return 3;
    if (g_arcActive) return 4;
    if (g_taskState == TASK_READY_TASK1) return 11;
    if (g_taskState == TASK_READY_TASK2) return 21;
    if (g_taskState == TASK_READY_TASK3) return 23;
    if (g_taskState == TASK_READY_TASK4) return 24;
    if (g_taskState == TASK2_SEARCH_ARC_BC) return 31;
    if (g_taskState == TASK2_TRACE_ARC_BC) return 32;
    if (g_taskState == TASK2_WAIT_ALIGN_C) return 37;
    if (g_taskState == TASK2_ALIGN_C) return 33;
    if (g_taskState == TASK2_SEARCH_ARC_DA) return 34;
    if (g_taskState == TASK2_TRACE_ARC_DA) return 35;
    if (g_taskState == TASK2_WAIT_ALIGN_A) return 38;
    if (g_taskState == TASK2_ALIGN_A) return 36;
    if (g_taskState == TASK2_FINISH) return 22;
    if (g_taskState == TASK3_WAIT_ALIGN_C) return 60;
    if (g_taskState == TASK3_ALIGN_C) return 61;
    if (g_taskState == TASK3_SEARCH_ARC_CB) return 51;
    if (g_taskState == TASK3_TRACE_ARC_CB) return 52;
    if (g_taskState == TASK3_WAIT_ALIGN_B) return 57;
    if (g_taskState == TASK3_ALIGN_B) return 53;
    if (g_taskState == TASK3_TURN_BD) return 54;
    if (g_taskState == TASK3_SEARCH_ARC_DA) return 55;
    if (g_taskState == TASK3_TRACE_ARC_DA) return 56;
    if (g_taskState == TASK3_WAIT_ALIGN_A) return 58;
    if (g_taskState == TASK3_ALIGN_A) return 59;
    if (g_taskState == TASK3_FINISH) return 63;
    if (g_taskState == TASK4_WAIT_YAW_ZERO_NEXT_LAP) return 65;
    if (g_taskState == TASK4_FINISH) return 64;
    if (g_taskState == TASK_FINISH) return 12;
    if (g_workMode == WORK_STANDBY)
    {
        if (g_localMode == LOCAL_MPU_DEBUG) return 42;
        return 40;
    }
    return (int)g_workMode;
}

static char *ModeString(void)
{
    if (g_safetyLocked) return "LOCK";
    if (g_localMode == LOCAL_MPU_DEBUG) return "MPU";
    if (g_taskState == TASK_FINISH || g_taskState == TASK2_FINISH ||
        g_taskState == TASK3_FINISH || g_taskState == TASK4_FINISH) return "DONE";
    if (g_taskRunning || g_straightActive || g_arcActive ||
        TaskAuto_IsSpecialState() || g_workMode == WORK_TRACING) return "RUN";
    return "STBY";
}


static void OLED_Show2Digit(uint8_t x, uint8_t y, uint8_t value)
{
    if (value > 99U) value = 99U;
    OLED_ShowChar(x, y, (char)('0' + value / 10U), OLED_8X16);
    OLED_ShowChar((int16_t)(x + 8U), y, (char)('0' + value % 10U), OLED_8X16);
}

static void OLED_ShowSignedYaw(uint8_t x, uint8_t y, int yaw)
{
    uint16_t absYaw;

    if (yaw > 180) yaw = 180;
    if (yaw < -180) yaw = -180;

    if (yaw < 0)
    {
        OLED_ShowChar(x, y, '-', OLED_8X16);
        absYaw = (uint16_t)(-yaw);
    }
    else
    {
        OLED_ShowChar(x, y, '+', OLED_8X16);
        absYaw = (uint16_t)yaw;
    }

    OLED_ShowChar((int16_t)(x + 8U), y, (char)('0' + (absYaw / 100U) % 10U), OLED_8X16);
    OLED_ShowChar((int16_t)(x + 16U), y, (char)('0' + (absYaw / 10U) % 10U), OLED_8X16);
    OLED_ShowChar((int16_t)(x + 24U), y, (char)('0' + absYaw % 10U), OLED_8X16);
}

static void OLED_ShowStatus(void)
{
    int state;
    uint8_t task;

    state = ModeCode();
    if (state < 0) state = 0;
    if (state > 99) state = 99;

    task = g_taskSelected;
    if (task < 1U) task = 1U;
    if (task > 9U) task = 9U;

    OLED_Clear();
    OLED_ShowString(0, 0, "MODE:", OLED_8X16);
    OLED_ShowString(40, 0, ModeString(), OLED_8X16);
    OLED_ShowString(0, 16, "TASK:", OLED_8X16);
    OLED_ShowChar(40, 16, (char)('0' + task), OLED_8X16);
    OLED_ShowString(0, 32, "STATE:", OLED_8X16);
    OLED_Show2Digit(48, 32, (uint8_t)state);
    OLED_ShowString(0, 48, "YAW:", OLED_8X16);
    OLED_ShowSignedYaw(32, 48, (int)g_yawDeg);
    OLED_Update();
}

static void Main_KeyProcess(void)
{
    uint8_t key = Key_GetNum();
    if (key == 0U) return;

    /*
     * 实体按键功能：
     *   K1：本地模式循环：待机 -> MPU 调试 -> 待机。
     *   K2：任务选择循环：task1 -> task2 -> task3 -> task4。
     *   K3：待机执行当前任务；MPU 调试清零 yaw。
     *   K4：等待命令阶段一键 MPU 调试校准；非等待阶段解锁停车回待机。
     */
    if (key == 1U)
    {
        Local_CycleMode();
        return;
    }

    if (key == 2U)
    {
        Local_SelectNextTask();
        return;
    }

    if (key == 3U)
    {
        Local_ExecuteSelectedTask();
        return;
    }

    if (key == 4U)
    {
        Local_Sw4MpuDebugAndCalib();
        return;
    }
}

/* ================================================================
 * 18. 主函数和 1ms 定时中断
 * ================================================================ */

int main(void)
{
    uint8_t mpuUpdateCount;

    OLED_Init();
    Key_Init();
    Grayscale_Init();
    Motor_Init();
    Encoder_Init();

    App_Line_GPIOForceInit();
    PromptIO_Init();

    MPU_AppInit();
    Serial_Init();
    Timer_Init();
    App_Control_Init();

    App_Protocol_ApplySpeedLimitPercent(g_btSpeedLimitPercent);
    PromptIO_AllOff();

    OLED_ShowStatus();

    while (1)
    {
        App_Protocol_Process();
        Main_KeyProcess();

        mpuUpdateCount = 0U;
        while (g_mpuUpdateMs >= MPU_UPDATE_PERIOD_MS && mpuUpdateCount < 2U)
        {
            g_mpuUpdateMs -= MPU_UPDATE_PERIOD_MS;
            MPU_UpdateYaw(MPU_UPDATE_PERIOD_MS);
            mpuUpdateCount++;
        }

        if (g_plotReportMs >= PLOT_REPORT_PERIOD_MS)
        {
            g_plotReportMs = 0;
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

        if (g_lastCmdTickMs < 60000U) g_lastCmdTickMs++;
        if (g_oledRefreshMs < 60000U) g_oledRefreshMs++;
        if (g_plotReportMs < 60000U) g_plotReportMs++;
        if (g_mpuUpdateMs < 60000U) g_mpuUpdateMs++;

        controlDiv++;
        if (controlDiv >= CONTROL_PERIOD_MS)
        {
            controlDiv = 0;
            Control_Run10ms();
        }
    }
}
