/*
 * 文件：User/main.c
 * 版本：task2_encoder_v2_local_keys
 *
 * 适用硬件：
 *   STM32F103C8T6 小车训练板，标准外设库风格代码。
 *
 * 本版改动重点：
 *   1. 修正实体按键真实 PCB 映射：K1=PB10，K2=PB11，K3=PA11，K4=PA12。
 *      Key.c 需要同步修改，否则 main.c 的按键含义会错位。
 *   2. 上电默认进入本地待机状态，不再自动进入蓝牙遥控模式；PWM 清零，小车不运动。
 *   3. 蓝牙遥控模式保留，但只允许通过网页命令 [key,Bluetooth,down] 进入。
 *   4. 实体按键作为本地操作面板：
 *      K1：待机 -> 编码器调试 -> MPU 调试 -> 待机 循环切换。
 *      K2：task1 -> task2 -> task3 -> task4 循环选择任务，只选择不启动。
 *      K3：待机状态执行当前任务；编码器调试清零脉冲；MPU 调试清零 yaw。
 *      K4：解除急停/锁定，PWM 清零，回到待机。
 *
 * task2_encoder_v2 架构保持前一版思路：
 *   A->B / C->D：MPU yaw 航向保持 + 左右轮平均脉冲判断距离。
 *   直线到 6500 pulse 后提前进入 SEARCH_ARC，继续直行找半圆弧黑线，此时丢线正常。
 *   看到黑线后进入 TRACE_ARC，使用八路灰度循迹跑半圆弧。
 *   已经见线后连续丢线，认为离开半圆弧。
 *   出弯后进入 ALIGN，用左右轮脉冲差补足到约 2850 pulse，再进入下一段直线。
 *
 * task2 状态顺序：
 *   TASK2_STRAIGHT_AB     A -> B 直线，平均脉冲到 task2SearchPulse 后提前找线
 *   TASK2_SEARCH_ARC_BC   继续直行，允许丢线，直到检测到 B->C 半圆弧黑线
 *   TASK2_TRACE_ARC_BC    灰度循迹跑 B->C 半圆弧
 *   TASK2_WAIT_ALIGN_C    C 出弯丢线后立即停车等待 0.5s
 *   TASK2_ALIGN_C         等待结束后补角，直到左右轮脉冲差达到 arcWheelDiff
 *   TASK2_STRAIGHT_CD     C -> D 直线，平均脉冲到 task2SearchPulse 后提前找线
 *   TASK2_SEARCH_ARC_DA   继续直行，允许丢线，直到检测到 D->A 半圆弧黑线
 *   TASK2_TRACE_ARC_DA    灰度循迹跑 D->A 半圆弧
 *   TASK2_WAIT_ALIGN_A    A 出弯丢线后立即停车等待 0.5s
 *   TASK2_ALIGN_A         等待结束后补角，到达 arcWheelDiff 后结束 task2
 *   TASK2_FINISH          停车并声光提示
 *
 * 网页命令保留：
 *   [key,Bluetooth,down]   进入蓝牙遥控模式
 *   [key,tracing,down]     进入普通循迹模式
 *   [key,emergency,down]   急停锁定
 *   [key,unlock,down]      解除急停
 *   [key,presetFast,down]  加载高速循迹参数
 *   [key,encDebug,down]    编码器调试回传
 *   [key,mpuDebug,down]    MPU 调试回传
 *   [key,mpuCalib,down]    GyroZ 零偏校准
 *   [key,yawZero,down]     yaw 清零
 *   [key,task1/2,down]     网页选择任务
 *   [key,start,down]       网页启动已选择任务
 *   [key,taskStop,down]    中止任务并回待机
 *
 * task2 可调参数：
 *   [slider,task2SearchPulse,6500]  直线提前找弧线阈值，单位 pulse
 *   [slider,arcWheelDiff,2850]      半圆弧左右轮脉冲差目标，单位 pulse
 *   [slider,arcMinPulse,6500]       弧线最小平均脉冲，防止中途误判丢线
 *   [slider,arcFoundMs,80]          找到黑线确认时间，单位 ms
 *   [slider,arcLostMs,0]            离开黑线确认时间，单位 ms；0 表示满足出弯条件后丢线立即补角
 *   [slider,toArcSpeed,12]          SEARCH_ARC 直行找线速度
 *   [slider,alignTurn,12]           出弯补角转向速度
 *   [slider,bcTurnSign,-1]          B->C 补角方向符号
 *   [slider,daTurnSign,1]           D->A 补角方向符号
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

#ifndef GRAYSCALE_CHANNELS
#define GRAYSCALE_CHANNELS 8U
#endif

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
    /* 本地待机：上电默认状态，不响应摇杆遥控，PWM 保持清零。 */
    WORK_STANDBY = 0,

    /* 蓝牙遥控：只由网页 [key,Bluetooth,down] 进入。 */
    WORK_BT = 1,

    /* 普通八路灰度循迹：由网页 [key,tracing,down] 进入。 */
    WORK_TRACING = 2
} WorkMode_t;

typedef enum
{
    LOCAL_STANDBY = 0,
    LOCAL_ENCODER_DEBUG = 1,
    LOCAL_MPU_DEBUG = 2
} LocalMode_t;

typedef enum
{
    TASK_IDLE = 0,
    TASK_READY_TASK1,
    TASK_READY_TASK2,
    TASK_STRAIGHT_AB,
    TASK_STOP_AT_B,
    TASK_FINISH,

    /* task2_encoder_v1 专用状态 */
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
    TASK2_FINISH
} TaskState_t;

volatile WorkMode_t g_workMode = WORK_STANDBY;
volatile LocalMode_t g_localMode = LOCAL_STANDBY;

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

volatile int32_t g_leftEncoderTotal = 0;
volatile int32_t g_rightEncoderTotal = 0;
volatile int32_t g_forwardEncoderTotal = 0;
volatile int32_t g_turnEncoderTotal = 0;

/* 0=普通回传，1=编码器调试，2=MPU调试，3=直线调试，4=弧线/task2调试 */
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
volatile uint8_t g_mpuWhoAmI = 0x00U;
volatile uint8_t g_mpuInitTryCount = 0U;

volatile float g_mpuYawSign = 1.0f;
volatile float g_gyroZRawDps = 0.0f;
volatile float g_gyroZBiasDps = 0.0f;
volatile float g_gyroZDps = 0.0f;
volatile float g_yawDeg = 0.0f;
volatile float g_yawTotalDeg = 0.0f;

/* straight 直线航向保持参数。 */
volatile uint8_t g_straightActive = 0;
volatile uint8_t g_straightDone = 0;
volatile float g_straightSpeed = 15.0f;
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
volatile float g_task1DistancePulse = STRAIGHT_DISTANCE_DEFAULT;
volatile float g_task2DistancePulse = STRAIGHT_DISTANCE_DEFAULT;
volatile float g_taskStopOffsetPulse = STRAIGHT_STOP_OFFSET_DEFAULT;

/* ================================================================
 * 5. task2_encoder_v1 新参数
 * ================================================================ */

/* A->B、C->D 提前进入 SEARCH_ARC 的平均脉冲阈值。
 * 6500 pulse 约等于 92.5 cm，给相切接入半圆弧预留距离。
 */
volatile float g_task2StraightToSearchPulse = 6500.0f;

/* 半圆弧理论左右轮脉冲差。
 * 你的轮距初值约 B=(14.7+11.1)/2=12.9 cm，pulsePerCm=70.30：
 * diff = 12.9*pi*70.30 ≈ 2850 pulse。
 */
volatile float g_task2ArcWheelDiffTarget = 2850.0f;

/* 弧线阶段平均脉冲下限。只有走过足够长的弧线后，丢线才允许判定为出弯。 */
volatile float g_task2ArcMinForwardPulse = 6500.0f;

/* 进入弧线前，lineValid 连续有效的确认时间。 */
volatile uint16_t g_task2LineFoundConfirmMs = 80U;

/* 离开弧线时，lineValid 连续无效的确认时间。 */
volatile uint16_t g_task2LineLostConfirmMs = 0U;

/* SEARCH_ARC 阶段：继续按直线 yaw 低速前进找半圆弧黑线。 */
volatile float g_task2SearchSpeed = 12.0f;

/* ALIGN 阶段：补足左右轮脉冲差时的目标转向速度。 */
volatile float g_task2AlignTurnSpeed = 12.0f;

/* SEARCH_ARC 安全保护：提前找线后，如果又走了这么多脉冲还没找到线，则停车。 */
volatile float g_task2SearchMaxPulse = 1800.0f;

/* B->C 通常是右转，按当前编码器符号预计“右轮-左轮”为负。
 * D->A 通常是左转，预计“右轮-左轮”为正。
 * 如果实测补角方向反了，调 bcTurnSign/daTurnSign 即可。
 */
volatile float g_task2BcTurnSign = -1.0f;
volatile float g_task2DaTurnSign = 1.0f;

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

/* ================================================================
 * 6. 前向声明
 * ================================================================ */

static void Line_Update(void);
static float Line_CalcTurnCmd(void);
static void Tracing_Control10ms(void);
static void Task2_SearchArcStart(float turnSign);
static uint8_t Task2_IsSpecialState(void);
static void Task2_Control10ms(void);
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

static int16_t limit_i16(int32_t value, int16_t minVal, int16_t maxVal)
{
    if (value < minVal) return minVal;
    if (value > maxVal) return maxVal;
    return (int16_t)value;
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

static char ascii_lower_char(char c)
{
    if (c >= 'A' && c <= 'Z') return (char)(c + ('a' - 'A'));
    return c;
}

static int str_equal_ignore_case(const char *a, const char *b)
{
    while (*a != '\0' && *b != '\0')
    {
        if (ascii_lower_char(*a) != ascii_lower_char(*b)) return 0;
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
    if (str_equal_ignore_case(s, a)) return 1;
    if (b && str_equal_ignore_case(s, b)) return 1;
    if (c && str_equal_ignore_case(s, c)) return 1;
    return 0;
}

/* ================================================================
 * 8. PB1 蜂鸣器与 PB5 外接 LED 声光提示
 * ================================================================ */

#define PROMPT_BEEP_PORT       GPIOB
#define PROMPT_BEEP_PIN        GPIO_Pin_1
#define PROMPT_LED_PORT        GPIOB
#define PROMPT_LED_PIN         GPIO_Pin_5

static void PromptIO_BeepOn(void)  { GPIO_ResetBits(PROMPT_BEEP_PORT, PROMPT_BEEP_PIN); }
static void PromptIO_BeepOff(void) { GPIO_SetBits(PROMPT_BEEP_PORT, PROMPT_BEEP_PIN); }
static void PromptIO_LedOn(void)   { GPIO_SetBits(PROMPT_LED_PORT, PROMPT_LED_PIN); }
static void PromptIO_LedOff(void)  { GPIO_ResetBits(PROMPT_LED_PORT, PROMPT_LED_PIN); }

static void PromptIO_BeepTurn(void)
{
    if (GPIO_ReadOutputDataBit(PROMPT_BEEP_PORT, PROMPT_BEEP_PIN)) PromptIO_BeepOn();
    else PromptIO_BeepOff();
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

static void PID_Reset(PID_TypeDef *pid)
{
    pid->Integral = 0.0f;
    pid->LastError = 0.0f;
}

static float PID_Calc(PID_TypeDef *pid, float target, float measure)
{
    float error = target - measure;
    float derivative = error - pid->LastError;
    float integralCandidate = pid->Integral + error;
    float output;

    integralCandidate = limit_float(integralCandidate, -pid->IntegralLimit, pid->IntegralLimit);
    output = pid->Kp * error + pid->Ki * integralCandidate + pid->Kd * derivative;

    if (output > pid->OutputLimit)
    {
        output = pid->OutputLimit;
        if (error < 0.0f) pid->Integral = integralCandidate;
    }
    else if (output < -pid->OutputLimit)
    {
        output = -pid->OutputLimit;
        if (error > 0.0f) pid->Integral = integralCandidate;
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
    g_arcActive = 0;
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

static void Encoder_DebugClearTotals(void)
{
    Encoder_ClearTotalsOnly();
    Prompt_Start(180);
}

static int32_t Task2_GetWheelDiffPulse(void)
{
    return g_rightEncoderTotal - g_leftEncoderTotal;
}

/* ================================================================
 * 10. MPU6050 yaw 读取、积分和零偏校准
 * ================================================================ */

static void MPU_ResetYaw(void)
{
    g_yawDeg = 0.0f;
    g_yawTotalDeg = 0.0f;
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

    PID_Reset(&ForwardPID);
    PID_Reset(&TurnPID);
    Prompt_Start(120);
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

    Control_ForcePWMZero();
    g_plotMode = 3U;
    Prompt_Start(500);
}

static void Straight_Control10ms(void)
{
    if (!g_straightActive) return;

    if ((float)g_forwardEncoderTotal >= limit_float(g_straightDistancePulse, 0.0f, 30000.0f))
    {
        Straight_Finish();
        return;
    }

    Task2_ApplyYawStraightHold(g_straightSpeed);
}

/* ================================================================
 * 12. 八路灰度循迹算法与 GPIO 直读
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

    if (channel & 0x01U) GPIO_SetBits(LINE_AD0_PORT, LINE_AD0_PIN);
    else GPIO_ResetBits(LINE_AD0_PORT, LINE_AD0_PIN);

    if (channel & 0x02U) GPIO_SetBits(LINE_AD1_PORT, LINE_AD1_PIN);
    else GPIO_ResetBits(LINE_AD1_PORT, LINE_AD1_PIN);

    if (channel & 0x04U) GPIO_SetBits(LINE_AD2_PORT, LINE_AD2_PIN);
    else GPIO_ResetBits(LINE_AD2_PORT, LINE_AD2_PIN);

    for (d = 0; d < 2000; d++) __NOP();
}

static uint8_t Line_ReadOneDirect(uint8_t channel)
{
    uint8_t a;
    uint8_t b;
    uint8_t c;
    volatile uint16_t d;

    Line_SelectChannelDirect(channel);
    a = (uint8_t)GPIO_ReadInputDataBit(LINE_OUT_PORT, LINE_OUT_PIN);
    for (d = 0; d < 300; d++) __NOP();
    b = (uint8_t)GPIO_ReadInputDataBit(LINE_OUT_PORT, LINE_OUT_PIN);
    for (d = 0; d < 300; d++) __NOP();
    c = (uint8_t)GPIO_ReadInputDataBit(LINE_OUT_PORT, LINE_OUT_PIN);

    return (uint8_t)(((uint16_t)a + (uint16_t)b + (uint16_t)c) >= 2U);
}

static void Line_ReadAllDirect(uint8_t raw[GRAYSCALE_CHANNELS])
{
    uint8_t i;
    for (i = 0; i < GRAYSCALE_CHANNELS; i++) raw[i] = Line_ReadOneDirect(i);
}

static void Line_Update(void)
{
    uint8_t raw[GRAYSCALE_CHANNELS];
    static const int16_t weight[GRAYSCALE_CHANNELS] = {-350, -250, -150, -50, 50, 150, 250, 350};
    int32_t sum = 0;
    int16_t count = 0;
    uint8_t mask = 0;
    uint8_t i;
    uint8_t blackLevel;
    uint8_t reverseOrder;

    blackLevel = (g_lineBlackLevelF <= 0.5f) ? 0U : 1U;
    reverseOrder = (g_lineReverseOrderF <= 0.5f) ? 0U : 1U;

    Line_ReadAllDirect(raw);

    g_lineRawMask = 0;
    for (i = 0; i < GRAYSCALE_CHANNELS; i++)
    {
        if (raw[i] == blackLevel) g_lineRawMask |= (uint8_t)(1U << i);
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
        if (g_lineLostMs < 60000U) g_lineLostMs += CONTROL_PERIOD_MS;
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
        if (e > 1.0f) e = 1.0f;

        slowRatio = 1.0f - limit_float(g_lineSlowGain, 0.0f, 0.95f) * e;
        if ((g_lineMask & 0xC3U) != 0U)
        {
            slowRatio *= limit_float(g_lineEdgeSpeedRatio, 0.05f, 1.0f);
        }

        forward = g_traceBaseSpeed * g_speedScale * slowRatio;
        minForward = g_traceSearchSpeed * g_speedScale;
        if (forward < minForward) forward = minForward;

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
    g_lastCmdTickMs = 0;
}

/* ================================================================
 * 13. arcTest 独立测试：保留旧 yaw 结束判断
 * ================================================================ */

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

    g_lineLostMs = 0;
    g_lineErrorFiltered = 0.0f;
    g_lineLastCtrlError = 0.0f;

    Control_ForcePWMZero();
    Encoder_ClearTotalsOnly();

    g_targetForwardSpeed = 0.0f;
    g_targetTurnSpeed = 0.0f;
    g_carEnable = 1;
    g_lastCmdTickMs = 0;
    g_arcActive = 1;

    PID_Reset(&ForwardPID);
    PID_Reset(&TurnPID);
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

    Tracing_Control10ms();
}

/* ================================================================
 * 14. task2_encoder_v1 状态机
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

    g_task2SearchStartPulse = (float)g_forwardEncoderTotal;

    g_lineLostMs = 0;
    g_lineErrorFiltered = 0.0f;
    g_lineLastCtrlError = 0.0f;

    g_carEnable = 1;
    g_lastCmdTickMs = 0;
}

static void Task2_StartTraceArc(void)
{
    /* 真正看到黑线后，才清零编码器，开始统计半圆弧段脉冲差。 */
    Encoder_ClearTotalsOnly();

    g_arcRunMs = 0;
    g_arcStartYaw = g_yawDeg;
    g_arcDeltaYaw = 0.0f;
    g_task2LineLostMs = 0;
    g_task2LineValidMs = 0;

    g_lineLostMs = 0;
    g_lineErrorFiltered = 0.0f;
    g_lineLastCtrlError = 0.0f;

    if (g_taskState == TASK2_SEARCH_ARC_BC) g_taskState = TASK2_TRACE_ARC_BC;
    else if (g_taskState == TASK2_SEARCH_ARC_DA) g_taskState = TASK2_TRACE_ARC_DA;

    PID_Reset(&ForwardPID);
    PID_Reset(&TurnPID);
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
        g_straightDistancePulse = g_task2StraightToSearchPulse;
        Straight_Start();

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
    g_arcDeltaYaw = wrap_180(g_yawDeg - g_arcStartYaw);

    /* SEARCH 阶段只读灰度，不调用普通循迹。
     * 因为刚从直线提前切入时还没接触黑线，丢线是正常状态。
     */
    Line_Update();

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

    Task2_ApplyYawStraightHold(g_task2SearchSpeed);
}

static void Task2_TraceLineNoLost10ms(void)
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
     * 本函数只用于 task2 的半圆弧 TRACE 阶段。
     * 外部已经调用过 Line_Update()。
     * 如果 lineValid=0，绝不执行普通循迹里的 lostTurn 找线逻辑，
     * 只停车等待状态机进入 WAIT_ALIGN/ALIGN。
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

    turn = Line_CalcTurnCmd() * g_speedScale;

    g_targetForwardSpeed = slew_float(g_targetForwardSpeed, forward, g_forwardSlewStep);
    g_targetTurnSpeed = slew_float(g_targetTurnSpeed, turn, g_turnSlewStep);
    g_carEnable = 1;
}

static void Task2_ControlTrace10ms(void)
{
    float forwardAbs;

    if (g_arcRunMs < 60000U - CONTROL_PERIOD_MS)
    {
        g_arcRunMs += CONTROL_PERIOD_MS;
    }
    g_arcDeltaYaw = wrap_180(g_yawDeg - g_arcStartYaw);

    /*
     * task2 半圆弧 TRACE 阶段只读取一次灰度。
     * 在线时使用 task2 专用循迹控制；
     * 丢线时不允许进入普通 Tracing_Control10ms() 的 lostTurn 找线逻辑。
     */
    Line_Update();
    forwardAbs = absf_local((float)g_forwardEncoderTotal);

    if (g_lineValid)
    {
        g_task2LineLostMs = 0;
        Task2_TraceLineNoLost10ms();
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
        (g_task2LineLostMs >= g_task2LineLostConfirmMs))
    {
        Task2_FinishTraceArc();
        return;
    }

    /* 弧线前半段异常丢线：保持停车，不找线，等待人工观察/急停。 */
}

static void Task2_ControlWaitAlign10ms(void)
{
    /* 出弯后强制停车等待 0.5s。 */
    g_targetForwardSpeed = 0.0f;
    g_targetTurnSpeed = 0.0f;
    g_carEnable = 0;
    Motor_StopAll();
    PID_Reset(&ForwardPID);
    PID_Reset(&TurnPID);
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
        g_taskState = TASK2_ALIGN_C;
        g_targetForwardSpeed = 0.0f;
        g_targetTurnSpeed = 0.0f;
        g_carEnable = 1;
        Prompt_Start(120);
        return;
    }

    if (g_taskState == TASK2_WAIT_ALIGN_A)
    {
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
    int32_t wheelDiffAbs;
    int32_t targetDiff;
    float turnTarget;

    wheelDiffAbs = abs_i32_local(Task2_GetWheelDiffPulse());
    targetDiff = (int32_t)g_task2ArcWheelDiffTarget;

    if (wheelDiffAbs >= targetDiff)
    {
        Task2_FinishAlign();
        return;
    }

    if (g_taskState == TASK2_ALIGN_C) turnTarget = g_task2BcTurnSign * g_task2AlignTurnSpeed;
    else turnTarget = g_task2DaTurnSign * g_task2AlignTurnSpeed;

    g_targetForwardSpeed = slew_float(g_targetForwardSpeed, 0.0f, g_forwardSlewStep);
    g_targetTurnSpeed = slew_float(g_targetTurnSpeed, turnTarget, g_turnSlewStep);
    g_carEnable = 1;
    g_lastCmdTickMs = 0;
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

static void Task_Reset(void)
{
    g_taskState = TASK_IDLE;
    /* 不清空 g_taskSelected：本地面板 K2 选中的任务需要在待机中保持。 */
    g_taskRunning = 0;
    g_straightActive = 0;
    g_arcActive = 0;
    g_task2LineValidMs = 0;
    g_task2LineLostMs = 0;
    g_task2AlignWaitMs = 0;
    g_task2SearchStartPulse = 0.0f;
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

    g_taskSelected = 2;
    g_taskRunning = 0;
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

    if (task == 1U)
    {
        g_taskState = TASK_READY_TASK1;
    }
    else if (task == 2U)
    {
        g_taskState = TASK_READY_TASK2;
    }
    else
    {
        /* task3/task4 暂未实现：仅保留选择状态，K3 执行时空执行提示。 */
        g_taskState = TASK_IDLE;
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
        g_straightDistancePulse = Task_GetStraightStopThreshold(g_task1DistancePulse);
        g_taskRunning = 1;
        g_taskState = TASK_STRAIGHT_AB;
        Straight_Start();

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
        g_straightDistancePulse = g_task2StraightToSearchPulse;
        g_taskRunning = 1;
        g_taskState = TASK2_STRAIGHT_AB;
        Straight_Start();

        if (!g_straightActive)
        {
            g_taskRunning = 0;
            g_taskState = TASK_READY_TASK2;
        }
        return;
    }

    /* task3/task4 暂未实现：空执行，只声光提示，不运动。 */
    Control_ForcePWMZero();
    g_taskRunning = 0;
    g_taskState = TASK_IDLE;
    Prompt_Start(700);
}

static void Task_Stop(void)
{
    g_straightActive = 0;
    g_arcActive = 0;
    g_taskRunning = 0;
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

static void Local_EnterEncoderDebug(void)
{
    g_localMode = LOCAL_ENCODER_DEBUG;
    g_workMode = WORK_STANDBY;
    g_plotMode = 1U;
    Control_ForcePWMZero();
    Encoder_DebugClearTotals();
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
    if (g_taskRunning || g_straightActive || g_arcActive || Task2_IsSpecialState())
    {
        Prompt_Start(80);
        return;
    }

    if (g_localMode == LOCAL_STANDBY)
    {
        Local_EnterEncoderDebug();
    }
    else if (g_localMode == LOCAL_ENCODER_DEBUG)
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
    if (g_taskRunning || g_straightActive || g_arcActive || Task2_IsSpecialState())
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

    if (g_taskRunning || g_straightActive || g_arcActive || Task2_IsSpecialState())
    {
        Prompt_Start(80);
        return;
    }

    if (g_localMode == LOCAL_ENCODER_DEBUG)
    {
        Encoder_DebugClearTotals();
        return;
    }

    if (g_localMode == LOCAL_MPU_DEBUG)
    {
        MPU_ResetYaw();
        Prompt_Start(180);
        return;
    }

    g_localMode = LOCAL_STANDBY;
    g_workMode = WORK_STANDBY;
    Task_StartSelected();
}

/* ================================================================
 * 15. 模式切换和安全锁定
 * ================================================================ */

void App_EmergencyStop(void)
{
    g_safetyLocked = 1;
    g_workMode = WORK_STANDBY;
    g_localMode = LOCAL_STANDBY;
    Task_Reset();
    Control_ForcePWMZero();
    Prompt_Start(500);
}

void App_UnlockControl(void)
{
    g_safetyLocked = 0;
    g_workMode = WORK_STANDBY;
    g_localMode = LOCAL_STANDBY;
    g_plotMode = 0U;
    Task_Reset();
    g_lastCmdTickMs = 0;
    Control_ForcePWMZero();
    Prompt_Start(180);
}

void App_StartBluetoothMode(void)
{
    if (g_safetyLocked) return;

    /* 蓝牙遥控模式只保留网页入口，实体按键不会进入该模式。 */
    g_workMode = WORK_BT;
    g_localMode = LOCAL_STANDBY;
    Task_Reset();
    g_lastCmdTickMs = 0;
    Control_ForcePWMZero();
    Prompt_Start(160);
}

void App_StartTracingMode(void)
{
    if (g_safetyLocked) return;

    Task_Reset();
    g_workMode = WORK_TRACING;
    g_localMode = LOCAL_STANDBY;
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
 * 16. 蓝牙/网页数据包解析
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

    if (str_is_name(name, "speedKp", "forwardKp", "fKp")) { g_forwardKp = limit_float(value, 0.0f, 80.0f); return; }
    if (str_is_name(name, "speedKi", "forwardKi", "fKi")) { g_forwardKi = limit_float(value, 0.0f, 20.0f); return; }
    if (str_is_name(name, "speedKd", "forwardKd", "fKd")) { g_forwardKd = limit_float(value, 0.0f, 30.0f); return; }
    if (str_is_name(name, "turnKp", "diffKp", "tKp")) { g_turnKp = limit_float(value, 0.0f, 80.0f); return; }
    if (str_is_name(name, "turnKi", "diffKi", "tKi")) { g_turnKi = limit_float(value, 0.0f, 20.0f); return; }
    if (str_is_name(name, "turnKd", "diffKd", "tKd")) { g_turnKd = limit_float(value, 0.0f, 30.0f); return; }
    if (str_is_name(name, "maxForward", "maxSpeed", "btSpeed")) { g_maxForwardCmd = limit_float(value, 0.0f, 200.0f); return; }
    if (str_is_name(name, "maxTurn", "btTurn", "remoteTurn")) { g_maxTurnCmd = limit_float(value, 0.0f, 200.0f); return; }

    if (str_is_name(name, "traceKp", "lineKp", "lineP")) { g_lineKp = limit_float(value, 0.0f, 1.0f); return; }
    if (str_is_name(name, "traceKd", "lineKd", "lineD")) { g_lineKd = limit_float(value, 0.0f, 1.0f); return; }
    if (str_is_name(name, "traceSpeed", "lineSpeed", "baseSpeed")) { g_traceBaseSpeed = limit_float(value, 0.0f, 120.0f); return; }
    if (str_is_name(name, "turnLimit", "lineTurnLimit", "traceTurnLimit")) { g_lineTurnLimit = limit_float(value, 0.0f, 180.0f); return; }
    if (str_is_name(name, "lostTurn", "lineLostTurn", "findTurn")) { g_lineLostTurn = limit_float(value, 0.0f, 180.0f); return; }

    if (str_is_name(name, "filter", "lineFilter", "alpha"))
    {
        if (value > 1.0f) value = value / 100.0f;
        g_lineFilterAlpha = limit_float(value, 0.0f, 1.0f);
        return;
    }
    if (str_is_name(name, "slowGain", "lineSlow", "curveSlow"))
    {
        if (value > 1.0f) value = value / 100.0f;
        g_lineSlowGain = limit_float(value, 0.0f, 0.95f);
        return;
    }
    if (str_is_name(name, "edgeBoost", "edgeTurn", "edgeExtra")) { g_lineEdgeTurnExtra = limit_float(value, 0.0f, 100.0f); return; }
    if (str_is_name(name, "edgeSlow", "edgeSpeed", "edgeRatio"))
    {
        if (value > 1.0f) value = value / 100.0f;
        g_lineEdgeSpeedRatio = limit_float(value, 0.05f, 1.0f);
        return;
    }
    if (str_is_name(name, "minTurn", "lineMinTurn", "traceMinTurn")) { g_lineMinTurn = limit_float(value, 0.0f, 80.0f); return; }
    if (str_is_name(name, "forwardSlew", "speedSlew", "fSlew")) { g_forwardSlewStep = limit_float(value, 0.0f, 30.0f); return; }
    if (str_is_name(name, "turnSlew", "diffSlew", "tSlew")) { g_turnSlewStep = limit_float(value, 0.0f, 60.0f); return; }
    if (str_is_name(name, "lineSign", "traceSign", "turnSign")) { g_lineTurnSign = (value < 0.0f) ? -1.0f : 1.0f; return; }
    if (str_is_name(name, "blackLevel", "lineLevel", "black")) { g_lineBlackLevelF = (value <= 0.0f) ? 0.0f : 1.0f; return; }
    if (str_is_name(name, "lineReverse", "reverseLine", "sensorReverse")) { g_lineReverseOrderF = (value <= 0.0f) ? 0.0f : 1.0f; return; }

    if (str_is_name(name, "yawSign", "gyroSign", "mpuSign")) { g_mpuYawSign = (value < 0.0f) ? -1.0f : 1.0f; return; }
    if (str_is_name(name, "yawKp", "headingKp", "straightKp")) { g_yawKp = limit_float(value, 0.0f, 20.0f); return; }
    if (str_is_name(name, "yawKd", "headingKd", "straightKd")) { g_yawKd = limit_float(value, 0.0f, 10.0f); return; }
    if (str_is_name(name, "straightSpeed", "straightV", "lineV")) { g_straightSpeed = limit_float(value, 0.0f, 80.0f); return; }

    if (str_is_name(name, "task2SearchPulse", "searchPulse", "toArcPulse"))
    {
        g_task2StraightToSearchPulse = limit_float(value, 1000.0f, 15000.0f);
        return;
    }
    if (str_is_name(name, "arcWheelDiff", "wheelDiff", "arcDiff"))
    {
        g_task2ArcWheelDiffTarget = limit_float(value, 500.0f, 8000.0f);
        return;
    }
    if (str_is_name(name, "arcMinPulse", "arcForwardPulse", "arcFwdPulse"))
    {
        g_task2ArcMinForwardPulse = limit_float(value, 1000.0f, 15000.0f);
        return;
    }
    if (str_is_name(name, "arcFoundMs", "lineFoundMs", "foundMs"))
    {
        g_task2LineFoundConfirmMs = (uint16_t)limit_float(value, 10.0f, 2000.0f);
        return;
    }
    if (str_is_name(name, "arcLostMs", "lineLostMs", "lostMs"))
    {
        g_task2LineLostConfirmMs = (uint16_t)limit_float(value, 0.0f, 3000.0f);
        return;
    }
    if (str_is_name(name, "alignWaitMs", "arcWaitMs", "waitAlign"))
    {
        g_task2AlignWaitTargetMs = (uint16_t)limit_float(value, 0.0f, 3000.0f);
        return;
    }
    if (str_is_name(name, "toArcSpeed", "arcSearchSpeed", "searchArcSpeed"))
    {
        g_task2SearchSpeed = limit_float(value, 0.0f, 60.0f);
        return;
    }
    if (str_is_name(name, "alignTurn", "alignTurnSpeed", "arcAlignTurn"))
    {
        g_task2AlignTurnSpeed = limit_float(value, 0.0f, 80.0f);
        return;
    }
    if (str_is_name(name, "bcTurnSign", "bcSign", "arcBcSign"))
    {
        g_task2BcTurnSign = (value < 0.0f) ? -1.0f : 1.0f;
        return;
    }
    if (str_is_name(name, "daTurnSign", "daSign", "arcDaSign"))
    {
        g_task2DaTurnSign = (value < 0.0f) ? -1.0f : 1.0f;
        return;
    }

    if (str_is_name(name, "plotMode", "debugMode", "dbg"))
    {
        if (value < 0.5f) g_plotMode = 0U;
        else if (value < 1.5f) g_plotMode = 1U;
        else if (value < 2.5f) g_plotMode = 2U;
        else if (value < 3.5f) g_plotMode = 3U;
        else g_plotMode = 4U;
        return;
    }
}

static void Main_ApplyJoystickPacket(char **tok, int n)
{
    int turnRaw;
    int forwardRaw;
    float maxForward;
    float maxTurn;

    if (n < 3) return;
    if (g_safetyLocked || g_straightActive || g_arcActive || g_taskRunning || Task2_IsSpecialState() || g_workMode != WORK_BT) return;

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

    if (n <= 0 || tok[0][0] == '\0') return;

    if (str_is_name(tok[0], "key", "k", 0))
    {
        if (n >= 3 && str_is_down(tok[2]))
        {
            if (str_is_name(tok[1], "emergency", "emg", "stop")) { App_EmergencyStop(); return; }
            if (str_is_name(tok[1], "unlock", "release", "resume")) { App_UnlockControl(); return; }
            if (str_is_name(tok[1], "presetFast", "fast", "fastPreset")) { ApplyFastPreset(); return; }
            if (str_is_name(tok[1], "encClear", "encoderClear", "clearEnc")) { Encoder_DebugClearTotals(); return; }
            if (str_is_name(tok[1], "encDebug", "encoderDebug", "encDbg")) { Local_EnterEncoderDebug(); return; }
            if (str_is_name(tok[1], "mpuDebug", "gyroDebug", "yawDebug")) { Local_EnterMpuDebug(); return; }
            if (str_is_name(tok[1], "mpuCalib", "gyroCalib", "calib")) { g_plotMode = 2U; MPU_CalibrateGyroZ(); return; }
            if (str_is_name(tok[1], "yawZero", "mpuZero", "zeroYaw")) { MPU_ResetYaw(); Prompt_Start(180); return; }
            if (str_is_name(tok[1], "task1", "selectTask1", "t1")) { Task_SelectTask1(); return; }
            if (str_is_name(tok[1], "task2", "selectTask2", "t2")) { Task_SelectTask2(); return; }
            if (str_is_name(tok[1], "task3", "selectTask3", "t3")) { Task_SelectOnly(3U); Prompt_Start(330); return; }
            if (str_is_name(tok[1], "task4", "selectTask4", "t4")) { Task_SelectOnly(4U); Prompt_Start(400); return; }
            if (str_is_name(tok[1], "start", "run", "taskStart")) { Task_StartSelected(); return; }
            if (str_is_name(tok[1], "taskStop", "taskReset", "taskIdle")) { Task_Stop(); return; }
            if (str_is_name(tok[1], "arcTest", "arc", "arcRun")) { Task_Reset(); Arc_Start(); return; }
            if (str_is_name(tok[1], "tracing", "trace", "line")) { App_StartTracingMode(); return; }
            if (str_is_name(tok[1], "Bluetooth", "BT", "remote")) { App_StartBluetoothMode(); return; }
        }
        return;
    }

    if (str_is_name(tok[0], "slider", "s", 0))
    {
        if (n >= 3) Main_ApplySliderPacket(tok[1], (float)atof(tok[2]));
        return;
    }

    if (str_is_name(tok[0], "joystick", "j", 0))
    {
        Main_ApplyJoystickPacket(tok, n);
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
            Main_ApplyPacket(packet);
            g_lastCmdTickMs = 0;
            continue;
        }
        if (c == '\r' || c == '\n') continue;
        if (index < sizeof(packet) - 1U) packet[index++] = c;
        else
        {
            receiving = 0;
            index = 0;
        }
    }
}

/* ================================================================
 * 17. 速度闭环、电机输出、显示和按键
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

    if (Task2_IsSpecialState())
    {
        Task2_Control10ms();
    }
    else if (g_straightActive)
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

static int ModeCode(void)
{
    if (g_safetyLocked) return 9;
    if (g_straightActive) return 3;
    if (g_arcActive) return 4;
    if (g_taskState == TASK_READY_TASK1) return 11;
    if (g_taskState == TASK_READY_TASK2) return 21;
    if (g_taskState == TASK2_SEARCH_ARC_BC) return 31;
    if (g_taskState == TASK2_TRACE_ARC_BC) return 32;
    if (g_taskState == TASK2_WAIT_ALIGN_C) return 37;
    if (g_taskState == TASK2_ALIGN_C) return 33;
    if (g_taskState == TASK2_SEARCH_ARC_DA) return 34;
    if (g_taskState == TASK2_TRACE_ARC_DA) return 35;
    if (g_taskState == TASK2_WAIT_ALIGN_A) return 38;
    if (g_taskState == TASK2_ALIGN_A) return 36;
    if (g_taskState == TASK2_FINISH) return 22;
    if (g_taskState == TASK_FINISH) return 12;
    if (g_workMode == WORK_STANDBY)
    {
        if (g_localMode == LOCAL_ENCODER_DEBUG) return 41;
        if (g_localMode == LOCAL_MPU_DEBUG) return 42;
        return 40;
    }
    return (int)g_workMode;
}

static char *ModeString(void)
{
    if (g_safetyLocked) return "LOCK";
    if (g_straightActive) return "STR";
    if (Task2_IsSearchState()) return "SRCH";
    if (Task2_IsTraceState()) return "ARC";
    if (Task2_IsAlignState()) return "ALGN";
    if (g_arcActive) return "ARC";
    if (g_workMode == WORK_TRACING) return "TRACE";
    if (g_workMode == WORK_BT) return "BT";
    if (g_localMode == LOCAL_ENCODER_DEBUG) return "ENC";
    if (g_localMode == LOCAL_MPU_DEBUG) return "MPU";
    return "STBY";
}

static void Serial_SendPlotStatus(void)
{
    int modeCode = ModeCode();

    if (g_plotMode == 1U)
    {
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
        Serial_Printf("[p,%d,%d,%d,%d,%d,%d,%ld,%ld,%d,%d]\r\n",
                      modeCode,
                      (int)(g_yawDeg * 10.0f),
                      (int)(g_arcDeltaYaw * 10.0f),
                      (int)(g_arcRunMs / 10U),
                      (int)g_lineError,
                      (int)g_lineMask,
                      (long)g_forwardEncoderTotal,
                      (long)Task2_GetWheelDiffPulse(),
                      (int)g_targetTurnSpeed,
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
        OLED_Printf(0, 32, OLED_8X16, "Y:%+04d", (int)g_yawDeg);
        OLED_Printf(0, 48, OLED_8X16, "G:%+04d B:%+03d", (int)g_gyroZDps, (int)g_gyroZBiasDps);
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
        OLED_Printf(0, 0, OLED_8X16, "T2 S:%02d R:%04d", (int)g_taskState, (int)g_arcRunMs);
        OLED_Printf(0, 16, OLED_8X16, "E:%+04d M:%02X", (int)g_lineError, (int)g_lineMask);
        OLED_Printf(0, 32, OLED_8X16, "F:%+05ld", (long)g_forwardEncoderTotal);
        OLED_Printf(0, 48, OLED_8X16, "D:%+05ld V:%d", (long)Task2_GetWheelDiffPulse(), (int)g_lineValid);
        OLED_Update();
        return;
    }

    OLED_Printf(0, 0, OLED_8X16, "M:%s T:%d RP:%03d", ModeString(), (int)g_taskSelected, (int)g_btSpeedLimitPercent);
    OLED_Printf(0, 16, OLED_8X16, "LK:%d S:%02d", (int)g_safetyLocked, (int)g_taskState);
    OLED_Printf(0, 32, OLED_8X16, "L:%+04d R:%+04d", (int)g_leftPwm, (int)g_rightPwm);
    OLED_Printf(0, 48, OLED_8X16, "R:%02X M:%02X E:%+03d", (int)g_lineRawMask, (int)g_lineMask, (int)(g_lineError / 10));
    OLED_Update();
}

static void Main_KeyProcess(void)
{
    uint8_t key = Key_GetNum();
    if (key == 0U) return;

    /*
     * 实体按键功能：
     *   K1：本地模式循环：待机 -> 编码器调试 -> MPU 调试 -> 待机。
     *   K2：任务选择循环：task1 -> task2 -> task3 -> task4。
     *   K3：待机执行当前任务；编码器调试清零脉冲；MPU 调试清零 yaw。
     *   K4：解锁小车，PWM 清零，回到待机。
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
        App_UnlockControl();
        return;
    }
}

/* ================================================================
 * 18. 主函数和 1ms 定时中断
 * ================================================================ */

int main(void)
{
    OLED_Init();
    Key_Init();
    Grayscale_Init();
    Motor_Init();
    Encoder_Init();

    Line_GPIOForceInit();
    PromptIO_Init();

    MPU_AppInit();
    Serial_Init();
    Timer_Init();
    Control_Init();

    ApplySpeedLimitPercent(g_btSpeedLimitPercent);
    PromptIO_AllOff();

    OLED_Printf(0, 0, OLED_8X16, "Local Standby");
    OLED_Printf(0, 16, OLED_8X16, "K1Mode K2Task");
    OLED_Printf(0, 32, OLED_8X16, "K3Run  K4Unlock");
    OLED_Printf(0, 48, OLED_8X16, "Task:%d", (int)g_taskSelected);
    OLED_Update();

    while (1)
    {
        Main_BTProcess();
        Main_KeyProcess();

        if (g_mpuUpdateMs >= MPU_UPDATE_PERIOD_MS)
        {
            uint16_t elapsedMs = g_mpuUpdateMs;
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
