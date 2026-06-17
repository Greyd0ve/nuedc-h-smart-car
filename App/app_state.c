#include "app_state.h"
#include "app_control.h"
#include "app_line.h"
#include "Encoder.h"
#include <stdint.h>

extern volatile WorkMode_t g_workMode;
extern volatile LocalMode_t g_localMode;
extern volatile float g_targetForwardSpeed;
extern volatile float g_targetTurnSpeed;
extern volatile uint8_t g_carEnable;
extern volatile uint8_t g_safetyLocked;
extern volatile uint8_t g_plotMode;
extern volatile uint32_t g_lastCmdTickMs;

extern void App_StateTaskReset(void);
extern void App_StateForcePWMZero(void);
extern void App_StatePromptStart(uint16_t ms);

void App_EmergencyStop(void)
{
    g_safetyLocked = 1;
    g_workMode = WORK_STANDBY;
    g_localMode = LOCAL_STANDBY;
    App_StateTaskReset();
    App_StateForcePWMZero();
    App_StatePromptStart(500);
}

void App_UnlockControl(void)
{
    g_safetyLocked = 0;
    g_workMode = WORK_STANDBY;
    g_localMode = LOCAL_STANDBY;
    g_plotMode = 0U;
    App_StateTaskReset();
    g_lastCmdTickMs = 0;
    App_StateForcePWMZero();
    App_StatePromptStart(180);
}

void App_StartBluetoothMode(void)
{
    /*
     * Bluetooth serial is kept for parameters, task commands, calibration and
     * telemetry only. Remote driving mode is intentionally disabled.
     */
    (void)g_safetyLocked;
    App_StatePromptStart(80);
}

void App_StartTracingMode(void)
{
    if (g_safetyLocked) return;

    App_StateTaskReset();
    g_workMode = WORK_TRACING;
    g_localMode = LOCAL_STANDBY;
    App_Line_ResetState();
    g_targetForwardSpeed = 0.0f;
    g_targetTurnSpeed = 0.0f;
    g_carEnable = 1;
    g_lastCmdTickMs = 0;
    Encoder_ClearAll();
    App_Control_ResetPID();
    App_StatePromptStart(160);
}
