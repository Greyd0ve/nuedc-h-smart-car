#ifndef __APP_PROTOCOL_H
#define __APP_PROTOCOL_H

#include <stdint.h>

#define ENABLE_VERBOSE_STATUS   0U
#define ENABLE_LEGACY_ARCTEST   0U

void App_Protocol_Process(void);
void App_Protocol_ApplySpeedLimitPercent(float percent);

/* Hooks implemented by User/main.c to keep legacy task logic in place. */
void App_ProtocolTaskReset(void);
void App_ProtocolForcePWMZero(void);
void App_ProtocolPromptStart(uint16_t ms);
void App_ProtocolEnterMpuDebug(void);
void App_ProtocolMpuCalibrateGyroZ(void);
void App_ProtocolMpuResetYaw(void);
void App_ProtocolSelectTask1(void);
void App_ProtocolSelectTask2(void);
void App_ProtocolSelectOnly(uint8_t task);
void App_ProtocolStartSelectedTask(void);
void App_ProtocolTaskStop(void);
#if ENABLE_LEGACY_ARCTEST
void App_ProtocolArcStart(void);
#endif
uint8_t App_ProtocolTask2IsSpecialState(void);

#endif
