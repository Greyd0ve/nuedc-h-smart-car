#ifndef __APP_STATE_H
#define __APP_STATE_H

typedef enum
{
    WORK_STANDBY = 0,
    WORK_BT = 1,
    WORK_TRACING = 2
} WorkMode_t;

typedef enum
{
    LOCAL_STANDBY = 0,
    LOCAL_ENCODER_DEBUG = 1,
    LOCAL_MPU_DEBUG = 2
} LocalMode_t;

void App_EmergencyStop(void);
void App_UnlockControl(void);
/* Compatibility shell only; Bluetooth remote driving is disabled. */
void App_StartBluetoothMode(void);
void App_StartTracingMode(void);

#endif
